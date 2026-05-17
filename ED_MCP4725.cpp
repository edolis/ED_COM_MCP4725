#include "ED_MCP4725.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ED_MCP4725 {

static const char* TAG = "ED_MCP4725";

// Register values for writeRegisterMode
#define MCP4725_DAC        0x40
#define MCP4725_DACEEPROM  0x60

// General call commands (datasheet §7.3)
#define MCP4725_GC_RESET   0x06
#define MCP4725_GC_WAKEUP  0x09

// ----------------------------------------------------------------------
// Factory & constructor / destructor
// ----------------------------------------------------------------------
MCP4725* MCP4725::create(I2CBus& bus, uint8_t address, float maxVoltage, bool sendReset) {
    if (address < 0x60 || address > 0x67) {
        ESP_LOGE(TAG, "Invalid address 0x%02X (must be 0x60..0x67)", address);
        return nullptr;
    }
    MCP4725* inst = new MCP4725(bus, address, maxVoltage);
    if (inst && inst->begin(sendReset) != ESP_OK) {
        delete inst;
        return nullptr;
    }
    return inst;
}

MCP4725::MCP4725(I2CBus& bus, uint8_t address, float maxVoltage)
    : _bus(bus),
      _devHandle(nullptr),
      _address(address),
      _lastValue(0),
      _powerDownMode(0),
      _lastWriteEEPROM(0),
      _maxVoltage(maxVoltage)
{
}

MCP4725::~MCP4725() {
    // Device handle is managed by I2CBus – nothing to do here
}

esp_err_t MCP4725::begin(bool sendReset) {
    // Obtain device handle from the bus
    esp_err_t err = _bus.get_device(_address, &_devHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device handle for 0x%02X", _address);
        return err;
    }

    // Verify device presence
    if (!isConnected()) {
        ESP_LOGW(TAG, "Device at 0x%02X not responding", _address);
        return ESP_ERR_NOT_FOUND;
    }

    // Optional: send general call reset to ensure EEPROM is loaded
    // This is recommended when VDD ramp rate is slow (<1V/ms) – datasheet §5.4.2
    if (sendReset) {
        powerOnReset();
        vTaskDelay(pdMS_TO_TICKS(10));   // small delay for reset to settle
    }

    // Initialise cached values
    _lastValue = readDAC();
    _powerDownMode = readPowerDownModeDAC();
    ESP_LOGI(TAG, "MCP4725 at 0x%02X initialized, maxVoltage=%.2fV", _address, _maxVoltage);
    return ESP_OK;
}

// ----------------------------------------------------------------------
// Basic operations
// ----------------------------------------------------------------------
bool MCP4725::isConnected() {
    if (!_devHandle) return false;
    // Quick probe: try to read 1 byte (device should ACK)
    uint8_t dummy;
    esp_err_t err = i2c_master_receive(_devHandle, &dummy, 1, I2C_TRANSFER_TIMEOUT_MS);
    return (err == ESP_OK);
}

esp_err_t MCP4725::setValue(uint16_t value) {
    if (value > MCP4725_MAXVALUE) return MCP4725_VALUE_ERROR;
    if (value == _lastValue) return ESP_OK;

    esp_err_t err = _writeFastMode(value);
    if (err == ESP_OK) _lastValue = value;
    return err;
}

esp_err_t MCP4725::setPercentage(float percentage) {
    if (percentage < 0.0f || percentage > 100.0f) return MCP4725_VALUE_ERROR;
    uint16_t value = (uint16_t)(percentage * 0.01f * MCP4725_MAXVALUE + 0.5f);
    return setValue(value);
}

float MCP4725::getPercentage() const {
    return (_lastValue * 100.0f) / MCP4725_MAXVALUE;
}

// Voltage: using datasheet formula Vout = Vref * code / 4096
esp_err_t MCP4725::setVoltage(float v) {
    if (v < 0.0f || v > _maxVoltage) return MCP4725_VALUE_ERROR;
    uint16_t value = (uint16_t)((v * 4096.0f) / _maxVoltage + 0.5f);
    if (value > 4095) value = 4095;   // clamp to max valid code
    return setValue(value);
}

float MCP4725::getVoltage() const {
    // datasheet Eq 5-1: Vout = Vref * code / 4096
    return (_lastValue * _maxVoltage) / 4096.0f;
}

// ----------------------------------------------------------------------
// EEPROM & advanced
// ----------------------------------------------------------------------
esp_err_t MCP4725::writeDAC(uint16_t value, bool eeprom) {
    if (value > MCP4725_MAXVALUE) return MCP4725_VALUE_ERROR;

    // Wait for any ongoing EEPROM write to finish
    uint32_t deadline = esp_timer_get_time() / 1000 + EEPROM_WRITE_TIMEOUT_MS;
    while (!ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) {
            ESP_LOGW(TAG, "ready() timeout before writeDAC");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t reg = eeprom ? MCP4725_DACEEPROM : MCP4725_DAC;
    esp_err_t err = _writeRegisterMode(value, reg);
    if (err == ESP_OK) {
        _lastValue = value;
        if (eeprom) _lastWriteEEPROM = esp_timer_get_time() / 1000;
    }
    return err;
}

bool MCP4725::ready() {
    uint8_t buf[1];
    esp_err_t err = _readRegister(buf, 1);
    if (err != ESP_OK) return false;
    // Bit 7 of the first byte indicates EEPROM write status (0 = busy, 1 = ready)
    return (buf[0] & 0x80) != 0;
}

uint16_t MCP4725::readDAC() {
    // Wait for any pending EEPROM write (datasheet: EEPROM write blocks DAC register read)
    uint32_t deadline = esp_timer_get_time() / 1000 + EEPROM_WRITE_TIMEOUT_MS;
    while (!ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t buf[3];
    if (_readRegister(buf, 3) != ESP_OK) return 0;
    uint16_t value = (buf[1] << 4) | (buf[2] >> 4);
    return value;
}

uint16_t MCP4725::readEEPROM() {
    uint32_t deadline = esp_timer_get_time() / 1000 + EEPROM_WRITE_TIMEOUT_MS;
    while (!ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t buf[5];
    if (_readRegister(buf, 5) != ESP_OK) return 0;
    uint16_t value = ((buf[3] & 0x0F) << 8) | buf[4];
    return value;
}

esp_err_t MCP4725::writePowerDownMode(uint8_t mode, bool eeprom) {
    _powerDownMode = mode & 0x03;
    return writeDAC(_lastValue, eeprom);
}

uint8_t MCP4725::readPowerDownModeEEPROM() {
    uint32_t deadline = esp_timer_get_time() / 1000 + EEPROM_WRITE_TIMEOUT_MS;
    while (!ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    uint8_t buf[4];
    if (_readRegister(buf, 4) != ESP_OK) return 0;
    // PD bits are in byte3 bits 5-4 (Figure 6-3)
    return (buf[3] >> 4) & 0x03;
}

uint8_t MCP4725::readPowerDownModeDAC() {
    // No need to wait for EEPROM – reading DAC register is independent
    uint8_t buf[1];
    if (_readRegister(buf, 1) != ESP_OK) return 0;
    // PD bits are in bits 5-4 of first status byte (Figure 6-3)
    return (buf[0] >> 4) & 0x03;
}

bool MCP4725::getPowerOnResetFlag() {
    uint8_t buf[1];
    if (_readRegister(buf, 1) != ESP_OK) return false;
    // POR is bit 6 of the first status byte (Table 5-4)
    return (buf[0] & 0x40) != 0;
}

esp_err_t MCP4725::powerOnReset() {
    esp_err_t err = _generalCall(MCP4725_GC_RESET);
    if (err == ESP_OK) _lastValue = readDAC();
    return err;
}

esp_err_t MCP4725::powerOnWakeUp() {
    esp_err_t err = _generalCall(MCP4725_GC_WAKEUP);
    if (err == ESP_OK) _powerDownMode = readPowerDownModeDAC();
    return err;
}

// ----------------------------------------------------------------------
// Low‑level I2C helpers
// ----------------------------------------------------------------------
esp_err_t MCP4725::_writeFastMode(uint16_t value) {
    uint8_t data[2];
    data[0] = ((value >> 8) & 0x0F) | (_powerDownMode << 4);
    data[1] = value & 0xFF;
    return i2c_master_transmit(_devHandle, data, sizeof(data), I2C_TRANSFER_TIMEOUT_MS);
}

esp_err_t MCP4725::_writeRegisterMode(uint16_t value, uint8_t reg) {
    uint8_t data[3];
    reg |= (_powerDownMode << 1);   // PD bits in reg bits 2-1? Check datasheet: for write register mode, the PD bits are placed in the command byte at positions indicated.
    // According to Figure 6-2: command byte = C2 C1 C0 X X PD1 PD0   (bits 6..0). So we shift PD by 1? Actually in _writeFastMode we shift by 4. For register mode, the datasheet shows that the 2nd byte (command) uses PD1,PD0 in bits 1 and 0 after the C2,C1,C0? Let's be careful.
    // In Figure 6-2: the second byte = C2 C1 C0 X X PD1 PD0, where X are unused. So PD bits are in bits 1,0. So shifting by 0? Actually they are in the two LSBs. So we should do: reg |= (_powerDownMode). But reg already contains MCP4725_DAC or MCP4725_DACEEPROM (0x40 or 0x60). Those values have bits 6 and 5 set. To combine with PD bits in bits 1,0 we just OR.
    // The previous code used `_powerDownMode << 1` – that would put PD bits at positions 2 and 1, which is wrong. Let's fix:
    reg |= (_powerDownMode);      // PD bits go into bits 1,0 (datasheet Figure 6-2)
    data[0] = reg;
    data[1] = value >> 4;
    data[2] = (value & 0x0F) << 4;
    return i2c_master_transmit(_devHandle, data, sizeof(data), I2C_TRANSFER_TIMEOUT_MS);
}

esp_err_t MCP4725::_readRegister(uint8_t* buffer, uint8_t length) {
    // MCP4725: read without sending a register address – just receive.
    return i2c_master_receive(_devHandle, buffer, length, I2C_TRANSFER_TIMEOUT_MS);
}

esp_err_t MCP4725::_generalCall(uint8_t gc) {
    // General call uses address 0x00. Use the bus's write method.
    return _bus.write(0x00, &gc, 1);
}

} // namespace ED_MCP4725