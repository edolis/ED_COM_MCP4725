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
      _address(address),
      _lastValue(0),
      _powerDownMode(0),
      _lastWriteEEPROM(0),
      _maxVoltage(maxVoltage)
{
}

MCP4725::~MCP4725() {
    // Nothing to clean up – bus handles are managed by I2CBus
}

esp_err_t MCP4725::begin(bool sendReset) {
    // Verify device presence
    if (!isConnected()) {
        ESP_LOGW(TAG, "Device at 0x%02X not responding", _address);
        return ESP_ERR_NOT_FOUND;
    }

    // Optional: send general call reset to ensure EEPROM is loaded
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
    // Probe the device: try to read 1 byte (device should ACK)
    uint8_t dummy;
    esp_err_t err = _bus.read(_address, &dummy, 1);
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

esp_err_t MCP4725::setVoltage(float v) {
    if (v < 0.0f || v > _maxVoltage) return MCP4725_VALUE_ERROR;
    uint16_t value = (uint16_t)((v * 4096.0f) / _maxVoltage + 0.5f);
    if (value > 4095) value = 4095;
    return setValue(value);
}

float MCP4725::getVoltage() const {
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
    // Wait for any pending EEPROM write
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
    uint8_t buf[1];
    if (_readRegister(buf, 1) != ESP_OK) return 0;
    // PD bits are in bits 5-4 of first status byte (Table 5-4)
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
// Low‑level I2C helpers (using _bus public methods)
// ----------------------------------------------------------------------
esp_err_t MCP4725::_writeFastMode(uint16_t value) {
    uint8_t data[2];
    data[0] = ((value >> 8) & 0x0F) | (_powerDownMode << 4);
    data[1] = value & 0xFF;
    return _bus.write(_address, data, sizeof(data));
}

esp_err_t MCP4725::_writeRegisterMode(uint16_t value, uint8_t reg) {
    // Combine PD bits according to datasheet Figure 6-2
    reg |= (_powerDownMode);   // PD bits go into bits 1,0
    uint8_t data[3];
    data[0] = reg;
    data[1] = value >> 4;
    data[2] = (value & 0x0F) << 4;
    return _bus.write(_address, data, sizeof(data));
}

esp_err_t MCP4725::_readRegister(uint8_t* buffer, uint8_t length) {
    // MCP4725: read without sending a register address – just receive.
    return _bus.read(_address, buffer, length);
}

esp_err_t MCP4725::_generalCall(uint8_t gc) {
    // General call uses address 0x00
    return _bus.write(0x00, &gc, 1);
}

} // namespace ED_MCP4725