#pragma once

#include "esp_err.h"
#include "ED_i2c.h"
#include <cstdint>

#define MCP4725_VERSION "0.5.0"

#define MCP4725_MAXVALUE 4095

// Error codes
#define MCP4725_VALUE_ERROR   ESP_ERR_INVALID_ARG
#define MCP4725_NOT_CONNECTED ESP_ERR_NOT_FOUND

// Power‑down modes (datasheet Table 5-2)
#define MCP4725_PDMODE_NORMAL 0x00
#define MCP4725_PDMODE_1K     0x01
#define MCP4725_PDMODE_100K   0x02
#define MCP4725_PDMODE_500K   0x03

#define MCP4725_MIDPOINT 2048

namespace ED_MCP4725 {

class MCP4725 {
public:
    /**
     * @brief Factory: create and initialize a MCP4725 instance.
     * @param bus        Reference to an already‑initialized I2CBus.
     * @param address    7‑bit I2C address (0x60…0x67).
     * @param maxVoltage Maximum output voltage (reference voltage, VDD).
     * @param sendReset  If true, sends a General Call Reset after init (recommended).
     * @return Pointer to a new MCP4725, or nullptr on failure.
     */
    static MCP4725* create(I2CBus& bus, uint8_t address, float maxVoltage = 3.3f, bool sendReset = true);
    ~MCP4725();

    // --- Basic DAC operations ---
    esp_err_t setValue(uint16_t value);           // 0…4095
    uint16_t getValue() const { return _lastValue; }

    esp_err_t setPercentage(float percentage);    // 0…100
    float getPercentage() const;

    void     setMaxVoltage(float v) { _maxVoltage = v; }
    float    getMaxVoltage() const { return _maxVoltage; }

    /**
     * @brief Set output voltage.
     * @note  Formula: Vout = Vref * code / 4096  (datasheet Eq 5‑1)
     *        Full scale code (4095) gives Vref * 4095/4096, not Vref.
     */
    esp_err_t setVoltage(float v);
    float    getVoltage() const;   // Uses 4096 denominator (datasheet)

    // --- EEPROM and advanced features ---
    esp_err_t writeDAC(uint16_t value, bool eeprom = false);
    bool      ready();                            // Checks if EEPROM write finished
    uint16_t  readDAC();                          // Reads current DAC register
    uint16_t  readEEPROM();                       // Reads EEPROM stored value
    uint32_t  getLastWriteEEPROM() const { return _lastWriteEEPROM; }

    esp_err_t writePowerDownMode(uint8_t mode, bool eeprom = false);
    uint8_t   readPowerDownModeEEPROM();
    uint8_t   readPowerDownModeDAC();

    /**
     * @brief Read the Power‑On‑Reset flag.
     * @return true if a POR has occurred since last read, false otherwise.
     * @note  The flag is cleared after reading.
     */
    bool      getPowerOnResetFlag();

    esp_err_t powerOnReset();                     // General call reset
    esp_err_t powerOnWakeUp();                    // General call wake‑up

    bool      isConnected();                      // Probes the device
    uint8_t   getAddress() const { return _address; }

private:
    MCP4725(I2CBus& bus, uint8_t address, float maxVoltage);
    esp_err_t begin(bool sendReset);

    // Low‑level I2C helpers (now use _bus directly)
    esp_err_t _writeFastMode(uint16_t value);
    esp_err_t _writeRegisterMode(uint16_t value, uint8_t reg);
    esp_err_t _readRegister(uint8_t* buffer, uint8_t length);
    esp_err_t _generalCall(uint8_t gc);

    I2CBus& _bus;
    uint8_t  _address;
    uint16_t _lastValue;
    uint8_t  _powerDownMode;
    uint32_t _lastWriteEEPROM;
    float    _maxVoltage;

    static constexpr uint32_t EEPROM_WRITE_TIMEOUT_MS = 100;
    static constexpr uint32_t I2C_TRANSFER_TIMEOUT_MS = 100;  // kept for reference, not used directly
};

} // namespace ED_MCP4725