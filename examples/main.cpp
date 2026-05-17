/**
* @file main.cpp
* @brief Full test for MCP4725 DAC (I2C 12‑bit)
 *
 * @author Emanuele Dolis (emanuele.dolis@gmail.com)
 * @version GIT_VERSION: v1.1.3-4-gf0e7061-dirty
 * @date 2026-05-17
 * @submodules-start
 *   ED_WIFI : v1.0.0-1-g10b3d09
 * @submodules-end
 */

// note! REQUIRES esp_timer esp_hw_support in CMakeLists.txt

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "ED_i2c.h"
#include "ED_MCP4725.h"
#include "ed_board.h"          // provides ED_I2C_SDA, ED_I2C_SCL

#define I2C_MASTER_FREQ_HZ      400000
#define MCP4725_I2C_ADDR        0x60      // typical default address
#define MAX_VOLTAGE             3.3f      // Vref = 3.3V

static const char *TAG = "mcp4725_full_test";

// Forward declarations of test functions
static void test_basic_setget(ED_MCP4725::MCP4725 &dac);
static void test_percentage_voltage(ED_MCP4725::MCP4725 &dac);
static void test_eeprom_write_read(ED_MCP4725::MCP4725 &dac);
static void test_power_down_modes(ED_MCP4725::MCP4725 &dac);
static void test_read_dac_eeprom(ED_MCP4725::MCP4725 &dac);
static void test_ready_and_connection(ED_MCP4725::MCP4725 &dac);
static void test_writedac_flags(ED_MCP4725::MCP4725 &dac);

static void test_max_resolution(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Maximum resolution test (12‑bit, 1 LSB step) ===");
    float v_ref = dac.getMaxVoltage();
    float lsb_voltage = v_ref / 4095.0f;
    ESP_LOGI(TAG, "Vref = %.3f V, 1 LSB = %.6f V (~%.1f µV)", v_ref, lsb_voltage, lsb_voltage * 1e6);

    // Test every 64th code (64 steps of 64 = 4096 values) to keep logs readable
    const uint16_t step = 1;
    uint16_t prev_value = 0;
    float prev_voltage = 0.0f;

    for (uint16_t val = 0; val <= 4095; val += step) {
        dac.setValue(val);
        float voltage = dac.getVoltage();
        ESP_LOGI(TAG, "Code %4u → voltage = %.6f V", val, voltage);
        if (val > 0) {
            float delta_code = (float)(val - prev_value);
            float delta_voltage = voltage - prev_voltage;
            float expected_delta = delta_code * lsb_voltage;
            ESP_LOGD(TAG, "  Δcode = %.0f, ΔV = %.6f V (expected %.6f V)", delta_code, delta_voltage, expected_delta);
        }
        prev_value = val;
        prev_voltage = voltage;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Also test the smallest possible change: 1 LSB (code 2048 → 2049)
    ESP_LOGI(TAG, "Smallest step (1 LSB) around mid‑scale:");
    dac.setValue(2048);
    float v1 = dac.getVoltage();
    dac.setValue(2049);
    float v2 = dac.getVoltage();
    ESP_LOGI(TAG, "Code 2048 → %.6f V, 2049 → %.6f V, difference = %.6f V (should be ~%.6f V)",
             v1, v2, v2 - v1, lsb_voltage);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== MCP4725 Full Feature Test ===");

    // 1. Create I2C bus using board pins (from ed_board.h)
    I2CBus i2cBus(I2C_NUM_0, (gpio_num_t)ED_I2C_SDA, (gpio_num_t)ED_I2C_SCL, I2C_MASTER_FREQ_HZ);

    // 2. Create MCP4725 instance
    ED_MCP4725::MCP4725 *dac = ED_MCP4725::MCP4725::create(i2cBus, MCP4725_I2C_ADDR, MAX_VOLTAGE);
    if (dac == nullptr) {
        ESP_LOGE(TAG, "Failed to create MCP4725 at address 0x%02X", MCP4725_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "MCP4725 created successfully");

    // 3. Run all tests
    test_basic_setget(*dac);
    test_percentage_voltage(*dac);
    test_eeprom_write_read(*dac);
    test_power_down_modes(*dac);
    test_read_dac_eeprom(*dac);
    test_ready_and_connection(*dac);
    test_writedac_flags(*dac);

    test_max_resolution(*dac);


    // 4. Final demo: slow ramp from 0 to max voltage
    ESP_LOGI(TAG, "=== Demo: Voltage ramp (0 → %.1f V) ===", MAX_VOLTAGE);
    for (float v = 0; v <= MAX_VOLTAGE + 0.05f; v += 0.05f) {
        dac->setVoltage(v);
        float actual = dac->getVoltage();
        ESP_LOGI(TAG, "Set=%.2f V, actual=%.2f V (raw=%d)", v, actual, dac->getValue());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }


    // 5. Optional: keep running with periodic random outputs
    while (1) {
        uint16_t raw = esp_random() & 0xFFF;   // 0..4095
        dac->setValue(raw);
        ESP_LOGI(TAG, "Random raw=%d, voltage=%.3f V", raw, dac->getVoltage());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Note: dac will be deleted by the system (not strictly necessary)
    // delete dac;   // if you ever break out of the loop
}

// ---------------------------------------------------------------------
// Test 1: setValue / getValue
// ---------------------------------------------------------------------
static void test_basic_setget(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: setValue / getValue ===");
    const uint16_t test_vals[] = {0, 1024, 2048, 3072, 4095};
    for (uint16_t val : test_vals) {
        ESP_ERROR_CHECK(dac.setValue(val));
        uint16_t readback = dac.getValue();
        ESP_LOGI(TAG, "set %u → get %u (expected %u)", val, readback, val);
        if (readback != val) ESP_LOGW(TAG, "Mismatch!");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------
// Test 2: Percentage and voltage scaling
// ---------------------------------------------------------------------
static void test_percentage_voltage(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: setPercentage / setVoltage ===");
    // Percentages
    for (float pct = 0; pct <= 100; pct += 25) {
        ESP_ERROR_CHECK(dac.setPercentage(pct));
        float actual_pct = dac.getPercentage();
        ESP_LOGI(TAG, "set %.1f%% → get %.1f%% (raw=%d)", pct, actual_pct, dac.getValue());
    }
    // Voltages
    dac.setMaxVoltage(MAX_VOLTAGE);
    for (float v = 0; v <= MAX_VOLTAGE + 0.2f; v += 0.5f) {
        ESP_ERROR_CHECK(dac.setVoltage(v));
        float actual_v = dac.getVoltage();
        ESP_LOGI(TAG, "set %.2f V → get %.2f V (raw=%d)", v, actual_v, dac.getValue());
    }
}

// ---------------------------------------------------------------------
// Test 3: Write to EEPROM and read back (power‑cycle not done here)
// ---------------------------------------------------------------------
static void test_eeprom_write_read(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: writeDAC with EEPROM ===");
    const uint16_t test_val = 2345;   // any value
    ESP_ERROR_CHECK(dac.writeDAC(test_val, true));   // write to DAC + EEPROM
    ESP_LOGI(TAG, "Wrote value %d to DAC + EEPROM", test_val);
    // Wait for EEPROM write to complete
    uint32_t deadline = esp_timer_get_time() / 1000 + 100;
    while (!dac.ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) {
            ESP_LOGW(TAG, "EEPROM write timeout");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    uint16_t eeprom_val = dac.readEEPROM();
    ESP_LOGI(TAG, "Read back from EEPROM: %u (expected %u)", eeprom_val, test_val);
    if (eeprom_val != test_val) ESP_LOGW(TAG, "EEPROM mismatch!");
}

// ---------------------------------------------------------------------
// Test 4: Power‑down modes
// ---------------------------------------------------------------------
static void test_power_down_modes(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: Power‑down modes ===");
    const uint8_t modes[] = {
        MCP4725_PDMODE_NORMAL,
        MCP4725_PDMODE_1K,
        MCP4725_PDMODE_100K,
        MCP4725_PDMODE_500K
    };
    const char *mode_names[] = {"Normal", "1kΩ load", "100kΩ load", "500kΩ load"};

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(dac.writePowerDownMode(modes[i], false));   // write to DAC only
        vTaskDelay(pdMS_TO_TICKS(50));
        uint8_t read_mode = dac.readPowerDownModeDAC();
        ESP_LOGI(TAG, "Set PD mode %s (0x%02X) → read from DAC: 0x%02X", mode_names[i], modes[i], read_mode);
        if (read_mode != modes[i]) ESP_LOGW(TAG, "PD mode mismatch!");
    }
    // Restore normal mode
    dac.writePowerDownMode(MCP4725_PDMODE_NORMAL, false);
}

// ---------------------------------------------------------------------
// Test 5: readDAC and readEEPROM directly
// ---------------------------------------------------------------------
static void test_read_dac_eeprom(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: readDAC / readEEPROM ===");
    uint16_t dac_val = dac.readDAC();
    uint16_t eeprom_val = dac.readEEPROM();
    ESP_LOGI(TAG, "Direct readDAC() = %u, readEEPROM() = %u", dac_val, eeprom_val);
    ESP_LOGI(TAG, "Last EEPROM write timestamp (ms): %u", dac.getLastWriteEEPROM());
}

// ---------------------------------------------------------------------
// Test 6: isConnected and ready status
// ---------------------------------------------------------------------
static void test_ready_and_connection(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: isConnected / ready ===");
    bool connected = dac.isConnected();
    ESP_LOGI(TAG, "isConnected() = %s", connected ? "true" : "false");
    bool rdy = dac.ready();
    ESP_LOGI(TAG, "ready() = %s (should be true if no EEPROM write pending)", rdy ? "true" : "false");
}

// ---------------------------------------------------------------------
// Test 7: writeDAC with and without EEPROM flag
// ---------------------------------------------------------------------
static void test_writedac_flags(ED_MCP4725::MCP4725 &dac) {
    ESP_LOGI(TAG, "=== Test: writeDAC with flags ===");
    uint16_t val1 = 1000;
    ESP_ERROR_CHECK(dac.writeDAC(val1, false));   // only DAC
    ESP_LOGI(TAG, "writeDAC(%u, false) done", val1);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t val2 = 2000;
    ESP_ERROR_CHECK(dac.writeDAC(val2, true));    // DAC + EEPROM
    ESP_LOGI(TAG, "writeDAC(%u, true) done", val2);
    // Wait for EEPROM completion
    uint32_t deadline = esp_timer_get_time() / 1000 + 100;
    while (!dac.ready()) {
        if ((esp_timer_get_time() / 1000) > deadline) {
            ESP_LOGW(TAG, "EEPROM write timeout");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGI(TAG, "EEPROM write finished, last timestamp %u ms", dac.getLastWriteEEPROM());
}