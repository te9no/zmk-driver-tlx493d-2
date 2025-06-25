/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tlx493d_a2bw

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
#include <zmk/behavior.h>
#include <drivers/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

// TLV493D-A2BW Registers
#define TLV493D_REG_BX_MSB      0x00  // Magnetic values MSBs
#define TLV493D_REG_BY_MSB      0x01
#define TLV493D_REG_BZ_MSB      0x02
#define TLV493D_REG_TEMP_MSB    0x03  // Temperature value MSBs
#define TLV493D_REG_BX2         0x04  // Magnetic values LSBs
#define TLV493D_REG_TEMP2       0x05  // Temperature and magnetic LSBs
#define TLV493D_REG_DIAG        0x06  // Sensor diagnostic and status
#define TLV493D_REG_CONFIG      0x10  // Configuration register
#define TLV493D_REG_MOD1        0x11  // Power mode, interrupt, address, parity
#define TLV493D_REG_MOD2        0x13  // Low Power Mode update rate
#define TLV493D_REG_CONFIG2     0x14  // Configuration register 2
#define TLV493D_REG_VER         0x16  // Version register
#define TLV493D_REG_MAP_SIZE    23    // レジスタマップのサイズ

// CONFIG register bits (0x10)
#define TLV493D_CONFIG_DT       BIT(7)  // Disable Temperature
#define TLV493D_CONFIG_AM       BIT(6)  // X/Y Angular Measurement
#define TLV493D_CONFIG_TRIG_MASK 0x30   // Trigger options bits [5:4]
#define TLV493D_CONFIG_X2       BIT(3)  // Short-range sensitivity
#define TLV493D_CONFIG_TL_MAG_MASK 0x06 // Magnetic temperature compensation [2:1]
#define TLV493D_CONFIG_CP       BIT(0)  // Configuration parity

// MOD1 register bits (0x11)
#define TLV493D_MOD1_FP         BIT(7)  // Fuse parity
#define TLV493D_MOD1_IICADR_MASK 0x60   // I2C address bits [6:5]
#define TLV493D_MOD1_PR         BIT(4)  // I2C 1-byte or 2-byte read protocol
#define TLV493D_MOD1_CA         BIT(3)  // Collision avoidance and clock stretching
#define TLV493D_MOD1_INT        BIT(2)  // Interrupt
#define TLV493D_MOD1_MODE_MASK  0x03    // Power mode bits [1:0]

// MOD2 register bits (0x13)
#define TLV493D_MOD2_PRD        BIT(7)  // Update rate settings

// CONFIG2 register bits (0x14)
#define TLV493D_CONFIG2_X4      BIT(0)  // Extra short range sensitivity

// DIAG register bits (0x06)
#define TLV493D_DIAG_P          BIT(7)  // Bus parity
#define TLV493D_DIAG_FF         BIT(6)  // Fuse parity flag
#define TLV493D_DIAG_CF         BIT(5)  // Configuration parity flag
#define TLV493D_DIAG_T          BIT(4)  // T bit
#define TLV493D_DIAG_PD3        BIT(3)  // Power-down flag 3
#define TLV493D_DIAG_PD0        BIT(2)  // Power-down flag 0
#define TLV493D_DIAG_FRM_MASK   0x03    // Frame counter bits [1:0]

// A2BW Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00
#define TLV493D_RESET_DELAY_US  30      // 30μs delay after reset sequence

// Power modes
#define TLV493D_MODE_LOW_POWER      0x00
#define TLV493D_MODE_MASTER_CTRL    0x01
#define TLV493D_MODE_FAST           0x03

// I2C addresses (from Table 4)
#define TLV493D_I2C_ADDR_DEFAULT    0x35  // 6AH/6BH (write/read)
#define TLV493D_I2C_ADDR_ALT1       0x22  // 44H/45H
#define TLV493D_I2C_ADDR_ALT2       0x78  // F0H/F1H
#define TLV493D_I2C_ADDR_ALT3       0x44  // 88H/89H

// Bar graph settings
#define BAR_GRAPH_WIDTH 40
#define SENSOR_VALUE_MIN -100
#define SENSOR_VALUE_MAX 100

// Deadzone for X/Y movement to prevent spurious input
#define XY_DEADZONE 3

// Default Z-axis threshold for press detection
#define Z_PRESS_THRESHOLD_DEFAULT 50

// Default Z-axis hysteresis to prevent bounce  
#define Z_HYSTERESIS_DEFAULT 10

// Hysteresis for movement detection
#define HYSTERESIS_THRESHOLD 20

// Calibration settings
#define CALIBRATION_SAMPLES 100

// Error threshold for auto-recovery
#define ERROR_THRESHOLD 5

// Maximum retry attempts for initialization
#define MAX_INIT_RETRIES 3

// 初期化ステップ間の待機時間 (ms)
#define INIT_STEP_DELAY_MS 50

// リセット後の待機時間 (ms)
#define RESET_DELAY_MS 10

// 自動キャリブレーションのための無移動時間閾値 (ms)
#define AUTO_RECALIBRATION_TIMEOUT_MS 3000

// I2Cバスリカバリーの最大試行回数
#define MAX_BUS_RECOVERY_ATTEMPTS 5

// Power management states
#define TLV493D_POWER_DOWN_DELAY_MS 10
#define TLV493D_POWER_UP_DELAY_MS   20
#define TLV493D_RESET_SEQUENCE_DELAY_MS 50

// ADC hang-up detection
#define ADC_HANG_UP_THRESHOLD 5 // フレームカウンターが停止していると判断する回数

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    bool addr_pin_high;
    int16_t z_press_threshold;
    int16_t z_hysteresis;
    struct zmk_behavior_binding normal_binding;
    struct zmk_behavior_binding pressed_binding;
};

struct tlx493d_data {
    const struct device *dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;             // Temperature value for A2BW
    int16_t origin_x;
    int16_t origin_y;
    int16_t origin_z;
    bool calibrated;
    struct k_work_delayable work;
    bool initialized;
    uint32_t log_counter;
    bool movement_active_x;
    bool movement_active_y;
    bool movement_active_z;
    uint8_t error_count;
    uint8_t init_retries;
    uint8_t reg_map[TLV493D_REG_MAP_SIZE];
    bool data_valid;
    uint8_t parity_errors;
    int64_t last_movement_time;
    bool movement_detected;
    uint8_t bus_recovery_attempts;
    uint8_t prev_frm_counter; // Frame counter for consistency check
    uint8_t hang_up_count;    // Hang-up detection counter
    bool z_pressed;           // Z軸押し込み状態
    bool prev_z_pressed;      // 前回のZ軸状態
};

// 関数プロトタイプ宣言
static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val);
static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val);
static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len);
static int tlv493d_send_recovery_frame(const struct device *dev);
static int tlv493d_send_reset_command(const struct device *dev);
static int tlv493d_general_reset(const struct device *dev);
static int tlv493d_read_factory_settings(const struct device *dev);
static int tlv493d_configure_sensor(const struct device *dev);
static int tlv493d_diagnose(const struct device *dev);
static int tlv493d_initialize_sensor(const struct device *dev);
static int tlx493d_read_sensor_data(const struct device *dev);
static void tlx493d_calibrate(const struct device *dev);
static void tlv493d_diagnose_error(const struct device *dev, int error_code);
static int tlv493d_power_cycle(const struct device *dev);

/**
 * @brief レジスタに値を書き込む
 */
static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    uint8_t buf[2] = {reg_addr, val};
    int ret;

    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret == 0) {
        if (reg_addr < TLV493D_REG_MAP_SIZE) {
            data->reg_map[reg_addr] = val;
        }
    } else {
        tlv493d_diagnose_error(dev, ret);
    }
    return ret;
}

/**
 * @brief レジスタから値を読み取る
 */
static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;

    ret = i2c_reg_read_byte_dt(&config->i2c, reg_addr, val);
    if (ret == 0) {
        if (reg_addr < TLV493D_REG_MAP_SIZE) {
            data->reg_map[reg_addr] = *val;
        }
    } else {
        tlv493d_diagnose_error(dev, ret);
    }
    return ret;
}

/**
 * @brief 複数のレジスタから連続して値を読み取る
 */
static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;

    ret = i2c_burst_read_dt(&config->i2c, start_addr, buf, len);
    if (ret == 0) {
        for (int i = 0; i < len && (start_addr + i) < TLV493D_REG_MAP_SIZE; i++) {
            data->reg_map[start_addr + i] = buf[i];
        }
    } else {
        tlv493d_diagnose_error(dev, ret);
    }
    return ret;
}

/**
 * @brief エラーの詳細診断を行う
 */
static void tlv493d_diagnose_error(const struct device *dev, int error_code)
{
    LOG_ERR("I2C error occurred: %d", error_code);
    switch (error_code) {
        case -EIO: LOG_ERR("I/O Error - Check physical connections"); break;
        case -ETIMEDOUT: LOG_ERR("Timeout Error - Sensor not responding"); break;
        default: LOG_ERR("Unknown I2C error"); break;
    }
}

/**
 * @brief A2BW専用リセットシーケンスを実行する
 */
static int tlv493d_a2bw_reset_sequence(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t recovery_cmd = TLV493D_RECOVERY_CMD;
    uint8_t reset_cmd = TLV493D_RESET_CMD;
    int ret;

    LOG_INF("Starting A2BW reset sequence");
    
    // Step 1: Send 0xFF twice
    ret = i2c_write_dt(&config->i2c, &recovery_cmd, 1);
    if (ret < 0) return ret;
    
    ret = i2c_write_dt(&config->i2c, &recovery_cmd, 1);
    if (ret < 0) return ret;
    
    // Step 2: Send 0x00 twice
    ret = i2c_write_dt(&config->i2c, &reset_cmd, 1);
    if (ret < 0) return ret;
    
    ret = i2c_write_dt(&config->i2c, &reset_cmd, 1);
    if (ret < 0) return ret;
    
    // Step 3: Wait 30μs as specified in A2BW manual
    k_usleep(TLV493D_RESET_DELAY_US);
    
    LOG_DBG("A2BW reset sequence completed");
    return 0;
}

/**
 * @brief A2BW診断レジスタを確認して初期化成功を判定する
 */
static int tlv493d_a2bw_check_diagnostic(const struct device *dev)
{
    uint8_t diag_val;
    int ret = tlx493d_read_reg(dev, TLV493D_REG_DIAG, &diag_val);
    if (ret < 0) {
        LOG_ERR("Failed to read diagnostic register");
        return ret;
    }
    
    LOG_DBG("Diagnostic register: 0x%02X", diag_val);
    
    // Check fuse parity flag (FF)
    if (!(diag_val & TLV493D_DIAG_FF)) {
        LOG_ERR("Fuse parity check failed - sensor defective");
        return -EIO;
    }
    
    // Check configuration parity flag (CF)
    if (!(diag_val & TLV493D_DIAG_CF)) {
        LOG_WRN("Configuration parity check failed");
    }
    
    // Check power-down flags (PD0, PD3)
    if (!(diag_val & TLV493D_DIAG_PD0)) {
        LOG_DBG("ADC conversion of Bx not yet completed");
    }
    
    if (!(diag_val & TLV493D_DIAG_PD3)) {
        LOG_DBG("ADC conversion of Temp not yet completed");
    }
    
    return 0;
}

/**
 * @brief I2C一般リセットコマンドをアドレス0x00に送信する
 *
 * この関数は、ユーザーマニュアルで説明されているように、センサーの完全なリセットをトリガーします。
 */
static int tlv493d_general_reset(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    LOG_WRN("Sending general I2C reset (address 0x00) to recover sensor.");

    // i2c_write関数を使用して、アドレス0x00に0バイトのデータを送信し、
    // 一般リセットをトリガーします。
    int ret = i2c_write(config->i2c.bus, NULL, 0, 0x00);
    if (ret != 0) {
        LOG_ERR("Failed to send general I2C reset command: %d", ret);
        return ret;
    }

    // センサーが内部リセットシーケンスを完了するのを待ちます。
    k_msleep(RESET_DELAY_MS);
    return 0;
}

/**
 * @brief A2BWバージョンレジスタを読み取って識別する
 */
static int tlv493d_a2bw_read_version(const struct device *dev)
{
    uint8_t ver_val;
    int ret = tlx493d_read_reg(dev, TLV493D_REG_VER, &ver_val);
    if (ret < 0) {
        LOG_ERR("Failed to read version register");
        return ret;
    }
    
    uint8_t type = (ver_val >> 4) & 0x03;
    uint8_t hwv = ver_val & 0x0F;
    
    LOG_INF("A2BW Version: TYPE=0x%X, HWV=0x%X", type, hwv);
    
    if (type != 0x3) {
        LOG_WRN("Unexpected TYPE field: 0x%X (expected 0x3)", type);
    }
    
    if (hwv != 0x9) {
        LOG_WRN("Unexpected HWV field: 0x%X (expected 0x9 for B21 design)", hwv);
    }
    
    return 0;
}

/**
 * @brief A2BWセンサーを設定する
 */
static int tlv493d_a2bw_configure_sensor(const struct device *dev)
{
    int ret;
    uint8_t config_val, mod1_val, mod2_val;
    
    // CONFIG register (0x10) - Enable all measurements, short range sensitivity
    config_val = TLV493D_CONFIG_X2;  // Short-range sensitivity for better precision
    // Calculate parity for config register (even parity)
    uint8_t parity = 0;
    for (int i = 1; i < 8; i++) {
        if (config_val & BIT(i)) parity ^= 1;
    }
    if (parity) config_val |= TLV493D_CONFIG_CP;
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_CONFIG, config_val);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    // MOD1 register (0x11) - Fast mode, interrupt enabled, default address
    mod1_val = TLV493D_MODE_FAST;  // Fast mode for continuous measurements
    mod1_val &= ~TLV493D_MOD1_INT; // Enable interrupt
    mod1_val |= TLV493D_MOD1_CA;   // Enable collision avoidance
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_val);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    // MOD2 register (0x13) - Fast update rate
    mod2_val = 0x00;  // Fast update rate (PRD=0)
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2_val);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    LOG_INF("A2BW configured: CONFIG=0x%02X, MOD1=0x%02X, MOD2=0x%02X", 
            config_val, mod1_val, mod2_val);
    return 0;
}

/**
 * @brief A2BW診断情報を表示する
 */
static int tlv493d_a2bw_diagnose(const struct device *dev)
{
    uint8_t diag_val;
    int ret = tlx493d_read_reg(dev, TLV493D_REG_DIAG, &diag_val);
    if (ret < 0) {
        LOG_ERR("Failed to read diagnostic register");
        return ret;
    }
    
    LOG_INF("A2BW Diagnostic Status:");
    LOG_INF("  Bus Parity (P): %s", (diag_val & TLV493D_DIAG_P) ? "OK" : "ERROR");
    LOG_INF("  Fuse Parity (FF): %s", (diag_val & TLV493D_DIAG_FF) ? "OK" : "ERROR");
    LOG_INF("  Config Parity (CF): %s", (diag_val & TLV493D_DIAG_CF) ? "OK" : "ERROR");
    LOG_INF("  T bit: %s", (diag_val & TLV493D_DIAG_T) ? "Valid" : "Invalid");
    LOG_INF("  Temp ADC (PD3): %s", (diag_val & TLV493D_DIAG_PD3) ? "Complete" : "Running");
    LOG_INF("  Bx ADC (PD0): %s", (diag_val & TLV493D_DIAG_PD0) ? "Complete" : "Running");
    LOG_INF("  Frame Counter: %d", diag_val & TLV493D_DIAG_FRM_MASK);
    
    return 0;
}

/**
 * @brief センサーの電源をサイクルする
 */
static int tlv493d_power_cycle(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;

    LOG_INF("Performing power cycle sequence");
    uint8_t power_down_cmd = TLV493D_RECOVERY_CMD; // Use recovery cmd for power down
    i2c_write_dt(&config->i2c, &power_down_cmd, 1);
    k_msleep(TLV493D_POWER_DOWN_DELAY_MS);

    data->initialized = false;
    data->calibrated = false;
    data->data_valid = false;
    data->error_count = 0;
    return 0;
}

/**
 * @brief A2BWセンサーを初期化する
 */
static int tlv493d_a2bw_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    int retry_count = 0;

    LOG_INF("Starting TLV493D-A2BW sensor initialization sequence");

    while (retry_count < MAX_INIT_RETRIES) {
        LOG_DBG("A2BW initialization attempt %d/%d", retry_count + 1, MAX_INIT_RETRIES);

        // Step 1: A2BW specific reset sequence
        ret = tlv493d_a2bw_reset_sequence(dev);
        if (ret < 0) {
            LOG_WRN("A2BW reset sequence failed (ret %d)", ret);
            goto next_attempt;
        }

        // Step 2: Wait for sensor to settle
        k_msleep(100);

        // Step 3: Read and verify version register
        ret = tlv493d_a2bw_read_version(dev);
        if (ret < 0) {
            LOG_WRN("Version register read failed (ret %d)", ret);
            goto next_attempt;
        }

        // Step 4: Check diagnostic status
        ret = tlv493d_a2bw_check_diagnostic(dev);
        if (ret < 0) {
            LOG_WRN("Diagnostic check failed (ret %d)", ret);
            goto next_attempt;
        }

        // Step 5: Configure sensor
        ret = tlv493d_a2bw_configure_sensor(dev);
        if (ret < 0) {
            LOG_WRN("Sensor configuration failed (ret %d)", ret);
            goto next_attempt;
        }

        // Step 6: Verify configuration was successful
        ret = tlv493d_a2bw_check_diagnostic(dev);
        if (ret < 0) {
            LOG_WRN("Post-configuration diagnostic check failed (ret %d)", ret);
            goto next_attempt;
        }

        // Step 7: Test data read
        uint8_t test_read;
        ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
        if (ret < 0) {
            LOG_WRN("Test data read failed (ret %d)", ret);
            goto next_attempt;
        }

        data->initialized = true;
        data->init_retries = 0;
        data->data_valid = false;
        LOG_INF("TLV493D-A2BW sensor initialization completed successfully");

        tlv493d_a2bw_diagnose(dev);
        return 0;

    next_attempt:
        retry_count++;
        k_msleep(INIT_STEP_DELAY_MS * (1 << retry_count));
    }

    LOG_ERR("TLV493D-A2BW sensor initialization failed after %d attempts", MAX_INIT_RETRIES);
    return -EIO;
}

/**
 * @brief バーグラフ文字列を生成する
 */
static void generate_bar_graph(int16_t value, char *buffer, size_t buffer_size)
{
    int normalized_value = (int)(((float)(value - SENSOR_VALUE_MIN) / (SENSOR_VALUE_MAX - SENSOR_VALUE_MIN)) * BAR_GRAPH_WIDTH);
    normalized_value = CLAMP(normalized_value, 0, BAR_GRAPH_WIDTH);

    int mid_point = BAR_GRAPH_WIDTH / 2;
    memset(buffer, ' ', BAR_GRAPH_WIDTH);
    buffer[BAR_GRAPH_WIDTH] = '\0';

    if (value < 0) {
        int bar_len = (int)((float)abs(value) / SENSOR_VALUE_MAX * mid_point);
        bar_len = MIN(bar_len, mid_point);
        for (int i = 0; i < bar_len; i++) {
            buffer[mid_point - 1 - i] = '#';
        }
    } else {
        int bar_len = (int)((float)value / SENSOR_VALUE_MAX * mid_point);
        bar_len = MIN(bar_len, mid_point);
        for (int i = 0; i < bar_len; i++) {
            buffer[mid_point + i] = '#';
        }
    }
    buffer[mid_point] = '|';
}

/**
 * @brief A2BWセンサーデータを読み取る
 */
static int tlx493d_a2bw_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t raw_data[7]; // Read registers 0x00-0x06
    int ret;
    char x_bar[BAR_GRAPH_WIDTH + 1], y_bar[BAR_GRAPH_WIDTH + 1], z_bar[BAR_GRAPH_WIDTH + 1];

    if (!data->initialized) {
        return -ENODEV;
    }

    // Read all measurement and diagnostic registers at once for consistency
    ret = tlx493d_read_multiple(dev, TLV493D_REG_BX_MSB, raw_data, 7);
    if (ret < 0) {
        LOG_ERR("Failed to read A2BW sensor data (ret %d)", ret);
        data->data_valid = false;
        return ret;
    }
    
    // Check diagnostic register for data validity
    uint8_t diag = raw_data[6];
    if (!(diag & TLV493D_DIAG_PD0) || !(diag & TLV493D_DIAG_PD3)) {
        LOG_DBG("ADC conversion not complete - data invalid");
        data->data_valid = false;
        return -EAGAIN;
    }
    
    // Verify bus parity if available
    uint8_t calc_parity = 0;
    for (int i = 0; i < 6; i++) {
        for (int bit = 0; bit < 8; bit++) {
            if (raw_data[i] & BIT(bit)) calc_parity ^= 1;
        }
    }
    
    if (((diag & TLV493D_DIAG_P) != 0) != calc_parity) {
        LOG_WRN("Bus parity error detected - data may be corrupt");
    }
    
    // A2BW data format: 12-bit values
    // Bx: MSB in reg[0], LSB[7:4] in reg[4]
    // By: MSB in reg[1], LSB[3:0] in reg[4] 
    // Bz: MSB in reg[2], LSB[3:0] in reg[5]
    // Temp: MSB in reg[3], LSB[7:6] in reg[5]
    
    int16_t bx_val = (raw_data[0] << 4) | (raw_data[4] >> 4);
    int16_t by_val = (raw_data[1] << 4) | (raw_data[4] & 0x0F);
    int16_t bz_val = (raw_data[2] << 4) | (raw_data[5] & 0x0F);
    int16_t temp_val = (raw_data[3] << 4) | ((raw_data[5] >> 6) & 0x03);

    // Apply sign extension for 12-bit two's complement values
    data->x = (bx_val << 4) >> 4;
    data->y = (by_val << 4) >> 4;
    data->z = (bz_val << 4) >> 4;
    data->temp = (temp_val << 4) >> 4; // Store temperature if needed

    data->data_valid = true;

    if ((data->log_counter++ % 10) == 0) {
        generate_bar_graph(data->x, x_bar, sizeof(x_bar));
        generate_bar_graph(data->y, y_bar, sizeof(y_bar));
        generate_bar_graph(data->z, z_bar, sizeof(z_bar));
        LOG_INF("A2BW X: %5d [%s]", data->x, x_bar);
        LOG_INF("A2BW Y: %5d [%s]", data->y, y_bar);
        LOG_INF("A2BW Z: %5d [%s]", data->z, z_bar);
        LOG_DBG("Frame: %d, Temp: %d", diag & TLV493D_DIAG_FRM_MASK, data->temp);
    }

    return 0;
}

/**
 * @brief センサーをキャリブレーションする
 */
static void tlx493d_calibrate(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    LOG_INF("Starting sensor calibration...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (tlx493d_a2bw_read_sensor_data(dev) != 0) {
            LOG_ERR("Calibration failed: Could not read data.");
            data->calibrated = false;
            return;
        }
        sum_x += data->x;
        sum_y += data->y;
        sum_z += data->z;
        k_msleep(10);
    }

    data->origin_x = sum_x / CALIBRATION_SAMPLES;
    data->origin_y = sum_y / CALIBRATION_SAMPLES;
    data->origin_z = sum_z / CALIBRATION_SAMPLES;
    data->calibrated = true;
    data->last_movement_time = k_uptime_get();
    data->movement_detected = false;

    LOG_INF("Calibration complete. Origin X: %d, Y: %d, Z: %d",
            data->origin_x, data->origin_y, data->origin_z);
}

/**
 * @brief ワークハンドラー
 */
static void tlx493d_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
    const struct device *dev = data->dev;
    int ret;

    if (!data->initialized) {
        ret = tlv493d_a2bw_initialize_sensor(dev);
        if (ret != 0) {
            LOG_ERR("Failed to initialize sensor in work handler (ret %d)", ret);
            k_work_schedule(&data->work, K_MSEC(1000));
            return;
        }
        if (data->initialized && !data->calibrated) {
            tlx493d_calibrate(dev);
        }
    }

    ret = tlx493d_a2bw_read_sensor_data(dev);
    if (ret != 0) {
        data->error_count++;
        if (data->error_count > ERROR_THRESHOLD) {
            LOG_WRN("Too many consecutive errors, forcing re-initialization");
            data->initialized = false;
        }
        k_work_schedule(&data->work, K_MSEC(500));
        return;
    }
    data->error_count = 0;

    // --- A2BW FRAME COUNTER CHECK ---
    // A2BWマニュアルによると、フレームカウンター(FRM)は診断レジスタ(0x06)のビット[1:0]にあります。
    uint8_t diag_reg;
    if (tlx493d_read_reg(dev, TLV493D_REG_DIAG, &diag_reg) == 0) {
        uint8_t current_frm = diag_reg & TLV493D_DIAG_FRM_MASK;
        
        if (data->initialized && data->calibrated && (current_frm == data->prev_frm_counter)) {
            data->hang_up_count++;
            if (data->hang_up_count > ADC_HANG_UP_THRESHOLD) {
                LOG_ERR("A2BW ADC hang-up detected! (FRM stuck at %d). Performing reset.", current_frm);
                ret = tlv493d_a2bw_reset_sequence(dev);
                if (ret == 0) {
                    // 次のサイクルで再初期化を強制する
                    data->initialized = false;
                    data->calibrated = false;
                }
                data->hang_up_count = 0;
                // リセットと再初期化のために遅延させて再スケジュール
                k_work_schedule(&data->work, K_MSEC(500));
                return;
            }
        } else {
            data->hang_up_count = 0;
        }
        data->prev_frm_counter = current_frm;
    }
    // --- END A2BW FRAME COUNTER CHECK ---

    if (!data->calibrated) {
        k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
        return;
    }

    int16_t delta_x = data->x - data->origin_x;
    int16_t delta_y = data->y - data->origin_y;
    int16_t delta_z = data->z - data->origin_z;
    bool report_sync = false;

    // Z軸状態検出（ヒステリシス付き）
    const struct tlx493d_config *config = dev->config;
    if (!data->z_pressed && abs(delta_z) > config->z_press_threshold) {
        data->z_pressed = true;
        LOG_DBG("Z-axis pressed: delta_z=%d", delta_z);
    } else if (data->z_pressed && abs(delta_z) < (config->z_press_threshold - config->z_hysteresis)) {
        data->z_pressed = false;
        LOG_DBG("Z-axis released: delta_z=%d", delta_z);
    }

    // Z軸状態変化の検出とbehavior binding呼び出し
    if (data->z_pressed != data->prev_z_pressed) {
        LOG_INF("Z-axis state changed: %s", data->z_pressed ? "PRESSED" : "RELEASED");
        
        // behavior bindingが設定されていれば呼び出す
        struct zmk_behavior_binding_event event = {
            .position = 0,
            .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
            .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
        };
        
        if (data->z_pressed) {
            // 押し込み状態: pressed-bindingを有効化
            if (config->pressed_binding.behavior_dev != NULL) {
                zmk_behavior_invoke_binding(&config->pressed_binding, event, true);
                LOG_DBG("Invoked pressed-binding");
            }
            // 通常状態のbindingを無効化
            if (config->normal_binding.behavior_dev != NULL) {
                zmk_behavior_invoke_binding(&config->normal_binding, event, false);
                LOG_DBG("Released normal-binding");
            }
        } else {
            // 通常状態: normal-bindingを有効化
            if (config->normal_binding.behavior_dev != NULL) {
                zmk_behavior_invoke_binding(&config->normal_binding, event, true);
                LOG_DBG("Invoked normal-binding");
            }
            // 押し込み状態のbindingを無効化
            if (config->pressed_binding.behavior_dev != NULL) {
                zmk_behavior_invoke_binding(&config->pressed_binding, event, false);
                LOG_DBG("Released pressed-binding");
            }
        }
        
        data->prev_z_pressed = data->z_pressed;
    }

    // XY移動の処理
    if (abs(delta_x) > XY_DEADZONE || abs(delta_y) > XY_DEADZONE) {
        LOG_DBG("XY movement detected: dx=%d, dy=%d, z_pressed=%s", 
                delta_x, delta_y, data->z_pressed ? "true" : "false");
        
        // 相対移動イベントを送信（input processorで処理される）
        input_report_rel(dev, INPUT_REL_X, delta_x, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_NO_WAIT);
        report_sync = true;
    }

    k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
}

/**
 * @brief デバイス初期化関数
 */
static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;

    data->dev = dev;
    data->initialized = false;
    data->calibrated = false;
    data->log_counter = 0;
    data->error_count = 0;
    data->init_retries = 0;
    data->data_valid = false;
    data->bus_recovery_attempts = 0;
    data->prev_frm_counter = 0xFF; // 無効な値で初期化
    data->hang_up_count = 0;
    data->z_pressed = false;
    data->prev_z_pressed = false;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    k_msleep(100);

    k_work_init_delayable(&data->work, tlx493d_work_handler);
    k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));

    LOG_INF("TLX493D driver initialized for %s", dev->name);
    return 0;
}

#define TLX493D_DEFINE(inst) \
    static struct tlx493d_data tlx493d_data_##inst; \
    static const struct tlx493d_config tlx493d_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
        .addr_pin_high = DT_INST_PROP_OR(inst, addr_pin_high, false), \
        .z_press_threshold = DT_INST_PROP_OR(inst, z_press_threshold, Z_PRESS_THRESHOLD_DEFAULT), \
        .z_hysteresis = DT_INST_PROP_OR(inst, z_hysteresis, Z_HYSTERESIS_DEFAULT), \
        .normal_binding = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, normal_binding), \
            ({ \
                .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(inst, normal_binding, 0)), \
                .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, normal_binding, 0, param1), (0), \
                                      (DT_INST_PHA_BY_IDX(inst, normal_binding, 0, param1))), \
                .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, normal_binding, 0, param2), (0), \
                                      (DT_INST_PHA_BY_IDX(inst, normal_binding, 0, param2))), \
            }), \
            ({.behavior_dev = NULL})), \
        .pressed_binding = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pressed_binding), \
            ({ \
                .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(inst, pressed_binding, 0)), \
                .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, pressed_binding, 0, param1), (0), \
                                      (DT_INST_PHA_BY_IDX(inst, pressed_binding, 0, param1))), \
                .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, pressed_binding, 0, param2), (0), \
                                      (DT_INST_PHA_BY_IDX(inst, pressed_binding, 0, param2))), \
            }), \
            ({.behavior_dev = NULL})), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)
