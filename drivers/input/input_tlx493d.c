/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

// TLV493D-A1B6 Registers
#define TLV493D_REG_BX_MSB      0x00
#define TLV493D_REG_TEMP_LSB    0x07

// Write registers
#define TLV493D_REG_MOD1        0x01
#define TLV493D_REG_MOD2        0x03

// MOD1 register bits
#define TLV493D_MOD1_FASTMODE   BIT(1)

// MOD2 register bits  
#define TLV493D_MOD2_TEMP_EN    BIT(0)

// Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00

// Bar graph settings
#define BAR_GRAPH_WIDTH 40
#define SENSOR_VALUE_MIN -2048
#define SENSOR_VALUE_MAX 2047

// Deadzone for X/Y movement to prevent spurious input
#define XY_DEADZONE 20 // Adjust as needed

// Deadzone for Z movement to prevent spurious input
#define Z_DEADZONE 20 // Adjust as needed

// Hysteresis for movement detection
#define HYSTERESIS_THRESHOLD 20 // Adjust as needed

// Calibration settings
#define CALIBRATION_SAMPLES 100

// Error threshold for auto-recovery
#define ERROR_THRESHOLD 5

// Maximum retry attempts for initialization
#define MAX_INIT_RETRIES 3

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    bool addr_pin_high;  // ADDR pin level configuration
};

struct tlx493d_data {
    const struct device *dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t origin_x;
    int16_t origin_y;
    int16_t origin_z;
    bool calibrated;
    struct k_work_delayable work;
    uint8_t factory_settings[3];  // Store bytes 7-9 for write operations
    bool initialized;
    uint32_t log_counter;
    bool movement_active_x; // Hysteresis state for X
    bool movement_active_y; // Hysteresis state for Y
    bool movement_active_z; // Hysteresis state for Z
    uint8_t error_count;    // Counter for consecutive errors
    uint8_t init_retries;   // Counter for initialization retries
};

static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t buf[2] = {reg_addr, val};
    
    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
    const struct tlx493d_config *config = dev->config;
    return i2c_reg_read_byte_dt(&config->i2c, reg_addr, val);
}

static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len)
{
    const struct tlx493d_config *config = dev->config;
    return i2c_burst_read_dt(&config->i2c, start_addr, buf, len);
}

/**
 * @brief I2Cバスをリカバリーする関数
 *
 * I2Cバスが不定状態になった場合に、バスをリセットして正常状態に戻す
 * 1. スタート・ストップコンディションを送信
 * 2. リカバリーフレーム(0xFF)を送信
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_i2c_bus_recovery(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    int ret;
    
    LOG_INF("Performing I2C bus recovery");
    
    // 1. スタート・ストップコンディションを送信
    // 空のメッセージを送信することでこれを実現
    uint8_t dummy = 0;
    ret = i2c_write_dt(&config->i2c, &dummy, 0);
    if (ret < 0) {
        LOG_WRN("Failed to send stop condition (ret %d)", ret);
        // エラーを返さず続行
    }
    
    // 少し待機
    k_msleep(20);
    
    // 2. リカバリーフレーム(0xFF)を送信
    ret = tlv493d_send_recovery_frame(dev);
    if (ret < 0) {
        LOG_WRN("Recovery frame failed (ret %d)", ret);
        // エラーを返さず続行
    }
    
    // 少し待機
    k_msleep(20);
    
    LOG_INF("I2C bus recovery completed");
    return 0;
}

static int tlv493d_send_recovery_frame(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t recovery_cmd = TLV493D_RECOVERY_CMD;
    
    LOG_DBG("Sending recovery frame");
    return i2c_write_dt(&config->i2c, &recovery_cmd, 1);
}

static int tlv493d_send_reset_command(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t reset_cmd = TLV493D_RESET_CMD;
    
    LOG_DBG("Sending reset command");
    return i2c_write_dt(&config->i2c, &reset_cmd, 1);
}

static int tlv493d_read_factory_settings(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    
    // Read factory settings from registers 7-9
    ret = tlx493d_read_multiple(dev, TLV493D_REG_TEMP_LSB, data->factory_settings, 3);
    if (ret < 0) {
        LOG_ERR("Failed to read factory settings (ret %d)", ret);
        return ret;
    }
    
    LOG_DBG("Factory settings: 0x%02X 0x%02X 0x%02X", 
            data->factory_settings[0], data->factory_settings[1], data->factory_settings[2]);
    
    return 0;
}

static int tlv493d_configure_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    uint8_t mod1_val, mod2_val;
    
    // Configure MOD1: Enable fast mode for continuous measurement
    // Preserve factory settings from byte 7 (bits 3-7)
    mod1_val = (data->factory_settings[0] & 0xF8) | TLV493D_MOD1_FASTMODE;
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_val);
    if (ret < 0) {
        LOG_ERR("Failed to write MOD1 register (ret %d)", ret);
        return ret;
    }
    
    // 書き込み後の待機時間を延長
    k_msleep(20);
    
    // Configure MOD2: Enable temperature measurement
    // Preserve factory settings and enable temperature
    mod2_val = data->factory_settings[2] | TLV493D_MOD2_TEMP_EN;
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2_val);
    if (ret < 0) {
        LOG_ERR("Failed to write MOD2 register (ret %d)", ret);
        return ret;
    }
    
    // 書き込み後の待機時間を追加
    k_msleep(20);
    
    LOG_INF("Sensor configured: MOD1=0x%02X, MOD2=0x%02X", mod1_val, mod2_val);
    return 0;
}

/**
 * @brief センサーの診断を行う関数
 *
 * センサーの状態を診断し、レジスタ値をログに出力する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_diagnose(const struct device *dev)
{
    uint8_t reg_values[10];
    int ret;
    
    LOG_INF("Performing TLV493D sensor diagnostics");
    
    // 各レジスタの値を読み取り
    ret = tlx493d_read_multiple(dev, 0, reg_values, sizeof(reg_values));
    if (ret < 0) {
        LOG_ERR("Failed to read registers for diagnostics (ret %d)", ret);
        return ret;
    }
    
    // レジスタ値をログに出力
    LOG_INF("Register values:");
    for (int i = 0; i < sizeof(reg_values); i++) {
        LOG_INF("  Reg[%d] = 0x%02X", i, reg_values[i]);
    }
    
    // MOD1レジスタの値を確認
    uint8_t mod1_val = reg_values[1];
    LOG_INF("MOD1 register: 0x%02X", mod1_val);
    if ((mod1_val & TLV493D_MOD1_FASTMODE) == 0) {
        LOG_WRN("Fast mode not enabled in MOD1 register");
    }
    
    // MOD2レジスタの値を確認
    uint8_t mod2_val = reg_values[3];
    LOG_INF("MOD2 register: 0x%02X", mod2_val);
    if ((mod2_val & TLV493D_MOD2_TEMP_EN) == 0) {
        LOG_WRN("Temperature measurement not enabled in MOD2 register");
    }
    
    return 0;
}

static int tlv493d_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    uint8_t test_read;
    int retry_count = 0;
    const int max_retries = MAX_INIT_RETRIES;
    
    LOG_INF("Starting TLV493D sensor initialization sequence");
    
    // I2Cバスリカバリーを最初に実行
    ret = tlv493d_i2c_bus_recovery(dev);
    if (ret < 0) {
        LOG_WRN("I2C bus recovery failed, continuing (ret %d)", ret);
    }
    
    while (retry_count < max_retries) {
        // Step 1: リカバリーフレーム送信
        ret = tlv493d_send_recovery_frame(dev);
        if (ret < 0) {
            LOG_WRN("Recovery frame failed, retry %d (ret %d)", retry_count, ret);
            retry_count++;
            k_msleep(10 * (retry_count + 1)); // 遅延を増やしながらリトライ
            continue;
        }
        
        // 待機時間を延長
        k_msleep(20);
        
        // Step 2: リセットコマンド送信
        ret = tlv493d_send_reset_command(dev);
        if (ret < 0) {
            LOG_WRN("Reset command failed, retry %d (ret %d)", retry_count, ret);
            retry_count++;
            k_msleep(10 * (retry_count + 1));
            continue;
        }
        
        // リセット後の待機時間を延長
        k_msleep(50);
        
        // Step 3: I2C通信テスト
        ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
        if (ret < 0) {
            LOG_WRN("I2C communication test failed, retry %d (ret %d)", retry_count, ret);
            retry_count++;
            k_msleep(10 * (retry_count + 1));
            continue;
        }
        
        // Step 4: 工場設定の読み取り
        ret = tlv493d_read_factory_settings(dev);
        if (ret < 0) {
            LOG_WRN("Failed to read factory settings, retry %d (ret %d)", retry_count, ret);
            retry_count++;
            k_msleep(10 * (retry_count + 1));
            continue;
        }
        
        // 待機時間を追加
        k_msleep(20);
        
        // Step 5: センサー設定
        ret = tlv493d_configure_sensor(dev);
        if (ret < 0) {
            LOG_WRN("Failed to configure sensor, retry %d (ret %d)", retry_count, ret);
            retry_count++;
            k_msleep(10 * (retry_count + 1));
            continue;
        }
        
        // 全ステップ成功
        data->initialized = true;
        data->init_retries = 0; // リトライカウンターをリセット
        LOG_INF("TLV493D sensor initialization completed successfully");
        
        // 診断を実行
        tlv493d_diagnose(dev);
        
        return 0;
    }
    
    // 最大リトライ回数を超えた場合
    LOG_ERR("TLV493D sensor initialization failed after %d retries", max_retries);
    data->init_retries++; // 初期化失敗回数をカウント
    return -EIO;
}

// Helper function to generate a bar graph string
static void generate_bar_graph(int16_t value, char *buffer, size_t buffer_size)
{
    int normalized_value = (int)(((float)value - SENSOR_VALUE_MIN) / (SENSOR_VALUE_MAX - SENSOR_VALUE_MIN) * BAR_GRAPH_WIDTH);
    if (normalized_value < 0) normalized_value = 0;
    if (normalized_value > BAR_GRAPH_WIDTH) normalized_value = BAR_GRAPH_WIDTH;

    int num_hashes = normalized_value;
    int num_spaces = BAR_GRAPH_WIDTH - normalized_value;

    memset(buffer, 0, buffer_size);
    
    // Add leading spaces for negative values, or if value is positive but small
    if (value < 0) {
        int negative_bar_length = (int)(((float)abs(value)) / (SENSOR_VALUE_MAX - SENSOR_VALUE_MIN) * BAR_GRAPH_WIDTH / 2);
        for (int i = 0; i < BAR_GRAPH_WIDTH / 2 - negative_bar_length; i++) {
            strcat(buffer, " ");
        }
        for (int i = 0; i < negative_bar_length; i++) {
            strcat(buffer, "#");
        }
        strcat(buffer, "|");
        for (int i = 0; i < BAR_GRAPH_WIDTH / 2; i++) {
            strcat(buffer, " ");
        }
    } else {
        for (int i = 0; i < BAR_GRAPH_WIDTH / 2; i++) {
            strcat(buffer, " ");
        }
        strcat(buffer, "|");
        for (int i = 0; i < num_hashes - BAR_GRAPH_WIDTH / 2; i++) {
            strcat(buffer, "#");
        }
        for (int i = 0; i < num_spaces - (num_hashes - BAR_GRAPH_WIDTH / 2); i++) {
            strcat(buffer, " ");
        }
    }
}

static int tlx493d_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t raw_data[6];
    int ret;
    char x_bar[BAR_GRAPH_WIDTH + 2]; // +2 for null terminator and potential '|'
    char y_bar[BAR_GRAPH_WIDTH + 2];
    char z_bar[BAR_GRAPH_WIDTH + 2];

    if (!data->initialized) {
        LOG_ERR("Sensor not initialized");
        return -ENODEV;
    }

    // Read magnetic field data (6 bytes: Bx, By, Bz)
    ret = tlx493d_read_multiple(dev, TLV493D_REG_BX_MSB, raw_data, 6);
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data (ret %d)", ret);
        return ret;
    }

    // Convert 12-bit values (MSB + 4 bits from LSB)
    data->x = (int16_t)(((raw_data[0] << 4) | (raw_data[4] >> 4)) << 4) >> 4;
    // data->y = (int16_t)(((raw_data[1] << 4) | (raw_data[4] & 0x0F)) << 4) >> 4;
    // data->z = (int16_t)(((raw_data[2] << 4) | (raw_data[5] & 0x0F)) << 4) >> 4;
    data->y = (int16_t)(((raw_data[2] << 4) | (raw_data[5] & 0x0F)) << 4) >> 4;
    data->z = (int16_t)(((raw_data[1] << 4) | (raw_data[4] & 0x0F)) << 4) >> 4;
    
    generate_bar_graph(data->x, x_bar, sizeof(x_bar));
    generate_bar_graph(data->y, y_bar, sizeof(y_bar));
    generate_bar_graph(data->z, z_bar, sizeof(z_bar));

    LOG_INF("X: %5d %s", data->x, x_bar);
    LOG_INF("Y: %5d %s", data->y, y_bar);
    LOG_INF("Z: %5d %s", data->z, z_bar);

    return 0;
}

static void tlx493d_calibrate(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int ret;

    LOG_INF("Starting sensor calibration...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        ret = tlx493d_read_sensor_data(dev);
        if (ret != 0) {
            LOG_ERR("Calibration: Failed to read sensor data, aborting.");
            data->calibrated = false;
            return;
        }
        sum_x += data->x;
        sum_y += data->y;
        sum_z += data->z;
        k_msleep(10); // Wait a bit between samples
    }

    data->origin_x = sum_x / CALIBRATION_SAMPLES;
    data->origin_y = sum_y / CALIBRATION_SAMPLES;
    data->origin_z = sum_z / CALIBRATION_SAMPLES;
    data->calibrated = true;

    LOG_INF("Calibration complete. Origin X: %d, Y: %d, Z: %d", 
            data->origin_x, data->origin_y, data->origin_z);
}

static void tlx493d_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
    const struct device *dev = data->dev;
    int ret;

    // センサーが初期化されていない場合、初期化を試みる
    if (!data->initialized) {
        ret = tlv493d_initialize_sensor(dev);
        if (ret != 0) {
            LOG_ERR("Failed to initialize sensor in work handler (ret %d)", ret);
            
            // 初期化リトライ回数が多すぎる場合、I2Cバスリカバリーを実行
            if (data->init_retries > MAX_INIT_RETRIES * 2) {
                LOG_WRN("Too many initialization failures, performing I2C bus recovery");
                tlv493d_i2c_bus_recovery(dev);
                data->init_retries = 0;
            }
            
            // 次回のワークハンドラーで再試行するためにスケジュール
            k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
            return;
        }
    }

    // キャリブレーションが必要な場合
    if (!data->calibrated) {
        tlx493d_calibrate(dev);
        if (!data->calibrated) {
            // Calibration failed, reschedule and try again
            k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
            return;
        }
    }

    int16_t current_x, current_y, current_z;
    int16_t delta_x, delta_y, delta_z;

    ret = tlx493d_read_sensor_data(dev);
    if (ret == 0) {
        // 正常に読み取れた場合、エラーカウントをリセット
        data->error_count = 0;
        
        current_x = data->x;
        current_y = data->y;
        current_z = data->z;

        delta_x = current_x - data->origin_x;
        delta_y = current_y - data->origin_y;
        delta_z = current_z - data->origin_z;

        bool report_sync = false;

        // Hysteresis and Deadzone for X-axis
        if (abs(delta_x) > HYSTERESIS_THRESHOLD) {
            data->movement_active_x = true;
        } else if (abs(delta_x) < XY_DEADZONE) {
            data->movement_active_x = false;
        }
        if (data->movement_active_x) {
            input_report_rel(dev, INPUT_REL_X, delta_x, false, K_FOREVER);
            report_sync = true;
        }

        // Hysteresis and Deadzone for Y-axis
        if (abs(delta_y) > HYSTERESIS_THRESHOLD) {
            data->movement_active_y = true;
        } else if (abs(delta_y) < XY_DEADZONE) {
            data->movement_active_y = false;
        }
        if (data->movement_active_y) {
            input_report_rel(dev, INPUT_REL_Y, delta_y, false, K_FOREVER);
            report_sync = true;
        }

        // Hysteresis and Deadzone for Z-axis (Scroll)
        if (abs(delta_z) > HYSTERESIS_THRESHOLD) {
            data->movement_active_z = true;
        } else if (abs(delta_z) < Z_DEADZONE) {
            data->movement_active_z = false;
        }
        if (data->movement_active_z) {
            int16_t scroll_value = (delta_z > 0) ? 1 : -1; // Simplified scroll
            input_report_rel(dev, INPUT_REL_WHEEL, scroll_value, false, K_FOREVER);
            report_sync = true;
        }

        if (report_sync) {
            input_report_rel(dev, INPUT_REL_X, 0, true, K_FOREVER); // Send sync event
        } else {
            // If no movement, send a dummy sync to keep ZMK active
            input_report_rel(dev, INPUT_REL_X, 0, true, K_FOREVER);
        }

        LOG_INF("Calibrated Delta: X=%d, Y=%d, Z=%d", delta_x, delta_y, delta_z);
    } else {
        // エラーカウントを増やす
        data->error_count++;
        
        // エラーカウントが閾値を超えた場合、センサーを再初期化
        if (data->error_count >= ERROR_THRESHOLD) {
            LOG_WRN("Error threshold reached (%d), reinitializing sensor", data->error_count);
            data->initialized = false;
            data->error_count = 0;
        }
    }

    // 次回のワークをスケジュール
    k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
}

static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;

    data->dev = dev;
    data->initialized = false;
    data->calibrated = false;
    data->log_counter = 0;
    data->movement_active_x = false;
    data->movement_active_y = false;
    data->movement_active_z = false;
    data->error_count = 0;
    data->init_retries = 0;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    // I2Cバスリカバリーを最初に実行
    tlv493d_i2c_bus_recovery(dev);

    // Perform sensor initialization sequence according to datasheet
    int ret = tlv493d_initialize_sensor(dev);
    if (ret != 0) {
        LOG_ERR("Sensor initialization failed (ret %d)", ret);
        // 初期化に失敗しても、ワークハンドラーで再試行するため、
        // ここではエラーを返さない
    }

    // Perform initial calibration
    if (data->initialized) {
        tlx493d_calibrate(dev);
        if (!data->calibrated) {
            LOG_WRN("Initial calibration failed. Will retry in work handler.");
        }
    }

    k_work_init_delayable(&data->work, tlx493d_work_handler);
    k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));

    LOG_INF("TLX493D driver initialized successfully on %s", dev->name);
    return 0;
}

#define TLX493D_DEFINE(inst) \
    static struct tlx493d_data tlx493d_data_##inst; \
    static const struct tlx493d_config tlx493d_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
        .addr_pin_high = DT_INST_PROP_OR(inst, addr_pin_high, false), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)
