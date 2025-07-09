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
#include <zmk/drivers/sensor/tlx493d_state.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

// A2BW register definitions from official library

// Custom I2C read function for TLX493D A2BW
static int tlx493d_i2c_reg_read_byte(const struct i2c_dt_spec *i2c_spec, uint8_t reg_addr, uint8_t *value) {
    int ret;
    
    // Phase 1: Write register address
    ret = i2c_write_dt(i2c_spec, &reg_addr, 1);
    if (ret < 0) {
        LOG_ERR("Failed to write register address 0x%02X: %d", reg_addr, ret);
        return ret;
    }
    
    // Small delay to ensure proper timing
    k_usleep(10);
    
    // Phase 2: Read data using separate read operation
    ret = i2c_read_dt(i2c_spec, value, 1);
    if (ret < 0) {
        LOG_ERR("Failed to read from register 0x%02X: %d", reg_addr, ret);
        return ret;
    }
    
    LOG_DBG("Successfully read 0x%02X from register 0x%02X", *value, reg_addr);
    return 0;
}

// Alternative implementation using manual message construction
static int tlx493d_i2c_reg_read_byte_manual(const struct i2c_dt_spec *i2c_spec, uint8_t reg_addr, uint8_t *value) {
    struct i2c_msg msgs[2];
    int ret;
    
    // Message 1: Write register address
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;
    
    // Message 2: Read data
    msgs[1].buf = value;
    msgs[1].len = 1;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
    
    ret = i2c_transfer_dt(i2c_spec, msgs, 2);
    if (ret < 0) {
        LOG_ERR("I2C transfer failed for register 0x%02X: %d", reg_addr, ret);
        return ret;
    }
    
    LOG_DBG("Manual read: 0x%02X from register 0x%02X", *value, reg_addr);
    return 0;
}

// Custom burst read function for TLX493D A2BW
static int tlx493d_i2c_burst_read(const struct i2c_dt_spec *i2c_spec, uint8_t start_addr, uint8_t *data, size_t len) {
    struct i2c_msg msgs[2];
    int ret;
    
    // Message 1: Write start register address
    msgs[0].buf = &start_addr;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;
    
    // Message 2: Read multiple bytes
    msgs[1].buf = data;
    msgs[1].len = len;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART;
    
    ret = i2c_transfer_dt(i2c_spec, msgs, 2);
    if (ret < 0) {
        LOG_ERR("I2C burst read failed from register 0x%02X: %d", start_addr, ret);
        return ret;
    }
    
    LOG_DBG("Burst read %zu bytes from register 0x%02X", len, start_addr);
    return 0;
}

// Using Infineon official library - register definitions are in the library

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

// Error threshold for auto-recovery
#define ERROR_THRESHOLD 5

// Maximum retry attempts for initialization
#define MAX_INIT_RETRIES 3

// 初期化ステップ間の待機時間 (ms)
#define INIT_STEP_DELAY_MS 50

// Calibration settings
#define CALIBRATION_SAMPLES 100

// 自動キャリブレーションのための無移動時間閾値 (ms)
#define AUTO_RECALIBRATION_TIMEOUT_MS 3000

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
    uint8_t error_count;
    uint8_t init_retries;
    bool data_valid;
    int64_t last_movement_time;
    bool movement_detected;
    bool z_pressed;           // Z軸押し込み状態
    bool prev_z_pressed;      // 前回のZ軸状態
};

// 関数プロトタイプ宣言 - 公式ライブラリ使用
static int tlv493d_a2bw_initialize_sensor(const struct device *dev);
static int tlx493d_a2bw_read_sensor_data(const struct device *dev);
static void tlx493d_calibrate(const struct device *dev);
static void generate_bar_graph(int16_t value, char *buffer, size_t buffer_size);

/**
 * @brief A2BW専用センサー初期化（改良版、公式ライブラリ知識活用）
 */
static int tlv493d_a2bw_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    int retry_count = 0;
    int ret;

    LOG_INF("Starting TLV493D-A2BW sensor initialization (improved implementation)");
    LOG_INF("I2C bus: %s, Address: 0x%02X", config->i2c.bus->name, config->i2c.addr);

    while (retry_count < MAX_INIT_RETRIES) {
        LOG_DBG("A2BW initialization attempt %d/%d", retry_count + 1, MAX_INIT_RETRIES);

        // Check if I2C device is present with simple probe
        uint8_t probe_data;
        ret = i2c_read_dt(&config->i2c, &probe_data, 1);
        LOG_INF("I2C probe result: %d (0x%02X)", ret, probe_data);
        
        // Skip reset sequence on first failure to test basic connectivity
        if (retry_count > 0) {
            // A2BW reset sequence (0xFF x2, 0x00 x2, 30μs delay)
            uint8_t reset_seq[] = {0xFF, 0xFF, 0x00, 0x00};
            for (int i = 0; i < 4; i++) {
                ret = i2c_write_dt(&config->i2c, &reset_seq[i], 1);

                k_usleep(30);  // 30μs delay as per A2BW spec
            }
            k_usleep(30);  // 30μs delay as per A2BW spec
        } else {
            LOG_INF("Skipping reset sequence on first attempt to test basic I2C connectivity");
        }

        // Read version register to verify communication
        uint8_t version;
        ret = tlx493d_i2c_reg_read_byte_manual(&config->i2c, 0x16, &version);
        LOG_INF("A2BW version read: 0x%02X (ret %d)", version, ret);
        if (ret == 0) {
            uint8_t type = (version >> 4) & 0x03;
            uint8_t hwv = version & 0x0F;
            LOG_INF("A2BW detected: TYPE=0x%X, HWV=0x%X", type, hwv);
            
            if (type == 0x3) {  // Expected TYPE for A2BW
                data->initialized = true;
                data->init_retries = 0;
                data->data_valid = false;
                LOG_INF("TLV493D-A2BW sensor initialization completed successfully");
                return 0;
            }
        }

    next_attempt:
        retry_count++;
        k_msleep(INIT_STEP_DELAY_MS * (1 << retry_count));
    }

    LOG_ERR("TLV493D-A2BW sensor initialization failed after %d attempts", MAX_INIT_RETRIES);
    return -EIO;
}

/**
 * @brief A2BWセンサーデータ読み取り（改良版、A2BW専用最適化）
 */
static int tlx493d_a2bw_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t raw_data[7]; // Read registers 0x00-0x06
    int ret;
    char x_bar[BAR_GRAPH_WIDTH + 1], y_bar[BAR_GRAPH_WIDTH + 1], z_bar[BAR_GRAPH_WIDTH + 1];

    if (!data->initialized) {
        return -ENODEV;
    }

    // Read measurement and diagnostic registers (0x00-0x06)
    ret = tlx493d_i2c_burst_read(&config->i2c, 0x00, raw_data, 7);
    if (ret < 0) {
        LOG_ERR("Failed to read A2BW sensor data (ret %d)", ret);
        data->data_valid = false;
        return ret;
    }
    
    // Check diagnostic register for data validity (register 0x06)
    uint8_t diag = raw_data[6];
    if (!(diag & 0x04) || !(diag & 0x08)) {  // PD0 and PD3 bits
        LOG_DBG("ADC conversion not complete - data invalid");
        data->data_valid = false;
        return -EAGAIN;
    }
    
    // A2BW data format: 12-bit values with specific bit layout
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
    data->temp = (temp_val << 4) >> 4;

    data->data_valid = true;

    // Log with bar graphs periodically
    if ((data->log_counter++ % 10) == 0) {
        generate_bar_graph(data->x, x_bar, sizeof(x_bar));
        generate_bar_graph(data->y, y_bar, sizeof(y_bar));
        generate_bar_graph(data->z, z_bar, sizeof(z_bar));
        LOG_INF("A2BW X: %5d [%s]", data->x, x_bar);
        LOG_INF("A2BW Y: %5d [%s]", data->y, y_bar);
        LOG_INF("A2BW Z: %5d [%s]", data->z, z_bar);
        LOG_DBG("Frame: %d, Temp: %d", diag & 0x03, data->temp);
    }

    return 0;
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

    // Z軸状態変化の検出とグローバル状態更新
    if (data->z_pressed != data->prev_z_pressed) {
        LOG_INF("Z-axis state changed: %s", data->z_pressed ? "PRESSED" : "RELEASED");
        
        // グローバル状態を更新（behavior_z_axis_morphで使用される）
        tlx493d_set_z_axis_pressed(data->z_pressed);
        
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
        /* Direct behavior bindings removed - use zmk,behavior-z-axis-morph instead */ \
        /* .normal_binding = ..., */ \
        /* .pressed_binding = ..., */ \
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)