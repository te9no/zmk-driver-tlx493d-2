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

// Infineon TLX493D official library includes
#include "../lib/tlx493d.h"
#include "../lib/TLx493D_A2BW.h"
#include "tlx493d_zephyr_bridge.h"

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

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
    TLx493D_t sensor;         // Infineon official library sensor instance
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
 * @brief A2BW専用センサー初期化（公式ライブラリ使用）
 */
static int tlv493d_a2bw_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    int retry_count = 0;

    LOG_INF("Starting TLV493D-A2BW sensor initialization using official library");

    // Set up Zephyr I2C bridge for official library
    tlx493d_set_zephyr_i2c(&config->i2c);

    while (retry_count < MAX_INIT_RETRIES) {
        LOG_DBG("A2BW initialization attempt %d/%d", retry_count + 1, MAX_INIT_RETRIES);

        // Initialize sensor using official library
        if (tlx493d_init(&data->sensor, TLx493D_A2BW_e)) {
            // Set default configuration
            if (tlx493d_setDefaultConfig(&data->sensor)) {
                // Verify sensor is functional
                if (tlx493d_isFunctional(&data->sensor)) {
                    data->initialized = true;
                    data->init_retries = 0;
                    data->data_valid = false;
                    LOG_INF("TLV493D-A2BW sensor initialization completed successfully");
                    return 0;
                }
            }
        }

        retry_count++;
        k_msleep(INIT_STEP_DELAY_MS * (1 << retry_count));
    }

    LOG_ERR("TLV493D-A2BW sensor initialization failed after %d attempts", MAX_INIT_RETRIES);
    return -EIO;
}

/**
 * @brief A2BWセンサーデータ読み取り（公式ライブラリ使用）
 */
static int tlx493d_a2bw_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    char x_bar[BAR_GRAPH_WIDTH + 1], y_bar[BAR_GRAPH_WIDTH + 1], z_bar[BAR_GRAPH_WIDTH + 1];

    if (!data->initialized) {
        return -ENODEV;
    }

    // Read all registers using official library
    if (!tlx493d_readRegisters(&data->sensor)) {
        LOG_ERR("Failed to read A2BW sensor registers");
        data->data_valid = false;
        return -EIO;
    }

    // Check if data is valid
    if (!tlx493d_hasValidData(&data->sensor)) {
        LOG_DBG("Sensor data not valid yet");
        data->data_valid = false;
        return -EAGAIN;
    }

    // Get raw magnetic field values using official library
    if (!tlx493d_getRawMagneticField(&data->sensor, &data->x, &data->y, &data->z)) {
        LOG_ERR("Failed to get raw magnetic field data");
        data->data_valid = false;
        return -EIO;
    }

    // Get raw temperature if needed
    if (!tlx493d_getRawTemperature(&data->sensor, &data->temp)) {
        LOG_DBG("Failed to get temperature data (non-critical)");
    }

    data->data_valid = true;

    // Log with bar graphs periodically
    if ((data->log_counter++ % 10) == 0) {
        generate_bar_graph(data->x, x_bar, sizeof(x_bar));
        generate_bar_graph(data->y, y_bar, sizeof(y_bar));
        generate_bar_graph(data->z, z_bar, sizeof(z_bar));
        LOG_INF("A2BW X: %5d [%s]", data->x, x_bar);
        LOG_INF("A2BW Y: %5d [%s]", data->y, y_bar);
        LOG_INF("A2BW Z: %5d [%s]", data->z, z_bar);
        LOG_DBG("Temp: %d", data->temp);
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