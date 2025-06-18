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
#define TLV493D_REG_MAP_SIZE    10  // レジスタマップのサイズ

// Write registers
#define TLV493D_REG_MOD1        0x01
#define TLV493D_REG_MOD2        0x03

// MOD1 register bits
#define TLV493D_MOD1_FASTMODE   BIT(1)

// MOD2 register bits
#define TLV493D_MOD2_TEMP_EN    BIT(7)

// Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00

// Bar graph settings
#define BAR_GRAPH_WIDTH 40
#define SENSOR_VALUE_MIN -100
#define SENSOR_VALUE_MAX 100

// Deadzone for X/Y movement to prevent spurious input
#define XY_DEADZONE 3

// Deadzone for Z movement to prevent spurious input
#define Z_DEADZONE 5

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
#define RESET_DELAY_MS 100

// 自動キャリブレーションのための無移動時間閾値 (ms)
#define AUTO_RECALIBRATION_TIMEOUT_MS 3000

// I2Cバスリカバリーの最大試行回数
#define MAX_BUS_RECOVERY_ATTEMPTS 5

// Power management states
#define TLV493D_POWER_DOWN_DELAY_MS 100
#define TLV493D_POWER_UP_DELAY_MS   200
#define TLV493D_RESET_SEQUENCE_DELAY_MS 50

// ADC hang-up detection
#define ADC_HANG_UP_THRESHOLD 5 // フレームカウンターが停止していると判断する回数

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    bool addr_pin_high;
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
    uint8_t factory_settings[3];
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
    uint8_t prev_frm_counter; // ADCハングアップ検出用
    uint8_t hang_up_count;    // ADCハングアップカウンター
};

// 関数プロトタイプ宣言
static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val);
static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val);
static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len);
static int tlv493d_send_recovery_frame(const struct device *dev);
static int tlv493d_send_reset_command(const struct device *dev);
static int tlv493d_general_reset(const struct device *dev);
static int tlv493d_i2c_bus_recovery(const struct device *dev);
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
 * @brief リカバリーフレームを送信する
 */
static int tlv493d_send_recovery_frame(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t recovery_cmd = TLV493D_RECOVERY_CMD;
    LOG_DBG("Sending recovery frame");
    return i2c_write_dt(&config->i2c, &recovery_cmd, 1);
}

/**
 * @brief リセットコマンドを送信する
 */
static int tlv493d_send_reset_command(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t reset_cmd = TLV493D_RESET_CMD;
    LOG_DBG("Sending reset command");
    return i2c_write_dt(&config->i2c, &reset_cmd, 1);
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
 * @brief I2Cバスをリカバリーする
 */
static int tlv493d_i2c_bus_recovery(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;

    data->bus_recovery_attempts++;
    LOG_INF("Performing I2C bus recovery (attempt %d/%d)",
            data->bus_recovery_attempts, MAX_BUS_RECOVERY_ATTEMPTS);

    uint8_t dummy = 0;
    i2c_write_dt(&config->i2c, &dummy, 0);
    k_msleep(50);

    for (int i = 0; i < 3; i++) {
        tlv493d_send_recovery_frame(dev);
        k_msleep(20);
    }
    k_msleep(50);

    tlv493d_send_reset_command(dev);
    k_msleep(RESET_DELAY_MS);

    uint8_t test_read;
    ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
    if (ret == 0) {
        LOG_INF("I2C bus recovery completed successfully");
        data->bus_recovery_attempts = 0;
        return 0;
    } else {
        LOG_WRN("I2C bus recovery: Post-recovery read test failed (ret %d)", ret);
        if (data->bus_recovery_attempts >= MAX_BUS_RECOVERY_ATTEMPTS) {
            LOG_ERR("I2C bus recovery failed after max attempts.");
            data->bus_recovery_attempts = 0;
            return -EIO;
        }
        return ret;
    }
}

/**
 * @brief 工場設定を読み取る
 */
static int tlv493d_read_factory_settings(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret = tlx493d_read_multiple(dev, TLV493D_REG_TEMP_LSB, data->factory_settings, 3);
    if (ret < 0) {
        LOG_ERR("Failed to read factory settings (ret %d)", ret);
    }
    return ret;
}

/**
 * @brief センサーを設定する
 */
static int tlv493d_configure_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    uint8_t mod1_val, mod2_val;

    mod1_val = (data->factory_settings[0] & 0xF8) | TLV493D_MOD1_FASTMODE;
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_val);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);

    mod2_val = data->factory_settings[2] & (~TLV493D_MOD2_TEMP_EN);
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2_val);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);

    LOG_INF("Sensor configured: MOD1=0x%02X, MOD2=0x%02X", mod1_val, mod2_val);
    return 0;
}

/**
 * @brief センサーの診断を行う
 */
static int tlv493d_diagnose(const struct device *dev)
{
    uint8_t reg_values[TLV493D_REG_MAP_SIZE];
    int ret = tlx493d_read_multiple(dev, 0, reg_values, sizeof(reg_values));
    if (ret < 0) {
        LOG_ERR("Failed to read registers for diagnostics (ret %d)", ret);
        return ret;
    }
    for (int i = 0; i < sizeof(reg_values); i++) {
        LOG_INF("  Reg[%d] = 0x%02X", i, reg_values[i]);
    }
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
 * @brief センサーを初期化する
 */
static int tlv493d_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    int retry_count = 0;

    LOG_INF("Starting TLV493D sensor initialization sequence");

    while (retry_count < MAX_INIT_RETRIES) {
        LOG_DBG("Initialization attempt %d/%d", retry_count + 1, MAX_INIT_RETRIES);

        tlv493d_power_cycle(dev);
        k_msleep(200);

        for (int i = 0; i < 3; i++) {
            tlv493d_send_recovery_frame(dev);
            k_msleep(INIT_STEP_DELAY_MS);
        }

        for (int i = 0; i < 3; i++) {
            tlv493d_send_reset_command(dev);
            k_msleep(RESET_DELAY_MS);
        }

        uint8_t test_read;
        ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
        if (ret < 0) goto next_attempt;

        ret = tlv493d_read_factory_settings(dev);
        if (ret < 0) goto next_attempt;
        k_msleep(INIT_STEP_DELAY_MS);

        ret = tlv493d_configure_sensor(dev);
        if (ret < 0) goto next_attempt;

        data->initialized = true;
        data->init_retries = 0;
        data->data_valid = false;
        LOG_INF("TLV493D sensor initialization completed successfully");

        tlv493d_diagnose(dev);
        return 0;

    next_attempt:
        retry_count++;
        k_msleep(INIT_STEP_DELAY_MS * (1 << retry_count));
    }

    LOG_ERR("TLV493D sensor initialization failed after %d attempts", MAX_INIT_RETRIES);
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
 * @brief センサーデータを読み取る
 */
static int tlx493d_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t raw_data[7]; // Read up to Reg 6 to get all data
    int ret;
    char x_bar[BAR_GRAPH_WIDTH + 1], y_bar[BAR_GRAPH_WIDTH + 1], z_bar[BAR_GRAPH_WIDTH + 1];

    if (!data->initialized) {
        return -ENODEV;
    }

    ret = tlx493d_read_multiple(dev, TLV493D_REG_BX_MSB, raw_data, 7);
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data (ret %d)", ret);
        data->data_valid = false;
        return ret;
    }
    
    // 12ビット値を正しく結合し、符号拡張を行う
    int16_t bx_val = (raw_data[0] << 4) | (raw_data[4] >> 4);
    int16_t by_val = (raw_data[1] << 4) | (raw_data[4] & 0x0F);
    int16_t bz_val = (raw_data[2] << 4) | (raw_data[5] & 0x0F);

    data->x = (bx_val << 4) >> 4;
    data->y = (by_val << 4) >> 4;
    data->z = (bz_val << 4) >> 4;

    data->data_valid = true;

    if ((data->log_counter++ % 10) == 0) {
        generate_bar_graph(data->x, x_bar, sizeof(x_bar));
        generate_bar_graph(data->y, y_bar, sizeof(y_bar));
        generate_bar_graph(data->z, z_bar, sizeof(z_bar));
        LOG_INF("X: %5d [%s]", data->x, x_bar);
        LOG_INF("Y: %5d [%s]", data->y, y_bar);
        LOG_INF("Z: %5d [%s]", data->z, z_bar);
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
        if (tlx493d_read_sensor_data(dev) != 0) {
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
        ret = tlv493d_initialize_sensor(dev);
        if (ret != 0) {
            LOG_ERR("Failed to initialize sensor in work handler (ret %d)", ret);
            k_work_schedule(&data->work, K_MSEC(1000));
            return;
        }
        if (data->initialized && !data->calibrated) {
            tlx493d_calibrate(dev);
        }
    }

    ret = tlx493d_read_sensor_data(dev);
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

    // --- ADC HANG-UP DETECTION ---
    // マニュアル[5.6]によると、フレームカウンター(FRM)の停止はADCのハングアップを示します。
    // FRMはTempレジスタ(アドレス0x03)のビット3:2にあります。
    uint8_t current_frm = (data->reg_map[3] >> 2) & 0x03;

    if (data->initialized && data->calibrated && (current_frm == data->prev_frm_counter)) {
        data->hang_up_count++;
        if (data->hang_up_count > ADC_HANG_UP_THRESHOLD) {
            LOG_ERR("ADC hang-up detected! (FRM stuck at %d). Performing general reset.", current_frm);
            ret = tlv493d_general_reset(dev);
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
    // --- END ADC HANG-UP DETECTION ---

    if (!data->calibrated) {
        k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
        return;
    }

    int16_t delta_x = data->x - data->origin_x;
    int16_t delta_y = data->y - data->origin_y;
    int16_t delta_z = data->z - data->origin_z;
    bool report_sync = false;

    if (abs(delta_x) > XY_DEADZONE) {
        input_report_rel(dev, INPUT_REL_X, delta_x, false, K_FOREVER);
        report_sync = true;
    }
    if (abs(delta_y) > XY_DEADZONE) {
        input_report_rel(dev, INPUT_REL_Y, delta_y, false, K_FOREVER);
        report_sync = true;
    }
    if (abs(delta_z) > Z_DEADZONE) {
        input_report_rel(dev, INPUT_REL_WHEEL, (delta_z > 0 ? 1 : -1), false, K_FOREVER);
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
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)
