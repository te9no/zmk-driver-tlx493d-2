/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

// TLV493D-A1B6 Registers
#define TLV493D_REG_BX_MSB      0x00
#define TLV493D_REG_BY_MSB      0x01
#define TLV493D_REG_BZ_MSB      0x02
#define TLV493D_REG_TEMP_MSB    0x03
#define TLV493D_REG_BX_LSB      0x04
#define TLV493D_REG_BY_LSB      0x05
#define TLV493D_REG_BZ_LSB      0x06
#define TLV493D_REG_TEMP_LSB    0x07
#define TLV493D_REG_FRM         0x08  // Frame Counter register
#define TLV493D_REG_CH          0x09  // Channel register
#define TLV493D_REG_MAP_SIZE    10    // レジスタマップのサイズ

// Write registers  
#define TLV493D_REG_MOD1        0x01
#define TLV493D_REG_MOD2        0x03

// MOD1 register bits
#define TLV493D_MOD1_FASTMODE   BIT(1)

// MOD2 register bits  
#define TLV493D_MOD2_TEMP_EN    BIT(7)  // 正しいビット位置に修正（ビット7）

// Frame Counter register bits (Register 0x08)
#define TLV493D_FRM_COUNTER_MASK 0x03  // Frame counter bits [1:0]

// Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00
#define TLV493D_GENERAL_RESET   0x00  // General address reset for ADC hang recovery

// Bar graph settings
#define BAR_GRAPH_WIDTH 40
#define SENSOR_VALUE_MIN -100
#define SENSOR_VALUE_MAX 100

// Deadzone for X/Y movement to prevent spurious input
#define XY_DEADZONE 3 // Adjust as needed

// Deadzone for Z movement to prevent spurious input
#define Z_DEADZONE 5 // Adjust as needed

// Hysteresis for movement detection
#define HYSTERESIS_THRESHOLD 20 // Adjust as needed

// Calibration settings
#define CALIBRATION_SAMPLES 100

// Error threshold for auto-recovery
#define ERROR_THRESHOLD 5

// Maximum retry attempts for initialization
#define MAX_INIT_RETRIES 3

// I2C通信タイムアウト (ms)
#define I2C_TIMEOUT_MS 100

// 初期化ステップ間の待機時間 (ms) - 増加
#define INIT_STEP_DELAY_MS 50  // 20msから50msに増加

// リセット後の待機時間 (ms) - 増加
#define RESET_DELAY_MS 100  // 50msから100msに増加

// 自動キャリブレーションのための無移動時間閾値 (ms)
#define AUTO_RECALIBRATION_TIMEOUT_MS 3000  // 30秒

// I2Cバスリカバリーの最大試行回数
#define MAX_BUS_RECOVERY_ATTEMPTS 5

// ADC hang detection settings
#define ADC_HANG_TIMEOUT_MS 5000        // 5秒でADCハング検出
#define FRAME_COUNTER_STUCK_THRESHOLD 10 // フレームカウンター停止検出の閾値

// Power management states
#define TLV493D_POWER_DOWN_DELAY_MS 100
#define TLV493D_POWER_UP_DELAY_MS   500
#define TLV493D_RESET_SEQUENCE_DELAY_MS 50

// Additional recovery commands
#define TLV493D_SOFT_RESET_CMD 0x00
#define TLV493D_POWER_DOWN_CMD 0xFF
#define TLV493D_POWER_UP_CMD   0x00

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    bool addr_pin_high;  // ADDR pin level configuration
    struct gpio_dt_spec reset_gpio;  // Hardware reset pin (active low)
    struct gpio_dt_spec power_gpio;  // Power control pin (active high)
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
    uint8_t reg_map[TLV493D_REG_MAP_SIZE]; // レジスタマップのキャッシュ
    bool data_valid;        // データ有効性フラグ
    uint8_t parity_errors;  // パリティエラーカウンター
    int64_t last_movement_time; // 最後に移動を検知した時間（ms）
    bool movement_detected; // 移動検知フラグ
    uint8_t bus_recovery_attempts; // I2Cバスリカバリー試行回数
    
    // ADC hang detection
    uint8_t last_frame_counter; // 前回のフレームカウンター値
    uint8_t frame_counter_stuck_count; // フレームカウンター停止回数
    int64_t last_frame_counter_check; // 前回のフレームカウンターチェック時間
    bool adc_hang_detected; // ADCハング検出フラグ
};

// 関数プロトタイプ宣言
static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val);
static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val);
static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len);
static int tlv493d_send_recovery_frame(const struct device *dev);
static int tlv493d_send_reset_command(const struct device *dev);
static int tlv493d_i2c_bus_recovery(const struct device *dev);
static int tlv493d_read_factory_settings(const struct device *dev);
static int tlv493d_configure_sensor(const struct device *dev);
static int tlv493d_diagnose(const struct device *dev);
static int tlv493d_initialize_sensor(const struct device *dev);
static int tlx493d_read_sensor_data(const struct device *dev);
static void tlx493d_calibrate(const struct device *dev);
static bool tlx493d_has_valid_data(const struct device *dev);
static bool tlx493d_is_functional(const struct device *dev);
static int tlx493d_update_reg_map(const struct device *dev);
static void tlx493d_check_auto_recalibration(const struct device *dev);
static void tlv493d_diagnose_error(const struct device *dev, int error_code);
static int tlv493d_power_cycle(const struct device *dev);
static int tlv493d_hardware_reset(const struct device *dev);
static int tlv493d_send_general_reset(const struct device *dev);
static bool tlv493d_check_adc_hang(const struct device *dev);
static int tlv493d_recover_from_adc_hang(const struct device *dev);

/**
 * @brief レジスタに値を書き込む
 *
 * @param dev デバイス構造体へのポインタ
 * @param reg_addr レジスタアドレス
 * @param val 書き込む値
 * @return 0: 成功、負の値: エラーコード
 */
static int tlx493d_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    uint8_t buf[2] = {reg_addr, val};
    int ret;
    
    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret == 0) {
        // 書き込みが成功したら、レジスタマップのキャッシュを更新
        if (reg_addr < TLV493D_REG_MAP_SIZE) {
            data->reg_map[reg_addr] = val;
        }
    } else {
        // エラー診断を実行
        tlv493d_diagnose_error(dev, ret);
    }
    
    return ret;
}

/**
 * @brief レジスタから値を読み取る
 *
 * @param dev デバイス構造体へのポインタ
 * @param reg_addr レジスタアドレス
 * @param val 読み取った値を格納するポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;
    
    ret = i2c_reg_read_byte_dt(&config->i2c, reg_addr, val);
    if (ret == 0) {
        // 読み取りが成功したら、レジスタマップのキャッシュを更新
        if (reg_addr < TLV493D_REG_MAP_SIZE) {
            data->reg_map[reg_addr] = *val;
        }
    } else {
        // エラー診断を実行
        tlv493d_diagnose_error(dev, ret);
    }
    
    return ret;
}

/**
 * @brief 複数のレジスタから連続して値を読み取る
 *
 * @param dev デバイス構造体へのポインタ
 * @param start_addr 開始レジスタアドレス
 * @param buf 読み取った値を格納するバッファ
 * @param len 読み取るバイト数
 * @return 0: 成功、負の値: エラーコード
 */
static int tlx493d_read_multiple(const struct device *dev, uint8_t start_addr, uint8_t *buf, uint8_t len)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;
    
    ret = i2c_burst_read_dt(&config->i2c, start_addr, buf, len);
    if (ret == 0) {
        // 読み取りが成功したら、レジスタマップのキャッシュを更新
        for (int i = 0; i < len && (start_addr + i) < TLV493D_REG_MAP_SIZE; i++) {
            data->reg_map[start_addr + i] = buf[i];
        }
    } else {
        // エラー診断を実行
        tlv493d_diagnose_error(dev, ret);
    }
    
    return ret;
}

/**
 * @brief エラーの詳細診断を行う
 *
 * @param dev デバイス構造体へのポインタ
 * @param error_code エラーコード
 */
static void tlv493d_diagnose_error(const struct device *dev, int error_code)
{
    LOG_ERR("I2C error occurred: %d", error_code);
    
    // エラーの種類に基づいた詳細診断
    switch (error_code) {
        case -5: // -EIO
            LOG_ERR("I/O Error - Check physical connections and pullup resistors");
            break;
        case -110: // -ETIMEDOUT
            LOG_ERR("Timeout Error - Sensor not responding");
            break;
        case -22: // -EINVAL
            LOG_ERR("Invalid parameter - Check I2C configuration");
            break;
        case -16: // -EBUSY
            LOG_ERR("Device or resource busy - I2C bus may be locked");
            break;
        default:
            LOG_ERR("Unknown error code");
            break;
    }
}

/**
 * @brief リカバリーフレームを送信する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_send_recovery_frame(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t recovery_cmd = TLV493D_RECOVERY_CMD;
    int ret;
    
    LOG_DBG("Sending recovery frame");
    ret = i2c_write_dt(&config->i2c, &recovery_cmd, 1);
    if (ret < 0) {
        tlv493d_diagnose_error(dev, ret);
    }
    return ret;
}

/**
 * @brief リセットコマンドを送信する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_send_reset_command(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    uint8_t reset_cmd = TLV493D_RESET_CMD;
    int ret;
    
    LOG_DBG("Sending reset command");
    ret = i2c_write_dt(&config->i2c, &reset_cmd, 1);
    if (ret < 0) {
        tlv493d_diagnose_error(dev, ret);
    }
    return ret;
}

/**
 * @brief I2Cバスをリカバリーする関数
 *
 * I2Cバスが不定状態になった場合に、バスをリセットして正常状態に戻す
 * 1. Zephyr I2Cリカバリー機能を使用
 * 2. クロックパルス生成でバスクリア
 * 3. リカバリーフレーム送信
 * 4. センサーリセット
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_i2c_bus_recovery(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;
    
    data->bus_recovery_attempts++;
    LOG_INF("Performing I2C bus recovery (attempt %d/%d)", 
            data->bus_recovery_attempts, MAX_BUS_RECOVERY_ATTEMPTS);

    // Step 1: ZephyrのI2Cバスリカバリー機能を試行（利用可能な場合）
    // Note: Zephyr I2C recover API は全てのドライバーで利用可能ではないため、
    // 手動リカバリーを実行する
    LOG_DBG("Performing manual I2C bus recovery sequence");

    // Step 2: 手動バスクリア - 9クロックパルス生成でSDAライン解放
    LOG_DBG("Generating 9 clock pulses to clear stuck I2C transaction");
    for (int i = 0; i < 9; i++) {
        // ダミーリードでクロックパルス生成
        uint8_t dummy;
        ret = i2c_reg_read_byte_dt(&config->i2c, 0xFF, &dummy);
        k_usleep(10); // 10μs間隔
    }
    k_msleep(10);

    // Step 3: ストップコンディション強制生成
    uint8_t stop_dummy = 0;
    ret = i2c_write_dt(&config->i2c, &stop_dummy, 0);
    if (ret < 0) {
        LOG_WRN("I2C bus recovery: Failed to send stop condition (ret %d)", ret);
    }
    k_msleep(50);

    // Step 4: センサー固有のリカバリーシーケンス
    // 複数のリカバリーフレーム(0xFF)を送信
    for (int i = 0; i < 5; i++) {
        ret = tlv493d_send_recovery_frame(dev);
        if (ret < 0) {
            LOG_WRN("I2C bus recovery: Recovery frame %d failed (ret %d)", i + 1, ret);
        }
        k_msleep(20);
    }
    k_msleep(100);

    // Step 5: センサーリセットシーケンス
    for (int i = 0; i < 3; i++) {
        ret = tlv493d_send_reset_command(dev);
        if (ret < 0) {
            LOG_WRN("I2C bus recovery: Reset command %d failed (ret %d)", i + 1, ret);
        }
        k_msleep(RESET_DELAY_MS);
    }
    k_msleep(200); // 最終安定化時間

test_recovery:
    // Step 6: バス状態検証 - 複数レジスタでテスト
    uint8_t test_values[3];
    bool recovery_success = true;
    
    for (int i = 0; i < 3; i++) {
        ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB + i, &test_values[i]);
        if (ret < 0) {
            LOG_WRN("I2C bus recovery: Test read %d failed (ret %d)", i, ret);
            recovery_success = false;
            break;
        }
        k_msleep(5);
    }
    
    if (recovery_success) {
        LOG_INF("I2C bus recovery completed successfully");
        LOG_DBG("Test reads: 0x%02X 0x%02X 0x%02X", test_values[0], test_values[1], test_values[2]);
        data->bus_recovery_attempts = 0;
        return 0;
    } else {
        LOG_WRN("I2C bus recovery failed validation");
        if (data->bus_recovery_attempts >= MAX_BUS_RECOVERY_ATTEMPTS) {
            LOG_ERR("I2C bus recovery: Maximum attempts reached, sensor may need hardware reset");
            data->bus_recovery_attempts = 0;
            return -EIO;
        }
        return -EAGAIN; // リトライを促す
    }
}

/**
 * @brief 工場設定を読み取る
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
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

/**
 * @brief センサーを設定する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
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
    k_msleep(INIT_STEP_DELAY_MS);
    
    // Configure MOD2: Disable temperature measurement (ユーザー要望により無効化)
    // Preserve factory settings but ensure temperature bit is cleared
    mod2_val = data->factory_settings[2] & (~TLV493D_MOD2_TEMP_EN);
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2_val);
    if (ret < 0) {
        LOG_ERR("Failed to write MOD2 register (ret %d)", ret);
        return ret;
    }
    
    // 書き込み後の待機時間を追加
    k_msleep(INIT_STEP_DELAY_MS);
    
    LOG_INF("Sensor configured: MOD1=0x%02X, MOD2=0x%02X (temperature disabled)", mod1_val, mod2_val);
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
    uint8_t reg_values[TLV493D_REG_MAP_SIZE];
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
    if ((mod2_val & TLV493D_MOD2_TEMP_EN) != 0) {
        LOG_WRN("Temperature measurement still enabled in MOD2 register");
    } else {
        LOG_INF("Temperature measurement correctly disabled in MOD2 register");
    }
    
    return 0;
}

/**
 * @brief General address 0x00 reset command for ADC hang recovery
 * 
 * Per TLX493D manual section 5.6, send general address 0x00 to reset
 * the sensor when ADC conversion hangs up in Master Controlled or Fast Mode.
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_send_general_reset(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    int ret;
    
    LOG_INF("Sending general address reset (0x00) for ADC hang recovery");
    
    // Send general address 0x00 - this is a special I2C transaction
    // that resets the sensor's ADC conversion state machine
    uint8_t general_reset_addr = 0x00;
    
    // Create an I2C message with the general reset address
    struct i2c_msg msg = {
        .buf = &general_reset_addr,
        .len = 1,
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP
    };
    
    ret = i2c_transfer(config->i2c.bus, &msg, 1, 0x00); // Use general call address 0x00
    if (ret < 0) {
        LOG_ERR("General reset command failed (ret %d)", ret);
        return ret;
    }
    
    LOG_DBG("General reset command sent successfully");
    k_msleep(RESET_DELAY_MS); // Wait for reset to complete
    
    return 0;
}

/**
 * @brief ADCハング状態をチェックする
 *
 * フレームカウンターの更新状況を監視し、ADCがハングアップしているかを検出する
 * TLX493D manual section 5.6に基づく実装
 *
 * @param dev デバイス構造体へのポインタ
 * @return true: ADCハング検出、false: 正常動作
 */
static bool tlv493d_check_adc_hang(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t current_frame_counter;
    int ret;
    int64_t current_time = k_uptime_get();
    
    // Read current frame counter (FRM register 0x08)
    ret = tlx493d_read_reg(dev, TLV493D_REG_FRM, &current_frame_counter);
    if (ret < 0) {
        LOG_WRN("Failed to read frame counter for ADC hang detection (ret %d)", ret);
        return false; // Can't detect hang if we can't read the register
    }
    
    // Extract frame counter bits [1:0]
    current_frame_counter &= TLV493D_FRM_COUNTER_MASK;
    
    // Initialize on first check
    if (data->last_frame_counter_check == 0) {
        data->last_frame_counter = current_frame_counter;
        data->last_frame_counter_check = current_time;
        data->frame_counter_stuck_count = 0;
        return false;
    }
    
    // Check if enough time has passed for a frame counter update
    int64_t time_elapsed = current_time - data->last_frame_counter_check;
    if (time_elapsed < 100) { // Check every 100ms minimum
        return data->adc_hang_detected;
    }
    
    // Check if frame counter has incremented
    if (current_frame_counter == data->last_frame_counter) {
        data->frame_counter_stuck_count++;
        LOG_DBG("Frame counter stuck at %d (count: %d)", 
                current_frame_counter, data->frame_counter_stuck_count);
        
        // If frame counter stuck for too long, declare ADC hang
        if (data->frame_counter_stuck_count >= FRAME_COUNTER_STUCK_THRESHOLD) {
            if (!data->adc_hang_detected) {
                LOG_WRN("ADC hang detected: Frame counter stuck at %d for %d checks", 
                        current_frame_counter, data->frame_counter_stuck_count);
                data->adc_hang_detected = true;
            }
            return true;
        }
    } else {
        // Frame counter incremented - ADC is working
        if (data->adc_hang_detected) {
            LOG_INF("ADC hang cleared: Frame counter incremented from %d to %d", 
                    data->last_frame_counter, current_frame_counter);
        }
        data->frame_counter_stuck_count = 0;
        data->adc_hang_detected = false;
        data->last_frame_counter = current_frame_counter;
        data->last_frame_counter_check = current_time;
    }
    
    return false;
}

/**
 * @brief ADCハングからのリカバリーを実行する
 *
 * TLX493D manual section 5.6に従って：
 * 1. General address 0x00でセンサーをリセット
 * 2. Master Controlled Mode / Fast Modeに再設定
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_recover_from_adc_hang(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    
    LOG_INF("Recovering from ADC hang condition");
    
    // Step 1: Send general address 0x00 to reset the sensor
    ret = tlv493d_send_general_reset(dev);
    if (ret < 0) {
        LOG_ERR("General reset failed during ADC hang recovery (ret %d)", ret);
        return ret;
    }
    
    // Step 2: Re-read factory settings
    ret = tlv493d_read_factory_settings(dev);
    if (ret < 0) {
        LOG_ERR("Failed to read factory settings during ADC hang recovery (ret %d)", ret);
        return ret;
    }
    
    // Step 3: Re-configure sensor to Master Controlled Mode / Fast Mode
    ret = tlv493d_configure_sensor(dev);
    if (ret < 0) {
        LOG_ERR("Failed to re-configure sensor during ADC hang recovery (ret %d)", ret);
        return ret;
    }
    
    // Reset ADC hang detection state
    data->adc_hang_detected = false;
    data->frame_counter_stuck_count = 0;
    data->last_frame_counter_check = 0;
    data->last_frame_counter = 0;
    
    LOG_INF("ADC hang recovery completed successfully");
    return 0;
}

/**
 * @brief ハードウェアリセットを実行する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_hardware_reset(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret = 0;

    LOG_INF("Performing hardware reset sequence");

    // Step 1: Hardware power cycle if power GPIO is available
    if (gpio_is_ready_dt(&config->power_gpio)) {
        LOG_DBG("Power cycling sensor via GPIO");
        
        // Power off
        ret = gpio_pin_set_dt(&config->power_gpio, 0);
        if (ret < 0) {
            LOG_WRN("Failed to power off sensor (ret %d)", ret);
        }
        k_msleep(200); // Extended power-off time
        
        // Power on
        ret = gpio_pin_set_dt(&config->power_gpio, 1);
        if (ret < 0) {
            LOG_WRN("Failed to power on sensor (ret %d)", ret);
        }
        k_msleep(TLV493D_POWER_UP_DELAY_MS); // Extended power-up time
    }

    // Step 2: Hardware reset if reset GPIO is available
    if (gpio_is_ready_dt(&config->reset_gpio)) {
        LOG_DBG("Performing hardware reset via GPIO");
        
        // Assert reset (active low)
        ret = gpio_pin_set_dt(&config->reset_gpio, 0);
        if (ret < 0) {
            LOG_WRN("Failed to assert reset (ret %d)", ret);
        }
        k_msleep(50); // Hold reset for 50ms
        
        // Release reset
        ret = gpio_pin_set_dt(&config->reset_gpio, 1);
        if (ret < 0) {
            LOG_WRN("Failed to release reset (ret %d)", ret);
        }
        k_msleep(TLV493D_POWER_UP_DELAY_MS); // Wait for sensor to stabilize
    }

    // Reset internal state
    data->initialized = false;
    data->calibrated = false;
    data->data_valid = false;
    data->error_count = 0;
    data->bus_recovery_attempts = 0;
    
    LOG_INF("Hardware reset sequence completed");
    return ret;
}

/**
 * @brief センサーの電源をサイクルする（ソフトウェア版）
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_power_cycle(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;

    LOG_INF("Performing software power cycle sequence");

    // Try hardware reset first if available
    if (gpio_is_ready_dt(&config->reset_gpio) || gpio_is_ready_dt(&config->power_gpio)) {
        return tlv493d_hardware_reset(dev);
    }

    // Fallback to software power cycle
    // Step 1: Extended power down sequence
    for (int i = 0; i < 5; i++) {
        uint8_t power_down = TLV493D_POWER_DOWN_CMD;
        ret = i2c_write_dt(&config->i2c, &power_down, 1);
        if (ret < 0) {
            LOG_WRN("Power down command %d failed (ret %d)", i + 1, ret);
        }
        k_msleep(50);
    }
    k_msleep(TLV493D_POWER_DOWN_DELAY_MS);

    // Step 2: Multiple recovery frames to clear any stuck states
    for (int i = 0; i < 10; i++) {
        ret = tlv493d_send_recovery_frame(dev);
        k_msleep(TLV493D_RESET_SEQUENCE_DELAY_MS);
    }

    // Step 3: Extended power up sequence
    for (int i = 0; i < 3; i++) {
        uint8_t power_up = TLV493D_POWER_UP_CMD;
        ret = i2c_write_dt(&config->i2c, &power_up, 1);
        if (ret < 0) {
            LOG_WRN("Power up command %d failed (ret %d)", i + 1, ret);
        }
        k_msleep(100);
    }
    k_msleep(TLV493D_POWER_UP_DELAY_MS);

    // Reset internal state
    data->initialized = false;
    data->calibrated = false;
    data->data_valid = false;
    data->error_count = 0;
    
    LOG_INF("Software power cycle sequence completed");
    return 0;
}

/**
 * @brief センサーを初期化する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    uint8_t test_read;
    int retry_count = 0;
    const int max_init_attempts = MAX_INIT_RETRIES;
    
    LOG_INF("Starting TLV493D sensor initialization sequence");
    
    // 初期化前に電源サイクルを実行
    ret = tlv493d_power_cycle(dev);
    if (ret < 0) {
        LOG_WRN("Power cycle failed (ret %d), proceeding with initialization attempts.", ret);
    }
    
    k_msleep(200); // 電源サイクル後の十分な待機時間
    
    while (retry_count < max_init_attempts) {
        LOG_DBG("Initialization attempt %d/%d", retry_count + 1, max_init_attempts);

        // Step 1: リカバリーフレームを複数回送信
        for (int i = 0; i < 3; i++) {
            ret = tlv493d_send_recovery_frame(dev);
            if (ret < 0) {
                LOG_WRN("Recovery frame %d failed (ret %d)", i + 1, ret);
            }
            k_msleep(INIT_STEP_DELAY_MS);
        }
        
        // Step 2: リセットコマンドを複数回送信
        for (int i = 0; i < 3; i++) {
            ret = tlv493d_send_reset_command(dev);
            if (ret < 0) {
                LOG_WRN("Reset command %d failed (ret %d)", i + 1, ret);
            }
            k_msleep(RESET_DELAY_MS);
        }
        
        // Step 3: I2C通信テスト
        ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
        if (ret < 0) {
            LOG_WRN("I2C communication test failed during init (attempt %d, ret %d)", retry_count + 1, ret);
            goto next_attempt; // 次の試行へ
        }
        
        // Step 4: 工場設定の読み取り
        ret = tlv493d_read_factory_settings(dev);
        if (ret < 0) {
            LOG_WRN("Failed to read factory settings during init (attempt %d, ret %d)", retry_count + 1, ret);
            goto next_attempt; // 次の試行へ
        }
        k_msleep(INIT_STEP_DELAY_MS); // 待機時間を追加
        
        // Step 5: センサー設定
        ret = tlv493d_configure_sensor(dev);
        if (ret < 0) {
            LOG_WRN("Failed to configure sensor during init (attempt %d, ret %d)", retry_count + 1, ret);
            goto next_attempt; // 次の試行へ
        }
        
        // 全ステップ成功
        data->initialized = true;
        data->init_retries = 0; // リトライカウンターをリセット
        data->data_valid = false; // 初期化直後はデータ無効
        LOG_INF("TLV493D sensor initialization completed successfully");
        
        tlv493d_diagnose(dev); // 診断を実行
        tlx493d_update_reg_map(dev); // レジスタマップを更新
        
        return 0; // 成功

    next_attempt:
        retry_count++;
        // 試行間の遅延を指数関数的に増やす（例: 50ms, 100ms, 200ms...）
        k_msleep(INIT_STEP_DELAY_MS * (1 << retry_count)); 
    }
    
    // 最大リトライ回数を超えた場合
    LOG_ERR("TLV493D sensor initialization failed after %d attempts", max_init_attempts);
    data->init_retries++; // 初期化失敗回数をカウント
    return -EIO;
}

/**
 * @brief レジスタマップを更新する
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlx493d_update_reg_map(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    
    // 全レジスタを読み取り
    ret = tlx493d_read_multiple(dev, 0, data->reg_map, TLV493D_REG_MAP_SIZE);
    if (ret < 0) {
        LOG_ERR("Failed to update register map (ret %d)", ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief センサーデータが有効かどうかを確認する
 *
 * @param dev デバイス構造体へのポインタ
 * @return true: データ有効、false: データ無効
 */
static bool tlx493d_has_valid_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    
    // 初期化されていない場合は無効
    if (!data->initialized) {
        return false;
    }
    
    // データ有効フラグをチェック
    return data->data_valid;
}

/**
 * @brief センサーが機能しているかどうかを確認する
 *
 * @param dev デバイス構造体へのポインタ
 * @return true: 機能している、false: 機能していない
 */
static bool tlx493d_is_functional(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t test_val;
    int ret;
    
    // 初期化されていない場合は機能していない
    if (!data->initialized) {
        return false;
    }
    
    // 通信テスト
    ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_val);
    if (ret < 0) {
        return false;
    }
    
    // MOD1レジスタの設定を確認
    ret = tlx493d_read_reg(dev, TLV493D_REG_MOD1, &test_val);
    if (ret < 0 || (test_val & TLV493D_MOD1_FASTMODE) == 0) {
        return false;
    }
    
    // MOD2レジスタの設定を確認（温度測定が無効になっていることを確認）
    ret = tlx493d_read_reg(dev, TLV493D_REG_MOD2, &test_val);
    if (ret < 0 || (test_val & TLV493D_MOD2_TEMP_EN) != 0) {
        return false;
    }
    
    return true;
}

/**
 * @brief バーグラフ文字列を生成する
 *
 * @param value センサー値
 * @param buffer 文字列を格納するバッファ
 * @param buffer_size バッファサイズ
 */
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

/**
 * @brief センサーデータを読み取る
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
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
        data->data_valid = false;
        return ret;
    }

    // Convert 12-bit values (MSB + 4 bits from LSB)
    data->x = (int16_t)(((raw_data[1] << 4) | (raw_data[4] & 0x0F)) << 4) >> 4;
    // data->y = (int16_t)(((raw_data[1] << 4) | (raw_data[4] & 0x0F)) << 4) >> 4;
    // data->z = (int16_t)(((raw_data[2] << 4) | (raw_data[5] & 0x0F)) << 4) >> 4;
    data->y = (int16_t)(((raw_data[2] << 4) | (raw_data[5] & 0x0F)) << 4) >> 4;
    data->z = (int16_t)(((raw_data[0] << 4) | (raw_data[4] >> 4)) << 4) >> 4;
    
    // データ有効フラグを設定
    data->data_valid = true;
    
    generate_bar_graph(data->x, x_bar, sizeof(x_bar));
    generate_bar_graph(data->y, y_bar, sizeof(y_bar));
    generate_bar_graph(data->z, z_bar, sizeof(z_bar));

    LOG_INF("X: %5d %s", data->x, x_bar);
    LOG_INF("Y: %5d %s", data->y, y_bar);
    LOG_INF("Z: %5d %s", data->z, z_bar);

    return 0;
}

/**
 * @brief センサーをキャリブレーションする
 *
 * @param dev デバイス構造体へのポインタ
 */
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
    
    // キャリブレーション後に最終移動時間を更新
    data->last_movement_time = k_uptime_get();
    data->movement_detected = false;

    LOG_INF("Calibration complete. Origin X: %d, Y: %d, Z: %d", 
            data->origin_x, data->origin_y, data->origin_z);
}

/**
 * @brief 自動キャリブレーションをチェックする
 * 
 * 30秒間移動が検出されなかった場合、自動的にキャリブレーションを実行する
 *
 * @param dev デバイス構造体へのポインタ
 */
static void tlx493d_check_auto_recalibration(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int64_t current_time = k_uptime_get();
    int64_t elapsed_time = current_time - data->last_movement_time;
    
    // 移動が検出されていない状態で30秒経過した場合、自動キャリブレーション
    if (!data->movement_detected && elapsed_time >= AUTO_RECALIBRATION_TIMEOUT_MS) {
        LOG_INF("No movement detected for %lld ms, performing auto-recalibration", elapsed_time);
        tlx493d_calibrate(dev);
        LOG_INF("Auto-recalibration completed");
    }
}

/**
 * @brief ワークハンドラー
 *
 * @param work ワーク構造体へのポインタ
 */
static void tlx493d_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
    const struct device *dev = data->dev;
    int ret;
    static int consecutive_errors = 0;
    static int64_t last_error_time = 0;
    int64_t current_time = k_uptime_get();

    // エラー状態の時間ベースリセット
    if (current_time - last_error_time > 60000) { // 1分以上エラーがない
        consecutive_errors = 0;
    }

    // センサーが初期化されていない場合の処理
    if (!data->initialized) {
        // 連続エラー回数に応じて異なるリカバリー戦略を実行
        if (consecutive_errors >= 3) {
            LOG_WRN("Multiple initialization failures, performing power cycle");
            ret = tlv493d_power_cycle(dev);
            k_msleep(500); // 電源サイクル後の十分な待機時間
            consecutive_errors = 0;
        }
        
        ret = tlv493d_initialize_sensor(dev);
        if (ret != 0) {
            LOG_ERR("Failed to initialize sensor in work handler (ret %d)", ret);
            consecutive_errors++;
            last_error_time = current_time;
            
            // エラー回数に応じて待機時間を指数関数的に増加
            int delay_ms = MIN(1000 * (1 << consecutive_errors), 30000);
            k_work_schedule(&data->work, K_MSEC(delay_ms));
            return;
        }
    }

    // ADCハング検出チェック
    if (tlv493d_check_adc_hang(dev)) {
        LOG_WRN("ADC hang detected, attempting recovery");
        ret = tlv493d_recover_from_adc_hang(dev);
        if (ret != 0) {
            LOG_ERR("ADC hang recovery failed (ret %d), performing full power cycle", ret);
            data->initialized = false;
            tlv493d_power_cycle(dev);
        }
        // ADCハング回復後は次回まで待機
        k_work_schedule(&data->work, K_MSEC(1000));
        return;
    }

    // センサーデータの読み取りと処理
    ret = tlx493d_read_sensor_data(dev);
    if (ret != 0) {
        consecutive_errors++;
        last_error_time = current_time;
        LOG_ERR("Failed to read sensor data (ret %d, consecutive errors: %d)", ret, consecutive_errors);

        // ADCハングの可能性をチェック
        if (consecutive_errors >= 3) {
            if (tlv493d_check_adc_hang(dev)) {
                LOG_INF("Sensor read errors may be due to ADC hang, attempting recovery");
                ret = tlv493d_recover_from_adc_hang(dev);
                if (ret == 0) {
                    consecutive_errors = 0; // ADCリカバリー成功
                    k_work_schedule(&data->work, K_MSEC(500));
                    return;
                }
            }
        }

        if (consecutive_errors >= 5) {
            LOG_WRN("Too many consecutive errors, performing recovery sequence");
            data->initialized = false;  // 強制的に再初期化
            tlv493d_power_cycle(dev);
            consecutive_errors = 0;
        }
        
        // エラー時は次回の実行を遅延
        int delay_ms = MIN(1000 * (1 << consecutive_errors), 30000);
        k_work_schedule(&data->work, K_MSEC(delay_ms));
        return;
    }

    // 正常に読み取れた場合
    consecutive_errors = 0;
    data->error_count = 0;

    int16_t current_x, current_y, current_z;
    int16_t delta_x, delta_y, delta_z;
    bool movement_detected_this_cycle = false;

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
        movement_detected_this_cycle = true;
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
        movement_detected_this_cycle = true;
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
        movement_detected_this_cycle = true;
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

    // 移動検出状態の更新
    if (movement_detected_this_cycle) {
        // 移動を検出した場合、最終移動時間を更新
        data->last_movement_time = k_uptime_get();
        data->movement_detected = true;
        LOG_DBG("Movement detected, updating last_movement_time: %lld", data->last_movement_time);
    } else if (data->movement_detected) {
        // 前回移動を検出していたが、今回検出しなかった場合
        data->movement_detected = false;
        LOG_DBG("Movement stopped, waiting for auto-recalibration timeout");
    }

    LOG_INF("Calibrated Delta: X=%d, Y=%d, Z=%d", delta_x, delta_y, delta_z);

    // 次回のワークをスケジュール（正常時は通常の間隔）
    k_work_schedule(&data->work, K_MSEC(DT_INST_PROP(0, polling_interval_ms)));
}

/**
 * @brief デバイス初期化関数
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
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
    data->data_valid = false;
    data->parity_errors = 0;
    data->last_movement_time = k_uptime_get();
    data->movement_detected = false;
    data->bus_recovery_attempts = 0;
    
    // Initialize ADC hang detection
    data->last_frame_counter = 0;
    data->frame_counter_stuck_count = 0;
    data->last_frame_counter_check = 0;
    data->adc_hang_detected = false;
    
    // レジスタマップを初期化
    memset(data->reg_map, 0, TLV493D_REG_MAP_SIZE);

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    // Initialize GPIO pins if present
    int ret;
    if (gpio_is_ready_dt(&config->reset_gpio)) {
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO (ret %d)", ret);
            return ret;
        }
        LOG_DBG("Reset GPIO configured");
    }
    
    if (gpio_is_ready_dt(&config->power_gpio)) {
        ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure power GPIO (ret %d)", ret);
            return ret;
        }
        // Ensure sensor is powered on
        ret = gpio_pin_set_dt(&config->power_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to power on sensor (ret %d)", ret);
            return ret;
        }
        k_msleep(TLV493D_POWER_UP_DELAY_MS);
        LOG_DBG("Power GPIO configured and sensor powered on");
    }

    // Perform hardware reset if available, otherwise I2C bus recovery
    if (gpio_is_ready_dt(&config->reset_gpio) || gpio_is_ready_dt(&config->power_gpio)) {
        tlv493d_hardware_reset(dev);
    } else {
        tlv493d_i2c_bus_recovery(dev);
    }

    // 初期化前に長めの待機時間を追加
    k_msleep(100);

    // Perform sensor initialization sequence according to datasheet
    ret = tlv493d_initialize_sensor(dev);
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
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)
