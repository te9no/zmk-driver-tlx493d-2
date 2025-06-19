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
#define TLV493D_REG_TEMP_LSB    0x06
#define TLV493D_REG_FACTORY1    0x07  // Factory setting register 1
#define TLV493D_REG_FACTORY2    0x08  // Factory setting register 2  
#define TLV493D_REG_FACTORY3    0x09  // Factory setting register 3
#define TLV493D_REG_MAP_SIZE    10  // レジスタマップのサイズ

// Write registers
#define TLV493D_REG_MOD1        0x01
#define TLV493D_REG_MOD2        0x03

// MOD1 register bits
#define TLV493D_MOD1_FASTMODE   BIT(1)

// MOD2 register bits  
#define TLV493D_MOD2_TEMP_EN    BIT(7)  // 正しいビット位置に修正（ビット7）

// Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00

// Bar graph settings
#define BAR_GRAPH_WIDTH 40
#define SENSOR_VALUE_MIN -100
#define SENSOR_VALUE_MAX 100

// Deadzone for X/Y movement to prevent spurious input
#define XY_DEADZONE 30 // Adjust as needed

// Deadzone for Z movement to prevent spurious input
#define Z_DEADZONE 30 // Adjust as needed

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

// ADCハング検出設定
#define ADC_HANG_DETECTION_SAMPLES 5  // 連続して同じ値の検出回数
#define ADC_HANG_DETECTION_THRESHOLD 2  // 最小変化量（ハング判定しない変化量）
#define ADC_HANG_RECOVERY_DELAY_MS 100  // ADCハング検出後の待機時間

// フレームカウンターとチャンネルフラグ整合性チェック設定
#define TLV493D_REG_TEMP_COUNTER_CHANNEL 0x03  // 温度、フレームカウンター、チャンネルレジスタ
#define FRM_MASK 0x0C    // ビット3:2 (フレームカウンター)
#define FRM_SHIFT 2      // フレームカウンターのビットシフト
#define CH_MASK 0x03     // ビット1:0 (チャンネルフラグ)
#define CH_VALID_VALUE 0x00  // 有効なチャンネル値（同じ変換からのデータ）
#define FRM_STUCK_THRESHOLD 3  // フレームカウンターが動かない許容回数

// Power management states
#define TLV493D_POWER_DOWN_DELAY_MS 100
#define TLV493D_POWER_UP_DELAY_MS   200
#define TLV493D_RESET_SEQUENCE_DELAY_MS 50

// Additional recovery commands
#define TLV493D_SOFT_RESET_CMD 0x00
#define TLV493D_POWER_DOWN_CMD 0xFF
#define TLV493D_POWER_UP_CMD   0x00

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
    uint8_t reg_map[TLV493D_REG_MAP_SIZE]; // レジスタマップのキャッシュ
    bool data_valid;        // データ有効性フラグ
    uint8_t parity_errors;  // パリティエラーカウンター
    int64_t last_movement_time; // 最後に移動を検知した時間（ms）
    bool movement_detected; // 移動検知フラグ
    uint8_t bus_recovery_attempts; // I2Cバスリカバリー試行回数
    
    // ADCハング検出用の変数
    int16_t prev_adc_x[ADC_HANG_DETECTION_SAMPLES];  // X軸の過去の値
    int16_t prev_adc_y[ADC_HANG_DETECTION_SAMPLES];  // Y軸の過去の値
    int16_t prev_adc_z[ADC_HANG_DETECTION_SAMPLES];  // Z軸の過去の値
    uint8_t adc_sample_index;  // サンプルインデックス
    bool adc_hang_detected;    // ADCハング検出フラグ
    uint8_t adc_hang_count;    // ADCハング検出回数
    
    // フレームカウンターとチャンネルフラグ用の変数
    uint8_t last_frm_counter;  // 前回のフレームカウンター値
    uint8_t frm_stuck_count;   // フレームカウンターが動かない回数
    bool frm_consistency_check_enabled; // フレームカウンター整合性チェック有効フラグ
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
static bool tlx493d_detect_adc_hang(const struct device *dev, int16_t x, int16_t y, int16_t z);
static int tlx493d_recover_from_adc_hang(const struct device *dev);
static bool tlx493d_check_frm_ch_consistency(const struct device *dev);

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
 * 1. スタート・ストップコンディションを送信
 * 2. 待機時間を延長
 * 3. リカバリーフレーム(0xFF)を送信
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

    // ZephyrのI2CバスリカバリーAPIがあれば利用する
    // 例: if (i2c_recover_bus(&config->i2c) == 0) {
    //         LOG_INF("Zephyr I2C bus recovery successful");
    //         return 0;
    //    }
    //    LOG_WRN("Zephyr I2C bus recovery failed, attempting manual recovery");

    // 1. I2Cバスをリセットするためのシーケンス
    //    空のメッセージを送信することで、バスの状態をリフレッシュする
    uint8_t dummy = 0;
    ret = i2c_write_dt(&config->i2c, &dummy, 0); // ストップコンディションを生成
    if (ret < 0) {
        LOG_WRN("I2C bus recovery: Failed to send stop condition (ret %d)", ret);
    }
    k_msleep(50); // 十分な待機時間

    // 2. リカバリーフレーム(0xFF)を複数回送信し、センサーをリセット状態に誘導
    for (int i = 0; i < 3; i++) { // 複数回送信を試みる
        ret = tlv493d_send_recovery_frame(dev);
        if (ret < 0) {
            LOG_WRN("I2C bus recovery: Recovery frame failed (attempt %d, ret %d)", i + 1, ret);
        }
        k_msleep(20); // 各送信間の短い待機
    }
    k_msleep(50); // 追加の待機時間

    // 3. リセットコマンド(0x00)を送信
    ret = tlv493d_send_reset_command(dev);
    if (ret < 0) {
        LOG_WRN("I2C bus recovery: Reset command failed (ret %d)", ret);
    }
    k_msleep(RESET_DELAY_MS); // リセット後の待機時間を確保

    // 4. 最終的なバスの状態確認（簡単な読み取りテスト）
    uint8_t test_read;
    ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
    if (ret == 0) {
        LOG_INF("I2C bus recovery completed successfully");
        data->bus_recovery_attempts = 0; // 成功したらカウンターをリセット
        return 0;
    } else {
        LOG_WRN("I2C bus recovery: Post-recovery read test failed (ret %d)", ret);
        if (data->bus_recovery_attempts >= MAX_BUS_RECOVERY_ATTEMPTS) {
            LOG_ERR("I2C bus recovery: Maximum attempts reached, bus may be unrecoverable without hardware reset.");
            data->bus_recovery_attempts = 0; // カウンターをリセット
            return -EIO; // 最終的に失敗
        }
        return ret; // リカバリー失敗
    }
}

/**
 * @brief 工場設定を読み取る
 *
 * マニュアル要求：レジスタ7-9を起動時に必ず読み取り、書き込み時に保持する
 * これらの値は書き込み操作時に必須（factory settingsを破損させないため）
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_read_factory_settings(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    
    // Read factory settings from registers 7-9 as required by user manual
    ret = tlx493d_read_multiple(dev, TLV493D_REG_FACTORY1, data->factory_settings, 3);
    if (ret < 0) {
        LOG_ERR("Failed to read factory settings from registers 7-9 (ret %d)", ret);
        return ret;
    }
    
    LOG_INF("Factory settings read: Reg7=0x%02X, Reg8=0x%02X, Reg9=0x%02X", 
            data->factory_settings[0], data->factory_settings[1], data->factory_settings[2]);
    
    return 0;
}

/**
 * @brief パリティビットを計算する
 *
 * マニュアル要求：全32ビット（書き込みレジスタ0-3）の和が奇数になるように
 * パリティビット（MOD1のビット7）を設定する
 *
 * @param reg0 レジスタ0の値
 * @param mod1 MOD1レジスタの値（パリティビット除く）
 * @param reg2 レジスタ2の値
 * @param mod2 MOD2レジスタの値
 * @return パリティビットを含むMOD1の値
 */
static uint8_t tlv493d_calculate_parity(uint8_t reg0, uint8_t mod1, uint8_t reg2, uint8_t mod2)
{
    uint32_t total_bits = 0;
    uint8_t regs[4] = {reg0, mod1, reg2, mod2};
    
    // 全32ビットの1の個数を数える
    for (int i = 0; i < 4; i++) {
        uint8_t reg_val = regs[i];
        for (int bit = 0; bit < 8; bit++) {
            if (reg_val & (1 << bit)) {
                total_bits++;
            }
        }
    }
    
    // 合計が偶数の場合、パリティビットを1にして奇数にする
    uint8_t parity_bit = (total_bits % 2 == 0) ? 0x80 : 0x00;
    uint8_t mod1_with_parity = (mod1 & 0x7F) | parity_bit;
    
    LOG_DBG("Parity calculation: total_bits=%d, parity_bit=0x%02X, MOD1=0x%02X", 
            total_bits, parity_bit, mod1_with_parity);
    
    return mod1_with_parity;
}

/**
 * @brief パリティチェック付きで4つの書き込みレジスタを一括設定する
 *
 * マニュアル要求に従い、パリティビットを自動計算して書き込む
 *
 * @param dev デバイス構造体へのポインタ
 * @param reg0 レジスタ0の値
 * @param mod1 MOD1レジスタの値（パリティビット除く）
 * @param reg2 レジスタ2の値
 * @param mod2 MOD2レジスタの値
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_write_config_with_parity(const struct device *dev, 
                                           uint8_t reg0, uint8_t mod1, 
                                           uint8_t reg2, uint8_t mod2)
{
    int ret;
    
    // パリティビット計算
    uint8_t mod1_with_parity = tlv493d_calculate_parity(reg0, mod1, reg2, mod2);
    
    // 全レジスタを順番に書き込み
    ret = tlx493d_write_reg(dev, 0x00, reg0);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_with_parity);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    ret = tlx493d_write_reg(dev, 0x02, reg2);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2);
    if (ret < 0) return ret;
    k_msleep(INIT_STEP_DELAY_MS);
    
    LOG_DBG("Config written with parity: Reg0=0x%02X, MOD1=0x%02X, Reg2=0x%02X, MOD2=0x%02X",
            reg0, mod1_with_parity, reg2, mod2);
    
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
    uint8_t reg0_val, mod1_val, reg2_val, mod2_val;
    
    // マニュアル要求に従った正しいfactory設定の保持
    // Register 0: Reserved (write 0x00)
    reg0_val = 0x00;
    
    // Register 1 (MOD1): Configure sensor operation mode
    // Preserve factory bits 6:3 from register 7, set operational bits
    // Note: パリティビット（bit 7）は後で計算される
    mod1_val = (data->factory_settings[0] & 0x78) |  // Keep factory bits 6:3
               TLV493D_MOD1_FASTMODE |                // Enable fast mode (bit 1)
               0x04;                                  // Enable interrupt (bit 2)
    
    // Register 2: Must contain factory settings from register 8
    reg2_val = data->factory_settings[1];  // All bits 7:0 from register 8
    
    // Register 3 (MOD2): Configure measurement settings  
    // Preserve factory bits 4:0 from register 9, configure measurement bits
    mod2_val = (data->factory_settings[2] & 0x1F) |  // Keep factory bits 4:0
               0x20;                                  // Enable parity test (bit 5)
               // Temperature disabled (bit 7 = 0) as requested
    
    // パリティビット計算付きで全レジスタを書き込み（マニュアル要求）
    ret = tlv493d_write_config_with_parity(dev, reg0_val, mod1_val, reg2_val, mod2_val);
    if (ret < 0) {
        LOG_ERR("Failed to write sensor configuration with parity (ret %d)", ret);
        return ret;
    }
    
    // パリティビット再計算（ログ表示用）
    mod1_val = tlv493d_calculate_parity(reg0_val, mod1_val, reg2_val, mod2_val);
    
    LOG_INF("Sensor configured with factory settings preserved:");
    LOG_INF("  Reg0=0x%02X, MOD1=0x%02X, Reg2=0x%02X, MOD2=0x%02X", 
            reg0_val, mod1_val, reg2_val, mod2_val);
    LOG_INF("  Factory: Reg7=0x%02X, Reg8=0x%02X, Reg9=0x%02X",
            data->factory_settings[0], data->factory_settings[1], data->factory_settings[2]);
    LOG_INF("  Parity bit: %s (MOD1 bit 7)", (mod1_val & 0x80) ? "SET" : "CLEAR");
    
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
 * @brief センサーの電源をサイクルする
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlv493d_power_cycle(const struct device *dev)
{
    const struct tlx493d_config *config = dev->config;
    struct tlx493d_data *data = dev->data;
    int ret;

    LOG_INF("Performing power cycle sequence");

    // Step 1: Power down sequence
    uint8_t power_down = TLV493D_POWER_DOWN_CMD;
    ret = i2c_write_dt(&config->i2c, &power_down, 1);
    if (ret < 0) {
        LOG_WRN("Power down command failed (ret %d)", ret);
    }
    k_msleep(TLV493D_POWER_DOWN_DELAY_MS);

    // Step 2: Send multiple reset frames
    for (int i = 0; i < 3; i++) {
        ret = tlv493d_send_recovery_frame(dev);
        k_msleep(TLV493D_RESET_SEQUENCE_DELAY_MS);
    }

    // Step 3: Power up sequence
    uint8_t power_up = TLV493D_POWER_UP_CMD;
    ret = i2c_write_dt(&config->i2c, &power_up, 1);
    if (ret < 0) {
        LOG_WRN("Power up command failed (ret %d)", ret);
    }
    k_msleep(TLV493D_POWER_UP_DELAY_MS);

    // Reset internal state
    data->initialized = false;
    data->calibrated = false;
    data->data_valid = false;
    data->error_count = 0;
    
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
    
    // ADCハング検出を実行
    bool adc_hang = tlx493d_detect_adc_hang(dev, data->x, data->y, data->z);
    if (adc_hang) {
        LOG_WRN("ADC hang detected, attempting recovery");
        ret = tlx493d_recover_from_adc_hang(dev);
        if (ret < 0) {
            LOG_ERR("ADC hang recovery failed (ret %d)", ret);
            data->data_valid = false;
            return ret;
        }
        // リカバリー成功後、データを再読み取り
        ret = tlx493d_read_multiple(dev, TLV493D_REG_BX_MSB, raw_data, 6);
        if (ret < 0) {
            LOG_ERR("Failed to re-read sensor data after ADC hang recovery (ret %d)", ret);
            data->data_valid = false;
            return ret;
        }
        
        // 値を再計算
        data->x = (int16_t)(((raw_data[1] << 4) | (raw_data[4] & 0x0F)) << 4) >> 4;
        data->y = (int16_t)(((raw_data[2] << 4) | (raw_data[5] & 0x0F)) << 4) >> 4;
        data->z = (int16_t)(((raw_data[0] << 4) | (raw_data[4] >> 4)) << 4) >> 4;
    }
    
    // フレームカウンター(FRM)とチャンネル(CH)フラグによるデータ整合性チェック
    bool frm_ch_valid = tlx493d_check_frm_ch_consistency(dev);
    if (!frm_ch_valid) {
        LOG_WRN("FRM/CH consistency check failed - data may be invalid");
        data->data_valid = false;
        return -EAGAIN; // データ整合性に問題があるため、再試行を促す
    }
    
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
 * @brief ADCハングを検出する
 *
 * Master Controlled ModeまたはFast Modeで連続して同じ値が読み取られる場合、
 * ADCがハングしている可能性があるため検出する。
 * 
 * @param dev デバイス構造体へのポインタ
 * @param x 現在のX軸値
 * @param y 現在のY軸値
 * @param z 現在のZ軸値
 * @return true: ADCハング検出、false: 正常
 */
static bool tlx493d_detect_adc_hang(const struct device *dev, int16_t x, int16_t y, int16_t z)
{
    struct tlx493d_data *data = dev->data;
    bool hang_detected = false;
    
    // 現在の値を履歴に保存
    data->prev_adc_x[data->adc_sample_index] = x;
    data->prev_adc_y[data->adc_sample_index] = y;
    data->prev_adc_z[data->adc_sample_index] = z;
    
    data->adc_sample_index = (data->adc_sample_index + 1) % ADC_HANG_DETECTION_SAMPLES;
    
    // 十分なサンプルが蓄積されてから検出開始
    static bool samples_filled = false;
    if (!samples_filled) {
        if (data->adc_sample_index == 0) {
            samples_filled = true;
        }
        return false;
    }
    
    // 過去N回のサンプルがすべて閾値以下の変化しかない場合、ADCハングと判定
    bool x_hung = true, y_hung = true, z_hung = true;
    
    for (int i = 1; i < ADC_HANG_DETECTION_SAMPLES; i++) {
        int prev_idx = (data->adc_sample_index - i + ADC_HANG_DETECTION_SAMPLES) % ADC_HANG_DETECTION_SAMPLES;
        int curr_idx = (data->adc_sample_index - i + 1 + ADC_HANG_DETECTION_SAMPLES) % ADC_HANG_DETECTION_SAMPLES;
        
        if (abs(data->prev_adc_x[curr_idx] - data->prev_adc_x[prev_idx]) > ADC_HANG_DETECTION_THRESHOLD) {
            x_hung = false;
        }
        if (abs(data->prev_adc_y[curr_idx] - data->prev_adc_y[prev_idx]) > ADC_HANG_DETECTION_THRESHOLD) {
            y_hung = false;
        }
        if (abs(data->prev_adc_z[curr_idx] - data->prev_adc_z[prev_idx]) > ADC_HANG_DETECTION_THRESHOLD) {
            z_hung = false;
        }
    }
    
    // すべての軸でハングが検出された場合
    if (x_hung && y_hung && z_hung) {
        hang_detected = true;
        data->adc_hang_count++;
        LOG_WRN("ADC hang detected: X=%d, Y=%d, Z=%d (count: %d)", x, y, z, data->adc_hang_count);
    } else {
        data->adc_hang_count = 0; // 正常な変化があった場合はカウンターをリセット
    }
    
    data->adc_hang_detected = hang_detected;
    return hang_detected;
}

/**
 * @brief ADCハングからリカバリーする
 *
 * ADCハングが検出された場合に、センサーをリセットして正常状態に戻す。
 * データシートに従って以下のシーケンスを実行：
 * 1. 一時的な電源サイクル（Master Controlled/Fast Modeのリセット）
 * 2. レジスタ設定の再初期化
 * 3. ADC状態のクリア
 *
 * @param dev デバイス構造体へのポインタ
 * @return 0: 成功、負の値: エラーコード
 */
static int tlx493d_recover_from_adc_hang(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    
    LOG_INF("Recovering from ADC hang (attempt %d)", data->adc_hang_count);
    
    // ステップ1: Fast ModeとMaster Controlled Modeの一時的な無効化
    uint8_t mod1_val = (data->factory_settings[0] & 0xF8); // Fast mode bit をクリア
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_val);
    if (ret < 0) {
        LOG_ERR("Failed to disable fast mode during ADC hang recovery (ret %d)", ret);
        return ret;
    }
    
    k_msleep(ADC_HANG_RECOVERY_DELAY_MS);
    
    // ステップ2: リカバリーフレームとリセットコマンドを送信
    ret = tlv493d_send_recovery_frame(dev);
    if (ret < 0) {
        LOG_WRN("Recovery frame failed during ADC hang recovery (ret %d)", ret);
    }
    
    k_msleep(50);
    
    ret = tlv493d_send_reset_command(dev);
    if (ret < 0) {
        LOG_WRN("Reset command failed during ADC hang recovery (ret %d)", ret);
    }
    
    k_msleep(ADC_HANG_RECOVERY_DELAY_MS);
    
    // ステップ3: Fast Modeを再度有効化
    mod1_val = (data->factory_settings[0] & 0xF8) | TLV493D_MOD1_FASTMODE;
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD1, mod1_val);
    if (ret < 0) {
        LOG_ERR("Failed to re-enable fast mode after ADC hang recovery (ret %d)", ret);
        return ret;
    }
    
    k_msleep(50);
    
    // ステップ4: ADCハング検出状態をリセット
    data->adc_hang_detected = false;
    data->adc_hang_count = 0;
    data->adc_sample_index = 0;
    memset(data->prev_adc_x, 0, sizeof(data->prev_adc_x));
    memset(data->prev_adc_y, 0, sizeof(data->prev_adc_y));
    memset(data->prev_adc_z, 0, sizeof(data->prev_adc_z));
    
    // ステップ5: センサーデータの読み取りテスト
    uint8_t test_data[6];
    ret = tlx493d_read_multiple(dev, TLV493D_REG_BX_MSB, test_data, 6);
    if (ret < 0) {
        LOG_ERR("Post-recovery sensor test failed (ret %d)", ret);
        return ret;
    }
    
    LOG_INF("ADC hang recovery completed successfully");
    return 0;
}

/**
 * @brief フレームカウンター(FRM)とチャンネル(CH)フラグによるデータ整合性チェック
 *
 * Infineonユーザーマニュアルに従って、センサーデータの整合性を確認する：
 * - FRM (Frame Counter): ビット3:2、連続する変換で増加する必要がある
 * - CH (Channel): ビット1:0、同じ変換からのデータには「00」である必要がある
 * 
 * FRMが増加しない場合はセンサーが「フリーズ」状態（例：電源ユニットが測定サイクルを
 * 開始しない、ADCが新しい測定を完了しない）を示す。
 * CHが「00」でない場合は、X/Y/Z/Tが異なる変換から来ていることを示し、変換が実行中。
 *
 * @param dev デバイス構造体へのポインタ
 * @return true: データ整合性OK、false: データ整合性に問題あり
 */
static bool tlx493d_check_frm_ch_consistency(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t temp_counter_channel_reg;
    int ret;
    
    // レジスタ3（温度、フレームカウンター、チャンネル）を読み取り
    ret = tlx493d_read_reg(dev, TLV493D_REG_TEMP_COUNTER_CHANNEL, &temp_counter_channel_reg);
    if (ret < 0) {
        LOG_ERR("Failed to read temp/counter/channel register for FRM/CH check (ret %d)", ret);
        return false;
    }
    
    // フレームカウンター（FRM）とチャンネル（CH）フラグを抽出
    uint8_t current_frm = (temp_counter_channel_reg & FRM_MASK) >> FRM_SHIFT;
    uint8_t current_ch = temp_counter_channel_reg & CH_MASK;
    
    LOG_DBG("FRM/CH check: FRM=%d, CH=%d, reg=0x%02X", current_frm, current_ch, temp_counter_channel_reg);
    
    // チャンネルフラグチェック：同じ変換からのデータは「00」である必要がある
    if (current_ch != CH_VALID_VALUE) {
        LOG_WRN("CH flag invalid: expected 0x%02X, got 0x%02X (conversion in progress)", 
                CH_VALID_VALUE, current_ch);
        return false;
    }
    
    // 初回読み取り時はフレームカウンターの初期化のみ行う
    if (!data->frm_consistency_check_enabled) {
        data->last_frm_counter = current_frm;
        data->frm_stuck_count = 0;
        data->frm_consistency_check_enabled = true;
        LOG_DBG("FRM/CH consistency check initialized with FRM=%d", current_frm);
        return true;
    }
    
    // フレームカウンターチェック：前回から変化しているかを確認
    if (current_frm == data->last_frm_counter) {
        data->frm_stuck_count++;
        LOG_WRN("FRM counter stuck: count=%d, FRM=%d (threshold=%d)", 
                data->frm_stuck_count, current_frm, FRM_STUCK_THRESHOLD);
        
        // 閾値を超えた場合、センサーがフリーズ状態と判定
        if (data->frm_stuck_count >= FRM_STUCK_THRESHOLD) {
            LOG_ERR("FRM counter stuck for %d consecutive reads - sensor frozen", 
                    data->frm_stuck_count);
            return false;
        }
    } else {
        // フレームカウンターが変化した場合、正常
        if (data->frm_stuck_count > 0) {
            LOG_INF("FRM counter resumed: %d -> %d (was stuck for %d reads)", 
                    data->last_frm_counter, current_frm, data->frm_stuck_count);
        }
        data->frm_stuck_count = 0;
        data->last_frm_counter = current_frm;
    }
    
    return true;
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

    // センサーが初期化されていない場合の処理（初回実行時または再初期化時）
    if (!data->initialized) {
        LOG_INF("Starting TLX493D sensor initialization...");
        
        // I2Cバスリカバリーを最初に実行
        tlv493d_i2c_bus_recovery(dev);

        // 初期化前に待機時間を追加
        k_msleep(100);
        
        // 連続エラー回数に応じて異なるリカバリー戦略を実行
        if (consecutive_errors >= 3) {
            LOG_WRN("Multiple initialization failures, performing power cycle");
            ret = tlv493d_power_cycle(dev);
            k_msleep(500); // 電源サイクル後の十分な待機時間
            consecutive_errors = 0;
        }
        
        // センサー初期化シーケンスを実行
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
        
        // 初期キャリブレーションを実行
        tlx493d_calibrate(dev);
        if (!data->calibrated) {
            LOG_WRN("Initial calibration failed. Will retry in next cycle.");
        }
        
        LOG_INF("TLX493D sensor initialization completed successfully");
    }

    // センサーデータの読み取りと処理
    ret = tlx493d_read_sensor_data(dev);
    if (ret != 0) {
        consecutive_errors++;
        last_error_time = current_time;
        LOG_ERR("Failed to read sensor data (ret %d, consecutive errors: %d)", ret, consecutive_errors);

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
    }
    // Remove dummy sync when no movement to reduce system load

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

    // Only log delta values when movement is detected to reduce log spam
    if (movement_detected_this_cycle) {
        LOG_DBG("Movement Delta: X=%d, Y=%d, Z=%d", delta_x, delta_y, delta_z);
    }

    // 次回のワークをスケジュール（移動検出時は高頻度、非検出時は低頻度）
    int next_interval = movement_detected_this_cycle ? 
                       DT_INST_PROP(0, polling_interval_ms) : 
                       DT_INST_PROP(0, polling_interval_ms) * 5; // 移動なしの場合は5倍の間隔
    k_work_schedule(&data->work, K_MSEC(next_interval));
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
    
    // ADCハング検出の初期化
    data->adc_sample_index = 0;
    data->adc_hang_detected = false;
    data->adc_hang_count = 0;
    memset(data->prev_adc_x, 0, sizeof(data->prev_adc_x));
    memset(data->prev_adc_y, 0, sizeof(data->prev_adc_y));
    memset(data->prev_adc_z, 0, sizeof(data->prev_adc_z));
    
    // フレームカウンター・チャンネルフラグ整合性チェックの初期化
    data->last_frm_counter = 0;
    data->frm_stuck_count = 0;
    data->frm_consistency_check_enabled = false;
    
    // レジスタマップを初期化
    memset(data->reg_map, 0, TLV493D_REG_MAP_SIZE);

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    // ZMK起動後3秒待機してからセンサー初期化を開始
    // ここでは基本的な設定のみ行い、実際のセンサー初期化は遅延させる
    LOG_INF("TLX493D driver will start initialization in 3 seconds...");

    k_work_init_delayable(&data->work, tlx493d_work_handler);
    // システムワークキューの代わりに専用ワークキューを使用してシステム負荷を軽減
    k_work_schedule(&data->work, K_MSEC(3000));

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
