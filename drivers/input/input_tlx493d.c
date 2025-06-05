#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <stdlib.h>

#include "input_tlx493d.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

#define HYSTERESIS_HIGH_THRESHOLD 15  // 動きを検出する上側しきい値
#define HYSTERESIS_LOW_THRESHOLD  5   // 動きを停止する下側しきい値

#define TLX493D_LOG_INTERVAL_MS 3000
#define TLX493D_SENSITIVITY_DIVISOR 10
#define TLX493D_CALIBRATION_DELAY_MS 50

#define GRAPH_WIDTH 40     // グラフの最大幅
#define GRAPH_SCALE 100    // グラフのスケール（値をこの値で割って正規化）

struct tlx493d_data {
    const struct device *dev;
    struct k_work_delayable work;
    int16_t last_x;
    int16_t last_y;
    int16_t last_z;
    bool sleep_mode;
    uint32_t log_timer;  // Add timer for logging
    int16_t calib_x;    // キャリブレーション基準値
    int16_t calib_y;
    int16_t calib_z;
    bool calibrated;    // キャリブレーション完了フラグ
    bool movement_active;  // 動きの状態を追跡
};

struct tlx493d_config {
    struct i2c_dt_spec i2c;
};

static int tlx493d_read_sensor_data(const struct device *dev, int16_t *x, int16_t *y, int16_t *z) {
    const struct tlx493d_config *config = dev->config;
    uint8_t data[6];
    
    // TLX493Dの各軸のデータを読み取り
    int ret = i2c_burst_read_dt(&config->i2c, TLX493D_R_BX, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    // Convert raw data to 12-bit signed values per datasheet
    *x = ((data[0] << 4) | (data[1] >> 4));
    *y = ((data[2] << 4) | (data[3] >> 4));
    *z = ((data[4] << 4) | (data[5] >> 4));

    return 0;
}

static int tlx493d_calibrate(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    int16_t x, y, z;
    int ret;
    
    // センサーの値が安定するまで少し待機
    k_sleep(K_MSEC(TLX493D_CALIBRATION_DELAY_MS));
    
    // 現在の値を読み取り、基準値として保存
    ret = tlx493d_read_sensor_data(dev, &x, &y, &z);
    if (ret < 0) {
        LOG_ERR("Calibration failed: %d", ret);
        return ret;
    }
    
    data->calib_x = x;
    data->calib_y = y;
    data->calib_z = z;
    data->calibrated = true;
    
    LOG_INF("Calibration done - X: %d, Y: %d, Z: %d", x, y, z);
    return 0;
}

static void log_bar_graph(const char *label, int16_t value) {
    char bar[GRAPH_WIDTH + 1] = {0};
    int bars = (abs(value) * GRAPH_WIDTH) / GRAPH_SCALE;
    if (bars > GRAPH_WIDTH) bars = GRAPH_WIDTH;

    // 棒グラフの作成
    if (value > 0) {
        memset(bar, '>', bars);
    } else {
        memset(bar, '<', bars);
    }
    
    LOG_INF("%s: %4d [%c%s%*s]", label, value, 
            value >= 0 ? '|' : ' ', bar, 
            GRAPH_WIDTH - bars, "");
}

static void tlx493d_work_cb(struct k_work *work) {
    struct tlx493d_data *data = CONTAINER_OF(work, struct tlx493d_data, work.work);
    int16_t x, y, z;
    
    if (data->sleep_mode) {
        // Skip reading when in sleep mode
        k_work_reschedule(&data->work, K_MSEC(CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS));
        return;
    }

    if (tlx493d_read_sensor_data(data->dev, &x, &y, &z) == 0) {
        // キャリブレーション基準値からの相対値を計算
        int16_t dx = x - data->calib_x;
        int16_t dy = y - data->calib_y;
        int16_t dz = z - data->calib_z;
        
        // ヒステリシス制御による動き検出
        bool report_movement = false;
        int16_t abs_dx = abs(dx);
        int16_t abs_dy = abs(dy);
        
        if (!data->movement_active) {
            // 非アクティブ状態での判定（上側しきい値）
            if (abs_dx > HYSTERESIS_HIGH_THRESHOLD || abs_dy > HYSTERESIS_HIGH_THRESHOLD) {
                data->movement_active = true;
                report_movement = true;
            }
        } else {
            // アクティブ状態での判定（下側しきい値）
            if (abs_dx > HYSTERESIS_LOW_THRESHOLD || abs_dy > HYSTERESIS_LOW_THRESHOLD) {
                report_movement = true;
            } else {
                data->movement_active = false;
            }
        }

        // センサー値のログ出力
        uint32_t now = k_uptime_get_32();
        if ((now - data->log_timer) >= TLX493D_LOG_INTERVAL_MS) {
            LOG_INF("Sensor values [%s]:", data->movement_active ? "ACTIVE" : "IDLE");
            log_bar_graph("X", dx);
            log_bar_graph("Y", dy);
            log_bar_graph("Z", dz);
            data->log_timer = now;
        }

        // 動きを報告
        if (report_movement) {
            input_report_rel(data->dev, INPUT_REL_X, dx / TLX493D_SENSITIVITY_DIVISOR, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, dy / TLX493D_SENSITIVITY_DIVISOR, true, K_FOREVER);
            
            data->last_x = x;
            data->last_y = y;
            data->last_z = z;
        }
    }

    k_work_reschedule(&data->work, K_MSEC(CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS));
}

int tlx493d_set_sleep(const struct device *dev, bool sleep) {
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t mode;

    data->sleep_mode = sleep;
    
    // Set power mode according to sleep state
    mode = sleep ? TLX493D_POWER_MODE_LOW : TLX493D_POWER_MODE_MCM;
    
    return i2c_reg_write_byte_dt(&config->i2c, TLX493D_R_MOD1, mode);
}

static int tlx493d_i2c_read_retried(const struct device *dev, uint8_t reg, uint8_t *data, size_t len) {
    const struct tlx493d_config *config = dev->config;
    int ret;

    for (int i = 0; i < TLX493D_I2C_RETRIES; i++) {
        ret = i2c_burst_read_dt(&config->i2c, reg, data, len);
        if (ret == 0) {
            return 0;
        }
        k_sleep(K_MSEC(TLX493D_I2C_RETRY_DELAY_MS));
    }
    
    return ret;
}

static int tlx493d_i2c_init(const struct device *dev) {
    const struct tlx493d_config *config = dev->config;
    int ret;

    // Simple communication test with the device
    uint8_t test_byte;
    ret = i2c_reg_read_byte_dt(&config->i2c, TLX493D_R_ID, &test_byte);
    if (ret < 0) {
        LOG_ERR("I2C communication test failed: %d", ret);
        return ret;
    }

    return 0;
}

static int tlx493d_verify_id(const struct device *dev) {
    const struct tlx493d_config *config = dev->config;
    uint8_t id;
    int ret;

    // まずI2C通信を初期化
    ret = tlx493d_i2c_init(dev);
    if (ret < 0) {
        LOG_ERR("I2C initialization failed");
        return ret;
    }

    for (int i = 0; i < TLX493D_INIT_RETRY_COUNT; i++) {
        ret = i2c_reg_read_byte_dt(&config->i2c, TLX493D_R_ID, &id);
        if (ret >= 0) {
            uint8_t product_id = id & TLX493D_PRODUCT_ID_MASK;
            uint8_t version = id & TLX493D_VERSION_MASK;
            
            LOG_INF("Read ID: 0x%02x (Product: 0x%02x, Version: 0x%02x)",
                   id, product_id, version);
                   
            if (product_id == TLX493D_PRODUCT_ID) {
                return 0;
            }
            LOG_WRN("Unexpected product ID: 0x%02x", product_id);
        } else {
            LOG_ERR("Failed to read ID: %d (0x%08x)", ret, ret);
        }
        
        LOG_INF("Retrying device identification (%d/%d)", i + 1, TLX493D_INIT_RETRY_COUNT);
        k_sleep(K_MSEC(TLX493D_INIT_RETRY_DELAY_MS));
    }
    
    LOG_ERR("Device identification failed after %d attempts", TLX493D_INIT_RETRY_COUNT);
    return -ENODEV;
}

static int tlx493d_reset(const struct device *dev) {
    const struct tlx493d_config *config = dev->config;
    int ret;

    // 1. Power down
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_R_MOD1, 0x00);
    if (ret < 0) return ret;
    LOG_INF("Power down command sent");
    k_sleep(K_MSEC(100));

    // 2. ソフトリセット実行
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_R_MOD1, 0x01);
    if (ret < 0) return ret;
    LOG_INF("Soft reset command sent");
    k_sleep(K_MSEC(100));

    // 3. Master Controlled Mode設定
    // PRD[1:0]=01, IICadr=1, MOD=01 -> 0x15
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_R_MOD1, 0x15);
    if (ret < 0) return ret;
    LOG_INF("Master Controlled Mode set");
    k_sleep(K_MSEC(100));

    // 4. センサー設定
    // P=0, T=1, Fast=1 -> 0x90
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_R_CONFIG, 0x90);
    if (ret < 0) return ret;
    LOG_INF("Sensor configuration set to 0x90");
    
    return 0;
}

static const struct tlx493d_retry_config INIT_RETRY = {
    .retries = TLX493D_INIT_RETRY_COUNT,
    .delay_ms = TLX493D_INIT_RETRY_DELAY_MS
};

static const struct tlx493d_retry_config I2C_RETRY = {
    .retries = TLX493D_I2C_RETRIES,
    .delay_ms = TLX493D_I2C_RETRY_DELAY_MS
};

static int tlx493d_i2c_write_with_retry(const struct device *dev, uint8_t reg, uint8_t *data, size_t len,
                                       const struct tlx493d_retry_config *retry_config) {
    const struct tlx493d_config *config = dev->config;
    int ret;
    
    for (uint8_t attempt = 0; attempt <= retry_config->retries; attempt++) {
        ret = i2c_burst_write_dt(&config->i2c, reg, data, len);
        if (ret == 0) {
            return 0;
        }
        
        if (attempt < retry_config->retries) {
            LOG_WRN("I2C write failed (attempt %d/%d): %d", 
                    attempt + 1, retry_config->retries + 1, ret);
            k_msleep(retry_config->delay_ms);
        }
    }
    
    LOG_ERR("I2C write failed after %d attempts: %d", 
            retry_config->retries + 1, ret);
    return ret;
}

static int tlx493d_init(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    LOG_INF("Initializing TLX493D");

    // デバイスの初期化
    ret = tlx493d_reset(dev);
    if (ret < 0) {
        return ret;
    }

    // 初期化後の動作モード確認
    uint8_t mode;
    ret = i2c_reg_read_byte_dt(&config->i2c, TLX493D_R_MOD1, &mode);
    if (ret < 0) {
        LOG_ERR("Failed to read mode: %d", ret);
        return ret;
    }
    LOG_INF("Current mode: 0x%02x", mode);

    data->dev = dev;
    data->sleep_mode = false;
    data->log_timer = k_uptime_get_32();
    data->calibrated = false;
    data->movement_active = false;  // 初期状態は非アクティブ

    // センサーのキャリブレーションを実行
    ret = tlx493d_calibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to calibrate sensor");
        return ret;
    }

    // Initialize and start polling work
    k_work_init_delayable(&data->work, tlx493d_work_cb);
    k_work_schedule(&data->work, K_MSEC(CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS));

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int tlx493d_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return tlx493d_set_sleep(dev, true);
    case PM_DEVICE_ACTION_RESUME:
        return tlx493d_set_sleep(dev, false);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define TLX493D_INIT(n) \
    static struct tlx493d_data tlx493d_data_##n; \
    static const struct tlx493d_config tlx493d_config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, tlx493d_pm_action); \
    DEVICE_DT_INST_DEFINE(n, tlx493d_init, \
                         PM_DEVICE_DT_INST_GET(n), \
                         &tlx493d_data_##n, \
                         &tlx493d_config_##n, \
                         POST_KERNEL, \
                         CONFIG_INPUT_TLX493D_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)

