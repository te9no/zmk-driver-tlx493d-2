#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include "input_tlx493d.h"

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

struct tlx493d_data {
    const struct device *dev;
    struct k_work_delayable work;
    int16_t last_x;
    int16_t last_y;
    int16_t last_z;
    bool sleep_mode;
    uint32_t log_timer;  // Add timer for logging
    bool active_x;        // X軸の動作状態
    bool active_y;        // Y軸の動作状態
    float scale;         // 変換係数
};

struct tlx493d_config {
    struct i2c_dt_spec i2c;
};

static int tlx493d_read_sensor_data(const struct device *dev, int16_t *x, int16_t *y, int16_t *z) {
    const struct tlx493d_config *config = dev->config;
    uint8_t data[6];
    int ret;
    
    // バースト読み出し（6バイト: BX, BY, BZ, TEMP, BX2, BZ2）
    ret = i2c_burst_read_dt(&config->i2c, TLX493D_REG_B_X, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    // データシートに従って12ビット値に変換
    *x = ((data[0] << 4) | ((data[4] & 0xF0) >> 4));
    *y = ((data[1] << 4) | (data[4] & 0x0F));
    *z = ((data[2] << 4) | ((data[5] & 0xF0) >> 4));

    // 12ビット符号付き整数への変換
    if (*x & 0x800) *x |= 0xF000;
    if (*y & 0x800) *y |= 0xF000;
    if (*z & 0x800) *z |= 0xF000;

    return 0;
}

static int convert_to_mouse_movement(float value, bool *is_active) {
    if (!*is_active && (abs(value) > TLX493D_THRESHOLD)) {
        *is_active = true;
    } else if (*is_active && (abs(value) < (TLX493D_THRESHOLD - TLX493D_HYSTERESIS))) {
        *is_active = false;
    }

    if (*is_active) {
        float movement = value * TLX493D_SCALE;
        return (int)CLAMP(movement, -10, 10);
    }
    return 0;
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
        float x_val = (float)x;
        float y_val = (float)y;
        
        int dx = convert_to_mouse_movement(x_val, &data->active_x);
        int dy = convert_to_mouse_movement(y_val, &data->active_y);

        // Log sensor values
        uint32_t now = k_uptime_get_32();
        if ((now - data->log_timer) >= TLX493D_LOG_INTERVAL_MS) {
            LOG_INF("Sensor: X=%d(%d) Y=%d(%d) Z=%d [%s]", 
                   x, dx, y, dy, z,
                   (data->active_x || data->active_y) ? "ACTIVE" : "IDLE");
            data->log_timer = now;
        }

        if (dx != 0 || dy != 0) {
            input_report_rel(data->dev, INPUT_REL_X, dx, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_FOREVER);
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
    mode = sleep ? TLX493D_MODE_LOW_POWER : TLX493D_MODE_MCM_FAST;
    
    return i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_B_MOD1, mode);
}

static int tlx493d_reset(const struct device *dev) {
    const struct tlx493d_config *config = dev->config;
    int ret;

    // Power down mode
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_B_MOD1, TLX493D_MODE_DISABLE);
    if (ret < 0) return ret;
    k_sleep(K_MSEC(TLX493D_RESET_DELAY_MS));

    // Fast mode + MCM configuration
    uint8_t config_val = TLX493D_CONFIG_FAST;  // Fast mode, temp disabled
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_CONFIG, config_val);
    if (ret < 0) return ret;

    // Enable MCM mode
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_REG_B_MOD1, TLX493D_MODE_MCM_FAST);
    if (ret < 0) return ret;

    k_sleep(K_MSEC(TLX493D_POWER_UP_DELAY_MS));
    return 0;
}

static int tlx493d_init(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // デバイスの初期化
    ret = tlx493d_reset(dev);
    if (ret < 0) {
        return ret;
    }

    data->dev = dev;
    data->sleep_mode = false;
    data->log_timer = k_uptime_get_32();  // Initialize log timer

    // Initialize and start polling work
    k_work_init_delayable(&data->work, tlx493d_work_cb);
    k_work_schedule(&data->work, K_MSEC(CONFIG_INPUT_TLX493D_POLLING_INTERVAL_MS));

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int tlx493d_pm_action(const struct device *dev, enum pm_device_action action) {
    struct tlx493d_data *data = dev->data;
    
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

