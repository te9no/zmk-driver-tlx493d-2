#define DT_DRV_COMPAT infineon_tlx493d

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

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
};

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    uint32_t polling_interval_ms;
};

static int tlx493d_read_sensor_data(const struct device *dev, int16_t *x, int16_t *y, int16_t *z) {
    const struct tlx493d_config *config = dev->config;
    uint8_t data[7];
    
    int ret = i2c_burst_read_dt(&config->i2c, TLX493D_READ_REG, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    // Convert raw data to 12-bit signed values per datasheet
    *x = ((data[0] << 4) | (data[4] >> 4)) << 4;
    *x >>= 4; // Sign extend
    
    *y = ((data[1] << 4) | (data[4] & 0x0F)) << 4;
    *y >>= 4;
    
    *z = ((data[2] << 4) | (data[5] >> 4)) << 4;
    *z >>= 4;

    return 0;
}

static void tlx493d_work_cb(struct k_work *work) {
    struct tlx493d_data *data = CONTAINER_OF(work, struct tlx493d_data, work.work);
    const struct tlx493d_config *config = data->dev->config;
    int16_t x, y, z;
    
    if (data->sleep_mode) {
        // Skip reading when in sleep mode
        k_work_reschedule(&data->work, K_MSEC(config->polling_interval_ms));
        return;
    }

    if (tlx493d_read_sensor_data(data->dev, &x, &y, &z) == 0) {
        // Calculate relative movement
        int16_t dx = x - data->last_x;
        int16_t dy = y - data->last_y;
        
        // Report relative movement if it exceeds threshold
        if (abs(dx) > 10 || abs(dy) > 10) {
            input_report_rel(data->dev, INPUT_REL_X, dx / 10, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, dy / 10, true, K_FOREVER);
            
            // Update last values
            data->last_x = x;
            data->last_y = y;
            data->last_z = z;
        }
    }

    k_work_reschedule(&data->work, K_MSEC(config->polling_interval_ms));
}

int tlx493d_set_sleep(const struct device *dev, bool sleep) {
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t mode;

    data->sleep_mode = sleep;
    
    // Set power mode according to sleep state
    mode = sleep ? TLX493D_POWER_MODE_LOW : TLX493D_POWER_MODE_MCM;
    
    return i2c_reg_write_byte_dt(&config->i2c, TLX493D_MOD1_REG, mode);
}

static int tlx493d_init(const struct device *dev) {
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;
    uint8_t chip_id;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // Read and verify chip ID
    ret = i2c_reg_read_byte_dt(&config->i2c, TLX493D_WHOAMI_REG, &chip_id);
    if (ret < 0 || chip_id != TLX493D_CHIP_ID) {
        LOG_ERR("Failed to verify chip ID");
        return -EINVAL;
    }

    // Initialize sensor in Master Controlled Mode
    ret = i2c_reg_write_byte_dt(&config->i2c, TLX493D_MOD1_REG, TLX493D_POWER_MODE_MCM);
    if (ret < 0) {
        LOG_ERR("Failed to set MCM mode");
        return ret;
    }

    data->dev = dev;
    data->sleep_mode = false;

    // Initialize and start polling work
    k_work_init_delayable(&data->work, tlx493d_work_cb);
    k_work_schedule(&data->work, K_MSEC(config->polling_interval_ms));

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int TLX493D_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define TLX493D_INIT(n) \
    static struct tlx493d_data tlx493d_data_##n; \
    static const struct tlx493d_config tlx493d_config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
        .polling_interval_ms = DT_INST_PROP(n, polling_interval_ms), \
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
