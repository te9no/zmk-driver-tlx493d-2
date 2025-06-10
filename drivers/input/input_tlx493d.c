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

LOG_MODULE_REGISTER(tlx493d, CONFIG_INPUT_LOG_LEVEL);

// TLX493D I2C Address (Adjust based on actual sensor configuration)
#define TLX493D_I2C_ADDR 0x35 // Example address, verify from datasheet

// TLX493D Registers (Refer to datasheet for complete list and details)
#define TLX493D_REG_BX1 0x00
#define TLX493D_REG_BX2 0x01
#define TLX493D_REG_BY1 0x02
#define TLX493D_REG_BY2 0x03
#define TLX493D_REG_BZ1 0x04
#define TLX493D_REG_BZ2 0x05
#define TLX493D_REG_TEMP1 0x06
#define TLX493D_REG_TEMP2 0x07
// ... other registers for configuration, status, etc.

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    // Add other configuration parameters if needed (e.g., sensitivity)
};

struct tlx493d_data {
    const struct device *dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t prev_x;
    int16_t prev_y;
    struct k_work_delayable work;
    // Add other data fields if needed
};

static int tlx493d_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
    const struct tlx493d_config *config = dev->config;
    return i2c_reg_read_byte_dt(&config->i2c, reg_addr, val);
}

static int tlx493d_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t bx1, bx2, by1, by2, bz1, bz2;
    int ret;

    // Read X-axis data
    ret = tlx493d_read_reg(dev, TLX493D_REG_BX1, &bx1);
    if (ret < 0) {
        LOG_ERR("Failed to read BX1 (ret %d)", ret);
        return ret;
    }
    ret = tlx493d_read_reg(dev, TLX493D_REG_BX2, &bx2);
    if (ret < 0) {
        LOG_ERR("Failed to read BX2 (ret %d)", ret);
        return ret;
    }
    data->x = (int16_t)(((bx1 & 0x0F) << 8) | bx2); // Assuming 12-bit data, adjust as per datasheet

    // Read Y-axis data
    ret = tlx493d_read_reg(dev, TLX493D_REG_BY1, &by1);
    if (ret < 0) {
        LOG_ERR("Failed to read BY1 (ret %d)", ret);
        return ret;
    }
    ret = tlx493d_read_reg(dev, TLX493D_REG_BY2, &by2);
    if (ret < 0) {
        LOG_ERR("Failed to read BY2 (ret %d)", ret);
        return ret;
    }
    data->y = (int16_t)(((by1 & 0x0F) << 8) | by2); // Assuming 12-bit data, adjust as per datasheet

    // Read Z-axis data
    ret = tlx493d_read_reg(dev, TLX493D_REG_BZ1, &bz1);
    if (ret < 0) {
        LOG_ERR("Failed to read BZ1 (ret %d)", ret);
        return ret;
    }
    ret = tlx493d_read_reg(dev, TLX493D_REG_BZ2, &bz2);
    if (ret < 0) {
        LOG_ERR("Failed to read BZ2 (ret %d)", ret);
        return ret;
    }
    data->z = (int16_t)(((bz1 & 0x0F) << 8) | bz2); // Assuming 12-bit data, adjust as per datasheet

    LOG_DBG("Sensor data: X=%d, Y=%d, Z=%d", data->x, data->y, data->z);
    return 0;
}

static void tlx493d_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
    const struct device *dev = data->dev;
    int ret;
    int16_t delta_x, delta_y;

    ret = tlx493d_read_sensor_data(dev);
    if (ret == 0) {
        // Calculate delta for mouse movement
        delta_x = data->x - data->prev_x;
        delta_y = data->y - data->prev_y;

        // Report relative mouse movement
        if (delta_x != 0) {
            input_report_rel(dev, INPUT_REL_X, delta_x, false, K_FOREVER);
        }
        if (delta_y != 0) {
            input_report_rel(dev, INPUT_REL_Y, delta_y, false, K_FOREVER);
        }
        input_sync(dev);

        // Update previous values
        data->prev_x = data->x;
        data->prev_y = data->y;
    }

    k_work_schedule(&data->work, K_MSEC(CONFIG_TLX493D_POLLING_INTERVAL_MS)); // Reschedule work
}

static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;

    data->dev = dev;
    data->prev_x = 0;
    data->prev_y = 0;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    // TODO: Add sensor initialization sequence if required by datasheet
    // (e.g., writing to configuration registers)

    // Initial read to set prev_x and prev_y
    int ret = tlx493d_read_sensor_data(dev);
    if (ret != 0) {
        LOG_ERR("Failed to read initial sensor data (ret %d)", ret);
        return ret;
    }
    data->prev_x = data->x;
    data->prev_y = data->y;

    k_work_init_delayable(&data->work, tlx493d_work_handler);
    k_work_schedule(&data->work, K_MSEC(CONFIG_TLX493D_POLLING_INTERVAL_MS));

    LOG_INF("TLX493D initialized successfully on %s", dev->name);
    return 0;
}

#define TLX493D_DEFINE(inst) \
    static struct tlx493d_data tlx493d_data_##inst; \
    static const struct tlx493d_config tlx493d_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, tlx493d_init, NULL, \
                          &tlx493d_data_##inst, &tlx493d_config_##inst, \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                          NULL); // No API struct for now

DT_INST_FOREACH_STATUS_OKAY(TLX493D_DEFINE)


