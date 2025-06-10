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

// TLV493D-A1B6 I2C Addresses (based on ADDR pin level)
#define TLV493D_ADDR_LOW_READ   0xBD
#define TLV493D_ADDR_LOW_WRITE  0xBC
#define TLV493D_ADDR_HIGH_READ  0x3F
#define TLV493D_ADDR_HIGH_WRITE 0x3E

// TLV493D-A1B6 Registers
#define TLV493D_REG_BX_MSB      0x00
#define TLV493D_REG_BX_LSB      0x01
#define TLV493D_REG_BY_MSB      0x02
#define TLV493D_REG_BY_LSB      0x03
#define TLV493D_REG_BZ_MSB      0x04
#define TLV493D_REG_BZ_LSB      0x05
#define TLV493D_REG_TEMP_MSB    0x06
#define TLV493D_REG_TEMP_LSB    0x07
#define TLV493D_REG_FRM         0x08
#define TLV493D_REG_CH          0x09

// Write registers
#define TLV493D_REG_MOD1        0x01
#define TLV493D_REG_MOD2        0x03

// MOD1 register bits
#define TLV493D_MOD1_FASTMODE   BIT(1)
#define TLV493D_MOD1_LPMODE     BIT(0)

// MOD2 register bits  
#define TLV493D_MOD2_TEMP_EN    BIT(0)

// Recovery and reset commands
#define TLV493D_RECOVERY_CMD    0xFF
#define TLV493D_RESET_CMD       0x00

struct tlx493d_config {
    struct i2c_dt_spec i2c;
    bool addr_pin_high;  // ADDR pin level configuration
};

struct tlx493d_data {
    const struct device *dev;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t prev_x;
    int16_t prev_y;
    struct k_work_delayable work;
    uint8_t factory_settings[3];  // Store bytes 7-9 for write operations
    bool initialized;
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
    
    // Configure MOD2: Enable temperature measurement
    // Preserve factory settings and enable temperature
    mod2_val = data->factory_settings[2] | TLV493D_MOD2_TEMP_EN;
    
    ret = tlx493d_write_reg(dev, TLV493D_REG_MOD2, mod2_val);
    if (ret < 0) {
        LOG_ERR("Failed to write MOD2 register (ret %d)", ret);
        return ret;
    }
    
    LOG_INF("Sensor configured: MOD1=0x%02X, MOD2=0x%02X", mod1_val, mod2_val);
    return 0;
}

static int tlv493d_initialize_sensor(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    int ret;
    uint8_t test_read;
    
    LOG_INF("Starting TLV493D sensor initialization sequence");
    
    // Step 1: Optional recovery frame
    ret = tlv493d_send_recovery_frame(dev);
    if (ret < 0) {
        LOG_WRN("Recovery frame failed, continuing (ret %d)", ret);
    }
    
    // Step 2: Send reset command  
    ret = tlv493d_send_reset_command(dev);
    if (ret < 0) {
        LOG_ERR("Reset command failed (ret %d)", ret);
        return ret;
    }
    
    // Wait for sensor to stabilize after reset
    k_msleep(10);
    
    // Step 3: Test I2C communication by reading a register
    ret = tlx493d_read_reg(dev, TLV493D_REG_BX_MSB, &test_read);
    if (ret < 0) {
        LOG_ERR("I2C communication test failed (ret %d)", ret);
        return ret;
    }
    
    // Step 4: Read and store factory settings (bytes 7-9)
    ret = tlv493d_read_factory_settings(dev);
    if (ret < 0) {
        return ret;
    }
    
    // Step 5: Configure sensor for operation
    ret = tlv493d_configure_sensor(dev);
    if (ret < 0) {
        return ret;
    }
    
    data->initialized = true;
    LOG_INF("TLV493D sensor initialization completed successfully");
    
    return 0;
}

static int tlx493d_read_sensor_data(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    uint8_t raw_data[6];
    int ret;

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
    data->x = (int16_t)(((raw_data[0] << 4) | (raw_data[1] >> 4)) << 4) >> 4; // Sign extend
    data->y = (int16_t)(((raw_data[2] << 4) | (raw_data[3] >> 4)) << 4) >> 4; // Sign extend  
    data->z = (int16_t)(((raw_data[4] << 4) | (raw_data[5] >> 4)) << 4) >> 4; // Sign extend

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

        // Report relative mouse movement if there's any change
        if (delta_x != 0 || delta_y != 0) {
            if (delta_x != 0 && delta_y != 0) {
                // Both X and Y movement - sync on the last event
                input_report_rel(dev, INPUT_REL_X, delta_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_FOREVER);
            } else if (delta_x != 0) {
                // Only X movement - sync on X
                input_report_rel(dev, INPUT_REL_X, delta_x, true, K_FOREVER);
            } else {
                // Only Y movement - sync on Y
                input_report_rel(dev, INPUT_REL_Y, delta_y, true, K_FOREVER);
            }
        }

        // Update previous values
        data->prev_x = data->x;
        data->prev_y = data->y;
    }

    k_work_schedule(&data->work, K_MSEC(10));
}

static int tlx493d_init(const struct device *dev)
{
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *config = dev->config;

    data->dev = dev;
    data->prev_x = 0;
    data->prev_y = 0;
    data->initialized = false;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s not ready", config->i2c.bus->name);
        return -ENODEV;
    }

    // Perform sensor initialization sequence according to datasheet
    int ret = tlv493d_initialize_sensor(dev);
    if (ret != 0) {
        LOG_ERR("Sensor initialization failed (ret %d)", ret);
        return ret;
    }

    // Initial read to set prev_x and prev_y
    ret = tlx493d_read_sensor_data(dev);
    if (ret != 0) {
        LOG_ERR("Failed to read initial sensor data (ret %d)", ret);
        return ret;
    }
    data->prev_x = data->x;
    data->prev_y = data->y;

    k_work_init_delayable(&data->work, tlx493d_work_handler);
    k_work_schedule(&data->work, K_MSEC(10));

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

