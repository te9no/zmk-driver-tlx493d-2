#define DT_DRV_COMPAT cirque_TLX493D

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "INPUT_TLX493D.h"

LOG_MODULE_REGISTER(TLX493D, CONFIG_INPUT_LOG_LEVEL);

static int TLX493D_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct TLX493D_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int TLX493D_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct TLX493D_config *config = dev->config;
    return config->write(dev, addr, val);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int TLX493D_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct TLX493D_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, TLX493D_READ | addr, buf, len);
}

static int TLX493D_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct TLX493D_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, TLX493D_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int set_int(const struct device *dev, const bool en) {
    const struct TLX493D_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int TLX493D_clear_status(const struct device *dev) {
    int ret = TLX493D_write(dev, TLX493D_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int TLX493D_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = TLX493D_write(dev, TLX493D_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_write(dev, TLX493D_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_write(dev, TLX493D_REG_ERA_CONTROL, TLX493D_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = TLX493D_seq_read(dev, TLX493D_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = TLX493D_seq_read(dev, TLX493D_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int TLX493D_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = TLX493D_write(dev, TLX493D_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_write(dev, TLX493D_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_write(dev, TLX493D_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = TLX493D_write(dev, TLX493D_REG_ERA_CONTROL, TLX493D_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = TLX493D_seq_read(dev, TLX493D_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = TLX493D_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static void TLX493D_report_data(const struct device *dev) {
    const struct TLX493D_config *config = dev->config;
    uint8_t packet[3];
    int ret;
    ret = TLX493D_seq_read(dev, TLX493D_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "TLX493D Status1");

    // Ignore 0xFF packets that indicate communcation failure, or if SW_DR isn't asserted
    if (packet[0] == 0xFF || !(packet[0] & TLX493D_STATUS1_SW_DR)) {
        return;
    }
    ret = TLX493D_seq_read(dev, TLX493D_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "TLX493D Packets");

    struct TLX493D_data *data = dev->data;
    uint8_t btn = packet[0] &
                  (TLX493D_PACKET0_BTN_PRIM | TLX493D_PACKET0_BTN_SEC | TLX493D_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & TLX493D_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & TLX493D_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    if (data->in_int) {
        LOG_DBG("Clearing status bit");
        ret = TLX493D_clear_status(dev);
        data->in_int = true;
    }

    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
            }
        }
    }

    data->btn_cache = btn;

    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);

    return;
}

static void TLX493D_work_cb(struct k_work *work) {
    struct TLX493D_data *data = CONTAINER_OF(work, struct TLX493D_data, work);
    TLX493D_report_data(data->dev);
}

static void TLX493D_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct TLX493D_data *data = CONTAINER_OF(cb, struct TLX493D_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    data->in_int = true;
    k_work_submit(&data->work);
}

static int TLX493D_adc_sensitivity_reg_value(enum TLX493D_sensitivity sensitivity) {
    switch (sensitivity) {
    case TLX493D_SENSITIVITY_1X:
        return TLX493D_TRACKING_ADC_CONFIG_1X;
    case TLX493D_SENSITIVITY_2X:
        return TLX493D_TRACKING_ADC_CONFIG_2X;
    case TLX493D_SENSITIVITY_3X:
        return TLX493D_TRACKING_ADC_CONFIG_3X;
    case TLX493D_SENSITIVITY_4X:
        return TLX493D_TRACKING_ADC_CONFIG_4X;
    default:
        return TLX493D_TRACKING_ADC_CONFIG_1X;
    }
}

static int TLX493D_tune_edge_sensitivity(const struct device *dev) {
    const struct TLX493D_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = TLX493D_era_read(dev, TLX493D_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = TLX493D_era_read(dev, TLX493D_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = TLX493D_era_write(dev, TLX493D_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = TLX493D_era_write(dev, TLX493D_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int TLX493D_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct TLX493D_config *config = dev->config;

    uint8_t val;
    int ret = TLX493D_era_read(dev, TLX493D_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= TLX493D_adc_sensitivity_reg_value(config->sensitivity);

    ret = TLX493D_era_write(dev, TLX493D_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = TLX493D_era_read(dev, TLX493D_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int TLX493D_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = TLX493D_seq_read(dev, TLX493D_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = TLX493D_write(dev, TLX493D_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        TLX493D_seq_read(dev, TLX493D_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int TLX493D_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = TLX493D_seq_read(dev, TLX493D_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & TLX493D_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, TLX493D_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = TLX493D_write(dev, TLX493D_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}

static int TLX493D_init(const struct device *dev) {
    struct TLX493D_data *data = dev->data;
    const struct TLX493D_config *config = dev->config;
    int ret;

    uint8_t fw_id[2];
    ret = TLX493D_seq_read(dev, TLX493D_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    data->in_int = false;
    k_msleep(10);
    ret = TLX493D_write(dev, TLX493D_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = TLX493D_write(dev, TLX493D_SYS_CFG, TLX493D_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = TLX493D_write(dev, TLX493D_Z_IDLE, 0x05); // No Z-Idle packets
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = TLX493D_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = TLX493D_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = TLX493D_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = TLX493D_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = TLX493D_seq_read(dev, TLX493D_SLEEP_INTERVAL, packet, 1);

    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = TLX493D_write(dev, TLX493D_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = TLX493D_FEED_CFG2_EN_IM | TLX493D_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps) {
        feed_cfg2 |= TLX493D_FEED_CFG2_DIS_TAP;
    }

    if (config->no_secondary_tap) {
        feed_cfg2 |= TLX493D_FEED_CFG2_DIS_SEC;
    }

    if (config->rotate_90) {
        feed_cfg2 |= TLX493D_FEED_CFG2_ROTATE_90;
    }
    ret = TLX493D_write(dev, TLX493D_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = TLX493D_FEED_CFG1_EN_FEED;
    if (config->x_invert) {
        feed_cfg1 |= TLX493D_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= TLX493D_FEED_CFG1_INV_Y;
    }
    if (feed_cfg1) {
        ret = TLX493D_write(dev, TLX493D_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    TLX493D_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, TLX493D_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, TLX493D_work_cb);

    TLX493D_write(dev, TLX493D_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

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

#define TLX493D_INST(n)                                                                           \
    static struct TLX493D_data TLX493D_data_##n;                                                 \
    static const struct TLX493D_config TLX493D_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = TLX493D_i2c_seq_read,   \
                     .write = TLX493D_i2c_write),                                                 \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                                     \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, TLX493D_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, TLX493D_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, TLX493D_init, PM_DEVICE_DT_INST_GET(n), &TLX493D_data_##n,          \
                          &TLX493D_config_##n, POST_KERNEL, CONFIG_INPUT_TLX493D_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INST)
