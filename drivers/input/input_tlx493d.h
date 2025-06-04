#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* TLX493D Register Map */
#define TLX493D_WHOAMI_REG    0x00
#define TLX493D_CHIP_ID       0x14

#define TLX493D_READ_REG      0x00   /* First register for burst read */
#define TLX493D_BX_REG        0x00   /* X magnetic data */
#define TLX493D_BY_REG        0x01   /* Y magnetic data */
#define TLX493D_BZ_REG        0x02   /* Z magnetic data */
#define TLX493D_TEMP_REG      0x03   /* Temperature data */
#define TLX493D_MOD1_REG      0x11   /* Mode configuration 1 */

/* Power Mode Settings */
#define TLX493D_POWER_MODE_MCM     0x13  /* Master Controlled Mode */
#define TLX493D_POWER_MODE_LOW     0x03  /* Low Power Mode */
#define TLX493D_POWER_MODE_ULTRA   0x0C  /* Ultra-Low Power Mode */

/* Function Declarations */
int tlx493d_set_sleep(const struct device *dev, bool sleep);

