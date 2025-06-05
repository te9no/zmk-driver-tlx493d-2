#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* TLX493D Registers */
#define TLX493D_R_ID        0x00     /* Chip ID register */
#define TLX493D_R_FACTSET1  0x01     /* Factory settings 1 */
#define TLX493D_R_FACTSET2  0x03     /* Factory settings 2 */
#define TLX493D_R_CONFIG    0x10     /* Configuration */
#define TLX493D_R_MOD1      0x11     /* Mode1 register */
#define TLX493D_R_MOD2      0x13     /* Mode2 register */
#define TLX493D_R_TMODE     0x14     /* Trigger mode */
#define TLX493D_R_BX        0x02     /* X magnetic data */
#define TLX493D_R_BY        0x04     /* Y magnetic data */
#define TLX493D_R_BZ        0x06     /* Z magnetic data */
#define TLX493D_R_TEMP      0x08     /* Temperature data */

/* Backward compatibility defines */
#define TLX493D_READ_REG    TLX493D_R_BX     /* Start address for burst read */
#define TLX493D_WHOAMI_REG  TLX493D_R_ID     /* ID register alias */
#define TLX493D_MOD1_REG    TLX493D_R_MOD1   /* Mode register alias */

#define TLX493D_CHIP_ID    0x92      /* Expected chip ID */

/* Power modes - corrected as per datasheet */
#define TLX493D_POWER_MODE_MCM     0x15  /* Master Controlled Mode (PRD=01, IICadr=1, MOD=01) */
#define TLX493D_POWER_MODE_LOW     0x13  /* Low Power Mode */

/* I2C Configuration */
#define TLX493D_I2C_ADDR      0x5E    /* 7-bit I2C address */
#define TLX493D_I2C_RETRIES   3       /* Number of I2C retries */
#define TLX493D_I2C_RETRY_DELAY_MS 5  /* Delay between retries */

/* Register access masks */
#define TLX493D_RESET_MASK    0x11    /* Reset bit mask */

/* Device Identification */
#define TLX493D_PRODUCT_ID_MASK  0xF0    /* Product ID mask */
#define TLX493D_PRODUCT_ID       0x90    /* Expected product ID */
#define TLX493D_VERSION_MASK     0x0F    /* Version mask */

/* Device initialization */
#define TLX493D_INIT_RETRY_COUNT    5    /* Number of init retries */
#define TLX493D_INIT_RETRY_DELAY_MS 10   /* Delay between init retries */
#define TLX493D_RESET_DELAY_MS      50   /* Delay after reset */

/* Configuration bits */
#define TLX493D_CONFIG_INT      BIT(3)    /* Enable interrupt */
#define TLX493D_CONFIG_FAST     BIT(4)    /* Fast mode */
#define TLX493D_CONFIG_TEMP_EN  BIT(7)    /* Temperature measurement enable */

/* Mode settings */
#define TLX493D_MODE_POWER_DOWN      0x00  /* Power down mode */
#define TLX493D_MODE_RESET           0x01  /* Reset mode */
#define TLX493D_MODE_MASTER_CONTROLLED    0x11 /* Master Controlled Mode (updated value) */
#define TLX493D_MODE_LOW_POWER           0x03
#define TLX493D_MODE_ULTRA_LOW_POWER     0x04

/* Default values */
#define TLX493D_DEFAULT_CONFIG   0x90  /* P=0, T=1, Fast=1 */
#define TLX493D_CONFIG_FAST_MODE  0x10  /* Fast mode bit */

/* Retry configuration structure */
struct tlx493d_retry_config {
    uint8_t retries;
    uint8_t delay_ms;
};

/* Function Declarations */
int tlx493d_set_sleep(const struct device *dev, bool sleep);

