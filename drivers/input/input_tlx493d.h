#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* Logging configuration */
#define TLX493D_LOG_INTERVAL_MS   3000   // ログ出力間隔(ms)

/* Sensor parameters */
#define TLX493D_THRESHOLD        5.0f    // 動作開始閾値
#define TLX493D_HYSTERESIS      2.0f    // ヒステリシス幅
#define TLX493D_SCALE           0.2f    // 磁気値からの変換係数

/* Configuration values */
#define TLX493D_POWER_UP_DELAY_MS  250   // パワーアップ待ち時間(ms)
#define TLX493D_RESET_DELAY_MS     100   // リセット後の待ち時間(ms)

/* TLX493D Register addresses and masks */
#define TLX493D_ADDR_DEFAULT    0x5E    /* Default I2C address */

/* Register map */
#define TLX493D_REG_B_X        0x00    /* X magnetic data */
#define TLX493D_REG_B_Y        0x01    /* Y magnetic data */
#define TLX493D_REG_B_Z        0x02    /* Z magnetic data */
#define TLX493D_REG_TEMP       0x03    /* Temperature data */
#define TLX493D_REG_BX2        0x04    /* X magnetic data LSBs */
#define TLX493D_REG_BZ2        0x05    /* Z magnetic data LSBs */
#define TLX493D_REG_CONFIG     0x10    /* Configuration register */
#define TLX493D_REG_B_MOD1     0x11    /* Mode register 1 */

/* Mode configurations */
#define TLX493D_MODE_DISABLE    0x00    /* Power down mode */
#define TLX493D_MODE_MCM        0x11    /* Master Controlled Mode */
#define TLX493D_MODE_MCM_FAST   0x13    /* Master Controlled Mode + Fast */
#define TLX493D_MODE_LOW_POWER  0x03    /* Low power mode */

/* Configuration bits */
#define TLX493D_CONFIG_T        BIT(7)  /* Temperature measurement enable */
#define TLX493D_CONFIG_LP       BIT(6)  /* Low power mode enable */
#define TLX493D_CONFIG_FAST     BIT(4)  /* Fast mode enable */
#define TLX493D_CONFIG_INT      BIT(3)  /* Interrupt enable */

/* Default configuration */
#define TLX493D_DEFAULT_CONFIG    (TLX493D_CONFIG_FAST)  /* Fast mode only */

/* Function Declarations */
int tlx493d_set_sleep(const struct device *dev, bool sleep);

