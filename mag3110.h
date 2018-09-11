#ifndef __MAG3110_H_
#define __MAG3110_H_

//by CAD1 CAD0 pin setting
//I2C address
#define MAG3110_I2C_ADD            0x0E //default

#define MAG3110_STATUS 0x00
#define MAG3110_OUT_X 0x01 /* MSB first */
#define MAG3110_OUT_Y 0x03
#define MAG3110_OUT_Z 0x05
#define MAG3110_WHO_AM_I 0x07
#define MAG3110_SYSMOD 0x08
#define MAG3110_OFF_X 0x09 /* MSB first */
#define MAG3110_OFF_Y 0x0b
#define MAG3110_OFF_Z 0x0d
#define MAG3110_DIE_TEMP 0x0f
#define MAG3110_CTRL_REG1 0x10
#define MAG3110_CTRL_REG2 0x11

#define MAG3110_STATUS_DRDY (BIT(2) | BIT(1) | BIT(0))

#define MAG3110_CTRL_DR_MASK (BIT(7) | BIT(6) | BIT(5))
#define MAG3110_CTRL_DR_SHIFT 5
#define MAG3110_CTRL_DR_DEFAULT 0

#define MAG3110_SYSMOD_MODE_MASK GENMASK(1, 0)

#define MAG3110_CTRL_TM BIT(1) /* trigger single measurement */
#define MAG3110_CTRL_AC BIT(0) /* continuous measurements */

#define MAG3110_CTRL_AUTO_MRST_EN BIT(7) /* magnetic auto-reset */
#define MAG3110_CTRL_RAW BIT(5) /* measurements not user-offset corrected */

#define MAG3110_DEVICE_ID 0xc4

#endif
