/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bmi090l_defs.h
 * @date       2020-09-04
 * @version    v1.0.3
 *
 */

#ifndef BMI090L_DEFS_H_
#define BMI090L_DEFS_H_

/*********************************************************************/
/**\ Header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#endif

/*********************************************************************/
/** \name       Common macros                   */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)                 S8_C(x)
#define UINT8_C(x)                U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)                S16_C(x)
#define UINT16_C(x)               U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)                S32_C(x)
#define UINT32_C(x)               U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)                S64_C(x)
#define UINT64_C(x)               U64_C(x)
#endif

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                      0
#else
#define NULL                      ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                      UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE                     UINT8_C(0)
#endif

/** \name Enable BMI09 accel sensor */
#define BMI090L_FEATURE_BMI09     UINT8_C(1)

#ifndef BMI090L_INTF_RET_TYPE
#define BMI090L_INTF_RET_TYPE     int8_t
#endif

#ifndef BMI090L_INTF_RET_SUCCESS
#define BMI090L_INTF_RET_SUCCESS  0
#endif

/*************************** BMI09 Accelerometer Macros *****************************/

/** Register map */
/* Accel registers */

/**\name    Accel Chip Id register */
#define BMI090L_REG_ACCEL_CHIP_ID                    UINT8_C(0x00)

/**\name    Accel Error condition register */
#define BMI090L_REG_ACCEL_ERR                        UINT8_C(0x02)

/**\name    Accel Status flag register */
#define BMI090L_REG_ACCEL_STATUS                     UINT8_C(0x03)

/**\name    Accel X LSB data register */
#define BMI090L_REG_ACCEL_X_LSB                      UINT8_C(0x12)

/**\name    Accel X MSB data register */
#define BMI090L_REG_ACCEL_X_MSB                      UINT8_C(0x13)

/**\name    Accel Y LSB data register */
#define BMI090L_REG_ACCEL_Y_LSB                      UINT8_C(0x14)

/**\name    Accel Y MSB data register */
#define BMI090L_REG_ACCEL_Y_MSB                      UINT8_C(0x15)

/**\name    Accel Z LSB data register */
#define BMI090L_REG_ACCEL_Z_LSB                      UINT8_C(0x16)

/**\name    Accel Z MSB data register */
#define BMI090L_REG_ACCEL_Z_MSB                      UINT8_C(0x17)

/**\name    Sensor time byte 0 register */
#define BMI090L_REG_ACCEL_SENSORTIME_0               UINT8_C(0x18)

/**\name    Sensor time byte 1 register */
#define BMI090L_REG_ACCEL_SENSORTIME_1               UINT8_C(0x19)

/**\name    Sensor time byte 2 register */
#define BMI090L_REG_ACCEL_SENSORTIME_2               UINT8_C(0x1A)

/**\name    Accel Interrupt status0 register */
#define BMI090L_REG_ACCEL_INT_STAT_0                 UINT8_C(0x1C)

/**\name    Accel Interrupt status1 register */
#define BMI090L_REG_ACCEL_INT_STAT_1                 UINT8_C(0x1D)

/**\name    Accel general purpose register 0*/
#define BMI090L_REG_ACCEL_GP_0                       UINT8_C(0x1E)

/**\name    Sensor temperature MSB data register */
#define BMI090L_REG_TEMP_MSB                         UINT8_C(0x22)

/**\name    Sensor temperature LSB data register */
#define BMI090L_REG_TEMP_LSB                         UINT8_C(0x23)

/**\name    Accel general purpose register 4*/
#define BMI090L_REG_ACCEL_GP_4                       UINT8_C(0x27)

/**\name    Orientation result register*/
#define BMI090L_REG_ORIENT_RES                       UINT8_C(0x29)

/**\name    Accel Internal status register */
#define BMI090L_REG_ACCEL_INTERNAL_STAT              UINT8_C(0x2A)

/**\name    Accel configuration register */
#define BMI090L_REG_ACCEL_CONF                       UINT8_C(0x40)

/**\name    Accel range setting register */
#define BMI090L_REG_ACCEL_RANGE                      UINT8_C(0x41)

/**\name    Accel Interrupt pin 1 configuration register */
#define BMI090L_REG_ACCEL_INT1_IO_CONF               UINT8_C(0x53)

/**\name    Accel Interrupt pin 2 configuration register */
#define BMI090L_REG_ACCEL_INT2_IO_CONF               UINT8_C(0x54)

/**\name    Accel Interrupt latch configuration register */
#define BMI090L_REG_ACCEL_INT_LATCH_CONF             UINT8_C(0x55)

/**\name    Accel Interrupt pin1 mapping register */
#define BMI090L_REG_ACCEL_INT1_MAP                   UINT8_C(0x56)

/**\name    Accel Interrupt pin2 mapping register */
#define BMI090L_REG_ACCEL_INT2_MAP                   UINT8_C(0x57)

/**\name    Accel Interrupt map register */
#define BMI090L_REG_ACCEL_INT1_INT2_MAP_DATA         UINT8_C(0x58)

/**\name    Accel Init control register */
#define BMI090L_REG_ACCEL_INIT_CTRL                  UINT8_C(0x59)

/**\name    Accel self-test register */
#define BMI090L_REG_ACCEL_SELF_TEST                  UINT8_C(0x6D)

/**\name    Accel Power mode configuration register */
#define BMI090L_REG_ACCEL_PWR_CONF                   UINT8_C(0x7C)

/**\name    Accel Power control (switch on or off ) register */
#define BMI090L_REG_ACCEL_PWR_CTRL                   UINT8_C(0x7D)

/**\name    Accel soft-reset register */
#define BMI090L_REG_ACCEL_SOFTRESET                  UINT8_C(0x7E)

/**\name    Feature Config related Registers */
#define BMI090L_REG_ACCEL_RESERVED_5B                UINT8_C(0x5B)
#define BMI090L_REG_ACCEL_RESERVED_5C                UINT8_C(0x5C)
#define BMI090L_REG_ACCEL_FEATURE_CFG                UINT8_C(0x5E)

/**\name    BMI09 Accel unique chip identifier */
#define BMI090L_ACCEL_CHIP_ID                        UINT8_C(0x1A)

/**\name    Accel I2C slave address */
#define BMI090L_ACCEL_I2C_ADDR_PRIMARY               UINT8_C(0x18)
#define BMI090L_ACCEL_I2C_ADDR_SECONDARY             UINT8_C(0x19)

/**\name    Interrupt masks */
#define BMI090L_ACCEL_DATA_READY_INT                 UINT8_C(0x80)
#define BMI090L_ACCEL_FIFO_WM_INT                    UINT8_C(0x02)
#define BMI090L_ACCEL_FIFO_FULL_INT                  UINT8_C(0x01)
#define BMI090L_ACCEL_DATA_SYNC_INT                  UINT8_C(0x01)
#define BMI090L_ACCEL_ANY_MOT_INT                    UINT8_C(0x02)
#define BMI090L_ACCEL_HIGH_G_INT                     UINT8_C(0x04)
#define BMI090L_ACCEL_LOW_G_INT                      UINT8_C(0x08)
#define BMI090L_ACCEL_ORIENT_INT                     UINT8_C(0x16)
#define BMI090L_ACCEL_NO_MOT_INT                     UINT8_C(0x32)

#define BMI090L_GYRO_DATA_READY_INT                  UINT8_C(0x80)

/**\name    Accel Bandwidth */
#define BMI090L_ACCEL_BW_OSR4                        UINT8_C(0x00)
#define BMI090L_ACCEL_BW_OSR2                        UINT8_C(0x01)
#define BMI090L_ACCEL_BW_NORMAL                      UINT8_C(0x02)

/**\name  BMI09 Accel Range */
#define BMI090L_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI090L_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI090L_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI090L_ACCEL_RANGE_24G                      UINT8_C(0x03)

/**\name    Accel Output data rate */
#define BMI090L_ACCEL_ODR_12_5_HZ                    UINT8_C(0x05)
#define BMI090L_ACCEL_ODR_25_HZ                      UINT8_C(0x06)
#define BMI090L_ACCEL_ODR_50_HZ                      UINT8_C(0x07)
#define BMI090L_ACCEL_ODR_100_HZ                     UINT8_C(0x08)
#define BMI090L_ACCEL_ODR_200_HZ                     UINT8_C(0x09)
#define BMI090L_ACCEL_ODR_400_HZ                     UINT8_C(0x0A)
#define BMI090L_ACCEL_ODR_800_HZ                     UINT8_C(0x0B)
#define BMI090L_ACCEL_ODR_1600_HZ                    UINT8_C(0x0C)

/**\name    Accel self-test */
#define BMI090L_ACCEL_SWITCH_OFF_SELF_TEST           UINT8_C(0x00)
#define BMI090L_ACCEL_POSITIVE_SELF_TEST             UINT8_C(0x0D)
#define BMI090L_ACCEL_NEGATIVE_SELF_TEST             UINT8_C(0x09)

/**\name    Accel Power mode */
#define BMI090L_ACCEL_PM_ACTIVE                      UINT8_C(0x00)
#define BMI090L_ACCEL_PM_SUSPEND                     UINT8_C(0x03)

/**\name    Accel Power control settings */
#define BMI090L_ACCEL_POWER_DISABLE                  UINT8_C(0x00)
#define BMI090L_ACCEL_POWER_ENABLE                   UINT8_C(0x04)

/**\name    Accel internal interrupt pin mapping */
#define BMI090L_ACCEL_INTA_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTA_ENABLE                    UINT8_C(0x01)
#define BMI090L_ACCEL_INTB_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTB_ENABLE                    UINT8_C(0x02)
#define BMI090L_ACCEL_INTC_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTC_ENABLE                    UINT8_C(0x04)
#define BMI090L_ACCEL_INTD_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTD_ENABLE                    UINT8_C(0x08)
#define BMI090L_ACCEL_INTE_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTE_ENABLE                    UINT8_C(0x10)
#define BMI090L_ACCEL_INTF_DISABLE                   UINT8_C(0x00)
#define BMI090L_ACCEL_INTF_ENABLE                    UINT8_C(0x20)

/**\name    Accel soft-reset delay */
#define BMI090L_ACCEL_SOFTRESET_DELAY_MS             UINT8_C(1)

/**\name    Asic Initialization value */
#define BMI090L_ASIC_INITIALIZED                     UINT8_C(0x01)

/**\name    Mask definitions for ACCEL_ERR_REG register */
#define BMI090L_FATAL_ERR_MASK                       UINT8_C(0x01)
#define BMI090L_ERR_CODE_MASK                        UINT8_C(0x1C)

/**\name    Position definitions for ACCEL_ERR_REG register */
#define BMI090L_CMD_ERR_POS                          UINT8_C(1)
#define BMI090L_ERR_CODE_POS                         UINT8_C(2)

/**\name    Mask definition for ACCEL_STATUS_REG register */
#define BMI090L_ACCEL_STATUS_MASK                    UINT8_C(0x80)

/**\name    Position definitions for ACCEL_STATUS_REG  */
#define BMI090L_ACCEL_STATUS_POS                     UINT8_C(7)

/**\name    Mask definitions for odr, bandwidth and range */
#define BMI090L_ACCEL_ODR_MASK                       UINT8_C(0x0F)
#define BMI090L_ACCEL_BW_MASK                        UINT8_C(0x70)
#define BMI090L_ACCEL_RANGE_MASK                     UINT8_C(0x03)

/**\name    Position definitions for odr, bandwidth and range */
#define BMI090L_ACCEL_BW_POS                         UINT8_C(4)

/**\name    Mask definitions for INT1_IO_CONF register */
#define BMI090L_ACCEL_INT_EDGE_MASK                  UINT8_C(0x01)
#define BMI090L_ACCEL_INT_LVL_MASK                   UINT8_C(0x02)
#define BMI090L_ACCEL_INT_OD_MASK                    UINT8_C(0x04)
#define BMI090L_ACCEL_INT_IO_MASK                    UINT8_C(0x08)
#define BMI090L_ACCEL_INT_IN_MASK                    UINT8_C(0x10)

/**\name    Position definitions for INT1_IO_CONF register */
#define BMI090L_ACCEL_INT_EDGE_POS                   UINT8_C(0)
#define BMI090L_ACCEL_INT_LVL_POS                    UINT8_C(1)
#define BMI090L_ACCEL_INT_OD_POS                     UINT8_C(2)
#define BMI090L_ACCEL_INT_IO_POS                     UINT8_C(3)
#define BMI090L_ACCEL_INT_IN_POS                     UINT8_C(4)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI090L_ACCEL_MAP_INTA_MASK                  UINT8_C(0x01)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI090L_ACCEL_MAP_INTA_POS                   UINT8_C(0x00)

/**\name    Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI090L_ACCEL_INT1_DRDY_MASK                 UINT8_C(0x04)
#define BMI090L_ACCEL_INT2_DRDY_MASK                 UINT8_C(0x40)

/**\name    Position definitions for INT1_INT2_MAP_DATA register */
#define BMI090L_ACCEL_INT1_DRDY_POS                  UINT8_C(2)
#define BMI090L_ACCEL_INT2_DRDY_POS                  UINT8_C(6)

/*************************** BMI09 Gyroscope Macros *****************************/
/** Register map */
/* Gyro registers */

/**\name    Gyro Chip Id register */
#define BMI090L_REG_GYRO_CHIP_ID                     UINT8_C(0x00)

/**\name    Gyro X LSB data register */
#define BMI090L_REG_GYRO_X_LSB                       UINT8_C(0x02)

/**\name    Gyro X MSB data register */
#define BMI090L_REG_GYRO_X_MSB                       UINT8_C(0x03)

/**\name    Gyro Y LSB data register */
#define BMI090L_REG_GYRO_Y_LSB                       UINT8_C(0x04)

/**\name    Gyro Y MSB data register */
#define BMI090L_REG_GYRO_Y_MSB                       UINT8_C(0x05)

/**\name    Gyro Z LSB data register */
#define BMI090L_REG_GYRO_Z_LSB                       UINT8_C(0x06)

/**\name    Gyro Z MSB data register */
#define BMI090L_REG_GYRO_Z_MSB                       UINT8_C(0x07)

/**\name    Gyro Interrupt status register */
#define BMI090L_REG_GYRO_INT_STAT_1                  UINT8_C(0x0A)

/**\name    Gyro Range register */
#define BMI090L_REG_GYRO_RANGE                       UINT8_C(0x0F)

/**\name    Gyro Bandwidth register */
#define BMI090L_REG_GYRO_BANDWIDTH                   UINT8_C(0x10)

/**\name    Gyro Power register */
#define BMI090L_REG_GYRO_LPM1                        UINT8_C(0x11)

/**\name    Gyro soft-reset register */
#define BMI090L_REG_GYRO_SOFTRESET                   UINT8_C(0x14)

/**\name    Gyro Interrupt control register */
#define BMI090L_REG_GYRO_INT_CTRL                    UINT8_C(0x15)

/**\name    Gyro Interrupt Pin configuration register */
#define BMI090L_REG_GYRO_INT3_INT4_IO_CONF           UINT8_C(0x16)

/**\name    Gyro Interrupt Map register */
#define BMI090L_REG_GYRO_INT3_INT4_IO_MAP            UINT8_C(0x18)

/**\name    Gyro self-test register */
#define BMI090L_REG_GYRO_SELF_TEST                   UINT8_C(0x3C)

/**\name    Gyro unique chip identifier */
#define BMI090L_GYRO_CHIP_ID                         UINT8_C(0x0F)

/**\name    Gyro I2C slave address */
#define BMI090L_GYRO_I2C_ADDR_PRIMARY                UINT8_C(0x68)
#define BMI090L_GYRO_I2C_ADDR_SECONDARY              UINT8_C(0x69)

/**\name    Gyro Range */
#define BMI090L_GYRO_RANGE_2000_DPS                  UINT8_C(0x00)
#define BMI090L_GYRO_RANGE_1000_DPS                  UINT8_C(0x01)
#define BMI090L_GYRO_RANGE_500_DPS                   UINT8_C(0x02)
#define BMI090L_GYRO_RANGE_250_DPS                   UINT8_C(0x03)
#define BMI090L_GYRO_RANGE_125_DPS                   UINT8_C(0x04)

/**\name    Gyro Output data rate and bandwidth */
#define BMI090L_GYRO_BW_532_ODR_2000_HZ              UINT8_C(0x00)
#define BMI090L_GYRO_BW_230_ODR_2000_HZ              UINT8_C(0x01)
#define BMI090L_GYRO_BW_116_ODR_1000_HZ              UINT8_C(0x02)
#define BMI090L_GYRO_BW_47_ODR_400_HZ                UINT8_C(0x03)
#define BMI090L_GYRO_BW_23_ODR_200_HZ                UINT8_C(0x04)
#define BMI090L_GYRO_BW_12_ODR_100_HZ                UINT8_C(0x05)
#define BMI090L_GYRO_BW_64_ODR_200_HZ                UINT8_C(0x06)
#define BMI090L_GYRO_BW_32_ODR_100_HZ                UINT8_C(0x07)
#define BMI090L_GYRO_ODR_RESET_VAL                   UINT8_C(0x80)

/**\name    Gyro Power mode */
#define BMI090L_GYRO_PM_NORMAL                       UINT8_C(0x00)
#define BMI090L_GYRO_PM_DEEP_SUSPEND                 UINT8_C(0x20)
#define BMI090L_GYRO_PM_SUSPEND                      UINT8_C(0x80)

/**\name    Gyro data ready interrupt enable value */
#define BMI090L_GYRO_DRDY_INT_DISABLE_VAL            UINT8_C(0x00)
#define BMI090L_GYRO_DRDY_INT_ENABLE_VAL             UINT8_C(0x80)

/**\name    Gyro data ready map values */
#define BMI090L_GYRO_MAP_DRDY_TO_INT3                UINT8_C(0x01)
#define BMI090L_GYRO_MAP_DRDY_TO_INT4                UINT8_C(0x80)
#define BMI090L_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4      UINT8_C(0x81)

/**\name    Gyro soft-reset delay */
#define BMI090L_GYRO_SOFTRESET_DELAY                 UINT8_C(30)

/**\name    Gyro power mode config delay */
#define BMI090L_GYRO_POWER_MODE_CONFIG_DELAY         UINT8_C(30)

/** Mask definitions for range, bandwidth and power */
#define BMI090L_GYRO_RANGE_MASK                      UINT8_C(0x07)
#define BMI090L_GYRO_BW_MASK                         UINT8_C(0x0F)
#define BMI090L_GYRO_POWER_MASK                      UINT8_C(0xA0)

/** Position definitions for range, bandwidth and power */
#define BMI090L_GYRO_POWER_POS                       UINT8_C(5)

/**\name    Mask definitions for BMI090L_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI090L_GYRO_INT3_LVL_MASK                   UINT8_C(0x01)
#define BMI090L_GYRO_INT3_OD_MASK                    UINT8_C(0x02)
#define BMI090L_GYRO_INT4_LVL_MASK                   UINT8_C(0x04)
#define BMI090L_GYRO_INT4_OD_MASK                    UINT8_C(0x08)

/**\name    Position definitions for BMI090L_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI090L_GYRO_INT3_LVL_POS                    UINT8_C(0)
#define BMI090L_GYRO_INT3_OD_POS                     UINT8_C(1)
#define BMI090L_GYRO_INT4_LVL_POS                    UINT8_C(2)
#define BMI090L_GYRO_INT4_OD_POS                     UINT8_C(3)

/**\name    Mask definitions for BMI090L_GYRO_INT_EN_REG register */
#define BMI090L_GYRO_INT_EN_MASK                     UINT8_C(0x80)

/**\name    Position definitions for BMI090L_GYRO_INT_EN_REG register */
#define BMI090L_GYRO_INT_EN_POS                      UINT8_C(7)

/**\name    Mask definitions for BMI09_GYRO_INT_MAP_REG register */
#define BMI090L_GYRO_INT3_MAP_MASK                   UINT8_C(0x01)
#define BMI090L_GYRO_INT4_MAP_MASK                   UINT8_C(0x80)

/**\name    Position definitions for BMI09_GYRO_INT_MAP_REG register */
#define BMI090L_GYRO_INT3_MAP_POS                    UINT8_C(0)
#define BMI090L_GYRO_INT4_MAP_POS                    UINT8_C(7)

/**\name    Mask definitions for BMI09_GYRO_INT_MAP_REG register */
#define BMI09_GYRO_INT3_MAP_MASK                     UINT8_C(0x01)
#define BMI09_GYRO_INT4_MAP_MASK                     UINT8_C(0x80)

/**\name    Position definitions for BMI09_GYRO_INT_MAP_REG register */
#define BMI09_GYRO_INT3_MAP_POS                      UINT8_C(0)
#define BMI09_GYRO_INT4_MAP_POS                      UINT8_C(7)

/**\name    Mask definitions for GYRO_SELF_TEST register */
#define BMI090L_GYRO_SELF_TEST_EN_MASK               UINT8_C(0x01)
#define BMI090L_GYRO_SELF_TEST_RDY_MASK              UINT8_C(0x02)
#define BMI090L_GYRO_SELF_TEST_RESULT_MASK           UINT8_C(0x04)
#define BMI090L_GYRO_SELF_TEST_FUNCTION_MASK         UINT8_C(0x08)

/**\name    Position definitions for GYRO_SELF_TEST register */
#define BMI090L_GYRO_SELF_TEST_RDY_POS               UINT8_C(1)
#define BMI090L_GYRO_SELF_TEST_RESULT_POS            UINT8_C(2)
#define BMI090L_GYRO_SELF_TEST_FUNCTION_POS          UINT8_C(3)

/**\name    Mask definitions for INT3_INT4_IO_MAP register */
#define BMI090L_GYRO_INT4_DATA_MASK                  UINT8_C(0x80)
#define BMI090L_GYRO_INT4_FIFO_MASK                  UINT8_C(0x20)
#define BMI090L_GYRO_INT3_FIFO_MASK                  UINT8_C(0x04)
#define BMI090L_GYRO_INT3_DATA_MASK                  UINT8_C(0x01)

/**\name    Position definitions for INT3_INT4_IO_MAP register */
#define BMI090L_GYRO_INT4_DATA_POS                   UINT8_C(7)
#define BMI090L_GYRO_INT4_FIFO_POS                   UINT8_C(5)
#define BMI090L_GYRO_INT3_FIFO_POS                   UINT8_C(2)
#define BMI090L_GYRO_INT3_DATA_POS                   UINT8_C(0)

/*************************** Common Macros for both Accel and Gyro *****************************/
/**\name    SPI read/write mask to configure address */
#define BMI090L_SPI_RD_MASK                          UINT8_C(0x80)
#define BMI090L_SPI_WR_MASK                          UINT8_C(0x7F)

/**\name API success code */
#define BMI090L_OK                                   INT8_C(0)

/**\name API error codes */
#define BMI090L_E_NULL_PTR                           INT8_C(-1)
#define BMI090L_E_COM_FAIL                           INT8_C(-2)
#define BMI090L_E_DEV_NOT_FOUND                      INT8_C(-3)
#define BMI090L_E_OUT_OF_RANGE                       INT8_C(-4)
#define BMI090L_E_INVALID_INPUT                      INT8_C(-5)
#define BMI090L_E_CONFIG_STREAM_ERROR                INT8_C(-6)
#define BMI090L_E_RD_WR_LENGTH_INVALID               INT8_C(-7)
#define BMI090L_E_INVALID_CONFIG                     INT8_C(-8)
#define BMI090L_E_FEATURE_NOT_SUPPORTED              INT8_C(-9)
#define BMI090L_E_SELF_TEST                          INT8_C(-10)

/***\name    Soft-reset Value */
#define BMI090L_SOFT_RESET_CMD                       UINT8_C(0xB6)

/**\name    Enable/disable macros */
#define BMI090L_DISABLE                              UINT8_C(0)
#define BMI090L_ENABLE                               UINT8_C(1)

/*! @name To define warnings for FIFO activity */
#define BMI090L_W_FIFO_EMPTY                         INT8_C(1)
#define BMI090L_W_PARTIAL_READ                       INT8_C(2)

/**\name    Constant values macros */
#define BMI090L_SENSOR_DATA_SYNC_TIME_MS             UINT8_C(1)
#define BMI090L_DELAY_BETWEEN_WRITES_MS              UINT8_C(1)
#define BMI090L_SELF_TEST_DELAY_MS                   UINT8_C(3)
#define BMI090L_POWER_CONFIG_DELAY                   UINT8_C(5)
#define BMI090L_SENSOR_SETTLE_TIME_MS                UINT8_C(30)
#define BMI090L_SELF_TEST_DATA_READ_MS               UINT8_C(50)
#define BMI090L_ASIC_INIT_TIME_MS                    UINT8_C(150)

#define BMI090L_CONFIG_STREAM_SIZE                   UINT16_C(6144)

/**\name    Sensor time array parameter definitions */
#define BMI090L_SENSOR_TIME_MSB_BYTE                 UINT8_C(2)
#define BMI090L_SENSOR_TIME_XLSB_BYTE                UINT8_C(1)
#define BMI090L_SENSOR_TIME_LSB_BYTE                 UINT8_C(0)

/**\name   int pin active state */
#define BMI090L_INT_ACTIVE_LOW                       UINT8_C(0)
#define BMI090L_INT_ACTIVE_HIGH                      UINT8_C(1)

/**\name   interrupt pin output definition  */
#define BMI090L_INT_MODE_PUSH_PULL                   UINT8_C(0)
#define BMI090L_INT_MODE_OPEN_DRAIN                  UINT8_C(1)

/**\name    Sensor bit resolution */
#define BMI090L_16_BIT_RESOLUTION                    UINT8_C(16)

/*********************************BMI090L FIFO Macros**********************************/
/** Register map */
/*! @name FIFO Header Mask definitions */
#define BMI090L_FIFO_HEADER_ACC_FRM                  UINT8_C(0x84)
#define BMI090L_FIFO_HEADER_ALL_FRM                  UINT8_C(0x9C)
#define BMI090L_FIFO_HEADER_SENS_TIME_FRM            UINT8_C(0x44)
#define BMI090L_FIFO_HEADER_SKIP_FRM                 UINT8_C(0x40)
#define BMI090L_FIFO_HEADER_INPUT_CFG_FRM            UINT8_C(0x48)
#define BMI090L_FIFO_HEAD_OVER_READ_MSB              UINT8_C(0x80)
#define BMI090L_FIFO_SAMPLE_DROP_FRM                 UINT8_C(0x50)

/* Accel registers */
#define BMI090L_FIFO_LENGTH_0_ADDR                   UINT8_C(0x24)
#define BMI090L_FIFO_LENGTH_1_ADDR                   UINT8_C(0x25)
#define BMI090L_FIFO_DATA_ADDR                       UINT8_C(0x26)
#define BMI090L_FIFO_DOWNS_ADDR                      UINT8_C(0x45)
#define BMI090L_FIFO_WTM_0_ADDR                      UINT8_C(0x46)
#define BMI090L_FIFO_WTM_1_ADDR                      UINT8_C(0x47)
#define BMI090L_FIFO_CONFIG_0_ADDR                   UINT8_C(0x48)
#define BMI090L_FIFO_CONFIG_1_ADDR                   UINT8_C(0x49)

/*! @name FIFO sensor data lengths */
#define BMI090L_FIFO_ACCEL_LENGTH                    UINT8_C(6)
#define BMI090L_FIFO_WTM_LENGTH                      UINT8_C(2)
#define BMI090L_FIFO_LENGTH_MSB_BYTE                 UINT8_C(1)
#define BMI090L_FIFO_DATA_LENGTH                     UINT8_C(2)
#define BMI090L_FIFO_CONFIG_LENGTH                   UINT8_C(2)
#define BMI090L_SENSOR_TIME_LENGTH                   UINT8_C(3)
#define BMI090L_FIFO_SKIP_FRM_LENGTH                 UINT8_C(1)
#define BMI090L_FIFO_INPUT_CFG_LENGTH                UINT8_C(1)

/*! @name FIFO byte counter mask definition */
#define BMI090L_FIFO_BYTE_COUNTER_MSB_MASK           UINT8_C(0x3F)

/*! @name FIFO frame masks */
#define BMI090L_FIFO_LSB_CONFIG_CHECK                UINT8_C(0x00)
#define BMI090L_FIFO_MSB_CONFIG_CHECK                UINT8_C(0x80)
#define BMI090L_FIFO_INTR_MASK                       UINT8_C(0x5C)

/*name FIFO config modes */
#define BMI090L_ACC_STREAM_MODE                      UINT8_C(0x00)
#define BMI090L_ACC_FIFO_MODE                        UINT8_C(0x01)

/*name Mask definitions for FIFO configuration modes */
#define BMI090L_ACC_FIFO_MODE_CONFIG_MASK            UINT8_C(0x01)

/*! @name Mask definitions for FIFO_CONFIG_1 register */
#define BMI090L_ACCEL_EN_MASK                        UINT8_C(0x40)
#define BMI090L_ACCEL_INT1_EN_MASK                   UINT8_C(0x08)
#define BMI090L_ACCEL_INT2_EN_MASK                   UINT8_C(0x04)

/*name Position definitions for FIFO_CONFIG_1 register */
#define BMI090L_ACCEL_EN_POS                         UINT8_C(6)
#define BMI090L_ACCEL_INT1_EN_POS                    UINT8_C(3)
#define BMI090L_ACCEL_INT2_EN_POS                    UINT8_C(2)

/*! @name Position definitions for FIFO_DOWNS register */
#define BMI090L_ACC_FIFO_DOWNS_MASK                  UINT8_C(0xF0)

/*! @name FIFO down sampling bit positions */
#define BMI090L_ACC_FIFO_DOWNS_POS                   UINT8_C(0x04)

/*! @name FIFO down sampling user macros */
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_0               UINT8_C(0)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_1               UINT8_C(1)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_2               UINT8_C(2)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_3               UINT8_C(3)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_4               UINT8_C(4)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_5               UINT8_C(5)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_6               UINT8_C(6)
#define BMI090L_ACC_FIFO_DOWN_SAMPLE_7               UINT8_C(7)

/*! @name Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI090L_ACCEL_INT2_FWM_MASK                  UINT8_C(0x20)
#define BMI090L_ACCEL_INT2_FFULL_MASK                UINT8_C(0x10)
#define BMI090L_ACCEL_INT1_FWM_MASK                  UINT8_C(0x02)
#define BMI090L_ACCEL_INT1_FFULL_MASK                UINT8_C(0x01)

/*! @name Positions definitions for INT1_INT2_MAP_DATA register */
#define BMI090L_ACCEL_INT1_FWM_POS                   UINT8_C(1)
#define BMI090L_ACCEL_INT2_FFULL_POS                 UINT8_C(4)
#define BMI090L_ACCEL_INT2_FWM_POS                   UINT8_C(5)

#define BMI090L_STATUS_SET                           UINT8_C(1)
#define BMI090L_STATUS_CLEAR                         UINT8_C(0)

/* Power mode switching delay 30ms - refer data sheet table 3 */
#define BMI090L_GYRO_POWER_MODE_SWITCHING_DELAY_MS   UINT8_C(30)

/* Accel device init delay - refer data sheet section 3 */
#define BMI090L_ACCEL_DEVICE_INIT_DELAY_MS           UINT8_C(50)

/* Accel power mode switching delay - data sheet section 4.1.1 */
#define BMI090L_ACCEL_POWER_MODE_SWITCHING_DELAY_MS  UINT8_C(5)

/* Delay required to get temperature data 1.28s - refer data sheet section 5.3.7 */
#define BMI090L_TEMPERATURE_DATA_READ_DELAY_MS       UINT8_C(1300)

/* Self-test: Resulting minimum difference signal in mg for BMI090L */
#define BMI090L_ST_ACC_X_AXIS_SIGNAL_DIFF            UINT16_C(1000)
#define BMI090L_ST_ACC_Y_AXIS_SIGNAL_DIFF            UINT16_C(1000)
#define BMI090L_ST_ACC_Z_AXIS_SIGNAL_DIFF            UINT16_C(500)

/**\name    Absolute value */
#ifndef BMI090L_ABS
#define BMI090L_ABS(a)                               ((a) >= 0 ? (a) : -(a))
#endif

/**\name    Utility Macros  */
#define BMI090L_SET_LOW_BYTE                         UINT16_C(0x00FF)
#define BMI090L_SET_HIGH_BYTE                        UINT16_C(0xFF00)
#define BMI090L_SET_LOW_NIBBLE                       UINT8_C(0x0F)

/**\name Macro to SET and GET BITS of a register */
#define BMI090L_SET_BITS(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     ((val << bitname##_POS) & bitname##_MASK))

#define BMI090L_GET_BITS(reg_var, bitname)           ((reg_var & (bitname##_MASK)) >> \
                                                      (bitname##_POS))

#define BMI090L_SET_BITS_POS_0(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     (val & bitname##_MASK))

#define BMI090L_GET_BITS_POS_0(reg_var, bitname)     (reg_var & (bitname##_MASK))

#define BMI090L_SET_BIT_VAL_0(reg_var, bitname)      (reg_var & ~(bitname##_MASK))

/**\name     Macro definition for difference between 2 values */
#define BMI090L_GET_DIFF(x, y)                       ((x) - (y))

/**\name     Macro definition to get LSB of 16 bit variable */
#define BMI090L_GET_LSB(var)                         (uint8_t)(var & BMI090L_SET_LOW_BYTE)

/**\name     Macro definition to get MSB of 16 bit variable */
#define BMI090L_GET_MSB(var)                         (uint8_t)((var & BMI090L_SET_HIGH_BYTE) >> 8)

/*************************************************************************/

/*!
 * @brief Interface selection enums
 */
enum bmi090l_intf {
    /*! I2C interface */
    BMI090L_I2C_INTF,
    /*! SPI interface */
    BMI090L_SPI_INTF
};

/*************************** Data structures *****************************/

/**\name    Typedef definitions */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 *
 *  @note : dev_addr is used for I2C read/write operations only.
 *          For SPI read/write operations this is dummy variable.
 */
typedef BMI090L_INTF_RET_TYPE (*bmi090l_read_fptr_t)(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
typedef BMI090L_INTF_RET_TYPE (*bmi090l_write_fptr_t)(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                                                      void *intf_ptr);

/**\name    Delay function pointer */
typedef void (*bmi090l_delay_fptr_t)(uint32_t period, void *intf_ptr);

/**\name    Structure Definitions */

/*!
 *  @brief Sensor XYZ data structure
 */
struct bmi090l_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;
};

/*!
 *  @brief Sensor XYZ data structure in float representation
 */
struct bmi090l_sensor_data_f
{
    /*! X-axis sensor data */
    float x;

    /*! Y-axis sensor data */
    float y;

    /*! Z-axis sensor data */
    float z;
};

/*!
 *  @brief Sensor configuration structure
 */
struct bmi090l_cfg
{
    /*! Power mode */
    uint8_t power;

    /*! Range */
    uint8_t range;

    /*! Bandwidth */
    uint8_t bw;

    /*! Output data rate */
    uint8_t odr;
};

/*!
 *  @brief Error Status structure
 */
struct bmi090l_err_reg
{
    /*! Indicates fatal error */
    uint8_t fatal_err;

    /*! Indicates error code */
    uint8_t err_code;
};

#define BMI090L_ACCEL_ANYMOTION_ADR                 UINT8_C(0x00)
#define BMI090L_ACCEL_ANYMOTION_LEN                 UINT8_C(0x02)
#define BMI090L_ACCEL_ANYMOTION_THRESHOLD_MASK      UINT16_C(0x07FF)
#define BMI090L_ACCEL_ANYMOTION_THRESHOLD_SHIFT     UINT8_C(0x00)
#define BMI090L_ACCEL_ANYMOTION_NOMOTION_SEL_MASK   UINT16_C(0x0800)
#define BMI090L_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT  UINT8_C(0x0B)
#define BMI090L_ACCEL_ANYMOTION_DURATION_MASK       UINT16_C(0x1FFF)
#define BMI090L_ACCEL_ANYMOTION_DURATION_SHIFT      UINT8_C(0x00)
#define BMI090L_ACCEL_ANYMOTION_X_EN_MASK           UINT16_C(0x2000)
#define BMI090L_ACCEL_ANYMOTION_X_EN_SHIFT          UINT8_C(0x0D)
#define BMI090L_ACCEL_ANYMOTION_Y_EN_MASK           UINT16_C(0x4000)
#define BMI090L_ACCEL_ANYMOTION_Y_EN_SHIFT          UINT8_C(0x0E)
#define BMI090L_ACCEL_ANYMOTION_Z_EN_MASK           UINT16_C(0x8000)
#define BMI090L_ACCEL_ANYMOTION_Z_EN_SHIFT          UINT8_C(0x0F)

/*!
 *  @brief Anymotion config structure
 */
struct bmi090l_anymotion_cfg
{
    /* 11 bit threshold of anymotion detection (threshold = X mg * 2,048 (5.11 format)) */
    uint16_t threshold;

    /* 1 bit select between any- and nomotion behavior */
    uint16_t nomotion_sel;

    /* 13 bit set the duration for any- and nomotion (time = duration * 20ms (@50Hz)) */
    uint16_t duration;

    /* Enable anymotion detection for x axis */
    uint16_t x_en;

    /* Enable anymotion detection for y axis */
    uint16_t y_en;

    /* Enable anymotion detection for z axis */
    uint16_t z_en;
};

#define BMI090L_ACCEL_DATA_SYNC_ADR          UINT8_C(0x02)
#define BMI090L_ACCEL_DATA_SYNC_LEN          UINT8_C(0x01)
#define BMI090L_ACCEL_DATA_SYNC_MODE_MASK    UINT16_C(0x0003)
#define BMI090L_ACCEL_DATA_SYNC_MODE_SHIFT   UINT16_C(0x0000)

#define BMI090L_ACCEL_DATA_SYNC_MODE_OFF     UINT8_C(0x00)
#define BMI090L_ACCEL_DATA_SYNC_MODE_400HZ   UINT8_C(0x01)
#define BMI090L_ACCEL_DATA_SYNC_MODE_1000HZ  UINT8_C(0x02)
#define BMI090L_ACCEL_DATA_SYNC_MODE_2000HZ  UINT8_C(0x03)

/*!
 *  @brief Data Sync config structure
 */
struct bmi090l_data_sync_cfg
{
    /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    uint8_t mode;
};

/*! @name Mask definitions for BMI08x high-g feature configuration */
#define BMI090L_HIGH_G_START_ADR      UINT8_C(0x03)
#define BMI090L_HIGH_G_THRES_MASK     UINT16_C(0x7FFF)
#define BMI090L_HIGH_G_HYST_MASK      UINT16_C(0x0FFF)
#define BMI090L_HIGH_G_X_SEL_MASK     UINT16_C(0x1000)
#define BMI090L_HIGH_G_Y_SEL_MASK     UINT16_C(0x2000)
#define BMI090L_HIGH_G_Z_SEL_MASK     UINT16_C(0x4000)
#define BMI090L_HIGH_G_ENABLE_MASK    UINT16_C(0x8000)
#define BMI090L_HIGH_G_DUR_MASK       UINT16_C(0x0FFF)
#define BMI090L_HIGH_G_OUT_CONF_MASK  UINT16_C(0xF000)

/*! @name Bit position definitions for BMI08x high-g feature configuration */
#define BMI090L_HIGH_G_THRES_POS      UINT8_C(0x00)
#define BMI090L_HIGH_G_HYST_POS       UINT8_C(0x00)
#define BMI090L_HIGH_G_OUT_CONF_POS   UINT8_C(0x0C)
#define BMI090L_HIGH_G_X_SEL_POS      UINT8_C(0x0C)
#define BMI090L_HIGH_G_Y_SEL_POS      UINT8_C(0x0D)
#define BMI090L_HIGH_G_Z_SEL_POS      UINT8_C(0x0E)
#define BMI090L_HIGH_G_ENABLE_POS     UINT8_C(0x0F)
#define BMI090L_HIGH_G_DUR_POS        UINT8_C(0x00)

/*! @name Structure to define high-g configuration */
struct bmi090l_high_g_cfg
{
    /*!  Acceleration threshold */
    uint16_t threshold;

    /*!  Hysteresis */
    uint16_t hysteresis;

    /*! To select per x-axis */
    uint16_t select_x;

    /*! To select per y-axis */
    uint16_t select_y;

    /*! To select per z-axis */
    uint16_t select_z;

    /*! high-g enable */
    uint16_t enable;

    /*!  Duration interval */
    uint16_t duration;

    /*! Enable bits for enabling output into the register status bits */
    uint16_t out_conf;
};

#define BMI090L_LOW_G_START_ADR    UINT8_C(0x06)
#define BMI090L_LOW_G_THRES_MASK   UINT16_C(0x7FFF)
#define BMI090L_LOW_G_HYST_MASK    UINT16_C(0x0FFF)
#define BMI090L_LOW_G_DUR_MASK     UINT16_C(0x0FFF)
#define BMI090L_LOW_G_ENABLE_MASK  UINT16_C(0x1000)

#define BMI090L_LOW_G_THRES_POS    UINT16_C(0x00)
#define BMI090L_LOW_G_HYST_POS     UINT16_C(0x00)
#define BMI090L_LOW_G_DUR_POS      UINT16_C(0x00)
#define BMI090L_LOW_G_ENABLE_POS   UINT16_C(0x0C)

/*! @name Structure to define low-g configuration */
struct bmi090l_low_g_cfg
{
    /*! Acceleration threshold */
    uint16_t threshold;

    /*! Hysteresis */
    uint16_t hysteresis;

    /*! Duration interval */
    uint16_t duration;

    /*! low-g enable */
    uint16_t enable;

};

#define BMI090L_ORIENT_START_ADR        UINT8_C(0x09)

/*! @name Mask definitions for BMI08x orientation feature configuration */
#define BMI090L_ORIENT_ENABLE_MASK      UINT16_C(0x0001)
#define BMI090L_ORIENT_UP_DOWN_MASK     UINT16_C(0x0002)
#define BMI090L_ORIENT_SYMM_MODE_MASK   UINT16_C(0x000C)
#define BMI090L_ORIENT_BLOCK_MODE_MASK  UINT16_C(0x0030)
#define BMI090L_ORIENT_THETA_MASK       UINT16_C(0x0FC0)
#define BMI090L_ORIENT_HYST_MASK        UINT16_C(0x07FF)

/*! @name Bit position definitions for BMI2 orientation feature configuration */
#define BMI090L_ORIENT_ENABLE_POS       UINT8_C(0x00)
#define BMI090L_ORIENT_UP_DOWN_POS      UINT8_C(0x01)
#define BMI090L_ORIENT_SYMM_MODE_POS    UINT8_C(0x02)
#define BMI090L_ORIENT_BLOCK_MODE_POS   UINT8_C(0x04)
#define BMI090L_ORIENT_THETA_POS        UINT8_C(0x06)
#define BMI090L_ORIENT_HYST_POS         UINT8_C(0x00)

struct bmi090l_orient_cfg
{
    /*!  Upside/down detection */
    uint16_t ud_en;

    /*!  Symmetrical, high or low Symmetrical */
    uint16_t mode;

    /*!  Blocking mode */
    uint16_t blocking;

    /*!  Threshold angle */
    uint16_t theta;

    /*!  Acceleration hysteresis for orientation detection */
    uint16_t hysteresis;

    /*! Orientation feature enable */
    uint16_t enable;
};

#define BMI090L_ORIENT_PORTRAIT_LANDSCAPE_POS   UINT8_C(0x00)
#define BMI090L_ORIENT_FACEUP_DOWN_POS          UINT8_C(0x02)

#define BMI090L_ORIENT_PORTRAIT_LANDSCAPE_MASK  UINT8_C(0x03)
#define BMI090L_ORIENT_FACEUP_DOWN_MASK         UINT8_C(0x04)

#define BMI090L_ORIENT_PORTRAIT_UPRIGHT         UINT8_C(0x00)
#define BMI090L_ORIENT_LANDSCAPE_LEFT           UINT8_C(0x01)
#define BMI090L_ORIENT_PORTRAIT_UPSIDE_DOWN     UINT8_C(0x02)
#define BMI090L_ORIENT_LANDSCAPE_RIGHT          UINT8_C(0x03)

#define BMI090L_ORIENT_FACE_UP                  UINT8_C(0x00)
#define BMI090L_ORIENT_FACE_DOWN                UINT8_C(0x01)

struct bmi090l_orient_out
{
    /*! Orientation portrait landscape */
    uint8_t portrait_landscape;

    /*! Orientation face-up down  */
    uint8_t faceup_down;
};

#define BMI090L_NO_MOTION_START_ADR       UINT8_C(0x0B)

#define BMI090L_NO_MOTION_THRESHOLD_MASK  UINT16_C(0x07FF)
#define BMI090L_NO_MOTION_SEL_MASK        UINT16_C(0x0800)
#define BMI090L_NO_MOTION_DURATION_MASK   UINT16_C(0x1FFF)
#define BMI090L_NO_MOTION_X_EN_MASK       UINT16_C(0x2000)
#define BMI090L_NO_MOTION_Y_EN_MASK       UINT16_C(0x4000)
#define BMI090L_NO_MOTION_Z_EN_MASK       UINT16_C(0x8000)

#define BMI090L_NO_MOTION_THRESHOLD_POS   UINT8_C(0)
#define BMI090L_NO_MOTION_SEL_POS         UINT8_C(11)
#define BMI090L_NO_MOTION_DURATION_POS    UINT8_C(0)
#define BMI090L_NO_MOTION_X_EN_POS        UINT8_C(13)
#define BMI090L_NO_MOTION_Y_EN_POS        UINT8_C(14)
#define BMI090L_NO_MOTION_Z_EN_POS        UINT8_C(15)

struct bmi090l_no_motion_cfg
{
    /*! Duration in 50Hz samples(20msec) */
    uint16_t duration;

    /*! Acceleration slope threshold */
    uint16_t threshold;

    /*! To select per x-axis */
    uint16_t select_x;

    /*! To select per y-axis */
    uint16_t select_y;

    /*! To select per z-axis */
    uint16_t select_z;

    /*! Enable no-motion feature */
    uint16_t enable;

};

/*!
 *  @brief Enum to select accelerometer Interrupt pins
 */
enum bmi090l_accel_int_channel {
    /* Interrupt Channel 1 for accel sensor */
    BMI090L_INT_CHANNEL_1,
    /* Interrupt Channel 2 for accel sensor */
    BMI090L_INT_CHANNEL_2
};

/*!
 *  @brief Enum to select gyroscope Interrupt pins
 */
enum bmi090l_gyro_int_channel {
    /* Interrupt Channel 3 for gyro sensor */
    BMI090L_INT_CHANNEL_3,
    /* Interrupt Channel 4 for gyro sensor */
    BMI090L_INT_CHANNEL_4
};

/*!
 *  @brief Enum to select accelerometer interrupts
 */
enum bmi090l_accel_int_types {
    BMI090L_ACCEL_DATA_RDY_INT,
    /* Accel data ready interrupt */
    BMI090L_ACCEL_SYNC_DATA_RDY_INT,
    /* Accel synchronized data ready interrupt */
    BMI090L_ACCEL_SYNC_INPUT,
    /* Accel FIFO watermark interrupt */
    BMI090L_FIFO_WM_INT,
    /* Accel FIFO full interrupt */
    BMI090L_FIFO_FULL_INT,
    /* Accel anymotion interrupt*/
    BMI090L_ANYMOTION_INT,
    /* Accel high-g interrupt */
    BMI090L_HIGH_G_INT,
    /* Accel low-g interrupt */
    BMI090L_LOW_G_INT,
    /* Accel orient interrupt */
    BMI090L_ORIENT_INT,
    /* Accel no-motion interrupt */
    BMI090L_NO_MOTION_INT
};

/*!
 *  @brief Enum to select gyroscope interrupts
 */
enum bmi090l_gyro_int_types {
    /* Gyro data ready interrupt */
    BMI090L_GYRO_DATA_RDY_INT
};

/*!
 *  @brief Interrupt pin configuration structure
 */
struct bmi090l_int_pin_cfg
{
    /*! Interrupt pin level configuration
     * Assignable macros :
     * - BMI090L_INT_ACTIVE_LOW
     * - BMI090L_INT_ACTIVE_HIGH
     */
    uint8_t lvl : 1;

    /*! Interrupt pin mode configuration
     * Assignable macros :
     * - BMI090L_INT_MODE_PUSH_PULL
     * - BMI090L_INT_MODE_OPEN_DRAIN
     */
    uint8_t output_mode : 1;

    /*! Enable interrupt pin
     * Assignable Macros :
     * - BMI090L_ENABLE
     * - BMI090L_DISABLE
     */
    uint8_t enable_int_pin : 1;
};

/*!
 *  @brief Interrupt channel structure for accel
 */
struct bmi090l_accel_int_channel_cfg
{
    /*! Accel Interrupt channel */
    enum bmi090l_accel_int_channel int_channel;

    /*! Select Accel Interrupt type */
    enum bmi090l_accel_int_types int_type;

    /*! Structure to configure accel interrupt pins */
    struct bmi090l_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt channel structure for gyro
 */
struct bmi090l_gyro_int_channel_cfg
{
    /*! Gyro Interrupt channel */
    enum bmi090l_gyro_int_channel int_channel;

    /*! Select Gyro Interrupt type */
    enum bmi090l_gyro_int_types int_type;

    /*! Structure to configure gyro interrupt pins */
    struct bmi090l_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt Configuration structure
 */
struct bmi090l_int_cfg
{
    /*! Configuration of first accel interrupt channel */
    struct bmi090l_accel_int_channel_cfg accel_int_config_1;

    /*! Configuration of second accel interrupt channel */
    struct bmi090l_accel_int_channel_cfg accel_int_config_2;

    /*! Configuration of first gyro interrupt channel */
    struct bmi090l_gyro_int_channel_cfg gyro_int_config_1;

    /*! Configuration of second gyro interrupt channel */
    struct bmi090l_gyro_int_channel_cfg gyro_int_config_2;
};

/*!
 *  @brief accel fifo configurations
 */
struct bmi090l_accel_fifo_config
{
    /*! Configure the fifo mode (0 = Stream mode, 1 = FIFO mode) */
    uint8_t mode;

    /*! To enable the accel */
    uint8_t accel_en;

    /*! To enable the interrupt_1 */
    uint8_t int1_en;

    /*! To enable the interrupt_2 */
    uint8_t int2_en;
};

/*!
 *  @brief gyro fifo configurations
 */
struct bmi090l_gyro_fifo_config
{
    /*! Configure the fifo mode (0 = Stream mode, 1 = FIFO mode) */
    uint8_t mode;

    /*! To enable the gyro */
    uint8_t gyro_en;

    /*! To enable the interrupt_1 */
    uint8_t int3_en;

    /*! To enable the interrupt_2 */
    uint8_t int4_en;
};

/*! @name Structure to define FIFO frame configuration */
struct bmi090l_fifo_frame
{
    /*! Pointer to FIFO data */
    uint8_t *data;

    /*! Number of user defined bytes of FIFO to be read */
    uint16_t length;

    /*! Enables type of data to be streamed - accelerometer */
    uint16_t data_enable;

    /*! To index accelerometer bytes */
    uint16_t acc_byte_start_idx;

    /*! To index gyro bytes */
    uint16_t gyr_byte_start_idx;

    /*! FIFO sensor time */
    uint32_t sensor_time;

    /*! Skipped frame count */
    uint8_t skipped_frame_count;

    /*! Type of data interrupt to be mapped */
    uint8_t data_int_map;

    /*! Water-mark level for water-mark interrupt */
    uint16_t wm_lvl;

    /*! Accelerometer frame length */
    uint8_t acc_frm_len;

    /*! Gyro frame length */
    uint8_t gyr_frm_len;

    /*! Accelerometer frame length */
    uint8_t all_frm_len;

    /*! FIFO accelerometer configurations */
    struct bmi090l_accel_fifo_config fifo_conf;
};

/*!
 *  @brief
 *  This structure holds all relevant information about BMI09
 */
struct bmi090l_dev
{
    /*! Accel chip Id */
    uint8_t accel_chip_id;

    /*! Gyro chip Id */
    uint8_t gyro_chip_id;

    /*! Accel device Id */
    uint8_t accel_id;

    /*! Gyro device Id */
    uint8_t gyro_id;

    /*! 0 - I2C , 1 - SPI Interface */
    enum bmi090l_intf intf;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Structure to configure accel sensor  */
    struct bmi090l_cfg accel_cfg;

    /*! Structure to configure gyro sensor  */
    struct bmi090l_cfg gyro_cfg;

    /*! Config stream data buffer address will be assigned */
    const uint8_t *config_file_ptr;

    /*! Max read/write length (maximum supported length is 32).
     * To be set by the user */
    uint8_t read_write_len;

    void *intf_ptr;

    BMI090L_INTF_RET_TYPE intf_rslt;

    /*! Read function pointer */
    bmi090l_read_fptr_t read;

    /*! Write function pointer */
    bmi090l_write_fptr_t write;

    /*! Delay function pointer */
    bmi090l_delay_fptr_t delay_us;
};

#endif /* BMI090L_DEFS_H_ */
