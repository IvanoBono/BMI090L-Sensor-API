/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    common.h
 * @brief   Common file for BMI090L examples
 *
 */

#include <stdio.h>
#include "coines.h"
#include <stdlib.h>
#include "bmi090l.h"

/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI090L_INTERFACE_I2C   0

/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI090L_INTERFACE_SPI   1

#if (!((BMI090L_INTERFACE_I2C == 1) && (BMI090L_INTERFACE_SPI == 0)) && \
    (!((BMI090L_INTERFACE_I2C == 0) && (BMI090L_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI090L_INTERFACE_I2C / BMI090L_INTERFACE_SPI"
#endif

/*! bmi090l Accel Device address */
#define BMI090L_ACCEL_DEV_ADDR  BMI090L_ACCEL_I2C_ADDR_PRIMARY

/*! bmi090l Gyro Device address */
#define BMI090L_GYRO_DEV_ADDR   BMI090L_GYRO_I2C_ADDR_PRIMARY

/*! bmi090l shuttle id*/
#define BMI090L_SHUTTLE_ID      0x86

/*! @brief This structure containing relevant bmi090l info */
static struct bmi090l_dev bmi090ldev;

#if BMI090L_INTERFACE_I2C == 1

/*!
 * I2C read function map to COINES platform
 */
static BMI090L_INTF_RET_TYPE bmi090l_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_i2c(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES platform
 */
static BMI090L_INTF_RET_TYPE bmi090l_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_i2c(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}
#endif

#if BMI090L_INTERFACE_SPI == 1

/*!
 * SPI read function map to COINES platform
 */
static BMI090L_INTF_RET_TYPE bmi090l_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * SPI write function map to COINES platform
 */
BMI090L_INTF_RET_TYPE bmi090l_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_spi(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}
#endif

/*!
 * Delay function map to COINES platform
 */
void bmi090l_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 *  @brief This internal API is used to initialize the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_sensor_interface(void)
{
    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(10);
#if BMI090L_INTERFACE_I2C == 1

    /* set the sensor interface as I2C with 400kHz speed
     * Use I2C Fast mode (400kHz) for reliable operation with high ODR/sampling time */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
    coines_delay_msec(10);

    /* PS pin is made high for selecting I2C protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
#if BMI090L_INTERFACE_SPI == 1

    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

    /* PS pin is made low for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

    coines_delay_msec(10);
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif
    coines_delay_msec(10);

    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi090l_sensor_driver_interface(void)
{
#if BMI090L_INTERFACE_I2C == 1
    bmi090ldev.write = bmi090l_i2c_write;
    bmi090ldev.read = bmi090l_i2c_read;

    bmi090ldev.accel_id = (unsigned char) BMI090L_ACCEL_DEV_ADDR;
    bmi090ldev.gyro_id = (unsigned char) BMI090L_GYRO_DEV_ADDR;
    bmi090ldev.intf = BMI090L_I2C_INTF;
#endif

#if BMI090L_INTERFACE_SPI == 1
    bmi090ldev.write = bmi090l_spi_write;
    bmi090ldev.read = bmi090l_spi_read;

    bmi090ldev.intf = BMI090L_SPI_INTF;
    bmi090ldev.accel_id = COINES_SHUTTLE_PIN_8;
    bmi090ldev.gyro_id = COINES_SHUTTLE_PIN_14;
#endif

    bmi090ldev.delay_us = bmi090l_delay_us;
}

static void init_app_board(void)
{
    struct coines_board_info board_info;

    int16_t rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);

    if (rslt < 0)
    {
        printf(
            "\n Unable to connect with Application Board ! \n"
            " 1. Check if the board is connected and powered on. \n"
            " 2. Check if Application Board USB driver is installed. \n"
            " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }

    rslt = coines_get_board_info(&board_info);

    if (rslt == COINES_SUCCESS)
    {
        if (board_info.shuttle_id != BMI090L_SHUTTLE_ID)
        {
            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }
}