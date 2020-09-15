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

/*! bmi090l shuttle id*/
#define BMI090L_SHUTTLE_ID  0x86

/*!
 * I2C read function map to COINES platform
 */
BMI090L_INTF_RET_TYPE bmi090l_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_i2c(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES platform
 */
BMI090L_INTF_RET_TYPE bmi090l_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_i2c(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * SPI read function map to COINES platform
 */
BMI090L_INTF_RET_TYPE bmi090l_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
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

/*!
 * Delay function map to COINES platform
 */
void bmi090l_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

void bmi090l_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI090L_OK:

            /* Do nothing */
            break;
        case BMI090L_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMI090L_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMI090L_E_INVALID_CONFIG:
            printf("Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMI090L_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMI090L_E_OUT_OF_RANGE:
            printf("Error [%d] : Out of Range\r\n", rslt);
            break;
        case BMI090L_E_INVALID_INPUT:
            printf("Error [%d] : Invalid Input\r\n", rslt);
            break;
        case BMI090L_E_CONFIG_STREAM_ERROR:
            printf("Error [%d] : Config Stream error\r\n", rslt);
            break;
        case BMI090L_E_RD_WR_LENGTH_INVALID:
            printf("Error [%d] : Invalid Read-write length\r\n", rslt);
            break;
        case BMI090L_E_FEATURE_NOT_SUPPORTED:
            printf("Error [%d] : Feature not supported\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

int8_t bmi090l_interface_init(struct bmi090l_dev *bmi090ldev, uint8_t intf)
{
    struct coines_board_info board_info;
    int8_t rslt = BMI090L_OK;

    if (bmi090ldev != NULL)
    {
        int16_t rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);

        if (rslt < 0)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(rslt);
        }

        rslt = coines_get_board_info(&board_info);

        if (rslt == COINES_SUCCESS)
        {
            if (board_info.shuttle_id != BMI090L_SHUTTLE_ID)
            {
                printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
                exit(COINES_E_FAILURE);
            }
        }

        /* Switch VDD for sensor off */
        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_msec(10);

        /* Bus configuration : I2C */
        if (intf == BMI090L_I2C_INTF)
        {
            printf("I2C Interface \n");

            bmi090ldev->write = bmi090l_i2c_write;
            bmi090ldev->read = bmi090l_i2c_read;

            bmi090ldev->accel_id = (unsigned char) BMI090L_ACCEL_I2C_ADDR_PRIMARY;
            bmi090ldev->gyro_id = (unsigned char) BMI090L_GYRO_I2C_ADDR_PRIMARY;
            bmi090ldev->intf = BMI090L_I2C_INTF;

            /* set the sensor interface as I2C with 400kHz speed
             * Use I2C Fast mode (400kHz) for reliable operation with high ODR/sampling time */
            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
            coines_delay_msec(10);

            /* PS pin is made high for selecting I2C protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
        }
        /* Bus configuration : SPI */
        else if (intf == BMI090L_SPI_INTF)
        {
            printf("SPI Interface \n");

            bmi090ldev->write = bmi090l_spi_write;
            bmi090ldev->read = bmi090l_spi_read;

            bmi090ldev->intf = BMI090L_SPI_INTF;
            bmi090ldev->accel_id = COINES_SHUTTLE_PIN_8;
            bmi090ldev->gyro_id = COINES_SHUTTLE_PIN_14;

            /* CS pin is made high for selecting SPI protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

            /* CS pin is made high for selecting SPI protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

            /* PS pin is made low for selecting SPI protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            coines_delay_msec(10);
            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
        }

        bmi090ldev->delay_us = bmi090l_delay_us;
        bmi090ldev->read_write_len = 32;

        coines_delay_msec(10);

        /* Switch VDD for sensor on */
        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        /* after sensor init introduce 200 msec sleep */
        coines_delay_msec(200);
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

void bmi090l_coines_deinit(void)
{
    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
