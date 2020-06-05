/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_high_g_interrupt.c
 * @brief   Test code to demonstrate on how to configure and use high-g
 *
 */

#include <stdio.h>
#include "coines.h"
#include <stdlib.h>
#include "bmi090l.h"
#include "common.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

volatile bool high_g_int_status = 0;

volatile uint32_t interrupt_count = 0;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi090l sensor
 */
static void init_bmi090l(void);

/*!
 * @brief    BMI090L high-g interrupt callback
 */
void high_g_int_callback(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi090l sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi090l(void)
{
    int8_t rslt = BMI090L_OK;

    /* Initialize bmi090l sensors (accel & gyro)*/
    if (bmi090la_init(&bmi090ldev) == BMI090L_OK && bmi090lg_init(&bmi090ldev) == BMI090L_OK)
    {
        printf("BMI090L initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi090ldev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi090ldev.gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi090la_soft_reset(&bmi090ldev);

        /* Wait for 1 ms - delay taken care inside the function*/
    }
    else
    {
        printf("BMI090L initialization failure!\n");
        exit(COINES_E_FAILURE);
    }

    /*! Max read/write length (maximum supported length is 32).
     * To be set by the user */
    bmi090ldev.read_write_len = 32;

    /*set accel power mode */
    bmi090ldev.accel_cfg.power = BMI090L_ACCEL_PM_ACTIVE;
    rslt = bmi090la_set_power_mode(&bmi090ldev);

    if (rslt == BMI090L_OK)
    {
        bmi090ldev.gyro_cfg.power = BMI090L_GYRO_PM_NORMAL;
        bmi090lg_set_power_mode(&bmi090ldev);
    }

    printf("Uploading config file !\n");
    rslt = bmi090la_apply_config_file(&bmi090ldev);

    /*API uploads the bmi090l config file onto the device*/
    if (rslt == BMI090L_OK)
    {
        printf("Upload done !\n");

        if (rslt == BMI090L_OK)
        {
            bmi090ldev.accel_cfg.range = BMI090L_ACCEL_RANGE_6G;
            bmi090ldev.accel_cfg.odr = BMI090L_ACCEL_ODR_200_HZ;
            bmi090ldev.accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
            bmi090la_set_meas_conf(&bmi090ldev);
        }
    }
}

static void configure_bmi090l_high_g_interrupt(void)
{

    struct bmi090l_high_g_cfg high_g_cfg = {};
    struct bmi090l_accel_int_channel_cfg high_g_int_cfg = { };

    bmi090la_get_high_g_config(&high_g_cfg, &bmi090ldev);

    /* Configure high-g settings */
    high_g_cfg.threshold = 4000; /* ~2g */
    high_g_cfg.hysteresis = 2000; /* ~1g */
    high_g_cfg.duration = 30; /* 150 ms --> 150/5 */
    high_g_cfg.enable = 1;
    high_g_cfg.select_x = 1;
    high_g_cfg.select_y = 1;
    high_g_cfg.select_z = 1;
    bmi090la_set_high_g_config(&high_g_cfg, &bmi090ldev);

    /* Map high-g interrupt to INT1 */
    high_g_int_cfg.int_channel = BMI090L_INT_CHANNEL_1;
    high_g_int_cfg.int_type = BMI090L_HIGH_G_INT;
    high_g_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    high_g_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    high_g_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&high_g_int_cfg, &bmi090ldev);
}

void high_g_int_callback(void)
{
    high_g_int_status = true;
    interrupt_count++;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])
{
    init_bmi090l_sensor_driver_interface();

    /* Initialize Application Board */
    init_app_board();

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    /*initialize the sensors*/
    init_bmi090l();

    configure_bmi090l_high_g_interrupt();

    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, high_g_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);

    while (1)
    {
        if (high_g_int_status == true)
        {
            high_g_int_status = false;
            printf("High-g detected %lu \n", interrupt_count);
        }
    }

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}
