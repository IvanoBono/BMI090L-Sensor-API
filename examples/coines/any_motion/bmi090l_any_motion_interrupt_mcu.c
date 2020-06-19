/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_no_motion_interrupt_mcu.c
 * @brief   Test code to demonstrate on how to configure and use any-motion feature
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

volatile bool any_motion_int_status = 0;

volatile uint32_t interrupt_count = 0;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi090l sensor
 */
static void init_bmi090l(void);

/*!
 * @brief    BMI090L any-motion interrupt callback
 */
void any_motion_int_callback(void);

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
            bmi090ldev.accel_cfg.range = BMI090L_ACCEL_RANGE_24G;
            bmi090ldev.accel_cfg.odr = BMI090L_ACCEL_ODR_50_HZ;
            bmi090ldev.accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
            bmi090la_set_meas_conf(&bmi090ldev);
        }
    }
}

static void configure_bmi090l_any_motion_interrupt(void)
{

    struct bmi090l_anymotion_cfg any_motion_cfg = {};
    struct bmi090l_accel_int_channel_cfg no_motion_int_cfg = {};

    /* Configure any-motion settings */
    any_motion_cfg.threshold = 0xAA; /* (0.124g * 2^15)/24g = 0xAA */
    any_motion_cfg.duration = 5;    /* 100ms/20 = 5 */
    any_motion_cfg.x_en = 1;
    any_motion_cfg.y_en = 1;
    any_motion_cfg.z_en = 1;
    bmi090la_configure_anymotion(any_motion_cfg, &bmi090ldev);

    /* Map any-motion interrupt to INT1 */
    no_motion_int_cfg.int_channel = BMI090L_INT_CHANNEL_1;
    no_motion_int_cfg.int_type = BMI090L_ANYMOTION_INT;
    no_motion_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    no_motion_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    no_motion_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&no_motion_int_cfg, &bmi090ldev);
}

void any_motion_int_callback(void)
{
    any_motion_int_status = true;
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

    configure_bmi090l_any_motion_interrupt();

    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, any_motion_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);

    while (1)
    {
        if (any_motion_int_status == true)
        {
            any_motion_int_status = false;
            printf("Movement detected %lu \n", interrupt_count);
        }
    }

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}
