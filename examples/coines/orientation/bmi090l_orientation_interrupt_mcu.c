/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_no_motion_interrupt_mcu.c
 * @brief   Test code to demonstrate on how to configure and use orientation interrupt
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

volatile bool orientation_int_status = 0;

volatile uint32_t interrupt_count = 0;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi090l sensor
 */
static void init_bmi090l(void);

/*!
 * @brief    BMI090L orientation interrupt callback
 */
void orientation_int_callback(void);

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

static void configure_bmi090l_orientation_interrupt(void)
{

    struct bmi090l_orient_cfg orient_cfg = { };
    struct bmi090l_accel_int_channel_cfg orient_int_cfg = { };

    bmi090la_get_orient_config(&orient_cfg, &bmi090ldev);

    orient_cfg.enable = 1;
    orient_cfg.ud_en = 1;
    orient_cfg.blocking = 3;
    orient_cfg.mode = 0;
    orient_cfg.theta = 0x28;
    orient_cfg.hysteresis = 0x80;
    bmi090la_set_orient_config(&orient_cfg, &bmi090ldev);

    /* Map orientation interrupt to INT1 */
    orient_int_cfg.int_channel = BMI090L_INT_CHANNEL_1;
    orient_int_cfg.int_type = BMI090L_ORIENT_INT;
    orient_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    orient_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    orient_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&orient_int_cfg, &bmi090ldev);
}

void orientation_int_callback(void)
{
    orientation_int_status = true;
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
    struct bmi090l_orient_out orient_out = { };

    init_bmi090l_sensor_driver_interface();

    /* Initialize Application Board */
    init_app_board();

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    /*initialize the sensors*/
    init_bmi090l();

    configure_bmi090l_orientation_interrupt();

    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, orientation_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);

    while (1)
    {
        if (orientation_int_status == true)
        {
            orientation_int_status = false;
            bmi090la_get_orient_output(&orient_out, &bmi090ldev);

            switch (orient_out.portrait_landscape)
            {
                case BMI090L_ORIENT_PORTRAIT_UPRIGHT:
                    printf("%lu - Portrait upright\n", interrupt_count);
                    break;
                case BMI090L_ORIENT_LANDSCAPE_LEFT:
                    printf("%lu - Landscape left\n", interrupt_count);
                    break;

                case BMI090L_ORIENT_PORTRAIT_UPSIDE_DOWN:
                    printf("%lu - Portrait upside down\n", interrupt_count);
                    break;
                case BMI090L_ORIENT_LANDSCAPE_RIGHT:
                    printf("%lu - Landscape right\n", interrupt_count);
                    break;

                default:
                    break;
            }

            switch (orient_out.faceup_down)
            {
                case BMI090L_ORIENT_FACE_UP:
                    printf("%lu - Face up\n", interrupt_count);
                    break;
                case BMI090L_ORIENT_FACE_DOWN:
                    printf("%lu - Face down\n", interrupt_count);
                    break;

                default:
                    break;
            }

            printf("\n");
        }
    }

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}
