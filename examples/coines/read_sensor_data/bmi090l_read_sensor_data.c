/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_read_sensor_data.c
 * @brief   Test code to read BMI090L sensor data
 *
 */

#include <stdio.h>
#include "coines.h"
#include <stdlib.h>
#include "bmi090l.h"
#include "math.h"
#include "common.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

struct bmi090l_sensor_data bmi090l_accel;

struct bmi090l_sensor_data bmi090l_gyro;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi090l sensor
 */
static void init_bmi090l(void);

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
    int8_t rslt = 0;

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
        printf("Warning : BMI090L initialization failure!\n");
        /*exit(COINES_E_FAILURE); */
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

    bmi090ldev.accel_cfg.odr = BMI090L_ACCEL_ODR_100_HZ;
    bmi090ldev.accel_cfg.range = BMI090L_ACCEL_RANGE_3G;

    bmi090ldev.accel_cfg.power = BMI090L_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
    bmi090ldev.accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

    bmi090la_set_power_mode(&bmi090ldev);
    coines_delay_msec(10);
    bmi090la_set_meas_conf(&bmi090ldev);
    coines_delay_msec(10);

    bmi090ldev.gyro_cfg.odr = BMI090L_GYRO_BW_32_ODR_100_HZ;
    bmi090ldev.gyro_cfg.range = BMI090L_GYRO_RANGE_2000_DPS;
    bmi090ldev.gyro_cfg.bw = BMI090L_GYRO_BW_32_ODR_100_HZ;
    bmi090ldev.gyro_cfg.power = BMI090L_GYRO_PM_NORMAL;

    bmi090lg_set_power_mode(&bmi090ldev);
    coines_delay_msec(10);
    bmi090lg_set_meas_conf(&bmi090ldev);
    coines_delay_msec(10);

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

    int times_to_read = 0;
    printf("\n  ax\t   ay\t   az\t    gx\t     gy\t     gz\n");
    while (times_to_read < 10)
    {
        bmi090la_get_data(&bmi090l_accel, &bmi090ldev);
        bmi090lg_get_data(&bmi090l_gyro, &bmi090ldev);
        printf("%+.3f\t %+.3f\t %+.3f\t",
               (float)bmi090l_accel.x / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G),
               (float)bmi090l_accel.y / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G),
               (float)bmi090l_accel.z / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G));
        printf("  %+07.1f  %+07.1f  %+07.1f\n",
               (float)bmi090l_gyro.x / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS),
               (float)bmi090l_gyro.y / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS),
               (float)bmi090l_gyro.z / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS));
        fflush(stdout);
        coines_delay_msec(10); // Since accel. and gyro. ODR is set to 100 Hz
        times_to_read++;
    }

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}
