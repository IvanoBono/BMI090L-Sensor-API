/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_read_sensor_data.c
 * @brief   Test code to read BMI090L accel and gyro sensor data
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
static void init_bmi090l(struct bmi090l_dev *bmi090ldev);

static void configure_accel_gyro_data_ready_interrupts(struct bmi090l_dev *bmi090ldev);

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
static void init_bmi090l(struct bmi090l_dev *bmi090ldev)
{
    int8_t rslt = BMI090L_OK;

    /* Initialize bmi090l sensors (accel & gyro) */
    if (bmi090la_init(bmi090ldev) == BMI090L_OK && bmi090lg_init(bmi090ldev) == BMI090L_OK)
    {
        printf("BMI090L initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi090ldev->accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi090ldev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi090la_soft_reset(bmi090ldev);
    }
    else
    {
        printf("BMI090L initialization failure!\n");
        exit(COINES_E_FAILURE);
    }

    /*! Max read/write length (maximum supported length is 32).
     * To be set by the user */
    bmi090ldev->read_write_len = 32;

    /* Set accel power mode */
    bmi090ldev->accel_cfg.power = BMI090L_ACCEL_PM_ACTIVE;
    rslt = bmi090la_set_power_mode(bmi090ldev);

    if (rslt == BMI090L_OK)
    {
        bmi090ldev->gyro_cfg.power = BMI090L_GYRO_PM_NORMAL;
        bmi090lg_set_power_mode(bmi090ldev);
    }

    printf("Uploading config file !\n");

    /* API uploads the bmi090l config file onto the device */
    rslt = bmi090la_apply_config_file(bmi090ldev);

    if (rslt == BMI090L_OK)
    {
        printf("Upload done !\n");
    }
    else
    {
        printf("Upload failure !\n");
        exit(-1);
    }

    bmi090ldev->accel_cfg.odr = BMI090L_ACCEL_ODR_100_HZ;
    bmi090ldev->accel_cfg.range = BMI090L_ACCEL_RANGE_3G;

    bmi090ldev->accel_cfg.power = BMI090L_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
    bmi090ldev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

    bmi090la_set_power_mode(bmi090ldev);
    coines_delay_msec(10);
    bmi090la_set_meas_conf(bmi090ldev);
    coines_delay_msec(10);

    bmi090ldev->gyro_cfg.odr = BMI090L_GYRO_BW_32_ODR_100_HZ;
    bmi090ldev->gyro_cfg.range = BMI090L_GYRO_RANGE_2000_DPS;
    bmi090ldev->gyro_cfg.bw = BMI090L_GYRO_BW_32_ODR_100_HZ;
    bmi090ldev->gyro_cfg.power = BMI090L_GYRO_PM_NORMAL;

    bmi090lg_set_power_mode(bmi090ldev);
    coines_delay_msec(10);
    bmi090lg_set_meas_conf(bmi090ldev);
    coines_delay_msec(10);

}

static void configure_accel_gyro_data_ready_interrupts(struct bmi090l_dev *bmi090ldev)
{
    int8_t rslt;
    struct bmi090l_accel_int_channel_cfg accel_int_config;
    struct bmi090l_gyro_int_channel_cfg gyro_int_config;

    /* Configure the Interrupt configurations for accel */
    accel_int_config.int_channel = BMI090L_INT_CHANNEL_1;
    accel_int_config.int_type = BMI090L_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi090la_set_int_config(&accel_int_config, bmi090ldev);

    if (rslt == BMI090L_OK)
    {
        /* Configure the Interrupt configurations for gyro */
        gyro_int_config.int_channel = BMI090L_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI090L_GYRO_DATA_RDY_INT;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
        gyro_int_config.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;

        rslt = bmi090lg_set_int_config(&gyro_int_config, bmi090ldev);
    }

    if (rslt != BMI090L_OK)
    {
        printf("Failure in interrupt configurations \n");
        exit(COINES_E_FAILURE);
    }
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
    struct bmi090l_dev bmi090l;
    uint8_t status = 0;
    int8_t rslt;
    uint8_t times_to_read = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMI090L_I2C_INTF
     *         For SPI : BMI090L_SPI_INTF
     */
    rslt = bmi090l_interface_init(&bmi090l, BMI090L_SPI_INTF);
    bmi090l_check_rslt("bmi090l_interface_init", rslt);

    /* Initialize the sensors */
    init_bmi090l(&bmi090l);

    configure_accel_gyro_data_ready_interrupts(&bmi090l);

    while (1)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi090la_get_data_int_status(&status, &bmi090l);

        if ((status & BMI090L_ACCEL_DATA_READY_INT) && (times_to_read < 25))
        {
            bmi090la_get_data(&bmi090l_accel, &bmi090l);

            printf("Accel  x: %+.3f\t y: %+.3f\t z: %+.3f\n",
                   (float)bmi090l_accel.x / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G),
                   (float)bmi090l_accel.y / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G),
                   (float)bmi090l_accel.z / 32768. * 3 * pow(2, BMI090L_ACCEL_RANGE_3G));

            times_to_read++;
        }

        /* Read gyro data ready interrupt status */
        rslt = bmi090lg_get_data_int_status(&status, &bmi090l);

        if ((status & BMI090L_GYRO_DATA_READY_INT) && (times_to_read >= 25))
        {
            bmi090lg_get_data(&bmi090l_gyro, &bmi090l);

            printf("Gyro  x: %+07.1f  y: %+07.1f  z: %+07.1f\n",
                   (float)bmi090l_gyro.x / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS),
                   (float)bmi090l_gyro.y / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS),
                   (float)bmi090l_gyro.z / 32768. * 2000 / pow(2, BMI090L_GYRO_RANGE_2000_DPS));

            times_to_read++;
        }

        fflush(stdout);
        coines_delay_msec(10); /* Since accel. and gyro. ODR is set to 100 Hz */

        if (times_to_read == 50)
        {
            break;
        }
    }

    bmi090l_coines_deinit();

    return rslt;
}
