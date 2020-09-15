/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_interrupt_streaming_mcu.c
 * @brief   Test code to demonstrate streaming of feature interrupts and data ready interrupts
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "bmi090l.h"
#include "coines.h"
#include "common.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

volatile bool acc_feat_int_status = 0;
volatile bool acc_drdy_int_status = 0;
volatile bool gyro_drdy_int_status = 0;

uint8_t interrupt_count = 0;

/* Names of feature interrupts */
char *feat_int_names[] = {
    "data-sync", "any-motion", "high-g", "low-g", "orientation", "no-motion"
};

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
    rslt = bmi090la_apply_config_file(bmi090ldev);

    /* API uploads the bmi090l config file onto the device */
    if (rslt == BMI090L_OK)
    {
        printf("Upload done !\n");

        if (rslt == BMI090L_OK)
        {
            bmi090ldev->accel_cfg.range = BMI090L_ACCEL_RANGE_24G;
            bmi090ldev->accel_cfg.odr = BMI090L_ACCEL_ODR_50_HZ;
            bmi090ldev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
            bmi090la_set_meas_conf(bmi090ldev);

            bmi090ldev->gyro_cfg.odr = BMI090L_GYRO_BW_116_ODR_1000_HZ;
            bmi090ldev->gyro_cfg.range = BMI090L_GYRO_RANGE_250_DPS;
            bmi090ldev->gyro_cfg.bw = BMI090L_GYRO_BW_116_ODR_1000_HZ;
            bmi090ldev->gyro_cfg.power = BMI090L_GYRO_PM_NORMAL;

            bmi090lg_set_meas_conf(bmi090ldev);

        }
    }
}

static void configure_bmi090l_high_g_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_high_g_cfg high_g_cfg = {};
    struct bmi090l_accel_int_channel_cfg high_g_int_cfg = { };

    bmi090la_get_high_g_config(&high_g_cfg, bmi090ldev);

    /* Configure high-g settings */
    high_g_cfg.threshold = 4000; /* 4000*24g/32768 = 2.93g */
    high_g_cfg.hysteresis = 2000; /* 2000*24g/32768 = 1.46g */
    high_g_cfg.duration = 30; /* 150 ms --> 150/5       */
    high_g_cfg.enable = BMI090L_ENABLE;
    high_g_cfg.select_x = BMI090L_ENABLE;
    high_g_cfg.select_y = BMI090L_ENABLE;
    high_g_cfg.select_z = BMI090L_ENABLE;
    bmi090la_set_high_g_config(&high_g_cfg, bmi090ldev);

    /* Map high-g interrupt to accel. INT2 */
    high_g_int_cfg.int_channel = BMI090L_INT_CHANNEL_2;
    high_g_int_cfg.int_type = BMI090L_HIGH_G_INT;
    high_g_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    high_g_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    high_g_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&high_g_int_cfg, bmi090ldev);
}

static void configure_bmi090l_no_motion_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_no_motion_cfg no_motion_cfg = {};
    struct bmi090l_accel_int_channel_cfg no_motion_int_cfg = { };

    bmi090la_get_no_motion_config(&no_motion_cfg, bmi090ldev);

    /* Configure no-motion settings */
    no_motion_cfg.threshold = 0xAA; /* (0.124g * 2^15)/24g = 0xAA */
    no_motion_cfg.duration = 5; /* 100ms/20 = 5 */
    no_motion_cfg.enable = BMI090L_ENABLE;
    no_motion_cfg.select_x = BMI090L_ENABLE;
    no_motion_cfg.select_y = BMI090L_ENABLE;
    no_motion_cfg.select_z = BMI090L_ENABLE;
    bmi090la_set_no_motion_config(&no_motion_cfg, bmi090ldev);

    /* Map high-g interrupt to accel. INT2 */
    no_motion_int_cfg.int_channel = BMI090L_INT_CHANNEL_2;
    no_motion_int_cfg.int_type = BMI090L_NO_MOTION_INT;
    no_motion_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    no_motion_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    no_motion_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&no_motion_int_cfg, bmi090ldev);
}

static void configure_bmi090l_orientation_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_orient_cfg orient_cfg = { };
    struct bmi090l_accel_int_channel_cfg orient_int_cfg = { };

    bmi090la_get_orient_config(&orient_cfg, bmi090ldev);

    orient_cfg.enable = BMI090L_ENABLE;
    orient_cfg.ud_en = BMI090L_ENABLE;
    orient_cfg.blocking = 3;
    orient_cfg.mode = 0;
    orient_cfg.theta = 0x28;
    orient_cfg.hysteresis = 0x80;
    bmi090la_set_orient_config(&orient_cfg, bmi090ldev);

    /* Map orientation interrupt to accel. INT2 */
    orient_int_cfg.int_channel = BMI090L_INT_CHANNEL_2;
    orient_int_cfg.int_type = BMI090L_ORIENT_INT;
    orient_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    orient_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    orient_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&orient_int_cfg, bmi090ldev);
}

static void configure_bmi090l_low_g_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_low_g_cfg low_g_cfg = {};
    struct bmi090l_accel_int_channel_cfg low_g_int_cfg = { };

    bmi090la_get_low_g_config(&low_g_cfg, bmi090ldev);

    /* Configure low-g settings */
    low_g_cfg.threshold = 512; /* 512*24g/32768 = 0.375g */
    low_g_cfg.hysteresis = 256; /* 256*24g/32768 = 0.187g */
    low_g_cfg.duration = 0;
    low_g_cfg.enable = BMI090L_ENABLE;
    bmi090la_set_low_g_config(&low_g_cfg, bmi090ldev);

    /* Map low-g interrupt to accel. INT2 */
    low_g_int_cfg.int_channel = BMI090L_INT_CHANNEL_2;
    low_g_int_cfg.int_type = BMI090L_LOW_G_INT;
    low_g_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    low_g_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    low_g_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&low_g_int_cfg, bmi090ldev);
}

static void configure_bmi090l_any_motion_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_anymotion_cfg any_motion_cfg = {};
    struct bmi090l_accel_int_channel_cfg any_motion_int_cfg = {};

    /* Configure any-motion settings */
    any_motion_cfg.threshold = 0xAA; /* (0.124g * 2^15)/24g = 0xAA */
    any_motion_cfg.duration = 5; /* 100ms/20 = 5 */
    any_motion_cfg.x_en = BMI090L_ENABLE;
    any_motion_cfg.y_en = BMI090L_ENABLE;
    any_motion_cfg.z_en = BMI090L_ENABLE;
    bmi090la_configure_anymotion(any_motion_cfg, bmi090ldev);

    /* Map any-motion interrupt to INT2 */
    any_motion_int_cfg.int_channel = BMI090L_INT_CHANNEL_2;
    any_motion_int_cfg.int_type = BMI090L_ANYMOTION_INT;
    any_motion_int_cfg.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    any_motion_int_cfg.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    any_motion_int_cfg.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    bmi090la_set_int_config(&any_motion_int_cfg, bmi090ldev);
}

static void configure_bmi090l_drdy_interrupt(struct bmi090l_dev *bmi090ldev)
{
    struct bmi090l_accel_int_channel_cfg accel_int_config;
    struct bmi090l_gyro_int_channel_cfg gyro_int_config;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI090L_INT_CHANNEL_1;
    accel_int_config.int_type = BMI090L_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;

    /* Enable accel data ready interrupt channel */
    bmi090la_set_int_config(&accel_int_config, bmi090ldev);

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI090L_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI090L_GYRO_DATA_RDY_INT;
    gyro_int_config.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;

    /* Enable gyro data ready interrupt channel */
    bmi090lg_set_int_config(&gyro_int_config, bmi090ldev);
}

static void acc_feat_int_cb(void)
{
    acc_feat_int_status = true;
}

static void acc_drdy_int_cb(void)
{
    acc_drdy_int_status = true;
}

static void gyro_drdy_int_cb(void)
{
    gyro_drdy_int_status = true;
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
    int8_t rslt;
    uint8_t accel_int_reg = 0;
    struct bmi090l_sensor_data bmi090l_accel, bmi090l_gyro;

    /* Interface reference is given as a parameter
     *         For I2C : BMI090L_I2C_INTF
     *         For SPI : BMI090L_SPI_INTF
     */
    rslt = bmi090l_interface_init(&bmi090l, BMI090L_I2C_INTF);

    /* Initialize the sensors */
    init_bmi090l(&bmi090l);

    configure_bmi090l_drdy_interrupt(&bmi090l);
    configure_bmi090l_high_g_interrupt(&bmi090l);
    configure_bmi090l_no_motion_interrupt(&bmi090l);
    configure_bmi090l_orientation_interrupt(&bmi090l);
    configure_bmi090l_low_g_interrupt(&bmi090l);
    configure_bmi090l_any_motion_interrupt(&bmi090l);

    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, acc_feat_int_cb, COINES_PIN_INTERRUPT_FALLING_EDGE);
    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, acc_drdy_int_cb, COINES_PIN_INTERRUPT_FALLING_EDGE);
    coines_attach_interrupt(COINES_SHUTTLE_PIN_22, gyro_drdy_int_cb, COINES_PIN_INTERRUPT_FALLING_EDGE);

    while (1)
    {
        if (acc_feat_int_status == true)
        {
            acc_feat_int_status = false;
            bmi090la_get_regs(BMI090L_REG_ACCEL_INT_STAT_0, &accel_int_reg, 1, &bmi090l);
            for (int i = 0; i < 6; i++)
            {
                if (accel_int_reg & (1 << i))
                {
                    printf("%s INT t:%lu\n", feat_int_names[i], coines_get_millis());
                }
            }

            interrupt_count++;
        }

        if (acc_drdy_int_status == true)
        {
            acc_drdy_int_status = false;
            bmi090la_get_data(&bmi090l_accel, &bmi090l);
            printf("ax:%d ay:%d az:%d t:%lu\n", bmi090l_accel.x, bmi090l_accel.y, bmi090l_accel.z, coines_get_millis());

            interrupt_count++;
        }

        if (gyro_drdy_int_status == true)
        {
            gyro_drdy_int_status = false;
            bmi090lg_get_data(&bmi090l_gyro, &bmi090l);
            printf("gx:%d gy:%d gz:%d t:%lu\n", bmi090l_gyro.x, bmi090l_gyro.y, bmi090l_gyro.z, coines_get_millis());

            interrupt_count++;
        }

        if (interrupt_count == 50)
        {
            break;
        }
    }

    bmi090l_coines_deinit();

    return rslt;
}
