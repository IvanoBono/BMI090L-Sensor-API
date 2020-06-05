/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi090l_read_synchronized_data_mcu.c
 * @brief   Test code to read synchronized sensor from BMI090L
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bmi090l.h"
#include "common.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/
volatile bool data_sync_int = false;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08x sensor
 */
static void init_bmi090l(void);

/*!
 * @brief    BMI090L data sync. interrupt callback
 */
void bmi090l_data_sync_int(void);

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
    struct bmi090l_data_sync_cfg sync_cfg = { };

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
        bmi090la_set_power_mode(&bmi090ldev);
    }

    if ((bmi090ldev.accel_cfg.power != BMI090L_ACCEL_PM_ACTIVE) ||
        (bmi090ldev.gyro_cfg.power != BMI090L_GYRO_PM_NORMAL))
    {
        printf("Accel/gyro sensor in suspend mode\nUse in active/normal mode !!");
        exit(EXIT_FAILURE);
    }

    printf("Uploading BMI090L data synchronization feature config !\n");

    /*API uploads the bmi08x config file onto the device*/
    if (rslt == BMI090L_OK)
    {
        rslt = bmi090la_apply_config_file(&bmi090ldev);

        /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
        if (rslt == BMI090L_OK)
        {
            bmi090ldev.accel_cfg.range = BMI090L_ACCEL_RANGE_24G;

            /*assign gyro range setting*/
            bmi090ldev.gyro_cfg.range = BMI090L_GYRO_RANGE_2000_DPS;

            /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
            sync_cfg.mode = BMI090L_ACCEL_DATA_SYNC_MODE_2000HZ;
            rslt = bmi090la_configure_data_synchronization(sync_cfg, &bmi090ldev);
        }
    }

    if (rslt == BMI090L_OK)
    {
        printf("BMI090L data synchronization feature configured !\n");
    }
    else
    {
        printf("BMI090L data synchronization feature configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void enable_bmi090l_data_synchronization_interrupt(void)
{
    int8_t rslt = BMI090L_OK;
    struct bmi090l_int_cfg int_config = { };

    /*set accel interrupt pin configuration*/
    /*configure host data ready interrupt */
    #if defined(MCU_APP20)
        int_config.accel_int_config_1.int_channel = BMI090L_INT_CHANNEL_1;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_1.int_channel = BMI090L_INT_CHANNEL_2;
    #endif
    int_config.accel_int_config_1.int_type = BMI090L_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;

    /*configure Accel syncronization input interrupt pin */
    #if defined(MCU_APP20)
        int_config.accel_int_config_2.int_channel = BMI090L_INT_CHANNEL_2;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_2.int_channel = BMI090L_INT_CHANNEL_1;
    #endif
    int_config.accel_int_config_2.int_type = BMI090L_ACCEL_SYNC_DATA_RDY_INT;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;

    /*set gyro interrupt pin configuration*/
    #if defined(MCU_APP20)
        int_config.gyro_int_config_1.int_channel = BMI090L_INT_CHANNEL_3;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_1.int_channel = BMI090L_INT_CHANNEL_4;
    #endif
    int_config.gyro_int_config_1.int_type = BMI090L_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI090L_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;

    #if defined(MCU_APP20)
        int_config.gyro_int_config_2.int_channel = BMI090L_INT_CHANNEL_4;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_2.int_channel = BMI090L_INT_CHANNEL_3;
    #endif
    int_config.gyro_int_config_2.int_type = BMI090L_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI090L_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;

    rslt = bmi090la_set_data_sync_int_config(&int_config, &bmi090ldev);

    if (rslt != BMI090L_OK)
    {
        printf("BMI090L data synchronization enable interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void disable_bmi090l_data_synchronization_interrupt(void)
{
    int8_t rslt;
    struct bmi090l_int_cfg int_config = { };
    struct bmi090l_data_sync_cfg sync_cfg = { };

    sync_cfg.mode = BMI090L_ACCEL_DATA_SYNC_MODE_OFF; /*turn off the sync feature*/

    rslt = bmi090la_configure_data_synchronization(sync_cfg, &bmi090ldev);

    /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
    /* configure synchronization interrupt pins */
    if (rslt == BMI090L_OK)
    {
        /*set accel interrupt pin configuration*/
        /*configure host data ready interrupt */
    #if defined(MCU_APP20)
        int_config.accel_int_config_1.int_channel = BMI090L_INT_CHANNEL_1;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_1.int_channel = BMI090L_INT_CHANNEL_2;
    #endif
        int_config.accel_int_config_1.int_type = BMI090L_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI090L_DISABLE;

        /*configure Accel synchronization input interrupt pin */
    #if defined(MCU_APP20)
        int_config.accel_int_config_2.int_channel = BMI090L_INT_CHANNEL_2;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_2.int_channel = BMI090L_INT_CHANNEL_1;
    #endif
        int_config.accel_int_config_2.int_type = BMI090L_ACCEL_SYNC_DATA_RDY_INT;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI090L_DISABLE;

        /*set gyro interrupt pin configuration*/
    #if defined(MCU_APP20)
        int_config.gyro_int_config_1.int_channel = BMI090L_INT_CHANNEL_3;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_1.int_channel = BMI090L_INT_CHANNEL_4;
    #endif
        int_config.gyro_int_config_1.int_type = BMI090L_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI090L_DISABLE;

    #if defined(MCU_APP20)
        int_config.gyro_int_config_2.int_channel = BMI090L_INT_CHANNEL_4;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_2.int_channel = BMI090L_INT_CHANNEL_3;
    #endif
        int_config.gyro_int_config_2.int_type = BMI090L_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI090L_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI090L_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI090L_INT_MODE_PUSH_PULL;

        rslt = bmi090la_set_data_sync_int_config(&int_config, &bmi090ldev);
    }

    if (rslt != BMI090L_OK)
    {
        printf("BMI090L data synchronization disable interrupt configuration failure!\n");
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
    struct bmi090l_sensor_data bmi090l_accel, bmi090l_gyro;

    init_bmi090l_sensor_driver_interface();

    /* Initialize Application Board */
    init_app_board();

    printf("Ensure that accel. INT pin is shorted with gyro. INT pin !!\n");

    /* here we have 2000ms to allow user to read the hint regarding the pin connection */
    coines_delay_msec(2000);
    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    /*initialize the sensors*/
    init_bmi090l();
    coines_delay_msec(200);

#if defined(MCU_APP20)
    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi090l_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#elif defined(MCU_APP30)
    coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6, bmi090l_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#endif

    /*Enable data ready interrupts*/
    enable_bmi090l_data_synchronization_interrupt();
    uint32_t start_time = coines_get_millis();

    /* Run data synchronization for 10s before disabling interrupts */
    while (coines_get_millis() - start_time < 10000)
    {
        if (data_sync_int == true)
        {
            data_sync_int = false;
            bmi090la_get_synchronized_data(&bmi090l_accel, &bmi090l_gyro, &bmi090ldev);
            printf("ax:%d \t ay:%d \t az:%d \t gx:%d \t gy:%d \t gz:%d \t ts:%lu\n",
                   bmi090l_accel.x,
                   bmi090l_accel.y,
                   bmi090l_accel.z,
                   bmi090l_gyro.x,
                   bmi090l_gyro.y,
                   bmi090l_gyro.z,
                   coines_get_millis() - start_time);

        }
    }

    /*disable data ready interrupts*/
    disable_bmi090l_data_synchronization_interrupt();

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

/* BMI090L data sync. interrupt callback */
void bmi090l_data_sync_int(void)
{
    data_sync_int = true;
}
