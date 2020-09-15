/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bmi090l.h"

int8_t bmi090l_interface_init(struct bmi090l_dev *bmi090l, uint8_t intf);

int8_t bmi090l_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

int8_t bmi090l_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);

int8_t bmi090l_spi_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

int8_t bmi090l_spi_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

void bmi090l_delay_us(uint32_t period_us);

void bmi090l_check_rslt(const char api_name[], int8_t rslt);

void bmi090l_coines_deinit(void);

#ifdef __cplusplus
}
#endif /*__cplusplus */
