/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bmi090la.c
 * @date       2020-08-28
 * @version    v1.0.2
 *
 */

/*! \file bmi090la.c
 * \brief Sensor Driver for BMI09x family of sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "bmi090l.h"

/****************************************************************************/

/** \name       Macros
 ****************************************************************************/

/**\name    Value of LSB_PER_G = (power(2, BMI090L_16_BIT_RESOLUTION) / (2 * range)) */
#define LSB_PER_G  UINT32_C(1365) /* for the 16-bit resolution and 24g range */

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/*!
 * @brief Accel self-test diff xyz data structure
 */
struct bmi090la_selftest_delta_limit
{
    /*! Accel X  data */
    int16_t x;

    /*! Accel Y  data */
    int16_t y;

    /*! Accel Z  data */
    int16_t z;
};

/**\name Feature configuration file */
const uint8_t bmi090l_config_file[] = {

    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x53, 0xb7, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x75,
    0xb7, 0x80, 0x2e, 0xc5, 0xb7, 0x80, 0x2e, 0xde, 0xb7, 0x80, 0x2e, 0x9e, 0xb7, 0x50, 0x39, 0x21, 0x2e, 0xb0, 0xf0,
    0x10, 0x30, 0x21, 0x2e, 0x16, 0xf0, 0x80, 0x2e, 0xcf, 0xb6, 0x02, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0x13,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x9a, 0x01, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xaa,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xe8, 0x03, 0x04, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x30, 0x0a,
    0x80, 0x00, 0xaa, 0x00, 0x05, 0x00, 0x1e, 0x00, 0x89, 0xf0, 0x21, 0x00, 0x80, 0x00, 0x39, 0xf0, 0x03, 0x01, 0x06,
    0x01, 0x0b, 0x01, 0x09, 0x01, 0x5f, 0xf0, 0xc0, 0x00, 0xb1, 0xf0, 0x00, 0x40, 0xaf, 0x00, 0xff, 0x00, 0xff, 0xb7,
    0x00, 0x02, 0x00, 0xb0, 0x05, 0x80, 0xb1, 0xf0, 0x49, 0x00, 0x51, 0x00, 0x81, 0x00, 0x4f, 0x00, 0x62, 0x00, 0x52,
    0xf0, 0x33, 0xf0, 0x5b, 0x00, 0x28, 0xf0, 0x66, 0x00, 0x69, 0x00, 0x69, 0x00, 0x00, 0xe0, 0xff, 0x07, 0x00, 0x20,
    0x00, 0x80, 0xff, 0x1f, 0x71, 0x00, 0x00, 0x10, 0x73, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x78, 0x00, 0x9a, 0x01, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x01, 0x2e,
    0x8b, 0x00, 0x02, 0x40, 0x2f, 0xbd, 0x00, 0x51, 0x2f, 0xb9, 0xf1, 0x7f, 0x80, 0xb2, 0xeb, 0x7f, 0x90, 0x2e, 0x44,
    0xb2, 0x02, 0x40, 0x50, 0x25, 0x14, 0x40, 0x03, 0x40, 0x40, 0x41, 0x4e, 0xbe, 0x47, 0x41, 0x4f, 0xba, 0x8a, 0xbe,
    0xd4, 0x7f, 0x5e, 0xba, 0xf4, 0xbf, 0xfa, 0xba, 0xc4, 0x7f, 0xb5, 0x7f, 0x04, 0x30, 0x50, 0x40, 0x00, 0xa8, 0x60,
    0x05, 0xac, 0xbf, 0x05, 0x22, 0xfe, 0xba, 0xa1, 0x7f, 0xb5, 0xbd, 0x6d, 0x5e, 0x52, 0x40, 0x7d, 0x01, 0x80, 0xa8,
    0xe2, 0x05, 0x97, 0x22, 0x47, 0x40, 0xc0, 0xa9, 0x27, 0x05, 0xfc, 0x23, 0xb5, 0xb9, 0x44, 0x41, 0x43, 0x05, 0x97,
    0x7f, 0x25, 0x2a, 0xce, 0x17, 0x15, 0x30, 0xfd, 0x09, 0xf7, 0x01, 0x7e, 0x82, 0xf1, 0xb7, 0x97, 0x0f, 0x81, 0x7f,
    0x04, 0x2f, 0xf1, 0x6f, 0x00, 0x2e, 0x41, 0x40, 0x40, 0xac, 0x26, 0x2f, 0x97, 0x0f, 0x04, 0x2f, 0xf1, 0x6f, 0x00,
    0x2e, 0x41, 0x40, 0x40, 0xa0, 0x1b, 0x2f, 0x20, 0x2a, 0x4e, 0x16, 0x4d, 0x08, 0x71, 0x00, 0x91, 0xb4, 0x4b, 0x28,
    0x91, 0x0e, 0x04, 0x2f, 0xa0, 0x6f, 0x00, 0x2e, 0x00, 0x40, 0x00, 0xa0, 0x0a, 0x2f, 0x91, 0x0e, 0x12, 0x2f, 0xa1,
    0x6f, 0x00, 0x2e, 0x41, 0x40, 0x40, 0xa0, 0x0d, 0x2f, 0x01, 0x30, 0x23, 0x2e, 0x76, 0x00, 0x0a, 0x2d, 0x21, 0x30,
    0x23, 0x2e, 0x76, 0x00, 0x06, 0x2d, 0x31, 0x30, 0x23, 0x2e, 0x76, 0x00, 0x02, 0x2d, 0x2b, 0x2e, 0x76, 0x00, 0xd0,
    0x6f, 0x00, 0xb2, 0xf2, 0x6f, 0x03, 0x2e, 0x75, 0x00, 0x12, 0x2f, 0x90, 0x6f, 0x71, 0x56, 0x43, 0x0f, 0x03, 0x2f,
    0x12, 0xb8, 0x21, 0x2e, 0x77, 0x00, 0x0b, 0x2d, 0x82, 0x80, 0x00, 0x2e, 0x00, 0x40, 0x00, 0xac, 0x02, 0x2f, 0x2b,
    0x2e, 0x77, 0x00, 0x03, 0x2d, 0x00, 0x30, 0x21, 0x2e, 0x77, 0x00, 0x07, 0x2e, 0x76, 0x00, 0x34, 0x30, 0x01, 0x2e,
    0x77, 0x00, 0xdc, 0x08, 0x02, 0xbc, 0x18, 0x0a, 0x08, 0x1a, 0xd0, 0x7f, 0x33, 0x2f, 0xc1, 0x6f, 0x40, 0xa4, 0x30,
    0x2f, 0xc1, 0x6f, 0x43, 0x90, 0x03, 0x30, 0x00, 0x30, 0x10, 0x2f, 0x09, 0x2e, 0x7d, 0x00, 0x10, 0x30, 0x60, 0x29,
    0x09, 0x2e, 0x7b, 0x00, 0xd7, 0x6f, 0x27, 0x1a, 0x2b, 0x2e, 0x7d, 0x00, 0x01, 0x2f, 0x27, 0x2e, 0x7d, 0x00, 0x09,
    0x2e, 0x7d, 0x00, 0x05, 0xa3, 0x03, 0x22, 0x00, 0x90, 0x04, 0x2f, 0x6f, 0x56, 0x98, 0x2e, 0x4b, 0xb2, 0xf2, 0x6f,
    0x03, 0x30, 0x00, 0x90, 0x12, 0x2f, 0x01, 0x2e, 0x7c, 0x00, 0x42, 0x7f, 0x1a, 0x25, 0xb2, 0x6f, 0x52, 0x7f, 0x63,
    0x7f, 0x74, 0x82, 0x70, 0x7f, 0x98, 0x2e, 0x92, 0xb2, 0x01, 0xb2, 0x21, 0x2e, 0x7c, 0x00, 0x11, 0x30, 0x00, 0x30,
    0x02, 0x2c, 0x08, 0x22, 0x00, 0x30, 0xd1, 0x6f, 0x01, 0xb2, 0x05, 0x2e, 0x75, 0x00, 0x11, 0x22, 0x23, 0x2e, 0x7b,
    0x00, 0x02, 0x1a, 0x02, 0x2f, 0x01, 0x31, 0x23, 0x2e, 0x5e, 0xf0, 0x82, 0x6f, 0x6f, 0x52, 0x93, 0x40, 0x53, 0x42,
    0x00, 0x2e, 0x93, 0x40, 0x53, 0x42, 0x00, 0x2e, 0x82, 0x40, 0x42, 0x42, 0x21, 0x2e, 0x75, 0x00, 0x01, 0x2d, 0x00,
    0x30, 0xeb, 0x6f, 0x00, 0x5f, 0xb8, 0x2e, 0xc0, 0x2e, 0x21, 0x2e, 0x8b, 0x00, 0x81, 0x8a, 0x82, 0x88, 0x41, 0xb2,
    0x01, 0x2f, 0x00, 0x2e, 0x02, 0x2d, 0x24, 0x2c, 0x00, 0x30, 0x42, 0xb2, 0x01, 0x2f, 0x43, 0x90, 0x37, 0x2f, 0x80,
    0x40, 0xc6, 0x40, 0x86, 0x05, 0x7e, 0x80, 0x69, 0x52, 0xc8, 0x01, 0x00, 0x30, 0x80, 0xa9, 0x46, 0x04, 0xc7, 0x41,
    0x71, 0x22, 0x4f, 0x0f, 0x28, 0x2f, 0xc1, 0x8c, 0x41, 0x41, 0x86, 0x41, 0x4e, 0x04, 0x40, 0xa8, 0x81, 0x05, 0x4e,
    0x22, 0x4f, 0x0f, 0x1f, 0x2f, 0xc2, 0x82, 0x03, 0x41, 0x41, 0x40, 0xd9, 0x04, 0xc0, 0xa8, 0x43, 0x04, 0xd9, 0x22,
    0x5f, 0x0f, 0x16, 0x2f, 0x82, 0x40, 0x80, 0xa8, 0xc2, 0x04, 0x93, 0x22, 0x6b, 0x56, 0x53, 0x0f, 0x0d, 0x2f, 0x42,
    0x41, 0x80, 0xa8, 0xc2, 0x04, 0x93, 0x22, 0x6b, 0x56, 0x53, 0x0f, 0x06, 0x2f, 0x02, 0x41, 0x80, 0xa8, 0xc2, 0x04,
    0x93, 0x22, 0x6b, 0x56, 0xd3, 0x0e, 0x05, 0x2f, 0x10, 0x30, 0xb8, 0x2e, 0x10, 0x30, 0xb8, 0x2e, 0x00, 0x30, 0xb8,
    0x2e, 0xb8, 0x2e, 0x30, 0x50, 0x42, 0x80, 0x42, 0x40, 0xf1, 0x7f, 0xe0, 0x7f, 0xdb, 0x7f, 0x02, 0x25, 0x67, 0x52,
    0x98, 0x2e, 0x67, 0xb6, 0xe2, 0x6f, 0x00, 0x2e, 0xa1, 0x40, 0xa0, 0x40, 0x82, 0x40, 0x82, 0x86, 0x94, 0x40, 0x24,
    0x18, 0x82, 0x40, 0x46, 0x25, 0x57, 0x25, 0x12, 0x18, 0xa6, 0x00, 0x2f, 0x03, 0xc3, 0x40, 0x1b, 0x18, 0x57, 0x25,
    0xaa, 0xb9, 0x46, 0x18, 0x46, 0xbe, 0x94, 0xbc, 0xe8, 0x18, 0xe3, 0x0a, 0x19, 0x00, 0x26, 0xbd, 0x47, 0x0e, 0x0f,
    0x2f, 0x47, 0x1a, 0x01, 0x2f, 0x56, 0x0e, 0x0b, 0x2f, 0x79, 0x00, 0x4b, 0x0e, 0x00, 0x30, 0x08, 0x2f, 0x4b, 0x1a,
    0x01, 0x2f, 0x72, 0x0e, 0x04, 0x2f, 0xf0, 0x6f, 0x03, 0x80, 0x02, 0x2c, 0x00, 0x40, 0x10, 0x30, 0xdb, 0x6f, 0xd0,
    0x5f, 0xb8, 0x2e, 0x05, 0x2e, 0x8a, 0x00, 0x91, 0x40, 0x82, 0x40, 0x9b, 0xb8, 0x5b, 0x56, 0x10, 0x50, 0xd3, 0x08,
    0x40, 0xb2, 0x06, 0x30, 0x61, 0x2f, 0xc0, 0xb2, 0x5f, 0x2f, 0x07, 0x2e, 0x6e, 0x00, 0x01, 0x82, 0xc0, 0x90, 0x0e,
    0x2f, 0xc1, 0x84, 0x25, 0x2e, 0x6e, 0x00, 0x00, 0x40, 0x21, 0x2e, 0x70, 0x00, 0x65, 0x50, 0x52, 0x40, 0x12, 0x42,
    0xf0, 0x5f, 0x41, 0x40, 0x01, 0x42, 0x2d, 0x2e, 0x6f, 0x00, 0xb8, 0x2e, 0x5f, 0x56, 0x13, 0x09, 0x00, 0xb3, 0x5d,
    0x56, 0xd3, 0x08, 0x04, 0x30, 0x06, 0x2f, 0x04, 0x40, 0x0b, 0x2e, 0x70, 0x00, 0x25, 0x05, 0x00, 0xa9, 0x74, 0x05,
    0x25, 0x23, 0x33, 0x5a, 0x55, 0x09, 0x40, 0xb3, 0x07, 0x2f, 0x45, 0x40, 0x0d, 0x2e, 0x71, 0x00, 0x6e, 0x05, 0x06,
    0x30, 0x40, 0xa9, 0xb5, 0x05, 0xae, 0x23, 0x61, 0x5a, 0x55, 0x09, 0x40, 0xb3, 0x05, 0x30, 0x07, 0x30, 0x0a, 0x2f,
    0x02, 0x8e, 0xf6, 0x7f, 0x00, 0x2e, 0xc6, 0x41, 0x0f, 0x2e, 0x72, 0x00, 0xb7, 0x05, 0x80, 0xa9, 0xee, 0x05, 0xf7,
    0x23, 0xf6, 0x6f, 0x63, 0x0f, 0x03, 0x2f, 0x73, 0x0f, 0x01, 0x2f, 0xfb, 0x0e, 0x02, 0x2f, 0x2b, 0x2e, 0x6f, 0x00,
    0x0a, 0x2d, 0x09, 0x2e, 0x6f, 0x00, 0x13, 0x30, 0x63, 0x5c, 0x23, 0x29, 0x96, 0x08, 0xe2, 0x0f, 0x29, 0x2e, 0x6f,
    0x00, 0x5d, 0x23, 0x00, 0x40, 0x21, 0x2e, 0x70, 0x00, 0x65, 0x50, 0x52, 0x40, 0x12, 0x42, 0x40, 0xb3, 0x41, 0x40,
    0x01, 0x42, 0x05, 0x2f, 0x00, 0x32, 0x21, 0x2e, 0x5e, 0xf0, 0x02, 0x2d, 0x2d, 0x2e, 0x6e, 0x00, 0xf0, 0x5f, 0xb8,
    0x2e, 0xc0, 0x2e, 0x21, 0x2e, 0x8a, 0x00, 0x03, 0x2e, 0x89, 0x00, 0x41, 0x80, 0x40, 0x50, 0x00, 0x40, 0x03, 0xbd,
    0x2f, 0xb9, 0xfb, 0x7f, 0x80, 0xb2, 0x0b, 0x30, 0x36, 0x2f, 0x05, 0x2e, 0x6c, 0x00, 0x80, 0x90, 0x04, 0x2f, 0x81,
    0x84, 0x25, 0x2e, 0x6c, 0x00, 0x37, 0x2e, 0x6d, 0x00, 0x42, 0x86, 0x42, 0x40, 0xc1, 0x40, 0xa1, 0xbd, 0x04, 0xbd,
    0x24, 0xb9, 0xe2, 0x7f, 0x94, 0xbc, 0x14, 0xb8, 0xc0, 0x7f, 0xb1, 0xb9, 0x1b, 0x52, 0xd3, 0x7f, 0x98, 0x2e, 0x86,
    0xb6, 0xe2, 0x6f, 0xd1, 0x6f, 0x8a, 0x28, 0x42, 0x0f, 0x0d, 0x2f, 0xc1, 0x0e, 0x05, 0x2e, 0x6d, 0x00, 0x13, 0x30,
    0x13, 0x28, 0x04, 0x2f, 0x80, 0xa6, 0x08, 0x2f, 0x21, 0x2e, 0x6d, 0x00, 0x06, 0x2d, 0x21, 0x2e, 0x6d, 0x00, 0x03,
    0x2d, 0x00, 0x30, 0x21, 0x2e, 0x6d, 0x00, 0x03, 0x2e, 0x6d, 0x00, 0xc0, 0x6f, 0xc8, 0x0e, 0x05, 0x2f, 0x80, 0x30,
    0x21, 0x2e, 0x5e, 0xf0, 0x02, 0x2d, 0x37, 0x2e, 0x6c, 0x00, 0xfb, 0x6f, 0xc0, 0x5f, 0xb8, 0x2e, 0x01, 0x2e, 0x80,
    0x00, 0x01, 0x82, 0xa0, 0x50, 0x43, 0x40, 0x2a, 0x25, 0x44, 0x40, 0xb6, 0x84, 0xb3, 0xbd, 0x46, 0x40, 0xbf, 0xb9,
    0x42, 0xbe, 0x83, 0x42, 0x4f, 0xba, 0x45, 0x40, 0x61, 0xbf, 0xf2, 0x7f, 0x6f, 0xb9, 0xdf, 0xb9, 0x74, 0x7f, 0xc0,
    0xb2, 0x82, 0x7f, 0xeb, 0x7f, 0x08, 0x2f, 0x62, 0x6f, 0x80, 0x90, 0x08, 0x2f, 0x72, 0x6f, 0x80, 0x90, 0x05, 0x2f,
    0x82, 0x6f, 0x80, 0x90, 0x02, 0x2f, 0x02, 0x30, 0x25, 0x2e, 0x65, 0x00, 0x07, 0x2e, 0x65, 0x00, 0xc0, 0x90, 0x02,
    0x30, 0x55, 0x58, 0x59, 0x5a, 0x07, 0x2f, 0x55, 0x5c, 0x00, 0x2e, 0x92, 0x43, 0x75, 0x0e, 0xfc, 0x2f, 0xc1, 0x86,
    0x27, 0x2e, 0x65, 0x00, 0x41, 0x40, 0x03, 0x40, 0xb1, 0xbd, 0x02, 0x80, 0xb1, 0xb9, 0x94, 0xbc, 0x00, 0x40, 0x94,
    0xb8, 0xb3, 0x7f, 0x04, 0xbc, 0x59, 0x04, 0xa2, 0x7f, 0xc2, 0x7f, 0xd2, 0x7f, 0x04, 0xb8, 0x57, 0x56, 0x12, 0x30,
    0x1f, 0x5a, 0xf6, 0x6f, 0x95, 0x7f, 0x00, 0x2e, 0x97, 0x41, 0xc0, 0x91, 0x07, 0x30, 0xf6, 0x7f, 0x02, 0x2f, 0x00,
    0x2e, 0x3b, 0x2c, 0x07, 0x43, 0x46, 0x41, 0x80, 0xa9, 0xfe, 0x05, 0xf7, 0x23, 0x05, 0x41, 0x40, 0xb3, 0x28, 0x2f,
    0xcb, 0x40, 0xf9, 0x0e, 0x05, 0x2f, 0x80, 0xa1, 0x06, 0x30, 0x96, 0x23, 0x7b, 0x25, 0x3e, 0x1a, 0x02, 0x2f, 0x05,
    0x30, 0x28, 0x2c, 0x05, 0x43, 0x6a, 0x29, 0x05, 0x43, 0xe8, 0x0e, 0x22, 0x2f, 0xc5, 0x6f, 0x40, 0x91, 0x1f, 0x2f,
    0xd6, 0x6f, 0xa4, 0x7f, 0x5b, 0x25, 0x16, 0x15, 0xd3, 0xbe, 0xc4, 0x7f, 0x2c, 0x0b, 0x55, 0x5a, 0xae, 0x01, 0x59,
    0x5e, 0x0b, 0x30, 0x2e, 0x1a, 0x00, 0x2f, 0x4b, 0x43, 0x41, 0x8b, 0x29, 0x2e, 0x85, 0x00, 0x6f, 0x0e, 0xf7, 0x2f,
    0xa4, 0x6f, 0x0b, 0x2c, 0xa2, 0x7f, 0xb5, 0x6f, 0xfd, 0x0e, 0x06, 0x2f, 0x96, 0x6f, 0x02, 0x43, 0x05, 0x30, 0x86,
    0x41, 0x80, 0xa1, 0x55, 0x23, 0xc5, 0x42, 0x01, 0x89, 0xd5, 0x6f, 0x41, 0x8d, 0x95, 0x6f, 0x41, 0x8b, 0xc1, 0x86,
    0x83, 0xa3, 0xd6, 0x7f, 0xb1, 0x2f, 0xa0, 0x6f, 0x00, 0xb2, 0x40, 0x30, 0x01, 0x2f, 0x21, 0x2e, 0x5e, 0xf0, 0xeb,
    0x6f, 0x60, 0x5f, 0xb8, 0x2e, 0x53, 0x52, 0x00, 0x2e, 0x64, 0x40, 0x51, 0x25, 0x62, 0x40, 0x40, 0x40, 0x00, 0xb2,
    0xa8, 0xb8, 0xa8, 0xbd, 0x61, 0x0a, 0x18, 0x0a, 0x00, 0x2f, 0xb8, 0x2e, 0x45, 0x41, 0x40, 0x91, 0x06, 0x2f, 0x05,
    0x2e, 0x28, 0xf0, 0x22, 0x1a, 0x4c, 0x22, 0xc0, 0x2e, 0xf2, 0x3f, 0x02, 0x22, 0x15, 0x1a, 0x37, 0x58, 0xc0, 0x2e,
    0x9c, 0x0a, 0x02, 0x22, 0x51, 0x52, 0x10, 0x50, 0x52, 0x40, 0xfb, 0x7f, 0x44, 0x80, 0x4b, 0x40, 0x12, 0x42, 0x0b,
    0x42, 0x37, 0x80, 0x05, 0x82, 0x0b, 0x40, 0x4b, 0x42, 0x7c, 0x80, 0x05, 0x82, 0x0b, 0x40, 0x4b, 0x42, 0x7c, 0x80,
    0x05, 0x82, 0x00, 0x40, 0x40, 0x42, 0x77, 0x80, 0x00, 0x2e, 0x11, 0x40, 0x04, 0x84, 0x0b, 0x40, 0x91, 0x42, 0xb7,
    0x80, 0x05, 0x82, 0x00, 0x40, 0x40, 0x42, 0x7c, 0x80, 0x8b, 0x42, 0x05, 0x82, 0x0b, 0x40, 0x4b, 0x42, 0x7c, 0x80,
    0x05, 0x82, 0x00, 0x40, 0x40, 0x42, 0x7c, 0x8c, 0x98, 0x2e, 0x22, 0xb4, 0x4f, 0x54, 0x90, 0x43, 0x81, 0x43, 0xbc,
    0x83, 0xa0, 0x40, 0x83, 0x8a, 0x83, 0x40, 0x08, 0xbe, 0x62, 0x41, 0x40, 0x41, 0x43, 0x8b, 0x23, 0x0b, 0x28, 0xbd,
    0x63, 0x41, 0x10, 0x0a, 0x54, 0x42, 0xb8, 0xbd, 0x42, 0x41, 0x50, 0x42, 0x1a, 0x0a, 0xfb, 0x6f, 0xc0, 0x2e, 0x40,
    0x42, 0xf0, 0x5f, 0x4b, 0x50, 0x4d, 0x52, 0x12, 0x40, 0x52, 0x42, 0xa8, 0xb5, 0x12, 0x40, 0x53, 0x42, 0x42, 0x42,
    0x42, 0x82, 0x00, 0x40, 0x88, 0xb5, 0x50, 0x42, 0x43, 0x42, 0x7e, 0x80, 0xa8, 0xb4, 0x01, 0x42, 0xb8, 0x2e, 0x01,
    0x2e, 0x02, 0x01, 0x8e, 0xbc, 0x01, 0x2e, 0x4c, 0x00, 0x9e, 0xb8, 0x01, 0x1a, 0x70, 0x2f, 0x01, 0x2e, 0x02, 0x01,
    0x0e, 0xbc, 0x0e, 0xb8, 0x03, 0xb2, 0x21, 0x2e, 0x4c, 0x00, 0x4a, 0x2f, 0x02, 0xb2, 0x28, 0x2f, 0x01, 0xb2, 0x07,
    0x2f, 0x49, 0x52, 0x00, 0x30, 0x50, 0x42, 0x40, 0x42, 0x7e, 0x82, 0xc0, 0x2e, 0x40, 0x42, 0x00, 0x2e, 0x01, 0x2e,
    0x7e, 0x00, 0x1f, 0x90, 0x07, 0x2f, 0x49, 0x50, 0x31, 0x37, 0x11, 0x42, 0x3e, 0x82, 0x62, 0x30, 0x47, 0x56, 0x02,
    0x42, 0x43, 0x42, 0x00, 0x2e, 0x01, 0x2e, 0x7e, 0x00, 0x1a, 0xb2, 0x03, 0x2f, 0x01, 0x2e, 0x7e, 0x00, 0x1e, 0x90,
    0x47, 0x2f, 0x49, 0x50, 0x61, 0x36, 0x11, 0x42, 0x62, 0x30, 0x47, 0x56, 0x02, 0x42, 0x3e, 0x82, 0xc0, 0x2e, 0x43,
    0x42, 0x00, 0x2e, 0x01, 0x2e, 0x7e, 0x00, 0x1f, 0xb2, 0x11, 0x2f, 0x01, 0x2e, 0x7e, 0x00, 0x1a, 0xb2, 0x03, 0x2f,
    0x01, 0x2e, 0x7e, 0x00, 0x1e, 0x90, 0x31, 0x2f, 0x49, 0x50, 0x21, 0x31, 0x11, 0x42, 0x52, 0x30, 0x3e, 0x82, 0x02,
    0x42, 0x23, 0x32, 0xc0, 0x2e, 0x43, 0x42, 0x00, 0x2e, 0x49, 0x50, 0x01, 0x32, 0x11, 0x42, 0x52, 0x30, 0x3e, 0x82,
    0x02, 0x42, 0x23, 0x32, 0xc0, 0x2e, 0x43, 0x42, 0x00, 0x2e, 0x01, 0x2e, 0x7e, 0x00, 0x1f, 0xb2, 0x11, 0x2f, 0x01,
    0x2e, 0x7e, 0x00, 0x1a, 0xb2, 0x03, 0x2f, 0x01, 0x2e, 0x7e, 0x00, 0x1e, 0x90, 0x11, 0x2f, 0x49, 0x50, 0xa1, 0x30,
    0x11, 0x42, 0x42, 0x30, 0x3e, 0x82, 0x02, 0x42, 0x13, 0x31, 0xc0, 0x2e, 0x43, 0x42, 0x00, 0x2e, 0x49, 0x50, 0x61,
    0x31, 0x11, 0x42, 0x3e, 0x82, 0x42, 0x30, 0x13, 0x31, 0x02, 0x42, 0x43, 0x42, 0xb8, 0x2e, 0x01, 0x2e, 0x50, 0x00,
    0x15, 0x30, 0xe8, 0x15, 0x06, 0x31, 0x30, 0x8a, 0x8f, 0x0f, 0xb0, 0x05, 0x11, 0x2f, 0x40, 0xa4, 0x0d, 0x2f, 0xda,
    0x04, 0x46, 0x25, 0x19, 0x18, 0x00, 0xb2, 0x07, 0x2f, 0x10, 0xa0, 0x02, 0x2f, 0x00, 0x2e, 0x04, 0x2c, 0xbd, 0x11,
    0xf0, 0x12, 0x7c, 0x14, 0x99, 0x0b, 0x96, 0x00, 0x02, 0x25, 0xb8, 0x2e, 0xa3, 0x04, 0x4f, 0x04, 0x46, 0x25, 0x00,
    0xb2, 0x11, 0x18, 0x07, 0x2f, 0x10, 0xa0, 0x02, 0x2f, 0x00, 0x2e, 0x04, 0x2c, 0xbd, 0x11, 0xb0, 0x12, 0x7c, 0x14,
    0x91, 0x0b, 0x1e, 0x00, 0xb8, 0x2e, 0x30, 0x50, 0xfb, 0x7f, 0x98, 0x2e, 0x22, 0xb4, 0x45, 0x58, 0x00, 0x2e, 0x10,
    0x43, 0x01, 0x43, 0x3a, 0x8b, 0x98, 0x2e, 0x91, 0xb4, 0x00, 0x2e, 0x41, 0x41, 0x40, 0xb2, 0x43, 0x2f, 0x04, 0x83,
    0x01, 0x2e, 0x51, 0x00, 0x42, 0x40, 0x77, 0x82, 0x02, 0x04, 0x00, 0xac, 0x40, 0x42, 0x01, 0x2f, 0x21, 0x2e, 0x4d,
    0x00, 0x50, 0x40, 0x52, 0x40, 0x02, 0x0f, 0x02, 0x30, 0x01, 0x2f, 0x00, 0xac, 0x01, 0x2f, 0x25, 0x2e, 0x4d, 0x00,
    0x7e, 0x88, 0x06, 0x85, 0x85, 0x86, 0xc5, 0x80, 0x41, 0x40, 0x04, 0x41, 0x4c, 0x04, 0x05, 0x8a, 0x82, 0x40, 0xc3,
    0x40, 0x04, 0x40, 0xe5, 0x7f, 0xd1, 0x7f, 0x98, 0x2e, 0x0b, 0xb5, 0xe5, 0x6f, 0x72, 0x83, 0x45, 0x84, 0x85, 0x86,
    0xc5, 0x8c, 0xc4, 0x40, 0x83, 0x40, 0x42, 0x40, 0xd1, 0x6f, 0x40, 0x43, 0xe6, 0x7f, 0x98, 0x2e, 0x0b, 0xb5, 0xe2,
    0x6f, 0xd1, 0x6f, 0x80, 0x42, 0xb2, 0x84, 0x85, 0x86, 0xc5, 0x88, 0x05, 0x81, 0x82, 0x40, 0xc3, 0x40, 0x04, 0x41,
    0xe0, 0x7f, 0x98, 0x2e, 0x0b, 0xb5, 0xe1, 0x6f, 0x14, 0x30, 0x40, 0x42, 0x98, 0x2e, 0x80, 0xb4, 0x29, 0x2e, 0x5e,
    0xf0, 0xfb, 0x6f, 0xd0, 0x5f, 0xb8, 0x2e, 0x03, 0x2e, 0x01, 0x01, 0x05, 0x2e, 0x01, 0x01, 0x92, 0xbd, 0x21, 0xbd,
    0x03, 0x2e, 0x01, 0x01, 0xbf, 0xb9, 0x2f, 0xbb, 0x10, 0x50, 0x9f, 0xba, 0xc0, 0x90, 0x02, 0x30, 0x03, 0x2f, 0x80,
    0x91, 0x01, 0x2f, 0x40, 0xb3, 0x73, 0x2f, 0x09, 0x2e, 0x7f, 0x00, 0x01, 0x82, 0x00, 0x91, 0x12, 0x2f, 0x01, 0x87,
    0x27, 0x2e, 0x7f, 0x00, 0x00, 0x40, 0x21, 0x2e, 0x48, 0x00, 0x43, 0x50, 0x53, 0x40, 0x13, 0x42, 0xf0, 0x5f, 0x41,
    0x40, 0x01, 0x42, 0x25, 0x2e, 0x46, 0x00, 0x25, 0x2e, 0x47, 0x00, 0x25, 0x2e, 0x4b, 0x00, 0xb8, 0x2e, 0x0f, 0x2e,
    0x00, 0x01, 0xc0, 0xb2, 0x09, 0x2e, 0x01, 0x01, 0xf5, 0xbf, 0xf5, 0xb9, 0x43, 0xbe, 0x43, 0xba, 0x07, 0x30, 0x09,
    0x2f, 0xf0, 0x7f, 0x00, 0x2e, 0x00, 0x40, 0x0f, 0x2e, 0x48, 0x00, 0x07, 0x04, 0x00, 0xa8, 0xd0, 0x05, 0xc7, 0x23,
    0xf0, 0x6f, 0x80, 0xb3, 0x07, 0x2f, 0x42, 0x40, 0x0d, 0x2e, 0x49, 0x00, 0x96, 0x04, 0x06, 0x30, 0x80, 0xa8, 0xb2,
    0x05, 0x96, 0x22, 0x40, 0xb3, 0x05, 0x30, 0x06, 0x30, 0x0a, 0x2f, 0x02, 0x8c, 0xf2, 0x7f, 0x00, 0x2e, 0x82, 0x41,
    0x0d, 0x2e, 0x4a, 0x00, 0x96, 0x04, 0x80, 0xa8, 0xaa, 0x05, 0x96, 0x23, 0xf2, 0x6f, 0x7b, 0x0f, 0x0f, 0x2f, 0x53,
    0x0f, 0x0d, 0x2f, 0x73, 0x0f, 0x0b, 0x2f, 0x05, 0x2e, 0x47, 0x00, 0x81, 0x84, 0x2b, 0x2e, 0x46, 0x00, 0x54, 0x0e,
    0x25, 0x2e, 0x47, 0x00, 0x0e, 0x2f, 0x2b, 0x2e, 0x4b, 0x00, 0x0c, 0x2d, 0x07, 0x2e, 0x46, 0x00, 0x12, 0x30, 0xda,
    0x28, 0x2b, 0x2e, 0x47, 0x00, 0x5c, 0x0e, 0x27, 0x2e, 0x46, 0x00, 0x01, 0x2f, 0x25, 0x2e, 0x4b, 0x00, 0x05, 0x2e,
    0x4b, 0x00, 0x80, 0xb2, 0x0d, 0x2f, 0x00, 0x40, 0x21, 0x2e, 0x48, 0x00, 0x43, 0x50, 0x52, 0x40, 0x12, 0x42, 0x22,
    0x30, 0x41, 0x40, 0x01, 0x42, 0x25, 0x2e, 0x5e, 0xf0, 0x02, 0x2d, 0x25, 0x2e, 0x7f, 0x00, 0xf0, 0x5f, 0xb8, 0x2e,
    0x41, 0x50, 0x41, 0x30, 0x02, 0x40, 0x51, 0x0a, 0x01, 0x42, 0x18, 0x82, 0x35, 0x50, 0x60, 0x42, 0x70, 0x3c, 0x37,
    0x54, 0x42, 0x42, 0x69, 0x82, 0x82, 0x32, 0x43, 0x40, 0x18, 0x08, 0x02, 0x0a, 0x40, 0x42, 0x42, 0x80, 0x02, 0x3f,
    0x01, 0x40, 0x10, 0x50, 0x4a, 0x08, 0xfb, 0x7f, 0x11, 0x42, 0x0b, 0x31, 0x0b, 0x42, 0x3e, 0x80, 0xd1, 0x30, 0x01,
    0x42, 0x00, 0x2e, 0x01, 0x2e, 0x40, 0xf0, 0x1a, 0x90, 0x20, 0x2f, 0x03, 0x30, 0x3b, 0x50, 0x39, 0x54, 0x64, 0x34,
    0x06, 0x30, 0x3f, 0x52, 0x55, 0x32, 0x1d, 0x1a, 0xe3, 0x22, 0x18, 0x1a, 0x3d, 0x58, 0xe3, 0x22, 0x04, 0x30, 0xd5,
    0x40, 0xb5, 0x0d, 0xe1, 0xbe, 0x6f, 0xbb, 0x80, 0x91, 0xa9, 0x0d, 0x01, 0x89, 0xb5, 0x23, 0x10, 0xa1, 0xf7, 0x2f,
    0xda, 0x0e, 0x64, 0x34, 0xeb, 0x2f, 0x01, 0x2e, 0x25, 0x00, 0x70, 0x1a, 0x00, 0x30, 0x21, 0x30, 0x02, 0x2c, 0x08,
    0x22, 0x30, 0x30, 0x00, 0xb2, 0x06, 0x2f, 0x21, 0x2e, 0x59, 0xf0, 0x98, 0x2e, 0xca, 0xb6, 0x00, 0x2e, 0x00, 0x2e,
    0xd0, 0x2e, 0xfb, 0x6f, 0xf0, 0x5f, 0xb8, 0x2e, 0x03, 0x30, 0x15, 0x40, 0xd9, 0x04, 0x2b, 0x0e, 0x1d, 0x23, 0x29,
    0x0f, 0x15, 0x40, 0x0c, 0x23, 0x2b, 0x0e, 0x9d, 0x23, 0x29, 0x0f, 0x4e, 0x23, 0x00, 0x40, 0x03, 0x0e, 0xd8, 0x22,
    0x01, 0x0f, 0x94, 0x42, 0x4b, 0x22, 0x95, 0x42, 0x81, 0x42, 0xb8, 0x2e, 0x83, 0x86, 0x01, 0x30, 0x00, 0x30, 0x94,
    0x40, 0x24, 0x18, 0x06, 0x00, 0x53, 0x0e, 0x4f, 0x02, 0xf9, 0x2f, 0xb8, 0x2e, 0x10, 0x50, 0xfb, 0x7f, 0x21, 0x25,
    0x98, 0x2e, 0x7c, 0xb6, 0xfb, 0x6f, 0x21, 0x25, 0xf0, 0x5f, 0x10, 0x25, 0x80, 0x2e, 0x91, 0xb6, 0x30, 0x50, 0x00,
    0x30, 0x33, 0x56, 0x05, 0x30, 0x05, 0x2c, 0xfb, 0x7f, 0x3e, 0xbe, 0xd2, 0xba, 0xb2, 0xb9, 0x6c, 0x0b, 0x53, 0x0e,
    0xf9, 0x2f, 0x53, 0x1a, 0x01, 0x2f, 0x4d, 0x0e, 0xf5, 0x2f, 0xd2, 0x7f, 0x04, 0x30, 0x1f, 0x2c, 0xe1, 0x7f, 0xc5,
    0x01, 0xa3, 0x03, 0x72, 0x0e, 0x03, 0x2f, 0x72, 0x1a, 0x0f, 0x2f, 0x79, 0x0f, 0x0d, 0x2f, 0xe1, 0x6f, 0x4f, 0x04,
    0x5f, 0xb9, 0xb1, 0xbf, 0xfa, 0x0b, 0xd2, 0x6f, 0x96, 0x06, 0xb1, 0x25, 0x51, 0xbf, 0xeb, 0x7f, 0x06, 0x00, 0xb2,
    0x25, 0x27, 0x03, 0xdb, 0x7f, 0xcf, 0xbf, 0x3e, 0xbf, 0x01, 0xb8, 0xd2, 0xba, 0x41, 0xba, 0xb2, 0xb9, 0x07, 0x0a,
    0x6e, 0x0b, 0xc0, 0x90, 0xdf, 0x2f, 0x40, 0x91, 0xdd, 0x2f, 0xfb, 0x6f, 0xd0, 0x5f, 0xb8, 0x2e, 0x01, 0x2e, 0x55,
    0xf0, 0xc0, 0x2e, 0x21, 0x2e, 0x55, 0xf0, 0x1a, 0x24, 0x26, 0x00, 0x80, 0x2e, 0xfa, 0xb6, 0x43, 0x86, 0x25, 0x40,
    0x04, 0x40, 0xd8, 0xbe, 0x2c, 0x0b, 0x22, 0x11, 0x54, 0x42, 0x03, 0x80, 0x4b, 0x0e, 0xf6, 0x2f, 0xb8, 0x2e, 0x01,
    0x2e, 0x80, 0x00, 0x01, 0x80, 0xc0, 0x2e, 0x00, 0x40, 0x0f, 0xb8, 0x31, 0x50, 0x2f, 0x52, 0x02, 0x40, 0x51, 0x0a,
    0x01, 0x42, 0x09, 0x80, 0x21, 0x52, 0xc0, 0x2e, 0x01, 0x42, 0x00, 0x2e, 0x10, 0x30, 0xc0, 0x2e, 0x21, 0x2e, 0xbb,
    0xf0, 0x00, 0x31, 0xc0, 0x2e, 0x21, 0x2e, 0xba, 0xf0, 0x40, 0x30, 0xc0, 0x2e, 0x21, 0x2e, 0xba, 0xf0, 0x10, 0x50,
    0x98, 0x2e, 0x17, 0xb6, 0x20, 0x26, 0x98, 0x2e, 0xf6, 0xb6, 0x98, 0x2e, 0xee, 0xb6, 0x98, 0x2e, 0xf2, 0xb6, 0x27,
    0x50, 0x21, 0x2e, 0x89, 0x00, 0x25, 0x52, 0x23, 0x2e, 0x80, 0x00, 0x29, 0x50, 0x98, 0x2e, 0x3c, 0xb3, 0x2b, 0x50,
    0x98, 0x2e, 0x48, 0xb2, 0x01, 0x2e, 0x40, 0xf0, 0x21, 0x2e, 0x7e, 0x00, 0x10, 0x30, 0x21, 0x2e, 0x59, 0xf0, 0x98,
    0x2e, 0xca, 0xb6, 0x00, 0x30, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x03, 0x2e, 0x1c, 0x00, 0x40, 0xb2, 0x24, 0x2f,
    0x21, 0x2e, 0x1c, 0x00, 0x20, 0x30, 0x21, 0x2e, 0x5f, 0xf0, 0x1b, 0x50, 0x98, 0x2e, 0x8c, 0xb5, 0x80, 0x30, 0x21,
    0x2e, 0x5f, 0xf0, 0x98, 0x2e, 0x3f, 0xb3, 0x00, 0x32, 0x21, 0x2e, 0x5f, 0xf0, 0x1b, 0x50, 0x98, 0x2e, 0xcb, 0xb2,
    0x2d, 0x52, 0x79, 0x84, 0xf2, 0x7f, 0x00, 0x31, 0x1b, 0x52, 0x21, 0x2e, 0x5f, 0xf0, 0x98, 0x2e, 0x69, 0xb1, 0x21,
    0x2e, 0x24, 0x00, 0xf0, 0x6f, 0x03, 0x2e, 0x24, 0x00, 0x01, 0x42, 0x98, 0x2e, 0xca, 0xb6, 0x00, 0x30, 0x03, 0x2e,
    0x1d, 0x00, 0x40, 0xb2, 0xcf, 0x2f, 0x41, 0x30, 0x23, 0x2e, 0x5f, 0xf0, 0x21, 0x2e, 0x1d, 0x00, 0x98, 0x2e, 0x86,
    0xb3, 0xc8, 0x2c, 0x00, 0x30, 0x90, 0x50, 0xe7, 0x7f, 0xf6, 0x7f, 0x16, 0x30, 0x0f, 0x2e, 0xb9, 0xf0, 0xfe, 0x09,
    0xc0, 0xb3, 0x10, 0x2f, 0xd5, 0x7f, 0xc4, 0x7f, 0xb3, 0x7f, 0xa2, 0x7f, 0x91, 0x7f, 0x80, 0x7f, 0x7b, 0x7f, 0x98,
    0x2e, 0x36, 0xb5, 0x80, 0x6f, 0x91, 0x6f, 0xa2, 0x6f, 0xb3, 0x6f, 0xc4, 0x6f, 0xd5, 0x6f, 0x7b, 0x6f, 0x16, 0x30,
    0x2d, 0x2e, 0xb9, 0xf0, 0x2d, 0x2e, 0x5f, 0xf0, 0xf6, 0x6f, 0xe7, 0x6f, 0x70, 0x5f, 0xc8, 0x2e, 0x70, 0x50, 0xf5,
    0x7f, 0xe1, 0x7f, 0xd0, 0x7f, 0xc3, 0x7f, 0xbb, 0x7f, 0x98, 0x2e, 0xde, 0xb6, 0x00, 0x90, 0xa2, 0x7f, 0x94, 0x7f,
    0x03, 0x2f, 0x21, 0x50, 0x21, 0x2e, 0xbc, 0xf0, 0x0d, 0x2d, 0x01, 0x2e, 0x01, 0xf0, 0x0e, 0xbc, 0x0e, 0xb8, 0x32,
    0x30, 0x90, 0x04, 0x23, 0x50, 0x1f, 0x52, 0x98, 0x2e, 0xd3, 0xb6, 0x10, 0x30, 0x21, 0x2e, 0x1d, 0x00, 0x21, 0x50,
    0x21, 0x2e, 0xb8, 0xf0, 0xf5, 0x6f, 0xe1, 0x6f, 0xa2, 0x6f, 0xd0, 0x6f, 0xc3, 0x6f, 0x94, 0x6f, 0xbb, 0x6f, 0x90,
    0x5f, 0xc8, 0x2e, 0x70, 0x50, 0xd1, 0x7f, 0xf5, 0x7f, 0xe4, 0x7f, 0x34, 0x30, 0x03, 0x2e, 0x01, 0xf0, 0x9e, 0xbc,
    0x9e, 0xba, 0x92, 0x7f, 0xc0, 0x7f, 0xbb, 0x7f, 0xa3, 0x7f, 0xa5, 0x04, 0x1b, 0x52, 0x1d, 0x50, 0x98, 0x2e, 0xd3,
    0xb6, 0x10, 0x30, 0x21, 0x2e, 0x1c, 0x00, 0x98, 0x2e, 0xde, 0xb6, 0x00, 0xb2, 0x01, 0x2f, 0x98, 0x2e, 0xe4, 0xb6,
    0x00, 0x31, 0x21, 0x2e, 0xb8, 0xf0, 0xe4, 0x6f, 0xd1, 0x6f, 0xf5, 0x6f, 0xc0, 0x6f, 0x92, 0x6f, 0xa3, 0x6f, 0xbb,
    0x6f, 0x90, 0x5f, 0xc8, 0x2e, 0x10, 0x50, 0xf7, 0x7f, 0x00, 0x2e, 0x0f, 0x2e, 0x29, 0xf0, 0xf8, 0xbf, 0xff, 0xbb,
    0xc1, 0x91, 0x09, 0x2f, 0x0f, 0x2e, 0x02, 0x01, 0xfe, 0xbf, 0xfe, 0xbb, 0xc0, 0xb3, 0x03, 0x2f, 0x0f, 0x2e, 0x37,
    0xf0, 0x2f, 0x2e, 0x37, 0xf0, 0x87, 0x30, 0x2f, 0x2e, 0xb8, 0xf0, 0xf7, 0x6f, 0xf0, 0x5f, 0xc8, 0x2e, 0x90, 0x50,
    0xf7, 0x7f, 0x00, 0x2e, 0x0f, 0x2e, 0x43, 0xf0, 0xf8, 0xbf, 0xff, 0xbb, 0xc0, 0xb3, 0x11, 0x2f, 0xe6, 0x7f, 0xd5,
    0x7f, 0xc4, 0x7f, 0xb3, 0x7f, 0xa2, 0x7f, 0x91, 0x7f, 0x80, 0x7f, 0x7b, 0x7f, 0x98, 0x2e, 0x3e, 0xb4, 0x80, 0x6f,
    0x91, 0x6f, 0xa2, 0x6f, 0xb3, 0x6f, 0xc4, 0x6f, 0xd5, 0x6f, 0xe6, 0x6f, 0x7b, 0x6f, 0x47, 0x30, 0x2f, 0x2e, 0xb8,
    0xf0, 0xf7, 0x6f, 0x70, 0x5f, 0xc8, 0x2e, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d
};

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bmi090l_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi090l_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev);

/*!
 *  @brief This API writes the given data to the register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmi090l_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev);

/*!
 * @brief This API configures the pins which fire the interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_int_pin_config(const struct bmi090l_accel_int_channel_cfg *int_config, const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_data_ready_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                       const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                            const struct bmi090l_dev *dev);

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_input(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_anymotion_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev);

/*!
 * @brief This API writes the config stream data in memory using burst mode
 *
 * @param[in] stream_data : Pointer to store data of 32 bytes
 * @param[in] indx       : Represents value in multiple of 32 bytes
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t indx, const struct bmi090l_dev *dev);

/*!
 * @brief This API performs the pre-requisites needed to perform the self-test
 *
 * @param[in] dev : structure instance of bmi090l_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_self_test(struct bmi090l_dev *dev);

/*!
 * @brief This API reads the accel data with the positive excitation
 *
 * @param[out] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev   : structure instance of bmi090l_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t positive_excited_accel(struct bmi090l_sensor_data *accel_pos, const struct bmi090l_dev *dev);

/*!
 * @brief This API reads the accel data with the negative excitation
 *
 * @param[out] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev   : structure instance of bmi090l_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t negative_excited_accel(struct bmi090l_sensor_data *accel_neg, const struct bmi090l_dev *dev);

/*!
 * @brief This API validates the self-test results
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> self-test fail
 */
static int8_t validate_accel_self_test(const struct bmi090l_sensor_data *accel_pos,
                                       const struct bmi090l_sensor_data *accel_neg);

/*!
 * @brief This API converts lsb value of axes to mg for self-test
 *
 * @param[in] accel_data_diff     : Pointer variable used to pass accel difference
 * values in g
 *
 * @param[out] accel_data_diff_mg : Pointer variable used to store accel
 * difference values in mg
 *
 * @return None
 */
static void convert_lsb_g(const struct bmi090la_selftest_delta_limit *accel_data_diff,
                          struct bmi090la_selftest_delta_limit *accel_data_diff_mg);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data.
 *
 * @param[out] acc              : Structure instance of bmi090l_sensor_data
 *                                where the parsed data bytes are stored.
 * @param[in]  data_start_indx : indx value of the accelerometer data bytes
 *                                which is to be parsed from the FIFO data.
 * @param[in]  fifo             : Structure instance of bmi090l_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void unpack_accel_data(struct bmi090l_sensor_data *acc,
                              uint16_t data_start_indx,
                              const struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in both header and header-less mode. It updates the current data
 * byte to be parsed.
 *
 * @param[in,out] acc       : Structure instance of bmi090l_sensor_data where
 *                            where the parsed data bytes are stored.
 * @param[in,out] idx       : indx value of number of bytes parsed.
 * @param[in,out] acc_idx   : indx value of accelerometer data (x,y,z axes)
 *                            frame to be parsed.
 * @param[in]     frame     : Either data is enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]     fifo      : Structure instance of bmi090l_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMI090L_OK - Success.
 * @retval BMI090L_W_FIFO_EMPTY - Warning : FIFO is empty
 */
static int8_t unpack_accel_frame(struct bmi090l_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 *
 * @param[in,out] data_indx : indx of the FIFO data which contains skipped
 *                             frame count.
 * @param[in] fifo           : Structure instance of bmi090l_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMI090L_OK - Success.
 * @retval BMI090L_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI090L_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t unpack_skipped_frame(uint16_t *data_indx, struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to move the data indx ahead of the
 * current frame length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 *
 * @param[in,out] data_indx           : indx of the FIFO data which is to be
 *                                       moved ahead of the current frame length
 * @param[in]     current_frame_length : Number of bytes in the current frame.
 * @param[in]     fifo                 : Structure instance of bmi090l_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMI090L_OK - Success.
 * @retval BMI090L_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI090L_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t move_next_frame(uint16_t *data_indx, uint8_t current_frame_length, const struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 *
 * @param[in,out] data_indx : indx of the FIFO data which has the sensor time.
 * @param[in]     fifo       : Structure instance of bmi090l_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMI090L_OK - Success.
 * @retval BMI090L_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI090L_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t unpack_sensortime_frame(uint16_t *data_indx, struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to reset the FIFO related configurations
 * in the FIFO frame structure for the next FIFO read.
 *
 * @param[in, out] fifo     : Structure instance of bmi090l_fifo_frame.
 * @param[in]      dev      : Structure instance of bmi090l_dev.
 *
 * @return None
 * @retval None
 */
static void reset_fifo_frame_structure(struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data in header mode.
 *
 * @param[out] acc          : Structure instance of bmi090l_sens_data where
 *                            the parsed accelerometer data bytes are stored.
 * @param[in] accel_length  : Number of accelerometer frames (x,y,z data).
 * @param[in] fifo          : Structure instance of bmi090l_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval BMIX8X_OK - Success.
 * @retval BMI090L_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI090L_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t extract_acc_header_mode(struct bmi090l_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct bmi090l_fifo_frame *fifo);

/*!
 * @brief This API sets the FIFO watermark interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_fifo_wm_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                    const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi090l_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi090l_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_fifo_full_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the high-g interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_high_g_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the low-g interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_low_g_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                  const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the orientation interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_orient_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev);

/*!
 * @brief This API sets the no-motion interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_no_motion_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
int8_t bmi090la_init(struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        if (dev->intf == BMI090L_SPI_INTF)
        {
            /* Set dummy byte in case of SPI interface */
            dev->dummy_byte = BMI090L_ENABLE;

            /* Dummy read of Chip-ID in SPI mode */
            rslt = get_regs(BMI090L_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);
        }
        else
        {
            /* Make dummy byte 0 in case of I2C interface */
            dev->dummy_byte = 0;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(BMI090L_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);

            if (rslt == BMI090L_OK)
            {
                /* Check for chip id validity */
                if (chip_id == BMI090L_ACCEL_CHIP_ID)
                {
                    /* Store the chip ID in dev structure */
                    dev->accel_chip_id = chip_id;
                }
                else
                {
                    rslt = BMI090L_E_DEV_NOT_FOUND;
                }
            }

            if (rslt == BMI090L_OK)
            {
                /* Initialize bmi09 gyro sensor */
                rslt = bmi090lg_init(dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API uploads the bmi09 config file onto the device.
 */
int8_t bmi090la_apply_config_file(struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        /* Assign stream file */
        dev->config_file_ptr = bmi090l_config_file;

        /* Upload binary */
        rslt = bmi090la_write_config_file(dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to write the binary configuration in the sensor.
 */
int8_t bmi090la_write_config_file(const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Config loading disable*/
    uint8_t config_load = BMI090L_DISABLE;
    uint8_t current_acc_pwr_ctrl = 0;
    uint16_t indx = 0;
    uint8_t reg_data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check if config file pointer is not null */
    if ((rslt == BMI090L_OK) && (dev->config_file_ptr != NULL))
    {
        /* Check whether the read/write length is valid */
        if (dev->read_write_len > 0)
        {
            /* Deactivate accel, otherwise post processing can not be enabled safely */
            rslt = get_regs(BMI090L_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
            if (rslt != BMI090L_OK)
            {
                return rslt;
            }

            rslt = set_regs(BMI090L_ACCEL_PWR_CTRL_REG, &config_load, 1, dev);
            if (rslt == BMI090L_OK)
            {
                /* Delay required to switch power modes */
                dev->delay_us(BMI090L_POWER_CONFIG_DELAY * 1000, dev->intf_ptr);
            }
            else
            {
                return rslt;
            }

            /* Disable config load */
            rslt = set_regs(BMI090L_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

            if (rslt == BMI090L_OK)
            {
                for (indx = 0; indx < BMI090L_CONFIG_STREAM_SIZE;
                     indx += dev->read_write_len)
                {
                    /* Write the config stream */
                    rslt = stream_transfer_write((dev->config_file_ptr + indx), indx, dev);
                }

                if (rslt == BMI090L_OK)
                {
                    /* Enable config loading and FIFO mode */
                    config_load = BMI090L_ENABLE;

                    rslt = set_regs(BMI090L_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

                    if (rslt == BMI090L_OK)
                    {
                        /* Wait till ASIC is initialized. Refer the data-sheet for more information */
                        dev->delay_us(BMI090L_ASIC_INIT_TIME_MS * 1000, dev->intf_ptr);

                        /* Check for config initialization status (1 = OK) */
                        rslt = get_regs(BMI090L_ACCEL_INTERNAL_STAT_REG, &reg_data, 1, dev);
                    }

                    if (rslt == BMI090L_OK && reg_data != 1)
                    {
                        rslt = BMI090L_E_CONFIG_STREAM_ERROR;
                    }
                    else
                    {
                        /* Reactivate accel */
                        rslt = set_regs(BMI090L_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
                        if (rslt == BMI090L_OK)
                        {
                            /* Delay required to switch power modes */
                            dev->delay_us(BMI090L_POWER_CONFIG_DELAY * 1000, dev->intf_ptr);
                        }
                    }
                }
            }
        }
        else
        {
            rslt = BMI090L_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 */
int8_t bmi090la_write_feature_config(uint8_t reg_addr,
                                     const uint16_t *reg_data,
                                     uint8_t len,
                                     const struct bmi090l_dev *dev)
{

    int8_t rslt;
    uint16_t read_length = (reg_addr * 2) + (len * 2);
    uint8_t feature_data[read_length];
    uint8_t indx = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        /* Read feature space up to the given feature position */
        rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);

        if (rslt == BMI090L_OK)
        {
            /* Apply the given feature config. */
            for (indx = 0; indx < len; ++indx)
            {
                /* Be careful: the feature config space is 16bit aligned! */
                feature_data[(reg_addr * 2) + (indx * 2)] = reg_data[indx] & 0xFF;
                feature_data[(reg_addr * 2) + (indx * 2) + 1] = reg_data[indx] >> 8;
            }

            /* Write back updated feature space */
            rslt = bmi090la_set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 */
int8_t bmi090la_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Reading from the register */
            rslt = get_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = BMI090L_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 */
int8_t bmi090la_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Writing to the register */
            rslt = set_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = BMI090L_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the error status from the accel sensor.
 */
int8_t bmi090la_get_error_status(struct bmi090l_err_reg *err_reg, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        if (err_reg != NULL)
        {
            /* Read the error codes */
            rslt = get_regs(BMI090L_ACCEL_ERR_REG, &data, 1, dev);

            if (rslt == BMI090L_OK)
            {
                /* Fatal error */
                err_reg->fatal_err = BMI090L_GET_BITS_POS_0(data, BMI090L_FATAL_ERR);

                /* User error */
                err_reg->err_code = BMI090L_GET_BITS(data, BMI090L_ERR_CODE);
            }
        }
        else
        {
            rslt = BMI090L_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of the accel sensor.
 */
int8_t bmi090la_get_status(uint8_t *status, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (status != NULL))
    {
        /* Read the status */
        rslt = get_regs(BMI090L_ACCEL_STATUS_REG, &data, 1, dev);

        if (rslt == BMI090L_OK)
        {
            /* Updating the status */
            *status = BMI090L_GET_BITS(data, BMI090L_ACCEL_STATUS);
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API resets the accel sensor.
 */
int8_t bmi090la_soft_reset(const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        data = BMI090L_SOFT_RESET_CMD;

        /* Reset accel device */
        rslt = set_regs(BMI090L_ACCEL_SOFTRESET_REG, &data, 1, dev);

        if (rslt == BMI090L_OK)
        {
            /* Delay 1 ms after reset value is written to its register */
            dev->delay_us(BMI090L_ACCEL_SOFTRESET_DELAY_MS * 1000, dev->intf_ptr);

            /* After soft-reset SPI mode in the initialization phase, need to  perform a dummy SPI read
             * operation, The soft-reset performs a fundamental reset to the device, which is largely
             * equivalent to a power cycle. */
            if (dev->intf == BMI090L_SPI_INTF)
            {
                /* Dummy SPI read operation of Chip-ID */
                rslt = get_regs(BMI090L_ACCEL_CHIP_ID_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel config value i.e. odr, band width and range from the sensor,
 * store it in the bmi090l_dev structure instance passed by the user.
 *
 */
int8_t bmi090la_get_meas_conf(struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        rslt = get_regs(BMI090L_ACCEL_CONF_REG, data, 2, dev);

        if (rslt == BMI090L_OK)
        {
            dev->accel_cfg.odr = data[0] & BMI090L_ACCEL_ODR_MASK;
            dev->accel_cfg.bw = (data[0] & BMI090L_ACCEL_BW_MASK) >> 4;
            dev->accel_cfg.range = data[1] & BMI090L_ACCEL_RANGE_MASK;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of accel sensor.
 */
int8_t bmi090la_set_meas_conf(const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint8_t bw, range, odr;
    uint8_t is_odr_invalid = FALSE, is_bw_invalid = FALSE, is_range_invalid = FALSE;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        odr = dev->accel_cfg.odr;
        bw = dev->accel_cfg.bw;
        range = dev->accel_cfg.range;

        /* Check for valid ODR */
        if ((odr < BMI090L_ACCEL_ODR_12_5_HZ) || (odr > BMI090L_ACCEL_ODR_1600_HZ))
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        /* Check for valid bandwidth */
        if (bw > BMI090L_ACCEL_BW_NORMAL)
        {
            /* Updating the status */
            is_bw_invalid = TRUE;
        }

        /* Check for valid Range */
        if (range > BMI090L_ACCEL_RANGE_24G)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If ODR, BW and Range are valid, write it to accel config. registers */
        if ((!is_odr_invalid) && (!is_bw_invalid) && (!is_range_invalid))
        {
            /* Read accel config. register */
            rslt = get_regs(BMI090L_ACCEL_CONF_REG, data, 2, dev);
            if (rslt == BMI090L_OK)
            {
                /* Update data with new odr and bw values */
                data[0] = BMI090L_SET_BITS_POS_0(data[0], BMI090L_ACCEL_ODR, odr);
                data[0] = BMI090L_SET_BITS(data[0], BMI090L_ACCEL_BW, bw);

                /* Update data with current range values */
                data[1] = BMI090L_SET_BITS_POS_0(data[1], BMI090L_ACCEL_RANGE, range);

                /* Write accel range to register */
                rslt = set_regs(BMI090L_ACCEL_CONF_REG, data, 2, dev);
            }
        }
        else
        {
            /* Invalid configuration present in ODR, BW, Range */
            rslt = BMI090L_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel power mode from the sensor, store it in the bmi090l_dev structure
 * instance passed by the user.
 */
int8_t bmi090la_get_power_mode(struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        rslt = get_regs(BMI090L_ACCEL_PWR_CONF_REG, &data, 1, dev);

        if (rslt == BMI090L_OK)
        {
            /* Updating the current power mode */
            dev->accel_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the accel sensor.
 */
int8_t bmi090la_set_power_mode(const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode;
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        power_mode = dev->accel_cfg.power;

        /* Configure data array to write to accel power configuration register */
        if (power_mode == BMI090L_ACCEL_PM_ACTIVE)
        {
            data[0] = BMI090L_ACCEL_PM_ACTIVE;
            data[1] = BMI090L_ACCEL_POWER_ENABLE;
        }
        else if (power_mode == BMI090L_ACCEL_PM_SUSPEND)
        {
            data[0] = BMI090L_ACCEL_PM_SUSPEND;
            data[1] = BMI090L_ACCEL_POWER_DISABLE;
        }
        else
        {
            /* Invalid power input */
            rslt = BMI090L_E_INVALID_INPUT;
        }

        if (rslt == BMI090L_OK)
        {
            /* Enable accel sensor */
            rslt = set_regs(BMI090L_ACCEL_PWR_CONF_REG, &data[0], 1, dev);

            if (rslt == BMI090L_OK)
            {
                /* Delay between power ctrl and power config */
                dev->delay_us(BMI090L_POWER_CONFIG_DELAY * 1000, dev->intf_ptr);

                /* Write to accel power configuration register */
                rslt = set_regs(BMI090L_ACCEL_PWR_CTRL_REG, &data[1], 1, dev);

                if (rslt == BMI090L_OK)
                {
                    /* Delay required to switch power modes */
                    dev->delay_us(BMI090L_POWER_CONFIG_DELAY * 1000, dev->intf_ptr);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the bmi090l_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi090la_get_data(struct bmi090l_sensor_data *accel, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (accel != NULL))
    {
        /* Read accel sensor data */
        rslt = get_regs(BMI090L_ACCEL_X_LSB_REG, data, 6, dev);

        if (rslt == BMI090L_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            accel->x = ((int16_t) msblsb); /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            accel->y = ((int16_t) msblsb); /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            accel->z = ((int16_t) msblsb); /* Data in Z axis */
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the bmi090l_int_cfg
 * structure instance.
 */
int8_t bmi090la_set_int_config(const struct bmi090l_accel_int_channel_cfg *int_config, const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (int_config != NULL))
    {
        switch (int_config->int_type)
        {
            case BMI090L_ACCEL_DATA_RDY_INT:

                /* Data ready interrupt */
                rslt = set_accel_data_ready_int(int_config, dev);
                break;
            case BMI090L_ACCEL_SYNC_DATA_RDY_INT:

                /* Synchronized data ready interrupt */
                rslt = set_accel_sync_data_ready_int(int_config, dev);
                break;
            case BMI090L_ACCEL_SYNC_INPUT:

                /* Input for synchronization on accel */
                rslt = set_accel_sync_input(int_config, dev);
                break;
            case BMI090L_ANYMOTION_INT:

                /* Anymotion interrupt */
                rslt = set_accel_anymotion_int(int_config, dev);
                break;
            case BMI090L_FIFO_WM_INT:

                /* FIFO watermark interrupt */
                rslt = set_accel_fifo_wm_int(int_config, dev);
                break;
            case BMI090L_FIFO_FULL_INT:

                /* FIFO full interrupt */
                rslt = set_accel_fifo_full_int(int_config, dev);
                break;
            case BMI090L_HIGH_G_INT:

                /* high-g interrupt */
                rslt = set_accel_high_g_int(int_config, dev);
                break;

            case BMI090L_LOW_G_INT:

                /* low-g interrupt */
                rslt = set_accel_low_g_int(int_config, dev);
                break;

            case BMI090L_ORIENT_INT:

                /* orientation interrupt */
                rslt = set_accel_orient_int(int_config, dev);
                break;

            case BMI090L_NO_MOTION_INT:

                /* no-motion interrupt */
                rslt = set_accel_no_motion_int(int_config, dev);
                break;

            default:
                rslt = BMI090L_E_INVALID_CONFIG;
                break;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the temperature of the sensor in degree Celcius.
 */
int8_t bmi090la_get_sensor_temperature(const struct bmi090l_dev *dev, int32_t *sensor_temp)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint16_t msb, lsb;
    uint16_t msblsb;
    int16_t temp;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (sensor_temp != NULL))
    {
        /* Read sensor temperature */
        rslt = get_regs(BMI090L_TEMP_MSB_REG, data, 2, dev);

        if (rslt == BMI090L_OK)
        {
            msb = (data[0] << 3); /* MSB data */
            lsb = (data[1] >> 5); /* LSB data */
            msblsb = (uint16_t) (msb + lsb);

            if (msblsb > 1023)
            {
                /* Updating the msblsb */
                temp = (int16_t) (msblsb - 2048);
            }
            else
            {
                temp = (int16_t) msblsb;
            }

            /* Sensor temperature */
            *sensor_temp = (temp * 125) + 23000;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 */
int8_t bmi090la_get_sensor_time(const struct bmi090l_dev *dev, uint32_t *sensor_time)
{
    int8_t rslt;
    uint8_t data[3] = { 0 };
    uint32_t byte2, byte1, byte0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (sensor_time != NULL))
    {
        /* Read 3-byte sensor time */
        rslt = get_regs(BMI090L_ACCEL_SENSORTIME_0_REG, data, 3, dev);

        if (rslt == BMI090L_OK)
        {
            byte0 = data[0]; /* Lower byte */
            byte1 = (data[1] << 8); /* Middle byte */
            byte2 = (data[2] << 16); /* Higher byte */

            /* Sensor time */
            *sensor_time = (byte2 | byte1 | byte0);
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self-test functionality of the sensor
 *  is working or not.
 */
int8_t bmi090la_perform_selftest(struct bmi090l_dev *dev)
{
    int8_t rslt;
    int8_t self_test_rslt = 0;
    struct bmi090l_sensor_data accel_pos, accel_neg;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        /* Pre-requisites for self-test */
        rslt = enable_self_test(dev);

        if (rslt == BMI090L_OK)
        {
            rslt = positive_excited_accel(&accel_pos, dev);

            if (rslt == BMI090L_OK)
            {
                rslt = negative_excited_accel(&accel_neg, dev);

                if (rslt == BMI090L_OK)
                {
                    /* Validate the self-test result */
                    rslt = validate_accel_self_test(&accel_pos, &accel_neg);

                    /* Store the status of self-test result */
                    self_test_rslt = rslt;

                    /* Perform soft-reset */
                    rslt = bmi090la_soft_reset(dev);

                    /* Check to ensure bus operations are success */
                    if (rslt == BMI090L_OK)
                    {
                        /* Restore self_test_rslt as return value */
                        rslt = self_test_rslt;
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
int8_t bmi090la_set_fifo_config(const struct bmi090l_accel_fifo_config *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store the default value of FIFO configuration
     * reserved registers
     */
    uint8_t data_array[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMI090L_OK)
    {
        /* Get the FIFO configurations from the FIFO configure_1 and configure_2 register */
        rslt = bmi090la_get_regs(BMI090L_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        if (rslt == BMI090L_OK)
        {
            /* To set the stream mode or FIFO mode */
            data_array[0] = BMI090L_SET_BITS_POS_0(data_array[0], BMI090L_ACC_FIFO_MODE_CONFIG, config->mode);

            /* To enable the Accel in FIFO configuration */
            data_array[1] = BMI090L_SET_BITS(data_array[1], BMI090L_ACCEL_EN, config->accel_en);

            /* To enable the interrupt_1 in FIFO configuration */
            data_array[1] = BMI090L_SET_BITS(data_array[1], BMI090L_ACCEL_INT1_EN, config->int1_en);

            /* To enable the interrupt_2 in FIFO configuration */
            data_array[1] = BMI090L_SET_BITS(data_array[1], BMI090L_ACCEL_INT2_EN, config->int2_en);

            rslt = bmi090la_set_regs(BMI090L_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO configuration from the sensor.
 */
int8_t bmi090la_get_fifo_config(struct bmi090l_accel_fifo_config *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI090L_OK) && (config != NULL))
    {
        /* Get the FIFO configuration value */
        rslt = bmi090la_get_regs(BMI090L_FIFO_CONFIG_0_ADDR, data, BMI090L_FIFO_CONFIG_LENGTH, dev);
        if (rslt == BMI090L_OK)
        {
            /* Get mode selection */
            config->mode = BMI090L_GET_BITS_POS_0(data[0], BMI090L_ACC_FIFO_MODE_CONFIG);

            /* Get the accel enable */
            config->accel_en = BMI090L_GET_BITS(data[1], BMI090L_ACCEL_EN);

            /* Get the interrupt_1 enable/disable */
            config->int1_en = BMI090L_GET_BITS(data[1], BMI090L_ACCEL_INT1_EN);

            /* Get the interrupt_2 enable/disable */
            config->int2_en = BMI090L_GET_BITS(data[1], BMI090L_ACCEL_INT2_EN);
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t bmi090la_read_fifo_data(struct bmi090l_fifo_frame *fifo, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO configuration data */
    uint8_t config_data = 0;

    /* Variable to define FIFO address */
    uint8_t addr = BMI090L_FIFO_DATA_ADDR;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI090L_OK) && (fifo != NULL))
    {
        /* Clear the FIFO data structure */
        reset_fifo_frame_structure(fifo);

        /* Read FIFO data */
        rslt = bmi090la_get_regs(addr, fifo->data, fifo->length, dev);
        if (rslt == BMI090L_OK)
        {
            /* Get the set FIFO frame configurations */
            rslt = bmi090la_get_regs(BMI090L_FIFO_CONFIG_1_ADDR, &config_data, 1, dev);
            if (rslt == BMI090L_OK)
            {
                /* Get sensor enable status, of which the data
                 * is to be read
                 */
                fifo->data_enable = (uint16_t)((uint16_t)config_data & BMI090L_ACCEL_EN_MASK);
            }
        }
        else
        {
            rslt = BMI090L_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t bmi090la_get_fifo_length(uint16_t *fifo_length, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO data length */
    uint8_t data[BMI090L_FIFO_DATA_LENGTH] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI090L_OK) && (fifo_length != NULL))
    {
        /* Read fifo length */
        rslt = bmi090la_get_regs(BMI090L_FIFO_LENGTH_0_ADDR, data, BMI090L_FIFO_DATA_LENGTH, dev);
        if (rslt == BMI090L_OK)
        {
            /* Get the MSB byte of FIFO length */
            data[1] = BMI090L_GET_BITS_POS_0(data[1], BMI090L_FIFO_BYTE_COUNTER_MSB);

            /* Get total FIFO length */
            (*fifo_length) = (uint16_t)((uint16_t)(data[1] << 8) | data[0]);
        }
        else
        {
            rslt = BMI090L_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t bmi090la_get_fifo_wm(uint16_t *wm, const struct bmi090l_dev *dev)
{
    int8_t rslt;

    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        rslt = bmi090la_get_regs(BMI090L_FIFO_WTM_0_ADDR, data, BMI090L_FIFO_WTM_LENGTH, dev);
        if ((rslt == BMI090L_OK) && (wm != NULL))
        {
            *wm = (data[1] << 8) | (data[0]);
        }
        else
        {
            rslt = BMI090L_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t bmi090la_set_fifo_wm(uint16_t wm, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        /* Get LSB value of FIFO water-mark */
        data[0] = BMI090L_GET_LSB(wm);

        /* Get MSB value of FIFO water-mark */
        data[1] = BMI090L_GET_MSB(wm);

        /* Set the FIFO water-mark level */
        rslt = bmi090la_set_regs(BMI090L_FIFO_WTM_0_ADDR, data, BMI090L_FIFO_WTM_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the accelerometer frames from FIFO data
 * read by the "bmi090l_read_fifo_data" API and stores it in the "accel_data"
 * structure instance.
 */
int8_t bmi090la_extract_accel(struct bmi090l_sensor_data *accel_data,
                              uint16_t *accel_length,
                              struct bmi090l_fifo_frame *fifo,
                              const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI090L_OK) && (accel_data != NULL) && (accel_length != NULL) && (fifo != NULL))
    {
        /* Parsing the FIFO data in header mode */
        rslt = extract_acc_header_mode(accel_data, accel_length, fifo);
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the down sampling rates which is configured for
 * accelerometer FIFO data.
 */
int8_t bmi090la_get_fifo_down_sample(uint8_t *fifo_downs, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI090L_OK) && (fifo_downs != NULL))
    {
        /* Read the accelerometer FIFO down data sampling rate */
        rslt = bmi090la_get_regs(BMI090L_FIFO_DOWNS_ADDR, &data, 1, dev);
        if (rslt == BMI090L_OK)
        {
            (*fifo_downs) = BMI090L_GET_BITS(data, BMI090L_ACC_FIFO_DOWNS);
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the down-sampling rates for accelerometer
 * FIFO data.
 *
 * @note Reduction of sample rate by a factor 2**fifo_downs
 */
int8_t bmi090la_set_fifo_down_sample(uint8_t fifo_downs, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        /* Set the accelerometer FIFO down sampling rate */
        rslt = bmi090la_get_regs(BMI090L_FIFO_DOWNS_ADDR, &data, 1, dev);
        if (rslt == BMI090L_OK)
        {
            data = BMI090L_SET_BITS(data, BMI090L_ACC_FIFO_DOWNS, fifo_downs);
            rslt = bmi090la_set_regs(BMI090L_FIFO_DOWNS_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi090l_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI090L_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI090L_OK;
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint16_t indx;
    uint16_t temp_len = len + dev->dummy_byte;
    uint8_t temp_buff[temp_len];
    uint8_t *dev_addr = (uint8_t *)&dev->accel_id;

    if (dev->intf == BMI090L_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = reg_addr | BMI090L_SPI_RD_MASK;
    }

    /* Read the data from the register */
    rslt = dev->read(reg_addr, temp_buff, temp_len, dev_addr);

    if (rslt == BMI090L_OK)
    {
        for (indx = 0; indx < len; indx++)
        {
            /* Updating the data buffer */
            reg_data[indx] = temp_buff[indx + dev->dummy_byte];
        }
    }
    else
    {
        /* Failure case */
        rslt = BMI090L_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API writes the data to the given register address.
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t *dev_addr = (uint8_t *)&dev->accel_id;

    if (dev->intf == BMI090L_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & BMI090L_SPI_WR_MASK);
    }

    /* Write to an accel register */
    rslt = dev->write(reg_addr, reg_data, len, dev_addr);

    if (rslt != BMI090L_OK)
    {
        /* Updating the error status */
        rslt = BMI090L_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi090l_accel_int_channel_cfg *int_config, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0, data, is_channel_invalid = FALSE;

    switch (int_config->int_channel)
    {
        case BMI090L_INT_CHANNEL_1:

            /* Update reg_addr based on channel inputs */
            reg_addr = BMI090L_ACCEL_INT1_IO_CONF_REG;
            break;
        case BMI090L_INT_CHANNEL_2:

            /* Update reg_addr based on channel inputs */
            reg_addr = BMI090L_ACCEL_INT2_IO_CONF_REG;
            break;
        default:
            is_channel_invalid = TRUE;
            break;
    }

    if (!is_channel_invalid)
    {
        /* Read interrupt pin configuration register */
        rslt = get_regs(reg_addr, &data, 1, dev);

        if (rslt == BMI090L_OK)
        {
            /* Update data with user configured bmi090l_int_cfg structure */
            data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT_LVL, int_config->int_pin_cfg.lvl);
            data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT_OD, int_config->int_pin_cfg.output_mode);

            if (int_config->int_type == BMI090L_ACCEL_SYNC_INPUT)
            {
                data = BMI090L_SET_BITS_POS_0(data, BMI090L_ACCEL_INT_EDGE, BMI090L_ENABLE);
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT_IN, int_config->int_pin_cfg.enable_int_pin);
                data = BMI090L_SET_BIT_VAL_0(data, BMI090L_ACCEL_INT_IO);
            }
            else
            {
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT_IO, int_config->int_pin_cfg.enable_int_pin);
                data = BMI090L_SET_BIT_VAL_0(data, BMI090L_ACCEL_INT_IN);
            }

            /* Write to interrupt pin configuration register */
            rslt = set_regs(reg_addr, &data, 1, dev);
        }
    }
    else
    {
        rslt = BMI090L_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static int8_t set_accel_data_ready_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                       const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == BMI090L_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT1_DRDY, conf);
                break;
            case BMI090L_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT2_DRDY, conf);
                break;
            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                            const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        data = BMI090L_ACCEL_INTA_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;
            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;
            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /* Interrupt A mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTA_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTA_ENABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /* Set input interrupt configuration */
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 */
static int8_t set_accel_sync_input(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        /* Set input interrupt configuration */
        rslt = set_int_pin_config(int_config, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 */
static int8_t set_accel_anymotion_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {
        data = BMI090L_ACCEL_INTB_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;

            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /* Interrupt B mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTB_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTB_ENABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /* Set input interrupt configuration */
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the high-g interrupt for accel sensor
 */
static int8_t set_accel_high_g_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {

        data = BMI090L_ACCEL_INTC_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;

            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /*interrupt B mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTC_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTC_ENABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the low-g interrupt for accel sensor
 */
static int8_t set_accel_low_g_int(const struct bmi090l_accel_int_channel_cfg *int_config, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {

        data = BMI090L_ACCEL_INTD_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;

            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /*interrupt B mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTD_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTD_DISABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the orientation interrupt for accel sensor
 */
static int8_t set_accel_orient_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                   const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {

        data = BMI090L_ACCEL_INTE_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;

            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /*interrupt B mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTE_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTE_ENABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the orientation interrupt for accel sensor
 */
static int8_t set_accel_no_motion_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BMI090L_OK)
    {

        data = BMI090L_ACCEL_INTF_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:
                reg_addr = BMI090L_ACCEL_INT1_MAP_REG;
                break;

            case BMI090L_INT_CHANNEL_2:
                reg_addr = BMI090L_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            rslt = get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI090L_ENABLE)
            {
                /*interrupt B mapped to INT1/INT2 */
                data |= BMI090L_ACCEL_INTF_ENABLE;
            }
            else
            {
                data &= ~BMI090L_ACCEL_INTF_ENABLE;
            }

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI090L_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API writes the config stream data in memory using burst mode.
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t indx, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t asic_msb = (uint8_t)((indx / 2) >> 4);
    uint8_t asic_lsb = ((indx / 2) & 0x0F);

    /* Write to feature config register */
    rslt = set_regs(BMI090L_ACCEL_RESERVED_5B_REG, &asic_lsb, 1, dev);
    if (rslt == BMI090L_OK)
    {
        /* Write to feature config register */
        rslt = set_regs(BMI090L_ACCEL_RESERVED_5C_REG, &asic_msb, 1, dev);

        if (rslt == BMI090L_OK)
        {
            /* Write to feature config registers */
            rslt = set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)stream_data, dev->read_write_len, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API performs the pre-requisites needed to perform the self-test
 */
static int8_t enable_self_test(struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Configuring sensors to perform accel self-test */
    dev->accel_cfg.odr = BMI090L_ACCEL_ODR_1600_HZ;
    dev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;

    /* Check the chip id of the accel variant and assign the range */

    dev->accel_cfg.range = BMI090L_ACCEL_RANGE_24G;

    dev->accel_cfg.power = BMI090L_ACCEL_PM_ACTIVE;

    /* Enable Accel sensor */
    rslt = bmi090la_set_power_mode(dev);
    if (rslt == BMI090L_OK)
    {
        /* Configure sensors with above configured settings */
        rslt = bmi090la_set_meas_conf(dev);

        if (rslt == BMI090L_OK)
        {
            /* Self-test delay */
            dev->delay_us(BMI090L_SELF_TEST_DELAY_MS * 1000, dev->intf_ptr);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the positive excitation
 */
static int8_t positive_excited_accel(struct bmi090l_sensor_data *accel_pos, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMI090L_ACCEL_POSITIVE_SELF_TEST;

    /* Enable positive excitation for all 3 axes */
    rslt = set_regs(BMI090L_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
    if (rslt == BMI090L_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_us(BMI090L_SELF_TEST_DATA_READ_MS * 1000, dev->intf_ptr);
        rslt = bmi090la_get_data(accel_pos, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the negative excitation
 */
static int8_t negative_excited_accel(struct bmi090l_sensor_data *accel_neg, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMI090L_ACCEL_NEGATIVE_SELF_TEST;

    /* Enable negative excitation for all 3 axes */
    rslt = set_regs(BMI090L_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
    if (rslt == BMI090L_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_us(BMI090L_SELF_TEST_DATA_READ_MS * 1000, dev->intf_ptr);
        rslt = bmi090la_get_data(accel_neg, dev);

        if (rslt == BMI090L_OK)
        {
            /* Disable self-test */
            reg_data = BMI090L_ACCEL_SWITCH_OFF_SELF_TEST;
            rslt = set_regs(BMI090L_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API validates the self-test results
 */
static int8_t validate_accel_self_test(const struct bmi090l_sensor_data *accel_pos,
                                       const struct bmi090l_sensor_data *accel_neg)
{
    int8_t rslt;

    /*! Structure for difference of accel values in g */
    struct bmi090la_selftest_delta_limit accel_data_diff = { 0 };

    /*! Structure for difference of accel values in mg */
    struct bmi090la_selftest_delta_limit accel_data_diff_mg = { 0 };

    accel_data_diff.x = (BMI090L_ABS(accel_pos->x - accel_neg->x));
    accel_data_diff.y = (BMI090L_ABS(accel_pos->y - accel_neg->y));
    accel_data_diff.z = (BMI090L_ABS(accel_pos->z - accel_neg->z));

    /*! Converting LSB of the differences of
     * accel values to mg */
    convert_lsb_g(&accel_data_diff, &accel_data_diff_mg);

    /* Validating accel data by comparing with minimum value of the axes in mg */
    /* x axis limit 1000mg, y axis limit 1000mg and z axis limit 500mg */
    if (accel_data_diff_mg.x >= BMI090L_ST_ACC_X_AXIS_SIGNAL_DIFF &&
        accel_data_diff_mg.y >= BMI090L_ST_ACC_Y_AXIS_SIGNAL_DIFF &&
        accel_data_diff_mg.z >= BMI090L_ST_ACC_Z_AXIS_SIGNAL_DIFF)
    {
        /* Updating Okay status */
        rslt = BMI090L_OK;
    }
    else
    {
        /* Updating Error status */
        rslt = BMI090L_W_SELF_TEST_FAIL;
    }

    return rslt;
}

/*!
 *  @brief This API converts lsb value of axes to mg for self-test.
 */
static void convert_lsb_g(const struct bmi090la_selftest_delta_limit *accel_data_diff,
                          struct bmi090la_selftest_delta_limit *accel_data_diff_mg)
{
    /* Accel x value in mg */
    accel_data_diff_mg->x = (int16_t) ((accel_data_diff->x / (int32_t)LSB_PER_G) * 1000);

    /* Accel y value in mg */
    accel_data_diff_mg->y = (int16_t) ((accel_data_diff->y / (int32_t)LSB_PER_G) * 1000);

    /* Accel z value in mg */
    accel_data_diff_mg->z = (int16_t) ((accel_data_diff->z / (int32_t)LSB_PER_G) * 1000);
}

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 */
static int8_t unpack_skipped_frame(uint16_t *data_indx, struct bmi090l_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI090L_OK;

    /* Validate data indx */
    if ((*data_indx) >= fifo->length)
    {
        /* Update the data indx to the last byte */
        (*data_indx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI090L_W_FIFO_EMPTY;
    }
    else
    {
        /* Update skipped frame count in the FIFO structure */
        fifo->skipped_frame_count = fifo->data[(*data_indx)];

        /* Move the data indx by 1 byte */
        (*data_indx) = (*data_indx) + 1;

        /* More frames could be read */
        rslt = BMI090L_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to reset the FIFO related configurations in
 * the FIFO frame structure for the next FIFO read.
 */
static void reset_fifo_frame_structure(struct bmi090l_fifo_frame *fifo)
{
    /* Reset FIFO data structure */
    fifo->acc_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
}

/*!
 * @brief This internal API is used to parse accelerometer data from the
 * FIFO data.
 */
static void unpack_accel_data(struct bmi090l_sensor_data *acc,
                              uint16_t data_start_indx,
                              const struct bmi090l_fifo_frame *fifo)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    /* Accelerometer raw x data */
    data_lsb = fifo->data[data_start_indx++];
    data_msb = fifo->data[data_start_indx++];
    acc->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw y data */
    data_lsb = fifo->data[data_start_indx++];
    data_msb = fifo->data[data_start_indx++];
    acc->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw z data */
    data_lsb = fifo->data[data_start_indx++];
    data_msb = fifo->data[data_start_indx++];
    acc->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in header mode. It updates the current data
 * byte to be parsed.
 */
static int8_t unpack_accel_frame(struct bmi090l_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct bmi090l_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI090L_OK;

    switch (frame)
    {
        /* If frame contains only accelerometer data */
        case BMI090L_FIFO_HEADER_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_frm_len) > fifo->length)
            {
                /* Update the data indx as complete */
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI090L_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], *idx, fifo);

            /* Update data indx */
            (*idx) = (*idx) + BMI090L_FIFO_ACCEL_LENGTH;

            /* Update accelerometer frame indx */
            (*acc_idx)++;
            break;
        default:

            /* Move the data indx to the last byte in case of invalid values */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = BMI090L_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to move the data indx ahead of the
 * current_frame_length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 */
static int8_t move_next_frame(uint16_t *data_indx, uint8_t current_frame_length, const struct bmi090l_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI090L_OK;

    /* Validate data indx */
    if (((*data_indx) + current_frame_length) > fifo->length)
    {
        /* Move the data indx to the last byte */
        (*data_indx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI090L_W_FIFO_EMPTY;
    }
    else
    {
        /* Move the data indx to next frame */
        (*data_indx) = (*data_indx) + current_frame_length;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 */
static int8_t unpack_sensortime_frame(uint16_t *data_indx, struct bmi090l_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI090L_OK;

    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Validate data indx */
    if (((*data_indx) + BMI090L_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Move the data indx to the last byte */
        (*data_indx) = fifo->length;

        /* FIFO is empty */
        rslt = BMI090L_W_FIFO_EMPTY;
    }
    else
    {
        /* Get sensor time from the FIFO data */
        sensor_time_byte3 = fifo->data[(*data_indx) + BMI090L_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_indx) + BMI090L_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_indx)];

        /* Update sensor time in the FIFO structure */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

        /* Move the data indx by 3 bytes */
        (*data_indx) = (*data_indx) + BMI090L_SENSOR_TIME_LENGTH;

    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO in header mode.
 */
static int8_t extract_acc_header_mode(struct bmi090l_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct bmi090l_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI090L_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to indx the data bytes */
    uint16_t data_indx;

    /* Variable to indx accelerometer frames */
    uint16_t accel_indx = 0;

    /* Variable to indicate accelerometer frames read */
    uint16_t frame_to_read = *accel_length;

    for (data_indx = fifo->acc_byte_start_idx; data_indx < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_indx];

        /* indx shifted to next byte where data starts */
        data_indx++;
        switch (frame_header)
        {
            /* If header defines accelerometer frame */
            case BMI090L_FIFO_HEADER_ACC_FRM:
            case BMI090L_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_accel_frame(acc, &data_indx, &accel_indx, frame_header, fifo);
                break;

            /* If header defines sensor time frame */
            case BMI090L_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_indx, fifo);
                break;

            /* If header defines skip frame */
            case BMI090L_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_indx, fifo);
                break;

            /* If header defines Input configuration frame */
            case BMI090L_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_indx, BMI090L_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines sample drop frame */
            case BMI090L_FIFO_SAMPLE_DROP_FRM:
                rslt = move_next_frame(&data_indx, BMI090L_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case BMI090L_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data indx to the last byte to mark completion */
                data_indx = fifo->length;

                /* FIFO is empty */
                rslt = BMI090L_W_FIFO_EMPTY;
                break;
            default:

                /* Move the data indx to the last byte in case of invalid values */
                data_indx = fifo->length;

                /* FIFO is empty */
                rslt = BMI090L_W_FIFO_EMPTY;
                break;
        }

        /* Break if Number of frames to be read is complete or FIFO is empty */
        if ((frame_to_read == accel_indx) || (rslt == BMI090L_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the accelerometer frame indx */
    (*accel_length) = accel_indx;

    /* Update the accelerometer byte indx */
    fifo->acc_byte_start_idx = data_indx;

    return rslt;
}

/*!
 * @brief This API sets the FIFO water mark interrupt for accel sensor.
 */
static int8_t set_accel_fifo_wm_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                    const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == BMI090L_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT1_FWM, conf);
                break;

            case BMI090L_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT2_FWM, conf);
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor.
 */
static int8_t set_accel_fifo_full_int(const struct bmi090l_accel_int_channel_cfg *int_config,
                                      const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == BMI090L_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI090L_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI090L_SET_BITS_POS_0(data, BMI090L_ACCEL_INT1_FFULL, conf);
                break;

            case BMI090L_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI090L_SET_BITS(data, BMI090L_ACCEL_INT2_FFULL, conf);
                break;

            default:
                rslt = BMI090L_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI090L_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI090L_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(BMI090L_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*! @endcond */

/*!
 *  @brief This API is used to enable/disable and configure the data synchronization
 *  feature.
 */
int8_t bmi090la_configure_data_synchronization(struct bmi090l_data_sync_cfg sync_cfg, struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI090L_ACCEL_DATA_SYNC_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        /* Change sensor meas config */
        switch (sync_cfg.mode)
        {
            case BMI090L_ACCEL_DATA_SYNC_MODE_2000HZ:
                dev->accel_cfg.odr = BMI090L_ACCEL_ODR_1600_HZ;
                dev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI090L_GYRO_BW_230_ODR_2000_HZ;
                dev->gyro_cfg.bw = BMI090L_GYRO_BW_230_ODR_2000_HZ;
                break;
            case BMI090L_ACCEL_DATA_SYNC_MODE_1000HZ:
                dev->accel_cfg.odr = BMI090L_ACCEL_ODR_800_HZ;
                dev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI090L_GYRO_BW_116_ODR_1000_HZ;
                dev->gyro_cfg.bw = BMI090L_GYRO_BW_116_ODR_1000_HZ;
                break;
            case BMI090L_ACCEL_DATA_SYNC_MODE_400HZ:
                dev->accel_cfg.odr = BMI090L_ACCEL_ODR_400_HZ;
                dev->accel_cfg.bw = BMI090L_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI090L_GYRO_BW_47_ODR_400_HZ;
                dev->gyro_cfg.bw = BMI090L_GYRO_BW_47_ODR_400_HZ;
                break;
            default:
                break;
        }

        rslt = bmi090la_set_meas_conf(dev);
        if (rslt != BMI090L_OK)
        {
            return rslt;
        }

        rslt = bmi090lg_set_meas_conf(dev);
        if (rslt != BMI090L_OK)
        {
            return rslt;
        }

        /* Enable data synchronization */
        data[0] = (sync_cfg.mode & BMI090L_ACCEL_DATA_SYNC_MODE_MASK);
        rslt = bmi090la_write_feature_config(BMI090L_ACCEL_DATA_SYNC_ADR, &data[0], BMI090L_ACCEL_DATA_SYNC_LEN, dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 */
int8_t bmi090la_configure_anymotion(struct bmi090l_anymotion_cfg anymotion_cfg, const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI090L_ACCEL_ANYMOTION_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI090L_OK)
    {
        /* Enable data synchronization */
        data[0] = (anymotion_cfg.threshold & BMI090L_ACCEL_ANYMOTION_THRESHOLD_MASK);
        data[0] |=
            ((anymotion_cfg.nomotion_sel << BMI090L_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT) &
             BMI090L_ACCEL_ANYMOTION_NOMOTION_SEL_MASK);
        data[1] = (anymotion_cfg.duration & BMI090L_ACCEL_ANYMOTION_DURATION_MASK);
        data[1] |= ((anymotion_cfg.x_en << BMI090L_ACCEL_ANYMOTION_X_EN_SHIFT) & BMI090L_ACCEL_ANYMOTION_X_EN_MASK);
        data[1] |= ((anymotion_cfg.y_en << BMI090L_ACCEL_ANYMOTION_Y_EN_SHIFT) & BMI090L_ACCEL_ANYMOTION_Y_EN_MASK);
        data[1] |= ((anymotion_cfg.z_en << BMI090L_ACCEL_ANYMOTION_Z_EN_SHIFT) & BMI090L_ACCEL_ANYMOTION_Z_EN_MASK);
        rslt = bmi090la_write_feature_config(BMI090L_ACCEL_ANYMOTION_ADR, &data[0], BMI090L_ACCEL_ANYMOTION_LEN, dev);
    }

    return rslt;
}

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi090l_sensor_data structure instance
 *  passed by the user.
 */
int8_t bmi090la_get_synchronized_data(struct bmi090l_sensor_data *accel,
                                      struct bmi090l_sensor_data *gyro,
                                      const struct bmi090l_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr, data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI090L_OK) && (accel != NULL) && (gyro != NULL))
    {
        /* Read accel x,y sensor data */
        reg_addr = BMI090L_ACCEL_GP_0_REG;
        rslt = bmi090la_get_regs(reg_addr, &data[0], 4, dev);

        if (rslt == BMI090L_OK)
        {
            /* Read accel sensor data */
            reg_addr = BMI090L_ACCEL_GP_4_REG;
            rslt = bmi090la_get_regs(reg_addr, &data[4], 2, dev);

            if (rslt == BMI090L_OK)
            {
                lsb = data[0];
                msb = data[1];
                msblsb = (msb << 8) | lsb;
                accel->x = ((int16_t) msblsb); /* Data in X axis */

                lsb = data[2];
                msb = data[3];
                msblsb = (msb << 8) | lsb;
                accel->y = ((int16_t) msblsb); /* Data in Y axis */

                lsb = data[4];
                msb = data[5];
                msblsb = (msb << 8) | lsb;
                accel->z = ((int16_t) msblsb); /* Data in Z axis */

                /* Read gyro sensor data */
                rslt = bmi090lg_get_data(gyro, dev);
            }
        }
    }
    else
    {
        rslt = BMI090L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi090l_int_cfg
 *  structure instance.
 */
int8_t bmi090la_set_data_sync_int_config(const struct bmi090l_int_cfg *int_config, const struct bmi090l_dev *dev)
{
    int8_t rslt;

    /* Configure accel sync data ready interrupt configuration */
    rslt = bmi090la_set_int_config(&int_config->accel_int_config_1, dev);
    if (rslt != BMI090L_OK)
    {
        return rslt;
    }

    rslt = bmi090la_set_int_config(&int_config->accel_int_config_2, dev);
    if (rslt != BMI090L_OK)
    {
        return rslt;
    }

    /* Configure gyro data ready interrupt configuration */
    rslt = bmi090lg_set_int_config(&int_config->gyro_int_config_1, dev);
    if (rslt != BMI090L_OK)
    {
        return rslt;
    }

    rslt = bmi090lg_set_int_config(&int_config->gyro_int_config_2, dev);

    return rslt;
}

/*!
 * @brief This API sets high-g configurations like threshold,
 * hysteresis, duration, and output configuration.
 */
int8_t bmi090la_set_high_g_config(const struct bmi090l_high_g_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_HIGH_G_START_ADR + 3] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        uint8_t idx = BMI090L_HIGH_G_START_ADR;

        /* Set threshold */
        feature_config[idx] = BMI090L_SET_BITS_POS_0(feature_config[idx], BMI090L_HIGH_G_THRES, config->threshold);

        /* Set hysteresis */
        feature_config[idx + 1] = BMI090L_SET_BITS_POS_0(feature_config[idx + 1],
                                                         BMI090L_HIGH_G_HYST,
                                                         config->hysteresis);

        /* Set x-select */
        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_X_SEL, config->select_x);

        /* Set y-select */
        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_Y_SEL, config->select_y);

        /* Set z-select */
        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_Z_SEL, config->select_z);

        /* High-g enable */
        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_ENABLE, config->enable);

        /* Set duration */
        feature_config[idx + 2] = BMI090L_SET_BITS_POS_0(feature_config[idx + 2], BMI090L_HIGH_G_DUR, config->duration);

        /* Set output configuration */
        feature_config[idx + 2] = BMI090L_SET_BITS(feature_config[idx + 2], BMI090L_HIGH_G_OUT_CONF, config->out_conf);

        rslt = bmi090la_set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t*) feature_config, 32, dev);
    }

    return rslt;
}

/*!
 * @brief This API gets high-g configurations like threshold,
 * hysteresis, duration, and output configuration.
 */
int8_t bmi090la_get_high_g_config(struct bmi090l_high_g_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_HIGH_G_START_ADR + 3] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for high-g select */
        uint8_t idx = BMI090L_HIGH_G_START_ADR;

        /* Get threshold */
        config->threshold = BMI090L_GET_BITS_POS_0(feature_config[idx], BMI090L_HIGH_G_THRES);

        /* Get hysteresis */
        config->hysteresis = BMI090L_GET_BITS_POS_0(feature_config[idx + 1], BMI090L_HIGH_G_HYST);

        /* Get x_select */
        config->select_x = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_X_SEL);

        /* Get y_select */
        config->select_y = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_Y_SEL);

        /* Get z_select */
        config->select_z = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_Z_SEL);

        /* Get high-g enable */
        config->enable = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_HIGH_G_ENABLE);

        /* Get duration */
        config->duration = BMI090L_GET_BITS_POS_0(feature_config[idx + 2], BMI090L_HIGH_G_DUR);

        /* Get output configuration */
        config->out_conf = BMI090L_GET_BITS_POS_0(feature_config[idx + 2], BMI090L_HIGH_G_OUT_CONF);
    }

    return rslt;
}

/*!
 * @brief This API sets high-g configurations like threshold,
 * hysteresis, duration, and output configuration.
 */
int8_t bmi090la_set_low_g_config(const struct bmi090l_low_g_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_LOW_G_START_ADR + 3] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        uint8_t idx = BMI090L_LOW_G_START_ADR;

        /* Set threshold */
        feature_config[idx] = BMI090L_SET_BITS_POS_0(feature_config[idx], BMI090L_LOW_G_THRES, config->threshold);

        /* Set hysteresis */
        feature_config[idx +
                       1] = BMI090L_SET_BITS_POS_0(feature_config[idx + 1], BMI090L_LOW_G_HYST, config->hysteresis);

        /* Low-g enable */
        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_LOW_G_ENABLE, config->enable);

        /* Set duration */
        feature_config[idx + 2] = BMI090L_SET_BITS_POS_0(feature_config[idx + 2], BMI090L_LOW_G_DUR, config->duration);

        rslt = bmi090la_set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t*) feature_config, sizeof(feature_config), dev);
    }

    return rslt;
}

/*!
 * @brief This API gets low-g configurations like threshold,
 * hysteresis and duration
 */
int8_t bmi090la_get_low_g_config(struct bmi090l_low_g_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_LOW_G_START_ADR + 3] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for low-g select */
        uint8_t idx = BMI090L_LOW_G_START_ADR;

        /* Get threshold */
        config->threshold = BMI090L_GET_BITS_POS_0(feature_config[idx], BMI090L_LOW_G_THRES);

        /* Get hysteresis */
        config->hysteresis = BMI090L_GET_BITS_POS_0(feature_config[idx + 1], BMI090L_LOW_G_HYST);

        /* Get high-g enable */
        config->enable = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_LOW_G_ENABLE);

        /* Get duration */
        config->duration = BMI090L_GET_BITS_POS_0(feature_config[idx + 2], BMI090L_LOW_G_DUR);

    }

    return rslt;
}

/*!
 * @brief This API sets orientation configurations like upside/down
 * detection, symmetrical modes, blocking mode, theta and hysteresis
 *
 */
int8_t bmi090la_set_orient_config(const struct bmi090l_orient_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_ORIENT_START_ADR + 2] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for orient select */
        uint8_t idx = BMI090L_ORIENT_START_ADR;

        /* Set orientation feature - enabled/disabled*/
        feature_config[idx] = BMI090L_SET_BITS_POS_0(feature_config[idx], BMI090L_ORIENT_ENABLE, config->enable);

        /* Set upside/down detection */
        feature_config[idx] = BMI090L_SET_BITS(feature_config[idx], BMI090L_ORIENT_UP_DOWN, config->ud_en);

        /* Set symmetrical modes */
        feature_config[idx] = BMI090L_SET_BITS(feature_config[idx], BMI090L_ORIENT_SYMM_MODE, config->mode);

        /* Set blocking mode */
        feature_config[idx] = BMI090L_SET_BITS(feature_config[idx], BMI090L_ORIENT_BLOCK_MODE, config->blocking);

        /* Set theta */
        feature_config[idx] = BMI090L_SET_BITS(feature_config[idx], BMI090L_ORIENT_THETA, config->theta);

        /* Set hysteresis */
        feature_config[idx + 1] = BMI090L_SET_BITS_POS_0(feature_config[idx + 1],
                                                         BMI090L_ORIENT_HYST,
                                                         config->hysteresis);

        rslt = bmi090la_set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    }

    return rslt;
}

/*!
 * @brief This API sets orientation configurations like upside/down
 * detection, symmetrical modes, blocking mode, theta, hysteresis and output
 * configuration.
 */
int8_t bmi090la_get_orient_config(struct bmi090l_orient_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_ORIENT_START_ADR + 2] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for orient select */
        uint8_t idx = BMI090L_ORIENT_START_ADR;

        /*Get orientation feature - enabled/disabled*/
        config->enable = BMI090L_GET_BITS_POS_0(feature_config[idx], BMI090L_ORIENT_ENABLE);

        /* Get upside/down detection */
        config->ud_en = BMI090L_GET_BITS(feature_config[idx], BMI090L_ORIENT_UP_DOWN);

        /* Get symmetrical modes */
        config->mode = BMI090L_GET_BITS(feature_config[idx], BMI090L_ORIENT_SYMM_MODE);

        /* Get blocking mode */
        config->blocking = BMI090L_GET_BITS(feature_config[idx], BMI090L_ORIENT_BLOCK_MODE);

        /* Get theta */
        config->theta = BMI090L_GET_BITS(feature_config[idx], BMI090L_ORIENT_THETA);

        /* Get hysteresis */
        config->hysteresis = BMI090L_GET_BITS_POS_0(feature_config[idx + 1], BMI090L_ORIENT_HYST);
    }

    return rslt;
}

/*!
 * @brief This internal API sets no-motion configurations like axes select,
 * duration, threshold and output-configuration.
 */
int8_t bmi090la_set_no_motion_config(const struct bmi090l_no_motion_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_NO_MOTION_START_ADR + 2] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for orient select */
        uint8_t idx = BMI090L_NO_MOTION_START_ADR;

        feature_config[idx] =
            BMI090L_SET_BITS_POS_0(feature_config[idx], BMI090L_NO_MOTION_THRESHOLD, config->threshold);

        feature_config[idx] = BMI090L_SET_BITS(feature_config[idx], BMI090L_NO_MOTION_SEL, config->enable);

        feature_config[idx + 1] = BMI090L_SET_BITS_POS_0(feature_config[idx + 1],
                                                         BMI090L_NO_MOTION_DURATION,
                                                         config->duration);

        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_X_EN, config->select_x);

        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_Y_EN, config->select_y);

        feature_config[idx + 1] = BMI090L_SET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_Z_EN, config->select_z);

        rslt = bmi090la_set_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);
    }

    return rslt;
}

int8_t bmi090la_get_no_motion_config(struct bmi090l_no_motion_cfg *config, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to define the feature configuration */
    uint16_t feature_config[BMI090L_NO_MOTION_START_ADR + 2] = { 0 };

    rslt = bmi090la_get_regs(BMI090L_ACCEL_FEATURE_CFG_REG, (uint8_t *)feature_config, sizeof(feature_config), dev);

    if (rslt == BMI090L_OK)
    {
        /* Define the offset in bytes for orient select */
        uint8_t idx = BMI090L_NO_MOTION_START_ADR;

        config->threshold = BMI090L_GET_BITS_POS_0(feature_config[idx], BMI090L_NO_MOTION_THRESHOLD);

        config->enable = BMI090L_GET_BITS(feature_config[idx], BMI090L_NO_MOTION_SEL);

        config->duration = BMI090L_GET_BITS_POS_0(feature_config[idx + 1], BMI090L_NO_MOTION_DURATION);

        config->select_x = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_X_EN);

        config->select_y = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_Y_EN);

        config->select_z = BMI090L_GET_BITS(feature_config[idx + 1], BMI090L_NO_MOTION_Z_EN);

    }

    return rslt;
}

/*!
 * @brief This internal API gets the output values of orientation: portrait-
 * landscape and face up-down.
 */
int8_t bmi090la_get_orient_output(struct bmi090l_orient_out *orient_out, const struct bmi090l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;
    uint8_t data;

    /* Search for orientation output feature and extract its configuration details */
    rslt = bmi090la_get_regs(BMI090L_ORIENT_RES_REG, &data, 1, dev);
    if (rslt == BMI090L_OK)
    {
        orient_out->portrait_landscape = BMI090L_GET_BITS_POS_0(data, BMI090L_ORIENT_PORTRAIT_LANDSCAPE);
        orient_out->faceup_down = BMI090L_GET_BITS(data, BMI090L_ORIENT_FACEUP_DOWN);
    }

    return rslt;
}

/** @}*/
