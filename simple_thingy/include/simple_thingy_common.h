/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SIMPLE_THINGY_COMMON_H__
#define SIMPLE_THINGY_COMMON_H__

#include <stdint.h>
#include "ble_uis.h"

/**
 * @defgroup SIMPLE_THINGY_MODEL Simple Thingy model
 * Example model implementing basic behavior for controling LED state and getting sensor information from Thingy.
 * @ingroup md_models_README
 * @{
 * @defgroup SIMPLE_THINGY_COMMON Common Simple Thingy definitions
 * @{
 */

/*lint -align_max(push) -align_max(1) */



typedef enum
{
    SENSOR_REPORT_NONE = 0,    //turn off the sensor report
    SENSOR_REPORT_EVERY_1S,    //config the sensor send report every 1 sec
    SENSOR_REPORT_EVERY_2S,    //config the sensor send report every 2 sec
    SENSOR_REPORT_EVERY_5S,    //config the sensor send report every 5 sec
    SENSOR_REPORT_EVERY_10S,   //config the sensor send report every 10 sec
    SENSOR_REPORT_MANUAL       //config the sensor send report manually
} sensor_report_config_t;

typedef PACKED( struct
{
    float humidity;
    float temperature;
}) sensor_reading_t;


typedef PACKED( struct
{
    uint8_t report_timer;
}) sensor_config_t;


/** Simple Thingy opcodes. */
typedef enum
{
    SIMPLE_THINGY_OPCODE_LED_SET = 0xC1,            /**< Simple Thingy LED Set. */
    SIMPLE_THINGY_OPCODE_LED_GET = 0xC2,            /**< Simple Thingy LED Get. */
    SIMPLE_THINGY_OPCODE_LED_SET_UNRELIABLE = 0xC3, /**< Simple Thingy LED Set Unreliable. */
    SIMPLE_THINGY_OPCODE_LED_STATUS = 0xC4,         /**< Simple Thingy LED Status. */
    SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET = 0xC5,  /**< Simple Thingy Sensor Config Set. */
    SIMPLE_THINGY_OPCODE_SENSOR_STATUS = 0xC6       /**< Simple THingy Sensor Status. */
} simple_thingy_opcode_t;

/** Message format for the Simple Thingy LED Set message. */
typedef struct __attribute((packed))
{
    ble_uis_led_t led_state;
} simple_thingy_msg_led_set_t;

/** Message format for the Simple Thingy LED Set Unreliable message. */
typedef struct __attribute((packed))
{
    ble_uis_led_t led_state;
} simple_thingy_msg_led_set_unreliable_t;

/** Message format for the Simple Thingy LED Status message. */
typedef struct __attribute((packed))
{
    ble_uis_led_t present_led_state;
} simple_thingy_msg_led_status_t;

/** Message format for the Simple Thingy Sensor Status message. */
typedef struct __attribute((packed))
{
    sensor_reading_t sensor_info;
} simple_thingy_msg_sensor_reading_t;

/** Message format for the Simple Thingy Sensor Config message. */
typedef struct __attribute((packed))
{
    sensor_config_t sensor_config;
} simple_thingy_msg_sensor_config_t;






/*lint -align_max(pop) */

/** @} end of SIMPLE_THINGY_COMMON */
/** @} end of SIMPLE_THINGY_MODEL */
#endif /* SIMPLE_THINGY_COMMON_H__ */
