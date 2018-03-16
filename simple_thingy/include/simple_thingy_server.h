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

#ifndef SIMPLE_THINGY_SERVER_H__
#define SIMPLE_THINGY_SERVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"
#include "ble_uis.h"
#include "simple_thingy_common.h"

/**
 * @defgroup SIMPLE_THINGY_SERVER Simple OnOff Server
 * @ingroup SIMPLE_THINGY_MODEL
 * This module implements a simple proprietary Simple OnOff Server.
 * @{
 */

/** Simple Thingy Server model ID. */
#define SIMPLE_THINGY_SERVER_MODEL_ID (0x0002)



/** Forward declaration. */
typedef struct __simple_thingy_server simple_thingy_server_t;

/**
 * Get callback type.
 * @param[in] p_self Pointer to the Simple Thingy Server context structure.
 * @returns current led configuration on the Thingy node
 */
typedef ble_uis_led_t (*simple_thingy_led_get_cb_t)(const simple_thingy_server_t * p_self);

/**
 * Let set callback type.
 * @param[in] p_self Pointer to the Simple Thingy Server context structure.
 * @param[in] led_state Desired state on Thingy node
 * @returns current led configuration on the Thingy node
 */
typedef ble_uis_led_t (*simple_thingy_led_set_cb_t)(const simple_thingy_server_t * p_self, ble_uis_led_t led_state);


/**
 * Sensor set callback type.
 * @param[in] p_self Pointer to the Simple Thingy Server context structure.
 * @param[in] sensor_cfg Desired state on Thingy node
 * @returns 
 */
typedef void (*simple_thingy_sensor_config_set_cb_t)(const simple_thingy_server_t * p_self, sensor_config_t sensor_cfg);


/** Simple Thingy Server state structure. */
struct __simple_thingy_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;
    /** Get LED status callback. */
    simple_thingy_led_get_cb_t led_get_cb;
    /** Set LED status callback. */
    simple_thingy_led_set_cb_t led_set_cb;
    /** Set sensor config callback. */
    simple_thingy_sensor_config_set_cb_t sensor_set_cb;

    ble_uis_led_t present_led_status;
};

/**
 * Initializes the Simple Thingy server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in] p_server      Simple Thingy Server structure pointer.
 * @param[in] element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added server.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t simple_thingy_server_init(simple_thingy_server_t * p_server, uint16_t element_index);


/**
 * Send Thingy sensor information to the client
 * @param[in] p_server      Simple Thingy Server structure pointer.
 * @param[in] sensor_data   Sensor data structure which contains the data need to send to client.
 */
uint32_t simple_thingy_sensor_report(simple_thingy_server_t * p_server, sensor_reading_t sensor_data);

/** @} end of SIMPLE_THINGY_SERVER */

#endif /* SIMPLE_THINGY_SERVER_H__ */
