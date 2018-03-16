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

#ifndef SIMPLE_THINGY_CLIENT_H__
#define SIMPLE_THINGY_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "simple_thingy_common.h"

/**
 * @defgroup SIMPLE_THINGY_CLIENT Simple Thingy Client
 * @ingroup SIMPLE_THINGY_MODEL
 * This module implements a simple proprietary Simple Thingy Client.
 *
 * @{
 */


/** Simple OnOff Client model ID. */
#define SIMPLE_THINGY_CLIENT_MODEL_ID (0x0003)


/** Forward declaration. */
typedef struct __simple_thingy_client simple_thingy_client_t;

/**
 * Simple Thingy LED status callback type.
 *
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 * @param[in] status The received status of the remote server.
 * @param[in] src    Element address of the remote server.
 */
typedef void (*simple_thingy_led_status_cb_t)(const simple_thingy_client_t * p_self, ble_uis_led_t led_state, uint16_t src);
typedef void (*simple_thingy_sensor_status_cb_t)(const simple_thingy_client_t * p_self, sensor_reading_t sensor_info, uint16_t src);

/** Simple OnOff Client state structure. */
struct __simple_thingy_client
{
    /** Model handle assigned to the client. */
    access_model_handle_t model_handle;
    /** Status callback called after status received from server. */
    simple_thingy_led_status_cb_t    led_status_cb;
    simple_thingy_sensor_status_cb_t sensor_status_cb;
    /** Internal client state. */
    struct
    {
        bool led_set_reliable_transfer_active; /**< Variable used to determine if a transfer for LED set is currently active. */
        bool sensor_config_reliable_transfer_active; /**< Variable used to determine if a transfer for sensor config is currently active. */
        simple_thingy_msg_led_set_t data;  /**< Variable reflecting the data stored in the server. */
    } state;
};

/**
 * Initializes the Simple Thingy client.
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in] p_client      Simple Thingy Client structure pointer.
 * @param[in] element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added client.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t simple_thingy_client_init(simple_thingy_client_t * p_client, uint16_t element_index);

/**
 * Sets the LED state of the Simple Thingy server.
 *
 * @param[in] p_client Simple Thingy Client structure pointer.
 * @param[in] led_status   Value to set the Simple Thingy Server LED state to.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t simple_thingy_client_led_set(simple_thingy_client_t * p_client, ble_uis_led_t led_status);

/**
 * Sets the LED state of the Simple OnOff Server unreliably (without acknowledgment).
 *
 * @param[in] p_client Simple Thingy Client structure pointer.
 * @param[in] on_off   Value to set the Simple Thingy Server LED state to.
 * @param[in] repeats  Number of messages to send in a single burst. Increasing the number may
 *                     increase probability of successful delivery.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t simple_thingy_client_led_set_unreliable(simple_thingy_client_t * p_client, ble_uis_led_t led_status, uint8_t repeats);

/**
 * Sets the sensor config of the Simple Thingy server.
 *
 * @param[in] p_client Simple Thingy Client structure pointer.
 * @param[in] sensor_config The configuration setting for the sensor on Thingy server
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */

uint32_t simple_thingy_client_sensor_set(simple_thingy_client_t * p_client, sensor_config_t sensor_config);

/** @} end of SIMPLE_THINGY_CLIENT */

#endif /* SIMPLE_THINGY_CLIENT_H__ */
