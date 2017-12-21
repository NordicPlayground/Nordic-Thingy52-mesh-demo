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

#include "simple_thingy_client.h"
#include "simple_thingy_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "device_state_manager.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "log.h"


/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void sensor_config_reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    simple_thingy_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->sensor_status_cb != NULL);

    p_client->state.sensor_config_reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT,"Sensor config transmit success\r\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT,"Sensor config transmit timeout\r\n");
            break;

        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void led_set_reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    simple_thingy_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->led_status_cb != NULL);

    p_client->state.led_set_reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT,"Led set transmit success\r\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT,"Led set transmit timeout\r\n");
            break;

        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}
/** Returns @c true if the message received was from the address corresponding to the clients
 * publish address. */
static bool is_valid_source(const simple_thingy_client_t * p_client,
                            const access_message_rx_t * p_message)
{
    /* Check the originator of the status. */
    dsm_handle_t publish_handle;
    nrf_mesh_address_t publish_address;
    if (access_model_publish_address_get(p_client->model_handle, &publish_handle) != NRF_SUCCESS ||
        publish_handle == DSM_HANDLE_INVALID ||
        dsm_address_get(publish_handle, &publish_address) != NRF_SUCCESS ||
        publish_address.value != p_message->meta_data.src.value)
    {
        return false;
    }
    else
    {
        return true;
    }
}

static uint32_t send_reliable_message(const simple_thingy_client_t * p_client,
                                      simple_thingy_opcode_t opcode_transmit,
                                      simple_thingy_opcode_t opcode_reply,
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_client->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = opcode_transmit;
    reliable.message.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reliable.reply_opcode.opcode = opcode_reply;
    reliable.reply_opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reliable.timeout = ACCESS_RELIABLE_TIMEOUT_MIN;
    if(opcode_transmit == SIMPLE_THINGY_OPCODE_LED_SET)
    {
        reliable.status_cb = led_set_reliable_status_cb;
    }
    else if (opcode_transmit == SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET)
    {
        reliable.status_cb = sensor_config_reliable_status_cb;
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Opcode setting error\r\n");
    }

    return access_model_reliable_publish(&reliable);
}

/*****************************************************************************
 * Opcode handler callback(s)
 *****************************************************************************/

static void handle_sensor_config_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->sensor_status_cb != NULL);
    if (!is_valid_source(p_client, p_message))
    {
        return;
    }
}


static void handle_sensor_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->sensor_status_cb != NULL);
    if (!is_valid_source(p_client, p_message))
    {
        return;
    }

    simple_thingy_msg_sensor_reading_t * p_sensor_reading = (simple_thingy_msg_sensor_reading_t *) p_message->p_data;
    p_client->sensor_status_cb(p_client, p_sensor_reading->sensor_info, p_message->meta_data.src.value);
}

static void handle_led_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->led_status_cb != NULL);
    if (!is_valid_source(p_client, p_message))
    {
        return;
    }

    simple_thingy_msg_led_status_t * p_led_status = (simple_thingy_msg_led_status_t *) p_message->p_data;
    p_client->led_status_cb(p_client, p_led_status->present_led_state, p_message->meta_data.src.value);
}
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET, ACCESS_COMPANY_ID_NORDIC}, handle_sensor_config_cb},
    {{SIMPLE_THINGY_OPCODE_SENSOR_STATUS, ACCESS_COMPANY_ID_NORDIC}, handle_sensor_status_cb},
    {{SIMPLE_THINGY_OPCODE_LED_STATUS, ACCESS_COMPANY_ID_NORDIC}, handle_led_status_cb}
};

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_thingy_client_init(simple_thingy_client_t * p_client, uint16_t element_index)
{
    if (p_client == NULL ||
        p_client->sensor_status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.model_id.model_id = SIMPLE_THINGY_CLIENT_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
    init_params.element_index = element_index;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = NULL;
    return access_model_add(&init_params, &p_client->model_handle);
}

uint32_t simple_thingy_client_led_set(simple_thingy_client_t * p_client, ble_uis_led_t led_config)
{
    if (p_client == NULL || p_client->led_status_cb == NULL || p_client->sensor_status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.led_set_reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_client->state.data.led_state = led_config;

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_THINGY_OPCODE_LED_SET,
                                            SIMPLE_THINGY_OPCODE_LED_STATUS,
                                            (const uint8_t *)&p_client->state.data,
                                            sizeof(simple_thingy_msg_led_set_t));
    if (status == NRF_SUCCESS)
    {
        p_client->state.led_set_reliable_transfer_active = true;
    }
    return status;

}


uint32_t simple_thingy_client_sensor_set(simple_thingy_client_t * p_client, sensor_config_t sensor_config)
{
    if (p_client == NULL || p_client->led_status_cb == NULL || p_client->sensor_status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.sensor_config_reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    simple_thingy_msg_sensor_config_t msg;
    msg.sensor_config = sensor_config;

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET,
                                            SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET,
                                            (const uint8_t *)&msg,
                                            sizeof(simple_thingy_msg_sensor_config_t));
    if (status == NRF_SUCCESS)
    {
        p_client->state.sensor_config_reliable_transfer_active = true;
    }
    return status;

}

uint32_t simple_thingy_client_led_set_unreliable(simple_thingy_client_t * p_client, ble_uis_led_t led_config, uint8_t repeats)
{
    simple_thingy_msg_led_set_unreliable_t led_set_unreliable;

    led_set_unreliable.led_state = led_config;


    access_message_tx_t message;
    message.opcode.opcode = SIMPLE_THINGY_OPCODE_LED_SET_UNRELIABLE;
    message.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    message.p_buffer = (const uint8_t*) &led_set_unreliable;
    message.length = sizeof(led_set_unreliable);

    uint32_t status = NRF_SUCCESS;
    for (uint8_t i = 0; i < repeats; ++i)
    {
        status = access_model_publish(p_client->model_handle, &message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}
