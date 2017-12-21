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

#include "simple_thingy_server.h"
#include "simple_thingy_common.h"

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "nrf_mesh_assert.h"

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_led_status(const simple_thingy_server_t * p_server,
                         const access_message_rx_t * p_message,
                         ble_uis_led_t led_status)
{
    simple_thingy_msg_led_status_t status;
    memcpy(&status.present_led_state, &led_status, sizeof(led_status));

    access_message_tx_t reply;
    reply.opcode.opcode = SIMPLE_THINGY_OPCODE_LED_STATUS;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reply.p_buffer = (const uint8_t *) &status;
    reply.length = sizeof(status);

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

static void reply_sensor_config_status(const simple_thingy_server_t * p_server,
                         const access_message_rx_t * p_message,
			 sensor_config_t present_sensor_cfg)
{
    simple_thingy_msg_sensor_config_t sensor_cfg;
    memcpy(&sensor_cfg.sensor_config, &present_sensor_cfg, sizeof(present_sensor_cfg));
    access_message_tx_t reply;
    reply.opcode.opcode = SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reply.p_buffer = (const uint8_t *) &sensor_cfg;
    reply.length = sizeof(sensor_cfg);

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void publish_led_state(simple_thingy_server_t * p_server, ble_uis_led_t led_status)
{
    simple_thingy_msg_led_status_t status;
    memcpy(&status.present_led_state, &led_status, sizeof(led_status));

    access_message_tx_t msg;
    msg.opcode.opcode = SIMPLE_THINGY_OPCODE_LED_STATUS;
    msg.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    msg.p_buffer = (const uint8_t *) &status;
    msg.length = sizeof(status);
    (void) access_model_publish(p_server->model_handle, &msg);
}

static void handle_led_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->led_set_cb != NULL);

    ble_uis_led_t led_status = (((simple_thingy_msg_led_set_t*) p_message->p_data)->led_state);
    led_status = p_server->led_set_cb(p_server, led_status);
    reply_led_status(p_server, p_message, led_status);
}

static void handle_led_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->led_get_cb != NULL);
    reply_led_status(p_server, p_message, p_server->led_get_cb(p_server));
}

static void handle_led_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->led_set_cb != NULL);
    ble_uis_led_t led_status = (((simple_thingy_msg_led_set_t*) p_message->p_data)->led_state);
    led_status = p_server->led_set_cb(p_server, led_status);
    //publish_led_state(p_server, led_status);
}

#include "log.h"
static void handle_sensor_config_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_thingy_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->sensor_set_cb != NULL);

    sensor_config_t sensor_cfg = (((simple_thingy_msg_sensor_config_t*) p_message->p_data)->sensor_config);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sensor cfg = %d\n", sensor_cfg.report_timer);
    p_server->sensor_set_cb(p_server, sensor_cfg);
    reply_sensor_config_status(p_server, p_message, sensor_cfg);
    //publish_state(p_server, value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_THINGY_OPCODE_LED_SET,            ACCESS_COMPANY_ID_NORDIC), handle_led_set_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_THINGY_OPCODE_LED_GET,            ACCESS_COMPANY_ID_NORDIC), handle_led_get_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_THINGY_OPCODE_LED_SET_UNRELIABLE, ACCESS_COMPANY_ID_NORDIC), handle_led_set_unreliable_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_THINGY_OPCODE_SENSOR_CONFIG_SET,  ACCESS_COMPANY_ID_NORDIC), handle_sensor_config_set_cb}
};

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_thingy_server_init(simple_thingy_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL ||
        p_server->led_get_cb == NULL ||
        p_server->led_set_cb == NULL ||
	p_server->sensor_set_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
    init_params.model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = NULL;
    return access_model_add(&init_params, &p_server->model_handle);
}

uint32_t simple_thingy_sensor_report(simple_thingy_server_t * p_server, sensor_reading_t sensor_data)
{
    access_message_tx_t msg;
    msg.opcode.opcode = SIMPLE_THINGY_OPCODE_SENSOR_STATUS;
    msg.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    msg.p_buffer = (const uint8_t *) & sensor_data;
    msg.length = sizeof(sensor_reading_t);
    return access_model_publish(p_server->model_handle, &msg);
}
