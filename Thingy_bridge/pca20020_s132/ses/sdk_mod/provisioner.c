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

#include "provisioner.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "log.h"
#include "nrf_mesh_assert.h"

#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "nrf_mesh_events.h"
#include "device_state_manager.h"

#include "config_client.h"
#include "access_config.h"
#include "simple_thingy_server.h"
#include "health_common.h"

#include "nrf_mesh_sdk.h"
#include "m_nus.h"

#define PROV_AUTO


#ifdef PROV_AUTO
bool m_auto_provisioning = true;
#else
bool m_auto_provisioning = false;
#endif



bool m_check_nearby_dev = false;

#define NEARBY_DEV_BUFFER_SIZE 10
uint8_t nearby_dev_info[NEARBY_DEV_BUFFER_SIZE][16] = {0};
uint8_t cur_index = 0;


typedef enum
{
    PROV_STATE_IDLE,
    PROV_STATE_WAIT,
    PROV_STATE_PROV,
    PROV_STATE_CONFIG_FIRST,
    PROV_STATE_CONFIG_COMPOSITION_GET,
    PROV_STATE_CONFIG_APPKEY_ADD,
    PROV_STATE_CONFIG_APPKEY_BIND_HEALTH,
    PROV_STATE_CONFIG_APPKEY_BIND_THINGY,
    PROV_STATE_CONFIG_PUBLICATION_HEALTH,
    PROV_STATE_CONFIG_PUBLICATION_THINGY,
    PROV_STATE_CONFIG_SUBSCRIPTION
} prov_state_t;

/* Provisioning encryption key storage (this is not how you should store your keys). */
static uint8_t m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];

static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;
static nrf_mesh_prov_ctx_t m_prov_ctx;

static prov_state_t m_prov_state;
static uint16_t m_target_address;

void provisioning_enable_auto(uint8_t is_enable)
{
    m_auto_provisioning = is_enable;
    if(m_auto_provisioning == true)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "m_auto_provisioning on\r\n");
    }
}

void start_scan_nearby_dev(bool is_enable)
{
    if(m_check_nearby_dev == true && is_enable == false)
    {
        m_check_nearby_dev = false;
        uint8_t i;
        for(i = 0; i < cur_index; i++){
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Device:", nearby_dev_info[i], NRF_MESH_UUID_SIZE);
            nus_response_send(NUS_RSP_SCAN_RST, i, nearby_dev_info[i], NRF_MESH_UUID_SIZE);
        }
    }
    m_check_nearby_dev = is_enable;
}

void nearby_dev_list_clean(void)
{
    uint8_t i;
    memset(&nearby_dev_info, 0, (sizeof(uint8_t)*16*NEARBY_DEV_BUFFER_SIZE));
    cur_index = 0;
}


static void start_provisioning(const uint8_t * p_uuid)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
        {
            .netkey = NETKEY,
            .netkey_index = NETKEY_INDEX,
            .iv_index = 0,
            .address = m_target_address,
            .flags.iv_update = false,
            .flags.key_refresh = false
        };
    ERROR_CHECK(nrf_mesh_prov_provision(&m_prov_ctx, p_uuid, &prov_data, NRF_MESH_PROV_BEARER_ADV));
    m_prov_state = PROV_STATE_PROV;
}

void provisioning_with_item(uint8_t dev_no)
{
    if(dev_no > cur_index){
        __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Error, out of list\n\r");
    }
    else
    {
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Going to do provisioning with ", nearby_dev_info[dev_no], NRF_MESH_UUID_SIZE);
        provisioning_with_uuid(nearby_dev_info[dev_no]);
        nearby_dev_list_clean();
        __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Clear list\r\n");
    }
}

void provisioning_with_uuid(uint8_t * device)
{
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Start provisioning for ", device, NRF_MESH_UUID_SIZE);
    start_provisioning(device);
    m_prov_state = PROV_STATE_PROV;
}

static void do_config_step(void)
{
    switch (m_prov_state)
    {
        case PROV_STATE_CONFIG_FIRST:
            m_prov_state = PROV_STATE_CONFIG_COMPOSITION_GET;
            /* fall through */

        /* Read the composition data from the node: */
        case PROV_STATE_CONFIG_COMPOSITION_GET:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Getting composition data\n");
            ERROR_CHECK(config_client_composition_data_get());
            break;
        }

        /* Add the application key to the node: */
        case PROV_STATE_CONFIG_APPKEY_ADD:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding appkey\n");
            uint8_t appkey[NRF_MESH_KEY_SIZE] = APPKEY;
            ERROR_CHECK(config_client_appkey_add(NETKEY_INDEX, APPKEY_INDEX, appkey));
            break;
        }

        /* Bind the health server to the application key: */
        case PROV_STATE_CONFIG_APPKEY_BIND_HEALTH:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Binding appkey to the Health model\n");
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NONE;
            model_id.model_id = HEALTH_SERVER_MODEL_ID;
            uint16_t element_address = m_target_address;
            ERROR_CHECK(config_client_model_app_bind(element_address, APPKEY_INDEX, model_id));
            break;
        }

        /* Bind the On/Off server to the application key: */
        case PROV_STATE_CONFIG_APPKEY_BIND_THINGY:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Binding appkey to the Simple Thingy model\n");
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
            uint16_t element_address = m_target_address;
            ERROR_CHECK(config_client_model_app_bind(element_address, APPKEY_INDEX, model_id));
            break;
        }

        /* Configure the publication parameters for the Simply Thingy server: */
        case PROV_STATE_CONFIG_PUBLICATION_HEALTH:
        {
            config_publication_state_t pubstate = {0};
            pubstate.element_address = m_target_address;
            pubstate.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            pubstate.publish_address.value = PROVISIONER_ADDRESS;
            pubstate.appkey_index = 0;
            pubstate.frendship_credential_flag = false;
            pubstate.publish_ttl = SERVER_COUNT;
            pubstate.publish_period.step_num = 1;
            pubstate.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_10MIN;
            pubstate.retransmit_count = 1;
            pubstate.retransmit_interval = 0;
            pubstate.model_id.company_id = ACCESS_COMPANY_ID_NONE;
            pubstate.model_id.model_id = HEALTH_SERVER_MODEL_ID;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address for the health server to 0x%04x\n", pubstate.publish_address.value);

            ERROR_CHECK(config_client_model_publication_set(&pubstate));
            break;
        }

        /* Configure the publication parameters for the Simple Thingy server: */
        case PROV_STATE_CONFIG_PUBLICATION_THINGY:
        {
            config_publication_state_t pubstate = {0};
            pubstate.element_address = m_target_address;
            pubstate.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            pubstate.publish_address.value = PROVISIONER_ADDRESS + m_target_address - UNPROV_START_ADDRESS;
            pubstate.appkey_index = 0;
            pubstate.frendship_credential_flag = false;
            pubstate.publish_ttl = SERVER_COUNT;
            pubstate.publish_period.step_num = 0;
            pubstate.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
            pubstate.retransmit_count = 1;
            pubstate.retransmit_interval = 0;
            pubstate.model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            pubstate.model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address for the On/Off server to 0x%04x\n", pubstate.publish_address.value);

            ERROR_CHECK(config_client_model_publication_set(&pubstate));
            break;
        }

        /* Add a subscription to the group address to the node: */
        case PROV_STATE_CONFIG_SUBSCRIPTION:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription\n");
            uint16_t element_address = m_target_address;
            nrf_mesh_address_t address = {NRF_MESH_ADDRESS_TYPE_INVALID, 0, NULL};
            address.type = NRF_MESH_ADDRESS_TYPE_GROUP;
            address.value = GROUP_ADDRESS;
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
            model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
            ERROR_CHECK(config_client_model_subscription_add(element_address, address, model_id));
            break;
        }

        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}




uint8_t nearby_dev_buffer_push(const uint8_t * beacon_info)
{
    uint8_t i;
    bool new_device = true;
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Got device:", beacon_info, NRF_MESH_UUID_SIZE);
    for(i = 0; i < NEARBY_DEV_BUFFER_SIZE; i++){
        if(strncmp(nearby_dev_info[i], beacon_info, NRF_MESH_UUID_SIZE) == 0)
        {
            new_device = false;
        }
    }

    if(new_device == true && cur_index < NEARBY_DEV_BUFFER_SIZE-1)
    {
        strcpy(nearby_dev_info[cur_index], beacon_info);
        cur_index++;
    }


    for(i = 0; i < cur_index; i++){
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "Got device:", nearby_dev_info[i], NRF_MESH_UUID_SIZE);
    }

}
static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED:
            if (m_prov_state == PROV_STATE_WAIT)
            {
                //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, " m_check_nearby_dev = %d\r\n", m_check_nearby_dev);                
                if(m_auto_provisioning == true)
                {
                    start_provisioning(p_evt->params.unprov.device_uuid);
                    m_prov_state = PROV_STATE_PROV;
                }
                else if(m_auto_provisioning == false && m_check_nearby_dev == true)
                {
                    //print nearby device
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "scan.....\r\n");
                    nearby_dev_buffer_push(p_evt->params.unprov.device_uuid);
                }
            }
            break;

        case NRF_MESH_PROV_EVT_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link closed\n");
            if (m_prov_state == PROV_STATE_PROV)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning failed. Retrying...\n");
                m_prov_state = PROV_STATE_WAIT;
            }
            else if (m_prov_state == PROV_STATE_CONFIG_FIRST)
            {
                do_config_step();
            }
            break;

        case NRF_MESH_PROV_EVT_COMPLETE:
            m_prov_state = PROV_STATE_IDLE;
            provisioner_prov_complete_cb(&p_evt->params.complete);
            break;

        case NRF_MESH_PROV_EVT_CAPS_RECEIVED:
        {
            uint32_t status = nrf_mesh_prov_oob_use(p_evt->params.oob_caps_received.p_context,
                                                    NRF_MESH_PROV_OOB_METHOD_STATIC,
                                                    0,
                                                    NRF_MESH_KEY_SIZE);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                      "Provisioning OOB selection rejected, error code %d. Retrying...\n", status);
                m_prov_state = PROV_STATE_WAIT;
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using static authentication\n");
            }
            break;
        }

        case NRF_MESH_PROV_EVT_STATIC_REQUEST:
        {
            uint8_t static_data[16] = STATIC_AUTH_DATA;
            ERROR_CHECK(nrf_mesh_prov_auth_data_provide(p_evt->params.static_request.p_context, static_data, 16));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Static authentication data provided\n");
            break;
        }

        case NRF_MESH_PROV_EVT_LINK_ESTABLISHED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link established\n");
            break;

        default:
            break;
    }
}

void provisioner_configure(uint16_t address)
{
    m_target_address = address;
    if (m_prov_state == PROV_STATE_PROV)
    {
        /* Wait for the NRF_MESH_PROV_EVT_LINK_CLOSED event. */
        m_prov_state = PROV_STATE_CONFIG_FIRST;
    }
    else
    {
        m_prov_state = PROV_STATE_CONFIG_FIRST;
        do_config_step();
    }
}

void provisioner_wait_for_unprov(uint16_t address)
{
    m_target_address = address;
    m_prov_state = PROV_STATE_WAIT;
}

void config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length)
{
    if (event_type == CONFIG_CLIENT_EVENT_TYPE_TIMEOUT)
    {
        provisioner_config_failed_cb();
        return;
    }

    NRF_MESH_ASSERT(p_event != NULL);

    if (p_event->opcode == CONFIG_OPCODE_COMPOSITION_DATA_STATUS &&
        m_prov_state == PROV_STATE_CONFIG_COMPOSITION_GET)
    {
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Composition data", ((const uint8_t *) &p_event->p_msg->composition_data_status), length);
        m_prov_state = PROV_STATE_CONFIG_APPKEY_ADD;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_APPKEY_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_APPKEY_ADD)
    {
        NRF_MESH_ASSERT(p_event->p_msg->appkey_status.status == ACCESS_STATUS_SUCCESS ||
                        p_event->p_msg->appkey_status.status == ACCESS_STATUS_KEY_INDEX_ALREADY_STORED);
        m_prov_state = PROV_STATE_CONFIG_APPKEY_BIND_HEALTH;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_APP_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_APPKEY_BIND_HEALTH)
    {
        NRF_MESH_ASSERT(p_event->p_msg->app_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_APPKEY_BIND_THINGY;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_APP_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_APPKEY_BIND_THINGY)
    {
        NRF_MESH_ASSERT(p_event->p_msg->app_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_PUBLICATION_HEALTH;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_PUBLICATION_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_PUBLICATION_HEALTH)
    {
        NRF_MESH_ASSERT(p_event->p_msg->publication_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_PUBLICATION_THINGY;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_PUBLICATION_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_PUBLICATION_THINGY)
    {
        NRF_MESH_ASSERT(p_event->p_msg->publication_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_CONFIG_SUBSCRIPTION;
        do_config_step();
    }
    else if (p_event->opcode == CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS &&
             m_prov_state == PROV_STATE_CONFIG_SUBSCRIPTION)
    {
        NRF_MESH_ASSERT(p_event->p_msg->subscription_status.status == ACCESS_STATUS_SUCCESS);
        m_prov_state = PROV_STATE_IDLE;
        provisioner_config_successful_cb();
    }
    else
    {
       if(p_event->opcode == CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS){
              __LOG(LOG_SRC_APP, LOG_LEVEL_DEFAULT, "config status = %X\r\n", p_event->p_msg->subscription_status.status);
       }
        /* Do nothing. */
    }
}

void provisioner_init(void)
{
    m_prov_state = PROV_STATE_IDLE;
    nrf_mesh_prov_oob_caps_t capabilities = NRF_MESH_PROV_OOB_CAPS_DEFAULT(ACCESS_ELEMENT_COUNT);

    ERROR_CHECK(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, m_public_key, m_private_key, &capabilities, prov_evt_handler));
    ERROR_CHECK(nrf_mesh_prov_bearer_add(&m_prov_ctx, nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
    ERROR_CHECK(nrf_mesh_prov_scan_start(prov_evt_handler));
}



