/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 *
 * @brief    Thingy application main file.
 *
 * This file contains the source code for the Thingy application that uses the Weather Station service.
 */

#include <stdint.h>
#include <float.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "m_ble.h"
#include "m_environment.h"
#include "m_sound.h"
#include "m_motion.h"
#include "m_ui_demo.h"
#include "m_batt_meas.h"
#include "drv_ext_light.h"
#include "drv_ext_gpio.h"
#include "nrf_delay.h"
#include "twi_manager.h"
#include "support_func.h"
#include "pca20020.h"
#include "app_error.h"
#include "ble_nus.h"
#include "nrf_nvmc.h"
#include "m_nus.h"
#include "nrf_drv_gpiote.h"
#include "nrf_mesh_config_core.h"
#include "hal.h"

#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "log.h"
#include "nrf_mesh_sdk.h"
#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "nrf_mesh_node_config.h"
#include "simple_thingy_example_common.h"
#include "simple_thingy_client.h"
#include "provisioner.h"
#include "health_client.h"
#include "simple_thingy_server.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#define  NRF_LOG_MODULE_NAME "main          "

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

#define CLIENT_COUNT        (SERVER_COUNT + 2)  //two group, first group for all the nodes, second group for user defined
#define GROUP_CLIENT_INDEX  (SERVER_COUNT)
#define CUSTOM_GROUP_CLIENT_INDEX  (SERVER_COUNT + 1)

static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];

static const uint8_t m_netkey[NRF_MESH_KEY_SIZE] = NETKEY;
static const uint8_t m_appkey[NRF_MESH_KEY_SIZE] = APPKEY;

static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_devkey_handles[SERVER_COUNT];
static dsm_handle_t m_server_handles[SERVER_COUNT];
static dsm_handle_t m_group_handle[2];

static simple_thingy_client_t m_clients[CLIENT_COUNT];
static health_client_t m_health_client;

static uint16_t m_provisioned_devices;
static uint16_t m_configured_devices;
static sensor_config_t m_sensor_config[CLIENT_COUNT];

APP_TIMER_DEF(m_scan_dev_timer);


/* Forward declarations */
static void client_status_cb(const simple_thingy_client_t * p_self, ble_uis_led_t led_status, uint16_t src);
static void sensor_status_cb(const simple_thingy_client_t * p_self, sensor_reading_t sensor_status, uint16_t src);
static void health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event);
/*****************************************************************************
 * Static functions
 *****************************************************************************/

/**
 * Retrieves stored device state manager configuration.
 * The number of provisioned devices is calculated from the number of device keys stored. The device
 * key for each server is stored on provisioning complete in the `provisioner_prov_complete_cb()`.
 *
 * @returns Number of provisioned devices.
 */

static uint32_t server_index_get(const simple_thingy_client_t * p_client)
{
    uint32_t index = (((uint32_t) p_client - ((uint32_t) &m_clients[0]))) / sizeof(m_clients[0]);
    NRF_MESH_ASSERT(index < SERVER_COUNT);
    return index;
}


static void sensor_status_cb(const simple_thingy_client_t * p_self, sensor_reading_t status, uint16_t src)
{
    uint8_t ret_packet[3];
    NRF_LOG_INFO("Sensor information from node address 0x%04X\r\n", src);
    NRF_LOG_INFO("Humidity:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(status.humidity));
    NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(status.temperature));
    uint32_t server_index = server_index_get(p_self);

    ret_packet[0] = (int8_t)status.temperature;
    ret_packet[1] = (uint8_t)((status.temperature - ((uint8_t)status.temperature))*100.0f);
    ret_packet[2] = status.humidity;

    nus_response_send(NUS_RSP_SENSOR_READING, server_index, ret_packet, sizeof(ret_packet));
}


static uint16_t provisioned_device_handles_load(void)
{
    uint16_t provisioned_devices = 0;

    /* Load the key handles. */
    uint32_t count = 1;
    ERROR_CHECK(dsm_subnet_get_all(&m_netkey_handle, &count));
    count = 1;
    ERROR_CHECK(dsm_appkey_get_all(m_netkey_handle, &m_appkey_handle, &count));

    /* Load all the address handles. */
    dsm_handle_t address_handles[DSM_ADDR_MAX];
    count = DSM_NONVIRTUAL_ADDR_MAX;
    ERROR_CHECK(dsm_address_get_all(&address_handles[0], &count));
    uint8_t j = 0;;
    for (uint32_t i = 0; i < count; ++i)
    {
        nrf_mesh_address_t address;
        ERROR_CHECK(dsm_address_get(address_handles[i], &address));

        /* If the address is a unicast address, it is one of the server's root element address and
         * we have should have a device key stored for it. If not, it is our GROUP_ADDRESS and we
         * load the handle for that.
         */
        if ((address.type == NRF_MESH_ADDRESS_TYPE_UNICAST) &&
            (dsm_devkey_handle_get(address.value, &m_devkey_handles[provisioned_devices]) == NRF_SUCCESS)
            && m_devkey_handles[provisioned_devices] != DSM_HANDLE_INVALID)
        {
            ERROR_CHECK(dsm_address_handle_get(&address, &m_server_handles[provisioned_devices]));
            provisioned_devices++;
        }
        else if (address.type == NRF_MESH_ADDRESS_TYPE_GROUP)
        {
            ERROR_CHECK(dsm_address_handle_get(&address, &m_group_handle[j]));
            j++;
        }
    }

    return provisioned_devices;
}

/**
 * Gets the number of configured devices.
 *
 * We exploit the fact that the publish address of the Simple Thingy clients is set at configuration
 * complete, i.e., in the `provisioner_config_successful_cb()`, and simply count the number of
 * clients with their publish address' set.
 */

void provisioner_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_configured_devices);

    /* Set publish address for the client to the corresponding server. */
    ERROR_CHECK(access_model_publish_address_set(m_clients[m_configured_devices].model_handle,
                                                 m_server_handles[m_configured_devices]));
    access_flash_config_store();
    uint32_t err_code = NRF_SUCCESS;
    nus_response_send(NUS_RSP_PROV_RST, m_configured_devices, (uint8_t * )&err_code, sizeof(uint32_t));

    m_configured_devices++;

    if (m_configured_devices < SERVER_COUNT)
    {
        provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "All servers provisioned\n");
    }
}

static uint16_t configured_devices_count_get(void)
{
    uint16_t configured_devices = 0;
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        dsm_handle_t address_handle = DSM_HANDLE_INVALID;
        if ((access_model_publish_address_get(m_clients[i].model_handle,
                                              &address_handle) == NRF_SUCCESS)
            && (DSM_HANDLE_INVALID != address_handle))
        {
            configured_devices++;
        }
        else
        {
            /* Clients are configured sequentially. */
            break;
        }
    }

    return configured_devices;
}

static void led_status_cb(const simple_thingy_client_t * p_self, ble_uis_led_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);
    uint8_t* packet = (uint8_t *)&status;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Report from server %d ,status packet = %02X %02X %02X %02X %02X, src addr = 0x%04X\n\r",
                          server_index, packet[0], packet[1], packet[2], packet[3], packet[4], src);
    nus_response_send(NUS_RSP_LED_STATE, server_index, (uint8_t *)&status, sizeof(ble_uis_led_t));
}

static void access_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting up access layer and models\n");

    dsm_init();
    access_init();

    m_netkey_handle = DSM_HANDLE_INVALID;
    m_appkey_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        m_devkey_handles[i] = DSM_HANDLE_INVALID;
        m_server_handles[i] = DSM_HANDLE_INVALID;
    }
    m_group_handle[0] = DSM_HANDLE_INVALID;
    m_group_handle[1] = DSM_HANDLE_INVALID;

    /* Initialize and enable all the models before calling ***_flash_config_load. */
    ERROR_CHECK(config_client_init(config_client_event_cb));
    ERROR_CHECK(health_client_init(&m_health_client, 0, health_event_cb));

    for (uint32_t i = 0; i < CLIENT_COUNT; ++i)
    {
        m_clients[i].led_status_cb = led_status_cb;
        m_clients[i].sensor_status_cb = sensor_status_cb;
        ERROR_CHECK(simple_thingy_client_init(&m_clients[i], i));
    }

    if (dsm_flash_config_load())
    {
        m_provisioned_devices = provisioned_device_handles_load();
    }
    else
    {
        /* Set and add local addresses and keys, if flash recovery fails. */
        dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, ACCESS_ELEMENT_COUNT};
        ERROR_CHECK(dsm_local_unicast_addresses_set(&local_address));
        ERROR_CHECK(dsm_address_publish_add(GROUP_ADDRESS, &m_group_handle[0]));
        ERROR_CHECK(dsm_address_publish_add(GROUP_ADDRESS_USER, &m_group_handle[1]));
        ERROR_CHECK(dsm_subnet_add(0, m_netkey, &m_netkey_handle));
        ERROR_CHECK(dsm_appkey_add(0, m_netkey_handle, m_appkey, &m_appkey_handle));
    }

    if (access_flash_config_load())
    {
        m_configured_devices = configured_devices_count_get();
    }
    else
    {
        /* Bind the keys to the health client. */
        ERROR_CHECK(access_model_application_bind(m_health_client.model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_health_client.model_handle, m_appkey_handle));

        /* Bind the keys to the Simple Thingy clients. */
        for (uint32_t i = 0; i < SERVER_COUNT; ++i)
        {
            ERROR_CHECK(access_model_application_bind(m_clients[i].model_handle, m_appkey_handle));
            ERROR_CHECK(access_model_publish_application_set(m_clients[i].model_handle, m_appkey_handle));
        }

        ERROR_CHECK(access_model_application_bind(m_clients[GROUP_CLIENT_INDEX].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_clients[GROUP_CLIENT_INDEX].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_address_set(m_clients[GROUP_CLIENT_INDEX].model_handle, m_group_handle[0]));

        ERROR_CHECK(access_model_application_bind(m_clients[GROUP_CLIENT_INDEX+1].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_clients[GROUP_CLIENT_INDEX+1].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_address_set(m_clients[GROUP_CLIENT_INDEX+1].model_handle, m_group_handle[1]));

        access_flash_config_store();
    }

    provisioner_init();
    if (m_configured_devices < m_provisioned_devices)
    {
        provisioner_configure(UNPROV_START_ADDRESS + m_configured_devices);
    }
    else if (m_provisioned_devices < SERVER_COUNT)
    {
        provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
    }
}

static void health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    switch (p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node 0x%04x alive with %u active fault(s), RSSI: %d\n",
                  p_event->p_meta_data->src.value, p_event->data.fault_status.fault_array_length,
                  p_event->p_meta_data->rssi);
/* the current nRFMesh APP cannot handle the health message, and may lead to APP crash            
            nus_response_send(NUS_RSP_HEALTH, 
                              p_event->p_meta_data->src.value - UNPROV_START_ADDRESS, 
                              (uint8_t * )&p_event->p_meta_data->rssi, sizeof(int8_t));
*/
            break;
        default:
            break;
    }
}

void provisioner_config_failed_cb(void)
{
    uint32_t err_code = NRF_ERROR_TIMEOUT;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed\n", m_configured_devices);
    nus_response_send(NUS_RSP_PROV_RST, m_configured_devices, (uint8_t * )&err_code, sizeof(uint32_t));
    /* Delete key and address. */
    ERROR_CHECK(dsm_address_publish_remove(m_server_handles[m_configured_devices]));
    ERROR_CHECK(dsm_devkey_delete(m_devkey_handles[m_configured_devices]));

    provisioner_wait_for_unprov(UNPROV_START_ADDRESS + m_provisioned_devices);
}

void provisioner_prov_complete_cb(const nrf_mesh_prov_evt_complete_t * p_prov_data)
{
    /* We should not get here if all servers are provisioned. */
    NRF_MESH_ASSERT(m_configured_devices < SERVER_COUNT);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning complete. Adding address 0x%04x.\n", p_prov_data->address);

    /* Add to local storage. */
    ERROR_CHECK(dsm_address_publish_add(p_prov_data->address, &m_server_handles[m_provisioned_devices]));
    ERROR_CHECK(dsm_devkey_add(p_prov_data->address, m_netkey_handle, p_prov_data->p_devkey, &m_devkey_handles[m_provisioned_devices]));

    /* Bind the device key to the configuration server and set the new node as the active server. */
    ERROR_CHECK(config_client_server_bind(m_devkey_handles[m_provisioned_devices]));
    ERROR_CHECK(config_client_server_set(m_devkey_handles[m_provisioned_devices],
                                         m_server_handles[m_provisioned_devices]));

    m_provisioned_devices++;

    /* Move on to the configuration step. */
    provisioner_configure(UNPROV_START_ADDRESS + m_configured_devices);
}



void scan_dev_timer_handler(void * p_context)
{
    start_scan_nearby_dev(false); 
}

void nus_command_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint32_t err_code;
    ble_uis_led_t led_state;
    uint16_t server_index = (p_data[0]<<8) | p_data[1];
    if(server_index == 0xffff)
    {
       server_index = 0x0A; //for legacy support
    }

    switch(p_data[2])
    {
        case NUS_CMD_AUTO_PROV:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "provisioning_enable_auto \n\r");   
            provisioning_enable_auto(p_data[3]);
            break;
        case NUS_CMD_SENSOR_SET:
            if(length == 3){  //legacy setting for nRFMesh APP
                m_sensor_config[server_index].report_timer ^= 1;
            }else{
                m_sensor_config[server_index].report_timer = p_data[3];
            }
            err_code = simple_thingy_client_sensor_set(&m_clients[server_index], m_sensor_config[server_index]);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Sensor set error code = %d\n\r", err_code);
            nus_response_send(NUS_RSP_SENSOR_SET, (uint32_t)server_index, (uint8_t * )&err_code, sizeof(err_code));
            break;
        case NUS_CMD_LED_SET:
            memcpy(&led_state, &p_data[3], length-3);
            if(server_index == 0x0A || server_index == 0x0B)
            { 
                err_code = simple_thingy_client_led_set_unreliable(&m_clients[server_index],led_state, 3);  
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Led unreliable set error code = %d\n\r", err_code);
                 nus_response_send(NUS_RSP_LED_SET, (uint32_t)server_index, (uint8_t * )&err_code, sizeof(err_code));
            }
            else
            {         
                err_code = simple_thingy_client_led_set(&m_clients[server_index],led_state); 
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Led set error code = %d\n\r", err_code);
                nus_response_send(NUS_RSP_LED_SET, (uint32_t)server_index, (uint8_t * )&err_code, sizeof(err_code));

            }
            break;
        case NUS_CMD_GROUP_ADD:
        {
             nrf_mesh_address_t device_address;
             dsm_handle_t address_handle = DSM_HANDLE_INVALID;

             access_model_publish_address_get(m_clients[server_index].model_handle, &address_handle);
             if(address_handle != DSM_HANDLE_INVALID)
             {
                 nrf_mesh_address_t device_address;
                 dsm_address_get(address_handle, &device_address);
                 config_client_server_set(m_devkey_handles[server_index], m_server_handles[server_index]);
                 nrf_mesh_address_t address;
                 address.type = NRF_MESH_ADDRESS_TYPE_GROUP;
                 address.value = GROUP_ADDRESS_USER;
                 access_model_id_t model_id;
                 model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
                 model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
                 err_code = config_client_model_subscription_add(device_address.value, address, model_id);
                 __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Group subscription add error code = %d\n\r", err_code);
             }
             break;
        }
        case NUS_CMD_GROUP_DELETE:
        {
            nrf_mesh_address_t device_address;
            dsm_handle_t address_handle = DSM_HANDLE_INVALID;

            access_model_publish_address_get(m_clients[server_index].model_handle, &address_handle);
            if(address_handle != DSM_HANDLE_INVALID)
            {
               nrf_mesh_address_t device_address;
               dsm_address_get(address_handle, &device_address);
               config_client_server_set(m_devkey_handles[server_index], m_server_handles[server_index]);
               nrf_mesh_address_t address;
               address.type = NRF_MESH_ADDRESS_TYPE_GROUP;
               address.value = GROUP_ADDRESS_USER;
               access_model_id_t model_id;
               model_id.company_id = ACCESS_COMPANY_ID_NORDIC;
               model_id.model_id = SIMPLE_THINGY_SERVER_MODEL_ID;
               err_code = config_client_model_subscription_delete(device_address.value, address, model_id);
               __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Group subscription delete error code = %d\n\r", err_code);
            }
            break;
        }
        case NUS_CMD_UNPROV_SCAN:
            app_timer_create(&m_scan_dev_timer, APP_TIMER_MODE_SINGLE_SHOT, scan_dev_timer_handler);
            app_timer_start(m_scan_dev_timer, APP_TIMER_TICKS(5000), NULL);
            start_scan_nearby_dev(true);
            break;
        case NUS_CMD_PROV_DEV:
        {
            uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
            provisioning_with_item(p_data[3]);
            break;
        }
        default:

            break;
    }


}



void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    #if NRF_LOG_ENABLED
        error_info_t * err_info = (error_info_t*)info;
        NRF_LOG_ERROR(" id = %d, pc = %d, file = %s, line number: %d, error code = %d = %s \r\n", \
        id, pc, nrf_log_push((char*)err_info->p_file_name), err_info->line_num, err_info->err_code, nrf_log_push((char*)nrf_strerror_find(err_info->err_code)));
    #endif
   
    NRF_LOG_FINAL_FLUSH();
    nrf_delay_ms(5);
    
    // On assert, the system can only recover with a reset.
    #ifndef DEBUG
        NVIC_SystemReset();
    #endif

    app_error_save_and_stop(id, pc, info);
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F
static void power_manage(void)
{
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 */
static void thingy_ble_evt_handler(m_ble_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case thingy_ble_evt_connected:
            NRF_LOG_INFO("Thingy_ble_evt_connected \r\n");
            break;

        case thingy_ble_evt_disconnected:
            NRF_LOG_INFO("Thingy_ble_evt_disconnected \r\n");
            NRF_LOG_FINAL_FLUSH();
            nrf_delay_ms(5);
            NVIC_SystemReset();
            break;

        case thingy_ble_evt_timeout:
            NRF_LOG_INFO("Thingy_ble_evt_timeout \r\n");
            NVIC_SystemReset();
            break;
    }
}


/**@brief Function for initializing the Thingy.
 */
static void thingy_init(void)
{
    uint32_t                 err_code;
    m_ui_init_t              ui_params;
    m_environment_init_t     env_params;
    m_motion_init_t          motion_params;
    m_ble_init_t             ble_params;

    /**@brief Initialize the TWI manager. */
    err_code = twi_manager_init(APP_IRQ_PRIORITY_THREAD);
    APP_ERROR_CHECK(err_code);

    /**@brief Initialize LED and button UI module. */
    ui_params.p_twi_instance = &m_twi_sensors;
    err_code = m_ui_init(&ui_params);
    APP_ERROR_CHECK(err_code);

    /**@brief Initialize environment module. */
    env_params.p_twi_instance = &m_twi_sensors;

    /**@brief Initialize motion module. */
    motion_params.p_twi_instance = &m_twi_sensors;


    m_nus_init(&m_ble_service_handles[0], nus_command_handler);

    /**@brief Initialize BLE handling module. */
    ble_params.evt_handler       = thingy_ble_evt_handler;
    ble_params.p_service_handles = m_ble_service_handles;
    ble_params.service_num       = THINGY_SERVICES_MAX;


    err_code = m_ble_init(&ble_params);
    APP_ERROR_CHECK(err_code);

    ble_uis_led_t led_breath_red = {.mode = BLE_UIS_LED_MODE_BREATHE, 
                                    .data.mode_breathe.color_mix = DRV_EXT_LIGHT_COLOR_RED,
                                    .data.mode_breathe.intensity  = DEFAULT_LED_INTENSITY_PERCENT,
                                    .data.mode_breathe.delay = DEFAULT_LED_OFF_TIME_MS};
     led_set(&led_breath_red, NULL);

}

static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
    static uint8_t led_state = 0;
    if(button_action == 1)
    {
        led_state ^= 1;
    }
    ble_uis_led_t led_on = {.mode = BLE_UIS_LED_MODE_CONST, 
                            .data.mode_const.r = 0xff, 
                            .data.mode_const.g = 0xff, 
                            .data.mode_const.b = 0xff};
    ble_uis_led_t led_off = {.mode = BLE_UIS_LED_MODE_CONST, 
                            .data.mode_const.r = 0x00, 
                            .data.mode_const.g = 0x00, 
                            .data.mode_const.b = 0x00};
    if (pin_no == BUTTON)
    {
        if(led_state == 1)
        {
            simple_thingy_client_led_set_unreliable(&m_clients[GROUP_CLIENT_INDEX],led_on, 3); 
        }
        else
        {
            simple_thingy_client_led_set_unreliable(&m_clients[GROUP_CLIENT_INDEX],led_off, 3); 
        }
    }
}
static ret_code_t button_init(void)
{
    ret_code_t err_code;

    /* Configure gpiote for the sensors data ready interrupt. */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        RETURN_IF_ERROR(err_code);
    }

    static const app_button_cfg_t button_cfg =
    {
        .pin_no         = BUTTON,
        .active_state   = APP_BUTTON_ACTIVE_LOW,
        .pull_cfg       = NRF_GPIO_PIN_PULLUP,
        .button_handler = button_evt_handler
    };

    err_code = app_button_init(&button_cfg, 1, APP_TIMER_TICKS(50));
    RETURN_IF_ERROR(err_code);

    return app_button_enable();
}


static void board_init(void)
{
    uint32_t            err_code;
    drv_ext_gpio_init_t ext_gpio_init;

    #if defined(THINGY_HW_v0_7_0)
        #error   "HW version v0.7.0 not supported."
    #elif defined(THINGY_HW_v0_8_0)
        NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.8.0 \r\n");
    #elif defined(THINGY_HW_v0_9_0)
        NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.9.0 \r\n");
    #endif

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg      = &twi_config
    };

    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    err_code = support_func_configure_io_startup(&ext_gpio_init);
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(100);
}



/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    NRF_LOG_INFO(NRF_LOG_COLOR_CODE_GREEN"===== Thingy started! =====  \r\n");

    nrf_gpio_cfg_input(BUTTON, NRF_GPIO_PIN_PULLUP);
    APP_ERROR_CHECK(err_code);

    if(nrf_gpio_pin_read(BUTTON) == 0)
    {
        uint8_t erase_page_num =  1 + ACCESS_FLASH_PAGE_COUNT + DSM_FLASH_PAGE_COUNT + NET_FLASH_PAGE_COUNT;
        uint32_t vptr_end = FLASH_MANAGER_RECOVERY_PAGE;
        uint32_t vptr= FLASH_MANAGER_RECOVERY_PAGE - erase_page_num*FDS_PHY_PAGE_SIZE*sizeof(uint32_t);
        for(; vptr_end >= vptr ; vptr += FDS_PHY_PAGE_SIZE*sizeof(uint32_t))
        {
            nrf_nvmc_page_erase(vptr);
        }
        NRF_LOG_INFO("Provisioning information erased\r\n");
        NVIC_SystemReset();
    }
    
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    board_init();
    thingy_init();
    button_init();
    mesh_core_setup();
    access_setup();


    for (;;)
    {
        app_sched_execute();

        if (!NRF_LOG_PROCESS()) // Process logs
        { 
            power_manage();
        }
    }
}
