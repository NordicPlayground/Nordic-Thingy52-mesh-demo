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

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "softdevice_handler_appsh.h"
#include "app_timer.h"
#include "fstorage.h"
#include "m_ble.h"
#include "m_ble_flash.h"
#include "thingy_config.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "support_func.h"
#include "ble_nus.h"

#define  NRF_LOG_MODULE_NAME "m_ble         "
#include "nrf_log.h"
#include "macros_common.h"

#ifdef BLE_DFU_APP_SUPPORT
    #include "ble_dfu.h"
#endif // BLE_DFU_APP_SUPPORT

#ifdef BLE_DFU_APP_SUPPORT
    #define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
    #define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
    #define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
    #define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
    #define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

    STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

#define NUS_SERVICE_UUID_TYPE     BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

static uint16_t                   m_conn_handle = BLE_CONN_HANDLE_INVALID;              /**< Handle of the current connection. */
static m_ble_evt_handler_t        m_evt_handler = 0;
static m_ble_service_handle_t   * m_service_handles = 0;
static uint32_t                   m_service_num = 0;
static bool                       m_flash_disconnect = false;
static bool                       m_major_minor_fw_ver_changed = false;
static char                       m_mac_addr[SUPPORT_FUNC_MAC_ADDR_STR_LEN];            /**< The device MAC address. */


static ble_uuid_t                 m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
//ble_nus_t                  m_nus;


#define NRF_BLE_MAX_MTU_SIZE            BLE_GATT_ATT_MTU_DEFAULT*12         /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

#ifdef BLE_DFU_APP_SUPPORT
    static ble_dfu_t                  m_dfus;                                   /**< Structure used to identify the DFU service. */

    static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
    {
        switch (p_evt->type)
        {
            case BLE_DFU_EVT_INDICATION_DISABLED:
                NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
                break;

            case BLE_DFU_EVT_INDICATION_ENABLED:
                NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
                break;

            case BLE_DFU_EVT_ENTERING_BOOTLOADER:
                NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
                break;
            default:
                NRF_LOG_WARNING("Unknown event from ble_dfu\r\n");
                break;
        }
    }

#endif // BLE_DFU_APP_SUPPORT
#define CONN_CFG_TAG_THINGY 1


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static uint32_t gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL_MS;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL_MS;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_MS;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        NRF_LOG_ERROR("on_conn_params_evt: BLE_CONN_PARAMS_EVT_FAILED\r\n");
        /*
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
        */
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        NRF_LOG_INFO("on_conn_params_evt: BLE_CONN_PARAMS_EVT_SUCCEEDED\r\n");
    }
    else
    {

    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("conn_params_error_handler: %d\r\n", nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static uint32_t conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&cp_init, 0, sizeof(cp_init));
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL_MS;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL_MS;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_MS;

    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


 /**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t      err_code;
    m_ble_evt_t evt;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("on_adv_evt: BLE_ADV_EVT_FAST\r\n");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("on_adv_evt: BLE_ADV_EVT_IDLE\r\n");
            evt.evt_type = thingy_ble_evt_timeout;
            m_evt_handler(&evt);

            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);

            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                       err_code;
    m_ble_evt_t                    evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("on_ble_evt: BLE_GAP_EVT_CONNECTED\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            evt.evt_type = thingy_ble_evt_connected;
            m_evt_handler(&evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("on_ble_evt: BLE_GAP_EVT_DISCONNECTED. Reason: 0x%x \r\n", p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        {
            NRF_LOG_INFO("on_ble_evt: BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST - %d\r\n", p_ble_evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu);
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
        }
        break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST

        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
            NRF_LOG_INFO("on_ble_evt: BLE_GATTC_EVT_EXCHANGE_MTU_RSP - %d\r\n", p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu);
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            /* Allow SoftDevice to choose Data Length Update Procedure parameters
            automatically. */
            ble_gap_data_length_params_t dl_params;
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            NRF_LOG_INFO("on_ble_evt: BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\r\n");
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        }
            break;
  
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
            NRF_LOG_INFO("on_ble_evt: BLE_GAP_EVT_DATA_LENGTH_UPDATE\r\n");
        break;
        
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    //ble_nus_on_ble_evt(&m_nus,p_ble_evt);
    for (uint32_t i = 0; i < m_service_num; i++)
    {
        if (m_service_handles[i].ble_evt_cb != NULL)
        {
            m_service_handles[i].ble_evt_cb(p_ble_evt);
        }
    }

#ifdef BLE_DFU_APP_SUPPORT
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching system events from the SoftDevice.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] evt_id  System event id.
 */
static void sys_evt_dispatch(uint32_t evt_id)
{
    ble_advertising_on_sys_evt(evt_id);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);
    //SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    uint32_t app_ram_start = 0;
    err_code = softdevice_app_ram_start_get(&app_ram_start);
    APP_ERROR_CHECK(err_code);
    
    ble_cfg_t ble_cfg;
    
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, app_ram_start);
    APP_ERROR_CHECK(err_code);
    
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.service_changed.service_changed = 1;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, app_ram_start);
    APP_ERROR_CHECK(err_code);

#define CONN_CFG_TAG_THINGY 1
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = CONN_CFG_TAG_THINGY;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, app_ram_start);
    APP_ERROR_CHECK(err_code);


    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = CONN_CFG_TAG_THINGY;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, app_ram_start);
    APP_ERROR_CHECK(err_code);

   

    err_code = softdevice_enable(&app_ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

/**@brief Function for initializing the Advertising functionality.
*/
static uint32_t advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    //ble_uuid_t    adv_uuids = {BLE_UUID_NUS_SERVICE,NUS_SERVICE_UUID_TYPE};
    
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type                       = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance              = false;
    advdata.flags                           = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL_MS;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
    
    // Set both advertisement data and scan response data.
    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    RETURN_IF_ERROR(err_code);
    
    ble_advertising_conn_cfg_tag_set(CONN_CFG_TAG_THINGY);

    return NRF_SUCCESS;
}



/**@brief Function for initializing the ble services.
 */
static uint32_t services_init(m_ble_service_handle_t * p_service_handles, uint32_t num_services)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    //nus_init.data_handler = nus_data_handler;

    //err_code = ble_nus_init(&m_nus, &nus_init);
    //APP_ERROR_CHECK(err_code);

    for (uint32_t i = 0; i < num_services; i++)
    {
        if (p_service_handles[i].init_cb != NULL)
        {
            err_code = p_service_handles[i].init_cb(m_major_minor_fw_ver_changed);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
    }

    #ifdef BLE_DFU_APP_SUPPORT
        ble_dfu_init_t   dfus_init;

        // Initialize the Device Firmware Update Service.
        memset(&dfus_init, 0, sizeof(dfus_init));

        dfus_init.evt_handler                               = ble_dfu_evt_handler;
        dfus_init.ctrl_point_security_req_write_perm        = SEC_OPEN;
        dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_OPEN;

        err_code = ble_dfu_init(&m_dfus, &dfus_init);
        APP_ERROR_CHECK(err_code);

    #endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


uint32_t m_ble_init(m_ble_init_t * p_params)
{
    uint32_t err_code;

    m_evt_handler     = p_params->evt_handler;
    m_service_handles = p_params->p_service_handles;
    m_service_num     = p_params->service_num;

    err_code = ble_stack_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("ble_stack_init failed - %d\r\n", err_code);
        return err_code;
    }

        /* Enable FPU again due to SD issue */
    #if (__FPU_USED == 1)
        SCB->CPACR |= (3UL << 20) | (3UL << 22);
        __DSB();
        __ISB();
    #endif

    err_code = gap_params_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("gap_params_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = services_init(m_service_handles, m_service_num);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("thingy_ble_init: services_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = advertising_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Advertising_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = conn_params_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Conn_params_init failed - %d\r\n", err_code);
        return err_code;
    }
    
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("ble_advertising_start failed - %d\r\n", err_code);
        return err_code;
    }
    
    err_code = support_func_ble_mac_address_get(m_mac_addr);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("mac address get failed - %d\r\n", err_code);
        return err_code;
    }
    
    NRF_LOG_RAW_INFO("MAC addr-> %s \r\n", nrf_log_push(m_mac_addr));
    
    nrf_delay_ms (10);

    return NRF_SUCCESS;
}
