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
#include "ble_uis.h"
#include "nrf_nvmc.h"
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
#include "simple_thingy_server.h"
#include "drv_humidity.h"

simple_thingy_server_t m_server;

#define  NRF_LOG_MODULE_NAME "main          "
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
APP_TIMER_DEF(m_sensor_timer_id);       // Sensor feedback timer

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

static void sensor_timer_handler()
{
    NRF_LOG_INFO("sensor log start\r\n");
    drv_humidity_sample();
    float humidity = drv_humidity_get();
    float temperature = drv_humidity_temp_get();
    sensor_reading_t sensor_data = {
      .humidity = humidity,
      .temperature = temperature
    };
    simple_thingy_sensor_report(&m_server, sensor_data);
}

void thingy_led_set(simple_thingy_server_t * server, ble_uis_led_t led_config)
{
    led_set(&led_config, NULL);
    server->present_led_status = led_config;
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

    APP_ERROR_CHECK(err_code);

    ble_uis_led_t init_stat;
    init_stat.mode = BLE_UIS_LED_MODE_BREATHE;
    init_stat.data.mode_breathe.color_mix = 0x02; 
    init_stat.data.mode_breathe.intensity = 100;
    init_stat.data.mode_breathe.delay = 100;
    thingy_led_set(&m_server, init_stat);

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

static void configuration_complete(void * p_unused)
{
    ble_uis_led_t led_cmd;
    led_cmd.mode = BLE_UIS_LED_MODE_CONST;
    led_cmd.data.mode_const.r = 0x0f;
    led_cmd.data.mode_const.g = 0x0f;
    led_cmd.data.mode_const.b = 0x0f;
    thingy_led_set(&m_server, led_cmd);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
}

static ble_uis_led_t led_get_cb()
{
    return m_server.present_led_status;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get led status\r\n");
}

static ble_uis_led_t led_set_cb(const simple_thingy_server_t * server, ble_uis_led_t led_config)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "set led status\r\n");
    thingy_led_set(&m_server, led_config);
    return led_config;
    
}

static void sensor_set_cb(const simple_thingy_server_t * server, sensor_config_t sensor_cfg)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "set sensor status, cfg = %x\r\n", sensor_cfg.report_timer);
    
    switch(sensor_cfg.report_timer)
    {
        case SENSOR_REPORT_NONE:
            app_timer_stop(m_sensor_timer_id);
            break;
        case SENSOR_REPORT_EVERY_1S:
            app_timer_stop(m_sensor_timer_id);
            app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(1000), NULL);
            break;
        case SENSOR_REPORT_EVERY_5S:
            app_timer_stop(m_sensor_timer_id);
            app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(5000), NULL);
            break;
        case SENSOR_REPORT_EVERY_10S:
            app_timer_stop(m_sensor_timer_id);
            app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(10000), NULL);
            break;
        case SENSOR_REPORT_MANUAL:
            app_timer_stop(m_sensor_timer_id);
            sensor_timer_handler();
            break;

        default:
          
            break;

    }
    
}

static void configuration_setup(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n")
    m_server.led_get_cb = led_get_cb;
    m_server.led_set_cb = led_set_cb;
    m_server.sensor_set_cb = sensor_set_cb;
    ERROR_CHECK(simple_thingy_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
}

void mesh_stack_init(void)
{
    static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
    static nrf_mesh_node_config_params_t config_params;
    config_params.prov_caps.num_elements = ACCESS_ELEMENT_COUNT;
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.p_static_data = static_auth_data;
    config_params.complete_callback = configuration_complete;
    config_params.setup_callback = configuration_setup;
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
    nrf_mesh_node_config(&config_params);
}

void drv_humidity_evt_handler(drv_humidity_evt_t evt)
{
    if(evt == DRV_HUMIDITY_EVT_ERROR)
    {
        NRF_LOG_INFO("Humidity sensor error\r\n");
    }
}

void sensor_init()
{
    ret_code_t err_code = NRF_SUCCESS;
    static const nrf_drv_twi_config_t twi_config = 
    {
        .scl = TWI_SCL,
        .sda = TWI_SDA,
        .frequency = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_init_t init_params = 
    {
        .twi_addr = HTS221_ADDR,
        .pin_int = HTS_INT,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg = &twi_config,
        .evt_handler = drv_humidity_evt_handler
    };
    ERROR_CHECK(drv_humidity_init(&init_params));
    ERROR_CHECK(drv_humidity_enable());

    app_timer_create(&m_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensor_timer_handler);

}
/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO(NRF_LOG_COLOR_CODE_GREEN"===== Thingy mesh demo node started! =====\r\n");
    nrf_gpio_cfg_input(BUTTON, NRF_GPIO_PIN_PULLUP);

    if(nrf_gpio_pin_read(BUTTON) == 0){
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

    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    
    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    board_init();
    thingy_init();
    sensor_init();
    mesh_stack_init();

    for (;;)
    {
        app_sched_execute();
        nrf_mesh_process();
        
        if (!NRF_LOG_PROCESS()) // Process logs
        { 
            power_manage();
        }
    }
}
