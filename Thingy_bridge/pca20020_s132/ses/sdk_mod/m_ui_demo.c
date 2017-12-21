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

#include "m_ui_demo.h"
#include "ble_uis.h"
#include "drv_ext_light.h"
#include "app_error.h"
#include "app_button.h"
#include <stdlib.h>
#include <string.h>
#include "pca20020.h"
#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
#include "app_timer.h"

#define  NRF_LOG_MODULE_NAME "m_ui          "
#include "nrf_log.h"
#include "macros_common.h"
static ble_uis_led_t      led_config_setting;
static ble_uis_led_t     * mp_config_ui = &led_config_setting ;
static ble_uis_t           m_uis;
static const ble_uis_led_t m_default_config_connected    = UI_CONFIG_DEFAULT_CONNECTED;
static const ble_uis_led_t m_default_config_disconnected = UI_CONFIG_DEFAULT_DISCONNECTED;
static const ble_uis_led_t m_default_config_error        = UI_CONFIG_DEFAULT_ERROR;

/**@brief Treats r, g, b integer values as boolean and returns corresponing color mix.
 *
 * @param[in] color_r                   red intensity   (0 to 255)
 * @param[in] color_g                   green intensity (0 to 255)
 * @param[in] color_b                   blue intensity  (0 to 255)
 *
 * @return    drv_ext_light_color_mix   corresponding binary primary color mix
 */
static drv_ext_light_color_mix_t rgb_to_color_mix(uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
    uint8_t color_mix = 0;

    if (color_r)
    {
        color_mix |= (1 << 0);
    }

    if (color_g)
    {
        color_mix |= (1 << 1);
    }

    if (color_b)
    {
        color_mix |= (1 << 2);
    }

    return (drv_ext_light_color_mix_t)color_mix;
}


/**@brief Sends commands to drv_ext_light for changing led configurations.
 *
 * @param[in] p_config_ui       Contains all settings for the LED according to the BLE characteristics.
 * @param[in] p_color_only      Contains data from program calls (not BLE) if only color is to be changed.
 *
 * @Note If both p_config_ui and p_color_only == NULL, the static config will be loaded. This may be written to over BLE.
 * if only p_color_only == NULL, p_config_ui will be used, if only p_config_ui == NULL, p_color_only will be used.
 * Supplied intensities will be converted according to the current mode.
 *
 * @return NRF_SUCCESS
 * @return M_IU_STATUS_CODE_INVALID_PARAM
 * @return Other codes from the underlying drivers
 */
ret_code_t led_set(ble_uis_led_t const * const p_config_ui,
          drv_ext_light_rgb_intensity_t const * const p_color_only)
{
    ret_code_t     err_code;
    ble_uis_led_t  conf_ui;

    // Either load config from supplied conf_ui parameter or get from static variable.
    if ((p_config_ui == NULL) && (p_color_only == NULL)) // No config supplied, use static config.
    {
        conf_ui = *mp_config_ui;
    }
    else if(p_color_only == NULL) // Config supplied, use it.
    {
        conf_ui = *p_config_ui;
    }
    else if(p_config_ui == NULL) // p_color_only supplied. Use the supplied colors according to the mode in mp_config_ui.
    {
        conf_ui = *mp_config_ui;
        switch(conf_ui.mode)
        {
            case BLE_UIS_LED_MODE_OFF:
                // No need to copy data as status = OFF.
                break;

            case BLE_UIS_LED_MODE_CONST:
                conf_ui.data.mode_const.r = p_color_only->r;
                conf_ui.data.mode_const.g = p_color_only->g;
                conf_ui.data.mode_const.b = p_color_only->b;
                break;

            case BLE_UIS_LED_MODE_BREATHE: // Breathe or blink mode, rgb colors must be converted to binary combinations.
                conf_ui.data.mode_breathe.color_mix          = rgb_to_color_mix(p_color_only->r, p_color_only->g, p_color_only->b);
                break;

            case BLE_UIS_LED_MODE_BREATHE_ONE_SHOT:
                conf_ui.data.mode_breathe_one_shot.color_mix = rgb_to_color_mix(p_color_only->r, p_color_only->g, p_color_only->b);
                break;

            default: // Should never be reached.
                return M_IU_STATUS_CODE_INVALID_PARAM;
        }
    }
    else
    {
        return M_IU_STATUS_CODE_INVALID_PARAM;
    }

    err_code = drv_ext_light_off(DRV_EXT_RGB_LED_LIGHTWELL);
    RETURN_IF_ERROR(err_code);

    if (mp_config_ui->mode == BLE_UIS_LED_MODE_OFF)
    {
        conf_ui.mode = BLE_UIS_LED_MODE_OFF;         // If LED configured as off in the static config, let the LED remain off when BLE is disconnected as well.
        NRF_LOG_INFO("Mode: BLE_UIS_LED_MODE_OFF \r\n");
        return NRF_SUCCESS;
    }
    else if (conf_ui.mode == BLE_UIS_LED_MODE_CONST)
    {
        if ( (conf_ui.data.mode_const.r == 0) && (conf_ui.data.mode_const.g == 0) && (conf_ui.data.mode_const.b == 0) )
        {
            return NRF_SUCCESS;   // LED should be off.
        }

        drv_ext_light_rgb_intensity_t color;

        color.r = conf_ui.data.mode_const.r;
        color.g = conf_ui.data.mode_const.g;
        color.b = conf_ui.data.mode_const.b;
        NRF_LOG_INFO("Mode: BLE_UIS_LED_MODE_CONST. Color hex: %x, %x, %x, \r\n", conf_ui.data.mode_const.r, conf_ui.data.mode_const.g, conf_ui.data.mode_const.b);

        return drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_LIGHTWELL, &color);
    }
    else if (conf_ui.mode == BLE_UIS_LED_MODE_BREATHE)
    {
        drv_ext_light_rgb_sequence_t led_sequence = SEQUENCE_DEFAULT_VALUES;

        led_sequence.sequence_vals.on_intensity = (uint8_t)(conf_ui.data.mode_breathe.intensity * 2.55f);
        led_sequence.sequence_vals.off_time_ms = conf_ui.data.mode_breathe.delay;
        led_sequence.color = (drv_ext_light_color_mix_t)conf_ui.data.mode_breathe.color_mix;

        NRF_LOG_INFO("Mode: BLE_UIS_LED_MODE_BREATHE \r\n");
        return drv_ext_light_rgb_sequence(DRV_EXT_RGB_LED_LIGHTWELL, &led_sequence);
    }
    else if (conf_ui.mode == BLE_UIS_LED_MODE_BREATHE_ONE_SHOT)
    {
        drv_ext_light_rgb_sequence_t led_sequence = SEQUENCE_DEFAULT_VALUES;

        led_sequence.sequence_vals.on_intensity = (uint8_t)(conf_ui.data.mode_breathe_one_shot.intensity * 2.55f);
        led_sequence.sequence_vals.off_time_ms = 0; // Set to zero so it is detected as one-shot
        led_sequence.color = (drv_ext_light_color_mix_t)conf_ui.data.mode_breathe_one_shot.color_mix;

        NRF_LOG_INFO("Mode: BLE_UIS_LED_MODE_BREATHE_ONE_SHOT \r\n");
        return drv_ext_light_rgb_sequence(DRV_EXT_RGB_LED_LIGHTWELL, &led_sequence);
    }
    else
    {
        return M_IU_STATUS_CODE_INVALID_PARAM;
    }
}



ret_code_t m_ui_led_set(uint8_t r, uint8_t g, uint8_t b)
{

    drv_ext_light_rgb_intensity_t rgb;
    rgb.r = r;
    rgb.g = g;
    rgb.b = b;

    return led_set(NULL, &rgb);
}


ret_code_t m_ui_led_set_event(ui_led_events event_code)
{
    ret_code_t err_code;

    if (event_code == M_UI_BLE_CONNECTED)
    {
        err_code = led_set(&m_default_config_connected, NULL);
        RETURN_IF_ERROR(err_code);
    }
    else if (event_code == M_UI_BLE_DISCONNECTED)
    {
        err_code = led_set(&m_default_config_disconnected, NULL);
        RETURN_IF_ERROR(err_code);
    }
    else
    {
        err_code = led_set(&m_default_config_error, NULL);
        RETURN_IF_ERROR(err_code);
    }
    return NRF_SUCCESS;
}

uint32_t m_ui_init(m_ui_init_t * p_params)
{
    uint32_t                        err_code;
    static drv_sx1509_cfg_t         sx1509_cfg;
    drv_ext_light_init_t            led_init;
    //lint --e{651} Potentially confusing initializer
    static const drv_ext_light_conf_t led_conf[DRV_EXT_LIGHT_NUM] = DRV_EXT_LIGHT_CFG;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    sx1509_cfg.twi_addr       = SX1509_ADDR;
    sx1509_cfg.p_twi_instance = p_params->p_twi_instance;
    sx1509_cfg.p_twi_cfg      = &twi_config;

    led_init.p_light_conf        = led_conf;
    led_init.num_lights          = DRV_EXT_LIGHT_NUM;
    led_init.clkx_div            = DRV_EXT_LIGHT_CLKX_DIV_8;
    led_init.p_twi_conf          = &sx1509_cfg;
    led_init.resync_pin          = SX_RESET;

    memcpy(mp_config_ui, &m_default_config_connected, sizeof(ble_uis_led_t));
    
    err_code = drv_ext_light_init(&led_init, false);
    APP_ERROR_CHECK(err_code);

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);
    (void)drv_ext_light_off(DRV_EXT_RGB_LED_LIGHTWELL);
    
    nrf_gpio_cfg_output(MOS_1);
    nrf_gpio_cfg_output(MOS_2);
    nrf_gpio_cfg_output(MOS_3);
    nrf_gpio_cfg_output(MOS_4);
    nrf_gpio_pin_clear(MOS_1);
    nrf_gpio_pin_clear(MOS_2);
    nrf_gpio_pin_clear(MOS_3);
    nrf_gpio_pin_clear(MOS_4);

    return NRF_SUCCESS;
}
