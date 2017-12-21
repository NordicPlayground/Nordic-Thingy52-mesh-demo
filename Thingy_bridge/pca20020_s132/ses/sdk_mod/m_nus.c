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


#include "ble_nus.h"
#include "macros_common.h"
#include "nrf_log.h"
#include "m_ble.h"
#include "m_nus.h"
#include "log.h"

ble_nus_t                  m_nus;
ble_nus_data_handler_t m_data_handler;



static void nus_on_ble_evt(ble_evt_t * p_ble_evt)
{
    ble_nus_on_ble_evt(&m_nus,p_ble_evt);
}

static uint32_t nus_service_init(bool major_minor_fw_ver_changed)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = m_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
}

static uint32_t nus_data_send(uint8_t * p_string, uint16_t length)
{
    return ble_nus_string_send(&m_nus, p_string, length);
}

uint32_t nus_response_send(nus_rsp_t rsp_type, uint32_t rsp_node, uint8_t * rsp_data, uint8_t rsp_data_length)
{
    uint8_t rtn_packet[23] = {0};
    uint32_t err_code;
    rtn_packet[0] = (rsp_node >> 8) & 0xff;
    rtn_packet[1] = (rsp_node & 0x00ff);
    rtn_packet[2] = rsp_type;
    memcpy(&rtn_packet[3], rsp_data, rsp_data_length);
    err_code = nus_data_send(rtn_packet, rsp_data_length + 3);
    return err_code;
} 


uint32_t m_nus_init(m_ble_service_handle_t * p_handle, ble_nus_data_handler_t data_handler)
{
    uint32_t err_code;

    NULL_PARAM_CHECK(p_handle);

    NRF_LOG_INFO("Init: \r\n");
    m_data_handler = data_handler;
    p_handle->ble_evt_cb = nus_on_ble_evt;
    p_handle->init_cb    = nus_service_init;

}
