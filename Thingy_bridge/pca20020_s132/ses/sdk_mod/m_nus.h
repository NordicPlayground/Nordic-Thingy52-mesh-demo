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

#ifndef _M_NUS_H_
#define  _M_NUS

#include "ble_nus.h"
#include "macros_common.h"
#include "nrf_log.h"
#include "m_ble.h"

typedef enum{
    NUS_RSP_SCAN_RST = 0x00,
    NUS_RSP_PROV_RST = 0x01,
    NUS_RSP_CONFIG = 0x02,
    NUS_RSP_HEALTH = 0x03,
    NUS_RSP_LED_SET = 0x04,
    NUS_RSP_SENSOR_SET = 0x05,
    NUS_RSP_SENSOR_READING = 0x06,
    NUS_RSP_LED_STATE = 0x07,
    NUS_RSP_NONE_ID = 0xffff,
}nus_rsp_t;

typedef enum
{
    NUS_CMD_AUTO_PROV = 0x01,
    NUS_CMD_SENSOR_SET = 0x02,
    NUS_CMD_LED_SET = 0x03,    
    NUS_CMD_GROUP_ADD = 0x04,
    NUS_CMD_GROUP_DELETE = 0x05,
    NUS_CMD_UNPROV_SCAN = 0x06,
    NUS_CMD_PROV_DEV = 0x07,
} nus_cmd_t;

uint32_t m_nus_init(m_ble_service_handle_t * p_handle, ble_nus_data_handler_t data_handler);
uint32_t nus_response_send(nus_rsp_t rsp_type, uint32_t rsp_node, uint8_t * rsp_data, uint8_t rsp_data_length);

#endif
