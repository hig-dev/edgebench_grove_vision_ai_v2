/*
 * comm_task.c
 *
 *  Created on: 2022/02/22
 *      Author: 902447
 */

#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "WE2_device.h"
#include "WE2_core.h"
#include "WE2_debug.h"
#include "board.h"
#include "xprintf.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif

#ifdef FREERTOS
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#endif
#ifdef TRUSTZONE_SEC
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
#ifdef NSC
#include "veneer_table.h"
#endif
/* Trustzone config. */

#ifndef TRUSTZONE_SEC_ONLY
/* FreeRTOS includes. */
#include "secure_port_macros.h"
#endif
#endif
#include "app_msg.h"
#include "app_state.h"
#include "xprintf.h"
#include "board.h"
#include "i2c_comm.h"
#include "evt_i2ccomm.h"
#include "app_msg.h"
#include "app_state.h"
#include "comm_task.h"
#include "allon_sensor_tflm.h"
#include "cvapp.h"

int iterations_ = 0;
uint8_t mode_ = 0;
static const int INPUT_BYTES = 196608;
int input_bytes_written_ = 0;
int latency_result_ms_ = 0;

#define DBG_EVT_IICS_CMD_LOG (0)
#define DBG_EVT_IICS_CALLBACK_LOG (0)

#if DBG_EVT_IICS_CMD_LOG
#define dbg_evt_iics_cmd(fmt, ...) xprintf(fmt, ##__VA_ARGS__)
#else
#define dbg_evt_iics_cmd(fmt, ...)
#endif

#if DBG_EVT_IICS_CALLBACK_LOG
#define dbg_evt_iics_cb(fmt, ...) xprintf(fmt, ##__VA_ARGS__)
#else
#define dbg_evt_iics_cb(fmt, ...)
#endif

/****************************************************
 * Constant Definition                              *
 ***************************************************/
#define DATA_SFT_OFFSET_0 0
#define DATA_SFT_OFFSET_8 8
#define DATA_SFT_OFFSET_16 16
#define DATA_SFT_OFFSET_24 24

#define EVT_I2CS_0_SLV_ADDR 0x62
// #define EVT_I2CS_1_SLV_ADDR         0x64
unsigned char gWrite_buf[1][I2CCOMM_MAX_WBUF_SIZE];
unsigned char gRead_buf[1][I2CCOMM_MAX_RBUF_SIZE];

extern QueueHandle_t xMainTaskQueue;
extern QueueHandle_t xCommTaskQueue;
extern volatile APP_COMM_TASK_STATE_E g_commtask_state;

void app_i2ccomm_init();
void i2cs_cb_tx(void *param);
void i2cs_cb_rx(void *param);
void i2cs_cb_err(void *param);
uint8_t evt_i2ccomm_0_rx_cb(void);

typedef void (*i2ccomm_customer)(void);

i2ccomm_customer i2ccomm_cmd_customer_process = NULL;

I2CCOMM_CFG_T gI2CCOMM_cfg[1] = {
    {EVT_I2CS_0_SLV_ADDR,
     i2cs_cb_tx,
     i2cs_cb_rx,
     i2cs_cb_err}};

/***************************************************
 * Function Implementation
 **************************************************/
const static uint16_t CRC16_MAXIM_TABLE[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1,
    0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81, 0x0b40,
    0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1,
    0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1,
    0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40,
    0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1,
    0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0,
    0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740,
    0xa501, 0x65c0, 0x6480, 0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01, 0x6ac0,
    0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1,
    0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181, 0x5140,
    0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0,
    0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0,
    0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341,
    0x4100, 0x81c1, 0x8081, 0x4040};

__attribute__((weak)) uint16_t el_crc16_maxim(const uint8_t *data, size_t length)
{
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < length; ++i)
    {
        uint8_t index = (uint8_t)(crc ^ data[i]);
        crc = (crc >> 8) ^ CRC16_MAXIM_TABLE[index];
    }

    return crc ^ 0xffff;
}

static void prv_evt_i2ccomm_clear_read_buf_header(USE_DW_IIC_SLV_E iic_id)
{
    memset((void *)&gRead_buf[iic_id], 0xFF, 4);
    hx_CleanDCache_by_Addr((void *)&gRead_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
}

void app_i2ccomm_init()
{
    USE_DW_IIC_SLV_E iic_id = USE_DW_IIC_SLV_0;

    hx_lib_i2ccomm_init(iic_id, gI2CCOMM_cfg[iic_id]);

    prv_evt_i2ccomm_clear_read_buf_header(iic_id);
    hx_lib_i2ccomm_start(iic_id, (unsigned char *)&gRead_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
}

void i2ccomm_cmd_customer_register_cb(void *cb_func)
{
    i2ccomm_cmd_customer_process = cb_func;
}

void i2cs_cb_tx(void *param)
{
    HX_DRV_DEV_IIC *iic_obj = param;
    HX_DRV_DEV_IIC_INFO *iic_info_ptr = &(iic_obj->iic_info);
    APP_MSG_T comm_send_msg;
    BaseType_t xHigherPriorityTaskWoken;
    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    if (iic_info_ptr->slv_addr == EVT_I2CS_0_SLV_ADDR)
    {
        dbg_evt_iics_cb("%s(iic_id:0) \n", __FUNCTION__);
        comm_send_msg.msg_data = iic_info_ptr->slv_addr;
        comm_send_msg.msg_event = APP_MSG_COMMEVENT_I2CCOMM_TX;
        // dbg_printf(DBG_LESS_INFO, "send to comm task 0x%x\r\n", comm_send_msg.msg_event);
        xQueueSendFromISR(xCommTaskQueue, &comm_send_msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            taskYIELD();
        }
    }
    else
    {
        dbg_evt_iics_cb("%s(iic_id:1) txcb not support\n", __FUNCTION__);
    }
}

void i2cs_cb_rx(void *param)
{
    HX_DRV_DEV_IIC *iic_obj = param;
    HX_DRV_DEV_IIC_INFO *iic_info_ptr = &(iic_obj->iic_info);
    APP_MSG_T comm_send_msg;
    BaseType_t xHigherPriorityTaskWoken;
    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    if (iic_info_ptr->slv_addr == EVT_I2CS_0_SLV_ADDR)
    {
        dbg_evt_iics_cb("%s(iic_id:0) \n", __FUNCTION__);
        comm_send_msg.msg_data = iic_info_ptr->slv_addr;
        comm_send_msg.msg_event = APP_MSG_COMMEVENT_I2CCOMM_RX;
        // dbg_printf(DBG_LESS_INFO, "send to comm task 0x%x\r\n", comm_send_msg.msg_event);
        xQueueSendFromISR(xCommTaskQueue, &comm_send_msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            taskYIELD();
        }
    }
    else
    {
        dbg_evt_iics_cb("%s(iic_id:1) rxcb not support\n", __FUNCTION__);
    }
}

void i2cs_cb_err(void *param)
{
    HX_DRV_DEV_IIC *iic_obj = param;
    HX_DRV_DEV_IIC_INFO *iic_info_ptr = &(iic_obj->iic_info);
    APP_MSG_T comm_send_msg;
    BaseType_t xHigherPriorityTaskWoken;
    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    if (iic_info_ptr->slv_addr == EVT_I2CS_0_SLV_ADDR)
    {
        dbg_evt_iics_cb("%s(iic_id:0) \n", __FUNCTION__);
        comm_send_msg.msg_data = iic_info_ptr->slv_addr;
        comm_send_msg.msg_event = APP_MSG_COMMEVENT_I2CCOMM_ERR;
        dbg_printf(DBG_LESS_INFO, "send to comm task 0x%x\r\n", comm_send_msg.msg_event);
        xQueueSendFromISR(xCommTaskQueue, &comm_send_msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            taskYIELD();
        }
    }
    else
    {
        dbg_evt_iics_cb("%s(iic_id:1) errcb not support\n", __FUNCTION__);
    }
}

uint8_t evt_i2ccomm_0_tx_cb(void)
{
    dbg_evt_iics_cmd("%s \n", __FUNCTION__);
    prv_evt_i2ccomm_clear_read_buf_header(USE_DW_IIC_SLV_0);
    hx_lib_i2ccomm_enable_read(USE_DW_IIC_SLV_0, (unsigned char *)&gRead_buf[USE_DW_IIC_SLV_0], I2CCOMM_MAX_RBUF_SIZE);
    return 0;
}

uint8_t evt_i2ccomm_0_err_cb(void)
{
    dbg_evt_iics_cmd("%s \n", __FUNCTION__);
    prv_evt_i2ccomm_clear_read_buf_header(USE_DW_IIC_SLV_0);
    hx_lib_i2ccomm_enable_read(USE_DW_IIC_SLV_0, (unsigned char *)&gRead_buf[USE_DW_IIC_SLV_0], I2CCOMM_MAX_RBUF_SIZE);
    return 0;
}

uint8_t evt_i2ccomm_0_rx_cb(void)
{
    uint8_t ret = 0;
    unsigned char feature = gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_FEATURE_OFFSET];

    USE_DW_IIC_SLV_E iic_id = USE_DW_IIC_SLV_0;

    int offset;
    int chunk_size;
    uint16_t write_crc;

    int payload_size = (gRead_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] << DATA_SFT_OFFSET_0) |
                       (gRead_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] << DATA_SFT_OFFSET_8);
    uint16_t crc_calculated = el_crc16_maxim((const uint8_t *)&gRead_buf[iic_id][I2CFMT_FEATURE_OFFSET], I2CCOMM_HEADER_SIZE + payload_size);
    uint16_t crc_received = ((uint16_t)(gRead_buf[iic_id][I2CCOMM_HEADER_SIZE + payload_size]) << 8) |
                            (uint16_t)(gRead_buf[iic_id][I2CCOMM_HEADER_SIZE + payload_size + 1]);
    if (crc_calculated != crc_received)
    {
        xprintf("CRC mismatch: calculated=0x%04x, received=0x%04x\n", crc_calculated, crc_received);
        prv_evt_i2ccomm_clear_read_buf_header(iic_id);
        hx_lib_i2ccomm_enable_read(iic_id, (unsigned char *)&gRead_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
        return -1;
    }

    switch (feature)
    {
    case I2CCOMM_FEATURE_MODE:
        xprintf("I2CCOMM_FEATURE_MODE\n");
        mode_ = gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0];
        xprintf("I2CCOMM_FEATURE_MODE mode=%d\n", mode_);
        break;
    case I2CCOMM_FEATURE_ITERATIONS:
        xprintf("I2CCOMM_FEATURE_ITERATIONS\n");
        // Assume big-endian 4-byte integer
        iterations_ =
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0]) << 24) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1]) << 16) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2]) << 8) |
            (int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3]);
        xprintf("I2CCOMM_FEATURE_ITERATIONS iterations=%d\n", iterations_);
        break;
    case I2CCOMM_FEATURE_INPUT:
        // xprintf("I2CCOMM_FEATURE_INPUT\n");
        if (input_tensor_ == NULL)
        {
            xprintf("I2CCOMM_FEATURE_INPUT input_tensor_ is NULL\n");
            ret = -1;
            break;
        }
        offset =
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0]) << 24) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1]) << 16) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2]) << 8) |
            (int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3]);
        // xprintf("I2CCOMM_FEATURE_INPUT payload_size=%d, offset=%d\n", payload_size, offset);
        chunk_size = payload_size - 4; // 4 bytes for offset
        if (offset != input_bytes_written_)
        {
            xprintf("I2CCOMM_FEATURE_INPUT offset mismatch: expected %d, got %d\n", input_bytes_written_, offset);
            ret = -1;
            break;
        }
        memcpy(input_tensor_ + offset, &gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 4], chunk_size);
        input_bytes_written_ += chunk_size;
        break;
    case I2CCOMM_FEATURE_CMD:
        xprintf("I2CCOMM_FEATURE_CMD\n");
        uint8_t cmd = gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0];
        xprintf("I2CCOMM_FEATURE_CMD cmd=0x%02x\n", cmd);
        if (cmd == I2CCOMM_START_COMMAND)
        {
            if (mode_ == I2CCOMM_LATENCY_TEST_MODE && iterations_ > 0)
            {
                if (input_bytes_written_ != INPUT_BYTES)
                {
                    xprintf("I2CCOMM_FEATURE_CMD input_bytes_written_ mismatch: expected %d, got %d\n", INPUT_BYTES, input_bytes_written_);
                    ret = -1;
                    break;
                }
                // Latency test
                latency_result_ms_ = cv_inference_test(iterations_);
            }
            else if (mode_ == I2CCOMM_ACCURACY_TEST_MODE)
            {
                if (input_bytes_written_ != INPUT_BYTES)
                {
                    xprintf("I2CCOMM_FEATURE_CMD input_bytes_written_ mismatch: expected %d, got %d\n", INPUT_BYTES, input_bytes_written_);
                    ret = -1;
                    break;
                }
                if (output_tensor_ == NULL)
                {
                    xprintf("I2CCOMM_FEATURE_CMD model_output_size_ is 0 or output_tensor_ is NULL\n");
                    ret = -1;
                    break;
                }
                // Calculate CRC16 of input tensor
                uint16_t input_crc = el_crc16_maxim((const uint8_t *)input_tensor_, INPUT_BYTES);
                xprintf("I2CCOMM_FEATURE_CMD input tensor checksum=0x%04x, size=%d\n", input_crc, model_input_size_);
                // Accuracy test
                if (cv_accuracy_test())
                {
                    input_bytes_written_ = 0; // Reset for next input
                    xprintf("I2CCOMM_FEATURE_CMD cv_accuracy_test passed\n");
                    uint16_t output_crc = el_crc16_maxim((const uint8_t *)output_tensor_, model_output_size_);
                    xprintf("I2CCOMM_FEATURE_CMD output tensor checksum=0x%04x, size=%d\n", output_crc, model_output_size_);
                }
                else
                {
                    xprintf("I2CCOMM_FEATURE_CMD cv_accuracy_test failed\n");
                    ret = -1;
                }
            }
            else
            {
                xprintf("I2CCOMM_FEATURE_CMD cmd=0x%02x mode=%d iterations=%d not supported\n", cmd, mode_, iterations_);
                ret = -1;
            }
        }
        else
        {
            xprintf("I2CCOMM_FEATURE_CMD cmd=0x%02x not supported\n", cmd);
            ret = -1;
        }

        break;
    case I2CCOMM_FEATURE_LATENCY_RESULT:
        // xprintf("I2CCOMM_FEATURE_LATENCY_RESULT\n");
        if (latency_result_ms_ <= 0)
        {
            xprintf("I2CCOMM_FEATURE_LATENCY_RESULT latency_result_ms_ is invalid: %d\n", latency_result_ms_);
            ret = -1;
            break;
        }
        const int msPayloadLength = 4;
        // prepare write buffer for write process
        gWrite_buf[iic_id][I2CFMT_FEATURE_OFFSET] = I2CCOMM_FEATURE_LATENCY_RESULT;
        gWrite_buf[iic_id][I2CFMT_COMMAND_OFFSET] = 0x0;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] = msPayloadLength >> 8;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] = msPayloadLength & 0xFF;

        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET] = (uint8_t)(latency_result_ms_ >> 24);
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1] = (uint8_t)(latency_result_ms_ >> 16);
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2] = (uint8_t)(latency_result_ms_ >> 8);
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3] = (uint8_t)(latency_result_ms_);
        // Checksum
        write_crc = el_crc16_maxim((const uint8_t *)&gWrite_buf[iic_id][I2CFMT_FEATURE_OFFSET], I2CCOMM_HEADER_SIZE + msPayloadLength);
        gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength] = write_crc >> 8;       // MSB
        gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength + 1] = write_crc & 0xFF; // LSB

        xprintf("I2CCOMM_FEATURE_LATENCY_RESULT latency_result_ms_=%d, checksum=0x%04x\n", latency_result_ms_, write_crc);

        hx_CleanDCache_by_Addr((void *)&gWrite_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
        hx_lib_i2ccomm_enable_write(iic_id, (unsigned char *)&gWrite_buf[iic_id]);
        break;
    case I2CCOMM_FEATURE_ACCURACY_RESULT:
        // xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT\n");
        const int max_chunk_size = 192;
        if (model_output_size_ <= 0)
        {
            xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT model_output_size_ is 0\n");
            ret = -1;
            break;
        }
        if (output_tensor_ == NULL)
        {
            xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT output_tensor_ is NULL\n");
            ret = -1;
            break;
        }
        int offset =
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0]) << 24) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1]) << 16) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2]) << 8) |
            (int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3]);
        int chunk_size =
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 4]) << 24) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 5]) << 16) |
            ((int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 6]) << 8) |
            (int)(gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 7]);

        int calc_chunk_size = model_output_size_ - offset < max_chunk_size ? model_output_size_ - offset : max_chunk_size;
        if (chunk_size != calc_chunk_size)
        {
            xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT chunk_size mismatch: expected %d, got %d\n", calc_chunk_size, chunk_size);
            ret = -1;
            break;
        }

        // prepare write buffer for write process
        gWrite_buf[iic_id][I2CFMT_FEATURE_OFFSET] = I2CCOMM_FEATURE_ACCURACY_RESULT;
        gWrite_buf[iic_id][I2CFMT_COMMAND_OFFSET] = 0x0;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] = chunk_size >> 8;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] = chunk_size & 0xFF;

        memcpy(&gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET], output_tensor_ + offset, chunk_size);
        // Checksum
        write_crc = el_crc16_maxim((const uint8_t *)&gWrite_buf[iic_id][I2CFMT_FEATURE_OFFSET], I2CCOMM_HEADER_SIZE + chunk_size);
        gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size] = write_crc >> 8;       // MSB
        gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size + 1] = write_crc & 0xFF; // LSB

        //xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT offset=%d, chunk_size=%d, crc=0x%04x\n", offset, chunk_size, write_crc);

        hx_CleanDCache_by_Addr((void *)&gWrite_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
        hx_lib_i2ccomm_enable_write(iic_id, (unsigned char *)&gWrite_buf[iic_id]);
        break;
    default:
        xprintf("I2CCOMM_FEATURE_UNKNOWN: 0x%02x\n", feature);
        break;
    }

    prv_evt_i2ccomm_clear_read_buf_header(iic_id);
    if (ret != -1)
    {
        hx_lib_i2ccomm_enable_read(iic_id, (unsigned char *)&gRead_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
    }
    else
    {
        xprintf("Error processing I2C command. Disabling read!\n");
    }
    return ret;
}

/*!
 * @brief Task responsible for task communication
 */
void comm_task(void *pvParameters)
{
    APP_MSG_T comm_recv_msg;
    APP_MSG_T main_send_msg;

    app_i2ccomm_init();

    g_commtask_state = APP_COMM_TASK_STATE_INIT;

    for (;;)
    {
        if (xQueueReceive(xCommTaskQueue, &(comm_recv_msg), __QueueRecvTicksToWait) == pdTRUE)
        {
            // dbg_printf(DBG_LESS_INFO, "comm_recv_msg=0x%x\r\n", comm_recv_msg.msg_event);
            switch (comm_recv_msg.msg_event)
            {
            case APP_MSG_COMMEVENT_I2CCOMM_RX:
                evt_i2ccomm_0_rx_cb();
                break;

            case APP_MSG_COMMEVENT_I2CCOMM_TX:
                evt_i2ccomm_0_tx_cb();
                break;

            case APP_MSG_COMMEVENT_I2CCOMM_ERR:
                evt_i2ccomm_0_err_cb();
                break;
            default:
                // TODO error
                g_commtask_state = APP_COMM_TASK_STATE_ERROR;
                break;
            }
        }
    }
}
