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

#define DBG_EVT_IICS_CMD_LOG (1)
#define DBG_EVT_IICS_CALLBACK_LOG (1)

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
        dbg_printf(DBG_LESS_INFO, "send to comm task 0x%x\r\n", comm_send_msg.msg_event);
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
        dbg_printf(DBG_LESS_INFO, "send to comm task 0x%x\r\n", comm_send_msg.msg_event);
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
    // xprintf("0x%02x 0x%02x 0x%02x 0x%02x (rx.s)\n", gRead_buf[USE_DW_IIC_SLV_0][0], gRead_buf[USE_DW_IIC_SLV_0][1], gRead_buf[USE_DW_IIC_SLV_0][2], gRead_buf[USE_DW_IIC_SLV_0][3]);
    // hx_InvalidateDCache_by_Addr((uint32_t) &gRead_buf[USE_DW_IIC_SLV_0], I2CCOMM_MAX_RBUF_SIZE);
    // xprintf("0x%02x 0x%02x 0x%02x 0x%02x (rx.e)\n", gRead_buf[USE_DW_IIC_SLV_0][0], gRead_buf[USE_DW_IIC_SLV_0][1], gRead_buf[USE_DW_IIC_SLV_0][2], gRead_buf[USE_DW_IIC_SLV_0][3]);
    uint8_t ret = 0;
    unsigned char feature = gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_FEATURE_OFFSET];
    xprintf("\n");
    xprintf("%s(feature:0x%02x) \n", __FUNCTION__, feature);

    USE_DW_IIC_SLV_E iic_id = USE_DW_IIC_SLV_0;

    unsigned char cmd;
    int payload_size;
    int offset;
    int chunk_size;
    unsigned short checksum;

    // ret = hx_lib_i2ccomm_validate_checksum((unsigned char *)&gRead_buf[iic_id]);
    // if (ret != I2CCOMM_NO_ERROR)
    // {
    //     dbg_evt_iics_cmd("%s - checksum validation : FAIL\n", __FUNCTION__);
    //     prv_evt_i2ccomm_clear_read_buf_header(iic_id);
    //     hx_lib_i2ccomm_enable_read(iic_id, (unsigned char *)&gRead_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
    //     return -1;
    // }

    cmd = gRead_buf[iic_id][I2CFMT_COMMAND_OFFSET];
    xprintf("%s(iic_id:%d, cmd:0x%02x) \n", __FUNCTION__, iic_id, cmd);

    switch (feature)
    {
    case I2CCOMM_FEATURE_MODE:
        xprintf("I2CCOMM_FEATURE_MODE\n");
        mode_ = gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0];
        xprintf("I2CCOMM_FEATURE_MODE mode=%d\n", mode_);
        break;
    case I2CCOMM_FEATURE_ITERATIONS:
        xprintf("I2CCOMM_FEATURE_ITERATIONS\n");
        iterations_ = (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0] << DATA_SFT_OFFSET_0) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1] << DATA_SFT_OFFSET_8) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2] << DATA_SFT_OFFSET_16) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3] << DATA_SFT_OFFSET_24);
        xprintf("I2CCOMM_FEATURE_ITERATIONS iterations=%d\n", iterations_);
        break;
    case I2CCOMM_FEATURE_INPUT:
        xprintf("I2CCOMM_FEATURE_INPUT\n");
        if (input_tensor_ == NULL)
        {
            xprintf("I2CCOMM_FEATURE_INPUT input_tensor_ is NULL\n");
            ret = -1;
            break;
        }
        payload_size = (gRead_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] << DATA_SFT_OFFSET_0) |
                       (gRead_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] << DATA_SFT_OFFSET_8);
        offset = (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0] << DATA_SFT_OFFSET_0) |
                 (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1] << DATA_SFT_OFFSET_8) |
                 (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2] << DATA_SFT_OFFSET_16) |
                 (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3] << DATA_SFT_OFFSET_24);
        xprintf("I2CCOMM_FEATURE_INPUT payload_size=%d, offset=%d\n", payload_size, offset);
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
                // Accuracy test
                if (cv_accuracy_test())
                {
                    input_bytes_written_ = 0; // Reset for next input
                    xprintf("I2CCOMM_FEATURE_CMD cv_accuracy_test passed\n");
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
        xprintf("I2CCOMM_FEATURE_LATENCY_RESULT\n");
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
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] = (msPayloadLength >> 8) & 0xFF;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] = msPayloadLength & 0xFF;

        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET] = (latency_result_ms_ >> DATA_SFT_OFFSET_0) & 0xFF;
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1] = (latency_result_ms_ >> DATA_SFT_OFFSET_8) & 0xFF;
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2] = (latency_result_ms_ >> DATA_SFT_OFFSET_16) & 0xFF;
        gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3] = (latency_result_ms_ >> DATA_SFT_OFFSET_24) & 0xFF;
        // Checksum
        auto retval = hx_lib_i2ccomm_generate_checksum((unsigned char *) &gWrite_buf[iic_id], I2CCOMM_HEADER_SIZE + msPayloadLength, &checksum);

        if (retval == I2CCOMM_NO_ERROR)
        {
            dbg_evt_iics_cmd("checksum generation : 0x%04x \n", checksum);
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength] = checksum & 0xFF;
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength + 1] = (checksum >> DATA_SFT_OFFSET_8) & 0xFF;
        }
        else
        {
            dbg_evt_iics_cmd("[Warning] i2c cmd - checksum generation : FAIL\n");
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength] = 0xFF;
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + msPayloadLength + 1] = 0xFF;
        }

        hx_CleanDCache_by_Addr((void *)&gWrite_buf[iic_id], I2CCOMM_MAX_RBUF_SIZE);
        hx_lib_i2ccomm_enable_write(iic_id, (unsigned char *)&gWrite_buf[iic_id]);
        break;
    case I2CCOMM_FEATURE_ACCURACY_RESULT:
        xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT\n");
        if (model_output_size_ <= 0 )
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
        int offset = (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 0] << DATA_SFT_OFFSET_0) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 1] << DATA_SFT_OFFSET_8) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 2] << DATA_SFT_OFFSET_16) |
                      (gRead_buf[iic_id][I2CFMT_PAYLOAD_OFFSET + 3] << DATA_SFT_OFFSET_24);

        int chunk_size = model_output_size_ - offset < I2CCOMM_MAX_PAYLOAD_SIZE ? model_output_size_ - offset : I2CCOMM_MAX_PAYLOAD_SIZE;
        
        // prepare write buffer for write process
        gWrite_buf[iic_id][I2CFMT_FEATURE_OFFSET] = I2CCOMM_FEATURE_ACCURACY_RESULT;
        gWrite_buf[iic_id][I2CFMT_COMMAND_OFFSET] = 0x0;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_MSB_OFFSET] = (chunk_size >> 8) & 0xFF;
        gWrite_buf[iic_id][I2CFMT_PAYLOADLEN_LSB_OFFSET] = chunk_size & 0xFF;

        memcpy(&gWrite_buf[iic_id][I2CFMT_PAYLOAD_OFFSET], output_tensor_ + offset, chunk_size);

        // Checksum
        auto retval2 = hx_lib_i2ccomm_generate_checksum((unsigned char *) &gWrite_buf[iic_id], I2CCOMM_HEADER_SIZE + chunk_size, &checksum);

        if (retval2 == I2CCOMM_NO_ERROR)
        {
            dbg_evt_iics_cmd("checksum generation : 0x%04x \n", checksum);
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size] = checksum & 0xFF;
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size + 1] = (checksum >> DATA_SFT_OFFSET_8) & 0xFF;
        }
        else
        {
            dbg_evt_iics_cmd("[Warning] i2c cmd - checksum generation : FAIL\n");
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size] = 0xFF;
            gWrite_buf[iic_id][I2CCOMM_HEADER_SIZE + chunk_size + 1] = 0xFF;
        }

        xprintf("I2CCOMM_FEATURE_ACCURACY_RESULT offset=%d, chunk_size=%d\n", offset, chunk_size);

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
            dbg_printf(DBG_LESS_INFO, "comm_recv_msg=0x%x\r\n", comm_recv_msg.msg_event);
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
