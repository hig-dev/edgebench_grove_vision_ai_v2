/*
 * app_msg.h
 *
 *  Created on: 2021�~12��15��
 *      Author: 902447
 */

#ifndef APP_SCENARIO_APP_FREERTOS_TZ_S_DP_SAMPLE_APP_MSG_H_
#define APP_SCENARIO_APP_FREERTOS_TZ_S_DP_SAMPLE_APP_MSG_H_

/**
 * \enum CDM_ERROR_E
 * \brief CDM Errors Type
 */
typedef enum
{
	//Main Control
	APP_MSG_MAINEVENT_LATENCY_TEST_DONE			=0x0200,
	APP_MSG_MAINEVENT_SENSOR_TIMER				=0x0201,
	APP_MSG_MAINEVENT_BENCH_ERROR				=0x0202,
	APP_MSG_MAINEVENT_ACCURACY_OUTPUT_READY	    =0x0204,
	APP_MSG_MAINEVENT_I2CCOMM					=0x020D,
	APP_MSG_MAINEVENT_CM55SRDY_NOTIFY			=0x0212,
	//COMM Control
	APP_MSG_COMMEVENT_I2CCOMM_RX				=0x0304,
	APP_MSG_COMMEVENT_I2CCOMM_TX				=0x0305,
	APP_MSG_COMMEVENT_I2CCOMM_ERR				=0x0306,
} APP_MSG_EVENT_E;

/**
 * \struct APP_MSG_T
 * \brief APP MSG
 */
typedef struct
{
	APP_MSG_EVENT_E  msg_event;				/*!< MSG E */
	uint32_t msg_data;				/*!< message data*/
} APP_MSG_T;

#endif /* APP_SCENARIO_APP_FREERTOS_TZ_S_DP_SAMPLE_APP_MSG_H_ */
