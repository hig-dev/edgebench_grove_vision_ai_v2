#include "allon_sensor_tflm.h"
#include "xprintf.h"
#include "WE2_debug.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include "driver_interface.h"
#ifdef IP_sensorctrl
#include "hx_drv_sensorctrl.h"
#endif
#ifdef IP_xdma
#include "hx_drv_xdma.h"
#endif
#ifdef IP_cdm
#include "hx_drv_cdm.h"
#endif
#ifdef IP_edm
#include "hx_drv_edm.h"
#endif
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif
#ifdef IP_swreg_aon
#include "hx_drv_swreg_aon.h"
#endif
#include "hx_drv_pmu.h"
#include "powermode.h"
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

#include "common_config.h"
#include "app_msg.h"
#include "app_state.h"
#include "comm_task.h"
#include "cvapp.h"
#include "sleep_mode.h"
#include "pinmux_cfg.h"

#define DEAULT_XHSUTDOWN_PIN    AON_GPIO2
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define comm_task_PRIORITY	(configMAX_PRIORITIES - 1)
#define main_task_PRIORITY	(configMAX_PRIORITIES - 2)

#define COMM_TASK_QUEUE_LEN   		10
#define MAIN_TASK_QUEUE_LEN   		10

volatile APP_MAIN_TASK_STATE_E g_maintask_state = APP_MAIN_TASK_STATE_UNINIT;
volatile APP_COMM_TASK_STATE_E g_commtask_state = APP_COMM_TASK_STATE_UNINIT;

QueueHandle_t     xMainTaskQueue;
QueueHandle_t     xCommTaskQueue;

uint32_t g_algo_done_frame = 0;
uint32_t g_enter_pmu_frame_cnt = 0;

extern void app_start_state(APP_STATE_E state);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void main_task(void *pvParameters);
void pinmux_init();


/*******************************************************************************
 * Code
 ******************************************************************************/
void pinmux_init()
{
	SCU_PINMUX_CFG_T pinmux_cfg;

	hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);

	/* Init UART0 pin mux to PB0 and PB1 */
	uart0_pinmux_cfg(&pinmux_cfg);

	/* Init AON_GPIO1 pin mux to PA1 for OV5647 enable pin */
	aon_gpio1_pinmux_cfg(&pinmux_cfg);

	/* Init I2C slave 0 pin mux to PA2, PA3 (SCL, SDA)*/
	i2cs0_pinmux_cfg(&pinmux_cfg);

	/* Init SPI master pin mux */
	spi_m_pinmux_cfg(&pinmux_cfg);

	/* Init Arm SWD interface pin mux to PB6, PB7, PB8 (nR, CLK, DIO)*/
	//swd_pinmux_cfg(&pinmux_cfg);

	hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
}


/*!
 * @brief Main function
 */
int app_main(void)
{
	pinmux_init();

	dbg_printf(DBG_LESS_INFO, "freertos rtos_app\r\n");

	g_maintask_state = APP_MAIN_TASK_STATE_UNINIT;
	g_commtask_state = APP_COMM_TASK_STATE_UNINIT;

	xCommTaskQueue  = xQueueCreate( COMM_TASK_QUEUE_LEN  , sizeof(APP_MSG_T) );
	if(xCommTaskQueue == 0)
	{
        dbg_printf(DBG_LESS_INFO, "xCommTaskQueue creation failed!.\r\n");
        while (1)
            ;
	}

	xMainTaskQueue  = xQueueCreate( MAIN_TASK_QUEUE_LEN  , sizeof(APP_MSG_T) );
	if(xMainTaskQueue == 0)
	{
        dbg_printf(DBG_LESS_INFO, "xMainTaskQueue creation failed!.\r\n");
        while (1)
            ;
	}


    if (xTaskCreate(comm_task, "Comm_task", 512, NULL, comm_task_PRIORITY, NULL) !=
        pdPASS)
    {
        dbg_printf(DBG_LESS_INFO, "comm_task creation failed!.\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(main_task, "Main_task", 512, NULL, main_task_PRIORITY, NULL) !=
        pdPASS)
    {
        dbg_printf(DBG_LESS_INFO, "main_task creation failed!.\r\n");
        while (1)
            ;
    }

	dbg_printf(DBG_LESS_INFO, "start scheduler\r\n");

    vTaskStartScheduler();

    for (;;)
        ;

    return 0;
}


void main_task(void *pvParameters)
{
	APP_MSG_T main_recv_msg;
    uint8_t main_motion_detect = 0;
    uint8_t main_waitstart_cap = 0;
    uint8_t gpioValue;
	uint32_t wakeup_event;
	uint32_t wakeup_event1;

	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);
    xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x\n", wakeup_event, wakeup_event1);

#if (FLASH_XIP_MODEL == 1)
    hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);
#endif

    if(cv_init(true, true)<0) {
    	xprintf("cv init fail\n");
        while (1)
            ;
    }

	g_maintask_state = APP_MAIN_TASK_STATE_INIT;

	if ((wakeup_event == PMU_WAKEUP_NONE) && (wakeup_event1 == PMU_WAKEUPEVENT1_NONE))
	{
		/*Cold Boot*/
		xprintf("### Cold Boot ###\n");
		g_enter_pmu_frame_cnt = SENSOR_AE_STABLE_CNT;
	}
	else
	{
		/*Warm Boot*/
		xprintf("### Warm Boot ###\n");
		g_enter_pmu_frame_cnt = ENTER_PMU_MODE_FRAME_CNT;
		xprintf("drv_interface_set_mipi_ctrl(SCU_MIPI_CTRL_CPU)\n");
		drv_interface_set_mipi_ctrl(SCU_MIPI_CTRL_CPU);
	}

#ifdef SUPPORT_DUAL_CORE
	hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_RESET);
	hx_drv_scu_set_cm55s_state(SCU_CM55S_STATE_NORMAL);
	hx_drv_scu_set_CM55S_CPUWAIT(SCU_CM55_CPU_RUN);
#endif
    for (;;)
    {
    	if (xQueueReceive ( xMainTaskQueue , &(main_recv_msg) , __QueueRecvTicksToWait ) == pdTRUE )
    	{
    	   	dbg_printf(DBG_LESS_INFO, "main_recv_msg=0x%x\r\n", main_recv_msg.msg_event);
    	   	switch(main_recv_msg.msg_event)
    	   	{
    	   	case APP_MSG_MAINEVENT_SENSOR_TIMER:
    	   		break;
    	   	case APP_MSG_MAINEVENT_AON_GPIO0_INT:
    	   	    hx_drv_gpio_get_in_value(AON_GPIO0, &gpioValue);
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_EVENT_MAIN_AON_GPIO0_INT %d AON_GPIO0=%d\n", main_recv_msg.msg_data, gpioValue);
    	   		break;
    	   	case APP_MSG_MAINEVENT_AON_GPIO1_INT:
    	   	    hx_drv_gpio_get_in_value(AON_GPIO1, &gpioValue);
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_EVENT_MAIN_AON_GPIO1_INT %d gpioValue=%d\n", main_recv_msg.msg_data, gpioValue);
    	   		break;
    	   	case APP_MSG_MAINEVENT_SB_GPIO0_INT:
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_EVENT_MAIN_SB_GPIO0_INT\n");
    	   		break;
    	   	case APP_MSG_MAINEVENT_SB_GPIO1_INT:
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_EVENT_MAIN_SB_GPIO1_INT\n");
    	   		break;
    	   	case APP_MSG_MAINEVENT_I2CCOMM:
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_MAINEVENT_I2CCOMM\n");
    	   		break;
    	   	case APP_MSG_MAINEVENT_CM55SRDY_NOTIFY:
    	   		dbg_printf(DBG_LESS_INFO, "APP_MSG_MAINEVENT_CM55SRDY_NOTIFY\n");
    	   		break;
    	   	default:
    	   		break;
    	   	}

    	}
    }
}
