/*
 * common_config.h
 *
 *  Created on: Nov 22, 2022
 *      Author: bigcat-himax
 */

#ifndef APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_
#define APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_

// Flash like:
// python3 xmodem/xmodem_send.py --port=/dev/ttyACM0 --baudrate=921600 --protocol=xmodem --file=we2_image_gen_local/output_case1_sec_wlcsp/output.img --model="<path_to_model_vela.tflite> 0x200000 0x00000"
#define MODEL_VARIANT_MOBILEONE     0
#define MODEL_VARIANT_EFFICIENTVIT  1
#define MODEL_VARIANT_DEIT          2
#define MODEL_VARIANT               MODEL_VARIANT_MOBILEONE // Change this to select the model variant
#define FLASH_XIP_MODEL             1
#define MODEL_FLASH_ADDR            0x3A200000
#define MEM_FREE_POS                (BOOT2NDLOADER_BASE)

#define SUPPORT_FATFS               0       // 0 : send images via SPI, 1 : save images to SD card
#define ENTER_SLEEP_MODE			0		// 0 : always on, 1 : enter Sleep mode
#define SENSOR_AE_STABLE_CNT		10
#define ENTER_PMU_MODE_FRAME_CNT	3

#endif /* APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_ */
