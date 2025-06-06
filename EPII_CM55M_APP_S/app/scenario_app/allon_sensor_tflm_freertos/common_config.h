/*
 * common_config.h
 *
 *  Created on: Nov 22, 2022
 *      Author: bigcat-himax
 */

#ifndef APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_
#define APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_

/** MODEL location:
 *	0: model file is a c file which will locate to memory.
 *		in this example, model data is "person_detect_model_data_vela.cc" file.
 *
 *	1: model file will off-line burn to dedicated location in flash,
 *		use flash memory mapped address to load model.
 *		in this example, model data is pre-burn to flash address: 0x180000
 * **/
#define IS_EFFICIENTVIT             1       // 0 : is mobileone_s0, 1 : is efficientvit_b0
#define FLASH_XIP_MODEL             1
#define MODEL_FLASH_ADDR            0x3AB7B000
#define MEM_FREE_POS                (BOOT2NDLOADER_BASE)

#define SUPPORT_FATFS               0       // 0 : send images via SPI, 1 : save images to SD card
#define ENTER_SLEEP_MODE			0		// 0 : always on, 1 : enter Sleep mode
#define SENSOR_AE_STABLE_CNT		10
#define ENTER_PMU_MODE_FRAME_CNT	3

#endif /* APP_SCENARIO_ALLON_SENSOR_TFLM_COMMON_CONFIG_H_ */
