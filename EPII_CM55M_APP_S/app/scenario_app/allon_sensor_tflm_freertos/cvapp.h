/*
 * cvapp.h
 *
 *  Created on: 2018�~12��4��
 *      Author: 902452
 */

#ifndef APP_SCENARIO_ALLON_SENSOR_TFLM_CVAPP_
#define APP_SCENARIO_ALLON_SENSOR_TFLM_CVAPP_

#include "spi_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int8_t* input_tensor_;
extern int8_t* output_tensor_;
extern size_t model_input_size_;

int cv_init(bool security_enable, bool privilege_enable);

int cv_inference_test(int iterations);
bool cv_accuracy_test();

int cv_deinit();
#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_ALLON_SENSOR_TFLM_CVAPP_ */
