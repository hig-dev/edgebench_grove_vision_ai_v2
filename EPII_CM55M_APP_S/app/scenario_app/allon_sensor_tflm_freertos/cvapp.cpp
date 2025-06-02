/*
 * cvapp.cpp
 *
 *  Created on: 2022/02/22
 *      Author: 902452
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "WE2_device.h"
#include "board.h"
#include "cvapp.h"

#include "WE2_core.h"
#include "WE2_device.h"

#include "ethosu_driver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"

#include "xprintf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "common_config.h"

int8_t *input_tensor_ = nullptr;
int8_t *output_tensor_ = nullptr;
size_t model_input_size_ = 0;

#ifdef TRUSTZONE_SEC
#define U55_BASE BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE BASE_ADDR_APB_U55_CTRL
#endif
#endif

#define TENSOR_ARENA_BUFSIZE (1575 * 1024)
__attribute__((section(".bss.NoInit"))) uint8_t tensor_arena_buf[TENSOR_ARENA_BUFSIZE] __ALIGNED(32);

using namespace std;

namespace
{

	constexpr int tensor_arena_size = TENSOR_ARENA_BUFSIZE;
	uint32_t tensor_arena = (uint32_t)tensor_arena_buf;

	struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
	tflite::MicroInterpreter *int_ptr = nullptr;
};

static void _arm_npu_irq_handler(void)
{
	/* Call the default interrupt handler from the NPU driver */
	ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void _arm_npu_irq_init(void)
{
	const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;

	/* Register the EthosU IRQ handler in our vector table.
	 * Note, this handler comes from the EthosU driver */
	EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);

	/* Enable the IRQ */
	NVIC_EnableIRQ(ethosu_irqnum);
}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
	int err = 0;

	/* Initialise the IRQ */
	_arm_npu_irq_init();

	/* Initialise Ethos-U55 device */
	void * const ethosu_base_address = (void *)(U55_BASE);

	if (0 != (err = ethosu_init(
				  &ethosu_drv,		   /* Ethos-U driver device pointer */
				  ethosu_base_address, /* Ethos-U NPU's base address. */
				  NULL,				   /* Pointer to fast mem area - NULL for U55. */
				  0,				   /* Fast mem region size. */
				  security_enable,	   /* Security enable. */
				  privilege_enable)))
	{ /* Privilege enable. */
		xprintf("failed to initalise Ethos-U device\n");
		return err;
	}

	xprintf("Ethos-U55 device initialised\n");

	return 0;
}

int cv_init(bool security_enable, bool privilege_enable)
{
	int ercode = 0;

	if (_arm_npu_init(security_enable, privilege_enable) != 0)
		return -1;

	static const tflite::Model *model = tflite::GetModel((const void *)MODEL_FLASH_ADDR);

	if (model->version() != TFLITE_SCHEMA_VERSION)
	{
		xprintf(
			"[ERROR] model's schema version %d is not equal "
			"to supported version %d\n",
			model->version(), TFLITE_SCHEMA_VERSION);
		return -1;
	}
	else
	{
		xprintf("model's schema version %d\n", model->version());
	}

	#if IS_EFFICIENTVIT
	static tflite::MicroMutableOpResolver<8> op_resolver;
	op_resolver.AddQuantize();
	op_resolver.AddDequantize();
	op_resolver.AddPadV2();
	op_resolver.AddBatchMatMul();
	op_resolver.AddTranspose();
	op_resolver.AddRelu();
	op_resolver.AddDiv();
	#else
	static tflite::MicroMutableOpResolver<1> op_resolver;
	#endif

	if (kTfLiteOk != op_resolver.AddEthosU())
	{
		xprintf("Failed to add Arm NPU support to op resolver.");
		return false;
	}

	static tflite::MicroInterpreter static_interpreter(model, op_resolver, (uint8_t *)tensor_arena, tensor_arena_size);

	if (static_interpreter.AllocateTensors() != kTfLiteOk)
	{
		return false;
	}
	int_ptr = &static_interpreter;
	input_tensor_ = static_interpreter.typed_input_tensor<int8_t>(0);
	output_tensor_ = static_interpreter.typed_output_tensor<int8_t>(0);
	model_input_size_ = static_interpreter.input_tensor(0)->bytes;

	xprintf("initial done\n");

	return ercode;
}

int cv_inference_test(int iterations)
{
	TickType_t t0 = xTaskGetTickCount();
	for (int i = 0; i < iterations; ++i)
	{
		auto result = int_ptr->Invoke();
		if (result != kTfLiteOk)
		{
			return -1;
		}
	}
	TickType_t t1 = xTaskGetTickCount();
	uint32_t elapsedTicks = (uint32_t)(t1 - t0);
	uint32_t ms = elapsedTicks * portTICK_PERIOD_MS;
	xprintf("cv_inference_test: %d iterations took %d ms\n", iterations, ms);
	return (int) ms;
}

bool cv_accuracy_test()
{
	auto result = int_ptr->Invoke();
	if (result != kTfLiteOk)
	{
		return false;
	}
	return true;
}

int cv_deinit()
{
	// TODO: add more deinit items here if need.
	return 0;
}
