/**
 ********************************************************************************************
 *  @file      evt_i2ccomm.h
 *  @details   This file contains all event handler related function
 *  @author    himax/902452
 *  @version   V1.0.0
 *  @date      14-July-2019
 *  @copyright (C) COPYRIGHT, Himax, Inc. ALL RIGHTS RESERVED
 *******************************************************************************************/
/**
 * \defgroup    EVT_I2CCOMM I2CCOMM Event
 * \ingroup EVENT_HANDLER
 * \brief   I2CCOMM EVENT declaration
 */

#ifndef EVENT_HANDLER_EVT_I2CCOMM_EVT_I2CCOMM_H_
#define EVENT_HANDLER_EVT_I2CCOMM_EVT_I2CCOMM_H_

#include "hx_drv_iic.h"

/****************************************************
 * Constant Definition                              *
 ***************************************************/

#define I2CCOMM_START_COMMAND          (0x01) /**< Start Command */
#define I2CCOMM_LATENCY_TEST_MODE      (0x01) /**< Latency Test Mode */
#define I2CCOMM_ACCURACY_TEST_MODE     (0x02) /**< Accuracy Test Mode */

/****************************************************
 * ENUM Declaration                                 *
 ***************************************************/
/**
 * \defgroup    I2C_COMM_ENUM   I2C Communication Library Enumeration
 * \ingroup I2C_COMM
 * \brief   Defines the required enumeration of I2C communication library.
 * @{
 */

/**
 * \enum EVT_IICCOM_ERR_E
 * \brief this enumeration use in i2c communication library, define the status of i2c communication process.
 */
typedef enum
{
    EVT_IICCOMM_NO_ERROR                 = 0,   /**< STATUS: No Error*/
    EVT_IICCOMM_ERR_DRVFAIL                     /**< STATUS: Driver Not Initialize Correctly */
} EVT_IICCOM_ERR_E;

/**
 * \enum I2CCOMM_FEATURE_E
 * \brief this enumeration use in i2c communication library, define the supported feature.
 */
typedef enum
{
    I2CCOMM_FEATURE_MODE                 = 0x00, /**< Set test mode*/
    I2CCOMM_FEATURE_ITERATIONS           = 0x01, /**< Set iterations for test mode*/
    I2CCOMM_FEATURE_MODEL                = 0x02, /**< Set model for test mode*/
    I2CCOMM_FEATURE_INPUT                = 0x03, /**< Set input for test mode*/
    I2CCOMM_FEATURE_STATUS               = 0x04, /**< Status*/
    I2CCOMM_FEATURE_CMD                  = 0x05, /**< Command*/
    I2CCOMM_FEATURE_LATENCY_RESULT       = 0x06, /**< Latency Result*/
    I2CCOMM_FEATURE_ACCURACY_RESULT      = 0x07, /**< Accuracy Result*/
} I2CCOMM_FEATURE_E;


#endif /* EVENT_HANDLER_EVT_I2CCOMM_EVT_I2CCOMM_H_ */
