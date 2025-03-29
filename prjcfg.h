/*
 * Copyright (c) 2025 SoundBoxPay
 * All rights reserved.
 *
 * This software is confidential and proprietary to SoundBoxPay.
 * The use, reproduction, distribution, or disclosure of this software outside
 * of the company premises is strictly prohibited without prior written consent.
 *
 * No part of this software may be copied, modified, or distributed for any
 * purpose other than as expressly permitted by Your Company Name.
 *
 * File Name        : prjcfg.h
 * File Description : project configuration
 * Author           : SanthoshG
 * Date             : 05Mar25
 * 
 * =========
 * History :
 * =========
 * Date         Author              Created/Modified Details 
 * -----        -------             ------------------------
 * 05Mar25      SanthoshG           Project configuration and supporting feature enable and disable           
 */

#ifndef __PROJECT_CONFIGURATION_H__
#define __PROJECT_CONFIGURATION_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*=====================================Include====================================================() */

/*=====================================Define====================================================() */

/* Enable the wifi support feature */
#define ENABLE_WIFI_SUPPORT         1

/* Enable the audio controller/Speaker/auido amplifier module */
#define ENABLE_AUDIO_SUPPORT         1

/* Enable the arduino json format */
#define ENABLE_ARDINOUNO_JSON           1

/* Enable the  tft display */
#define ENABLE_TFT_DISPLAY           1

/* Enable the  web server */
#define ENABLE_WEB_SERVER          1

/* Enable the  chat gpt  */
#define ENABLE_CHAT_GPT          1

/* Enable the  Free RTOS  */
#define ENABLE_FREE_RTOS          1

/* Enable the  TST eSPI DISPLAY  */
#define ENABLE_TFT_eSPI          1

/* Enable the  MQTT CLIENT  */
//#define ENABLE_PUB_SUB_CLIENT          1

/* Enable the  MQTT CLIENT  */
//#define ENABLE_MATRIX_KEYPAD          1

/* Enable the  Tiny GSM CLIENT  */
//#define ENABLE_TINY_GSM_MODULE          1

/* Enable the  wifi enable module  */
#define ENABLE_WIFI_MODULE          1

/* Enable the  ping for testing module  */
#define ENABLE_PING_MODULE          1

// Define modem interface
#define TINY_GSM_MODEM_EC200U 

/*===================================== GPIO Pin Macro ===========================================() */

#define TFT_CS     5
#define TFT_RST    17
#define TFT_DC     16

/*=====================================Macro======================================================() */

#define ENABLE(__enable_feature__)     ( defined (__enable_feature__) && __enable_feature__ )


// Wifi settings
#define WIFI_CONNECTION_TIMEOUT 30000  // 30 seconds timeout


/*{{{*/
#define PSB_MQTT_SERVER          "io.adafruit.com"
#define PSB_MQTT_SERVERPORT      1883
#define PSB_MQTT_USERNAME        "USERNAME"
#define PSB_MQTT_KEY             "ACTIVE KEY"

#define PSB_MQTT_SERVER_RETRY_COUNT        5
#define PSB_MQTT_SERVER_RETRY_DELAY     1000
/*}}}*/


/*{{{ MQTT State */

#if 1
#define MQTT_ERR_CODE_UNATHORIZED           -7
#define MQTT_ERR_CODE_BAD_CLIENT_ID         -6
#define MQTT_ERR_CODE_DISCONNECT            -5
#define MQTT_ERR_CODE_CONNECT_FAIL          -4
#define MQTT_ERR_CODE_CONNECT_LOST          -3
#define MQTT_ERR_CODE_TIMEOUT               -2

#define MQTT_ERR_CODE_UNKNOWN               -8 /* Defined by own */

#else

typedef enum tageMQTTStateErrorCode
{
    /* -7 */ MQTT_ERR_CODE_UNATHORIZED     =  -7
  , /* -6 */ MQTT_ERR_CODE_BAD_CLIENT_ID 
  , /* -5 */ MQTT_ERR_CODE_DISCONNECT    
  , /* -4 */  MQTT_ERR_CODE_CONNECT_FAIL
  , /* -3 */  MQTT_ERR_CODE_CONNECT_LOST
  , /* -2 */  MQTT_ERR_CODE_TIMEOUT 
  , /* -1 */  MQTT_ERR_CODE_UNKNOWN

} MQTT_STATE_ERR_CODE_ENUM;
#endif
/*}}}*/
/*=====================================typedef=====================================================() */

// Struct to send data through the message queue
struct Message 
{
    char key;  // For text characters, we use `char`
};
/*=====================================Global Variable=============================================() */

/*=====================================Private Variable============================================() */

/*=====================================Function Prototype==========================================() */

/*=====================================Function Defination=========================================() */




#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PROJECT_CONFIGURATION_H__ */
