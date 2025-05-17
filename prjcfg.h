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

/* Enable the wifi secure connect  */
//#define ENABLE_SECURE_WIFI_CONNECT 1

#define ENABLE_HARD_CODE_CERTIFICATE 1

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
#define ENABLE_PUB_SUB_CLIENT          1

/* Enable the  MQTT CLIENT  */
#define ENABLE_MATRIX_KEYPAD          1

/* Enable the  Tiny GSM CLIENT  */
#define ENABLE_TINY_GSM_MODULE          1

/* Enable the  wifi enable module  */
#define ENABLE_WIFI_MODULE          1

/* Enable the  ping for testing module  */
#define ENABLE_PING_MODULE          1

/* Enable the  ping for testing module  */
//#define ENABLE_QRCODE_MODULE          1

/* Enable the  for user input console instead of using keypad  */
#define ENABLE_USER_INPUT_CONSOLE_FOR_TEST          1

// Define modem interface
#define TINY_GSM_MODEM_EC200U 

//#define RTOS_THREAD_TEST_ODD_EVEN   1

/*===================================== GPIO Pin Macro ===========================================() */

#define TFT_CS     5
#define TFT_RST    17
#define TFT_DC     16

// QR Code size settings
#define MAX_QR_GENERATOR_SIZE 200

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

/*{{{ Pay Sound Box thread configuration */
#define PAY_SOUND_BOX_MANAGER_THREAD_NAME       "PSBMGR"
#define PAY_SOUND_BOX_MANAGER_STACK_SIZE        ( 6 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Mqtt Worker thread configuration */
#define MQTT_WORKER_THREAD_NAME       "MQTTWK"
#define MQTT_WORKER_STACK_SIZE        ( 8 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Mqtt Worker thread configuration */
#define LTE_WORKER_THREAD_NAME       "LTEWK"
#define LTE_WORKER_STACK_SIZE        ( 10 * 1024 )  /* 1Kb */
#define LTE_TASK_PRIORITY   1
/*}}}*/

/*{{{ Mqtt Worker thread configuration */
#define NW_WORKER_THREAD_NAME       "NWWK"
#define NW_WORKER_STACK_SIZE        ( 12 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Pay Sound Box thread configuration */
#define PSB_WORKER_THREAD_NAME       "WORKER"
#define PAY_WORKER_STACK_SIZE        ( 4 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Keypad thread configuration */
#define KEYPAD_THREAD_NAME       "KEYPAD"
#define KEYPAD_STACK_SIZE        ( 4 * 1024 )  /* 1Kb */
/*}}}*/

#define MAX_UNIQUE_ID_LEN       ( 20 )
#define MAX_MQTT_TOPIC_SUBSCRIBE_LEN       ( 128 )


#define MQTT_SUBS_TOPIC_PAYMENTS_STATUS       "paymentstatus"


#define TINY_GSM_MODEM_BG96
#define TINY_GSM_DEBUG Serial

// Define the RX and TX pins for the modem.  Make sure these match *your* hardware.
#define MODEM_RX 17
#define MODEM_TX 16


#define EVT_WIFI_CONNECTED (1 << 0)
#define EVT_4G_CONNECTED   (1 << 1)
#define EVT_MQTT_CONNECTED (1 << 2)
#define EVT_WIFI_CONNECTING (1 << 3) // Add connecting states
#define EVT_4G_CONNECTING   (1 << 4)
#define EVT_INTERNET_CONNECTED (EVT_WIFI_CONNECTED | EVT_4G_CONNECTED) //helper



#define ARRAY_LENGTH( __array_base__ )      ( sizeof( __array_base__ ) / sizeof( __array_base__[0] ) )

/*=====================================typedef=====================================================() */

// Struct to send data through the message queue
typedef struct tageMessageInfoStruct
{
    char cKey;  // For text characters, we use `char`
    char acReserved[3];
    float f32Amount;
} MESSAGE_INFO_STRUCT;
/*=====================================Global Variable=============================================() */

/*=====================================Private Variable============================================() */

/*=====================================Function Prototype==========================================() */

/*=====================================Function Defination=========================================() */




#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PROJECT_CONFIGURATION_H__ */
