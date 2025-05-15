/*
 * Copyright (c) 2025 PaySoundBox. All rights reserved.
 *
 * This software is confidential and proprietary to PaySoundBox.
 * Unauthorized use, reproduction, distribution, or disclosure
 * of this software outside the company premises is strictly
 * prohibited without prior written consent.
 *
 * No part of this software may be copied, modified, or
 * distributed for any purpose other than as expressly
 * permitted by PaySoundBox.
 *
 * File Name        : PaySoundBoxMain.cpp
 * File Description : Main implementation for PaySoundBox
 * Author           : Santhosh G
 * Date Created     : March 8, 2025
 *
 * ============
 * Revision History:
 * ============
 * Date         Author             Description of Changes
 * -----        -------            ----------------------
 * 08Mar25      Santhosh G         Initial creation of PaySoundBoxMain.cpp.
 *
 */

/*
 * Function: setup
 * Purpose: Initilisaton of all function.
 * Parameters: void.
 * Returns: void
 */


 /*===================================== Includes =====================================*/
#include "datatypes.h"
#include "prjcfg.h"
// Libraries
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

//#include <esp_system.h>  // Make sure this is included

#ifdef ENABLE_ARDINOUNO_JSON
#include <ArduinoJson.h>
#endif /* ENABLE_ARDINOUNO_JSON */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef ENABLE_PING_MODULE
//#include <Ping.h>
#endif /* ENABLE_PING_MODULE */

#ifdef ENABLE(TFT_eSPI)
/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
 *   *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
 *   *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
 *   FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd/ or https://RandomNerdTutorials.com/esp32-tft/
 */
#include <TFT_eSPI.h>  // Include the TFT_eSPI library
#endif /* ENABLE_TFT_eSPI */

#ifdef ENABLE_PUB_SUB_CLIENT
#include <PubSubClient.h> // Lib version - 2.8
#endif /* ENABLE_PUB_SUB_CLIENT */

#ifdef ENABLE_SECURE_WIFI_CONNECT
#include <WiFiClientSecure.h>  // For HTTPS
#endif /* ENABLE_SECURE_WIFI_CONNECT */

#if 1 //def ENABLE_QRCODE_MODULE
#include "qrcode.h"  // Include Espressif QR Code library header
#endif /* ENABLE_QRCODE_MODULE */

#ifdef ENABLE_MATRIX_KEYPAD
#include <Keypad.h>
#endif /* ENABLE_MATRIX_KEYPAD */



#ifdef TINY_GSM_MODEM_EC200U
  #ifdef ENABLE_TINY_GSM_MODULE
    #include <TinyGsmClient.h>
  #else
    #include <HardwareSerial.h>
  #endif /* ENABLE_TINY_GSM_MODULE */
#endif /* TINY_GSM_MODEM_EC200U */

#include "SoundBoxProcessMgr.h"

/*===================================== Macros =======================================*/


/*===================================== Type Definitions ============================*/


typedef enum tageSoundBoxMsgEventEnum
{
    /* 0x00 */ PSB_PAY_UPI
  , /* 0x01 */ PSB_PAY_NFC_CARDLESS
  , /* 0x02 */ PSB_PAY_HISTORY_STATUS

}PSB_MSG_EVENT_ENUM;

typedef struct tageSoundBoxMsgEventStruct
{
    /*{{{ WORD_START_01 */
     PSB_MSG_EVENT_ENUM     enEvent;
    /* WORD_END_01 }}} */

    /*{{{ WORD_START_02 */
     float                  f32amount;
    /* WORD_END_02 }}} */

     /*{{{ WORD_START_03 */
     unsigned int bui5CurrentType : 6;
     unsigned int bui5Reserved    : 26;
    /* WORD_END_03 }}} */

}PSM_MSG_EVENT_STRUCT;

/*===================================== Global Variables ===========================*/


#ifdef ENABLE_SECURE_WIFI_CONNECT
const short int cgpi16MqttPort = 8883; // SSL/TLS port for Adafruit IO
#else
const short int cgpi16MqttPort = 1883; // Default MQTT port
#endif /* ENABLE_SECURE_WIFI_CONNECT */

// Define Task Handles
TaskHandle_t gtskMainProcessHndl;
TaskHandle_t gtskKeyboardHndl;
TaskHandle_t gtskDisplayHndl;

/*===================================== Private Variables ===========================*/

static const char * gscpcFileName = "PAYSOUND.INO";


/*===================================== Function Prototypes =========================*/


/*===================================== Function Definitions ========================*/

/*
 *
 *
 *
 * Reference Video Url :
 * https://www.youtube.com/watch?v=S027n6r4mSo
 *
 *
 *
 * -------------------------------------------------
 *      Payment Gateway
 * ------------------------------------------------
 * Google Pay: No fees for UPI payments. Best if you're focusing on mobile app payments or simple UPI links
 * Razorpay: A great option for businesses, as it supports multiple payment methods (including UPI) and offers
 * easy integration.
 *
 *  URL : https://github.com/razorpay/razorpay-node.git
 * -------------------------------------------------
 *      Library Used
 * ------------------------------------------------
 * PubSubClient v2.8 by Nick O'Leary
 *
 *
 * Project: MQTT Communication using EC200U-CN Module
 * Description: This project demonstrates the use of the EC200U-CN module for MQTT communication.
 *              The code establishes an MQTT connection and publishes data at regular intervals.
 *
 * Features:
 * - Publishes incrementing counter data to a specified MQTT topic.
 *
 * Requirements:
 * - Ensure correct APN settings for your network provider.
 * - Replace MQTT server, username, password, and topics with your broker's configuration.
 *
 * Hardware Connections:
 * - ESP32:
 *   - RX2 (16) (ESP32) -> TX (EC200U-CN)
 *   - TX2 (17) (ESP32) -> RX (EC200U-CN)
 *   - GND (ESP32) -> GND (EC200U-CN)

 * Author and credits: Sachin Soni
 * YouTube: Check out tech tutorials and projects at **techiesms**: https://www.youtube.com/techiesms
 */

/*{{{ setup()*/
void setup()
{
  Serial.begin(115200);  // Start the Serial communication
  while (!Serial);       // Wait for serial monitor

  InitAllmodules();

}
/*}}}*/

void loop()
{
  MainRunLoop();
}
