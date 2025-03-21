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

#ifdef ENABLE_ARDINOUNO_JSON
#include <ArduinoJson.h>
#endif /* ENABLE_ARDINOUNO_JSON */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef ENABLE(TFT_eSPI)
#include <TFT_eSPI.h>  // Include the TFT_eSPI library
#endif /* ENABLE_TFT_eSPI */

#ifdef ENABLE_PUB_SUB_CLIENT
#include <PubSubClient.h> // Lib version - 2.8
#endif /* ENABLE_PUB_SUB_CLIENT */

/*===================================== Macros =======================================*/

/*{{{ Pay Sound Box thread configuration */
#define PAY_SOUND_BOX_MANAGER_THREAD_NAME       "PSBMGR" 
#define PAY_SOUND_BOX_MANAGER_STACK_SIZE        ( 2 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Pay Sound Box thread configuration */
#define PSB_WORKER_THREAD_NAME       "WORKER" 
#define PAY_WORKER_STACK_SIZE        ( 1 * 1024 )  /* 1Kb */
/*}}}*/

#define MAX_UNIQUE_ID_LEN       ( 20 )
#define MAX_MQTT_TOPIC_SUBSCRIBE_LEN       ( 128 )


#define MQTT_SUBS_TOPIC_PAYMENTS_STATUS       "paymentstatus"
/*===================================== Type Definitions ============================*/


/*===================================== Global Variables ===========================*/
const char* cgpcSSID = "PSBOX";
const char* cgpcWifiPassword = "12345678";

// MQTT broker details
const char*  MqttServer = "192.168.0.0"; // or hostname
const short int cgpi16MqttPort = 1883; // Default MQTT port

const char* cgpcMqttUser = "SanthoshG"; // Optional
const char* cgpcMqttPassword = "6379175223"; // Optional


char gacUniqueID [ MAX_UNIQUE_ID_LEN ]; // Buffer to store the unique ID

// Global variables for wifi management
boolean gbIsWifiConnected = false;
long gi32WifiDisconnectTime = 0;

// Create an instance of the TFT_eSPI class
TFT_eSPI tft = TFT_eSPI();  

#ifdef ENABLE_PUB_SUB_CLIENT
WiFiClient gWifiespClient;
PubSubClient gMQTTClient(gWifiespClient);
#endif /* ENABLE_PUB_SUB_CLIENT */

/*===================================== Private Variables ===========================*/


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
 */

 /*
 * Args    : long ling int64 => unique ID MAC address or other.
 * Return  : Success or failure status.
 * Description :
 */
/*{{ PSB_GetUniqueID() */
UINT64 PSB_GetUniqueID( void )
{
  UINT64 ui64chipId = ESP.getEfuseMac();  // Get the chip's MAC address (unique ID)

  return ui64chipId;
}
/*}}*/

/*
 * Args    : long ling int64 => unique ID MAC address or other.
 * Return  : Success or failure status.
 * Description :
 */
/*{{ PSB_GetUniqueIDString() */
char* PSB_GetUniqueIDString( void )
{
  UINT64 ui64chipId = ESP.getEfuseMac();  // Get the chip's MAC address (unique ID)

  if( '\0' == gacUniqueID [0] )
  {
    snprintf(gacUniqueID, sizeof(gacUniqueID), "%llX", chipid); // Format as hexadecimal
    Serial.print("gacUniqueID ID: ");
    Serial.println(gacUniqueID);
  }

  return gacUniqueID;
}
/*}}*/

/*
 * Args    : int ui32WifiLinkProcess  => reserved
 * Return  : Success or failure status.
 * Description :
 */
/*{{ WifiLinkProcess() */
BOOL WifiLinkProcess( void )
{
  BOOL  bReturnStatus = PSB_FALSE;

  WiFi.mode(WIFI_STA);

  //WiFi.disconnect();

    // wait for WiFi connection
  WiFi.begin(cgpcSSID, cgpcWifiPassword);
  Serial.print("Connecting to ");
  Serial.println(cgpcSSID);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  return bReturnStatus;
}
/*}}*/

// Error Codes
#define MQTT_ERR_CODE_UNATHORIZED           -7
#define MQTT_ERR_CODE_BAD_CLIENT_ID         -6
#define MQTT_ERR_CODE_DISCONNECT            -5
#define MQTT_ERR_CODE_CONNECT_FAIL          -4
#define MQTT_ERR_CODE_CONNECT_LOST          -3
#define MQTT_ERR_CODE_TIMEOUT               -2
#define MQTT_ERR_CODE_UNKNOWN               -8

/*
 * Args    : MQTT State
 * Return  : void
 * Description :
 */
/*{{ MQTT_HandleErrorCode() */
void MQTT_HandleErrorCode( int errorCode )
{
    switch (errorCode)
    {
      case MQTT_ERR_CODE_UNATHORIZED:
      {
        Serial.println("Unauthorized error");
      }
        break; /* MQTT_ERR_CODE_UNATHORIZED */

      case MQTT_ERR_CODE_BAD_CLIENT_ID:
      {
        Serial.println("Bad client ID error");
      }
        break; /* MQTT_ERR_CODE_BAD_CLIENT_ID */

      case MQTT_ERR_CODE_DISCONNECT:
      {
        Serial.println("Disconnect error");
      }
        break; /* MQTT_ERR_CODE_DISCONNECT */

      case MQTT_ERR_CODE_CONNECT_FAIL:
      {
        Serial.println("Connection failed");
      }
        break; /* MQTT_ERR_CODE_UNATHORIZED */

      case MQTT_ERR_CODE_CONNECT_LOST:
      {
        Serial.println("Connection lost");
      }
        break; /* MQTT_ERR_CODE_UNATHORIZED */

      case MQTT_ERR_CODE_TIMEOUT:
      {
        Serial.println("Timeout error");
      }
        break; /* MQTT_ERR_CODE_UNATHORIZED */

      default:
      {
        Serial.println("Unknown error code");
      }
        break; /* default */          
    }
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ MQTT_Reconnect() */
boolean MQTT_Reconnect( void )
{
  boolean  bRetMqttConnectStatus = false;

  while ( ! gMQTTClient.connected() )
  {
    Serial.print("Attempting MQTT connection...");

    if ( gMQTTClient.connect( gacUniqueID, cgpcMqttUser, cgpcMqttPassword ) )
    { 
      char acMQttSubscribeTopic[ MAX_MQTT_TOPIC_SUBSCRIBE_LEN ];

      strcpy( acMQttSubscribeTopic, PSB_GetUniqueIDString( ) );

      strcat( acMQttSubscribeTopic, MQTT_SUBS_TOPIC_PAYMENTS_STATUS );

      Serial.println("connected");

      gMQTTClient.subscribe(acMQttSubscribeTopic); //subscribe to your topic
    } 
    else
    {
      Serial.print("failed, rc=");
      Serial.print( gMQTTClient.state() );
      
      MQTT_HandleErrorCode( gMQTTClient.state() );

      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
  
  rerurn bRetMqttConnectStatus;
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread00() */
void PSB_ProcessThread00(void *pvParameters)
{
  for (;;)
  {
    Serial.println("Task 1 is running...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread00() */
void PSB_PaySoundBoxManagerThread ( void *pvParameters )
{
  while ( 1 )
  {
    Serial.println("Sound Box Main thead...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread01() */
void PSB_ProcessThread01(void *pvParameters)
{
  for (;;) 
  {
    Serial.println("Task 2 is running...");
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay for 0.5 second
  }
}
/*}}*/

/*
 * Args    : int ui32WifiLinkProcess  => reserved
 * Return  : Success or failure status.
 * Description :
 */
/*{{ PSB_WifiConnect() */
void PSB_WifiSetup( void )
{
  Serial.println(F("[WIFI] Initialization"));
  WiFi.mode( WIFI_STA );
  WiFi.enableSTA( true );
  WiFi.begin( cgpcSSID, cgpcWifiPassword );
}
/*}}*/

/*
 * Args    : void
 * Return  : Success or failure status.
 * Description :
 */
/*{{ PSB_WifiResetIfTimeout() */
void PSB_WifiResetIfTimeout( void )
{
  // Force reconnect if fail
  if( ( millis() - gi32WifiDisconnectTime ) > WIFI_CONNECTION_TIMEOUT )
  {
    gi32WifiDisconnectTime = millis();
    Serial.println(F("[WIFI] connection timeout"));
    WiFi.disconnect(true);
    PSB_WifiSetup();
  }
}
/*}}*/

/*{{{ setup()*/
void setup() 
{
  Serial.begin(115200);  // Start the Serial communication
  while (!Serial);       // Wait for serial monitor

  /* Setup Wifi*/
  PSB_WifiSetup( );

  // Create FreeRTOS tasks
  xTaskCreate(PSB_ProcessThread00, "Task1", 1000, NULL, 1, NULL);
  xTaskCreate(PSB_ProcessThread01, "Task2", 1000, NULL, 1, NULL);

  /* Pay Sound Box Manager thread create */
  xTaskCreate(  PSB_PaySoundBoxManagerThread
              , PAY_SOUND_BOX_MANAGER_THREAD_NAME
              , PAY_SOUND_BOX_MANAGER_STACK_SIZE
              , NULL
              , 1
              , NULL );
}
/*}}}*/

void loop()
{
 
  if( WiFi.status() != WL_CONNECTED )
  {
    if(gbIsWifiConnected)
    {
      // Acknowledge disconnection
      gbIsWifiConnected = false;
      gi32WifiDisconnectTime = millis();
      Serial.println(F("[WIFI] disconnected"));
    }

    PSB_WifiResetIfTimeout( );
  }  
  else
  {
    // Wifi connected
    if( !gbIsWifiConnected )
    {
      // Acknowledge connection
      gbIsWifiConnected = true;
      Serial.print(F("[WIFI] connected, IP: "));
      Serial.println(WiFi.localIP());
    }

  }
}
