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

#ifdef ENABLE_QRCODE_MODULE
#include <QRCode.h>     // QRCode library for generating QR codes
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

/*===================================== Macros =======================================*/

/*{{{ Pay Sound Box thread configuration */
#define PAY_SOUND_BOX_MANAGER_THREAD_NAME       "PSBMGR" 
#define PAY_SOUND_BOX_MANAGER_STACK_SIZE        ( 2 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Pay Sound Box thread configuration */
#define PSB_WORKER_THREAD_NAME       "WORKER" 
#define PAY_WORKER_STACK_SIZE        ( 1 * 1024 )  /* 1Kb */
/*}}}*/

/*{{{ Keypad thread configuration */
#define KEYPAD_THREAD_NAME       "KEYPAD" 
#define KEYPAD_STACK_SIZE        ( 2 * 1024 )  /* 1Kb */
/*}}}*/

#define MAX_UNIQUE_ID_LEN       ( 20 )
#define MAX_MQTT_TOPIC_SUBSCRIBE_LEN       ( 128 )


#define MQTT_SUBS_TOPIC_PAYMENTS_STATUS       "paymentstatus"
/*===================================== Type Definitions ============================*/


typedef enum tageSoundBoxMsgEventEnum
{
    /* 0x00 */ PSB_PAY_UPI
  , /* 0x01 */ PSB_PAY_NFC_CARDLESS
     
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
const char* cgpcSSID = "SLN 1st Floor";
const char* cgpcWifiPassword = "Sln@1111";

// MQTT broker details
const char*  cgpcMqttServer = "192.168.0.0"; // or hostname
const short int cgpi16MqttPort = 1883; // Default MQTT port

const char* cgpcMqttUser = "SanthoshG"; // Optional
const char* cgpcMqttPassword = "6379175223"; // Optional


char gacUniqueID [ MAX_UNIQUE_ID_LEN ]; // Buffer to store the unique ID

// Global variables for wifi management
bool gbIsWifiConnected = false;
unsigned long gi32WifiDisconnectTime = 0;

// Create an instance of the TFT_eSPI class
TFT_eSPI gTftDisplayHndl = TFT_eSPI();  

WiFiClient gWifiespClient;

#ifdef ENABLE_PUB_SUB_CLIENT
PubSubClient gMQTTClient(gWifiespClient);
#endif /* ENABLE_PUB_SUB_CLIENT */

#ifdef ENABLE_MATRIX_KEYPAD
// Keypad setup (4x4 matrix)
const byte ROW_NUM = 4;
const byte COLUMN_NUM = 4;
char keys[ROW_NUM][COLUMN_NUM] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
byte pin_rows[ROW_NUM] = {23, 22, 21, 19}; // Adjust to your wiring
byte pin_column[COLUMN_NUM] = {18, 5, 4, 0}; // Adjust to your wiring
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

#endif /* ENABLE_MATRIX_KEYPAD */


// Define Task Handles
TaskHandle_t gtskMainProcessHndl;
TaskHandle_t gtskKeyboardHndl;
TaskHandle_t gtskDisplayHndl;

// Message Queue Setup for communication between threads
QueueHandle_t msgQueue;
#define QUEUE_SIZE 10
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
    snprintf(gacUniqueID, sizeof(gacUniqueID), "%llX", ui64chipId ); // Format as hexadecimal
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

// Function to handle Wi-Fi scan and send the result as an HTTP response
void GetWifiScanList( ) 
{
  int networksFound = WiFi.scanNetworks();
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: application/json\r\n";
  response += "Connection: close\r\n\r\n";  // End of headers
  
  // Create the JSON response body
  response += "{ \"status\": \"success\", \"networks\": [";
  
  int i32MaxNetworks = min(networksFound, 15);  // Limit to 15 networks
  
  for (int i = 0; i < i32MaxNetworks; i++) 
  {
    response += "{";
    response += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
    response += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    response += "\"encryption\":\"" + getEncryptionType(WiFi.encryptionType(i)) + "\"";
    response += "}"; // end of network object
    if (i < i32MaxNetworks - 1) response += ",";  // Add comma if not the last network
  }

  response += "] }";  // End of JSON body
  
  // Send the response to the client
  //client.print(response);
  printf("RESPONSE[%s]\n", response.c_str( ) );
}

String getEncryptionType(wifi_auth_mode_t authMode) 
{
  switch ( authMode ) 
  {
    case WIFI_AUTH_OPEN: return "Open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    default: return "Unknown";
  }
}

#ifdef ENABLE_PING_MODULE
// Function to check internet connection by connecting to a host and port
bool pingTest(const char* host)
{
  uint16_t port = 53;  // Port 53 is the DNS port

  Serial.print("Connecting to ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  // Try to connect to the remote IP and port
  if (gWifiespClient.connect(host, port))
  {
    Serial.println("[INTERNET] Connection successful!");
    gWifiespClient.stop();  // Close the connection after successful test
    return true;  // Return true if the connection is successful
  } 
  else
  {
    Serial.println("[INTERNET] Connection failed!");
    return false;  // Return false if the connection failed
  }
}

#endif /* ENABLE_PING_MODULE */

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
#ifdef ENABLE_PUB_SUB_CLIENT
/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ MQTT_Reconnect() */
boolean MQTT_Reconnect( void )
{
  boolean  bRetMqttConnectStatus = true;

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

      bRetMqttConnectStatus = false;
    }
  }
  
  return bRetMqttConnectStatus;
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread00() */
void MQTT_ResponseCallback( char* pcMqttSubcTopic, byte* pcPayload, unsigned int ui32Length ) 
{
  Serial.print("Message arrived [");
  Serial.print(pcMqttSubcTopic);
  Serial.print("] ");

  for ( int i = 0; i < ui32Length; i++ ) 
  {
    Serial.print((char)pcPayload[i]);
  }
  Serial.println();

  // Handle incoming messages (e.g., commands)

}
/*}}*/
#endif /* ENABLE_PUB_SUB_CLIENT */


void GenerateAndDisplayQR( String upiData ) 
{
  int size = MAX_QR_GENERATOR_SIZE;
  QRCode qrcode;
  
  // Create QR code from the UPI string
  qrcode.init();
  qrcode.setData(upiData.c_str());
  qrcode.generate();

  // Display the QR code on the TFT screen
  int offsetX = (gTftDisplayHndl.width() - size) / 2;  // Center the QR code
  int offsetY = (gTftDisplayHndl.height() - size) / 2;

  gTftDisplayHndl.fillRect(0, 0, gTftDisplayHndl.width(), gTftDisplayHndl.height(), TFT_WHITE);  // Clear the screen
  
  for ( int y = 0; y < size; y++ ) 
  {
    for ( int x = 0; x < size; x++ ) 
    {
      if (qrcode.getModule(x, y)) 
      {
        gTftDisplayHndl.fillRect(offsetX + x * 2, offsetY + y * 2, 2, 2, TFT_BLACK);  // Draw QR code module
      }
    }
  }

  Serial.println("UPI QR Code generated and displayed.");
}

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ CreateUPIString() */
String CreateUPIString( unsigned int ui32Amount )
{

  float amount  = ui32Amount; 
  
  // Format the UPI string
  String upiString = "upi://pay?pa=" + upiID + "&pn=Merchant&mc=123456&tid=1234567890&tr=1234567890&tn=Payment%20for%20goods&am=";
  upiString += String( amount, 2 );  // Append the amount to the UPI string
  upiString += "&cu=INR&url=https://www.merchantwebsite.com";  // Add currency and URL
  return upiString;
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

unsigned int GetUserValueFromKeypad( void )
{
  unsigned int ui32RetUserValue = -1;
  
#ifdef ENABLE_USER_INPUT_CONSOLE_FOR_TEST
  while ( Serial.available() == 0 )
  {
    delay(100);
  }
  
  ui32RetUserValue = Serial.parseInt();
#else
  char key = keypad.getKey();
#endif /* ENABLE_USER_INPUT_CONSOLE_FOR_TEST */  

  return ui32RetUserValue;
}
/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread00() */
void PSB_PaySoundBoxManagerThread ( void *pvParameters )
{
  Message msg;
  int ui32Event = 0;
  
  initDisplay();

  unsigned ui32TransAmount = 0;

  while ( true )
  {      
    //if (xQueueReceive(msgQueue, &msg, portMAX_DELAY))
    {
          ui32Event = GetUserValueFromKeypad( );
        
          switch ( ui32Event )
          {
            case PSB_PAY_UPI:
            {

              ui32TransAmount = GetUserValueFromKeypad( );
                          
              PSB_DEBUG_PRINT(( "%s %u > $PSB-MN-SBMGR$AMT[%u]", __DRV_CONSOLE_LINE__
                        , gscpcFileName
                        , __LINE__

                        , ui32TransAmount
                        ));

              // Generate UPI QR Code
              String upiQRData = CreateUPIString( ui32TransAmount );
  
              GenerateAndDisplayQR( upiQRData );           
            }
              break; /* PSB_PAY_UPI */
      
            case PSB_PAY_NFC_CARDLESS:
            {
              Serial.println("Bad client ID error");
            }
              break; /* PSB_PAY_NFC_CARDLESS */
              
            case PSB_PAY_HISTORY_STATUS:
            {
              Serial.println("Last payment history status");
            }
              break; /* PSB_PAY_HISTORY_STATUS */
                            
            default:
            {
              Serial.println("Unknown error code");
            }
              break; /* default */          
          }
   
    //Serial.println("Sound Box Main thead...");
    //vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}
/*}}*/
#ifdef ENABLE_MATRIX_KEYPAD
/*
 * Args    : void *
 * Return  : void
 * Description : Keyboard Monitor Task: Wait for input and send messages to main thread
 */
/*{{ PSB_KeyboardMonitorTask() */
void PSB_KeyboardMonitorTask(void *pvParameters)
{
    while ( true ) 
    {
        char key = keypad.getKey();
        if ( key ) 
        {
            Message msg;
            msg.key = key;
            // Send the key to the main thread via the message queue
            xQueueSend(msgQueue, &msg, portMAX_DELAY);
        }
        delay(100);
    }
}
/*}}*/
#endif /* ENABLE_MATRIX_KEYPAD */

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
  WiFi.mode(WIFI_STA);
  WiFi.enableSTA(true);
  WiFi.begin(cgpcSSID, cgpcWifiPassword);

  unsigned long startMillis = millis();
  
  while ( WiFi.status() != WL_CONNECTED ) 
  {
     // Timeout after 10 seconds
    if (millis() - startMillis > 10000) 
    { 
      Serial.println(F("[WIFI] connection timeout"));
      return;  // Exit if connection times out
    }
    
    delay(500);
    Serial.print(".");
  }
  Serial.print(F("[WIFI] connected, IP: "));
  Serial.println(WiFi.localIP());
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
    delay(1000);  // Optional delay before reconnecting
    PSB_WifiSetup();
  }
}
/*}}*/

/*
 * Args    : void
 * Return  : void
 * Description :Function to initialize the display
 */
/*{{ initDisplay() */
void initDisplay()
{
    // Initialize TFT display
    gTftDisplayHndl.init();

    /* older initlise method like ST7735 or ILI9341 when the library is using specific configuration files */
    //gTftDisplayHndl.begin();
    
    gTftDisplayHndl.setRotation(1);  // Adjust screen orientation

     /* Clear the screen */
    gTftDisplayHndl.fillScreen(TFT_WHITE);

   
    //gTftDisplayHndl.setTextColor(TFT_WHITE); 
}
/*}}*/

/*
 * Args    : void
 * Return  : void
 * Description :Function to initialize the display
 */
/*{{ initDisplay() */
void drawTextSync(const String & strUsertext)
{
    gTftDisplayHndl.fillScreen(TFT_BLACK);
    gTftDisplayHndl.drawString(strUsertext, 10, 10);
}
/*}}*/

/*{{{ setup()*/
void setup() 
{
  Serial.begin(115200);  // Start the Serial communication
  while (!Serial);       // Wait for serial monitor

  GetWifiScanList();
  
#ifdef ENABLE_WIFI_MODULE
  /* Setup Wifi*/
  PSB_WifiSetup( );
#endif /* ENABLE_WIFI_MODULE */

#if 0
#ifdef ENABLE_PUB_SUB_CLIENT
  gMQTTClient.setServer( cgpcMqttServer, cgpi16MqttPort );
  gMQTTClient.setCallback( MQTT_ResponseCallback );
#endif

  // Create message queue
  msgQueue = xQueueCreate( QUEUE_SIZE, sizeof( Message ) );

  // Create FreeRTOS tasks
  xTaskCreate(PSB_ProcessThread00, "Task1", 1000, NULL, 1, NULL);
  xTaskCreate(PSB_ProcessThread01, "Task2", 1000, NULL, 1, NULL);

  xTaskCreate( PSB_KeyboardMonitorTask
              , KEYPAD_THREAD_NAME
              , KEYPAD_STACK_SIZE
              , NULL
              , 1
              , &gtskKeyboardHndl
              );
#endif
  /* Pay Sound Box Manager thread create */
  xTaskCreate(  PSB_PaySoundBoxManagerThread
              , PAY_SOUND_BOX_MANAGER_THREAD_NAME
              , PAY_SOUND_BOX_MANAGER_STACK_SIZE
              , NULL
              , 1
              , &gtskMainProcessHndl );
            
}
/*}}}*/

void loop()
{
 
#ifdef ENABLE_WIFI_MODULE
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

    if (pingTest("8.8.8.8"))
    {
      Serial.println("[INTERNET] Connection is active");
    }
    else
    {
      Serial.println("[INTERNET] No internet connection");
    }
     
     delay(5000);  // Check every 10 seconds
  }
#endif /* ENABLE_WIFI_MODULE */
}
