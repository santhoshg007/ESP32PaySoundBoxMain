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

/*===================================== Macros =======================================*/

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
const char* cgpcSSID = "SLN 1st Floor";
const char* cgpcWifiPassword = "Sln@1111";

// MQTT broker details
const char*  cgpcMqttServer = "io.adafruit.com"; // or hostname

#ifdef ENABLE_SECURE_WIFI_CONNECT
const short int cgpi16MqttPort = 8883; // SSL/TLS port for Adafruit IO
#else
const short int cgpi16MqttPort = 1883; // Default MQTT port
#endif /* ENABLE_SECURE_WIFI_CONNECT */

const char* cgpcMqttUser = "SanthoshG"; // Optional
const char* cgpcMqttPassword = "aio_uebF37sNt31BXInllYdbEUGgeSGD"; // Optional


char gacUniqueID [ MAX_UNIQUE_ID_LEN ]; // Buffer to store the unique ID

// Global variables for wifi management
bool gbIsWifiConnected = false;
unsigned long gi32WifiDisconnectTime = 0;

// Create an instance of the TFT_eSPI class
TFT_eSPI gTftDisplayHndl = TFT_eSPI();  

#ifdef ENABLE_SECURE_WIFI_CONNECT
WiFiClientSecure gWifiespClient;  // Secure Wi-Fi client
#else
WiFiClient gWifiespClient;
#endif /* ENABLE_SECURE_WIFI_CONNECT */

String clientId = "ESP32Client";
String mqttTopicSubscribe = "my/topic/in";
String mqttTopicPublish = "my/topic/out";
String mqttMessage = "Hello MQTT from EC200U!";
// Configuration
String mqttBroker = "broker.hivemq.com";
int mqttPort = 1883;

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
TaskHandle_t gtskMqttHndl;

// Message Queue Setup for communication between threads
QueueHandle_t msgQueue;
#define QUEUE_SIZE 10

String upiID = "merchant@upi";
bool gbDisplayConnecStatus = false;

// Amazon's Root CA Certificate (used by Adafruit IO)
#ifdef ENABLE_HARD_CODE_CERTIFICATE
#if 0
const char* gcpcCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDdzCCAl+gAwIBAgIJAKz3+/T2bbSaMA0GCSqGSIb3DQEBBQUAMF0xCzAJBgNV\n" \
"BAYTAklOMQswCQYDVQQIDAJFSjELMAkGA1UEBwwCSVYxFDASBgNVBAoMC0FtYXpv\n" \
"biBDb3Jwb3JhdGlvbnMxITAfBgNVBAMMGGd1c3RyYXdpeC1jYS5hd2Ntcy5jb20w\n" \
"HhcNMjMwNjAyMTE0NzMyWhcNMjQwNjAyMTE0NzMyWjBdMQswCQYDVQQGEwJJTjEL\n" \
"MAkGA1UECAwCRUoxCzAJBgNVBAcMAlJZMRQwEgYDVQQKDAtBbWF6b24gQ29ycG9y\n" \
"YXRpb25zMR0wGwYDVQQLDBRDb3Jwb3JhdGUgU2VjdXJpdHkxHDAaBgNVBAMME2d1\n" \
"c3RyYXdpeC1jYS5hd2Ntcy5jb20wggEwDQYJKoZIhvcNAQEBBQADggEBAD5QpkFz\n" \
"DAp6k7TyaHZmckqOa7s/XuGB5rHfpAfqdkZt+nbvVtd1gbh8gfBcnqkk2gtZ6vHq\n" \
"nBz1DaXFcCuKtrckFXHqgHfJMyf3HZF8szCcnRDoToExs0eR8Biy9z/Fth04/kfC\n" \
"QyCkExOJl3uYtATqlkbk49OmyKnI5vUK6GLyY1X1Ay1tqxtmqlE6zwg64zYY2Hb0\n" \
"t4QwHz7lZ7wF5gqOo0tse7cxUSbmjzgZf3Yg3n4uD2by9UsvGnKNN0G0iADewhZG\n" \
"fvCjzF5HKhjAwTLaf7pPCE33bp3diHB4zSv7tL0i/taSdz+9N6A3PTQ0WFMpNJfM\n" \
"2lLOFjmFRw0Pj8WwgyN6gl0jbjsw1Te9HtXKnTnTmOx7zy7YGOaRA9gD0lYdso=\n" \
"-----END CERTIFICATE-----\n";
#else
// io.adafruit.com root CA
const char* gcpcCACertificate = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n";
#endif 
#endif /* ENABLE_HARD_CODE_CERTIFICATE */

#ifdef RTOS_THREAD_TEST_ODD_EVEN
// Semaphores for synchronizing odd and even tasks
SemaphoreHandle_t xSemaphoreOdd;  
SemaphoreHandle_t xSemaphoreEven;

volatile int gui32Number = 0;  // Shared variable to increment (start with 0)

#endif /* RTOS_THREAD_TEST_ODD_EVEN */

#ifdef TINY_GSM_MODEM_EC200U
//bool isNetworkConnected = false;
// Define AT Commands
const char* AT = "AT\r\n";
const char* AT_CPIN = "AT+CPIN?\r\n";  // Check SIM PIN
//const char* AT_CREG = "AT+CREG?\r\n";  // Check network registration
//const char* AT_CGATT = "AT+CGATT=1\r\n";  // Attach to the GPRS network
const char* AT_CGDCONT = "AT+CGDCONT=1,\"IP\",\"jionet\"\r\n";  // Set the APN (replace with your provider's APN)
const char* AT_CIPSTART = "AT+CIPSTART=\"TCP\",\"yourserver.com\",\"80\"\r\n";  // Start TCP connection to a server (replace with a real server)
const char* AT_CIPSEND = "AT+CIPSEND\r\n";  // Send data over the connection
const char* AT_CIPCLOSE = "AT+CIPCLOSE\r\n";  // Close the TCP connection

HardwareSerial& lteSerial = Serial1; // Or try Serial2 if Serial1 doesn't work

// Define the ESP32 pins connected to the EC200U (adjust based on your wiring)
const int lteRxPin = 16; // ESP32 RX connected to EC200U TX
const int lteTxPin = 17; // ESP32 TX connected to EC200U RX
const int powerKeyPin = 4; // Example: If the EC200U needs a power key pulse

// --- Network Configuration ---
const String APN = "jionet";
const String PING_TARGET = "8.8.8.8";
const String AT_CREG = "AT+CEREG?"; // LTE network registration command

// --- SMS Configuration ---
const String recipientNumber = "+919361674809"; // Replace with actual number
const String smsMessage = "Hello from ESP32 and EC200U!";

// --- Global Flags ---
volatile bool isLteModulePowered = false;
volatile bool isSimReady = false;
volatile bool isNetworkRegistered = false;
volatile bool isPDPContextActive = false;
volatile bool isNetworkConnected = false;

#endif /* TINY_GSM_MODEM_EC200U */
/*===================================== Private Variables ===========================*/

static const char * gscpcFileName = "PAYSOUND.INO";


/*===================================== Function Prototypes =========================*/
bool initDisplay( void );

bool sendATCommand(String command, String expectedResponse, unsigned long timeout, String *fullResponse = nullptr);
void powerOnLteModule();
bool checkSimStatus();
bool checkNetworkRegistration();
bool setupPDPContext();
bool activatePDPContext();
bool pingNetwork(String target, int contextId = 1, int timeout = 10000);
void sendSMS(String phoneNumber, String message);
bool connectToInternet();
void connectAndMonitorInternetTask(void *pvParameters);
void lteSetup();


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

// Assuming you have connected the EC200U-CN TX pin to ESP32 RX pin (e.g., GPIO16)
// and the EC200U-CN RX pin to ESP32 TX pin (e.g., GPIO17)
// and the GND pins of both modules are connected.

// Choose the ESP32 hardware serial port you want to use.
// Common options are Serial (usually connected to USB), Serial1, or Serial2.
// You might need to adjust the pin numbers depending on your ESP32 board.

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
  
  //WiFi.setTxPower(WIFI_POWER_19dBm);  // Optional: Adjust Wi-Fi transmission power

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

#if 0 //def ENABLE_PING_MODULE
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
#ifdef ENABLE_SECURE_WIFI_CONNECT
void SetClientCertificate()
{
#ifdef ENABLE_HARD_CODE_CERTIFICATE
  gWifiespClient.setCACert(gcpcCACertificate);
#else
  if (!SPIFFS.begin(true)) 
  {
    Serial.println("Failed to mount file system");
    return;
  }

  File certFile = SPIFFS.open("/AmazonRootCA.pem", "r");
  if (!certFile) 
  {
    Serial.println("Failed to open certificate file");
    return;
  }

  String cert = "";
  while (certFile.available()) {
    cert += (char)certFile.read();
  }
  certFile.close();

  gWifiespClient.setCACert(cert.c_str());
#endif  
  
}
#endif /* ENABLE_SECURE_WIFI_CONNECT */
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
      char acMQttSubscribeTopic[ MAX_MQTT_TOPIC_SUBSCRIBE_LEN ] = { 0 };
#if 0
      strcpy( acMQttSubscribeTopic, PSB_GetUniqueIDString( ) );

      strcat( acMQttSubscribeTopic, MQTT_SUBS_TOPIC_PAYMENTS_STATUS );
#else
      strcpy( acMQttSubscribeTopic, "SanthoshG/feeds/paymenttrigger" );
#endif
      Serial.println("connected:");
      Serial.println(acMQttSubscribeTopic);
      if (gMQTTClient.subscribe(acMQttSubscribeTopic))
      {
          Serial.print("Successfully subscribed to ");
          Serial.println(acMQttSubscribeTopic);
      }
      else 
      {
          Serial.print("Failed to subscribe to ");
          Serial.println(acMQttSubscribeTopic);
          bRetMqttConnectStatus = false;
      }
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
/*{{ MQTT_ResponseCallback() */
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

#ifdef TINY_GSM_MODEM_EC200U
// Function to check if the UART port is properly connected
bool checkUartConnection() {
  long start = millis();
  while (millis() - start < 2000) {  // Wait for 2 seconds for a response
    if (lteSerial.available()) {
      return true;  // UART is responsive
    }
  }
  return false;  // UART is not responding within timeout period
}
#endif 
#ifdef ENABLE_QRCODE_MODULE
// This function generates and displays a QR code
void GenerateAndDisplayQR(String upiData)
{
    // Set up the QR code configuration
    esp_qrcode_config_t config = ESP_QRCODE_CONFIG_DEFAULT();  // Use default config
    config.max_qrcode_version = 10;  // You can change this if needed
    config.qrcode_ecc_level = ESP_QRCODE_ECC_MED;  // Set error correction level (medium)

    // Generate the QR code and pass the UPI data as the text to encode
    esp_err_t err = esp_qrcode_generate(&config, upiData.c_str());

    if (err != ESP_OK) {
        Serial.println("Error generating QR Code");
        return;
    }

    // Create the QR code handle
    esp_qrcode_handle_t qrcode = (esp_qrcode_handle_t)upiData.c_str();  // Using the UPI data as the handle (assuming `esp_qrcode_generate` generates and stores QR code data)

    // Get the size of the QR code
    int size = esp_qrcode_get_size(qrcode);

    // Set offsets for positioning the QR code on the screen
    int offsetX = (gTftDisplayHndl.width() - size * 2) / 2;
    int offsetY = (gTftDisplayHndl.height() - size * 2) / 2;

    // Clear the screen before drawing the QR code
    gTftDisplayHndl.fillRect(0, 0, gTftDisplayHndl.width(), gTftDisplayHndl.height(), TFT_WHITE);

    // Loop through each module of the QR code
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            // Check if the current module is dark (black)
            if (esp_qrcode_get_module(qrcode, x, y)) {
                gTftDisplayHndl.fillRect(offsetX + x * 2, offsetY + y * 2, 2, 2, TFT_BLACK);  // Draw the black module
            }
        }
    }

    // Print a success message
    Serial.println("UPI QR Code generated and displayed.");
}
#endif /* ENABLE_QRCODE_MODULE */
// This function generates a UPI string with the given amount
String CreateUPIString(float f32Amount)
{
    String upiString = "upi://pay?pa=" + upiID + "&pn=Merchant&mc=123456&tid=1234567890&tr=1234567890&tn=Payment%20for%20goods&am=";
    upiString += String(f32Amount, 2);  // Append the amount to the UPI string
    upiString += "&cu=INR&url=https://www.merchantwebsite.com";  // Add currency and URL
    return upiString;
}

#ifdef RTOS_THREAD_TEST_ODD_EVEN
/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread00() */
void PSB_ProcessThreadOdd(void *pvParameters)
{
  Serial.print("Odd Thread Init: ");

  for (;;)
  {
    if (xSemaphoreTake(xSemaphoreOdd, portMAX_DELAY) == pdTRUE) 
    {
        Serial.print("Odd: ");
        Serial.println(gui32Number);

        // Increment the shared variable
        gui32Number++;

        // Give the semaphore to the even task
        xSemaphoreGive(xSemaphoreEven);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
      }
  }
}
/*}}*/

/*
 * Args    : void *
 * Return  : void
 * Description :
 */
/*{{ PSB_ProcessThread01() */
void PSB_ProcessThreadEven(void *pvParameters) 
{
  Serial.print("Even Thread Init: ");
  for (;;)
  {
    if (xSemaphoreTake(xSemaphoreEven, portMAX_DELAY) == pdTRUE)
    {
      Serial.print("Even: ");
      Serial.println(gui32Number);

      // Increment the shared variable
      gui32Number++;

      // Give the semaphore to the odd task
      xSemaphoreGive(xSemaphoreOdd);

      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
  }
}
/*}}*/
#endif /* RTOS_THREAD_TEST_ODD_EVEN */
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
/*{{ PSB_PaySoundBoxManagerThread() */
void PSB_PaySoundBoxManagerThread ( void *pvParameters )
{
  Message msg;
  int ui32Event = 0;
  float f32TransAmount = 0;

 

  while ( true )
  { 
#if 0
    //if (xQueueReceive(msgQueue, &msg, portMAX_DELAY))
    {
          ui32Event = GetUserValueFromKeypad( );
        
          switch ( ui32Event )
          {
            case PSB_PAY_UPI:
            {

              f32TransAmount = GetUserValueFromKeypad( );

              Serial.printf("%s %u > $PSB-MN-SBMGR$AMT[%f]\n"
                    , __DRV_CONSOLE_LINE__

                    , gscpcFileName
                    , __LINE__
                    , f32TransAmount
                    );
              // Generate UPI QR Code
              String upiQRData = CreateUPIString( f32TransAmount );

#ifdef ENABLE_QRCODE_MODULE
              GenerateAndDisplayQR( upiQRData );
#endif /* ENABLE_QRCODE_MODULE */                         
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
    }

    vTaskDelay(4000 / portTICK_PERIOD_MS);  // Delay for 1 second
#else   
      Serial.println("Sound Box Main thead...");
      vTaskDelay(10000 / portTICK_PERIOD_MS);  // Delay for 1 second
#endif     
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

void setupMQTT4G() {
  Serial.println("Setting up MQTT...");

  // Configure client ID
  sendATCommand("AT+QMTCFG=\"client\",\"" + clientId + "\"", "OK", 5000);

  // Open MQTT connection (assuming connection ID 0)
  sendATCommand("AT+QMTOPEN=0,\"" + mqttBroker + "\"," + String(mqttPort), "+QMTOPEN: 0,0", 10000);

  // Connect to MQTT broker
  sendATCommand("AT+QMTCONN=0,\"" + clientId + "\"", "OK", 10000); // No auth in this example

  // Subscribe to a topic
  sendATCommand("AT+QMTSUB=0,1,\"" + mqttTopicSubscribe + "\",0", "+QMTSUB: 0,1,0", 10000);

  Serial.println("MQTT setup complete.");
}

void publishMQTT(String topic, String message) {
  Serial.print("Publishing to topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);
  String publishCommand = "AT+QMTPUB=0,0,0,0,\"" + topic + "\"," + String(message.length());
  if (sendATCommand(publishCommand, ">", 5000)) {
    lteSerial.print(message);
    lteSerial.write(26); // Ctrl+Z to send
    sendATCommand("", "+QMTPUB: 0,1,0", 10000); // Wait for publish confirmation
  } else {
    Serial.println("Error publishing MQTT message.");
  }
}

/*
 * Args    : void *
 * Return  : void
 * Description :MQTT Task..
 */
/*{{ MQTT_MonitorTask() */
void MQTT_MonitorTask(void* parameter)
 {
#ifdef ENABLE_PUB_SUB_CLIENT
  gMQTTClient.setServer( cgpcMqttServer, cgpi16MqttPort );
  gMQTTClient.setCallback( MQTT_ResponseCallback );
#endif

  while ( true ) 
  {
    if ( !gMQTTClient.connected() )
    {
        MQTT_Reconnect( );
    }
    // Ensure the MQTT client loop runs so we can process incoming messages and keep the connection alive
    gMQTTClient.loop();    // Keep the MQTT connection alive 
    delay(10); // Small delay to prevent watchdog timeout
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
bool initDisplay( void )
{
  
  Serial.println("Init display...");

  // Try initializing the display and handle potential errors
    try {
        gTftDisplayHndl.init();
        gTftDisplayHndl.setRotation(1);
        gTftDisplayHndl.fillScreen(TFT_WHITE);
        gbDisplayConnecStatus = true;
    }
    catch (const std::exception& e) {
        Serial.println("Error initializing display: ");
        Serial.println(e.what());
        gbDisplayConnecStatus = false;
        //return;  // Return gracefully if an error occurs
    }
    return gbDisplayConnecStatus;
}
/*}}*/

/*
 * Args    : void
 * Return  : void
 * Description :Function to initialize the display
 */
/*{{ drawTextSync() */
void drawTextSync(const String & strUsertext)
{
    gTftDisplayHndl.fillScreen(TFT_BLACK);
    gTftDisplayHndl.drawString(strUsertext, 10, 10);
}
/*}}*/

// --- LTE Module Initialization API ---
void lteSetup() {
  Serial.println("Initializing LTE Module...");
  lteSerial.begin(115200, SERIAL_8N1, lteRxPin, lteTxPin);
  Serial.print("LTE Serial Initialized on RX:");
  Serial.print(lteRxPin);
  Serial.print(", TX:");
  Serial.println(lteTxPin);

  powerOnLteModule();
  isSimReady = checkSimStatus();
  if (isSimReady) {
    isNetworkRegistered = checkNetworkRegistration();
    if (isNetworkRegistered) {
      if (setupPDPContext()) {
        isPDPContextActive = activatePDPContext();
        if (isPDPContextActive) {
          isNetworkConnected = true; // Initial connection assumed successful
          Serial.println("LTE Module Initialized Successfully.");
        } else {
          Serial.println("Failed to activate PDP Context.");
        }
      } else {
        Serial.println("Failed to setup PDP Context.");
      }
      sendSMS(recipientNumber, smsMessage); // Attempt to send SMS on setup
    } else {
      Serial.println("Failed to register on the network.");
    }
  } else {
    Serial.println("SIM card not ready.");
  }
}

void TestHttpNetwork()
{
  sendATCommand("AT+QHTTPURL=26,\"http://example.com\"", "OK", 5000);
  sendATCommand("AT+QHTTPGET=80", "+QHTTPGET:", 15000);
}
// --- Helper Functions ---

// Modified sendATCommand to optionally return the full response
bool sendATCommand(String command, String expectedResponse, unsigned long timeout, String *fullResponse  /*= nullptr */) {
  Serial.print("Sending AT: ");
  Serial.println(command);
  lteSerial.println(command);
  String response = "";
  unsigned long startTime = millis();
  const unsigned long yieldInterval = 100; // Yield every 100ms

  while (millis() - startTime < timeout) {
    while (lteSerial.available()) {
      char c = lteSerial.read();
      response += c;
    }
    if (response.indexOf(expectedResponse) != -1) {
      Serial.print("Response: ");
      Serial.println(response);
      if (fullResponse != nullptr) {
        *fullResponse = response;
      }
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(yieldInterval)); // Allow other tasks to run
  }

  Serial.print("Timeout - No response or expected response not found: ");
  Serial.println(response);
  if (fullResponse != nullptr) {
    *fullResponse = response;
  }
  return false;
}

void powerOnLteModule() {
  Serial.println("Powering on LTE Module...");
  pinMode(powerKeyPin, OUTPUT);
  digitalWrite(powerKeyPin, LOW);
  delay(100);
  digitalWrite(powerKeyPin, HIGH);
  delay(1000);
  delay(5000); // Wait for boot
  isLteModulePowered = true;
  Serial.println("LTE Module Power ON sequence complete.");
}

bool checkSimStatus() {
  Serial.println("Checking SIM status...");
  return sendATCommand("AT+CPIN?", "+CPIN: READY", 5000);
}

bool checkNetworkRegistration() {
  Serial.println("Checking network registration...");
  return sendATCommand(AT_CREG, "+CEREG: 0,1", 10000) || sendATCommand(AT_CREG, "+CEREG: 0,5", 10000);
}

bool setupPDPContext() {
  Serial.println("Setting up PDP Context...");
  return sendATCommand("AT+CGDCONT?", "OK", 5000);
#if 0  
  #if 0  
    return sendATCommand("AT+CGDCONT=1,\"IPV4V6\",\"" + APN + "\"", "OK", 5000);
  #else
    return sendATCommand("AT+CGDCONT=1,\"IP\",\"" + APN + "\"", "OK", 5000);
  #endif 
#endif
}

bool activatePDPContext() {
  Serial.println("Activating PDP Context...");
  return sendATCommand("AT+CGACT=1,1", "OK", 10000);
}

bool sendHTTPRequest(String url, String& responseContent) {
  Serial.println("Setting HTTP URL: " + url);
  String urlCommand = "AT+QHTTPURL=" + String(url.length()) + ",\"" + url + "\"";
  if (!sendATCommand(urlCommand, "OK", 5000)) {
    Serial.println("Failed to set HTTP URL.");
    return false;
  }

  Serial.println("Performing HTTP GET...");
  if (!sendATCommand("AT+QHTTPGET=60000", "+QHTTPGET:", 65000)) {
    Serial.println("HTTP GET failed.");
    return false;
  }

  // You might want to check the +QHTTPGET result code here

  Serial.println("Reading HTTP Response Content...");
  String readCommand = "AT+QHTTPREAD,0,1024"; // Adjust length as needed
  String readResponse;
  if (sendATCommand(readCommand, "+QHTTPREAD:", 10000, &readResponse)) {
    int startIndex = readResponse.indexOf("\r\n\r\n") + 4; // Find the start of the content after headers
    if (startIndex > 3 && startIndex < readResponse.length()) {
      responseContent = readResponse.substring(startIndex);
      Serial.println("HTTP Response Content:\n" + responseContent);
      return true;
    } else {
      responseContent = readResponse;
      Serial.println("Could not parse HTTP content:\n" + readResponse);
      return true; // Or false depending on your error handling
    }
  } else {
    Serial.println("Failed to read HTTP response.");
    return false;
  }
}

bool pingNetwork(String target, int contextId /*= 1 */, int timeout /*= 10000 */)
{
  Serial.print("Pinging ");
  Serial.print(target);
  Serial.print(" with Context ID: ");
  Serial.print(contextId);
  Serial.print(" and Timeout: ");
  Serial.print(timeout);
  Serial.println("ms");

  String command = "AT+QPING=" + String(contextId) + ",\"" + target + "\"," + String(timeout);
  return sendATCommand(command, "+QPING:", timeout * 2); // Increase overall timeout for multiple pings
}

void sendSMS(String phoneNumber, String message) {
  Serial.print("Sending SMS to: ");
  Serial.println(phoneNumber);
  Serial.print("Message: ");
  Serial.println(message);

  sendATCommand("AT+CMGF=1", "OK", 1000);
  lteSerial.print("AT+CMGS=\"" + phoneNumber + "\"\r");
  delay(100);
  if (lteSerial.find(">")) {
    Serial.print("> ");
    lteSerial.print(message);
    lteSerial.write(26); // Ctrl+Z
    // Wait for SMS sending status report
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      if (lteSerial.available()) {
        String response = lteSerial.readStringUntil('\n');
        Serial.println(response);
        if (response.indexOf("+CMGS:") != -1 || response.indexOf("+CMS ERROR:") != -1) {
          break;
        }
      }
    }
  } else {
    Serial.println("Error sending SMS: > prompt not received.");
  }
}

void getSignalStrength() {
  Serial.println("Getting Signal Strength...");
  String response;
  if (sendATCommand("AT+CSQ", "+CSQ:", 5000, &response)) {
    // Response format: +CSQ: <rssi>,<ber>
    int rssiStartIndex = response.indexOf("+CSQ:") + 6;
    int commaIndex = response.indexOf(",", rssiStartIndex);
    if (rssiStartIndex >= 6 && commaIndex > rssiStartIndex) {
      String rssi = response.substring(rssiStartIndex, commaIndex);
      Serial.print("Signal Strength (RSSI): ");
      Serial.println(rssi);
      // You can also extract the BER (Bit Error Rate) if needed
    } else {
      Serial.println("Failed to parse signal strength response.");
      Serial.println("Raw CSQ Response: " + response);
    }
  } else {
    Serial.println("Failed to get signal strength.");
  }
}

// --- Internet Connection Management Task ---
void connectAndMonitorInternetTask(void *pvParameters) {
  while (true) {

    if (!isNetworkConnected) {
      Serial.println("Attempting to connect to the internet...");
      if (isLteModulePowered && isSimReady && isNetworkRegistered) {
        if (setupPDPContext()) {
          if (activatePDPContext()) {
              delay(7000); // Wait 7 seconds before pinging
               getSignalStrength();
            isNetworkConnected = true;
            Serial.println("Successfully connected to the internet!");
          } else {
            Serial.println("Failed to activate PDP Context.");
          }
        } else {
          Serial.println("Failed to setup PDP Context.");
        }
      } else {
        Serial.println("LTE module not fully initialized, cannot connect to internet yet.");
      }
    } else {
      Serial.print("Network Pinging ");
      Serial.println(PING_TARGET);
      if (!pingNetwork(PING_TARGET)) {
        Serial.println("Ping failed, assuming network disconnected.");
        isNetworkConnected = false;
      }

      delay(5000);

      // In your loop or another task:
      String httpResponse = "";
      if (sendHTTPRequest("http://example.com", httpResponse)) {
        Serial.println("HTTP Request successful!");
        // Process the httpResponse
      } else {
        Serial.println("HTTP Request failed.");
      }

    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
  }
}

/*{{{ setup()*/
void setup() 
{
  Serial.begin(115200);  // Start the Serial communication
  while (!Serial);       // Wait for serial monitor

  Serial.println("Starting EC200U LTE connection...");

  lteSetup();

  delay(2000);  // Wait for EC200U to boot up
  
#if 0 //def ENABLE_WIFI_MODULE
  GetWifiScanList();

  /* Setup Wifi*/
  PSB_WifiSetup( );
#endif /* ENABLE_WIFI_MODULE */

  // Check if PSRAM is available
  if (psramFound()) {
    Serial.println("PSRAM is available.");
  }

#if 0 //def ENABLE_SECURE_WIFI_CONNECT
 // SetClientCertificate();
 gWifiespClient.setInsecure();  // This disables certificate verification (useful for debugging)
#endif /* ENABLE_SECURE_WIFI_CONNECT */

#if 0 //def ENABLE_PUB_SUB_CLIENT
  gMQTTClient.setServer( cgpcMqttServer, cgpi16MqttPort );
  gMQTTClient.setCallback( MQTT_ResponseCallback );
#endif

   //initDisplay();

#if 0
  // Create message queue
  msgQueue = xQueueCreate( QUEUE_SIZE, sizeof( Message ) );

  xTaskCreate( PSB_KeyboardMonitorTask
              , KEYPAD_THREAD_NAME
              , KEYPAD_STACK_SIZE
              , NULL
              , 1
              , &gtskKeyboardHndl
              );

#if 0
  // Create the MQTT task
  xTaskCreatePinnedToCore( MQTT_MonitorTask // Function to run          
              , MQTT_WORKER_THREAD_NAME             // Name of the task
              , MQTT_WORKER_STACK_SIZE                    // Stack size (bytes)
              , NULL                   // Parameters to pass to the task
              , 1                      // Task priority (1 is low priority)
              , &gtskMqttHndl        // Task handle (optional)
              , 0                        // Core where the task runs (0 or 1)
            );
#endif
#endif

#if 0
  /* Pay Sound Box Manager thread create */
  xTaskCreate(  MQTT_MonitorTask
              , MQTT_WORKER_THREAD_NAME
              , MQTT_WORKER_STACK_SIZE
              , NULL
              , 1
              , &gtskMqttHndl );

  /* Pay Sound Box Manager thread create */
  xTaskCreate(  PSB_PaySoundBoxManagerThread
              , PAY_SOUND_BOX_MANAGER_THREAD_NAME
              , PAY_SOUND_BOX_MANAGER_STACK_SIZE
              , NULL
              , 1
              , &gtskMainProcessHndl );
#endif
  // Start the connection/reconnection task
  xTaskCreate( connectAndMonitorInternetTask
              , LTE_WORKER_THREAD_NAME
              , LTE_WORKER_STACK_SIZE
              , NULL
              , 1
              , NULL );

#ifdef RTOS_THREAD_TEST_ODD_EVEN
  // Create the semaphores
  xSemaphoreOdd = xSemaphoreCreateBinary();
  xSemaphoreEven = xSemaphoreCreateBinary();

  // Check if the semaphores were created successfully
  if (xSemaphoreOdd == NULL || xSemaphoreEven == NULL) {
    Serial.println("Error creating semaphores!");
    while (1);  // Hang here if semaphore creation fails
  }

  // Initially, give control to the odd task first
  xSemaphoreGive(xSemaphoreOdd);

  // Create FreeRTOS tasks
  xTaskCreate(PSB_ProcessThreadEven, "Task1", 2048, NULL, 1, NULL);
  xTaskCreate(PSB_ProcessThreadOdd, "Task2", 2048, NULL, 1, NULL);
#endif /* RTOS_THREAD_TEST_ODD_EVEN */

}
/*}}}*/

void loop()
{
 
#if 0 //def ENABLE_WIFI_MODULE
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
#if 0
    if (pingTest("8.8.8.8"))
    {
      Serial.println("[INTERNET] Connection is active");
    }
    else
    {
      Serial.println("[INTERNET] No internet connection");
    }
  #endif 
    
    delay(4000);  // Check every 4 seconds 
  }
#endif /* ENABLE_WIFI_MODULE */
}
