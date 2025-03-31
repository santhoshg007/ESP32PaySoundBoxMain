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

/*===================================== Private Variables ===========================*/

static const char * gscpcFileName = "PAYSOUND.INO";


/*===================================== Function Prototypes =========================*/
bool initDisplay( void );

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

  float f32TransAmount = 0;

  while ( true )
  { 
#if 1    
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

#ifdef ENABLE_SECURE_WIFI_CONNECT
 // SetClientCertificate();
 gWifiespClient.setInsecure();  // This disables certificate verification (useful for debugging)
#endif /* ENABLE_SECURE_WIFI_CONNECT */

#ifdef ENABLE_PUB_SUB_CLIENT
  gMQTTClient.setServer( cgpcMqttServer, cgpi16MqttPort );
  gMQTTClient.setCallback( MQTT_ResponseCallback );
#endif

#if 0
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

  /* Pay Sound Box Manager thread create */
  xTaskCreate(  PSB_PaySoundBoxManagerThread
              , PAY_SOUND_BOX_MANAGER_THREAD_NAME
              , PAY_SOUND_BOX_MANAGER_STACK_SIZE
              , NULL
              , 1
              , &gtskMainProcessHndl );
#endif

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
      
      if ( !gMQTTClient.connected() )
      {
          MQTT_Reconnect( );
      }

     delay(4000);  // Check every 5 seconds

       // Ensure the MQTT client loop runs so we can process incoming messages and keep the connection alive
        gMQTTClient.loop();
  }
#endif /* ENABLE_WIFI_MODULE */
}
