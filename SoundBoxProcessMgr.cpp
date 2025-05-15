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
#include "Network4G.h"

#include <freertos/event_groups.h> // For Event Groups

/*===================================== Macros =======================================*/

#define MAX_KEYPAD_ROWS    4
#define MAX_KEYPAD_COLMS   4

#define KEYPAD_TASK_NAME        "KEYTSK"
#define KEYPAD_TASK_STACK_SIZE  (2 * 1024) // 2KB


#define MQTT_SUBS_TOPIC_PAYMENTS_STATUS       "paymentstatus"

// Color order setting
#define TFT_RGB_ORDER TFT_RGB

// Colors
#define BG_COLOR 0x00008B      // Dark Blue

#if 1
#define TEXT_COLOR TFT_WHITE    // White
#define HIGHLIGHT_COLOR 0xADD8E6 // Light Blue
#else
#define TEXT_COLOR TFT_BLACK    // White
#define HIGHLIGHT_COLOR TFT_BLUE // Light Blue
#endif


#if 1
#define MENU_BACKGROUND_COLOR   BG_COLOR
#else
#define MENU_BACKGROUND_COLOR   TFT_LIGHTGREY
#endif

// Keypad Navigation Mapping
#define KEY_UP 'B'
#define KEY_DOWN 'C'
#define KEY_SELECT 'D'

/*===================================== Type Definitions ============================*/

typedef enum tageSoundBoxMsgEventEnum
{
    /* 0x00 */ PSB_PAY_UPI
  , /* 0x01 */ PSB_PAY_NFC_CARDLESS
  , /* 0x02 */ PSB_PAY_HISTORY_STATUS

}PSB_MSG_EVENT_ENUM;



typedef enum tageNetworkStatusStruct
{
    /* 0x00 */ NW_STATUS_NONE               = 0
  , /* 0x01 */ NW_STATUS_SELECTED_WIFI      = ( 1 << 0 )
  , /* 0x02 */ NW_STATUS_CONNECTED_WIFI     = ( NW_STATUS_SELECTED_WIFI << 1 )
  , /* 0x04 */ NW_STATUS_SELECTED_4G        = ( NW_STATUS_CONNECTED_WIFI << 1 )
  , /* 0x08 */ NW_STATUS_CONNECTED_4G       = ( NW_STATUS_SELECTED_4G << 1  )
}NETWORK_STATUS_STRUCT;

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

#if 0
const char* cgpcSSID = "SLN 1st Floor";
const char* cgpcWifiPassword = "Sln@1111";
#endif

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


// Keypad matrix layout
char gcaKeyMapMtx[MAX_KEYPAD_ROWS][MAX_KEYPAD_COLMS] = {
  {'1','2','3', 'A'},
  {'4','5','6', 'B'},
  {'7','8','9', 'C'},
  {'*','0','#', 'D'}
};

// Row and column pins - Based on your initial request (may need adjustment for your board)
uint8_t gacKeypadRowPins[MAX_KEYPAD_ROWS] = {14, 21, 45, 38};
uint8_t gacKeypadColPins[MAX_KEYPAD_COLMS] = {7, 5, 15, 6};

// Keypad object
Keypad gKeypadMHndl = Keypad(makeKeymap(gcaKeyMapMtx), gacKeypadRowPins, gacKeypadColPins, MAX_KEYPAD_ROWS, MAX_KEYPAD_COLMS);
#endif /* ENABLE_MATRIX_KEYPAD */


// Positions and sizes (adjust based on your display)
int logoX = gTftDisplayHndl.width() / 2;
int logoY = 30;
int logoFontSize = 2;
String logoText = "Sound Box";

int itemStartY = 80;
int itemSpacing = 30;
int iconSize = 24;
int textOffsetX = 30;
int itemFontSize = 2;

#define SBOX_PAY_UPI_PAY_NOW       0
#define SBOX_PAY_NFC_CARDLESS      1
#define SBOX_PAY_LAST_PAY_AMOUNT   2
#define SBOX_PAY_SETTING_MENU      3


#define SBOX_SUBMENU_WIFI_CONNECT     0
#define SBOX_SUBMENU_4GLTE_CONNECT    1

// Menu Items with Icons (Basic Text Icons for this example)
char * gstrHomeMenuItems[] = {"QR Code Pay", "NFC Pay", "History", "Settings"};
char * gstrHomeMenuIcons[] = {"QR", "NFC", "Hist", "Set"};

// Menu Items with Icons (Basic Text Icons for this example)
char * gstrSettingMenuItems[] = {"Wifi", "4G LTE"};
char * gstrSettingMenuIcons[] = {"Y5", "4G"};

int gi32HomeSelectItem = 0;
int gi32HomePreviousItem = 0;

unsigned char ui8SettingSelectItem = 0;

unsigned int gui32NetworkStatus = NW_STATUS_NONE;


// MESSAGE_INFO_STRUCT Queue Setup for communication between threads
QueueHandle_t gmsgMainQueueHndl;
#define MAX_PROCESS_MSG_QUE_SIZE 10

String upiID = "6379175223@ptaxis";
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

extern EventGroupHandle_t networkEventGroup;

TaskHandle_t gtskNwMonitorHndl;

unsigned char gui8CurrentMainMenu = 0;
/*===================================== Private Variables ===========================*/

static const char * gscpcFileName = "SOUNDBOXMGR.CPP";


/*===================================== Function Prototypes =========================*/

void connectAndMonitorInternetTask(void *pvParameters);
bool IsInternetConnect( void );
void promptWiFiConnection();
void startMQTT();
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
  
  if( '\0' == gacUniqueID [0] )
  {
    UINT64 ui64chipId = ESP.getEfuseMac();  // Get the chip's MAC address (unique ID)

    snprintf(gacUniqueID, sizeof(gacUniqueID), "%llX", ui64chipId ); // Format as hexadecimal
    Serial.print("gacUniqueID ID: ");
    Serial.println(gacUniqueID);
  }

  return gacUniqueID;
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

float readFloatFromKeypadAndDisplay()
{
  String inputBuffer = "";
  gTftDisplayHndl.fillScreen(TFT_BLACK);
  gTftDisplayHndl.setCursor(10, 20);
  gTftDisplayHndl.setTextSize(2);
  gTftDisplayHndl.println("Enter float:");

  gTftDisplayHndl.setCursor(10, 50);
  gTftDisplayHndl.setTextSize(3);

  while (true)
  {
    char key = gKeypadMHndl.getKey();

    if ( key )
    {
      if (key >= '0' && key <= '9')
      {
        inputBuffer += key;
        gTftDisplayHndl.fillScreen(TFT_BLACK);
        gTftDisplayHndl.setCursor(10, 20);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("Enter float:");
        gTftDisplayHndl.setCursor(10, 50);
        gTftDisplayHndl.setTextSize(3);
        gTftDisplayHndl.print(inputBuffer);
      }
      else if ( key == 'A' )
      {
        if (inputBuffer.indexOf('.') == -1)
        {
          inputBuffer += '.';
          gTftDisplayHndl.fillScreen(TFT_BLACK);
          gTftDisplayHndl.setCursor(10, 20);
          gTftDisplayHndl.setTextSize(2);
          gTftDisplayHndl.println("Enter float:");
          gTftDisplayHndl.setCursor(10, 50);
          gTftDisplayHndl.setTextSize(3);
          gTftDisplayHndl.print(inputBuffer);
        }
      }
      else if ( key == 'D' )
      {
        float number = inputBuffer.toFloat();
        Serial.print("Entered Float: ");
        Serial.println(number);
        gTftDisplayHndl.fillScreen(TFT_BLACK);
        gTftDisplayHndl.setCursor(10, 20);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("You entered:");
        gTftDisplayHndl.setCursor(10, 50);
        gTftDisplayHndl.setTextSize(3);
        gTftDisplayHndl.print(number);
        delay(1000); // Show result for a short time
        return number;
      }
      else if ( key == '*' )
      {
        inputBuffer = "";
        Serial.println("Input cleared");
        gTftDisplayHndl.fillScreen(TFT_BLACK);
        gTftDisplayHndl.setCursor(10, 20);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("Enter float:");
        gTftDisplayHndl.setCursor(10, 50);
        gTftDisplayHndl.setTextSize(3);
        gTftDisplayHndl.print(inputBuffer);
      }

      Serial.print("Current Input: ");
      Serial.println(inputBuffer);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// This function generates a UPI string with the given amount
String CreateUPIString( float f32Amount )
{
    String upiString = "upi://pay?pa=" + upiID + "&pn=Merchant&mc=123456&tid=1234567890&tr=1234567890&tn=Payment%20for%20goods&am=";
    upiString += String( f32Amount, 2 );  // Append the amount to the UPI string
    upiString += "&cu=INR&url=https://www.merchantwebsite.com";  // Add currency and URL
    return upiString;
}

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
  char key = gKeypadMHndl.getKey();
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
  MESSAGE_INFO_STRUCT stMsgInfo;
  int ui32Event = 0;
  float f32TransAmount = 0;

  while ( true )
  {

    if( pdTRUE != xQueueReceive(gmsgMainQueueHndl, &stMsgInfo, portMAX_DELAY) )
    {
      continue;
    }

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

    vTaskDelay(4000 / portTICK_PERIOD_MS);  // Delay for 1 second

  }

}
/*}}*/

#if 0
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
#endif

// Function to get payment amount
String getPaymentAmount()
{
    gTftDisplayHndl.fillScreen(BG_COLOR);
    gTftDisplayHndl.setTextColor(TEXT_COLOR);
    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.setTextDatum(MC_DATUM);
    gTftDisplayHndl.drawString("Enter Amount:", gTftDisplayHndl.width() / 2, gTftDisplayHndl.height() / 2 - 30, 2);

    String amount = "";
    bool decimalPointEntered = false;
    int xPos = gTftDisplayHndl.width() / 2;
    int yPos = gTftDisplayHndl.height() / 2;

    while (true)
    {
        char key = gKeypadMHndl.getKey();
        if (key)
        {
            if ( key >= '0' && key <= '9')
            {
                amount += key;
                gTftDisplayHndl.fillRoundRect(xPos - 50, yPos - 15, 100, 30, 5, BG_COLOR);
                gTftDisplayHndl.drawString(amount, xPos, yPos, 2);
            }
            else if (key == 'A')
            {
                if (!decimalPointEntered)
                {
                    amount += '.';
                    decimalPointEntered = true;
                    gTftDisplayHndl.fillRoundRect(xPos - 50, yPos - 15, 100, 30, 5, BG_COLOR);
                    gTftDisplayHndl.drawString(amount, xPos, yPos, 2);
                }
            }
            else if (key == 'D')
            {
                if (amount.length() > 0)
                {
                    return amount;
                }
            }
            else if (key == '*')
            {
                amount = "";
                decimalPointEntered = false;
                gTftDisplayHndl.fillScreen(BG_COLOR);
                gTftDisplayHndl.setTextColor(TEXT_COLOR);
                gTftDisplayHndl.setTextSize(2);
                gTftDisplayHndl.setTextDatum(MC_DATUM);
                gTftDisplayHndl.drawString("Enter Amount:", gTftDisplayHndl.width() / 2, gTftDisplayHndl.height() / 2 - 30, 2);
            }
            delay(100);
        }
    }
}

// Function to display the QR code on the gTftDisplayHndl using ESP-IDF data
void displayQRCodeOnTFT(esp_qrcode_handle_t qrcode)
{
    int size = esp_qrcode_get_size(qrcode);
    int moduleSize;
    int offsetX, offsetY;

    // Calculate module size to fit QR code with a margin
    if (size > 0) {
        moduleSize = min((gTftDisplayHndl.width() - 20) / size, (gTftDisplayHndl.height() - 20) / size);  // 20 for margin
        if (moduleSize < 1) {
          moduleSize = 1;
        }
        offsetX = (gTftDisplayHndl.width() - size * moduleSize) / 2;
        offsetY = (gTftDisplayHndl.height() - size * moduleSize) / 2;
    } else {
        Serial.println("Error: QR code size is invalid.");
        return;
    }

    gTftDisplayHndl.fillScreen(TFT_BLUE); // Clear the screen with the defined background color
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            if (esp_qrcode_get_module(qrcode, x, y))
            {
                gTftDisplayHndl.fillRect(offsetX + x * moduleSize, offsetY + y * moduleSize, moduleSize, moduleSize, TFT_BLACK);
            }
        }
    }
}

// Function to generate and display QR code using ESP-IDF
void GenerateAndDisplayQR( const char *upiData )
{
    esp_qrcode_config_t qrcode_config = ESP_QRCODE_CONFIG_DEFAULT();
    qrcode_config.display_func = displayQRCodeOnTFT; // Set our display function

    esp_err_t err = esp_qrcode_generate(&qrcode_config, upiData);
    if (err != ESP_OK)
    {
        Serial.println("QR code generation failed!");
        if (err == ESP_ERR_NO_MEM)
        {
            Serial.println("Error: Not enough memory to generate QR code.");
        }
        // Consider adding a gTftDisplayHndl display message here to inform the user
        return; // Exit the function on error
    }
    // The display function is called by esp_qrcode_generate
}

void drawSettingScreen(  )
{
    gTftDisplayHndl.fillScreen(BG_COLOR);
    gTftDisplayHndl.setTextColor(TEXT_COLOR);

    // Draw Logo
    gTftDisplayHndl.setTextSize(logoFontSize);
    gTftDisplayHndl.setTextDatum(MC_DATUM);
    gTftDisplayHndl.drawString(logoText, logoX, logoY, 2);

    // Draw Menu Items
    gTftDisplayHndl.setTextSize(itemFontSize);
    gTftDisplayHndl.setTextDatum(ML_DATUM);
    for (int i = 0; i < 2; i++)
    {
        int yPos = itemStartY + i * itemSpacing;
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(gstrSettingMenuIcons[i], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(gstrSettingMenuItems[i], 25 + iconSize + textOffsetX - 5, yPos, 2);
    }
    //draw initial highlight
    int yPos = itemStartY + ui8SettingSelectItem * itemSpacing;
    gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, HIGHLIGHT_COLOR);
    gTftDisplayHndl.setTextColor(TEXT_COLOR);
    gTftDisplayHndl.setTextSize(1);
    gTftDisplayHndl.drawString(gstrHomeMenuIcons[ui8SettingSelectItem], 25, yPos, 2);
    gTftDisplayHndl.setTextSize(itemFontSize);
    gTftDisplayHndl.drawString(gstrSettingMenuItems[ui8SettingSelectItem], 25 + iconSize + textOffsetX - 5, yPos, 2);
}

void drawUIListScreen( char ** ppcMenuIcons, char ** ppcMenuItems, int MaximumListSize )
{
    gTftDisplayHndl.fillScreen(MENU_BACKGROUND_COLOR);   
    delay(50);

    gTftDisplayHndl.setTextColor(TEXT_COLOR);

    // Draw Logo
    gTftDisplayHndl.setTextSize(logoFontSize);
    gTftDisplayHndl.setTextDatum(MC_DATUM);
    gTftDisplayHndl.drawString(logoText, logoX, logoY, 2);
    delay(5);

    // Draw Menu Items
    gTftDisplayHndl.setTextSize(itemFontSize);
    gTftDisplayHndl.setTextDatum(ML_DATUM);
    delay(5);
    
    for (int i = 0; i < MaximumListSize; i++)
    {
        int yPos = itemStartY + i * itemSpacing;
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(ppcMenuIcons[i], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(ppcMenuItems[i], 25 + iconSize + textOffsetX - 5, yPos, 2);
    }
    //draw initial highlight
    int yPos = itemStartY + gi32HomeSelectItem * itemSpacing;
    gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, HIGHLIGHT_COLOR);
    delay(50);
    gTftDisplayHndl.setTextColor(TEXT_COLOR);
    gTftDisplayHndl.setTextSize(1);
    gTftDisplayHndl.drawString(ppcMenuIcons[gi32HomeSelectItem], 25, yPos, 2);
    delay(5);
    gTftDisplayHndl.setTextSize(itemFontSize);
    gTftDisplayHndl.drawString(ppcMenuItems[gi32HomeSelectItem], 25 + iconSize + textOffsetX - 5, yPos, 2);
    delay(5);
}

void UpdateSettingSelection(char key)
{

}

void UpdateUISelection(char key, char ** ppcMenuIcons, char ** ppcMenuItems, int MaximumListSize )
{
    if ( key == KEY_UP )
    {
        gi32HomePreviousItem = gi32HomeSelectItem;
        gi32HomeSelectItem--;
        if (gi32HomeSelectItem < 0)
            gi32HomeSelectItem = ;
        //erase previous highlight
        int yPos = itemStartY + gi32HomePreviousItem * itemSpacing;
        gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, MENU_BACKGROUND_COLOR);
        delay(10);
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(ppcMenuIcons[gi32HomePreviousItem], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(ppcMenuItems[gi32HomePreviousItem], 25 + iconSize + textOffsetX - 5, yPos, 2);

        //draw new highlight
        yPos = itemStartY + gi32HomeSelectItem * itemSpacing;
        gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, HIGHLIGHT_COLOR);
        delay(10);
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(ppcMenuIcons[gi32HomeSelectItem], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(ppcMenuItems[gi32HomeSelectItem], 25 + iconSize + textOffsetX - 5, yPos, 2);
    }
    else if ( key == KEY_DOWN )
    {
        gi32HomePreviousItem = gi32HomeSelectItem;
        gi32HomeSelectItem++;
        if (gi32HomeSelectItem > 3)
            gi32HomeSelectItem = 0;
        //erase previous highlight.
        int yPos = itemStartY + gi32HomePreviousItem * itemSpacing;
        gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, BG_COLOR);
        delay(10);
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(ppcMenuIcons[gi32HomePreviousItem], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(ppcMenuItems[gi32HomePreviousItem], 25 + iconSize + textOffsetX - 5, yPos, 2);

        //draw new highlight
        yPos = itemStartY + gi32HomeSelectItem * itemSpacing;
        gTftDisplayHndl.fillRect(10, yPos - 12, gTftDisplayHndl.width() - 20, 24, HIGHLIGHT_COLOR);
        delay(10);
        gTftDisplayHndl.setTextColor(TEXT_COLOR);
        gTftDisplayHndl.setTextSize(1);
        gTftDisplayHndl.drawString(ppcMenuIcons[gi32HomeSelectItem], 25, yPos, 2);
        gTftDisplayHndl.setTextSize(itemFontSize);
        gTftDisplayHndl.drawString(ppcMenuItems[gi32HomeSelectItem], 25 + iconSize + textOffsetX - 5, yPos, 2);
    }
}

void handleUserSelection( char key )
{
  if ( key != KEY_SELECT )
  {
      return;
  }

  gTftDisplayHndl.fillScreen( BG_COLOR );
  delay(10);
  gTftDisplayHndl.setTextColor ( TEXT_COLOR );
  gTftDisplayHndl.setTextSize( 2 );

  Serial.println( "User Selection Value :" );

  Serial.println( gi32HomeSelectItem );

  if ( SBOX_PAY_SETTING_MENU == gui8CurrentMainMenu  )
  {
    promptWiFiConnection( );

    delay ( 1000 );

    Serial.println( "Drawing Home Screen :" );

    gui8CurrentMainMenu = SBOX_PAY_UPI_PAY_NOW;

    drawUIListScreen( gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );

    return;
  }

  if ( ( !IsInternetConnect( ) ) && ( SBOX_PAY_SETTING_MENU != gi32HomeSelectItem ))
  {
    gTftDisplayHndl.println("Please connect internet 4G or Wifi...");
    delay ( 5000 ); 
    drawUIListScreen( gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );
    return;
  }

  switch ( gi32HomeSelectItem )
  {
    case SBOX_PAY_UPI_PAY_NOW:
    {
      Serial.println("Pay Now selected");

      // QR Code Pay selected
      String amount = getPaymentAmount();
      Serial.println("Amount entered: " + amount);
      String upi_string = CreateUPIString(amount.toFloat());
      GenerateAndDisplayQR(upi_string.c_str()); // Pass as const char*
      delay(5000); // Display QR code for 5 seconds
      Serial.println("Implement the QR code preference");
    }
      break; /* PSB_PAY_UPI */

    case SBOX_PAY_NFC_CARDLESS:
    {
      Serial.println("Pay NFC cardless ");
      gTftDisplayHndl.println("Ready for NFC...");
      delay(1000);
      drawUIListScreen( gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );         
      
    }
      break; /* PSB_PAY_NFC_CARDLESS */

    case SBOX_PAY_SETTING_MENU:
    {
      Serial.println("Pay setting menu");
      gui8CurrentMainMenu = SBOX_PAY_SETTING_MENU;
     drawUIListScreen( gstrSettingMenuIcons, gstrSettingMenuItems, ARRAY_LENGTH( gstrSettingMenuIcons ) );            
    }
      break; /* PSB_PAY_HISTORY_STATUS */

    case SBOX_PAY_LAST_PAY_AMOUNT:
    {
      Serial.println("Last Pay amount play");
      gTftDisplayHndl.println("Payment History...");
      delay(1000);
      drawUIListScreen( gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );            
    }
      break; /* PSB_PAY_HISTORY_STATUS */
    default:
    {
      Serial.println("Unknown error code");
    }
      break; /* default */
  }    
}

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
    try 
    {

	    gTftDisplayHndl.init();
	    gTftDisplayHndl.setRotation(1);
	    gTftDisplayHndl.fillScreen(BG_COLOR);
      delay(10);
	    drawUIListScreen( gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );

      gbDisplayConnecStatus = true;
    }
    catch (const std::exception& e) 
    {
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

/*
 * Args    : void
 * Return  : void
 * Description :Init all the required modules.
 */
/*{{ InitAllmodules() */
void InitAllmodules( void )
{
  Serial.println("Starting EC200U LTE connection...");

  initDisplay();

  delay(2000);  // Wait for EC200U to boot up

  InitLTEModule();

  WiFi.mode(WIFI_STA);

#ifdef USE_HTTPS
    Serial.println("HTTPS is enabled");
#else
    Serial.println("HTTPS is disabled");
#endif

  // Initialize Event Group
  networkEventGroup = xEventGroupCreate();

  if ( networkEventGroup == NULL ) 
  {
      Serial.println("Failed to create network event group!");
      while (true); // Fatal error.
  }

  // Create message queue
  gmsgMainQueueHndl = xQueueCreate( MAX_PROCESS_MSG_QUE_SIZE, sizeof( MESSAGE_INFO_STRUCT ) );

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

  #if 0

    /* Pay Sound Box Manager thread create */
    xTaskCreate(  PSB_PaySoundBoxManagerThread
                , PAY_SOUND_BOX_MANAGER_THREAD_NAME
                , PAY_SOUND_BOX_MANAGER_STACK_SIZE
                , NULL
                , 1
                , &gtskMainProcessHndl );
  #endif

  startMQTT();
#if 0
    /* Pay Sound Box Manager thread create */
    xTaskCreate(  connectAndMonitorInternetTask
                , NW_WORKER_THREAD_NAME
                , NW_WORKER_STACK_SIZE
                , NULL
                , 1
                , &gtskNwMonitorHndl );
#endif                

}
/*}}*/

/*
 * Args    : void
 * Return  : void
 * Description :Main run loop.
 */
/*{{ MainRunLoop() */
void MainRunLoop()
{
  char cKeyEvent = 0;

  while ( true )
  {

    cKeyEvent = gKeypadMHndl.getKey();

    if ( cKeyEvent )
    {
        Serial.print("Key Pressed: ");
        Serial.println( cKeyEvent );
        
        if ( SBOX_PAY_SETTING_MENU == gui8CurrentMainMenu  )
        {
          UpdateUISelection( cKeyEvent, gstrSettingMenuIcons, gstrSettingMenuItems, ARRAY_LENGTH( gstrSettingMenuIcons ) );
        }
        else
        {
           UpdateUISelection( cKeyEvent, gstrHomeMenuIcons, gstrHomeMenuItems, ARRAY_LENGTH( gstrHomeMenuIcons ) );
        }
       
        handleUserSelection( cKeyEvent );
    }

    delay(100);

  }

}
/*}}*/