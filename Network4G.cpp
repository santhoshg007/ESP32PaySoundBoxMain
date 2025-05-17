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
 * File Name        : Network4G.cpp
 * File Description : Network 4G handling
 * Author           : Santhosh G
 * Date Created     : March 8, 2025
 *
 * ============
 * Revision History:
 * ============
 * Date         Author             Description of Changes
 * -----        -------            ----------------------
 * 01May25      Santhosh G         Initial creation of Network4G.cpp.
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

#include <Arduino.h>

#include "Network4G.h"

#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TFT_eSPI.h>       // TFT library
#include <Keypad.h>           // Keypad library
#include <FreeRTOS.h>       // For tasks
#include <string.h>           // For string functions
#include <freertos/event_groups.h> // For Event Groups

// Configuration: HTTPS Enable/Disable
//#define USE_HTTPS // Comment out this line to disable HTTPS

#ifdef ENABLE_SECURE_WIFI_CONNECT
  #include <WiFiClientSecure.h>
#else
  #include <WiFiClient.h>
#endif

#include "SoundBoxProcessMgr.h"

/*===================================== Macros =======================================*/

#define MAX_VISIBLE_SSIDS 5
#define UP_KEY 'B'
#define DOWN_KEY 'C'
#define REFRESH_KEY '#'
#define SCROLL_DELAY 100 // Delay in milliseconds for scrolling

#define PASSWORD_BG_COLOR TFT_SKYBLUE
#define SCAN_LIST_BG_COLOR TFT_LIGHTGREY
#define TEXT_COLOR TFT_BLACK

/*===================================== Type Definitions ============================*/

/*===================================== Global Variables ===========================*/
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);


// MQTT broker details
const char*  cgpcMqttServer = "54.162.150.3"; // or hostname

#ifdef ENABLE_SECURE_WIFI_CONNECT
#error
const short int cgpi16MqttPort = 8883; // SSL/TLS port for Adafruit IO
#else
const short int cgpi16MqttPort = 1883; // Default MQTT port
#endif /* ENABLE_SECURE_WIFI_CONNECT */

const char* cgpcMqttUser = "konguess"; // Optional
const char* cgpcMqttPassword = "konguess#$007"; // Optional


// 4G APN and credentials
//const char apn[] = "jionet";
const char apn[] = "airtelgprs.com"; // Store the Airtel APN

const char simUser[] = "";
const char simPass[] = "";

volatile bool isSimReady = false;
volatile bool isNetworkReady = false;
volatile bool isInternetConnected = false;

// Global flags
volatile bool is4GConnected = false;
volatile bool isWifiConnected = false;
volatile bool gbWifiConnectCanceled = false; // Add this flag


// Global objects
TinyGsmClient gsmClient(modem);

// WiFi credentials - moved to global scope
char gacSSID[32];
char gacUserWifiPassword[32];

// Event Group to manage network and MQTT states
EventGroupHandle_t networkEventGroup;


String selectedSSID;

// Task handles
TaskHandle_t gtskMqttHndl = NULL; // Handle for the MQTT task

/*===================================== Extern Variables ===========================*/

// TFT and Keypad setup
extern TFT_eSPI gTftDisplayHndl;

extern Keypad gKeypadMHndl;

#ifdef ENABLE_SECURE_WIFI_CONNECT
extern WiFiClientSecure gWifiespClient;  // Secure Wi-Fi client
#else
extern WiFiClient gWifiespClient;
#endif /* ENABLE_SECURE_WIFI_CONNECT */

extern PubSubClient gMQTTClient;

/*===================================== Private Variables ===========================*/
static const char * gscpcFileName = "NETWORK4G.CPP";

/*===================================== Function Prototypes =========================*/

char* PSB_GetUniqueIDString( void );
/*===================================== Function Definitions ========================*/

/*
 * Function: pingNetwork
 * Purpose: Send ICMP ping using AT command to verify connectivity.
 * Parameters: const char* target - IP/domain to ping.
 * Returns: void
 */
void pingNetwork( const char* target ) 
{
    if (!modem.isGprsConnected()) {
        Serial.println("GPRS not connected. Cannot ping.");
        return;
    }

    Serial.printf("[Ping] Pinging %s...\n", target);

    Serial.println("Pinging google.com...");

    String command = String("AT+QPING=1,\"") + target + "\"";
    SerialAT.println(command);

    String response = "";
    unsigned long startTime = millis();

    while (millis() - startTime < 10000)
    {
        if (SerialAT.available()) {
            response += SerialAT.readString();
            if (response.indexOf("+QPING:") != -1) {
                Serial.println("[Ping] Success:\n" + response);
                return;
            }
        }
    }

    Serial.println("[Ping] Failed or timeout.");
}

void switchMQTTClient(bool use4G)
{
    if (gMQTTClient.connected()) {
        gMQTTClient.disconnect();
        delay(100);
    }

    if (use4G) {
        gMQTTClient.setClient(gsmClient);
    } else {
        gMQTTClient.setClient(gWifiespClient);
    }

    //startMQTT();  // reconnect MQTT
}

// Function to disconnect from MQTT
void stopMQTT() 
{
    if ( gMQTTClient.connected() ) 
    {
        Serial.println("Disconnecting from MQTT...");
        gMQTTClient.disconnect();
        xEventGroupClearBits(networkEventGroup, EVT_MQTT_CONNECTED);

        if (gtskMqttHndl != NULL) 
        {
            vTaskDelete(gtskMqttHndl);
            gtskMqttHndl = NULL;
            Serial.println("MQTT Task stopped.");
        }
    }
}

// Function to start the MQTT task
void startMQTT() 
{
    if ( gtskMqttHndl == NULL ) 
    {
        /* Pay Sound Box Manager thread create */
        xTaskCreate(  MQTT_MonitorTask
                , MQTT_WORKER_THREAD_NAME
                , MQTT_WORKER_STACK_SIZE
                , NULL
                , 1
                , &gtskMqttHndl );

        Serial.println("MQTT Task started.");
    } 
    else
    {
        Serial.println("MQTT Task already running.");
    }
}


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
 * Description :  connect to MQTT broker.
 */
/*{{ MQTT_Reconnect() */
bool MQTT_Reconnect( void )
{
  boolean  bRetMqttConnectStatus = true;

  while (  IsInternetConnect() && !gMQTTClient.connected() )
  {
    Serial.print("Attempting MQTT connection...");

    if ( gMQTTClient.connect( PSB_GetUniqueIDString(), cgpcMqttUser, cgpcMqttPassword ) )
    {
      char acMQttSubscribeTopic[ MAX_MQTT_TOPIC_SUBSCRIBE_LEN ] = { 0 };

      Serial.println("Connected to MQTT");
      xEventGroupSetBits(networkEventGroup, EVT_MQTT_CONNECTED); //set mqtt connected
#if 0
      strcpy( acMQttSubscribeTopic, PSB_GetUniqueIDString( ) );

      strcat( acMQttSubscribeTopic, MQTT_SUBS_TOPIC_PAYMENTS_STATUS );
#else
      strcpy( acMQttSubscribeTopic, "dbs/txnResponse" );
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

/*
 * Args    : void *
 * Return  : void
 * Description :MQTT Task..
 */
/*{{ MQTT_MonitorTask() */
void MQTT_MonitorTask( void* parameter )
{

    Serial.println("MQTT and monitor thread start...");
    //InitLTEModule();

    gMQTTClient.setServer( cgpcMqttServer, cgpi16MqttPort );
    gMQTTClient.setCallback( MQTT_ResponseCallback );

    while ( true )
    {
        if (!(xEventGroupGetBits(networkEventGroup) & EVT_INTERNET_CONNECTED)) 
        {
            //Serial.println("MQTT Task: No internet connection.  Stopping MQTT.");
            //stopMQTT();
            connectToInternet();
            vTaskDelay(pdMS_TO_TICKS(5000)); //check again in 5 seconds.
            continue;
        }

        if (!gMQTTClient.connected()) {
            MQTT_Reconnect( );
        }
        
        gMQTTClient.loop(); // Keep MQTT connection alive
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 1 second.  Use FreeRTOS delay.     
    }
}
/*}}*/


// Function to initialize the LTE module
void InitLTEModule() 
{
    Serial.println("[Network4G] Starting EC200U LTE monitoring");

    // Initialize the modem serial port
    SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX); // Corrected RX/TX
    delay(3000);

    // Reset the modem
    modem.restart();
    delay(1000);
}


// Function to set up the 4G connection
void internetSetup() 
{
    Serial.println("[InternetSetup] Checking SIM...");
    isSimReady = modem.getSimStatus(); // Simplified
    if (!isSimReady) 
    {
        Serial.println("[InternetSetup] SIM not inserted.");
        return;
    }
    Serial.println("[InternetSetup] SIM is ready.");

    Serial.println("[InternetSetup] Waiting for network...");
    xEventGroupSetBits(networkEventGroup, EVT_4G_CONNECTING); // Set 4G connecting
    isNetworkReady = modem.waitForNetwork(30000); // Increased timeout
    if (!isNetworkReady) 
    {
        Serial.println("[InternetSetup] Network unavailable.");
        xEventGroupClearBits(networkEventGroup, EVT_4G_CONNECTING); // Clear 4G connecting
        is4GConnected = false;
        return;
    }
    Serial.println("[InternetSetup] Network connected.");

    Serial.println("[InternetSetup] Connecting to GPRS...");
    is4GConnected = modem.gprsConnect(apn, simUser, simPass);
    if (!is4GConnected) 
    {
        Serial.println("[InternetSetup] GPRS connection failed.");
        xEventGroupClearBits(networkEventGroup, EVT_4G_CONNECTING); // Clear 4G connecting
        return;
    }
    Serial.println("[InternetSetup] GPRS connected.");
    xEventGroupClearBits(networkEventGroup, EVT_4G_CONNECTING); // Clear 4G connecting
    xEventGroupSetBits(networkEventGroup, EVT_4G_CONNECTED);   // Set 4G connected
}

// Function to perform an HTTP GET request
bool httpGetRequest(const char* host, const char* path, String& response) 
{
    Serial.printf("[HTTP] Connecting to %s...\n", host);
#ifdef USE_HTTPS
    WiFiClientSecure httpclient;
#else
    WiFiClient httpclient;
#endif
    if (!httpclient.connect(host, 80)) 
    {
        Serial.println("[HTTP] Connection failed.");
        return false;
    }

    httpclient.print(String("GET ") + path + " HTTP/1.1\r\n");
    httpclient.print(String("Host: ") + host + "\r\n");
    httpclient.print("Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    response = "";
    while (httpclient.connected() && millis() - timeout < 10000) 
    {
        while (httpclient.available()) {
            char c = httpclient.read();
            response += c;
            timeout = millis(); // Reset timeout on data received
        }
    }
    httpclient.stop();
    if (response.length() > 0)
    {
         Serial.println("[HTTP] Response received.");
         return true;
    }
    else
    {
        Serial.println("[HTTP] No response.");
        return false;
    }
}

// Function to perform an HTTPS GET request
bool httpsGetRequest(const char* host, const char* path, String& response) 
{
#ifdef USE_HTTPS
    Serial.printf("[HTTPS] Connecting to %s...\n", host);
    WiFiClientSecure httpsClient;

    // HTTPS uses port 443
    if (!httpsClient.connect(host, 443)) 
    { 
        Serial.println("[HTTPS] Connection failed.");
        return false;
    }

    httpsClient.print(String("GET ") + path + " HTTP/1.1\r\n");
    httpsClient.print(String("Host: ") + host + "\r\n");
    httpsClient.print("Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    response = "";
    while (httpsClient.connected() && millis() - timeout < 10000) {
        while (httpsClient.available()) {
            char c = httpsClient.read();
            response += c;
            timeout = millis(); // Reset timeout on data received
        }
    }
    httpsClient.stop();
    if (response.length() > 0) {
        Serial.println("[HTTPS] Response received.");
        return true;
    } else {
        Serial.println("[HTTPS] No response.");
        return false;
    }
#else
    Serial.println("[HTTPS] HTTPS is not enabled.  Define USE_HTTPS to use this function.");
    return false;
#endif
}

void drawSSIDList( int startIdx, int highlightIdx, int totalNetworks) 
{
    gTftDisplayHndl.fillRect(0, 0, gTftDisplayHndl.width(), gTftDisplayHndl.height() - 30, SCAN_LIST_BG_COLOR); // Set grey background
   
    delay(50);

    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.setCursor(10, 0);
    gTftDisplayHndl.setTextColor(TEXT_COLOR, SCAN_LIST_BG_COLOR);
    gTftDisplayHndl.printf("WiFi Networks (%d total)\n\n", totalNetworks);

    for (int i = 0; i < MAX_VISIBLE_SSIDS && (startIdx + i) < totalNetworks; i++) 
    {
        int y = 40 + i * 24; // Spacing between SSIDs
        if ( i == highlightIdx ) 
        {
            gTftDisplayHndl.fillRect(5, y - 2, gTftDisplayHndl.width() - 10, 22, TFT_BLUE);
            delay(50);
            gTftDisplayHndl.setTextColor(TFT_WHITE, TFT_BLUE);
        } 
        else
        {
            gTftDisplayHndl.setTextColor(TEXT_COLOR, SCAN_LIST_BG_COLOR);
        }
        gTftDisplayHndl.setCursor(10, y);
        String ssid = WiFi.SSID(startIdx + i);
        if (ssid.length() > 20) 
        {
            ssid = ssid.substring(0, 17) + "..."; // Truncate long SSID names
        }
        gTftDisplayHndl.print(ssid);
        delay(10);
    }
}

void drawActionBar()
{
    gTftDisplayHndl.fillRect(0, gTftDisplayHndl.height() - 30, gTftDisplayHndl.width(), 30, TFT_DARKGREY);  // Footer bar
    delay(50);
    gTftDisplayHndl.setTextColor(TFT_WHITE);
    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.setCursor(10, gTftDisplayHndl.height() - 24);
    gTftDisplayHndl.print("[D]Sel");
    delay(10);

    gTftDisplayHndl.setCursor(gTftDisplayHndl.width() - 110, gTftDisplayHndl.height() - 24);
    gTftDisplayHndl.print("[*]Cancel");
    delay(10);
    gTftDisplayHndl.setCursor(gTftDisplayHndl.width() - 220, gTftDisplayHndl.height() - 24);
    gTftDisplayHndl.print("[#]Ref");
    delay(10);
}

void drawPasswordScreen(const char* selectedSSID) 
{
    delay(50);
    gTftDisplayHndl.fillScreen(PASSWORD_BG_COLOR); // Set background color
    delay(50);// Give the display time to settle

    gTftDisplayHndl.setTextColor(TFT_BLACK, PASSWORD_BG_COLOR);
    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.setCursor(10, 10);

    // Show selected SSID
    gTftDisplayHndl.print("SSID: ");
    delay(10);
    gTftDisplayHndl.println(selectedSSID);
    delay(10);

    // Prompt for password
    gTftDisplayHndl.setCursor(10, 40);
    gTftDisplayHndl.print("Enter Password:");
    delay(10);

    // Password display area
    gTftDisplayHndl.fillRect(10, 70, gTftDisplayHndl.width() - 20, 24, TFT_WHITE); // White background for password
    delay(50);
    gTftDisplayHndl.setTextColor(TFT_BLACK);
    gTftDisplayHndl.setCursor(15, 74); // Start position for the asterisks

    // Show bottom bar (Connect / Cancel)
    gTftDisplayHndl.fillRect(0, gTftDisplayHndl.height() - 30, gTftDisplayHndl.width(), 30, TFT_DARKGREY);  // Footer bar
    delay(50);
    gTftDisplayHndl.setTextColor(TFT_WHITE);
    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.setCursor(10, gTftDisplayHndl.height() - 24);
    gTftDisplayHndl.print("[D]Connect");
    delay(3);
    gTftDisplayHndl.setCursor(gTftDisplayHndl.width() - 110, gTftDisplayHndl.height() - 24);
    gTftDisplayHndl.print("[*]Cancel");

    // Input password
    int i = 0;
    int cursorX = 15; // Track the x-position for the next asterisk
    while (i < 31) 
    {
        char key = gKeypadMHndl.getKey();
        if (key) 
        {
            if (key == 'D')
            {
                gacUserWifiPassword[i] = '\0';
                return;
            } 
            else if (key == '*')
            {
                gbWifiConnectCanceled = true;
                return;
            }
            else if (key == '#') 
            {
                gacUserWifiPassword[i] = '\0';
                break;
            }
            else if (key == 'A' || key == 'B' || key == 'C') 
            {
                continue; // Ignore navigation keys
            } 
            else
            {
                if (i < sizeof(gacUserWifiPassword) - 1) 
                {
                    gacUserWifiPassword[i] = key;
                    i++;
                    gTftDisplayHndl.setCursor(cursorX, 74);
                    gTftDisplayHndl.print('*');  // Mask gacUserWifiPassword
                    cursorX += 12;   // Move cursor for the next asterisk (adjust as needed based on font size)
                }
            }
            delay(200);
        }
    }
    gacUserWifiPassword[ i ] = '\0';
}

bool connectToWiFiNetwork( const char* ssid, const char* password ) 
{
    WiFi.begin(ssid, password);

    gTftDisplayHndl.fillScreen(PASSWORD_BG_COLOR);
    delay(50);
    gTftDisplayHndl.setTextColor(TFT_BLACK, PASSWORD_BG_COLOR);
    gTftDisplayHndl.setCursor(10, 40);
    gTftDisplayHndl.setTextSize(2);
    gTftDisplayHndl.println("Connecting...");
    delay(20);

    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 15000;

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) 
    {
        delay(500);
        gTftDisplayHndl.print(".");
    }

    if ( WiFi.status() == WL_CONNECTED ) 
    {
        isWifiConnected = true;
        gTftDisplayHndl.fillScreen(TFT_GREEN);
        delay(50);
        gTftDisplayHndl.setCursor(10, 40);
        gTftDisplayHndl.setTextColor(TFT_BLACK);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("WiFi Connected!");
        xEventGroupSetBits(networkEventGroup, EVT_WIFI_CONNECTED);
        xEventGroupClearBits(networkEventGroup, EVT_4G_CONNECTED);
        delay(2000); // Display success briefly
        return true;
    }
    else
    {
        isWifiConnected = false;
        gTftDisplayHndl.fillScreen(TFT_RED);
        delay(50);
        gTftDisplayHndl.setCursor(10, 40);
        gTftDisplayHndl.setTextColor(TFT_WHITE);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("Connection Failed!");
        delay(3000);
        return false;
    }
}

void promptWiFiConnection() 
{
    gTftDisplayHndl.fillScreen(TFT_BLACK);
    delay(50);
    gTftDisplayHndl.setRotation(1);

    Serial.println("[WIFI_SCAN] Starting WiFi scan...");
    int n = WiFi.scanNetworks();
    Serial.printf("[WIFI_SCAN] Scan complete. %d networks found.\n", n);

    if ( n == 0 ) 
    {
        gTftDisplayHndl.setCursor(10, 10);
        gTftDisplayHndl.setTextColor(TFT_RED);
        gTftDisplayHndl.setTextSize(2);
        gTftDisplayHndl.println("No networks found.");
        delay(3000);
        gbWifiConnectCanceled = true;
        return;
    }

    int topIndex = 0;
    int i32WifiSelectIndex = 0;

    Serial.println("[WIFI_DISPLAY] Drawing SSID list (initial)...");
    drawSSIDList(topIndex, i32WifiSelectIndex - topIndex, n);
    drawActionBar();
    Serial.println("[WIFI_DISPLAY] Initial draw complete.");

    while (true) 
    {
        char key = gKeypadMHndl.getKey();
        if ( key ) 
        {
            Serial.printf("[KEYPAD] Key pressed: %c\n", key);
            if (key == UP_KEY && i32WifiSelectIndex > 0) 
            {
                i32WifiSelectIndex--;
                if (i32WifiSelectIndex < topIndex) topIndex--;
                Serial.println("[WIFI_DISPLAY] Drawing SSID list (UP)");
                delay(20);
                drawSSIDList(topIndex, i32WifiSelectIndex - topIndex, n);
            } 
            else if (key == DOWN_KEY && i32WifiSelectIndex < n - 1) 
            {
                i32WifiSelectIndex++;
                if (i32WifiSelectIndex >= topIndex + MAX_VISIBLE_SSIDS) topIndex++;
                Serial.println("[WIFI_DISPLAY] Drawing SSID list (DOWN)");
                delay(20);
                drawSSIDList(topIndex, i32WifiSelectIndex - topIndex, n);
            } 
            else if (key == 'D') 
            { // Confirm selection
                Serial.printf("[WIFI_SELECT] Selected index: %d, SSID: %s\n", i32WifiSelectIndex, WiFi.SSID(i32WifiSelectIndex).c_str());
                break;
            } 
            else if (key == '*')
            { // Cancel
                Serial.println("[WIFI_CANCEL] WiFi connection canceled.");
                gbWifiConnectCanceled = true;
                return;
            } 
            else if (key == REFRESH_KEY) // Refresh scan list
            { 
                Serial.println("[WIFI_REFRESH] Starting WiFi scan (refresh)...");
                n = WiFi.scanNetworks();
                Serial.printf("[WIFI_REFRESH] Scan complete. %d networks found.\n", n);
                Serial.println("[WIFI_DISPLAY] Drawing SSID list (refresh)");
                delay(50);
                drawSSIDList(topIndex, i32WifiSelectIndex - topIndex, n);
                delay(100);
                drawActionBar();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
    }

    // Save selected SSID
    Serial.printf("[WIFI_SAVE] Saving selected SSID: %s\n", WiFi.SSID(i32WifiSelectIndex).c_str());
    strcpy(gacSSID, WiFi.SSID(i32WifiSelectIndex).c_str());

    // Move to password entry screen
    Serial.printf("[PASSWORD] Moving to password entry for SSID: %s\n", gacSSID);
    delay(50);// Give the display time to settle
    drawPasswordScreen(gacSSID);

    if ( gbWifiConnectCanceled )
    {
        gbWifiConnectCanceled = false;
        promptWiFiConnection();
        return;
    }

    // Attempt to connect using the new function
    Serial.printf("[WIFI_CONNECT] Attempting to connect to: %s\n", gacSSID);
    if ( !connectToWiFiNetwork( gacSSID, gacUserWifiPassword ) ) 
    {
        Serial.println("[WIFI_CONNECT] Connection failed, retrying prompt.");
        promptWiFiConnection(); // Retry if failed
    }
    else
    {
        Serial.println("[WIFI_CONNECT] Connection successful.");
    }
}

void connectToInternet() 
{
    // Priority 1: Try to connect to 4G
    internetSetup(); // This function sets is4GConnected
    if (is4GConnected) 
    {
        Serial.println("Connected to 4G.");
        //gMQTTClient.setClient(gsmClient);
        switchMQTTClient(true);
        xEventGroupSetBits(networkEventGroup, EVT_4G_CONNECTED);
        xEventGroupClearBits(networkEventGroup, EVT_WIFI_CONNECTED);        
        return;
    }
    // Priority 2: If 4G fails, try to connect to WiFi
    Serial.println("4G connection failed.  Trying WiFi...");
#if 0
    promptWiFiConnection();

    if (isWifiConnected) 
    {
        Serial.println("Connected to WiFi.");
        gTftDisplayHndl.println("Connected to WiFi.");
        return;
    }

    // If both WiFi and 4G failed
    Serial.println("Failed to connect to either WiFi or 4G.");
    // Handle the error.  For now, loop.
    while (true);
#endif    
}

void connectAndMonitorInternetTask( void *pvParameters ) 
{
    Serial.println("Connection and monitor thread start...");
    //InitLTEModule();

    while (true) 
    {
        if (!(xEventGroupGetBits(networkEventGroup) & EVT_INTERNET_CONNECTED)) 
        {
             connectToInternet();

             if ( is4GConnected ) 
             {
                gMQTTClient.setClient(gsmClient);
                xEventGroupSetBits(networkEventGroup, EVT_4G_CONNECTED);
                xEventGroupClearBits(networkEventGroup, EVT_WIFI_CONNECTED);
             }
             else if ( isWifiConnected )
             {
                gMQTTClient.setClient(gWifiespClient);
                xEventGroupSetBits(networkEventGroup, EVT_WIFI_CONNECTED);
                xEventGroupClearBits(networkEventGroup, EVT_4G_CONNECTED);
             }

             if ( xEventGroupGetBits(networkEventGroup) & EVT_INTERNET_CONNECTED )
             {
                 startMQTT();
             }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));  //check every 10 seconds
    }
}

bool IsInternetConnect( void )
{
    return ( xEventGroupGetBits(networkEventGroup) & EVT_INTERNET_CONNECTED ); 
}