/****************************************************************************

  Copyright (c) 2021 Ronald Sieber

  Project:      ESP32SmartBoard / MQTT Sensors
  Description:  MQTT Client Firmware to interact with ESP32SmartBoard

  -------------------------------------------------------------------------

    Arduino IDE Settings:

    Board:              "ESP32 Dev Module"
    Upload Speed:       "115200"
    CPU Frequency:      "240MHz (WiFi/BT)"
    Flash Frequency:    "80Mhz"
    Flash Mode:         "QIO"
    Flash Size:         "4MB (32Mb)"
    Partition Scheme:   "No OTA (2MB APP/2MB SPIFFS)"
    PSRAM:              "Disabled"

  -------------------------------------------------------------------------

  Revision History:

  2021/02/02 -rs:   V1.00 Initial version

****************************************************************************/

#define DEBUG                                                           // Enable/Disable TRACE
// #define DEBUG_DUMP_BUFFER


#include <WiFi.h>
#include <PubSubClient.h>                                               // Requires Library "PubSubClient" by Nick O'Leary
#include <DHT.h>                                                        // Requires Library "DHT sensor library" by Adafruit
#include <MHZ19.h>                                                      // Requires Library "MH-Z19" by Jonathan Dempsey
#include <SoftwareSerial.h>                                             // Requires Library "EspSoftwareSerial" by Dirk Kaar, Peter Lerup
#include <esp_wifi.h>
#include "Trace.h"





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Application Configuration
//---------------------------------------------------------------------------

const int       APP_VERSION                         = 1;                // 1.xx
const int       APP_REVISION                        = 0;                // x.00
const char      APP_BUILD_TIMESTAMP[]               = __DATE__ " " __TIME__;

const int       CFG_ENABLE_NETWORK_SCAN             = 1;
const int       CFG_ENABLE_DI_DO                    = 1;
const int       CFG_ENABLE_DHT_SENSOR               = 1;
const int       CFG_ENABLE_MHZ_SENSOR               = 1;
const int       CFG_ENABLE_STATUS_LED               = 1;



//---------------------------------------------------------------------------
//  Definitions
//---------------------------------------------------------------------------

typedef  void (*tMqttSubCallback)(const char* pszMsgTopic_p, const uint8_t* pMsgPayload_p, unsigned int uiMsgPayloadLen_p);

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])



//---------------------------------------------------------------------------
//  WLAN/WIFI Configuration
//---------------------------------------------------------------------------

// WIFI Station/Client Configuration
const char*     WIFI_STA_SSID                       = "YourNetworkName";            // <--- ADJUST HERE!
const char*     WIFI_STA_PASSWORD                   = "YourNetworkPassword";        // <--- ADJUST HERE!
IPAddress       WIFI_STA_LOCAL_IP(0,0,0,0);                             // DHCP: IP(0,0,0,0), otherwise static IP Address if running in Station Mode

// PowerSavingMode for WLAN Interface
//
// Modem-sleep mode includes minimum and maximum power save modes.
// In minimum power save mode, station wakes up every DTIM to receive beacon.
// Broadcast data will not be lost because it is transmitted after DTIM.
// However, it can not save much more power if DTIM is short for DTIM is determined by AP.
//
// WIFI_PS_NONE:        No PowerSaving,     Power Consumption: 110mA, immediate processing of receipt mqtt messages
// WIFI_PS_MIN_MODEM:   PowerSaving active, Power Consumption:  45mA, up to 5 sec delayed processing of receipt mqtt messages
// WIFI_PS_MAX_MODEM:   interval to receive beacons is determined by the listen_interval parameter in <wifi_sta_config_t>
wifi_ps_type_t  WIFI_POWER_SAVING_MODE              = WIFI_PS_MIN_MODEM;    // WIFI_PS_NONE

const uint32_t  WIFI_TRY_CONNECT_TIMEOUT            = 30000;            // Timeout for trying to connect to WLAN [ms]

const String    astrWiFiStatus_g[]                  = { "WL_IDLE_STATUS", "WL_NO_SSID_AVAIL", "WL_SCAN_COMPLETED", "WL_CONNECTED", "WL_CONNECT_FAILED", "WL_CONNECTION_LOST", "WL_DISCONNECTED" };



//---------------------------------------------------------------------------
//  Runtime Configuration
//---------------------------------------------------------------------------

// Toggle Time Periods for Status LED
const uint32_t  STATUS_LED_PERIOD_NET_SCAN          = 50;               // 10Hz   = 50ms On + 50ms Off
const uint32_t  STATUS_LED_PERIOD_NET_CONNECT       = 100;              // 5Hz    = 100ms On + 100ms Off
const uint32_t  STATUS_LED_PERIOD_MQTT_CONNECT      = 200;              // 2.5Hz  = 200ms On + 200ms Off
const uint32_t  STATUS_LED_PERIOD_MAIN_LOOP         = 2000;             // 0.25Hz = 2000ms On + 2000ms Off

// Data Sources for LED Bar Indicator
typedef enum
{
    kLedBarNone             = 0,
    kLedBarDht22Temperature = 1,
    kLedBarDht22Humidity    = 2,
    kLedBarMhz19Co2Level    = 3

} tLedBar;

// Default Board Configuration Settings
const int       DEFAULT_STATUS_LED_HEARTBEAT        = true;
const tLedBar   DEFAULT_LED_BAR_INDICATOR           = kLedBarMhz19Co2Level;
const bool      DEFAULT_PRINT_SENSOR_VALUES         = true;
const bool      DEFAULT_PRINT_MQTT_DATA_PROC        = true;

// MH-Z19 Settings
const bool      DEFAULT_MHZ19_AUTO_CALIBRATION      = true;
const bool      DEFAULT_DBG_MHZ19_PRINT_COMM        = false;



//---------------------------------------------------------------------------
//  Hardware/Pin Configuration
//---------------------------------------------------------------------------

const int       PIN_KEY_BLE_CFG                     = 36;               // PIN_KEY_BLE_CFG      (GPIO36 -> Pin02)
const int       PIN_KEY0                            = 35;               // KEY0                 (GPIO35 -> Pin05)
const int       PIN_KEY1                            = 34;               // KEY1                 (GPIO34 -> Pin04)

const int       PIN_LED0                            = 13;               // LED0 (green)         (GPIO13 -> Pin13)
const int       PIN_LED1                            = 12;               // LED1 (green)         (GPIO12 -> Pin12)
const int       PIN_LED2                            = 14;               // LED2 (green)         (GPIO14 -> Pin11)
const int       PIN_LED3                            =  4;               // LED3 (yellow)        (GPIO04 -> Pin20)
const int       PIN_LED4                            =  5;               // LED4 (yellow)        (GPIO05 -> Pin23)
const int       PIN_LED5                            = 18;               // LED5 (yellow)        (GPIO18 -> Pin24)
const int       PIN_LED6                            = 19;               // LED6 (red)           (GPIO19 -> Pin25)
const int       PIN_LED7                            = 21;               // LED7 (red)           (GPIO21 -> Pin26)
const int       PIN_LED8                            = 22;               // LED8 (red)           (GPIO22 -> Pin29)
const int       PIN_STATUS_LED                      =  2;               // On-board LED (blue)  (GPIO02 -> Pin19)

const int       DHT_TYPE                            = DHT22;            // DHT11, DHT21 (AM2301), DHT22 (AM2302,AM2321)
const int       DHT_PIN_SENSOR                      = 23;               // PIN used for DHT22 (AM2302/AM2321)
const uint32_t  DHT_SENSOR_SAMPLE_PERIOD            = 60000             // Sample Period for DHT-Sensor in [ms]

const int       MHZ19_PIN_SERIAL_RX                 = 32;               // ESP32 Rx pin which the MH-Z19 Tx pin is attached to
const int       MHZ19_PIN_SERIAL_TX                 = 33;               // ESP32 Tx pin which the MH-Z19 Rx pin is attached to
const int       MHZ19_BAUDRATE_SERIAL               = 9600;             // Serial baudrate for communication with MH-Z19 Device
const uint32_t  MHZ19_SENSOR_SAMPLE_PERIOD          = 60000             // Sample Period for MH-Z19-Sensor in [ms]



//---------------------------------------------------------------------------
//  MQTT Configuration
//---------------------------------------------------------------------------

const char*     MQTT_SERVER                         = "192.168.69.1";
const int       MQTT_PORT                           = 1883;
const char*     MQTT_USER                           = "SmBrd_IoT";
const char*     MQTT_PASSWORD                       = "xxx";
const char*     MQTT_CLIENT_PREFIX                  = "SmBrd_";         // will be extended by ChipID
const char*     MQTT_DEVICE_ID                      = NULL;             // will be used to generate Device specific Topics
const char*     MQTT_SUPERVISOR_TOPIC               = "SmBrd_Supervisor";

// IMPORTANT: The Entries in this Array are accessed via fixed indices
//            within the Function 'AppProcessMqttDataMessage()'.
const char*     MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE[] =
{
    "SmBrd/<%>/Settings/Heartbeat",                                     // "<%>" will be replaced by DevID
    "SmBrd/<%>/Settings/LedBarIndicator",                               // "<%>" will be replaced by DevID
    "SmBrd/<%>/Settings/PrintSensorVal",                                // "<%>" will be replaced by DevID
    "SmBrd/<%>/Settings/PrintMqttDataProc",                             // "<%>" will be replaced by DevID
    "SmBrd/<%>/OutData/LedBar",                                         // "<%>" will be replaced by DevID
    "SmBrd/<%>/OutData/LedBarInv",                                      // "<%>" will be replaced by DevID
    "SmBrd/<%>/OutData/Led"                                             // "<%>" will be replaced by DevID
};

const char*     MQTT_PUBLISH_TOPIC_LIST_TEMPLATE[] =
{
    "SmBrd/<%>/InData/Key0",                                            // "<%>" will be replaced by DevID
    "SmBrd/<%>/InData/Key1",                                            // "<%>" will be replaced by DevID
    "SmBrd/<%>/InData/Temperature",                                     // "<%>" will be replaced by DevID
    "SmBrd/<%>/InData/Humidity",                                        // "<%>" will be replaced by DevID
    "SmBrd/<%>/InData/CO2",                                             // "<%>" will be replaced by DevID
    "SmBrd/<%>/InData/SensTemp"                                         // "<%>" will be replaced by DevID
};



//---------------------------------------------------------------------------
//  Local Variables
//---------------------------------------------------------------------------

static  WiFiClient      WiFiClient_g;
static  PubSubClient    PubSubClient_g(WiFiClient_g);

static  String          strChipID_g;
static  String          strDeviceID_g;
static  String          strClientName_g;

static  unsigned int    uiMainLoopProcStep_g        = 0;
static  int             iLastStateKey0_g            = -1;
static  int             iLastStateKey1_g            = -1;

static  DHT             DhtSensor_g(DHT_PIN_SENSOR, DHT_TYPE);
static  uint32_t        ui32LastTickDhtRead_g       = 0;
static  float           flDhtTemperature_g          = 0;
static  float           flDhtHumidity_g             = 0;

static  MHZ19           Mhz19Sensor_g;
static  SoftwareSerial  Mhz19SoftSerial_g(MHZ19_PIN_SERIAL_RX, MHZ19_PIN_SERIAL_TX);
static  uint32_t        ui32LastTickMhz19Read_g     = 0;
static  int             iMhz19Co2Value_g            = 0;
static  int             iMhz19Co2SensTemp_g         = 0;
static  bool            fMhz19AutoCalibration_g     = DEFAULT_MHZ19_AUTO_CALIBRATION;
static  bool            fMhz19DbgPrintComm_g        = DEFAULT_DBG_MHZ19_PRINT_COMM;

static  bool            fStatusLedHeartbeat_g       = DEFAULT_STATUS_LED_HEARTBEAT;
static  hw_timer_t*     pfnOnTimerISR_g             = NULL;
static  volatile byte   bStatusLedState_g           = LOW;
static  uint32_t        ui32LastTickStatLedToggle_g = 0;

static  tLedBar         LedBarIndicator_g           = DEFAULT_LED_BAR_INDICATOR;
static  bool            fPrintSensorValues_g        = DEFAULT_PRINT_SENSOR_VALUES;
static  bool            fPrintMqttDataProc_g        = DEFAULT_PRINT_MQTT_DATA_PROC;

static  String          astrMqttSubscribeTopicList_g[ARRAY_SIZE(MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE)];
static  String          astrMqttPublishTopicList_g[ARRAY_SIZE(MQTT_PUBLISH_TOPIC_LIST_TEMPLATE)];





//=========================================================================//
//                                                                         //
//          S K E T C H   P U B L I C   F U N C T I O N S                  //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Application Setup
//---------------------------------------------------------------------------

void setup()
{

char  szTextBuff[64];
char  acMhz19Version[4];
int   iMhz19Param;
bool  fMhz19AutoCalibration;
bool  fStateKey0;
bool  fStateKey1;
bool  fSensorCalibrated;
int   iIdx;


    // Serial console
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Serial.println("======== APPLICATION START ========");
    Serial.println();
    Serial.flush();


    // Application Version Information
    snprintf(szTextBuff, sizeof(szTextBuff), "App Version:      %u.%02u", APP_VERSION, APP_REVISION);
    Serial.println(szTextBuff);
    snprintf(szTextBuff, sizeof(szTextBuff), "Build Timestamp:  %s", APP_BUILD_TIMESTAMP);
    Serial.println(szTextBuff);
    Serial.println();
    Serial.flush();


    // Device Identification
    strChipID_g = GetChipID();
    Serial.print("Unique ChipID:    ");
    Serial.println(strChipID_g);
    Serial.println();
    Serial.flush();


    // Initialize Workspace
    flDhtTemperature_g  = 0;
    flDhtHumidity_g     = 0;
    iMhz19Co2Value_g    = 0;
    iMhz19Co2SensTemp_g = 0;


    // Setup DI/DO (Key/LED)
    if ( CFG_ENABLE_DI_DO )
    {
        Serial.println("Setup DI/DO...");
        pinMode(PIN_KEY0, INPUT);
        pinMode(PIN_KEY1, INPUT);
        pinMode(PIN_LED0, OUTPUT);
        pinMode(PIN_LED1, OUTPUT);
        pinMode(PIN_LED2, OUTPUT);
        pinMode(PIN_LED3, OUTPUT);
        pinMode(PIN_LED4, OUTPUT);
        pinMode(PIN_LED5, OUTPUT);
        pinMode(PIN_LED6, OUTPUT);
        pinMode(PIN_LED7, OUTPUT);
        pinMode(PIN_LED8, OUTPUT);
    }

    iLastStateKey0_g = -1;
    iLastStateKey1_g = -1;


    // Setup DHT22 Sensor (Temerature/Humidity)
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        Serial.println("Setup DHT22 Sensor...");
        DhtSensor_g.begin();
        snprintf(szTextBuff, sizeof(szTextBuff), "  Sample Period:    %d [sec]", (DHT_SENSOR_SAMPLE_PERIOD / 1000));
        Serial.println(szTextBuff);
    }


    // Setup MH-Z19 Sensor (CO2)
    if ( CFG_ENABLE_MHZ_SENSOR )
    {
        Serial.println("Setup MH-Z19 Sensor...");
        Mhz19SoftSerial_g.begin(MHZ19_BAUDRATE_SERIAL);                 // Initialize Software Serial Device to communicate with MH-Z19 sensor
        Mhz19Sensor_g.begin(Mhz19SoftSerial_g);                         // Initialize MH-Z19 Sensor (using Software Serial Device)
        if ( fMhz19DbgPrintComm_g )
        {
            Mhz19Sensor_g.printCommunication(false, true);              // isDec=false, isPrintComm=true
        }
        fStateKey0 = !digitalRead(PIN_KEY0);                            // Keys are inverted (1=off, 0=on)
        if ( fStateKey0 )                                               // KEY0 pressed?
        {
            fSensorCalibrated = AppMhz19CalibrateManually();
            if ( fSensorCalibrated )
            {
                fStateKey0 = false;                                     // if calibration was done, don't invert AutoCalibration mode
            }
        }
        fStateKey1 = !digitalRead(PIN_KEY1);                            // Keys are inverted (1=off, 0=on)
        if ( fStateKey1 )                                               // KEY1 pressed?
        {
            AppMhz19CalibrateUnattended();
            ESP.restart();
        }
        fMhz19AutoCalibration = fMhz19AutoCalibration_g ^ fStateKey0;
        Mhz19Sensor_g.autoCalibration(fMhz19AutoCalibration);           // Set AutoCalibration Mode (true=ON / false=OFF)
        Mhz19Sensor_g.getVersion(acMhz19Version);
        snprintf(szTextBuff, sizeof(szTextBuff), "  Firmware Version: %c%c.%c%c", acMhz19Version[0], acMhz19Version[1], acMhz19Version[2], acMhz19Version[3]);
        Serial.println(szTextBuff);
        iMhz19Param = (int)Mhz19Sensor_g.getABC();
        snprintf(szTextBuff, sizeof(szTextBuff), "  AutoCalibration:  %s", (iMhz19Param) ? "ON" : "OFF");
        Serial.println(szTextBuff);
        iMhz19Param = Mhz19Sensor_g.getRange();
        snprintf(szTextBuff, sizeof(szTextBuff), "  Range:            %d", iMhz19Param);
        Serial.println(szTextBuff);
        iMhz19Param = Mhz19Sensor_g.getBackgroundCO2();
        snprintf(szTextBuff, sizeof(szTextBuff), "  Background CO2:   %d", iMhz19Param);
        Serial.println(szTextBuff);
        snprintf(szTextBuff, sizeof(szTextBuff), "  Sample Period:    %d [sec]", (MHZ19_SENSOR_SAMPLE_PERIOD / 1000));
        Serial.println(szTextBuff);
    }


    // Status LED Setup
    if ( CFG_ENABLE_STATUS_LED )
    {
        pinMode(PIN_STATUS_LED, OUTPUT);
        ui32LastTickStatLedToggle_g = 0;
        bStatusLedState_g = LOW;
    }


    // Default Board Configuration Settings
    fStatusLedHeartbeat_g = DEFAULT_STATUS_LED_HEARTBEAT;
    LedBarIndicator_g     = DEFAULT_LED_BAR_INDICATOR;
    fPrintSensorValues_g  = DEFAULT_PRINT_SENSOR_VALUES;
    fPrintMqttDataProc_g  = DEFAULT_PRINT_MQTT_DATA_PROC;


    // Network Scan
    if ( CFG_ENABLE_NETWORK_SCAN )
    {
        Serial.println();
        WifiScanNetworks();
    }


    // Network Setup
    WifiConnectStationMode(WIFI_STA_SSID, WIFI_STA_PASSWORD, WIFI_STA_LOCAL_IP, WIFI_POWER_SAVING_MODE);


    // MQTT Setup
    MqttCheckNetworkConfig(WiFi.localIP(), MQTT_SERVER);
    strDeviceID_g = GetDeviceID(MQTT_DEVICE_ID);
    strClientName_g = GetUniqueClientName(MQTT_CLIENT_PREFIX);
    for (iIdx=0; iIdx<ARRAY_SIZE(MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE); iIdx++)
    {
        astrMqttSubscribeTopicList_g[iIdx] = MqttBuildTopicFromTemplate(MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE[iIdx], strChipID_g.c_str());
    }
    for (iIdx=0; iIdx<ARRAY_SIZE(MQTT_PUBLISH_TOPIC_LIST_TEMPLATE); iIdx++)
    {
        astrMqttPublishTopicList_g[iIdx] = MqttBuildTopicFromTemplate(MQTT_PUBLISH_TOPIC_LIST_TEMPLATE[iIdx], strChipID_g.c_str());
    }
    MqttPrintTopicLists (String(MQTT_SUPERVISOR_TOPIC), astrMqttSubscribeTopicList_g, ARRAY_SIZE(astrMqttSubscribeTopicList_g), astrMqttPublishTopicList_g, ARRAY_SIZE(astrMqttPublishTopicList_g));
    MqttSetup(MQTT_SERVER, MQTT_PORT, AppMqttSubCallback);
    MqttConnect(MQTT_USER, MQTT_PASSWORD, strClientName_g.c_str(), MQTT_SUPERVISOR_TOPIC, fPrintMqttDataProc_g);
    MqttSubscribeTopicList(astrMqttSubscribeTopicList_g, ARRAY_SIZE(astrMqttSubscribeTopicList_g), fPrintMqttDataProc_g);


    // Publish initial values
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        AppProcessDhtSensor(0, fPrintSensorValues_g, fPrintMqttDataProc_g);
    }
    if ( CFG_ENABLE_MHZ_SENSOR )
    {
        AppProcessMhz19Sensor(0, fPrintSensorValues_g, fPrintMqttDataProc_g);
    }


    return;

}



//---------------------------------------------------------------------------
//  Application Main Loop
//---------------------------------------------------------------------------

void loop()
{

unsigned int  uiProcStep;
uint32_t      ui32CurrTick;


    // ensure that WLAN/WIFI and MQTT are available
    AppEnsureNetworkAvailability();

    // process WLAN/WIFI and MQTT
    PubSubClient_g.loop();

    // process local periphery (i/o, sensors)
    uiProcStep = uiMainLoopProcStep_g++ % 10;
    switch (uiProcStep)
    {
        case 0:
        {
            // process local digital inputs
            if ( CFG_ENABLE_DI_DO )
            {
                AppProcessInputs();
            }
            break;
        }

        case 1:
        {
            // process DHT Sensor (Temperature/Humidity)
            if ( CFG_ENABLE_DHT_SENSOR )
            {
                AppProcessDhtSensor(DHT_SENSOR_SAMPLE_PERIOD, fPrintSensorValues_g, fPrintMqttDataProc_g);
            }
            break;
        }

        case 2:
        {
            // process MH-Z19 Sensor (CO2Value/SensorTemperature)
            if ( CFG_ENABLE_MHZ_SENSOR )
            {
                AppProcessMhz19Sensor(MHZ19_SENSOR_SAMPLE_PERIOD, fPrintSensorValues_g, fPrintMqttDataProc_g);
            }
            break;
        }
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        {
            break;
        }

        case 9:
        {
            // process LED Bar Indicator depending on its data source
            AppProcessLedBarIndicator();
            break;
        }

        default:
        {
            break;
        }
    }

    // toggle Status LED
    if ( CFG_ENABLE_STATUS_LED )
    {
        if ( fStatusLedHeartbeat_g )
        {
            ui32CurrTick = millis();
            if ((ui32CurrTick - ui32LastTickStatLedToggle_g) >= STATUS_LED_PERIOD_MAIN_LOOP)
            {
                bStatusLedState_g = !bStatusLedState_g;
                digitalWrite(PIN_STATUS_LED, bStatusLedState_g);

                ui32LastTickStatLedToggle_g = ui32CurrTick;
            }
        }
    }

    delay(50);

    return;

}





//=========================================================================//
//                                                                         //
//          S K E T C H   P R I V A T E   F U N C T I O N S                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Ensure that Network (WLAN/WiFi+MQTT) is available
//---------------------------------------------------------------------------

void  AppEnsureNetworkAvailability()
{

int  iWiFiStatus;


    // verify that WLAN/WiFi connection is still active
    iWiFiStatus = WiFi.status();
    if (iWiFiStatus != WL_CONNECTED)
    {
        Serial.println();
        Serial.println("ERROR: WLAN Connection lost, try to Reconnet...");
        Serial.print("       (-> '");
        Serial.print(astrWiFiStatus_g[iWiFiStatus]);
        Serial.println("'");

        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);

        WifiConnectStationMode(WIFI_STA_SSID, WIFI_STA_PASSWORD, WIFI_STA_LOCAL_IP, WIFI_POWER_SAVING_MODE);

        // force MQTT reconnect in next step
        PubSubClient_g.disconnect();
    }

    // verify that MQTT connection is still active
    if ( !PubSubClient_g.connected() )
    {
        Serial.println();
        Serial.println("ERROR: MQTT Connection lost, try to Reconnet...");

        MqttConnect(MQTT_USER, MQTT_PASSWORD, strClientName_g.c_str(), MQTT_SUPERVISOR_TOPIC, fPrintMqttDataProc_g);
        MqttSubscribeTopicList(astrMqttSubscribeTopicList_g, ARRAY_SIZE(astrMqttSubscribeTopicList_g), fPrintMqttDataProc_g);
    }

    return;

}





//=========================================================================//
//                                                                         //
//          W I F I   N E T W O R K   F U N C T I O N S                    //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Scan WLAN/WiFi Networks
//---------------------------------------------------------------------------

void  WifiScanNetworks()
{

int  iNumOfWlanNets;
int  iIdx;


    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStart(STATUS_LED_PERIOD_NET_SCAN);                // 10Hz = 50ms On + 50ms Off
    }


    Serial.println("Scanning for WLAN Networks...");
    iNumOfWlanNets = WiFi.scanNetworks();
    if (iNumOfWlanNets > 0)
    {
        Serial.print("Networks found within Range: ");
        Serial.println(iNumOfWlanNets);
        Serial.println("---------------------------------+------+----------------");
        Serial.println("               SSID              | RSSI |    AUTH MODE   ");
        Serial.println("---------------------------------+------+----------------");
        for (iIdx=0; iIdx<iNumOfWlanNets; iIdx++)
        {
            Serial.printf("%32.32s | ", WiFi.SSID(iIdx).c_str());
            Serial.printf("%4d | ", WiFi.RSSI(iIdx));
            Serial.printf("%15s\n", WifiAuthModeToText(WiFi.encryptionType(iIdx)).c_str());
        }
    }
    else
    {
        Serial.println("No Networks found.");
    }

    Serial.println("");

    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStop();
    }

    return;

}



//---------------------------------------------------------------------------
//  Setup WLAN/WiFi SubSystem in Station Mode
//---------------------------------------------------------------------------

void  WifiConnectStationMode (const char* pszWifiSSID_p, const char* pszWifiPassword_p, IPAddress LocalIP_p, wifi_ps_type_t WifiPowerSavingMode_p)
{

IPAddress  WIFI_STA_GATEWAY(0,0,0,0);                               // no Gateway functionality supported
IPAddress  WIFI_STA_SUBNET(255,255,255,0);
uint32_t   ui32ConnectStartTicks;
bool       fResult;


    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStart(STATUS_LED_PERIOD_NET_CONNECT);             // 5Hz = 100ms On + 100ms Off
    }

    Serial.println("Setup WiFi Interface as STATION (Client)");

    // Set PowerSavingMode for WLAN Interface
    Serial.print("Set PowerSavingMode for WLAN Interface: ");
    switch (WifiPowerSavingMode_p)
    {
        case WIFI_PS_NONE:          Serial.println("OFF (WIFI_PS_NONE)");                   break;
        case WIFI_PS_MIN_MODEM:     Serial.println("ON / using DTIM (WIFI_PS_MIN_MODEM)");  break;
        case WIFI_PS_MAX_MODEM:     Serial.println("ON (WIFI_PS_MAX_MODEM)");               break;
        default:                    Serial.println("???unknown???");                        break;
    }
    esp_wifi_set_ps(WifiPowerSavingMode_p);

    // Set WiFi Mode to Station
    Serial.println("Set WiFi Mode to Station:");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    Serial.println("  -> done.");

    // Set IPAddress configuration
    // 0.0.0.0 = DHCP, otherwise set static IPAddress
    Serial.print("IP Address configuration: ");
    if (LocalIP_p != IPAddress(0,0,0,0))
    {
        Serial.print("static ");
        Serial.println(LocalIP_p.toString());
        WiFi.config(LocalIP_p, WIFI_STA_GATEWAY, WIFI_STA_SUBNET);
        Serial.println("  -> done.");
    }
    else
    {
        Serial.println("DHCP");
    }

    // Connecting to WLAN/WiFi
    Serial.print("Connecting to WLAN '");
    Serial.print(pszWifiSSID_p);
    Serial.print("' ");
    WiFi.begin(pszWifiSSID_p, pszWifiPassword_p);

    ui32ConnectStartTicks = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if ((millis() - ui32ConnectStartTicks) > WIFI_TRY_CONNECT_TIMEOUT)
        {
            Serial.println("");
            Serial.print("ERROR: Unable to connect to WLAN since more than ");
            Serial.print(WIFI_TRY_CONNECT_TIMEOUT/1000);
            Serial.println(" seconds.");
            Serial.println("-> REBOOT System now...");
            Serial.println("");

            ESP.restart();
        }

        delay(500);
        Serial.print(".");
    }
    Serial.println(" -> connected.");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("   (MAC: ");
    Serial.print(GetChipMAC());
    Serial.println(")");

    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStop();
    }

    return;

}



//---------------------------------------------------------------------------
//  Get WLAN/WiFi Authentication Mode as Text
//---------------------------------------------------------------------------

String  WifiAuthModeToText (wifi_auth_mode_t WifiAuthMode_p)
{

String  strWifiAuthMode;


    switch (WifiAuthMode_p)
    {
        case WIFI_AUTH_OPEN:
        {
            strWifiAuthMode = "Open";
            break;
        }

        case WIFI_AUTH_WEP:
        {
            strWifiAuthMode = "WEP";
            break;
        }

        case WIFI_AUTH_WPA_PSK:
        {
            strWifiAuthMode = "WPA PSK";
            break;
        }

        case WIFI_AUTH_WPA2_PSK:
        {
            strWifiAuthMode = "WPA2 PSK";
            break;
        }

        case WIFI_AUTH_WPA_WPA2_PSK:
        {
            strWifiAuthMode = "WPA/WPA2 PSK";
            break;
        }

        case WIFI_AUTH_WPA2_ENTERPRISE:
        {
            strWifiAuthMode = "WPA2 ENTERPRISE";
            break;
        }

        case WIFI_AUTH_MAX:
        {
            strWifiAuthMode = "MAX";
            break;
        }

        default:
        {
            strWifiAuthMode = "Unknown";
            break;
        }
    }

    return (strWifiAuthMode);

}





//=========================================================================//
//                                                                         //
//          M Q T T   M E S S A G E   F U N C T I O N S                    //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  MQTT Check Network Configuration
//---------------------------------------------------------------------------

bool  MqttCheckNetworkConfig (IPAddress LocalIP_p, const char* pszMqttServer_p)
{

IPAddress  MqttServer;
bool       fMismatch;
int        iIdx;


    MqttServer.fromString(pszMqttServer_p);

    // check LocalIP_p[0][1][2][*] with MqttServer[0][1][2][*]
    fMismatch = false;
    for (iIdx=0; iIdx<3; iIdx++)
    {
        if (LocalIP_p[iIdx] != MqttServer[iIdx])
        {
            fMismatch = true;
            break;
        }
    }

    if ( fMismatch )
    {
        Serial.println();
        Serial.println("******** WARNING ********");
        Serial.println("MQTT Server is located in a external Subnet:");
        Serial.print("MQTT Server:      ");  Serial.println(MqttServer);
        Serial.print("Local IP Address: ");  Serial.println(LocalIP_p);
        Serial.println("This Configuration can lead to a hang during connect.");
        Serial.println("*************************");
        Serial.println();
    }

    return (fMismatch);

}



//---------------------------------------------------------------------------
//  MQTT Setup
//---------------------------------------------------------------------------

void  MqttSetup (const char* pszMqttServer_p, int iMqttPort_p, tMqttSubCallback pfnMqttSubCallback_p)
{

    Serial.print("MQTT Configuration: Server='");
    Serial.print(pszMqttServer_p);
    Serial.print("', Port=");
    Serial.println(iMqttPort_p);

    PubSubClient_g.setServer(pszMqttServer_p, iMqttPort_p);

    if (pfnMqttSubCallback_p != NULL)
    {
        Serial.println("Register MQTT Callback Function");
        PubSubClient_g.setCallback(pfnMqttSubCallback_p);
    }

    return;

}



//---------------------------------------------------------------------------
//  MQTT Connect
//---------------------------------------------------------------------------

bool  MqttConnect (const char* pszMqttUser_p, const char* pszMqttPassword_p, const char* pszClientName_p, const char* pszSupervisorTopic_p, bool fPrintMqttDataProc_p)
{

String     strChipID;
String     strClientId;
IPAddress  LocalIP;
String     strRuntimeConfig;
String     strPayloadMsgBody;
String     strPayloadMsgConnect;
String     strPayloadMsgGotLost;
bool       fConnected;
bool       fRes;


    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStart(STATUS_LED_PERIOD_MQTT_CONNECT);            // 2.5Hz = 200ms On + 200ms Off
    }

    // Get unique ClientID
    if (pszClientName_p != NULL)
    {
        // reuse given ClientID
        strClientId = pszClientName_p;
    }
    else
    {
        // Create a unique ClientID, based on ChipID (the ChipID is essentially its 6byte MAC address)
        strChipID    = GetChipID();
        strClientId  = "SmBrdClient-";
        strClientId += strChipID;
    }

    // If SupervisorTopic is set, then create 'Connect' and 'GotLost' message payload
    if (pszSupervisorTopic_p != NULL)
    {
        LocalIP = WiFi.localIP();
        strRuntimeConfig = AppGetRuntimeConfig();
        strPayloadMsgBody = "CI=" + strClientId + ", IP=" + LocalIP.toString() + ", RC=" + strRuntimeConfig;
        strPayloadMsgConnect = strPayloadMsgBody + ", ST=Connected";
        strPayloadMsgGotLost = strPayloadMsgBody + ", ST=GotLost";
    }


    // Connect to MQTT Server
    Serial.print("Connecting to MQTT Server: User='");
    Serial.print(pszMqttUser_p);
    Serial.print("', ClientId='");
    Serial.print(strClientId);
    Serial.print("' ...");

    fConnected = false;
    do
    {
        // check if WLAN/WiFi Connection is established
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println();
            Serial.println("ERROR: No WLAN Connection -> unable to establish MQTT!");
            Serial.println();
            goto Exit;
        }

        if (pszSupervisorTopic_p != NULL)
        {
            fConnected = PubSubClient_g.connect(strClientId.c_str(),            // ClientID
                                                pszMqttUser_p,                  // Username
                                                pszMqttPassword_p,              // Password
                                                pszSupervisorTopic_p,           // LastWillTopic
                                                0,                              // LastWillQoS
                                                1,                              // LastWillRetain
                                                strPayloadMsgGotLost.c_str(),   // LastWillMessage
                                                true);                          // CleanSession
        }
        else
        {
            fConnected = PubSubClient_g.connect(strClientId.c_str(),            // ClientID
                                                pszMqttUser_p,                  // Username
                                                pszMqttPassword_p);             // Password
        }

        if ( fConnected )
        {
            Serial.println(" -> connected.");
        }
        else
        {
            Serial.println();
            Serial.print(" -> Failed, rc=");
            Serial.print(PubSubClient_g.state());
            Serial.println(", try again in 5 seconds");
            delay(5000);
        }
    }
    while ( !fConnected );


    // If SupervisorTopic is set, then publish 'Connect' message
    if (pszSupervisorTopic_p != NULL)
    {
        if ( fPrintMqttDataProc_p )
        {
            Serial.print("Publish Supervisor Message: Topic='");
            Serial.print(pszSupervisorTopic_p);
            Serial.print("', Payload='");
            Serial.print(strPayloadMsgConnect.c_str());
            Serial.print("'");
        }

        fRes = PubSubClient_g.publish(pszSupervisorTopic_p,                     // Topic
                                      strPayloadMsgConnect.c_str(),             // Payload
                                      1);                                       // Retain
        if ( fPrintMqttDataProc_p )
        {
            if ( fRes )
            {
                Serial.println(" -> ok.");
            }
            else
            {
                Serial.println(" -> failed.");
            }
        }
    }


Exit:

    if ( CFG_ENABLE_STATUS_LED )
    {
        Esp32TimerStop();
    }

    return (fConnected);

}



//---------------------------------------------------------------------------
//  MQTT Subscribe Topic / Topic List
//---------------------------------------------------------------------------

bool  MqttSubscribeTopicList (String astrMqttSubscribeTopicList_p[], int iSizeOfTopicList_p, bool fPrintMqttDataProc_p)
{

int   iIdx;
bool  fRes;


    for (iIdx=0; iIdx<iSizeOfTopicList_p; iIdx++)
    {
        fRes = MqttSubscribeTopic(astrMqttSubscribeTopicList_p[iIdx].c_str(), fPrintMqttDataProc_p);
        if ( !fRes )
        {
            break;
        }
    }

    return (fRes);

}

//---------------------------------------------------------------------------

bool  MqttSubscribeTopic (const char* pszMsgTopic_p, bool fPrintMqttDataProc_p)
{

bool  fRes;


    if ( fPrintMqttDataProc_p )
    {
        Serial.print("Subscribe: Topic='");
        Serial.print(pszMsgTopic_p);
        Serial.print("'");
        Serial.flush();
    }

    fRes = PubSubClient_g.subscribe(pszMsgTopic_p);

    if ( fPrintMqttDataProc_p )
    {
        if ( fRes )
        {
            Serial.println(" -> ok.");
        }
        else
        {
            Serial.println(" -> failed.");
        }
    }

    return (fRes);

}



//---------------------------------------------------------------------------
//  MQTT Publish Data Message
//---------------------------------------------------------------------------

bool  MqttPublishData (const char* pszMsgTopic_p, const char* pszMsgPayload_p, bool fRetain_p, bool fPrintMqttDataProc_p)
{

bool  fRes;


    if ( fPrintMqttDataProc_p )
    {
        Serial.print("Publish Data Message: Topic='");
        Serial.print(pszMsgTopic_p);
        Serial.print("', Payload='");
        Serial.print(pszMsgPayload_p);
        Serial.print("'");
        Serial.flush();
    }

    fRes = PubSubClient_g.publish(pszMsgTopic_p,                                // Topic
                                  pszMsgPayload_p,                              // Payload
                                  fRetain_p);                                   // Retain
    if ( fPrintMqttDataProc_p )
    {
        if ( fRes )
        {
            Serial.println(" -> ok.");
        }
        else
        {
            Serial.println(" -> failed.");
        }
    }

    return (fRes);

}



//---------------------------------------------------------------------------
//  MQTT Receive Data Nessage (for subscribed MQTT Topics)
//---------------------------------------------------------------------------

void  AppMqttSubCallback (const char* pszMsgTopic_p, const uint8_t* pMsgPayload_p, unsigned int uiMsgPayloadLen_p)
{

String  strMsgTopic;
String  strMsgPayload;


    // convert Topic and Payload Buffers into String
    strMsgTopic   = MqttConvBuffToString((const uint8_t*)pszMsgTopic_p, -1);
    strMsgPayload = MqttConvBuffToString(pMsgPayload_p, uiMsgPayloadLen_p);

    if ( fPrintMqttDataProc_g )
    {
        Serial.println("");
        Serial.print("Received Data Message: Topic='");
        Serial.print(strMsgTopic);
        Serial.print("', Payload='");
        Serial.print(strMsgPayload);
        Serial.print("', PayloadLen=");
        Serial.print(uiMsgPayloadLen_p);
        Serial.println("");
    }

    // process received MQTT Data Message
    AppProcessMqttDataMessage(strMsgTopic, strMsgPayload);

    if ( fPrintMqttDataProc_g )
    {
        Serial.println("");
    }

    return;

}



//---------------------------------------------------------------------------
//  MQTT Build Topic from Template
//---------------------------------------------------------------------------

String  MqttBuildTopicFromTemplate (const char* pszTopicTemplate_p, const char* pszSignatur_p)
{

String  strTemplate;
String  strTopic;
int     iIdx;


    strTemplate = String(pszTopicTemplate_p);

    // split-up Template into left and right part, insert signature in between
    iIdx = strTemplate.indexOf("<%>");
    if ( (iIdx >= 0) && ((iIdx+sizeof("<%>")-1) < strTemplate.length()) )
    {
        strTopic  = strTemplate.substring(0, iIdx);
        strTopic += String(pszSignatur_p);
        strTopic += strTemplate.substring((iIdx+sizeof("<%>")-1), strTemplate.length());
    }
    else
    {
        strTopic = strTemplate;
    }

    return (strTopic);

}



//---------------------------------------------------------------------------
//  MQTT Print Subscribe and Publish Topic Lists
//---------------------------------------------------------------------------

void  MqttPrintTopicLists (String strMqttSupervisorTopic_p, String astrMqttSubscribeTopicList_p[], int iSizeOfSubscribeTopicList_p, String astrMqttPublishTopicList_p[], int iSizeOfPublishTopicList_p)
{

char  szLineBuff[128];
int   iIdx;


    Serial.println("MQTT Supervisor Topic:");
    snprintf(szLineBuff, sizeof(szLineBuff), "  SUP: '%s'", strMqttSupervisorTopic_p.c_str());
    Serial.println(szLineBuff);

    Serial.println("MQTT Subscribe Topic List:");
    for (iIdx=0; iIdx<iSizeOfSubscribeTopicList_p; iIdx++)
    {
        snprintf(szLineBuff, sizeof(szLineBuff), "  S%02d: '%s'", iIdx, astrMqttSubscribeTopicList_p[iIdx].c_str());
        Serial.println(szLineBuff);
    }
    Serial.flush();

    Serial.println("MQTT Publish Topic List:");
    for (iIdx=0; iIdx<iSizeOfPublishTopicList_p; iIdx++)
    {
        snprintf(szLineBuff, sizeof(szLineBuff), "  P%02d: '%s'", iIdx, astrMqttPublishTopicList_p[iIdx].c_str());
        Serial.println(szLineBuff);
    }
    Serial.flush();

    return;

}



//---------------------------------------------------------------------------
//  MQTT Convert Buffer to String
//---------------------------------------------------------------------------

String  MqttConvBuffToString (const uint8_t* pBuffer_p, unsigned int uiBufferLen_p)
{

unsigned int  uiBufferLen;
char          acBuffer[MQTT_MAX_PACKET_SIZE+1];
String        strBuffer;


    // Is ptr to buffer vald?
    if (pBuffer_p != NULL)
    {
        // Is buffer length declared explicitly?
        if (uiBufferLen_p < 0)
        {
            // no buffer length declared -> calculate string length
            uiBufferLen = strlen((const char*)pBuffer_p);
        }
        else
        {
            // use declared buffer length
            uiBufferLen = uiBufferLen_p;
        }

        // limit length to local buffer size
        if (uiBufferLen > MQTT_MAX_PACKET_SIZE)
        {
            uiBufferLen = MQTT_MAX_PACKET_SIZE;
        }

        memcpy(acBuffer, pBuffer_p, uiBufferLen);
        acBuffer[uiBufferLen] = '\0';
        strBuffer = String(acBuffer);
    }
    else
    {
        strBuffer = "<invalid>";
    }

    return (strBuffer);

}





//=========================================================================//
//                                                                         //
//          S M A R T B O A R D   A P P   F U N C T I O N S                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Process received MQTT Data Messages (for subscribed MQTT Topics)
//---------------------------------------------------------------------------

void  AppProcessMqttDataMessage (String strMsgTopic_p, String strMsgPayload_p)
{

String  strMsgTopic;
String  strMsgPayload;
int     iValue;
int     iLedNum;
bool    fLedState;


    TRACE2("+ 'AppProcessMqttDataMessage()': strMsgTopic_p=%s, strMsgPayload_p=%s\n", strMsgTopic_p.c_str(), strMsgPayload_p.c_str());

    strMsgTopic = strMsgTopic_p;
    strMsgTopic.toLowerCase();

    strMsgPayload = strMsgPayload_p;
    strMsgPayload.toLowerCase();

    if (strMsgTopic.indexOf("settings") > 0)
    {
        TRACE0("   Recognized Messages Typ: 'Settings'\n");

        // Message scheme: 'SmBrd/<%>/Settings/Heartbeat'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[0])
        {
            iValue = strMsgPayload.substring(0,1).toInt();
            fStatusLedHeartbeat_g = (iValue != 0) ? true : false;
            TRACE1("   Set 'Heartbeat' to: %s\n", String(fStatusLedHeartbeat_g));
            if ( !fStatusLedHeartbeat_g )
            {
                digitalWrite(PIN_STATUS_LED, LOW);
            }
        }

        // Message scheme: 'SmBrd/<DevID>/Settings/LedBarIndicator'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[1])
        {
            iValue = strMsgPayload.substring(0,1).toInt();
            switch (iValue)
            {
                case 0:     LedBarIndicator_g = kLedBarNone;                break;
                case 1:     LedBarIndicator_g = kLedBarDht22Temperature;    break;
                case 2:     LedBarIndicator_g = kLedBarDht22Humidity;       break;
                case 3:     LedBarIndicator_g = kLedBarMhz19Co2Level;       break;
                default:    LedBarIndicator_g = kLedBarNone;                break;
            }
            TRACE1("   Set 'LedBarIndicator' to: %d\n", LedBarIndicator_g);
            AppPresentLedBar(0);                                    // clear all LEDs in LED Bar
        }

        // Message scheme: 'SmBrd/<DevID>/Settings/PrintSensorVal'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[2])
        {
            iValue = strMsgPayload.substring(0,1).toInt();
            fPrintSensorValues_g = (iValue != 0) ? true : false;
            TRACE1("   Set 'PrintSensorVal' to: %s\n", String(fPrintSensorValues_g));
        }

        // Message scheme: 'SmBrd/<DevID>/Settings/PrintMqttDataProc'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[3])
        {
            iValue = strMsgPayload.substring(0,1).toInt();
            fPrintMqttDataProc_g = (iValue != 0) ? true : false;
            TRACE1("   Set 'PrintMqttDataProc' to: %s\n", String(fPrintMqttDataProc_g));
        }
    }
    else if (strMsgTopic.indexOf("outdata") > 0)
    {
        TRACE0("   Recognized Messages Typ: 'OutData'\n");

        // Message scheme: 'SmBrd/<DevID>/OutData/LedBar'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[4])
        {
            LedBarIndicator_g = kLedBarNone;                        // prevent overwriting of LED Bar by other sources
            iValue = strMsgPayload.substring(0,1).toInt();
            TRACE1("   Set 'LedBar (normal)' to: %d\n", iValue);
            AppPresentLedBar(iValue, false);
        }

        // Message scheme: 'SmBrd/<DevID>/OutData/LedBarInv'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[5])
        {
            LedBarIndicator_g = kLedBarNone;                        // prevent overwriting of LED Bar by other sources
            iValue = strMsgPayload.substring(0,1).toInt();
            TRACE1("   Set 'LedBar (inverted)' to: %d\n", iValue);
            AppPresentLedBar(iValue, true);
        }

        // Message scheme: 'SmBrd/<DevID>/OutData/Led'
        if (strMsgTopic_p == astrMqttSubscribeTopicList_g[6])
        {
            LedBarIndicator_g = kLedBarNone;                        // prevent overwriting of LED Bar by other sources
            iLedNum = strMsgPayload.substring(0,1).toInt();
            iValue = strMsgPayload.substring(2,3).toInt();
            fLedState = (iValue != 0) ? HIGH : LOW;
            TRACE2("   Set 'Led': %d to: %d\n", iLedNum, fLedState);
            AppSetLed(iLedNum, fLedState);
        }
    }
    else
    {
        TRACE0("   WARNING: Unknown Messages Typ\n");
    }


    TRACE0("- 'AppProcessMqttDataMessage()'\n");

    return;

}




//---------------------------------------------------------------------------
//  Process LED Bar Indicator depending on its Data Source
//---------------------------------------------------------------------------

void  AppProcessLedBarIndicator()
{

    switch (LedBarIndicator_g)
    {
        case kLedBarNone:
        {
            break;
        }

        case kLedBarDht22Temperature:
        {
            AppSetDht22TemperatureLedBarIndicator();
            break;
        }

        case kLedBarDht22Humidity:
        {
            AppSetDht22HumidityLedBarIndicator();
            break;
        }

        case kLedBarMhz19Co2Level:
        {
            AppSetMhz19Co2LedBarIndicator();
            break;
        }

        default:
        {
            AppPresentLedBar(0);
            break;
        }
    }

    return;

}



//---------------------------------------------------------------------------
//  Set LED Bar Indicator for DHT22 Temperature
//---------------------------------------------------------------------------

void  AppSetDht22TemperatureLedBarIndicator()
{

int iBarValue;


    iBarValue = 0;

    //------------------------------------------
    if (flDhtTemperature_g < 0)
    {
        iBarValue = 0;
    }
    //------------------------------------------
    else if (flDhtTemperature_g <= 15)              // -+-
    {                                               //  |
        iBarValue = 1;                              //  |
    }                                               //  |
    else if (flDhtTemperature_g <= 18)              //  |
    {                                               //  |
        iBarValue = 2;                              //  | Green
    }                                               //  |
    else if (flDhtTemperature_g <= 20)              //  |
    {                                               //  |
        iBarValue = 3;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (flDhtTemperature_g <= 22)              // -+-
    {                                               //  |
        iBarValue = 4;                              //  |
    }                                               //  |
    else if (flDhtTemperature_g <= 24)              //  |
    {                                               //  |
        iBarValue = 5;                              //  | Yellow
    }                                               //  |
    else if (flDhtTemperature_g <= 26)              //  |
    {                                               //  |
        iBarValue = 6;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (flDhtTemperature_g <= 30)              // -+-
    {                                               //  |
        iBarValue = 7;                              //  |
    }                                               //  |
    else if (flDhtTemperature_g <= 35)              //  |
    {                                               //  |
        iBarValue = 8;                              //  | Red
    }                                               //  |
    else                                            //  |
    {                                               //  |
        iBarValue = 9;                              //  |
    }                                               // -+-


    AppPresentLedBar(iBarValue);


    return;

}



//---------------------------------------------------------------------------
//  Set LED Bar Indicator for DHT22 Humidity
//---------------------------------------------------------------------------

void  AppSetDht22HumidityLedBarIndicator()
{

int iBarValue;


    iBarValue = 0;

    //------------------------------------------
    if (flDhtHumidity_g < 5)
    {
        iBarValue = 0;
    }
    //------------------------------------------
    else if (flDhtHumidity_g <= 20)                 // -+-
    {                                               //  |
        iBarValue = 1;                              //  |
    }                                               //  |
    else if (flDhtHumidity_g <= 30)                 //  |
    {                                               //  |
        iBarValue = 2;                              //  | Green
    }                                               //  |
    else if (flDhtHumidity_g <= 40)                 //  |
    {                                               //  |
        iBarValue = 3;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (flDhtHumidity_g <= 50)                 // -+-
    {                                               //  |
        iBarValue = 4;                              //  |
    }                                               //  |
    else if (flDhtHumidity_g <= 60)                 //  |
    {                                               //  |
        iBarValue = 5;                              //  | Yellow
    }                                               //  |
    else if (flDhtHumidity_g <= 70)                 //  |
    {                                               //  |
        iBarValue = 6;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (flDhtHumidity_g <= 80)                 // -+-
    {                                               //  |
        iBarValue = 7;                              //  |
    }                                               //  |
    else if (flDhtHumidity_g <= 90)                 //  |
    {                                               //  |
        iBarValue = 8;                              //  | Red
    }                                               //  |
    else                                            //  |
    {                                               //  |
        iBarValue = 9;                              //  |
    }                                               // -+-


    AppPresentLedBar(iBarValue);


    return;

}



//---------------------------------------------------------------------------
//  Set LED Bar Indicator for MH-Z19 CO2 Level
//---------------------------------------------------------------------------

void  AppSetMhz19Co2LedBarIndicator()
{

int iBarValue;


    iBarValue = 0;

    //------------------------------------------
    if (iMhz19Co2Value_g < 10)
    {
        iBarValue = 0;
    }
    //------------------------------------------
    else if (iMhz19Co2Value_g <= 500)               // -+-
    {                                               //  |
        iBarValue = 1;                              //  |
    }                                               //  |
    else if (iMhz19Co2Value_g <= 750)               //  |
    {                                               //  |
        iBarValue = 2;                              //  | Green
    }                                               //  |
    else if (iMhz19Co2Value_g <= 1000)              //  |
    {                                               //  |
        iBarValue = 3;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (iMhz19Co2Value_g <= 1333)              // -+-
    {                                               //  |
        iBarValue = 4;                              //  |
    }                                               //  |
    else if (iMhz19Co2Value_g <= 1666)              //  |
    {                                               //  |
        iBarValue = 5;                              //  | Yellow
    }                                               //  |
    else if (iMhz19Co2Value_g <= 2000)              //  |
    {                                               //  |
        iBarValue = 6;                              //  |
    }                                               // -+-
    //------------------------------------------
    else if (iMhz19Co2Value_g <= 3000)              // -+-
    {                                               //  |
        iBarValue = 7;                              //  |
    }                                               //  |
    else if (iMhz19Co2Value_g <= 4000)              //  |
    {                                               //  |
        iBarValue = 8;                              //  | Red
    }                                               //  |
    else                                            //  |
    {                                               //  |
        iBarValue = 9;                              //  |
    }                                               // -+-


    AppPresentLedBar(iBarValue);


    return;

}



//---------------------------------------------------------------------------
//  Get App Runtime Configuration
//---------------------------------------------------------------------------

String  AppGetRuntimeConfig()
{

String  strRuntimeConfig;


    strRuntimeConfig  = "";
    strRuntimeConfig += CFG_ENABLE_NETWORK_SCAN ? "N" : "";     // (N)etworkScan
    strRuntimeConfig += CFG_ENABLE_DI_DO        ? "I" : "";     // (I)/O
    strRuntimeConfig += CFG_ENABLE_DHT_SENSOR   ? "T" : "";     // (T)emperature+Humidity
    strRuntimeConfig += CFG_ENABLE_MHZ_SENSOR   ? "C" : "";     // (C)O2
    strRuntimeConfig += CFG_ENABLE_STATUS_LED   ? "L" : "";     // Status(L)ed

    return (strRuntimeConfig);

}





//=========================================================================//
//                                                                         //
//          S M A R T B O A R D   H A R D W A R E   F U N C T I O N S      //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Process Inputs
//---------------------------------------------------------------------------

void  AppProcessInputs()
{

int  iStateKey0;
int  iStateKey1;


    // process KEY0
    iStateKey0 = (digitalRead(PIN_KEY0) == LOW) ? 1 : 0;                // Keys are inverted (1=off, 0=on)
    if (iLastStateKey0_g != iStateKey0)
    {
        if (iStateKey0 == 1)
        {
            if ( fPrintMqttDataProc_g )
            {
                Serial.println("State changed: KEY0=1");
            }
            MqttPublishData(astrMqttPublishTopicList_g[0].c_str(), "1", true, fPrintMqttDataProc_g);
        }
        else
        {
            if ( fPrintMqttDataProc_g )
            {
                Serial.println("State changed: KEY0=0");
            }
            MqttPublishData(astrMqttPublishTopicList_g[0].c_str(), "0", true, fPrintMqttDataProc_g);
        }

        iLastStateKey0_g = iStateKey0;
    }

    // process KEY1
    iStateKey1 = (digitalRead(PIN_KEY1) == LOW) ? 1 : 0;                // Keys are inverted (1=off, 0=on)
    if (iLastStateKey1_g != iStateKey1)
    {
        if (iStateKey1 == 1)
        {
            if ( fPrintMqttDataProc_g )
            {
                Serial.println("State changed: KEY1=1");
            }
            MqttPublishData(astrMqttPublishTopicList_g[1].c_str(), "1", true, fPrintMqttDataProc_g);
        }
        else
        {
            if ( fPrintMqttDataProc_g )
            {
                Serial.println("State changed: KEY1=0");
            }
            MqttPublishData(astrMqttPublishTopicList_g[1].c_str(), "0", true, fPrintMqttDataProc_g);
        }

        iLastStateKey1_g = iStateKey1;
    }

    return;

}



//---------------------------------------------------------------------------
//  Process DHT Sensor (Temperature/Humidity)
//---------------------------------------------------------------------------

void  AppProcessDhtSensor (uint32_t ui32DhtReadInterval_p, bool fPrintSensorValues_p, bool fPrintMqttDataProc_p)
{

char      szSensorValues[64];
uint32_t  ui32CurrTick;
float     flTemperature;
float     flHumidity;
String    strTemperature;
String    strHumidity;


    ui32CurrTick = millis();
    if ((ui32CurrTick - ui32LastTickDhtRead_g) >= ui32DhtReadInterval_p)
    {
        flTemperature = DhtSensor_g.readTemperature(false);                 // false = Temp in Celsius degrees, true = Temp in Fahrenheit degrees
        flHumidity    = DhtSensor_g.readHumidity();

        // check if values read from DHT22 sensor are valid
        if ( !isnan(flTemperature) && !isnan(flHumidity) )
        {
            // chache DHT22 sensor values for displaying in the LED Bar indicator
            flDhtTemperature_g = flTemperature;
            flDhtHumidity_g    = flHumidity;

            strTemperature = String(flTemperature, 1);                      // convert float to String with one decimal place
            strHumidity = String(flHumidity, 1);                            // convert float to String with one decimal place

            // print DHT22 sensor values in Serial Console (Serial Monitor)
            if ( fPrintSensorValues_p )
            {
                snprintf(szSensorValues, sizeof(szSensorValues), "DHT22: Temperature=%s, Humidity=%s", strTemperature.c_str(), strHumidity.c_str());
                Serial.println(szSensorValues);
            }

            // publish DHT22 sensor values
            MqttPublishData(astrMqttPublishTopicList_g[2].c_str(), strTemperature.c_str(), true, fPrintMqttDataProc_p);
            MqttPublishData(astrMqttPublishTopicList_g[3].c_str(), strHumidity.c_str(), true, fPrintMqttDataProc_p);
        }
        else
        {
            // reading sensor failed
            Serial.println("ERROR: Failed to read from DHT sensor!");
        }

        ui32LastTickDhtRead_g = ui32CurrTick;
    }

    return;

}



//---------------------------------------------------------------------------
//  Process MH-Z19 Sensor (CO2Value/SensorTemperature)
//---------------------------------------------------------------------------

void  AppProcessMhz19Sensor (uint32_t ui32Mhz19ReadInterval_p, bool fPrintSensorValues_p, bool fPrintMqttDataProc_p)
{

char      szSensorValues[64];
uint32_t  ui32CurrTick;
int       iMhz19Co2Value;
int       iMhz19Co2SensTemp;
String    strCo2Value;
String    strCo2SensTemp;


    ui32CurrTick = millis();
    if ((ui32CurrTick - ui32LastTickMhz19Read_g) >= ui32Mhz19ReadInterval_p)
    {
        iMhz19Co2Value    = Mhz19Sensor_g.getCO2();                     // MH-Z19: Request CO2 (as ppm)
        iMhz19Co2SensTemp = Mhz19Sensor_g.getTemperature();             // MH-Z19: Request Sensor Temperature (as Celsius)

        // chache MH-Z19 sensor value for processing in HTML document and for displaying in the LED Bar indicator
        iMhz19Co2Value_g    = iMhz19Co2Value;
        iMhz19Co2SensTemp_g = iMhz19Co2SensTemp;

        strCo2Value = String(iMhz19Co2Value);                           // convert int to String
        strCo2SensTemp = String(iMhz19Co2SensTemp);                     // convert int to String

        // print MH-Z19 sensor values in Serial Console (Serial Monitor)
        if ( fPrintSensorValues_p )
        {
            snprintf(szSensorValues, sizeof(szSensorValues), "MH-Z19: CO2(ppm)=%s, SensTemp=%s", strCo2Value.c_str(), strCo2SensTemp.c_str());
            Serial.println(szSensorValues);
        }

        // publish MH-Z19 sensor values
        MqttPublishData(astrMqttPublishTopicList_g[4].c_str(), strCo2Value.c_str(), true, fPrintMqttDataProc_p);
        MqttPublishData(astrMqttPublishTopicList_g[5].c_str(), strCo2SensTemp.c_str(), true, fPrintMqttDataProc_p);

        ui32LastTickMhz19Read_g = ui32CurrTick;
    }

    return;

}



//---------------------------------------------------------------------------
//  Calibrate MH-Z19 Sensor Manually
//---------------------------------------------------------------------------

bool  AppMhz19CalibrateManually()
{

int   iCountDownCycles;
bool  fCalibrateSensor;


    // turn off auto calibration
    Mhz19Sensor_g.autoCalibration(false);

    Serial.println();
    Serial.println();
    Serial.println("************************************************");
    Serial.println("---- Manually MH-Z19 Sensor Calibration ----");
    Serial.print("Countdown: ");

    // run calibration start countdown
    // (KEY0=pressed -> continue countdown, KEY0=released -> abort countdown)
    iCountDownCycles = 9;                                               // LED Bar Length = 9 LEDs
    fCalibrateSensor = true;
    do
    {
        Serial.print(iCountDownCycles); Serial.print(" ");
        if ( digitalRead(PIN_KEY0) )                                    // Keys are inverted (1=off, 0=on)
        {
            fCalibrateSensor = false;
            break;
        }
        AppPresentLedBar(iCountDownCycles);
        delay(500);

        if ( digitalRead(PIN_KEY0) )                                    // Keys are inverted (1=off, 0=on)
        {
            fCalibrateSensor = false;
            break;
        }
        AppPresentLedBar(0);
        delay(500);
    }
    while (iCountDownCycles-- > 0);

    // run calibration or abort?
    if ( fCalibrateSensor )
    {
        // calibrate sensor
        Serial.println();
        Serial.println("MH-Z19 Sensor Calibration...");
        Mhz19Sensor_g.calibrate();
        Serial.println("  -> done.");

        // signal calibration finished
        iCountDownCycles = 3;
        do
        {
            AppPresentLedBar(9);
            delay(125);
            AppPresentLedBar(0);
            delay(125);
        }
        while (iCountDownCycles-- > 1);
    }
    else
    {
        Serial.println("ABORT");
    }

    Serial.println("************************************************");
    Serial.println();
    Serial.println();

    return (fCalibrateSensor);

}



//---------------------------------------------------------------------------
//  Calibrate MH-Z19 Sensor Unattended
//---------------------------------------------------------------------------

bool  AppMhz19CalibrateUnattended()
{

// According to DataSheet the MH-Z19 Sensor must be in stable 400ppm ambient
// environment for more than 20 minutes.
uint32_t  ui32LeadTime = (25 * 60 * 1000);                       // 25 Minutes in [ms]

char      szTextBuff[64];
uint32_t  ui32StartTime;
int32_t   i32RemainTime;
int       iCountDownCycles;
int       iLedBarValue;


    // turn off auto calibration
    Mhz19Sensor_g.autoCalibration(false);

    Serial.println();
    Serial.println();
    Serial.println("************************************************");
    Serial.println("---- Unattended MH-Z19 Sensor Calibration ----");
    Serial.println("Countdown:");

    // run countdown to stabilize sensor in a 400ppm ambient environment for more than 20 minutes
    ui32StartTime = millis();
    do
    {
        i32RemainTime = (int) (ui32LeadTime - (millis() - ui32StartTime));

        // divide the remaining time over the 9 LEDs of LED Bar
        iLedBarValue = (int)((i32RemainTime / (ui32LeadTime / 9)) + 1);
        if (iLedBarValue > 9)
        {
            iLedBarValue = 9;
        }

        snprintf(szTextBuff, sizeof(szTextBuff), "  Remaining Time: %d [sec] -> LedBarValue=%d", i32RemainTime/1000, iLedBarValue);
        Serial.println(szTextBuff);

        AppPresentLedBar(iLedBarValue);
        delay(500);
        AppPresentLedBar(0);
        delay(500);
    }
    while (i32RemainTime > 0);

    // calibrate sensor
    Serial.println();
    Serial.println("MH-Z19 Sensor Calibration...");
    Mhz19Sensor_g.calibrate();
    Serial.println("  -> done.");

    // signal calibration finished
    iCountDownCycles = 3;
    do
    {
        AppPresentLedBar(9);
        delay(125);
        AppPresentLedBar(0);
        delay(125);
    }
    while (iCountDownCycles-- > 1);

    Serial.println("************************************************");
    Serial.println();
    Serial.println();

    return (true);

}



//---------------------------------------------------------------------------
//  Present LED Bar (0 <= iBarValue_p <= 9)
//---------------------------------------------------------------------------

void  AppPresentLedBar (int iBarValue_p)
{
    AppPresentLedBar(iBarValue_p, false);
}
//---------------------------------------------------------------------------
void  AppPresentLedBar (int iBarValue_p, bool fInvertBar_p)
{

uint32_t  ui32BarBitMap;


    if (iBarValue_p < 0)
    {
        ui32BarBitMap = 0x0000;             // set LED0..LED8 = OFF
    }
    else if (iBarValue_p > 9)
    {
        ui32BarBitMap = 0x01FF;             // set LED0..LED8 = ON
    }
    else
    {
        ui32BarBitMap = 0x0000;
        while (iBarValue_p > 0)
        {
            ui32BarBitMap <<= 1;
            ui32BarBitMap  |= 1;
            iBarValue_p--;
        }
    }

    if ( fInvertBar_p )
    {
        ui32BarBitMap ^= 0x01FF;
    }

    digitalWrite(PIN_LED0, (ui32BarBitMap & 0b000000001) ? HIGH : LOW);
    digitalWrite(PIN_LED1, (ui32BarBitMap & 0b000000010) ? HIGH : LOW);
    digitalWrite(PIN_LED2, (ui32BarBitMap & 0b000000100) ? HIGH : LOW);
    digitalWrite(PIN_LED3, (ui32BarBitMap & 0b000001000) ? HIGH : LOW);
    digitalWrite(PIN_LED4, (ui32BarBitMap & 0b000010000) ? HIGH : LOW);
    digitalWrite(PIN_LED5, (ui32BarBitMap & 0b000100000) ? HIGH : LOW);
    digitalWrite(PIN_LED6, (ui32BarBitMap & 0b001000000) ? HIGH : LOW);
    digitalWrite(PIN_LED7, (ui32BarBitMap & 0b010000000) ? HIGH : LOW);
    digitalWrite(PIN_LED8, (ui32BarBitMap & 0b100000000) ? HIGH : LOW);

    return;

}



//---------------------------------------------------------------------------
//  Set Single LED
//---------------------------------------------------------------------------

void  AppSetLed (int iLedNum_p, bool fLedState_p)
{

    switch (iLedNum_p)
    {
        case 0:     digitalWrite(PIN_LED0, fLedState_p);    break;
        case 1:     digitalWrite(PIN_LED1, fLedState_p);    break;
        case 2:     digitalWrite(PIN_LED2, fLedState_p);    break;
        case 3:     digitalWrite(PIN_LED3, fLedState_p);    break;
        case 4:     digitalWrite(PIN_LED4, fLedState_p);    break;
        case 5:     digitalWrite(PIN_LED5, fLedState_p);    break;
        case 6:     digitalWrite(PIN_LED6, fLedState_p);    break;
        case 7:     digitalWrite(PIN_LED7, fLedState_p);    break;
        case 8:     digitalWrite(PIN_LED8, fLedState_p);    break;
        default:                                            break;
    }

    return;

}





//=========================================================================//
//                                                                         //
//          E S P 3 2   H A R D W A R E   T I M E R   F U N C T I O N S    //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  ESP32 Hardware Timer ISR
//---------------------------------------------------------------------------

void  IRAM_ATTR  OnTimerISR()
{

    bStatusLedState_g = !bStatusLedState_g;
    digitalWrite(PIN_STATUS_LED, bStatusLedState_g);

    return;

}



//---------------------------------------------------------------------------
//  ESP32 Hardware Timer Start
//---------------------------------------------------------------------------

void  Esp32TimerStart (uint32_t ui32TimerPeriod_p)
{

uint32_t  ui32TimerPeriod;


    // stop a timer that may still be running
    Esp32TimerStop();

    // use 1st timer of 4
    // 1 tick take 1/(80MHZ/80) = 1us -> set divider 80 and count up
    pfnOnTimerISR_g = timerBegin(0, 80, true);

    // attach OnTimerISR function to timer
    timerAttachInterrupt(pfnOnTimerISR_g, &OnTimerISR, true);

    // set alarm to call OnTimerISR function, repeat alarm automatically (third parameter)
    ui32TimerPeriod = (unsigned long)ui32TimerPeriod_p * 1000L;         // ms -> us
    timerAlarmWrite(pfnOnTimerISR_g, ui32TimerPeriod, true);

    // start periodically alarm
    timerAlarmEnable(pfnOnTimerISR_g);

    return;

}



//---------------------------------------------------------------------------
//  ESP32 Hardware Timer Stop
//---------------------------------------------------------------------------

void  Esp32TimerStop()
{

    if (pfnOnTimerISR_g != NULL)
    {
        // stop periodically alarm
        timerAlarmDisable(pfnOnTimerISR_g);

        // dettach OnTimerISR function from timer
        timerDetachInterrupt(pfnOnTimerISR_g);

        // stop timer
        timerEnd(pfnOnTimerISR_g);
    }

    bStatusLedState_g = LOW;
    digitalWrite(PIN_STATUS_LED, bStatusLedState_g);

    return;

}





//=========================================================================//
//                                                                         //
//          P R I V A T E   G E N E R I C   F U N C T I O N S              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Get DeviceID
//---------------------------------------------------------------------------

String  GetDeviceID (const char* pszDeviceID_p)
{

String  strDeviceID;


    TRACE1("+ GetDeviceID(pszDeviceID_p='%s')\n", (pszDeviceID_p ? pszDeviceID_p : "[NULL]"));
    if ( (pszDeviceID_p != NULL) && (strlen(pszDeviceID_p) > 0) )
    {
        TRACE0("  -> use calling paameter 'pszDeviceID_p'\n");
        strDeviceID = pszDeviceID_p;
    }
    else
    {
        // use ChipID (the ChipID is essentially its 6byte MAC address) as DeviceID
        TRACE0("  -> use ChipID\n");
        strDeviceID = GetChipID();
    }

    TRACE1("- GetDeviceID(strDeviceID='%s')\n", strDeviceID.c_str());
    return (strDeviceID);

}



//---------------------------------------------------------------------------
//  Get Unique Client Name
//---------------------------------------------------------------------------

String  GetUniqueClientName (const char* pszClientPrefix_p)
{

String  strChipID;
String  strClientName;


    // Create a unique client name, based on ChipID (the ChipID is essentially its 6byte MAC address)
    strChipID = GetChipID();
    strClientName  = pszClientPrefix_p;
    strClientName += strChipID;

    return (strClientName);

}



//---------------------------------------------------------------------------
//  Get ChipID as String
//---------------------------------------------------------------------------

String  GetChipID()
{

String  strChipID;


    strChipID = GetEsp32MacId(false);

    return (strChipID);

}



//---------------------------------------------------------------------------
//  Get ChipMAC as String
//---------------------------------------------------------------------------

String  GetChipMAC()
{

String  strChipMAC;


    strChipMAC = GetEsp32MacId(true);

    return (strChipMAC);

}



//---------------------------------------------------------------------------
//  Get GetEsp32MacId as String
//---------------------------------------------------------------------------

String  GetEsp32MacId (bool fUseMacFormat_p)
{

uint64_t  ui64MacID;
String    strMacID;
byte      bDigit;
char      acDigit[2];
int       iIdx;


    ui64MacID = ESP.getEfuseMac();
    strMacID = "";
    for (iIdx=0; iIdx<6; iIdx++)
    {
        bDigit = (byte) (ui64MacID >> (iIdx * 8));
        sprintf(acDigit, "%02X", bDigit);
        strMacID += String(acDigit);

        if (fUseMacFormat_p && (iIdx<5))
        {
            strMacID += ":";
        }
    }

    strMacID.toUpperCase();

    return (strMacID);

}





//=========================================================================//
//                                                                         //
//          D E B U G   F U N C T I O N S                                  //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  DEBUG: Dump Buffer
//---------------------------------------------------------------------------

#ifdef DEBUG_DUMP_BUFFER

void  DebugDumpBuffer (String strBuffer_p)
{

int            iBufferLen = strBuffer_p.length();
unsigned char  abDataBuff[iBufferLen];

    strBuffer_p.getBytes(abDataBuff, iBufferLen);
    DebugDumpBuffer(abDataBuff, strBuffer_p.length());

    return;

}

//---------------------------------------------------------------------------

void  DebugDumpBuffer (const void* pabDataBuff_p, unsigned int uiDataBuffLen_p)
{

#define COLUMNS_PER_LINE    16

const unsigned char*  pabBuffData;
unsigned int          uiBuffSize;
char                  szLineBuff[128];
unsigned char         bData;
int                   nRow;
int                   nCol;

    // get pointer to buffer and length of buffer
    pabBuffData = (const unsigned char*)pabDataBuff_p;
    uiBuffSize  = (unsigned int)uiDataBuffLen_p;


    // dump buffer contents
    for (nRow=0; ; nRow++)
    {
        sprintf(szLineBuff, "\n%04lX:   ", (unsigned long)(nRow*COLUMNS_PER_LINE));
        Serial.print(szLineBuff);

        for (nCol=0; nCol<COLUMNS_PER_LINE; nCol++)
        {
            if ((unsigned int)nCol < uiBuffSize)
            {
                sprintf(szLineBuff, "%02X ", (unsigned int)*(pabBuffData+nCol));
                Serial.print(szLineBuff);
            }
            else
            {
                Serial.print("   ");
            }
        }

        Serial.print(" ");

        for (nCol=0; nCol<COLUMNS_PER_LINE; nCol++)
        {
            bData = *pabBuffData++;
            if ((unsigned int)nCol < uiBuffSize)
            {
                if ((bData >= 0x20) && (bData < 0x7F))
                {
                    sprintf(szLineBuff, "%c", bData);
                    Serial.print(szLineBuff);
                }
                else
                {
                    Serial.print(".");
                }
            }
            else
            {
                Serial.print(" ");
            }
        }

        if (uiBuffSize > COLUMNS_PER_LINE)
        {
            uiBuffSize -= COLUMNS_PER_LINE;
        }
        else
        {
            break;
        }

        Serial.flush();     // give serial interface time to flush data
    }

    Serial.print("\n");

    return;

}

#endif




// EOF
