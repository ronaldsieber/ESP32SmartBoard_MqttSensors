# ESP32SmartBoard_MqttSensors

This Arduino project enables the *ESP32SmartBoard* (see hardware project [ESP32SmartBoard_PCB](https://github.com/ronaldsieber/ESP32SmartBoard_PCB)) to communicate with an MQTT broker. The values of the temperature and humidity sensor (DHT22), the CO2 sensor (MH-Z19) and the other peripherals of the board are sent as MQTT messages. Incoming MQTT messages can be used to set outputs and to configure the board.

In addition to this Arduino project, the [ESP32SmartBoard_HttpSensors](https://github.com/ronaldsieber/ESP32SmartBoard_HttpSensors) project is an alternative firmware for the *ESP32SmartBoard* that realizes a functionality that is basically similar. It differs from the project described here in that it presents the values of the sensors with the help of an Embedded WebServer over HTTP pages. The two projects were designed for the following different use cases:
   
 - This project - **ESP32SmartBoard_MqttSensors**:
More complex setup, but supports evaluation and analysis of historical data
Part of an overall system made up of several *ESP32SmartBoards*, each board publishes its data via MQTT to a (shared) broker, who writes the data e.g. into a database (e.g. InfluxDB), from where it can be displayed and evaluated using graphical dashboards (e.g. Grafana)

 - Alternative - **ESP32SmartBoard_HttpSensors**:
Easy to use, clear display of current sensor values
Stand-alone solution with Embedded WebServer, presentation of the current sensor values via HTTP pages in real time, direct access to the *ESP32SmartBoard* from devices such as PCs, laptop, tablet or smartphone

## Project Overview

When the system starts, the *ESP32SmartBoard* connects to an MQTT broker. The settings required for this are described in the two sections "WLAN configuration" and "MQTT configuration" below. After logging in to the broker, the board cyclically sends ("published") the values ​​of the temperature and humidity sensor (DHT22) as well as the CO2 sensor (MH-Z19). The two buttons KEY0 and KEY1 are transmitted event-triggered. The board receives data from the broker by "subscribing" and can thus be controlled and configured and  at runtime. Details are described in the section "MQTT messages of the ESP32SmartBoard" below.

In a typical scenario, several *ESP32SmartBoards* are installed in different rooms of an apartment or in several offices or classrooms. All of these boards transmit their data via MQTT to a common broker, which provides all sensor values ​​in a central instance. This is then accessed by other clients who, for example, display the current temperature or CO2 level in different rooms in the form of dashboards (e.g. Node-RED) or collect the sensor values ​​in a database (e.g. InfluxDB). The measurement data saved as time series can then afterwards be graphically displayed and evaluated as time curves of the sensor values ​​in charts (e.g. Grafana).

![\[Project Overview\]](Documentation/ESP32SmartBoard_MqttSensors.png)

The *ESP32SmartBoard_MqttSensors* project only contains the MQTT-based firmware for the *ESP32SmartBoard* from the setup described. Broker, database, dashboards etc. are not part of this project.

In addition to this Arduino project, [ESP32SmartBoard_NodeRED](https://github.com/ronaldsieber/ESP32SmartBoard_NodeRED) implements a Node-RED based dashboard to display the sensor data and for the runtime configuration of the board.

The open source tool 'MQTT Explorer' (https://mqtt-explorer.com/) is very suitable for commissioning and diagnosis.

## WLAN Configuration Section

The *ESP32SmartBoard* works in client mode (CM, station) and logs into an existing WLAN. The runtime configuration takes place via the following configuration section:

    // WIFI Station/Client Configuration
    const char*  WIFI_STA_SSID            = "YourNetworkName";
    const char*  WIFI_STA_PASSWORD        = "YourNetworkPassword";
    IPAddress    WIFI_STA_LOCAL_IP(0,0,0,0);          // DHCP: IP(0,0,0,0)

With MQTT, the clients connect to the broker. The client's IP address is irrelevant. It is therefore easiest to have the *ESP32SmartBoard* assigned an address via DHCP. This minimizes the configuration effort and allows one and the same firmware to be used for all boards.

If the *ESP32SmartBoard* is to use a dedicated IP address instead, this must be defined using the constant `WIFI_STA_LOCAL_IP`:

    IPAddress WIFI_STA_LOCAL_IP (192,168,30,100); // DHCP: IP (0,0,0,0)

## MQTT Configuration Section

The MQTT specific settings are defined in the following configuration section:

    // MQTT Configuration
    const char*  MQTT_SERVER            = "192.168.69.1";
    const int    MQTT_PORT              = 1883;
    const char*  MQTT_USER              = "SmBrd_IoT";
    const char*  MQTT_PASSWORD          = "xxx";
    const char*  MQTT_CLIENT_PREFIX     = "SmBrd_";     // will be extended by ChipID
    const char*  MQTT_DEVICE_ID         = NULL;         // used for Device specific Topics
    const char*  MQTT_SUPERVISOR_TOPIC  = "SmBrd_Supervisor";

The two constants `MQTT_SERVER` and `MQTT_PORT` determine the destination address of the MQTT broker.

The constants `MQTT_USER` and `MQTT_PASSWORD` are used to register the *ESP32SmartBoard* with the broker. If the broker works with dedicated user accounts, the data must be adapted accordingly. Each client must identify itself with a unique client ID. `MQTT_CLIENT_PREFIX` forms the basis for this, which is expanded by the individual Board ID derived from the MAC address.

With the help of `MQTT_DEVICE_ID`, the generic topic templates are individualized for the respective board (for details see section "Individualization of MQTT topics at runtime" below). If `MQTT_DEVICE_ID` is not equal to NULL, the string defined here is used, otherwise the individual Board ID derived from the MAC address.

## Application Configuration Section

At the beginning of the sketch there is the following configuration section:

    const int  CFG_ENABLE_NETWORK_SCAN  = 1;
    const int  CFG_ENABLE_DI_DO         = 1;
    const int  CFG_ENABLE_DHT_SENSOR    = 1;
    const int  CFG_ENABLE_MHZ_SENSOR    = 1;
    const int  CFG_ENABLE_STATUS_LED    = 1;

This enables the runtime execution of the associated code sections to be activated *(= 1)* or blocked *(= 0)*. This allows the sketch to be used for boards on which not all components are fitted, without the lack of components leading to runtime errors.

## Calibration of the CO2 sensor MH-Z19

**An incorrectly performed calibration can brick the sensor. It is therefore important to understand how calibration works.**

The sensor is designed for use in 24/7 continuous operation. It supports the modes AutoCalibration, calibration via Hardware Signal (triggered manually) and calibration via Software Command (also triggered manually). The *ESP32SmartBoard* uses the AutoCalibration and manual calibration modes via software commands. Regardless of the method used, the calibration sets the sensor-internal reference value of 400 ppm CO2. A concentration of 400 ppm is considered to be the normal CO2 value in the earth's atmosphere, i.e. a typical value for the outside air in rural areas.

**(1) AutoCalibration:**

With AutoCalibration, the sensor permanently monitors the measured CO2 values. The lowest value measured within 24 hours is interpreted as the reference value of 400 ppm CO2. This method is recommended by the sensor manufacturer for use of the sensor in normal living rooms or in offices that are regularly well ventilated. It is implicitly assumed that the air inside the roorm is completely exchanged during ventilation and thus the CO2 concentration in the room falls down to the normal value of the earth's atmosphere / outside air.

However, the sensor manufacturer explicitly states in the datasheet that the AutoCalibration method cannot be used for use in agricultural greenhouses, farms, refrigerators, etc. AutoCalibration should be disabled here.

In the *ESP32SmartBoard* Sketch the constant `DEFAULT_MHZ19_AUTO_CALIBRATION` is used to activate (true) or deactivate (false) the AutoCalibration method. The AutoCalibration method is set on system startup. If necessary, the AutoCalibration method can be inverted using a push button. The following steps are required for this:

1. Press KEY0 and keep it pressed until step 3
2. Press and release Reset on the ESP32DevKit
3. Hold down KEY0 for another 2 seconds
4. Release KEY0

**Note:** The AutoCalibration mode can lead to sudden changes in the CO2 value, especially in the first few days after powering on the sensor. After some time in 24/7 continuous operation, this effect decreases more and more.

![\[CO2 Sensor AutoCalibration Discontinuity\]](Documentation/CO2Sensor_Autocalibration_Discontinuity.png)

The spontaneous discontinuities caused by the AutoCalibration mode can be avoided by manually calibrating the sensor.

**(2) Manually Calibration:**

Before a manual calibration, the sensor must be operated for at least 20 minutes in a stable reference environment with 400 ppm CO2. This requirement can only be approximated in the amateur and hobby area without a defined calibration environment. For this purpose, the *ESP32SmartBoard* can be operated outdoors in a shady place or inside a room near an open window, also in the shadow. In this environment, the *ESP32SmartBoard* must work for at least 20 minutes before the calibration can be triggered.

* (2a) Direct calibration:

If the *ESP32SmartBoard* has been working in the 400 ppm reference environment for at least 20 minutes (outdoors or inside at the open window), the sensor can be calibrated directly. The following steps are required for this:

1. Press KEY0 and keep it pressed until step 4
2. Press and release Reset on the ESP32DevKit
3. The *ESP32SmartBoard* starts a countdown of 9 seconds, the LED Bar blinkes every second and shows the remaining seconds
4. Keep KEY0 pressed the entire time until the countdown has elapsed and the LED Bar acknowledges the completed calibration with 3x quick flashes
5. Release KEY0

If KEY0 is released during the countdown, the *ESP32SmartBoard* aborts the procedure without calibrating the sensor. 

![\[Sequence Schema for direct manually Calibration\]](Documentation/SequenceSchema_ManuallyCalibration_Direct.png)
* (2b) Unattended, delayed Calibration:

The calibration can also be carried out unsupervised and with a time delay, when the *ESP32SmartBoard* has been placed in the 400 ppm reference environment (outdoors or inside at the open window). The following steps are required for this:

1. Press KEY1 and hold it down until step 2
2. Press and release Reset on the ESP32DevKit
3. Release KEY1

The *ESP32SmartBoard* starts a countdown of 25 minutes. During this countdown the LED Bar blinkes every second and shows the remaining time (25 minutes / 9 LEDs = 2:46 minutes / LED). After the countdown has elapsed, the calibration process is triggered and acknowledged with 3x quick flashing of the LED Bar. The *ESP32SmartBoard* is then restarted with a software reset.

![\[Sequence Schema for unattended, delayed Calibration\]](Documentation/SequenceSchema_ManuallyCalibration_Unattended.png)
## Run-time Output in the Serial Terminal Window

At runtime, all relevant information is output in the serial terminal window (115200Bd). In particular during the system start (sketch function `setup()`), error messages are also displayed here, which may be due to a faulty software configuration. These messages should be observed in any case, especially during commissioning.

In the main loop of the program (sketch function `main()`) the values of the DHT22 (temperature and humidity) and the MH-Z19 (CO2 level and sensor temperature) are displayed cyclically. In addition, messages here again provide information about any problems with accessing the sensors.

By activating the line `#define DEBUG` at the beginning of the sketch, further, very detailed runtime messages are displayed. These are particularly helpful during program development or for troubleshooting. By commenting out the line `#define DEBUG`, the output is suppressed again.

## MQTT Client Basics

The *ESP32SmartBoard* Sketch uses the "Arduino Client for MQTT" library (https://github.com/knolleary/pubsubclient) for MQTT communication with the broker. This library encapsulates the entire protocol implementation and is therefore very easy and convenient for the user to use:

The runtime object is created directly via the constructor:

    PubSubClient  PubSubClient(WiFiClient);

In order to inform the *ESP32SmartBoard* of the broker to be used, the IP address and port of the server must be set:

    PubSubClient.setServer (pszMqttServer, iMqttPort);

A corresponding callback handler must be registered so that the client on the *ESP32SmartBoard* can receive messages from the broker via subscribe:

    PubSubClient.setCallback (pfnMqttSubCallback);

In order to connect the local client on the *ESP32SmartBoard* to the broker, at least one unique ClientID and the login data in the form of a username and password are required. The ClientID is derived from the MAC address of the board, which ensures that each board uses its unique ID. Username and password must be specified on the client side, but whether these are actually evaluated depends on the configuration of the broker. Many brokers are operated openly, so that the login data used can be selected freely. For brokers with active authentication, the user data assigned by the server operator must be used.

Optionally, the topic and payload of a "LastWill" message can be defined when establishing the connection. The broker first saves this message and then publishes it later when the connection to the client is lost. When establishing the connection, the client can thus determine the message with which the broker informs all other participants about the termination of the connection between client and broker:

    PubSubClient.connect(strClientId.c_str(),            // ClientID
                         pszMqttUser,                    // Username
                         pszMqttPassword,                // Password
                         pszSupervisorTopic,             // LastWillTopic
                         0,                              // LastWillQoS
                         1,                              // LastWillRetain
                         strPayloadMsgGotLost.c_str(),   // LastWillMessage
                         true);                          // CleanSession

To publish a message, its topic and payload must be specified. A message marked as "Retain" is temporarily stored by the broker. This means that it can still be sent to a client if the client subscribes to the relevant topic only after the message has been published. This means that other clients can be synchronized with the current status of their partners when they log in to the broker:

    PubSubClient.publish(pszSupervisorTopic,                       // Topic
                         strPayloadMsgConnect.c_str(),             // Payload
                         1);                                       // Retain

Since the library does not use its own scheduler, it depends on being regularly supplied with CPU time:

    PubSubClient.loop();

The current connection status to the broker can be queried at runtime:

    PubSubClient.state();

## Individualization of the MQTT Topics at Runtime

In a typical scenario, several *ESP32SmartBoards* connects to a common broker. In order to be able to assign the data to the individual boards, each board uses individualized topics. This customization only takes place at runtime, so that the same sketch source can be used for all boards.

The generic topic templates are defined in static string arrays:

    const char*  MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE[] =
    {
        "SmBrd/<%>/Settings/Heartbeat",            // "<%>" will be replaced by DevID
        "SmBrd/<%>/Settings/LedBarIndicator",      // "<%>" will be replaced by DevID
        "SmBrd/<%>/Settings/PrintSensorVal",       // "<%>" will be replaced by DevID
        "SmBrd/<%>/Settings/PrintMqttDataProc",    // "<%>" will be replaced by DevID
        "SmBrd/<%>/OutData/LedBar",                // "<%>" will be replaced by DevID
        "SmBrd/<%>/OutData/LedBarInv",             // "<%>" will be replaced by DevID
        "SmBrd/<%>/OutData/Led"                    // "<%>" will be replaced by DevID
    };
    
    const char*  MQTT_PUBLISH_TOPIC_LIST_TEMPLATE[] =
    {
        "SmBrd/<%>/InData/Key0",                   // "<%>" will be replaced by DevID
        "SmBrd/<%>/InData/Key1",                   // "<%>" will be replaced by DevID
        "SmBrd/<%>/InData/Temperature",            // "<%>" will be replaced by DevID
        "SmBrd/<%>/InData/Humidity",               // "<%>" will be replaced by DevID
        "SmBrd/<%>/InData/CO2",                    // "<%>" will be replaced by DevID
        "SmBrd/<%>/InData/SensTemp"                // "<%>" will be replaced by DevID
    };

The board-specific customization is implemented by the `MqttBuildTopicFromTemplate()` function. The individualized stings are assigned to the following two arrays:

    String  astrMqttSubscribeTopicList_g[ARRAY_SIZE(MQTT_SUBSCRIBE_TOPIC_LIST_TEMPLATE)];
    String  astrMqttPublishTopicList_g[ARRAY_SIZE(MQTT_PUBLISH_TOPIC_LIST_TEMPLATE)];

For example, the topic for publishing the temperature has the following generic format:

    "SmBrd/<DevID>/InData/Temperature"

For customization, the substring `<DevID>` is replaced on a board-specific basis. Either the constant `MQTT_DEVICE_ID` is used if it's not NULL. Otherwise, the individual board ID derived from the MAC address is used here, e.g .:

    "SmBrd/246F2822A4B8/InData/Temperature"

## MQTT Messages of the ESP32SmartBoard

The *ESP32SmartBoards* communicates with the broker via the MQTT messages described below. For both topics and payload only Strings are used as data type.

    Type:    Publish
    Topic:   SmBrd_Supervisor
    Payload: "CI=<ClientId>, IP=<LocalIP>, RC=[N][I][T][C][L], ST=[Connected|GotLost]"

This message is a diagnostic message. It is sent once as the first message after the connection has been successfully established to the broker. It contains the ClientID, the IP Address and the Runtime Configuration of the board. The same message is also used as "LastWill". The status "Connected" or "GotLost" indicates the establishment or termination of the connection.

The term used to describe the current runtime configuration ("RC=") contains a symbol for each code section activated at runtime:

    N :	CFG_ENABLE_NETWORK_SCAN == 1	// (N)etworkScan
    I :	CFG_ENABLE_DI_DO        == 1	// (I)/O 
    T :	CFG_ENABLE_DHT_SENSOR   == 1	// (T)emperature+Humidity
    C :	CFG_ENABLE_MHZ_SENSOR   == 1	// (C)O2 
    L :	CFG_ENABLE_STATUS_LED   == 1	// Status(L)ed 

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/Temperature
    Payload: <Temperature>

Temperature value of the DHT22 sensor (is transmitted cyclically with the period defined as `DHT_SENSOR_SAMPLE_PERIOD`)

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/Humidity
    Payload: <Temperature>

Humidity value of the DHT22 sensor (is transmitted cyclically with the period defined as `DHT_SENSOR_SAMPLE_PERIOD`)

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/CO2
    Payload: <CO2Val>

CO2 level of the MH-Z19 sensor (is transmitted cyclically with the period defined as `MHZ19_SENSOR_SAMPLE_PERIOD`)

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/SensTemp
    Payload: <SensTemp>

Sensor temperature of the MH-Z19 sensor (is transmitted cyclically with the period defined as `MHZ19_SENSOR_SAMPLE_PERIOD`)

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/Key0
    Payload: Payload=['0'|'1']

Current state of the push button KEY0 (event-triggered, is only transferred if there is a change)

---

    Typ:     Publish
    Topic:   SmBrd/<DevID>/InData/Key1
    Payload: Payload=['0'|'1']

Current state of the push button KEY1 (event-triggered, is only transferred if there is a change)

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/Settings/Heartbeat
    Payload: Payload=['0'|'1']

Sets the Heartbeat Mode (flashing of the blue LED on the ESP32DevKit):
0 = off
1 = on

The constant `DEFAULT_STATUS_LED_HEARTBEAT` defines the default value for this.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/Settings/LedBarIndicator
    Payload: Payload=['0'|'1'|'2'|'3']

Selects the data source for the LED Bar Indicator:
0 = kLedBarNone
1 = kLedBarDht22Temperature
2 = kLedBarDht22Humidity
3 = kLedBarMhz19Co2Level

The constant `DEFAULT_LED_BAR_INDICATOR` defines the default value for this.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/Settings/PrintSensorVal
    Payload: Payload=['0'|'1']

Specifies whether the values read from the sensors are also displayed in the serial terminal window (115200Bd)
0 = off
1 = on

The constant `DEFAULT_PRINT_SENSOR_VALUES` defines the default value for this.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/Settings/PrintMqttDataProc
    Payload: Payload=['0'|'1']

Specifies whether the published and received MQTT messages are also displayed in the serial terminal window (115200Bd)
0 = off
1 = on

The constant `DEFAULT_PRINT_MQTT_DATA_PROC` defines the default value for this.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/OutData/LedBar
    Payload: Payload=['0'-'9']

Displays the specified value on the LED Bar Indicator. The data source for the LED Bar Indicator is set to 0 = kLedBarNone.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/OutData/LedBarInv
    Payload: Payload=['0'-'9']

Displays the specified value inversely on the LED Bar Indicator. The data source for the LED Bar Indicator is set to 0 = kLedBarNone.

---

    Typ:     Subscribe
    Topic:   SmBrd/<DevID>/OutData/Led
    Payload: Payload=['0'-'9']=['0'|'1']

Turns a single LED on the LED Bar Indicator on or off. The data source for the LED Bar Indicator is set to 0 = kLedBarNone.
0 = off
1 = on

## Used Third Party Components 

1. **MQTT Library**
For the MQTT communication the library "Arduino Client for MQTT" is used: https://github.com/knolleary/pubsubclient. The installation is done with the Library Manager of the Arduino IDE.

2. **MH-Z19 CO2 Sensor**
The following driver libraries are used for the MH-Z19 CO2 sensor:
https://www.arduino.cc/reference/en/libraries/mh-z19/ and https://www.arduino.cc/en/Reference/SoftwareSerial. The installation is done with the Library Manager of the Arduino IDE.

3. **DHT Sensor**
The driver library from Adafruit is used for the DHT sensor (temperature, humidity). The installation is done with the Library Manager of the Arduino IDE.


