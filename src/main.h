#include <Arduino.h>

/* Passwords & Ports
 * wifi: ssid, password
 * ISY: hash, isy, isyport
 * MQTT mqtt_server, mqtt_serverport
 */
#include <../../../../../../../../../Projects/keys/sej/sej.h>

// WiFi stuff
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

// OTA stuff
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Camera stuff
#include "OV2640.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"

// TODO XXX
// Button stuff
/* #include <OneButton.h> */

// MQTT stuff
#include <PubSubClient.h>
#include <ArduinoJson.h>

// time stuff
#include <Timezone.h>

// file stuff
#include <FS.h>
// #include <LittleFS.h> // use for esp8266
#include <SPIFFS.h> // use for esp32
#define LittleFS SPIFFS //use for esp32
#define FORMAT_FS_IF_FAILED true // use for esp32

// Select which of the servers are active
// Select only one or the streaming will be very slow!
#define ENABLE_WEBSERVER
//#define ENABLE_RTSPSERVER

// Camera class
extern OV2640 cam;

// RTSP stuff
void initRTSP(void);
void stopRTSP(void);

// Web server stuff
void initWebStream(void);
void stopWebStream(void);
void handleWebServer(void);

// OTA stuff
void startOTA(void);
void stopOTA(void);
extern boolean otaStarted;
