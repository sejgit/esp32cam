/*
 *  ESP32cam IP cam and MQTT control w/OTA
 *
 *  Written for an ESP32cam
 *  --fqbn esp32:esp32:esp32 used to allow ota to work
 *  --fqbn esp32:esp32:esp32 not used as ota will not work
 *
 *  init   SeJ 10 03 2020 init merge awningMQTT and esp32-cam-video
 *  update SeJ
 */

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <FS.h>
// #include <LittleFS.h> // use for esp8266
#include <SPIFFS.h> // use for esp32
#define LittleFS SPIFFS //use for esp32
#define FORMAT_FS_IF_FAILED true // use for esp32

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

/*
 * Significant portions of the below used in this project:
 *
 *  Rui Santos
 *  Complete project details at
 * https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
 *
 * IMPORTANT!!!
 *   - Select Board "AI Thinker ESP32-CAM"
 *   - GPIO 0 must be connected to GND to upload a sketch
 *   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button
 *     to put your board in flashing mode
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files.
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 */

/* Passwords & Ports
 * wifi: ssid, password
 * ISY: hash, isy, isyport
 * MQTT mqtt_server, mqtt_serverport
 */
#include <../../../../../../../../../Projects/keys/sej/sej.h>


/*
 * Time
 */
// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";

const int timeZone = 0;     // use UTC due to Timezone corr
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     // Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr;        // pointer to the time change rule, use to get TZ abbrev

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
const char* defaultTime = "00:00:00";
char stringTime[10];
int oldmin = 99;
time_t local;


/*
 *  MQTT
 */
int status = WL_IDLE_STATUS;
boolean mqttConnected = false;

const char* topic = "sej"; // main topic
String clientId = "esp32cam"; // client ID for this unit
char buffer[256];

// MQTT topics
char topic_control_motion [30];
String topic_control_motion_s = "sej/" + clientId + "/control/motion";

const char* message_control_motion[] = {"OFF", "ON"};

char topic_status_motion [30];
String topic_status_motion_s = "sej/" + clientId + "/status/motion"; // bounce back
const char* message_status_motion[] = {"OFF", "ON"};
boolean motion = false; // check for motion boolean

char topic_control_reset[30];
String topic_control_reset_s= "sej/" + clientId + "/control/reset"; // reset position
const char* message_control_reset[] = {"--", "RESET", "Resetting"};

char topic_status_hb[30];
String topic_status_hb_s = "sej/" + clientId +"/status/hb"; // hb topic
const char* message_status_hb[] = {"OFF", "ON"};

const char* willTopic = topic; // will topic
byte willQoS = 0;
boolean willRetain = false;
const char* willMessage = ("lost connection " + clientId).c_str();
boolean heartbeat = false; // heartbeat to mqtt

WiFiClient espClient;
PubSubClient mqttClient(espClient);
long mqttLastMsg = 0;
int mqttValue = 0;


// cfg updates
boolean cfgChangeFlag = false;

/*
 * timers
 */
unsigned long currentMillis = 0;
unsigned long hbMillis = 0;
const long hbInterval = 60000; // how often to send hb
unsigned long ledMillis = 0;
const long ledInterval = 3000; // blink led h
bool ledState = false;
unsigned long OTAavailableMillis = 0;
const long OTAavailable = 5 * 60 * 1000; //OTA only available this long after boot
boolean newpic = true;
unsigned long newpicMillis = 0;
const long newpicInterval = 80; // frame interval

/*
 * I/O
 */
#define LED_BUILTIN 33


/*
 * ESP32cam
 */
#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27

  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23

  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23

  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27

  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;


/*
 * Declare Subroutines
 */

/*
 * exp stream handler
 */
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
      while(!newpic) {
          delay(10);
      }
      newpic = false;
      fb = esp_camera_fb_get();
      if (!fb) {
          Serial.println("Camera capture failed");
          res = ESP_FAIL;
      } else {
          if(fb->width > 400){
              if(fb->format != PIXFORMAT_JPEG){
                  bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                  esp_camera_fb_return(fb);
                  fb = NULL;
                  if(!jpeg_converted){
                      Serial.println("JPEG compression failed");
                      res = ESP_FAIL;
                  }
              } else {
                  _jpg_buf_len = fb->len;
                  _jpg_buf = fb->buf;
              }
          }
      }
      if(res == ESP_OK){
          size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
          res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if(res == ESP_OK){
          res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if(res == ESP_OK){
          res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      if(fb){
          esp_camera_fb_return(fb);
          fb = NULL;
          _jpg_buf = NULL;
      } else if(_jpg_buf){
          free(_jpg_buf);
          _jpg_buf = NULL;
      }
      if(res != ESP_OK){
          break;
      }
      //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}



/*
 * Start Camera Server
 */
void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}


/*
 * Establish Wi-Fi connection
 */
boolean initWifi(int tries = 2, int waitTime = 2) {
    int status = WL_IDLE_STATUS;
    WiFi.mode(WIFI_STA);

    while(status != WL_CONNECTED && (tries-- > 0)) {
        status = WiFi.begin(ssid, password);
        int timeout = waitTime;

        while (WiFi.status() != WL_CONNECTED && (timeout-- > 0)) {
            delay(1000);
        }
        if (WiFi.status() == WL_CONNECTED) {
            break;
        }
    }

    if(WiFi.status() != WL_CONNECTED) {
        return false;
    }
    else {
        return true;
    }
}


/*
 * MQTT client init connection
 */
boolean initMQTT() {
    Serial.print(F("Attempting MQTT connection..."));

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), clientId.c_str(), password,
                           willTopic, willQoS, willRetain, willMessage)) {
        Serial.println(F("connected"));
        // Once connected, publish an announcement...
        mqttClient.publish(topic, ("connected " + clientId).c_str() , true );
        mqttClient.subscribe(topic_control_motion);
        mqttClient.subscribe(topic_control_reset);
        return true;
    } else {
        return false;
    }
}


/*
 * refresh config parameters to MQTT
 */
boolean mqttRefreshConfig() {
    if(mqttClient.connected()) {
        mqttClient.publish(topic_status_motion, message_status_motion[motion], true);
        mqttClient.publish(topic_control_reset, message_control_reset[0], true);
        return true;
    } else {
        return false;
    }
}


/*
 * updateLocalTime
 */
boolean updateLocalTime() {
    if(timeStatus() == timeSet) {
        if(oldmin != minute()) {
            local = myTZ.toLocal(now(), &tcr);
            sprintf(stringTime, "%02d:%02d", hour(local), minute(local));
            oldmin = minute();
            return true;
        }
    }
    return false;
}


/*
 * MQTT Callback message
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    char mypayload[length+1];
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        mypayload[i] = (char)payload[i];
    }
    mypayload[length] = '\0';
    Serial.println();

    // Change motion if ON or OFF is received
    if(strcmp(topic, topic_control_motion)==0){
        if(strcmp(mypayload, message_control_motion[1]) == 0) {
            motion = true;
            cfgChangeFlag = true;
        }
        if(strcmp(mypayload, message_control_motion[0]) == 0) {
            motion = false;
            cfgChangeFlag = true;
        }
        if(mqttClient.connected()) {
            mqttClient.publish(topic_status_motion, message_status_motion[motion], true);
        }
    }

    // Reset the camera if RESET received
    if(strcmp(topic, topic_control_reset)==0){
        if(strcmp(mypayload, message_control_reset[1]) == 0) {
            Serial.println(F("Rebooting..."));
            if(mqttClient.connected()) {
                mqttClient.publish(topic_control_reset, message_control_reset[2], true);
                delay(5000);
                mqttClient.publish(topic_control_reset, message_control_reset[0], true);
            }
            // reset the camera
            ESP.restart();
        }
    }
}


/*
 * NTP code
 */
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    WiFi.hostByName(ntpServerName, ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE) {
            Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
        }
    }
    return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}


/*
 * saveConfig file
 */
void saveConfig() {
    Serial.println(F("Saving config."));
    File f = LittleFS.open("/esp32cam.cnf", "w");
    if(!f){
        Serial.println(F("Write failed."));
    } else {
        f.print(F("motion="));
        f.println(motion);
        f.flush();
        f.close();
        Serial.println(F("Saved values."));
    }
}


/*
 * Setup
 */
void setup() {
    Serial.println(F("Boot Start."));

    // Set-up I/O
    Serial.begin(9600);
    delay(10);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

    // get WiFi up and going
	Serial.println("");
	Serial.println(F("Connecting to: "));
    Serial.print(ssid);
    if (!initWifi(5, 10)) {
        Serial.println(F("Initial Connection Failed! Rebooting..."));
        delay(5000);
        ESP.restart();
    }
    Serial.println(F("Connected."));

    // OTA set-up

    // Port defaults to 3232
    //ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    // ArduinoOTA.setHostname("myesp32");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_FS
                type = "filesystem";
            }

            // NOTE: if updating FS this would be the place to unmount FS using FS.end()
            LittleFS.end();
            Serial.println("Start updating " + type);
        });

    ArduinoOTA.onEnd([]() {
            Serial.println("\nEnd");
        });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        });

    ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println(F("Auth Failed"));
            } else if (error == OTA_BEGIN_ERROR) {
                Serial.println(F("Begin Failed"));
            } else if (error == OTA_CONNECT_ERROR) {
                Serial.println(F("Connect Failed"));
            } else if (error == OTA_RECEIVE_ERROR) {
                Serial.println(F("Receive Failed"));
            } else if (error == OTA_END_ERROR) {
                Serial.println(F("End Failed"));
            }
        });

    // OTA
    ArduinoOTA.begin();
    Serial.println(F("OTA Ready."));

    // time
    sprintf(stringTime, "%s", defaultTime);
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);
    setSyncInterval(300);
    if(!updateLocalTime()){
        Serial.println(F("Time update failed"));
        sprintf(stringTime, "%s", defaultTime);
    }
    oldmin = 99;

    // MQTT
    clientId = clientId + WiFi.localIP();
    topic_control_motion_s.toCharArray(topic_control_motion, 30); // control topic
    topic_status_motion_s.toCharArray(topic_status_motion, 30); // bounce back
    topic_control_reset_s.toCharArray(topic_control_reset, 30); // reset position
    topic_status_hb_s.toCharArray(topic_status_hb, 30); // hb topic
    mqttClient.setServer(mqtt_server, mqtt_serverport);
    mqttClient.setCallback(mqttCallback);
    if(initMQTT()) {
        Serial.println(F("MQTT connected."));
    } else {
        Serial.println(F("MQTT failed."));
    }

    /* // LittleFS */
    /* // LittleFSConfig cfg; // not used in ESP32 LITTLEFS */
    /* // LittleFS.setConfig(cfg); // not used in ESP32 LITTLEFS */
    LittleFS.begin(FORMAT_FS_IF_FAILED);
    Serial.println(F("Loading config"));
    File f = LittleFS.open("/esp32cam.cnf", "r");
    if (!f) {
        //File does not exist -- first run or someone called format()
        //Will not create file; run save code to actually do so (no need here since
        //it's not changed)
        Serial.println(F("Failed to open config file"));
        motion = false;
    } else {
        while(f.available()){
            String key = f.readStringUntil('=');
            String value = f.readStringUntil('\n');
            Serial.println(key + F(" = [") + value + ']');
            Serial.println(key.length());
            if (key == F("motion")) {
                motion = value.toInt();
            }
        }
        Serial.println(F("Config file"));
    }
    f.close();


    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if(psramFound()){
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 2);     // -2 to 2
    s->set_contrast(s, 1);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

    Serial.print("Camera Stream Ready! Go to: http://");
    Serial.println(WiFi.localIP());

    // Start streaming web server
    startCameraServer();

    // clean-up
    mqttRefreshConfig();
    Serial.println(F("Boot complete."));
    delay(1000);
}


/*
 * loop
 */
void loop() {
    currentMillis = millis();
    status = WiFi.status();
    mqttConnected = mqttClient.connected();

    // only do OTA for a few munutes after boot or reboot
    // send a MQTT reset if you want to do an OTA
    if(OTAavailableMillis == 0) {
        OTAavailableMillis = currentMillis;
    }
    if(currentMillis <= (OTAavailableMillis + OTAavailable)) {
        ArduinoOTA.handle();
    }

    // frame timer
    if(currentMillis - newpicInterval > newpicMillis) {
        newpicMillis = currentMillis;
        newpic = true;
    }

    // Wifi status & init if dropped
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Reconnecting WiFi."));
        if(initWifi()) {
            Serial.println(F("WiFi connected."));
        }
    } else {
        // MQTT status & init if dropped
        if(!mqttClient.connected()) {
            Serial.println(F("Reconnecting MQTT."));
            if(initMQTT()) {
                Serial.println(F("MQTT connected."));
            }
        } else {
            mqttClient.loop();
        }
    }

    // MQTT Heartbeat
    if(currentMillis - hbMillis > hbInterval) {
        hbMillis = currentMillis;
        heartbeat = not(heartbeat);
        if(mqttClient.connected()) {
            mqttClient.publish(topic_status_hb, message_status_hb[heartbeat] , true);
        }
    }

    // update Local Time
    updateLocalTime();

    // flash local led hb if any non-standard condition otherwise off
    if(WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
        if(currentMillis - ledMillis > ledInterval) {
            ledMillis = currentMillis;
            ledState = not(ledState);
            digitalWrite(LED_BUILTIN, !ledState);
        }
    } else {
        ledState = false;
        digitalWrite(LED_BUILTIN, !ledState);
    }

// update the config file if required
    if(cfgChangeFlag) {
        saveConfig();
        cfgChangeFlag = false;
    }
}
