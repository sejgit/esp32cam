#include "main.h"

/* Passwords & Ports
 * wifi: ssid, password
 * ISY: hash, isy, isyport
 * MQTT mqtt_server, mqtt_serverport
 */
#include "sej.h"


/*
 * Camera class
 */
OV2640 cam;


/*
 *  MQTT
 */
bool status = false;
boolean mqttConnected = false;

const char* topic = "sej"; // main topic
String clientId = "esp32cam"; // client ID for this unit
char buffer[256];

const int buffsize = 45;

char topic_control_reset[buffsize];
String topic_control_reset_s= "sej/" + clientId + "/control/reset"; // reset position
const char* message_control_reset[] = {"--", "RESET", "Resetting", "OTA", "OTA available"};

char topic_status_hb[buffsize];
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


/*
 * timers
 */
unsigned long currentMillis = 0;
unsigned long hbMillis = 0;
const long hbInterval = 60000; // how often to send hb
unsigned long ledMillis = 0;
const long ledInterval = 3000; // blink led h
bool ledState = false;


/*
 * I/O
 */
#define LED_BUILTIN 33


/*
 * Function declarations
 */
boolean initWifi(int tries, int waitTime);
boolean initMQTT(void);
boolean mqttRefreshConfig(void);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void enableOTA(void);
void resetDevice(void);

/*
 * Set-up called once after reboot/power-up
 */
void setup()
 {
    Serial.println(F("Boot Start."));

   // Set-up I/O
    Serial.begin(115000);
    delay(10);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

	Serial.println("\n\n##################################");
	Serial.printf("Total heap %d, Free Heap %d\r\n", ESP.getHeapSize(), ESP.getFreeHeap());
	Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\r\n", ESP.getPsramSize(), ESP.getFreePsram());
	Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\r\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
	Serial.printf("Flash Size %d, Flash Speed %d\r\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
	Serial.println("##################################\n\n");

	// Initialize the ESP32 CAM, here we use the AIthinker ESP32 CAM
	delay(100);
	cam.init(esp32cam_aithinker_config);
	delay(100);

	// Connect the WiFi
	Serial.println("");
	Serial.println(F("Connecting to: "));
    Serial.print(ssid);
    if (!initWifi(5, 10)) {
        Serial.println(F("Initial Connection Failed! Rebooting..."));
        delay(5000);
        resetDevice();
    }

    // Print information how to contact the camera server
	IPAddress ip = WiFi.localIP();
	Serial.print("\nWiFi connected with IP ");
	Serial.println(ip);
#ifdef ENABLE_RTSPSERVER
	Serial.print("Stream Link: rtsp://");
	Serial.print(ip);
	Serial.println(":8554/mjpeg/1\n");
#endif
#ifdef ENABLE_WEBSERVER
	Serial.print("Browser Stream Link: http://");
	Serial.println(ip);
	Serial.print("Browser Single Picture Link: http://");
	Serial.print(ip);
	Serial.println("/jpg\n");
#endif
#ifdef ENABLE_WEBSERVER
	// Initialize the HTTP web stream server
	initWebStream();
#endif

#ifdef ENABLE_RTSPSERVER
	// Initialize the RTSP stream server
	initRTSP();
#endif

    // MQTT
    clientId = clientId + WiFi.localIP().toString();
    topic_control_reset_s = "sej/" + clientId + "/control/reset"; // reset position
    topic_status_hb_s = "sej/" + clientId +"/status/hb"; // hb topic
    topic_control_reset_s.toCharArray(topic_control_reset, buffsize); // reset position
    topic_status_hb_s.toCharArray(topic_status_hb, buffsize); // hb topic
    mqttClient.setServer(mqtt_server, mqtt_serverport);
    mqttClient.setCallback(mqttCallback);
    if(!initMQTT()) {
        Serial.println(F("MQTT failed."));
    } else {
        mqttRefreshConfig();
    }

    // clean-up
    Serial.println(F("Boot complete."));
    delay(1000);
}


/*
 * main loop
 */
void loop()
{
    currentMillis = millis();
    status = (WiFi.status() == WL_CONNECTED);
    mqttConnected = mqttClient.connected();

    // Wifi status & init if dropped
    if(!status) {
        Serial.println(F("Reconnecting WiFi."));
        status = initWifi(2, 2);
    } else {
        // OTA
        if (otaStarted)
        {
            ArduinoOTA.handle();
        }

        // MQTT status & init if dropped
        if(!mqttConnected) {
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
        if(mqttConnected) {
            mqttClient.publish(topic_status_hb, message_status_hb[heartbeat] , true);
        }
    }

    // flash local led hb if any non-standard condition otherwise off
    if(!status || !mqttConnected || otaStarted) {
        if(currentMillis - ledMillis > ledInterval) {
            ledMillis = currentMillis;
            ledState = not(ledState);
            digitalWrite(LED_BUILTIN, !ledState);
        }
    } else {
        ledState = false;
        digitalWrite(LED_BUILTIN, !ledState);
    }
}


/*
 * Establish Wi-Fi connection
 */
boolean initWifi(int tries = 2, int waitTime = 2) {
    int initStatus = WL_IDLE_STATUS;
    WiFi.mode(WIFI_STA);

    while(initStatus != WL_CONNECTED && (tries-- > 0)) {
        status = WiFi.begin(ssid, password);
        int timeout = waitTime;

        while (WiFi.status() != WL_CONNECTED && (timeout-- > 0)) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(F("Connected."));
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
boolean initMQTT(void) {
    Serial.print(F("Attempting MQTT connection..."));

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), clientId.c_str(), password,
                           willTopic, willQoS, willRetain, willMessage)) {
        Serial.println(F("connected"));
        // Once connected, publish an announcement...
        mqttClient.publish(topic, ("connected " + clientId).c_str() , true );
        mqttClient.subscribe(topic_control_reset);
        return true;
    } else {
        return false;
    }
}


/*
 * refresh config parameters to MQTT
 */
boolean mqttRefreshConfig(void) {
    if(mqttClient.connected()) {
        mqttClient.publish(topic_control_reset, message_control_reset[0], true);
        return true;
    } else {
        return false;
    }
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

    // Reset the camera if RESET received
    if(strcmp(topic, topic_control_reset)==0){
        if(strcmp(mypayload, message_control_reset[1]) == 0) {
            resetDevice();
        } else if(strcmp(mypayload, message_control_reset[3]) == 0 && !otaStarted) {
            enableOTA();
        } else if(strcmp(mypayload, message_control_reset[3]) == 0 && otaStarted) {
            enableOTA();
        }
    }
}


/**
 * Handle OTA through MQTT & reset
 */
void enableOTA(void)
{
	// If OTA is not enabled
	if (!otaStarted)
	{
        Serial.println(F("OTA turned on..."));
        if(mqttConnected) {
            mqttClient.publish(topic_control_reset, message_control_reset[4], true);
        }
		// Stop the camera servers
#ifdef ENABLE_WEBSERVER
		stopWebStream();
#endif
#ifdef ENABLE_RTSPSERVER
		stopRTSP();
#endif
		delay(100);
		// Start the OTA server
		startOTA();
		otaStarted = true;
	}
	else
	{
		// If OTA was enabled
        Serial.println(F("OTA turned off..."));
        if(mqttConnected) {
            mqttClient.publish(topic_control_reset, message_control_reset[0], true);
        }
        otaStarted = false;
		// Stop the OTA server
		stopOTA();
		// Restart the camera servers
#ifdef ENABLE_WEBSERVER
		initWebStream();
#endif
#ifdef ENABLE_RTSPSERVER
		initRTSP();
#endif
	}
}


/**
 * Handle reset through MQTT or wifi fail
 */
void resetDevice(void)
{
    Serial.println(F("Rebooting..."));
    if(mqttConnected) {
        mqttClient.publish(topic_control_reset, message_control_reset[2], true);
        delay(5000);
        mqttClient.publish(topic_control_reset, message_control_reset[0], true);
    }
    // reset the camera
	WiFi.disconnect();
	esp_restart();
}
