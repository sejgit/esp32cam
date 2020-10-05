# esp32cam

Arduino based IP cam using esp32-cam hardware
contains:
MQTT communications
Timezone corrected time
Storage of config using SPIFFS (as LiffleFS not supported with ESP32)
OTA updating

warning no security on MQTT or OTA as meant to be used on local closed network.

