#include <Arduino.h>
#include <ArduinoOTA.h>
#include <BluetoothSerial.h>
#include <MQTT.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <Wire.h>

#include "config.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "handleBT.h"
#include "handle_actuators.h"
#include "led_helper.h"
#include "I2CSoilMoistureSensor.h"
#include "version.h"

BluetoothSerial SerialBT;

WiFiClient wifiClient;
MQTTClient mqtt;

Config config;

String ROLLBACK_TOPIC = "autogarden/<client>/rollback";
String PUMP_TOPIC = "autogarden/pump";
String SOLENOID_TOPIC = "autogarden/<client>/solenoid";
String SOIL_MOISTURE_TOPIC = "autogarden/<client>/soil_moisture";
String WATER_LEVEL_HIGH_TOPIC = "autogarden/water-level/high";
String WATER_LEVEL_LOW_TOPIC = "autogarden/water-level/low";
String ONLINE_TOPIC = "autogarden/<client>/online";
String VERSION_HASH_TOPIC = "autogarden/<client>/version_hash";

constexpr uint8_t WATER_LEVEL_LOW_PIN = 18;
constexpr uint8_t WATER_LEVEL_HIGH_PIN = 19; 

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

ActuatorManager* actuator;

// Needed to override ArduinoOTA's default rollback verification and allow us to call esp_ota_mark_app_valid_cancel_rollback() later.
//  See: https://github.com/espressif/arduino-esp32/issues/5871
extern "C" bool verifyRollbackLater() { return true; }

void setupWifi() {
    Serial.println("Setting up wifi");
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.mode(WIFI_STA);


    WiFi.config(config.staticIP, config.gateway, config.subnet);
    WiFi.begin(config.ssid, config.password);

    int i = 0;
    startLedAnimation(WIFI_CONNECTING);
    while (WiFi.status() != WL_CONNECTED && i++ < 3000) {
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        // Success!
        startLedAnimation(SUCCESS);

        Serial.println("Connected to wifi!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

    } else {
        // Wifi connection failed! Let's sleep for a bit and hope that rebooting will help
        Serial.println("Wifi connection failed!");
        delay(5000);
        ESP.restart();
    }
}

void messageReceived(String& topic, String& payload) {
    Serial.println("mqtt message: " + topic + " - " + payload);
    SerialBT.println("mqtt message: " + topic + " - " + payload);

    if (topic.equals(ROLLBACK_TOPIC)) {
        Serial.println("Rollback OTA!");
        startLedAnimation(ERROR);
        delay(3000);

        esp_ota_mark_app_invalid_rollback_and_reboot();
    } else if (topic.equals(PUMP_TOPIC) || topic.equals(SOLENOID_TOPIC)) {
        int32_t timer_millis = payload.toInt();
        if (timer_millis > 0) {
            actuator->setState(true, timer_millis);
        } else {
            actuator->setState(false, 0);
        }
    }
}

void replaceTemplate(String& str) { str.replace("<client>", config.clientName); }

boolean setupMqtt() {
    Serial.print("Setting up MQTT...");
    SerialBT.print("Setting up MQTT...");
    replaceTemplate(ROLLBACK_TOPIC);
    replaceTemplate(PUMP_TOPIC);
    replaceTemplate(SOLENOID_TOPIC);
    replaceTemplate(SOIL_MOISTURE_TOPIC);
    replaceTemplate(WATER_LEVEL_HIGH_TOPIC);
    replaceTemplate(WATER_LEVEL_LOW_TOPIC);
    replaceTemplate(ONLINE_TOPIC);
    replaceTemplate(VERSION_HASH_TOPIC);

    mqtt.begin(config.mqttBrokerIP, wifiClient);
    mqtt.onMessage(messageReceived);

    bool result;
    if ( (result = mqtt.connect(config.clientName)) ) {
        Serial.println("Connected!");
        SerialBT.println("Connected!");
        // Overwrite existing will
        mqtt.publish(ONLINE_TOPIC, "1", true, 1);
        mqtt.publish(VERSION_HASH_TOPIC, VERSION_HASH, true, 1);
    } else {
        Serial.println("ERROR!");
        SerialBT.println("ERROR!");
    }

    // Send 0 if connection fails in case we lose power or crash
    mqtt.setWill(ONLINE_TOPIC.c_str(), "0", true, 1);

    mqtt.subscribe(ROLLBACK_TOPIC);
    if (config.isPump) {
        mqtt.subscribe(PUMP_TOPIC);
    } else {
        mqtt.subscribe(SOLENOID_TOPIC);
    }
    if (config.hasWaterLevelSensors) {
        mqtt.subscribe(WATER_LEVEL_HIGH_TOPIC);
        mqtt.subscribe(WATER_LEVEL_LOW_TOPIC);
    }
    return result; 
}

int cmp(const void* pa, const void* pb) {
    uint32_t a = *(const uint32_t*)pa;
    uint32_t b = *(const uint32_t*)pb;
    if (a < b) {
        return -1;
    } else if (a > b) {
        return 1;
    } else {
        return 0;
    }
}

QueueHandle_t moistureQueue;

double getSoilMoisture(I2CSoilMoistureSensor& moistureSensor, uint32_t numSamples, uint32_t delayTime) {
    //return moistureSensor.getCapacitance();
    uint32_t moistureArray[numSamples];
    for (int i = 0; i < numSamples; i++) {
        moistureArray[i] = moistureSensor.getCapacitance();
        delay(delayTime);
    }
    qsort(&moistureArray, numSamples, sizeof(uint32_t), cmp);
    
    double capacitance = 0;
    double startIdx = numSamples*0.25;
    double endIdx = numSamples*0.75;

    for (int i = startIdx; i < endIdx; i++) {
        capacitance += moistureSensor.getCapacitance();
    }
    return capacitance/(endIdx - startIdx + 1);
}

void readMoisture(void* pvParameter) {
    const auto INTERVAL = 5*60*1000; // 5 minutes

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    I2CSoilMoistureSensor moistureSensor;


    while (true) {
    moistureSensor.begin();
    delay(1000); // Wait for sensor to boot
        auto moisture = getSoilMoisture(moistureSensor, 1000, 10);
        xQueueSend(moistureQueue, (void*)&moisture, portMAX_DELAY);
        moistureSensor.sleep();

        delay(INTERVAL);
    }
}

void readSensors() {
    if (config.hasWaterLevelSensors) {
        // Water level

        static int8_t lastWaterLevelLow = -1;
        static int8_t lastWaterLevelHigh = -1;

        int8_t waterLevelLow = digitalRead(WATER_LEVEL_LOW_PIN);
        int8_t waterLevelHigh = digitalRead(WATER_LEVEL_HIGH_PIN);
        
        if (waterLevelLow != lastWaterLevelLow) {
            mqtt.publish(WATER_LEVEL_LOW_TOPIC, String(waterLevelLow), true, 2);
            lastWaterLevelLow = waterLevelLow;
        }

        if (waterLevelHigh != lastWaterLevelHigh) {
            mqtt.publish(WATER_LEVEL_HIGH_TOPIC, String(waterLevelHigh), true, 2);
            lastWaterLevelHigh = waterLevelHigh;
        }
    }
    double moisture;
    if (xQueueReceive(moistureQueue, &moisture, 0 /* Don't block */) == pdTRUE) {
        mqtt.publish(SOIL_MOISTURE_TOPIC, String(moisture));
    }
}

void runOTA() {
    static bool markedValid = false;
    ArduinoOTA.handle();
    if (!markedValid && millis() > 1000 * 30) {
        Serial.println("Marking OTA app as valid!");
        esp_ota_mark_app_valid_cancel_rollback();
        markedValid = true;
    }
}

void setupOTA() {
    ArduinoOTA
        .onStart([]() {
            Serial.println("Starting OTA download");
            startLedAnimation(OTA_DOWNLOADING);
        })
        .onEnd([]() {
            Serial.println("OTA updated successfully!");
            startLedAnimation(NONE);
            delay(350);
        })
        .onError([](ota_error_t error) {
            Serial.println("OTA Error!");
            startLedAnimation(ERROR);
            delay(5000);
            ESP.restart();
        });
    ArduinoOTA.begin();

    Serial.println("Finished setting up OTA");
}

void checkRollback() {
    // Get image state of previous partition so we can detect a rollback.
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* prev_part = esp_ota_get_next_update_partition(running);
    esp_ota_img_states_t ota_state;
    esp_ota_get_state_partition(prev_part, &ota_state);

    if (ota_state == ESP_OTA_IMG_ABORTED) {
        Serial.println("******************************************");
        Serial.println("*** WARNING! This image is a rollback! ***");
        Serial.println("******************************************");

        startLedAnimation(OTA_ROLLBACK);
    }
}


void setup() {
    Serial.begin(115200);
    
    while (!Serial) {
    }
    
    Serial.print("Version hash: ");
    Serial.println(VERSION_HASH);

    pinMode(WATER_LEVEL_HIGH_PIN, INPUT);
    pinMode(WATER_LEVEL_LOW_PIN, INPUT);

    loadConfig(&config);
    setupStatusLed();
    setupBT(&config, &SerialBT);
    setupWifi();
    setupOTA();
    setupMqtt();
    checkRollback();
    if (config.isPump) {
        actuator = new PumpManager();
    } else {
       actuator = new SolenoidManager();
    }
    actuator->begin();

    moistureQueue = xQueueCreate(10, sizeof(double));
    xTaskCreate(&readMoisture, "read_moisture", 1024*8, NULL, 5, NULL);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        setupWifi();
    }
    if (!mqtt.connected()) {
        Serial.println("MQTT disconnected...");
        if (!setupMqtt()) {
            if (getRunningLedAnimation() != SUCCESS) {
                startLedAnimation(ERROR);
            }
        } else {
            if (getRunningLedAnimation() != ERROR) {
                startLedAnimation(ERROR);
            }
        }
    }
    mqtt.loop();
    runOTA();
    readSensors();
    delay(50);
}
