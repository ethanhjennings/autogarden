#include "config.h"

#include <Arduino.h>
#include <WiFi.h>

#include "Preferences.h"

Preferences p;

void Config::toString(char* strOut) {
    sprintf(strOut,
            "ssid: %s\n"
            "password: %s\n"
            "clientName: %s\n"
            "gateway: %s\n"
            "subnet: %s\n"
            "staticIP: %s\n"
            "mqttBrokerIP: %s\n"
            "btPassword: <hidden>\n"
            "isPump: %s\n"
            "hasWaterLevelSensors: %s\n",
            ssid, password, clientName, gateway.toString().c_str(), subnet.toString().c_str(), staticIP.toString().c_str(),
            mqttBrokerIP.toString().c_str(), isPump ? "true" : "false", hasWaterLevelSensors ? "true" : "false");
}

void loadConfig(Config* config) {
    p.begin("config", true);  // Read only
    p.getBytes("config", (void*)config, sizeof(Config));
    p.end();
    if (config->magicNumTest != MAGIC_NUM) {
        strcpy(config->clientName, "unconfigured");
        strcpy(config->btPassword, "password");
    }
}

void saveConfig(Config& config) {
    config.magicNumTest = MAGIC_NUM;
    p.begin("config", false);  // Read-write
    p.putBytes("config", (void*)&config, sizeof(Config));
    p.end();
}
