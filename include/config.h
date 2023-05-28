#ifndef AUTOGARDEN_CONFIG_H
#define AUTOGARDEN_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>

#include "Preferences.h"

// Used to check if config has been saved yet
constexpr uint64_t MAGIC_NUM = 0x0123456789abcdef;

struct Config {
    uint64_t magicNumTest;
    char ssid[128];
    char password[128];
    char clientName[256];
    IPAddress gateway;
    IPAddress subnet;
    IPAddress staticIP;
    IPAddress mqttBrokerIP;
    char btPassword[128];
    bool isPump;
    bool hasWaterLevelSensors;

    void toString(char* strOut);
};

void loadConfig(Config* config);
void saveConfig(Config& config);

#endif