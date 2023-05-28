#include "handleBT.h"

#include <Arduino.h>
#include <BluetoothSerial.h>

#include "config.h"
#include "led_helper.h"

// Handles configuration settings via a CLI Bluetooth Serial interface.
// Should work with a phone app or via windows COM port.

// TODO: Maybe switch to secure BLE

BluetoothSerial* bts;
volatile bool needToAuthenitcate = true;

void btClientCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        needToAuthenitcate = true;
        bts->println("Please enter password:");
    }
}

void setupBT(Config* config, const BluetoothSerial* SerialBT) {
    Serial.print("Starting bluetooth serial...");
    bts = (BluetoothSerial*)SerialBT;
    bts->begin("autogarden-" + String(config->clientName));
    bts->setPin("1234");
    bts->register_callback(btClientCallback);

    xTaskCreate(&handleBT, "handleBT", 1024 * 10, (void*)config, 20, NULL);
}

void printBTHelp() {
    bts->print(
        "Here are the available commands:\n"
        "help\n"
        "show\n"
        "reboot\n"
        "rollback\n"
        "set_ssid ...\n"
        "set_password ...\n"
        "set_clientName ...\n"
        "set_gateway <ip address>\n"
        "set_subnet <ip address>\n"
        "set_staticIP <ip address>\n"
        "set_mqttBrokerIP <ip address>\n"
        "set_btPassword ...\n"
        "set_isPump <bool>\n"
        "set_hasWaterLevelSensors <bool>\n");
}

bool strToBool(char* str, bool& val) {
    for (int i = 0; i < strlen(str); i++) str[i] = tolower(str[i]);
    if ((strcmp(str, "true") == 0) || (strcmp(str, "t") == 0) || (strcmp(str, "1") == 0)) {
        val = true;
        return true;
    } else if ((strcmp(str, "false") == 0) || (strcmp(str, "f") == 0) || (strcmp(str, "0") == 0)) {
        val = false;
        return true;
    }

    return false;
}

void handleBT(void* pvParameter) {
    Config* config = (Config*)pvParameter;
    strcpy(config->btPassword,"password");
    while (true) {
        if (bts->available()) {
            char command[100];
            char* name;
            char* argument;
            int len = bts->readBytesUntil('\n', command, 99);

            if (command[len - 1] == '\r')  // Some apps use '\r\n' as line ending
                command[len - 1] = 0;
            else
                command[len] = 0;


            name = strtok(command, " ");
            argument = strtok(NULL, " ");

            Serial.printf("Got bluetooth command, name: %s, argument: %s\n", command, (argument != NULL ? argument : ""));

            if (needToAuthenitcate) {
                if (strcmp(name, config->btPassword) == 0) {
                    bts->println("Authenicated successfully!\nUse 'help' for list of commands.");
                    needToAuthenitcate = false;
                } else {
                    bts->println("Authentication failed.");
                }
            } else {
                bool error = false;
                bool saving = true;
                if (strcmp(name, "help") == 0) {
                    saving = false;
                    printBTHelp();
                } else if (strcmp(name, "show") == 0) {
                    saving = false;
                    char output[1024];
                    config->toString(output);
                    bts->print(output);
                } else if (strcmp(name, "reboot") == 0) {
                    saving = false;
                    bts->println("Rebooting...");
                    startLedAnimation(CONFIRM_FLASH);
                    delay(2000);
                    ESP.restart();
                } else if (strcmp(name, "rollback") == 0) {
                    saving = false;
                    bts->println("Rolling back OTA...");
                    startLedAnimation(CONFIRM_FLASH);
                    delay(2000);
                    ESP.restart();
                } else if (strcmp(name, "set_ssid") == 0 && argument != NULL) {
                    strcpy(config->ssid, argument);
                } else if (strcmp(name, "set_password") == 0 && argument != NULL) {
                    strcpy(config->password, argument);
                } else if (strcmp(name, "set_clientName") == 0 && argument != NULL) {
                    strcpy(config->clientName, argument);
                } else if (strcmp(name, "set_gateway") == 0 && argument != NULL) {
                    config->gateway.fromString(argument);
                } else if (strcmp(name, "set_subnet") == 0 && argument != NULL) {
                    config->subnet.fromString(argument);
                } else if (strcmp(name, "set_staticIP") == 0 && argument != NULL) {
                    config->staticIP.fromString(argument);
                } else if (strcmp(name, "set_mqttBrokerIP") == 0 && argument != NULL) {
                    config->mqttBrokerIP.fromString(argument);
                } else if (strcmp(name, "set_btPassword") == 0 && argument != NULL) {
                    strcpy(config->btPassword, argument);
                } else if (strcmp(name, "set_isPump") == 0 && argument != NULL) {
                    if (!strToBool(argument, config->isPump)) {
                        error = true;
                        bts->println("Command not understood.");
                    }
                } else if (strcmp(name, "set_hasWaterLevelSensors") == 0 && argument != NULL) {
                    if (!strToBool(argument, config->hasWaterLevelSensors)) {
                        error = true;
                        bts->println("Command not understood.");
                    }
                } else {
                    error = true;
                    bts->println("Command not understood.");
                }

                if (!error) {
                    startLedAnimation(CONFIRM_FLASH);
                    if (saving) {
                        bts->println("Saving config.");
                        saveConfig(*config);
                    }
                } else {
                    startLedAnimation(ERROR_FLASH);
                }
            }
        }
        delay(100);
    }
}