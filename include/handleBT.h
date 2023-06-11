#ifndef _HANDLE_BT_H
#define _HANLDE_BT_H

#include <BluetoothSerial.h>

#include "config.h"

void setupBT(Config* config, const BluetoothSerial* SerialBT);
void printBTHelp();
void btClientCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void handleBT(void* pvParameter);

#endif
