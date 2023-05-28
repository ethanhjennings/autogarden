// Helpers to control LED status

#ifndef LED_HELPER_H
#define LED_HELPER_H

#include <stdint.h>

constexpr uint8_t LED_R_PIN = 16;
constexpr uint8_t LED_G_PIN = 13;
constexpr uint8_t LED_B_PIN = 2;

constexpr uint8_t LED_R_CHANNEL = 0;
constexpr uint8_t LED_G_CHANNEL = 1;
constexpr uint8_t LED_B_CHANNEL = 2;

constexpr uint32_t LED_FREQ = 5000;  // in hz

enum led_anim_t {
    NONE = 0,
    WIFI_CONNECTING = 1,
    OTA_DOWNLOADING = 2,
    OTA_ROLLBACK = 3,
    SUCCESS = 4,
    ERROR = 5,
    ACTIVATED = 6,
    CONFIRM_FLASH = 7,
    ERROR_FLASH = 8,
};

void setupStatusLed();
void setLedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 30 /* 0 through 31 */);
void hsvToRGB(double hue, double saturation, double value, uint8_t& red, uint8_t& green, uint8_t& blue);
void startLedAnimation(const led_anim_t animation_type);
void runLedAnimation(void* pvParameter);
led_anim_t getRunningLedAnimation();

#endif