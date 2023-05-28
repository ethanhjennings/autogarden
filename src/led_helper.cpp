// Helpers to control LED status

#include "led_helper.h"

#include <Arduino.h>
#include <math.h>

TaskHandle_t led_task_handle = NULL;

void setupStatusLed() {
    // red
    ledcSetup(LED_R_CHANNEL, LED_FREQ, 13);
    ledcAttachPin(LED_R_PIN, LED_R_CHANNEL);
    // green
    ledcSetup(LED_G_CHANNEL, LED_FREQ, 13);
    ledcAttachPin(LED_G_PIN, LED_G_CHANNEL);
    // blue
    ledcSetup(LED_B_CHANNEL, LED_FREQ, 13);
    ledcAttachPin(LED_B_PIN, LED_B_CHANNEL);

    xTaskCreate(&runLedAnimation, "run_led_anim", 8 * 1024, 0, 30, &led_task_handle);
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness /* brightness can be 0 through 31 */) {
    ledcWrite(LED_R_CHANNEL, r * brightness);
    ledcWrite(LED_G_CHANNEL, g * brightness * 0.8);
    ledcWrite(LED_B_CHANNEL, b * brightness * 0.8);
}

void hsvToRGB(double hue, double saturation, double value, uint8_t& red, uint8_t& green, uint8_t& blue) {
    double r, g, b;

    auto i = static_cast<int>(hue * 6);
    auto f = hue * 6 - i;
    auto p = value * (1 - saturation);
    auto q = value * (1 - f * saturation);
    auto t = value * (1 - (1 - f) * saturation);

    switch (i % 6) {
        case 0:
            r = value, g = t, b = p;
            break;
        case 1:
            r = q, g = value, b = p;
            break;
        case 2:
            r = p, g = value, b = t;
            break;
        case 3:
            r = p, g = q, b = value;
            break;
        case 4:
            r = t, g = p, b = value;
            break;
        case 5:
        default:
            r = value, g = p, b = q;
            break;
    }

    red = static_cast<uint8_t>(r * 255);
    green = static_cast<uint8_t>(g * 255);
    blue = static_cast<uint8_t>(b * 255);
}

led_anim_t animationState = NONE;
void startLedAnimation(const led_anim_t animationType) {
    animationState = animationType;
    xTaskNotify(led_task_handle, (u_int32_t)animationType, eSetValueWithOverwrite);
}

led_anim_t getRunningLedAnimation() { return animationState; }

void runLedAnimation(void* pvParameter) {
    led_anim_t animationType = NONE;
    led_anim_t returnType = NONE;

    // True when actively animating, false when in a steady state
    // Animating means that it will not wait for xTaskNotifyWait()
    bool animating = false;
    int64_t startTime = esp_timer_get_time();

    while (true) {
        if (xTaskNotifyWait(0, ULONG_MAX, (uint32_t*)&returnType, (animating ? 0 : portMAX_DELAY)) == pdTRUE) {
            startTime = esp_timer_get_time();
            animationType = returnType;
        }

        double seconds = (esp_timer_get_time() - startTime) / 1e6;

        float alpha;
        switch (animationType) {
            case NONE:
                animating = false;
                setLedColor(0, 0, 0);
                break;
            case WIFI_CONNECTING:
                animating = true;
                setLedColor(0, 0, 255 * (0.5 * (sin(seconds * 20 + 1.5 * PI) + 1)));
                break;
            case OTA_DOWNLOADING:
                animating = false;
                setLedColor(255, 0, 255);
                break;
            case OTA_ROLLBACK:
                animating = true;
                alpha = 0.5 * (sin(seconds * 20 + 1.5 * PI) + 1);
                setLedColor(alpha * 255, 0, alpha * 255);
                if (seconds > 3.0) {
                    animating = false;
                    setLedColor(0, 0, 0);
                }
                break;
            case SUCCESS:
                animating = true;
                if (seconds < 0.5) {
                    setLedColor(0, 255, 0);
                } else if (seconds < 1.0) {
                    setLedColor(0, 255 - 255 * (seconds - 0.5) / 0.5, 0);
                } else {
                    setLedColor(0, 0, 0);
                    animating = false;
                }
                break;
            case ERROR:
                animating = true;
                setLedColor(255 * (0.5 * (sin(seconds * 20 + 1.5 * PI) + 1)), 0, 0);
                break;
            case ACTIVATED:
                animating = true;
                alpha = 0.5 * (sin(seconds * 5 + 1.5 * PI) + 1);
                setLedColor(0, alpha * 255, alpha * 255);
                break;
            case CONFIRM_FLASH:
                animating = true;
                if (seconds < 0.5) {
                    setLedColor(0, 255 * (seconds / 0.5), 0);
                } else if (seconds < 1.0) {
                    setLedColor(0, 255 - 255 * (seconds - 0.5) / 0.5, 0);
                } else {
                    setLedColor(0, 0, 0);
                    animating = false;
                }
                break;
            case ERROR_FLASH:
                animating = true;
                if (seconds < 0.5) {
                    setLedColor(255 * (seconds / 0.5), 0, 0);
                } else if (seconds < 1.0) {
                    setLedColor(255 - 255 * (seconds - 0.5) / 0.5, 0, 0);
                } else {
                    setLedColor(0, 0, 0);
                    animating = false;
                }
                break;
        }

        delay(2);
    }
}
