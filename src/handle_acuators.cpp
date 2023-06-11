#include <Arduino.h>

#include "handle_actuators.h"
#include "led_helper.h"

void ActuatorManager::timerCallback(TimerHandle_t timer) {
    ActuatorManager* this_obj = (ActuatorManager*)pvTimerGetTimerID(timer);
    ActuatorCommand cmd(false, 0);
    xQueueOverwrite(this_obj->queue, &cmd);
}

void ActuatorManager::taskCallback(void* pvParameter) {
    ActuatorManager* this_obj = (ActuatorManager*)pvParameter;
    this_obj->setup();
    while (true) {
        ActuatorCommand cmd;
        if (xQueueReceive(this_obj->queue, (void*)&cmd, portMAX_DELAY)) {
            this_obj->stateChange(cmd);
        }
    }
}

void ActuatorManager::startTimer(uint32_t millis) {
    xTimerChangePeriod(timer, pdMS_TO_TICKS(millis), portMAX_DELAY);
    xTimerStart(timer, portMAX_DELAY);
}

void ActuatorManager::cancelTimer() { xTimerStop(timer, portMAX_DELAY); }

void ActuatorManager::begin() {
    queue = xQueueCreate(1, sizeof(ActuatorCommand));

    timer = xTimerCreate("actuator_timer", 1, false, this, &timerCallback);
    xTaskCreate(&taskCallback, "actuator_task", 1024 * 8, this, 20, &task);
}

void ActuatorManager::setState(bool enable, uint32_t lifeSeconds) {
    ActuatorCommand cmd(enable, lifeSeconds);
    xQueueOverwrite(queue, &cmd);
}

void PumpManager::setup() {
    pinMode(PUMP_PIN, OUTPUT);
}

void PumpManager::stateChange(ActuatorCommand& cmd) {
    if (cmd.onState) {
        Serial.println("Turning pump on!");

        startTimer(cmd.lifeSeconds*1000);
        startLedAnimation(ACTIVATED);

        digitalWrite(PUMP_PIN, HIGH);
    } else {
        // This may run twice if another mqtt off message comes in while timer
        // is on, which should be fine
        Serial.println("Turning pump off!");

        cancelTimer();
        startLedAnimation(NONE);

        digitalWrite(PUMP_PIN, LOW);
    }
}

void SolenoidManager::setup() {
    ledcSetup(SOLENOID_PWM_CHANNEL, SOLENOID_PWM_FREQ, 8);
    ledcAttachPin(SOLENOID_PIN, SOLENOID_PWM_CHANNEL);
}

void SolenoidManager::stateChange(ActuatorCommand& cmd) {
    if (cmd.onState) {
        Serial.println("Turning solenoid on!");

        startTimer(cmd.lifeSeconds*1000);
        startLedAnimation(ACTIVATED);

        // Full strength for one second to fully open valve
        ledcWrite(SOLENOID_PWM_CHANNEL, 255);
        delay(1000);

        // Now lower strength just to keep open
        ledcWrite(SOLENOID_PWM_CHANNEL, 120);
    } else {
        Serial.println("Turning solenoid off!");

        cancelTimer();

        // Fade out from whatever our current pwm is
        int32_t start = ledcRead(SOLENOID_PWM_CHANNEL);
        for (int32_t i = start; i >= 0; i--) {
            ledcWrite(SOLENOID_PWM_CHANNEL, i);
            delay(20);
        }
        ledcWrite(SOLENOID_PWM_CHANNEL, 0);
        startLedAnimation(NONE);
    }
}
