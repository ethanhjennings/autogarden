#ifndef HANDLE_ACTUATORS_H
#define HANDLE_ACTUATORS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct ActuatorCommand {
    bool onState;
    uint32_t lifeSeconds;
    ActuatorCommand(bool onState, uint32_t lifeSeconds) : onState(onState), lifeSeconds(lifeSeconds) {}
    ActuatorCommand() {}
};

class ActuatorManager {
   private:
    TaskHandle_t task;
    TimerHandle_t timer;
    QueueHandle_t queue = 0;
    static void timerCallback(TimerHandle_t xTimer);
    static void taskCallback(void* pvParameter);

   protected:
    virtual void setup() = 0;
    virtual void stateChange(ActuatorCommand& cmd) = 0;
    void startTimer(uint32_t millis);
    void cancelTimer();

   public:
    void begin();
    void setState(bool enable, uint32_t lifeSeconds);
};

class PumpManager : public ActuatorManager {
   protected:
    virtual void setup();
    virtual void stateChange(ActuatorCommand& cmd);

   public:
    static constexpr uint8_t PUMP_PIN = 17;
    static constexpr void* PUMP_TIMER = 0;
};

class SolenoidManager : public ActuatorManager {
   protected:
    virtual void setup();
    virtual void stateChange(ActuatorCommand& cmd);

   public:
    static constexpr uint8_t SOLENOID_PIN = 17;
    static constexpr uint8_t SOLENOID_TIMER = 1;
    static constexpr uint8_t SOLENOID_PWM_CHANNEL = 4;
    static constexpr uint32_t SOLENOID_PWM_FREQ = 1000;
};

#endif