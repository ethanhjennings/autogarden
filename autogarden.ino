#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoOTA.h>

WiFiClient wifi_client;
PubSubClient mqtt(wifi_client);

Adafruit_BME280 bme;

// Max channels, each channel represents one plant/pot/bed etc.
#define NUM_CHANNELS 8 

// # of samples for moisture reading
#define NUM_SAMPLES 200 

// Interval to read sensors (in ms)
#define SENSOR_READ_INTERVAL 60*1000 

// Max time to keep solenoid on so it doesn't get too hot (in ms)
#define SOLENOID_TIMEOUT 15*60*1000

int MOISTURE_PINS[] = {
  36, // starts at channel 1
  39,
  34, 
  35,
  32,
  33,
  25,
  26
};

int SOLENOID_PINS[] = {
  13, // starts at channel 1
  12,
  14,
  27,
  23,
  19,
  18,
  5
}; 

#define WATER_SENSOR_PIN 17
#define DALLAS_TEMP_PIN 16
#define SENSOR_POWER_PIN 4
#define LED_PIN 2
#define PUMP_PIN 0

bool pump = false;
bool water_level = false;
int8_t solenoid_state = -1;
uint64_t solenoid_start_time = 0;

// Config that gets saved into EEPROM
struct AutoGardenConfig {
  char ssid[32];
  char password[64];
  char mqtt_server[64];
  char ota_hostname[32];
  char ota_password[32];
  DeviceAddress dallas_temp_addr[8];
  int wet_adc[8];
  int dry_adc[8];
  bool channel_enabled[8];
};

AutoGardenConfig config;

OneWire oneWire(DALLAS_TEMP_PIN);
DallasTemperature dallas_temp(&oneWire);

class Log {
 // Logs to both serial and to the network for debugging
 public:
  bool send_mqtt = false;
  String buffer_str;
  void print(String str) {
    Serial.print(str);
    if (send_mqtt) {
      buffer_str += str;
    }
  }
  void println(String str = "") {
    Serial.println(str);
    if (send_mqtt) {
      buffer_str += str;
      mqtt.publish("autogarden/console", buffer_str.c_str());
      buffer_str = "";
    }
  }
  template <typename... T>
  void printlnf(String str, T... args) {
    char buff[256];
    snprintf(buff, 256, str.c_str(), args...);
    println(buff);
  }
} Log;


void save_config() {
  EEPROM.begin(512);
  EEPROM.put(0, config);
  EEPROM.end();
  Log.println("Config saved!");
}

void load_config() {
  EEPROM.begin(512);
  EEPROM.get(0, config);
  EEPROM.end();
  Log.println("Config loaded!");
}

void check_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Log.println("Reconnecting to wifi...");
    int attempts = 0;
    WiFi.begin(config.ssid, config.password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(50);
      attempts++;
      if (attempts >= 50) {
        Log.println("Giving up on connecting to wifi...");
        return;
      }
    }
  }
  if (!mqtt.connected()) {
    Serial.println("Reconnecting to MQTT...");
    mqtt.setServer(config.mqtt_server, 1883);
    mqtt.setCallback(mqtt_callback);
    mqtt.connect("esp32-autogarden",NULL,NULL,0,0,0,0,0);
    if (!mqtt.connected()) {
      Serial.println("Giving up on connecting to mqtt...");
      return;
    }
    mqtt.subscribe("autogarden/#");
  }
}

void flash_led(int delay_ms, int num_times) {
  for (int i = 0; i < num_times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN, LOW);
    delay(delay_ms);
  }
}

void setup_OTA() {
  ArduinoOTA.setHostname(config.ota_hostname);
  ArduinoOTA.setPassword(config.ota_password);
  ArduinoOTA
    .onStart([]() {
      Log.println("Starting OTA upload...");
      digitalWrite(LED_PIN, HIGH);
    })
    .onEnd([]() {
      Log.println("OTA finished!");
      flash_led(100, 3);
      digitalWrite(LED_PIN, LOW);
    })
    .onError([](ota_error_t error) {
      Log.println("OTA failed!");
      flash_led(100, 20);
      delay(5000);
      ESP.restart();
    });
    ArduinoOTA.begin();
}

char payload_buffer[2048];

char* to_null_terminated(byte* bytes, unsigned int length) {
  if (length >= 2048) {
    return (char*)0;
  }
  memcpy(payload_buffer, bytes, length);
  payload_buffer[length] = '\0';
  return payload_buffer;
}

void print_addr(DeviceAddress addr) {
  Log.print("0x");
  for (int i = 0; i < 8; i++) {
   String s(addr[i], HEX);
    Log.print(s);
  }
  Log.println();
}

void save_dallas_sensor(int index) {
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(500);
  dallas_temp.requestTemperatures();
  DeviceAddress addr;
  dallas_temp.getAddress(addr, 0);
  Log.print("Found address: ");
  print_addr(addr);
  memcpy(config.dallas_temp_addr[index], addr, sizeof(DeviceAddress));
  save_config();
}

void save_moisture_adc(int channel, bool wet) {
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(500);
  int adc = take_moisture_reading(MOISTURE_PINS[channel], 200);
  Log.print("Saving adc: ");
  Log.println(String(adc));
  if (wet) {
    config.wet_adc[channel] = adc;
  } else {
    config.dry_adc[channel] = adc;
  }
  save_config();
}

void set_channel_status(int id, bool status) {
    Log.print("Enabling channel: ");
    Log.println(String(id+1));
    config.channel_enabled[id] = status;
    pinMode(MOISTURE_PINS[id], INPUT);
    pinMode(SOLENOID_PINS[id], OUTPUT);
    save_config();
}

void set_solenoid_state(int channel) {
  if (channel != solenoid_state) {
    Log.printlnf("Setting solenoid state: %d", channel);
    mqtt.publish("autogarden/solenoid", (channel == -1 ? "off" : String(channel+1).c_str()));
    for (int i = 0; i < NUM_CHANNELS; i++) {
      ledcDetachPin(SOLENOID_PINS[i]);
    }
    solenoid_state = channel;
    solenoid_start_time = millis();
    if (channel != -1) {
      // Power solenoid at full voltage for 300ms to force it open
      ledcAttachPin(SOLENOID_PINS[channel], 0);
      ledcWrite(0, 255);
      delay(300);
      // Power solenoid at about 70% voltage to just keep it open
      ledcWrite(0, 180);
    }
  }
}

void set_pump_state(bool state) {
  if (state != pump) {
    Log.printlnf("Setting pump state: %s", state ? "on" : "off");
    pump = state;
    digitalWrite(SENSOR_POWER_PIN, pump);
    if (state == false) {
      set_solenoid_state(-1);
    }
  }
}

void check_solenoids() {
  // Need to have a timeout so solenoids don't get hot and melt their case!
  if (millis() - solenoid_start_time > SOLENOID_TIMEOUT && solenoid_state != -1) {
    Log.println("Solenoid on for too long, shutting off!");
    set_solenoid_state(-1);
    pump = false;
    digitalWrite(PUMP_PIN, LOW);
    mqtt.publish("autogarden/pump", "off", true);
  }
}

void mqtt_callback(char* topic, byte* payload_bytes, unsigned int length) {
  if (length >= 2048)
    return;
  // Convert payload to null-terminated
  static char payload[2048];
  memcpy(payload, payload_bytes, length);
  payload[length] = '\0';

  if (strcmp(topic, "autogarden/save_temp_address") == 0) {
    save_dallas_sensor(atoi(payload)-1);
  }
  else if (strcmp(topic, "autogarden/save_water_adc_wet") == 0) {
    save_moisture_adc(atoi(payload)-1, true);
  }
  else if (strcmp(topic, "autogarden/save_water_adc_dry") == 0) {
    save_moisture_adc(atoi(payload)-1, false);
  }
  else if (strcmp(topic, "autogarden/enable_channel") == 0) {
      set_channel_status(atoi(payload)-1, true);
  } else if (strcmp(topic, "autogarden/disable_channel") == 0) {
      set_channel_status(atoi(payload)-1, false);
  } else if (strcmp(topic, "autogarden/console/enable") == 0) {
      Log.send_mqtt = strcmp(payload, "on") == 0;
  }
  else if (strcmp(topic, "autogarden/pump") == 0) {
    if (strcmp(payload, "on") == 0) {
      if (solenoid_state == -1) {
        Log.println("Not starting pump, need to set solenoid state first!");
      } else {
        Log.println("Starting pump");
        set_pump_state(true);
      }
    } else {
      Log.println("Stopping pump");
      set_pump_state(false);
    }
  }
  else if (strcmp(topic, "autogarden/solenoid") == 0) {
    if (strcmp(payload, "off") == 0) {
      set_solenoid_state(-1);
    } else {
      int channel = atoi(payload)-1;
      if (channel < 0 || channel >= NUM_CHANNELS || !config.channel_enabled[channel]) {
        Log.printlnf("Not enabling solenoid at channel %d, that channel doesn't exist or isn't enabled.", channel+1);
      } else {
        set_solenoid_state(channel);
      }
    }
  }
}

void setup() {
  // Pump pin will be outputting pwm from boot, so turn it off first thing
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  
  Serial.begin(115200);
  Serial.setTimeout(120*1000);
  while (!Serial) {delay(10);}

  // For solenoid
  ledcSetup(0, 5000, 8);
  
  load_config();
  check_wifi();
  delay(100);
  Log.println("Rebooted, running setup!");
  
  setup_OTA();
  dallas_temp.begin();
    
  pinMode(WATER_SENSOR_PIN, INPUT_PULLDOWN);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Log.print("Enabled channels: ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (config.channel_enabled[i]) {
      Log.print(String(i+1));
      Log.print(" ");
      pinMode(MOISTURE_PINS[i], INPUT);
      pinMode(SOLENOID_PINS[i], OUTPUT);
      digitalWrite(SOLENOID_PINS[i], LOW);
    }
  }
  Log.println(" ");
  Log.println(String(sizeof(AutoGardenConfig)));
}

int compare(const void * a, const void * b) {
  return ( *(int*)a - *(int*)b );
}

float take_moisture_reading(int pin, int num_readings) {
  // ADC readings are kind of unreliable, so we need to average them
  // and remove the outliers
  
  static int samples[NUM_SAMPLES];
  float sum = 0;
  int avg_count = 0;
  for (int j = 0; j < num_readings; j++) {
    for (int i = 0; i < NUM_SAMPLES; i++) {
      samples[i] = analogRead(pin);
    }
    qsort(samples, NUM_SAMPLES, sizeof(int), compare);
    for (int i = NUM_SAMPLES*0.4; i < NUM_SAMPLES*0.6; i++) {
      sum += samples[i];
      avg_count++;
    }
  }
  return sum/avg_count;
}

void check_serial() {
  // For setting network secrets over serial for a first time setup
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "setup") {
      Serial.println("Enter ssid:");
      strcpy(config.ssid, Serial.readStringUntil('\n').c_str());
      Serial.println("Enter password:");
      strcpy(config.password, Serial.readStringUntil('\n').c_str());
      Serial.println("Enter mqtt server:");
      strcpy(config.mqtt_server, Serial.readStringUntil('\n').c_str());
      Serial.println("Enter OTA hostname:");
      strcpy(config.ota_hostname, Serial.readStringUntil('\n').c_str());
      Serial.println("Enter OTA password:");
      strcpy(config.ota_password, Serial.readStringUntil('\n').c_str());
      save_config();
    }
  }
}

void take_readings() {
  static unsigned long last_reading = millis();
  if (millis() - last_reading > SENSOR_READ_INTERVAL) {
    Log.printlnf("Solenoid state: %d", solenoid_state);
    
    last_reading = millis();
    Log.println("Taking readings...");
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(200);

    // Air temp/humidity sensor
    if (!bme.begin(0x76)) {
      Log.println("Unable to connect to bme temp/humidity sensor");
    }
    else {
      float air_temp = bme.readTemperature()*9.0/5.0 + 32;
      float air_humidity = bme.readHumidity();
      Log.printlnf("Air temp: %.2fF, Air humidity: %.2f%%", air_temp, air_humidity);
      mqtt.publish("autogarden/air_temp", String(air_temp).c_str());
      mqtt.publish("autogarden/air_humidity", String(air_humidity).c_str());
    }

    // Water level
    water_level = digitalRead(WATER_SENSOR_PIN);
    String water_level_str = water_level == HIGH ? "ok" : "low";
    Log.print("Water level: ");
    Log.println(water_level_str);
    mqtt.publish("autogarden/water_level", water_level_str.c_str());

    // Temp sensors
    dallas_temp.requestTemperatures();
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (config.channel_enabled[i]) {
        if (dallas_temp.isConnected(config.dallas_temp_addr[i])) {
          float temp = dallas_temp.getTempF(config.dallas_temp_addr[i]);
          String topic = "autogarden/soil_temp/" + String(i+1);
          mqtt.publish(topic.c_str(), String(temp).c_str());
          Log.printlnf("Temp channel %d - %.2f", i+1, temp);
        } else {
          Log.printlnf("Warning, temp channel %d disconnected", i+1);
        }
      }
    }
  
    // Moisture sensors
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (config.channel_enabled[i]) {
        int adc = take_moisture_reading(MOISTURE_PINS[i], 200);
        float percent = map(adc, config.dry_adc[i], config.wet_adc[i], 0, 10000)/100.0;
        String topic = "autogarden/soil_moist/" + String(i+1);
        mqtt.publish(topic.c_str(), String(percent).c_str());
        Log.printlnf("Mositure channel %d - %.2f, adc: %d", i+1, percent, adc);
      }
    }
    digitalWrite(SENSOR_POWER_PIN, LOW);
  }
}

void loop() {
  check_wifi();
  mqtt.loop();
  check_serial();
  ArduinoOTA.handle();
  take_readings();
  check_solenoids();
  if (pump) {
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(100);
    water_level = digitalRead(WATER_SENSOR_PIN);
    if (water_level == HIGH) {
      digitalWrite(PUMP_PIN, HIGH);
    }
    if (water_level == LOW) {
      Log.println("Low water level while running pump, stopping!");
      set_pump_state(false);
      mqtt.publish("autogarden/pump", "off", true);
    }
    digitalWrite(SENSOR_POWER_PIN, LOW);
  } else {
    digitalWrite(PUMP_PIN, LOW);
  }
  delay(100);
}
