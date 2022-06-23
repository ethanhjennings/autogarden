#include <Wire.h>
#include "SparkFun_Qwiic_Relay.h"

#define IS_SINGLE_RELAY true // Change to false if using Quad Relay

#define SDA_PIN 32
#define SCL_PIN 33

bool scan_i2c() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    return false;
  } else {
    Serial.println("done\n");
    return true;
  }
}

void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(1000);
}

byte readAddress() {
  while (Serial.available() == 0) {}
  return strtol(Serial.readString().c_str(), 0, 16);
}

void loop()
{
  bool any_found = scan_i2c();
  if (any_found) {
    Serial.println("Enter old address: (enter hex without 0x)");
    byte old_addr = readAddress();
    Serial.print("old address = 0x");
    Serial.println(old_addr, HEX);
    Serial.println("Enter new address: (enter hex without 0x)");
    byte new_addr = readAddress();
    Serial.print("new address = 0x");
    Serial.println(new_addr, HEX);

    delay(1000);
  
    Qwiic_Relay relay(old_addr);
    
    if(relay.begin())
      Serial.println("Connected!");
    else
      Serial.println("Check connections to Qwiic Relay.");
  
    if(relay.changeAddress(new_addr, IS_SINGLE_RELAY))
      Serial.println("Address changed successfully."); 
    else
      Serial.println("Address change failed...");
  }
  delay(2000);
}
