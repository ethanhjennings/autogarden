#include <Wire.h>
#include "SparkFun_Qwiic_Relay.h"

// All available product addresses labeled below. Close the onboard jumpers if
// you want to access the alternate addresses. 
#define SINGLE_DEFAULT_ADDR 0x10 // Alternate jumper address 0x19
#define QUAD_DEFAULT_ADDR 0x6D // Alternate jumper address 0x6C

#define IS_SINGLE_RELAY true // Change to true if using Single Relay

int SDA_PIN = 32;
int SCL_PIN = 33;

// After changing the address you'll need to apply that address to a new
// instance of the Qwiic_Relay class: "Qwiic_Relay relay(YOUR_NEW_ADDRESS_HERE)".

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
  
    if(relay.changeAddress(new_addr, true))
      Serial.println("Address changed successfully."); 
    else
      Serial.println("Address change failed...");
  }
  delay(2000);
}
