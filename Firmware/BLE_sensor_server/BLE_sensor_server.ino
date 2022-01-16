/*
  pIRfusiX sensor BLE firmware

  Service:
  - sensorService (service encapsulating various characteristics)
  - connectService (service to ensure the device is unique)

  Characterstics for sensorService:
  - sensorChar (sensor value output)
  - batteryChar (battery voltage remaining)

  Characterstics for connectService:
  - connectChar (used for connection algorithm)

  The circuit:
  - Arduino Nano 33 BLE or custom pIRfusiX PCB board

  Interacts with the ESP32 BLE client firmware

  Author: Khaled Elmalawany
*/

#include <ArduinoBLE.h>

//*****Shared with NANO 33 TODO: move to a common refererred .h file***
// Parameters only for Sensor
#define BATTERY_INTERVAL_MS 2000

// Parameters only for Gateway
#define ONBOARD_LED 2

// Shared parameters
#define TOTAL_POSSIBLE_LOCATIONS 4
#define LEFT_ARM 0
#define RIGHT_ARM 1
#define LEFT_LEG 2
#define RIGHT_LEG 3

#define CONNECT_UUID "6164e702-7565-11ec-90d6-0242ac120003"

#define SENSOR_CHAR_UUID "fec40b26-757a-11ec-90d6-0242ac120003"
#define BATTERY_CHAR_UUID "fec40dc4-757a-11ec-90d6-0242ac120003"
//********************************************************************
uint8_t location = LEFT_ARM;

// BLE Service (NOTE: Consider moving into setup() to reduce dynamic memory)
BLEService sensorService(CONNECT_UUID);

// BLE Sensor Data Characteristic
BLEFloatCharacteristic sensorChar(SENSOR_CHAR_UUID,  // standard 16-bit characteristic UUID
                                  BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


// BLE Battery Level Characteristic
BLEIntCharacteristic batteryChar(BATTERY_CHAR_UUID,  // standard 16-bit characteristic UUID
                                 BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

// Battery global variables
int oldBatteryLevel = 0;
long previousMillis = 0;

void setup() {
  Serial.begin(9600);    // initialize serial communication

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("pIRfusiX sensor");
  BLE.setAdvertisedService(sensorService); // add the service UUID
  sensorService.addCharacteristic(sensorChar); // add the sensor characteristic
  sensorService.addCharacteristic(batteryChar); // add the battery characteristic
  BLE.addService(sensorService); // Add the connection service
  sensorChar.writeValue(float(location));

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    previousMillis = millis();

    // while the central is connected:
    while (central.connected()) {
      //**********IR Sensor*****************
      if (sensorChar.subscribed()) {
        sensorChar.writeValue(readSensor());
      }

      //**********BATERY*****************
      // if some interval time has passed, check the battery level:
      if (batteryChar.subscribed() && millis() - previousMillis > BATTERY_INTERVAL_MS) {
        previousMillis = millis();
        updateBatteryLevel();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());

    // Reset the location
    sensorChar.writeValue(float(location));
  }
}

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryChar.writeValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}

float readSensor() {
  // TODO: Update to proper code
  return float(millis());
}
