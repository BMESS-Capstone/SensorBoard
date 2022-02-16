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
  - Sparkfun AS7265x Spectral Triade (SCL/SDA Conncetion)

  Interacts with the ESP32 BLE client firmware

  Author: Khaled Elmalawany
*/

#include "C:\Users\elmal\Documents\GitHub\Algorithm-Data\arduinoProjects\parameters.h"

// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>

//#include <Wire.h>
//#include <SparkFun_AS7265X.h>
//AS7265X sensor;
//float counter = 0.0;

int location = LEFT_ARM;

// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);

// BLE Service (NOTE: Consider moving into setup() to reduce dynamic memory)
BLEService sensorService = BLEService(CONNECT_UUID);

// BLE Sensor Data Characteristic
BLECharacteristic sensorChar = BLECharacteristic(SENSOR_CHAR_UUID,  // standard 16-bit characteristic UUID
                                                  BLERead | BLENotify,  // remote clients will be able to get notifications if this characteristic changes
                                                  sizeof(float)*SENSOR_DATA_LENGTH);


// BLE Battery Level Characteristic
BLEIntCharacteristic batteryChar = BLEIntCharacteristic(BATTERY_CHAR_UUID,  // standard 16-bit characteristic UUID
                                                         BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

//Sensor global variables
float sensorValue[SENSOR_DATA_LENGTH];

// Battery global variables
int oldBatteryLevel = 0;
long previousMillis = 0;

void setup() {
  pinMode(LED_RED_PIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

//  if (sensor.begin() == false)
//  {
//    while (1) {
//      digitalWrite(LED_RED_PIN, LOW);
//      delay(100);
//      digitalWrite(LED_RED_PIN, HIGH);
//      delay(100);
//    }
//  }
//
//  //Once the sensor is started we can increase the I2C speed
//  Wire.setClock(400000);
//
//  // Note: AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT is the default
//  sensor.setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS); //All 6 channels on all devices
//
//  //0-255 valid but 0 seems to cause the sensors to read very slowly
//  //(1+1)*2.8ms = 5.6ms per reading
//  //But we need two integration cycles so 89Hz is aproximately the fastest read rate
//  sensor.setIntegrationCycles(1);
//
//  //Only turn on IR bulb
//  //IR on board is rated at 875 nm while standalone IR is rate at 940 nm
//  sensor.enableBulb(AS7265x_LED_IR);


  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  blePeripheral.setLocalName("pIRfusiX sensor");

  blePeripheral.setAdvertisedServiceUuid(sensorService.uuid()); // Flage service UUID to be advertised

  blePeripheral.addAttribute(sensorService); // add the sensor service
  blePeripheral.addAttribute(sensorChar); // add the sensor characteristic
  blePeripheral.addAttribute(batteryChar); // add the battery characteristic
  
  batteryChar.setValue(location);

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  // begin initialization
  blePeripheral.begin();
}

void loop() {
  // wait for a BLE central
  BLECentral central = blePeripheral.central();

  // if a central is connected to the peripheral:
  if (central) {
    // turn on the LED to indicate the connection:
    digitalWrite(LED_RED_PIN, HIGH);
    previousMillis = millis();

    // while the central is connected:
    while (central.connected()) {
      //**********IR Sensor*****************
      if (sensorChar.subscribed()) {
        readSensor();
        sensorChar.setValue((const unsigned char*)&sensorValue, sizeof(float)*SENSOR_DATA_LENGTH);
      }

      //**********BATERY*****************
      // if some interval time has passed, check the battery level:
      if (batteryChar.subscribed() && millis() - previousMillis > BATTERY_INTERVAL_MS) {
        previousMillis = millis();
        updateBatteryLevel();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_RED_PIN, LOW);

    // Reset the location
    batteryChar.setValue(location);
  }
}

void updateBatteryLevel() {
  /* Read the current voltage level on the battery's analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(BATTERY_PIN);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  batteryChar.setValue(batteryLevel);  // and update the battery level characteristic
}

void readSensor() {
  delay(50);
  sensorValue[0] = millis();
  sensorValue[1] = millis();
//  //Wait two integration cycles to get all values
//  while (sensor.dataAvailable() == false) {} //Do nothing
//
//  sensorValue[0] = sensor.getCalibratedU();
//  sensorValue[1] = sensor.getCalibratedW();
}
