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
  - Custom pIRfusiX PCB board with Adafruit Feather nRF52840 Sense bootloader
  - Sparkfun AS7265x Spectral Triade (SCL/SDA Conncetion)

  Interacts with the ESP32 BLE client firmware

  Author: Khaled Elmalawany
*/

#include <bluefruit.h>
#include "C:\Users\elmal\Documents\GitHub\Algorithm-Data\arduinoProjects\parameters.h"

#include <Wire.h>

#include <SparkFun_AS7265X.h>
AS7265X sensor;
float counter = 0.0;

int location = LEFT_ARM;

// BLE Service (NOTE: Consider moving into setup() to reduce dynamic memory)
BLEService sensorService(CONNECT_UUID);

// BLE Sensor Data Characteristic
BLECharacteristic sensorChar(SENSOR_CHAR_UUID,  // standard 16-bit characteristic UUID
                             BLERead | BLENotify,  // remote clients will be able to get notifications if this characteristic changes
                             sizeof(float)*SENSOR_DATA_LENGTH);


// BLE Battery Level Characteristic
BLECharacteristic batteryChar(BATTERY_CHAR_UUID,  // standard 16-bit characteristic UUID
                                 BLERead | BLENotify, // remote clients will be able to get notifications if this characteristic changes
                                 sizeof(float)*1);

//Sensor global variables
float sensorValue[SENSOR_DATA_LENGTH];

// Battery global variables
int oldBatteryLevel = 0;
long previousMillis = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!Bluefruit.begin()) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  if (sensor.begin() == false)
  {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  //Once the sensor is started we can increase the I2C speed
  Wire.setClock(400000);

  // Note: AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT is the default
  sensor.setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS); //All 6 channels on all devices

  //0-255 valid but 0 seems to cause the sensors to read very slowly
  //(1+1)*2.8ms = 5.6ms per reading
  //But we need two integration cycles so 89Hz is aproximately the fastest read rate
  sensor.setIntegrationCycles(1);

  //Only turn on IR bulb
  //IR on board is rated at 875 nm while standalone IR is rate at 940 nm
  sensor.enableBulb(AS7265x_LED_IR);


  // Set the connect/disconnect callback handlers
  // Bluefruit.Periph.setConnectCallback(connect_callback);
  // Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("pIRfusiX sensor");
  sensorService.begin();
  // Notr: Calling .begin() on a BLECharacteristic will cause it to be added to the last BLEService that was 'begin()'ed!
  batteryChar.begin();  // add the sensor characteristic
  sensorChar.begin(); // add the battery characteristic
  batteryChar.write32(location); // Set the initial value of battery to respective location

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  // Set advertising interval (in unit of 0.625ms):
  Bluefruit.Advertising.setInterval(32, 244);
  // number of seconds in fast mode:
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);  
}

void loop() {
  // // wait for a BLE central
  // BLEDevice central = Bluefruit.central();

  // // if a central is connected to the peripheral:
  // if (central) {
  //   // turn on the LED to indicate the connection:
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   previousMillis = millis();

  //   // while the central is connected:
  //   while (central.connected()) {
  //     //**********IR Sensor*****************
  //     if (sensorChar.subscribed()) {
  //       readSensor();
  //       sensorChar.notify(sensorValue, sizeof(float)*SENSOR_DATA_LENGTH);
  //     }

  //     //**********BATERY*****************
  //     // if some interval time has passed, check the battery level:
  //     if (batteryChar.subscribed() && millis() - previousMillis > BATTERY_INTERVAL_MS) {
  //       previousMillis = millis();
  //       updateBatteryLevel();
  //     }
  //   }
  //   // when the central disconnects, turn off the LED:
  //   digitalWrite(LED_BUILTIN, LOW);

  //   // Reset the location
  //   batteryChar.write32(location);
  // }
}

void updateBatteryLevel() {
  /* Read the current voltage level on the battery's analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(BATTERY_PIN);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  batteryChar.write32(batteryLevel);  // and update the battery level characteristic
}

void readSensor() {
  //Wait two integration cycles to get all values
  while (sensor.dataAvailable() == false) {} //Do nothing

  sensorValue[0] = sensor.getCalibratedU();
  sensorValue[1] = sensor.getCalibratedW();
}
