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
*/

#include <ArduinoBLE.h>

// BLE Sensor Services
BLEService sensorService();
BLEService connectService("7e161f79-2ce9-46d1-917a-8e20f6b1f675");

// BLE Sensor Data Characteristic
BLEUnsignedCharCharacteristic sensorChar("175f0001-73f7-11ec-90d6-0242ac120003",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryChar("175f0002-73f7-11ec-90d6-0242ac120003",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

// BLE Connection Characteristic
BLEUnsignedCharCharacteristic connectChar("cc58110b-d173-4a9f-b0d5-1b0dd006c357",  // standard 16-bit characteristic UUID
    BLERead | BLEWrite | BLENotify); // remote clients will be able to get notifications if this characteristic changes

// Location of the sensor on the body (can be expanded)
// 0: Unspecified
// 1: Left Arm
// 2: Right Arm
// 3: Left Leg
// 4: Right leg
int location = 0;

// Battery global variables
int oldBatteryLevel = 0;
long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  Serial.begin(9600);    // initialize serial communication
  while(Serial.available() == 0) {}
  while (!Serial);

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
  BLE.setAdvertisedService(connectService); // add the service UUID
  connectService.addCharacteristic(connectChar); // add the connection characteristic
  BLE.addService(connectService); // Add the connection service
  connectChar.writeValue(location); // set initial value for this characteristic

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

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        // updateBatteryLevel();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

// void updateBatteryLevel() {
//   /* Read the current voltage level on the A0 analog input pin.
//      This is used here to simulate the charge level of a battery.
//   */
//   int battery = analogRead(A0);
//   int batteryLevel = map(battery, 0, 1023, 0, 100);

//   if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
//     Serial.print("Battery Level % is now: "); // print it
//     Serial.println(batteryLevel);
//     batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
//     oldBatteryLevel = batteryLevel;           // save the level for next comparison
//   }
// }