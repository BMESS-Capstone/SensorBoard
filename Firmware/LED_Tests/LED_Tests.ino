/*
  Blink using Serial communication

  Make sure to set Serial Monitor to line ending
  NOTE: This 4-pin RGP is common cahtode so LOW turns ON and HIGH turns OFF
*/

#define LED_PIN_RED 18 //verified
#define LED_PIN_GREEN 3 //verified
#define LED_PIN_BLUE 5 //verified
#define ON LOW
#define OFF HIGH

int ledPin;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  while (!Serial);
}

// the loop function runs over and over again forever
void loop() {
  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    int temp = Serial.parseInt();
    if (temp == 99) {
      digitalWrite(ledPin, OFF);
      Serial.print("Pin ");
      Serial.print(ledPin);
      Serial.println(" set to LOW");
    } else if (temp == 100) {
      digitalWrite(ledPin, ON);
      Serial.print("Pin ");
      Serial.print(ledPin);
      Serial.println(" set to HIGH");
    } else if (temp < 35 && temp > 0) {
      ledPin = temp;
      pinMode(ledPin, OUTPUT);
      Serial.print("LED Pin is ");
      Serial.println(ledPin);
    }
  }
}
