/*
  Blink using Serial communication

  Make sure to set Serial Monitor to line ending
*/

#define LED_PIN_RED 18 //verified
#define LED_PIN_GREEN 3 //verified
#define LED_PIN_BLUE 4

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
      digitalWrite(ledPin, LOW);
      Serial.print("Pin ");
      Serial.print(ledPin);
      Serial.println(" set to LOW");
    } else if (temp == 100) {
      digitalWrite(ledPin, HIGH);
      Serial.print("Pin ");
      Serial.print(ledPin);
      Serial.println(" set to HIGH");
    } else if (temp < 37 && temp > 0) {
      ledPin = temp;
      pinMode(ledPin, OUTPUT);
      Serial.print("LED Pin is ");
      Serial.println(ledPin);
    }
  }
}
