#include <ArduinoJson.h>

#define BUZ 9
#define GREEN 8
#define led1 12

int pinMotion = 2;  // Input from PIR sensor //
int value = 0;
int pirState = LOW;

#define LM35 A3
float temp;
char val;


void setup() {

  Serial.begin(9600);

  pinMode(pinMotion, INPUT);
  pinMode(led, OUTPUT); //Led Pin//

  pinMode(BUZ, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(led1, OUTPUT);


//================================================================

void loop() {


  value = digitalRead(pinMotion);

  if (value == HIGH) {
    digitalWrite(led, HIGH);

    if (pirState == LOW) {
      Serial.println("PIR Motion Detected ");
      pirState = HIGH;
    }
  }
  else {
    digitalWrite(led, LOW);

    if (pirState == HIGH) {
      Serial.println("PIR Motion Ended ");
      pirState = LOW;
    }
  }

  //================================================================

  temp = analogRead(LM35);
  temp = (temp * 500) / 1023;
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" C");
  Serial.println();

  if (temp > 40) {
    digitalWrite(BUZ, HIGH);
    digitalWrite(GREEN, LOW);
  }
  else {
    digitalWrite(BUZ, LOW);
    digitalWrite(GREEN, HIGH);
  }


 //================================================================
 
  // Create the JSON document
  StaticJsonDocument<200> doc;
  doc["temparatue"] = temp;
  doc["pirState"] = pirState;

  // Send the JSON document over the "link" serial port
  serializeJson(doc, Serial);

}
