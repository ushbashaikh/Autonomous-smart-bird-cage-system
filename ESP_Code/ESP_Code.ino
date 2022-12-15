#define DEBUG_SW 1 // Make it 1 to see all debug messages in Serial Monitor

#include <WiFi.h>
#include <WiFiClient.h>
#include <FirebaseESP32.h>

#include <Servo.h>

#include <Wire.h>
#include "RTClib.h"

#include <ArduinoJson.h>

// Firebase Credentials
#define FIREBASE_HOST "autonomous-smart-cage-system-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "yZdzm9KoeU9EhXjokD8IoqDwjEwVkPqTH1PkmLst"

// WiFi Credentials
//#define WIFI_SSID "Maaz Ahmed Shaikh"
//#define WIFI_PASSWORD "87fqf54"

#define WIFI_SSID "Galaxy A30s"
#define WIFI_PASSWORD "ijre1328"

// Pins of Relay (Appliances Control)
#define R6 5

RTC_DS3231 rtc;
Servo servo;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Variable Declaration
int motion;
float temp;


//Functions Declaration
void Servo1();
void Data_from_firebase();


//Define FirebaseESP32 data object
FirebaseData firebaseData;
FirebaseJson json;

//Pins Declaration
#define RXp2 16
#define TXp2 17



void setup() {
  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);


  
  digitalWrite(R6, LOW);
  pinMode(R6, OUTPUT);
  pinMode(26, INPUT);
  servo.attach(14);

  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

   //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
 
 //=============================================================================================
   
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

   if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // (2014, 1, 21, 3, 0, 0) (Year, Month,Date, Hours, Min, Sec)
   //  rtc.adjust(DateTime(2022, 8, 21, 12, 45, 00));
  
  }

  
}
   
void loop() {
//=====================Communication Between Arduino and Node MCU=================================

    StaticJsonDocument<300> doc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, Serial2);

    if (err == DeserializationError::Ok) 
    { 
      Serial.print("temparatue = ");
      Serial.println(doc["temparatue"].as<long>());
      temp =  doc["temparatue"].as<long>();
      Firebase.setFloat( firebaseData,"Temperature:", temp );
       
      Serial.print("pirState = ");
      Serial.println(doc["pirState"].as<int>());
      motion = doc["pirState"].as<int>();
      Firebase.setInt( firebaseData,"MotionState:", motion);
    } 
    else 
    {
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());     
  }

 //===================Wifi Connection testing==========================================

  
  if (WiFi.status() != WL_CONNECTED)
  {
    if (DEBUG_SW) Serial.println("Not Connected");
   
  }

  else
  {
    if (DEBUG_SW) Serial.println(" Connected");
    Data_from_firebase();
  }

    servo.write(0);

 //=======================Feeding system Automatic====================================== 
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);

     if(now.hour()==6 && now.minute()==30 && now.second()==30 || now.hour()==17 && now.minute()==45  && now.second()==30)
  {
    Servo1();
  }

  else
  {
   servo.write(0);
  }

 //==============================Watering system Automatic=====================================================
 
  int SensorValue = digitalRead(26);  //take a sample
  if (SensorValue == 0) {             // Sensor in Water
    Serial.println("Sensor is not in the water or water level is low");
    digitalWrite(R6, HIGH);
  }

  else      //  Water level is low
  {
    Serial.println("Water container is full");
    digitalWrite(R6, LOW);
  } 
}

//=========================Firebase Function=====================================

void Data_from_firebase()
{
  
//=========================Watering through app==================================

    if (Firebase.getString(firebaseData, "/Pump")) {
      if (DEBUG_SW)  Serial.println(firebaseData.stringData());  
      if (firebaseData.stringData() == "0")
      {
         Serial.println("Relay2 - ------IF---------------");
         Serial.println(firebaseData.stringData());
         digitalWrite(R6, HIGH);
      }
      else
      {
        Serial.println("Relay2 - ------ELSE-----------------");
        Serial.println(firebaseData.stringData());
        digitalWrite(R6, LOW);
      }
    }
    
//=========================Feeding through app======================================

    if (Firebase.getString(firebaseData, "/Servo")) {
      if (DEBUG_SW)  Serial.println(firebaseData.stringData());
      if (firebaseData.stringData() == "1")
      {
        Serial.println("----------SERVO ----------IF----------------");
        Serial.println(firebaseData.stringData());
  
        servo.write(90);
        delay(20);
        servo.write(0);
      }
      }
      else
      {  Serial.println("----------SERVO ---------ELSE-------------");
   
        servo.write(0);
      }
    }
    
//=========================Servo1 Function===================================
   
void Servo1()
{
    servo.write(90); 
    Serial.print("Shutter Open");
    Serial.println();
    delay(200);
    servo.write(0);
    Serial.print("Shutter Close");
    Serial.println();
     
}
