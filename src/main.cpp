#include <Arduino.h>

#include "DHT.h"

#define acidPin A0                      // analog pin for pH
#define tempPin 2
#define lightPin A1                     // analog pin for photoresistor
#define moistPin A2

#define DHTTYPE DHT11                   // DHT Type using
#define LIGHT 750                       // tentative value for shade

DHT dht(tempPin, DHTTYPE);              // initialize DHT object

// Global variables
float acidVal = 0;
float tempVal = 0;                      // temperature value
float lightLev = 0;
float moistLev = 0;

unsigned long currentMills = 0;
unsigned long prevMills = 0;

void setup() {
  Serial.begin(9600);  
  Serial.println("Start");              // Test the serial monitor
  delay (2000);                         // give time to measure
}

void loop() {
  /////// pH SECTION ///////
  acidVal = analogRead(acidPin);
  Serial.print("Acidity: ");            // test in serial monitor
  Serial.println(acidVal);

  /////// TEMPERATURE SECTION ///////
  tempVal = dht.readTemperature(true);  // in Fahrenheit (true)

  /** Failed Case
   if(isnan(t)) {
    Serial.println("Failed Measurement...")
   }
  */

  Serial.print("Temperature: ");        // test in serial monitor
  Serial.println(tempVal);

  
  /////// LIGHT SECTION ///////
  currentMills = millis();
  Serial.print(currentMills);
  Serial.print("     Light Section");

  lightLev = analogRead(lightPin);

  /* start recording time when sun is up
  if(lightLev > LIGHT) {
    upMillis = millis();
    WE MAY NEED A DS1307 MODULE instead
  }
    downMillis = millis();
    float upTime = downMillis - upMillis;
    float totalTime = totalTime + upTime
  */

  /////// MOISTURE SECTION ///////
  moistLev = analogRead(moistPin);
  moistLev = map(moistLev, 0, 550, 0, 100);
  Serial.print("Moisture: ");        // test in serial monitor
  Serial.print(moistLev);
  Serial.println("%");
}