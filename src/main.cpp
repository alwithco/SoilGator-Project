#include <Arduino.h>
#include "DHT.h"

#define acidPin A0                      // analog pin for pH
#define tempPin 1
#define lightPin A1                     // analog pin for photoresistor
#define moistPin A2

#define RE 6
#define DE 7

#define DHTTYPE DHT11                   // DHT Type using
#define LIGHT 750                       // tentative value for shade

DHT dht(tempPin, DHTTYPE);              // initialize DHT object

// Global variables
float acidVal = 0;
float tempVal = 0;                      // temperature value
float lightLev = 0;
float moistLev = 0;

byte values[10];

const byte pH[] = {0x01, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x15, 0xC9}; //Different depending on sensor

unsigned long currentMills = 0;
unsigned long prevMills = 0;

void setup() {
  Serial.begin(9600);  
  Serial.println("Start");              // Test the serial monitor
  
  dht.begin();

  //mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  delay (2000);                         // give time to measure
}

void loop() {
  delay (2000);

  currentMills = millis();
  Serial.println(currentMills);
  Serial.println("     Start");

  /////// pH SECTION ///////
  /*
  acidVal = analogRead(acidPin);
  Serial.print("Acidity: ");            // test in serial monitor
  Serial.println(acidVal);
  */

  /////// TEMPERATURE SECTION //////

  /** Failed Case
   if(isnan(t)) {
    Serial.println("Failed Measurement...")
   }
  */

  tempVal = dht.readTemperature(true);  // in Fahrenheit (true)

  Serial.print("Temperature: ");        // test in serial monitor
  Serial.print(tempVal);
  Serial.println("F");

  
  /////// LIGHT SECTION ///////
  lightLev = analogRead(lightPin);
  Serial.print("Light: ");        // test in serial monitor
  Serial.println(lightLev);
 
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
  Serial.println("END");
}

// serial read and print what's in the transmission
/**
float SensorQuerypH()
{
  float svalp;
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);

  if (mod.write(pH, sizeof(pH)) == 8)
  {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    for (byte i = 0; i < 10; i++)//Use ten to over read extra data in register
    {
      values[i] = mod.read();
    }
    svalp = (values[4])/10;
  }
  return svalp;
}
*/
