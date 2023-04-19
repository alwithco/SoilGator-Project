#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DHT.h"
#include "wiring_private.h"
#include <RTCZero.h>

#define tempPin 7
#define lightPin A1                     // analog pin for photoresistor
#define moistPin A2                     // analog pin for moisture

#define DHTTYPE DHT11                   // DHT Type using
#define LIGHT 750                       // tentative value for shade

#define DE 6                            // Direction digital pin

// INITIALIZATIONS //
BLEService newService("180A");                                            // creating the service
BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite);             // creating the LED characteristic

Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);           // rx8 tx9

RTCZero rtc;
DHT dht(tempPin, DHTTYPE);              // initialize DHT object

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;

// Global variables //
float acidVal = 0;
float tempVal = 0;                      // temperature value
float lightLev = 0;
float moistLev = 0;

// Testing variables //
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
long upMillis = 0;
long downMillis = 0;
long upTime = 0;
long totalTime = 0;
const int ledPin = 2;

volatile bool alarmFlag = true;

byte values[10];
byte send = 0;

const byte pH[] = {0x01, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x15, 0xC9}; //Different depending on sensor

void SERCOM3_Handler();
void alarmMatch();

void temp_sensor();
void light_sensor();
void moist_sensor();
void pH_sensor();

void setup() {
  Serial.begin(9600);  
  Serial.println("Start");              // Test the serial monitor
  
  rtc.begin(); 
  dht.begin();
 
  pinMode(DE, OUTPUT);

  rtc.setTime(hours, minutes, seconds);
  //rtc.setDate(day, month, year);

  rtc.setAlarmTime(0, 1, 10);                                               // testing for 1 minute; set 8 hour alarm
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  
  rtc.attachInterrupt(alarmMatch);

  mySerial.begin(9600);
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 8
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 9

  while (!Serial);                                                          // starts the program if we open the serial monitor.

  pinMode(LED_BUILTIN, OUTPUT);                                             // initialize the built-in LED pin to indicate when a central is connected
  pinMode(ledPin, OUTPUT);                                                  // initialize the built-in LED pin to indicate when a central is connected

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("MKR WiFi 1010n");                                        // Setting a name that will appear when scanning for Bluetooth® devices
  BLE.setAdvertisedService(newService);

  newService.addCharacteristic(switchChar);                                 // add characteristics to a service
  newService.addCharacteristic(randomReading);

  BLE.addService(newService);                                               // adding the service

  switchChar.writeValue(0);                                                 // set initial value for characteristics
  randomReading.writeValue(0);

  BLE.advertise();                                                          // start advertising the service
  Serial.println("Bluetooth® device active, waiting for connections...");

  delay (2000);                                                             // give time to measure
}
////////////////////////////
void loop() {
  BLEDevice central = BLE.central();                                        // wait for a Bluetooth® Low Energy central

  if (central) {                                                            // if a central is connected to the peripheral
    Serial.print("Connected to central: ");
    Serial.println(central.address());                                      // print the central's BT address
    digitalWrite(LED_BUILTIN, HIGH);                                        // turn on the LED to indicate the connection

    while (central.connected()) {                                           // while the central is connected:

      currentMillis = millis();

      if (currentMillis - previousMillis >= 2000) {                          // print every 200ms

        previousMillis = currentMillis;

        Serial.println(currentMillis);                                      // check time 

        temp_sensor();
        light_sensor();
        moist_sensor();
        //pH_sensor();

        Serial.println("");        
      }
    }     

    digitalWrite(LED_BUILTIN, LOW);                                        // when the central disconnects, turn off the LED
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
///////////////////////////////////////////////////////////////////////////
void alarmMatch()
{
  Serial.println("Stop Light Monitoring");
  alarmFlag = true;
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}

void temp_sensor() {
  tempVal = dht.readTemperature(true);
  Serial.print("Temperature = ");
  Serial.print(tempVal);
  Serial.print(char(176));
  Serial.println("F");
}

void moist_sensor() {
  moistLev = analogRead(moistPin);
  moistLev = map(moistLev, 1023, 0, 0, 100);
  Serial.print("Moisture: ");        // test in serial monitor
  Serial.print(moistLev);
  Serial.println("%"); 
}

void light_sensor() {
  //lightLev = analogRead(lightPin);

  int lightST = 0;
    lightLev = analogRead(lightPin);  
                        
    Serial.print("Light: ");                                          // test in serial monitor
    Serial.println(lightLev);

  while (alarmFlag) {                                                    // alarmFlag is set after 8 hours

    if(lightLev > LIGHT) {                                            // count time when light is high
      upMillis = millis();
    }
    downMillis = millis();                                            // read time duration since start when light is low
    upTime = upTime + (downMillis - upMillis);

    lightST = 1;                                                      // set to 1 after 8 hours  
  }
    // After 8 hours
  if(lightST == 1) {
    Serial.print("Light: ");
    if(upTime > 2500) {
      Serial.println("FULL");
    }else if((upTime < 2500) & (upTime > 1000)) {
      Serial.println("PARTIAL");
    }else {
      Serial.println("SHADE");
    }
  }
}

void pH_sensor() {
  digitalWrite(DE, HIGH);                                                   // Sets the max485 to send data

  delay(10);

  for (int j = 0; j < 8; j++) {
    send = pH[j];                                                       // send request to the sensor
    mySerial.write(send);
    //Serial.print(send, HEX);
  }

  delay(10);

  digitalWrite(DE, LOW);                                                // Sets the max485 to recieve data

  while (mySerial.available()) {    
    Serial.println("Receive data: ");

    for (byte i = 0; i < 7; i++) {                                     // Use ten to over read extra data in register
      values[i] = mySerial.read();
      Serial.print(values[i], HEX);
    }
  }
  acidVal = float ((values[3] << 8) | (values[4])) / 10;
  Serial.print("Acidity: ");
  Serial.println(acidVal);
}