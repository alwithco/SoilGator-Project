#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DHT.h"
#include "wiring_private.h"

BLEService newService("180A");                                            // creating the service
BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite);             // creating the LED characteristic

Uart mySerial (&sercom3, 8, 9, SERCOM_RX_PAD_1, UART_TX_PAD_0);           // rx8 tx9

void SERCOM3_Handler();

#define acidPin A0                      // analog pin for pH
#define tempPin 1
#define lightPin A1                     // analog pin for photoresistor
#define moistPin A2                     // analog pin for moisture

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

// Testing variables
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
long upMillis = 0;
long downMillis = 0;
long upTime = 0;
long totalTime = 0;
const int ledPin = 2;

byte values[10];
byte send = 0;

const byte pH[] = {0x01, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x15, 0xC9}; //Different depending on sensor

void setup() {
  Serial.begin(9600);  
  Serial.println("Start");              // Test the serial monitor
  
  dht.begin();

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  mySerial.begin(9600);
  pinPeripheral(8, PIO_SERCOM); //Assign RX function to pin 8
  pinPeripheral(9, PIO_SERCOM); //Assign TX function to pin 9

  while (!Serial);                                                          // starts the program if we open the serial monitor.

  pinMode(LED_BUILTIN, OUTPUT);                                             // initialize the built-in LED pin to indicate when a central is connected
  pinMode(ledPin, OUTPUT);                                                  // initialize the built-in LED pin to indicate when a central is connected

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("MKR WiFi 1010");                                        // Setting a name that will appear when scanning for Bluetooth® devices
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

        /////// ACIDITY ///////
        //acidVal = analogRead(acidPin);
        //Serial.print("Acidity: ");            // test in serial monitor
        //Serial.println(acidVal);
        digitalWrite(DE, HIGH);                                                   // Sets the max485 to send data
        digitalWrite(RE, HIGH);                                                   // Sets the max485 to send data

        delay(10);

        for (int j = 0; j < 8; j++) {
          send = pH[j];                                         // 
          mySerial.write(send);
          Serial.print(send, HEX);
        }

        delay(10);

        digitalWrite(DE, LOW);//Sets the max485 to recieve data
        digitalWrite(RE, LOW);//Sets the max485 to recieve data

        while (mySerial.available()) {    
          Serial.println("Receive data:");

          for (byte i = 0; i < 7; i++) {                                        //Use ten to over read extra data in register
            values[i] = mySerial.read();
            Serial.print(values[i], HEX);
          }
        Serial.println();
    }

  acidVal = ((values[3] << 8) | (values[4])) / 10;
  Serial.print("pH: ");
  Serial.println(acidVal);

        Serial.println(currentMillis);                                      // check time 

        /////// TEMPERATURE ///////
        tempVal = dht.readTemperature(true);
        Serial.print("Temperature = ");
        Serial.print(tempVal);
        Serial.print(char(176));
        Serial.println("F");
        //Serial.println(" \xB0""F");                                        // inline version

        /////// LIGHT ///////
        lightLev = analogRead(lightPin);
        Serial.print("Light: ");        // test in serial monitor
        Serial.println(lightLev);

        randomReading.writeValue(lightLev);                                  // look at 2A58

        if(lightLev > LIGHT) {
          upMillis = millis();
        }else{
          downMillis = millis();  
        }
        
        upTime = downMillis - upMillis;
        totalTime = totalTime + upTime;

        /////// MOISTURE ///////
        moistLev = analogRead(moistPin);
        //Serial.println(moistLev);                     // check what's output
        moistLev = map(moistLev, 0, 1023, 0, 100);
        Serial.print("Moisture: ");        // test in serial monitor
        Serial.print(moistLev);
        Serial.println("%"); 

        Serial.println("");
      }
    }     

    digitalWrite(LED_BUILTIN, LOW);                                        // when the central disconnects, turn off the LED
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
// Attach the interrupt handler to the SERCOM

  /** TEMPERATURE Failed Case
   if(isnan(t)) {
    Serial.println("Failed Measurement...")
   }
  */
  
  /////// LIGHT SECTION ///////
  /* start recording time when sun is up
  if(lightLev > LIGHT) {
    upMillis = millis();
    WE MAY NEED A DS1307 MODULE instead
  }
    downMillis = millis();
    float upTime = downMillis - upMillis;
    float totalTime = totalTime + upTime
  */


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
