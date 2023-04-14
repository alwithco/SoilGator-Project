#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DHT.h"
#include "wiring_private.h"
#include "wiring_private.h"

BLEService newService("180A");                                            // creating the service
BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite);             // creating the LED characteristic

Uart mySerial (&sercom3, 8, 9, SERCOM_RX_PAD_1, UART_TX_PAD_0);           // rx8 tx9

void SERCOM3_Handler() {
  mySerial.IrqHandler();
}

#define tempPin 0
#define lightPin A3                     // analog pin for photoresistor
#define moistPin A2                     // analog pin for moisture

#define DE 1

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
  mySerial.begin(9600); 
  
  ///// DHT11 Temp /////
  dht.begin();

  ///// pH Sensor /////
  pinMode(DE, OUTPUT);
  pinPeripheral(4, PIO_SERCOM); //Assign RX function to pin 8
  pinPeripheral(5, PIO_SERCOM); //Assign TX function to pin 9
    
    ///// Bluetooth /////
  while (!Serial);                                                          // starts the program if we open the serial monitor.
  pinMode(LED_BUILTIN, OUTPUT);                                             // initialize the built-in LED pin to indicate when a central is connected
  pinMode(ledPin, OUTPUT);                                                  // initialize the built-in LED pin to indicate when a central is connected
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }
  BLE.setLocalName("SoilGator");                                        // Setting a name that will appear when scanning for Bluetooth® devices
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

      if (currentMillis - previousMillis >= 3000) {                          // print every 200ms
        previousMillis = currentMillis;
        Temp();
        Light();
        Moist();
      }
    }     

    digitalWrite(LED_BUILTIN, LOW);                                        // when the central disconnects, turn off the LED
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

///// CALL FUNCTIONS /////


void pH_sensor() {

  digitalWrite(DE, HIGH);//Sets the max485 to send data
  delay(10);

    for (int j = 0; j < 8; j++)
    {
      send = pH[j];
      mySerial.write(send);
      Serial.print(send, HEX);
    }
    delay(10);
    digitalWrite(DE, LOW);//Sets the max485 to recieve data

    while (mySerial.available()) 
    {    
      Serial.println("Receive data:");
      for (byte i = 0; i < 7; i++)//Use ten to over read extra data in register
        {
          values[i] = mySerial.read();
          Serial.print(values[i], HEX);
        }
      Serial.println();
    }
  float acidval = ((values[3] << 8) | (values[4])) / 10;
  Serial.print(" pH: ");
  Serial.println(acidval);
  delay(3000);
}
void Temp() {
  tempVal = dht.readTemperature(true);
  Serial.print("Temperature = ");
  Serial.print(tempVal);
  Serial.print(char(176)); 
  Serial.println("F");
}

void Light() {
  lightLev = analogRead(lightPin);
  Serial.print("Light: ");                      // test in serial monitor
  Serial.println(lightLev);
  randomReading.writeValue(lightLev);          // look at 2A58
}

void Moist() {
  moistLev = analogRead(moistPin);
  moistLev = map(moistLev, 0, 1023, 0, 100);
  Serial.print("Moisture: ");        // test in serial monitor
  Serial.print(moistLev);
  Serial.println("%"); 
  pH_sensor();
  Serial.println();
}

