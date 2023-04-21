#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DHT.h"
#include "wiring_private.h"
#include <RTCZero.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#define tempPin 7                                                         // digital pin for temperature
#define lightPin A1                                                       // analog pin for photoresistor
#define moistPin A2                                                       // analog pin for moisture

#define DHTTYPE DHT11                                                     // assigns DHT Type the program is using
#define LIGHT 750                                                         // tentative value for shade
#define FULL 7000
#define SHADE 3000
#define DE 6                                                              // RX/TX digital pin

/* OBJECT INITIALIZATIONS */
BLEService newService("180A");                                            // creating the service
BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite);             // creating the LED characteristic

/*
BLEUnsignedCharCharacteristic pHdata("2A58", BLERead | BLENotify);
BLEUnsignedCharCharacteristic Tempdata("2A57", BLERead | BLENotify);
BLEUnsignedCharCharacteristic Moistdata("2A56", BLERead | BLENotify);
BLEUnsignedCharCharacteristic Lightdata("2A55", BLERead | BLENotify);
*/

Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);           // initialize Uart pins

RTCZero rtc;                                                              // initialize RTC object
DHT dht(tempPin, DHTTYPE);                                                // initialize DHT object

/* GLOBAL VARIABLE INITIALIZATIONS*/
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;

float acidVal = 0;
float tempVal = 0;              
float lightLev = 0;
float moistLev = 0.0;

unsigned long currentMillis = 0;                                           
unsigned long previousMillis = 0;

unsigned long upMillis = 0;
unsigned long downMillis = 0;
unsigned long highDuration = 0;
unsigned long upTime = 0;

volatile bool alarmFlag = false;                                          // initialize as false

byte values[10];
byte send = 0;

const byte pH[] = {0x01, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x15, 0xC9};       // dependent on sensor

/* FUNCTION PROTOTYPES */
void SERCOM3_Handler();                                                   // attach the interrupt handler to the SERCOM
void alarmMatch();

void temp_sensor();
void light_sensor();
void moist_sensor();
void pH_sensor();

/* SETUP FUNCTION */
void setup() {
  Serial.begin(9600);  
  
  dht.begin();
  rtc.begin(); 
  
  pinMode(DE, OUTPUT);

  /* SETUP: RTC */
  rtc.setTime(hours, minutes, seconds);
  rtc.setAlarmTime(0, 1, 10);                                             // testing for 1 minute; set 8 hour alarm
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);

  /* SETUP: UART */
  mySerial.begin(9600);
  pinPeripheral(1, PIO_SERCOM);                                           // assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM);                                           // assign TX function to pin 0
 
  /* SETUP: BLUETOOTH */
  while (!Serial);                                                        // starts the program if we open the serial monitor.

  pinMode(LED_BUILTIN, OUTPUT);                                           // initialize the built-in LED pin to indicate when a central is connected

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("MKR WiFi 1010");                                      // name that will appear when scanning for Bluetooth® devices
  BLE.setAdvertisedService(newService);
  newService.addCharacteristic(switchChar);                               // add characteristics to a service
  newService.addCharacteristic(randomReading);
  /*
  newService.addCharacteristic(Tempdata); 
  newService.addCharacteristic(pHdata);
  newService.addCharacteristic(Moistdata); 
  newService.addCharacteristic(Lightdata);
  */
  BLE.addService(newService);                                             // adding the service
  switchChar.writeValue(0);                                               // set initial value for characteristics
  randomReading.writeValue(0);
  /*
  Tempdata.writeValue(0); 
  pHdata.writeValue(0);
  Moistdata.writeValue(0);
  Lightdata.writeValue(0);
  */
  BLE.advertise();                                                        // start advertising the service
  Serial.println("Bluetooth® device active, waiting for connections...");

  /* SETUP: MODBUS */
  if (!ModbusRTUClient.begin(9600)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }

  delay (2000);                                                           // give time to measure
}

/* LOOP FUNCTION */
void loop() {
  BLEDevice central = BLE.central();                                      // wait for a Bluetooth® Low Energy central

  if (central) {                                                          // if a central is connected to the peripheral
    Serial.print("Connected to central: ");
    Serial.println(central.address());                                    // print the central's BT address
    digitalWrite(LED_BUILTIN, HIGH);                                      // turn on the LED to indicate the connection

    while (central.connected()) {                                         // while the central is connected:

      currentMillis = millis();

      if (currentMillis - previousMillis >= 2000) {                       // print every 200ms

        previousMillis = currentMillis;

        Serial.println(currentMillis);                                    // check time; testing only 

        //temp_sensor();
        light_sensor();
        //moist_sensor();
        //pH_sensor();

        /*
        float temp = readTemperature();
        Tempdata.writeValue(temp);
        float acid = readpH();
        pHdata.writeValue(acid);
        float moist = readMoisture();
        Moistdata.writeValue(moist);
        float light = readLight();
        Lightdata.writeValue(light);
        */

        Serial.println("");        
      }
    }     

    digitalWrite(LED_BUILTIN, LOW);                                       // when the central disconnects, turn off the LED
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

/* FUNCTION DEFINITIONS */
void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}

void alarmMatch()
{
  Serial.println("End of Light Monitoring");                              // test message
  alarmFlag = true;
}

void temp_sensor() {
  tempVal = dht.readTemperature(true);                                    // set true to measure in farenheit
  Serial.print("Temperature = ");
  Serial.print(tempVal);
  Serial.print(char(176));
  Serial.println("F");
}

void moist_sensor() {
  moistLev = analogRead(moistPin);
  Serial.println(moistLev);
  moistLev = map(moistLev, 1023, 0, 0, 100);                              // inverted to properly show humidity

  Serial.print("Moisture: ");                                             // test in serial monitor
  Serial.print(moistLev); Serial.println("%"); 
}

void light_sensor() {
  //highDuration = 0;
  lightLev = analogRead(lightPin);                       
  Serial.print("Light: ");                                                // test in serial monitor
  Serial.println(lightLev);

  /*
  float light = 0.;
  int lightval = analogRead(photo);
  light = lightval/10.23;
  return light;
  */

  if (alarmFlag == false) {                                               // while flag is false, continue to monitor; alarmFlag is set true after 8 hours
    if((lightLev > LIGHT) & (upMillis == 0)) {                            // count time only when light is high and when there's no record yet
      upMillis = millis(); 
    }
    if(lightLev < LIGHT) {
      downMillis = millis();                                              // count time when light is low; will keep updating 
    }

    /* TEST MESSAGES
    Serial.print("Start: ");
    Serial.println(upMillis);
    Serial.print("End: ");
    Serial.println(downMillis); 
    */

    if ((upMillis>0)&(downMillis>upMillis)) {                             // make sure light is high then low at least before calculation
      highDuration = downMillis - upMillis;                               // will depend on the duration set on loop; may move millis functions
      //Serial.print("Duration: ");
      //Serial.println(highDuration);
      upMillis = 0;                                                       // make sure previous values aren't used again
      downMillis = 0;
    }
    
    upTime += highDuration;                                               // used separate variable; using highDuration sometimes give huge number
    Serial.print("Uptime: ");
    Serial.println(upTime);

    highDuration = 0;                                                     // reset to 0 so it doesn't update upTime with the same value
  }else {                                                                 // after set duration (8 hours)
    Serial.print("Light: ");
    if(upTime > FULL) {
      Serial.println("FULL");
    }else if((upTime < FULL) & (upTime > SHADE)) {
      Serial.println("PARTIAL");
    }else {
      Serial.println("SHADE");
    }
  }
}

void pH_sensor() {
    float temp = 0.;                                                      // 0x000d is the register address for reading pH sensor
  if (!ModbusRTUClient.requestFrom(0x01, HOLDING_REGISTERS, 0x000d, 1)) {
    Serial.print("failed to read current! ");
    Serial.println(ModbusRTUClient.lastError());
  }else{
    uint16_t word1 = ModbusRTUClient.read();
    Serial.print("Word1: ");
    Serial.println(word1);
    temp = word1/10.0;
    Serial.print("temp: ");
    Serial.println(temp);    
  }  
}

/*
float readpH()
{
  float acid = 0.;
  if (!ModbusRTUClient.requestFrom(0x01, HOLDING_REGISTERS, acidity, 1)) {
    Serial.print("failed to read value: ");
    Serial.println(ModbusRTUClient.lastError());
  }else{
    uint16_t word1 = ModbusRTUClient.read();
    acid = word1/10.0;   
  }  
  return acid;
}
*/

/*
void pH_sensor() {
  digitalWrite(DE, HIGH);                                                 // sets the max485 to send data

  delay(10);

  for (int j = 0; j < 8; j++) {
    send = pH[j];                                                         // send request to the sensor
    mySerial.write(send);
    //Serial.print(send, HEX);
  }

  delay(10);

  digitalWrite(DE, LOW);                                                  // sets the max485 to recieve data

  while (mySerial.available()) {    
    Serial.println("Receive data: ");

    for (byte i = 0; i < 7; i++) {                                        // may use ten to over read extra data in register
      values[i] = mySerial.read();
      Serial.print(values[i], HEX);
    }
  }
  acidVal = float ((values[3] << 8) | (values[4])) / 10;
  Serial.print("Acidity: ");
  Serial.println(acidVal);
}
*/