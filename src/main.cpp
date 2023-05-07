#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DHT.h"
#include "wiring_private.h"
#include <RTCZero.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#define pinTemp 7                                                         // digital pin for temperature
#define pinLight A1                                                       // analog pin for photoresistor
#define pinMoist A2                                                       // analog pin for moisture
#define DE 6                                                              // RX/TX digital pin

#define DHTTYPE DHT11                                                     // assigns DHT Type the program is using
#define LIGHT 750                                                         // tentative value for shade
#define FULL 7000
#define SHADE 3000
#define DATASIZE 20

/* OBJECT INITIALIZATIONS */
BLEService newService("180A");                                            // create the service
BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite);             // creating the LED characteristic

BLEFloatCharacteristic dataTemp("2A58", BLERead);                         // create analog characteristics for each measurement to send
BLEFloatCharacteristic dataMoist("2A57", BLERead);
BLEFloatCharacteristic dataLight("2A56", BLERead);
BLEFloatCharacteristic dataAcid("2A55", BLERead);

Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);           // initialize Uart pins

RTCZero rtc;                                                              // initialize RTC object
DHT dht(pinTemp, DHTTYPE);                                                // initialize DHT object

/* GLOBAL VARIABLE INITIALIZATIONS*/
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 0;
const byte day = 0;
const byte month = 0;
const byte year = 0;

float valueTemp = 0;              
float valueMoist = 0;
float valueLight = 0;
float valueAcid = 0;

unsigned long currentMillis = 0;                                           
unsigned long previousMillis = 0;

unsigned long upMillis = 0;
unsigned long downMillis = 0;
unsigned long highDuration = 0;
unsigned long upTime = 0;

volatile bool alarmFlag = false;                                          // initialize as false

byte values[10];
byte send = 0;

const byte pH[] = {0x01, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x15, 0xC9};       // sensor registers

String plantName[DATASIZE] = {"Bugbane", "Carrots", "Corn", "Cucumber", "Daffodils", "Daisies", "Dayliyly", "Garlic", "Hollyhocks", "Lavender", "Lettuce", "Lily of the Valley", "Marigold", "Onion", "Peony", "Poppies", "Potatoes", "Rose", "Squash", "Sunflower"};
float minPH[DATASIZE] = {5, 5.5, 5.5, 5.5, 6, 6, 6, 5.5, 6, 6, 6, 7.1, 5.5, 6, 6, 7, 4.8, 6, 5.5, 6};
float maxPH[DATASIZE] = {6, 6.5, 7.5, 7, 6.5, 8, 8, 8, 8, 7.5, 7, 8, 7.5, 7, 7.5, 8, 6.5, 7, 7, 7.5};
int minTemp[DATASIZE] = {55, 50, 32, 75, 60, 70, 65, 32, 55, 68, 60, 60, 40, 55, 32, 50, 65, 60, 50, 70};

typedef struct {
  char name;
  float acidMin;
  float acidMax;
  int tempMin;
  int tempMax;
  char lightTag;                                                          // either f = full (>750); p = partial; s = shade (<400)
  char moistTag;                                                          // either m = moist (>30); p = partial; d = dry (<20)
} plantData;

plantData plant[20];                                                      // struct array with 20 plant info

/* FUNCTION PROTOTYPES */
void SERCOM3_Handler();                                                   // attach the interrupt handler to the SERCOM
void alarmMatch();

void setPlantInfo();
float readTemp();
float readMoist();
float readLight();
float readAcid();

/* SETUP FUNCTION */
void setup() {
  Serial.begin(9600);  
  
  dht.begin();
  rtc.begin(); 
  
  pinMode(DE, OUTPUT);                                                    // set communication direction pin as output

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

  if (!BLE.begin()) {                                                     // fail case for Bluetooth connection 
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("SoilGatorRS485");                                     // name that will appear when scanning for Bluetooth® devices
  BLE.setAdvertisedService(newService);
  newService.addCharacteristic(switchChar);                               // add characteristics to a service
  newService.addCharacteristic(randomReading);
  
  newService.addCharacteristic(dataTemp); 
  newService.addCharacteristic(dataMoist); 
  newService.addCharacteristic(dataLight);
  newService.addCharacteristic(dataAcid);
  
  BLE.addService(newService);                                             // add the service
  switchChar.writeValue(0);                                               // set initial value for characteristics
  randomReading.writeValue(0);
  
  dataTemp.writeValue(0); 
  dataMoist.writeValue(0);
  dataLight.writeValue(0);
  dataAcid.writeValue(0);
  
  BLE.advertise();                                                        // start advertising the service
  Serial.println("Bluetooth® device active, waiting for connections...");

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

      if (currentMillis - previousMillis >= 2000) {                       // print every 200ms; may change to 15 min

        previousMillis = currentMillis;

        /* Print Date
        print2digits(rtc.getMonth());
        Serial.print("/");
        print2digits(rtc.getDay());
        Serial.print("/");
        print2digits(rtc.getYear());
        Serial.print(" ");
        */

        Serial.println(currentMillis);                                    // check time; testing only 

        float temp = readTemp();
        dataTemp.writeValue(temp);
        float acid = readAcid();
        dataAcid.writeValue(acid);
        float moist = readMoist();
        dataMoist.writeValue(moist);
        float light = readLight();
        dataLight.writeValue(light);      
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

void getDate () {

}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0"); // print a 0 before if the number is < than 10
  }
  Serial.print(number);
}

void setPlantInfo() {
  for (int i = 0; i < 20; i++) {
    plant[i].name = {};
    plant[i].tempMin = {}; 
    plant[i].tempMax = {};
    plant[i].moistTag = {};
    plant[i].lightTag = {};  
    plant[i].acidMin = {}; 
    plant[i].acidMax = {}; 
  }

}

float readTemp() {
  valueTemp = dht.readTemperature(true);                                    // set true to measure in farenheit
  if (isnan(valueTemp)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  return valueTemp;
}

float readMoist() {
  valueMoist = analogRead(pinMoist);
  valueMoist = map(valueMoist, 1023, 0, 0, 100);                          // invert map to properly show humidity

  return valueMoist;
}

float readLight() {
  int light = 0;
  light = analogRead(pinLight); 
  valueLight = light/10.23;                      

/* CALCULATE SUN EXPOSURE (TIME) *
  if (alarmFlag == false) {                                               // continue to monitor while flag is false; alarmFlag is set true after 8 hours
    if((light > LIGHT) & (upMillis == 0)) {                               // count time only when light is high and when there's no record yet
      upMillis = millis(); 
    }
    if(light < LIGHT) {
      downMillis = millis();                                              // count time when light is low; will keep updating 
    }

    if ((upMillis>0)&(downMillis>upMillis)) {                             // make sure light is high then low at least before calculation
      highDuration = downMillis - upMillis;                               // will depend on the duration set on loop; may move millis functions
      upMillis = 0;                                                       // make sure previous values aren't used again
      downMillis = 0;
    }
    
    upTime += highDuration;                                               // used separate variable; using highDuration sometimes give huge number
    Serial.print("Uptime: ");
    Serial.println(upTime);

    highDuration = 0;                                                     // reset to 0 so it doesn't update upTime with the same value

    alarmFlag = false;                                                    // reset alarm; measurement every set alarm minutes

  }else {                                                                 // do after set duration; testing only, proccess occurs in application
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
*/
  return valueLight;
}

float readAcid() {
  digitalWrite(DE, HIGH);                                                 // sets the max485 to send data

  delay(10);

  for (int j = 0; j < 8; j++) {
    send = pH[j];                                                         // send request to the sensor
    mySerial.write(send);
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
  valueAcid = float ((values[3] << 8) | (values[4])) / 10;
  
  return valueAcid;
}