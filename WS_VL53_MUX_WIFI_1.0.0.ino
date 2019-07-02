/*
  Author: Dean Greenhough 2019

  This project incorporates ultra low power usage for the ESP32, only taking a single 
  power measurement every 24 hours. Battery chemistry LiFePo4, no regulator used to save   
  20% overhead
  
  The Program uses 2 x VL53 Lidar sensors to measure the distance in a salt block softener.
  Utilises TPL5110, ATtiny13a, ESP32, NPN & PNP Mosfets, INA219 for onboard testing only.
  ESP32 will hold its own power supply on, until realeased by Micro Controller.
  TPL5110 35nA standby mode, wakes every 2 hours, wakes the ATtiny13a and checks a counter
  counter is stored in NV memory and in my case at >12 turns on  PNP Mosfet to power ESP32.
  On Handover, Attiny13a shuts down (2.3mA for 162mS every 2 hours)
  ESP32 turns on another NPN Mosfet to take over from the ATtiny Mosfet and ESP32 self powers 
  down when work complete (currently 3-4 Secs every 24 hours) 

  For the purposes of testing MQTT data is sent to a Server based on a Rasp Pi and displayed on Grafana using influxDB,
  Node Red is used to covert the JSON payload  back to an int, to enable its storage on influxDB (not shown in this code)

  An ePaper display is used to display the readings, not shown in this code
*/
//FIRMWARE UPDATE OTA////////////////////////////////////////////////////
#define COMPDATE __DATE__ __TIME__
#define MODEBUTTON 0                    
#include <IOTAppStory.h>                      // IotAppStory.com library
IOTAppStory IAS(COMPDATE, MODEBUTTON);        // Initialize IotAppStory
/////////////////////////////////////////////////////////////////////////
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_INA219.h>
#include "esp_system.h"                    // WDT
// PREFERENCES NVS FLASH
#include <Preferences.h>                   // NV STORAGE
Preferences preferences;
#include "credentials.h"                   //DELETE IF USING YOUR OWN CREDENTIALS
//CREDENTIALS
//char auth[]       = "";
//char WIFI_SSID[]  = "";
//char WIFI_PWD []  = "";

#define DEBUG 1                           // DEBUG On/Off
#define MUX_Address    0x70               // TCA9548A ADDRESS
#define LOOPTIME       0                  // LOOPTIME ONLY OUTPUT    
#define DONEPIN        25                 // TPL5110 SWITCH OFF PIN
#define PROG_BUTTON    26                 // ON_DEMAND
#define PROG_LED       2
#define SERIAL_SPEED   115200             // FOR IOTAPPSTORY
#define MQTTdelay      20				  // MQTT DELAY 20mS

//VL53 INSTANCES
Adafruit_VL53L0X left  = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();      // ADDED A SECOND INSTANCE

Adafruit_INA219 ina219;                           // INA219
//WIFI & MQTT & BLYNK
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG                            // Optional, this enables more detailed prints
//#define mqtt_server       ""                   // server name or IP
//#define mqtt_user         ""                   // username
//#define mqtt_password     ""                   // password
#define looptime_topic    "WS/looptime"          // Topic looptime
#define battery_topic     "WS/battery"
#define current_topic     "WS/current"
#define leftBlock_topic   "WS/leftBlock"
#define rightBlock_topic  "WS/rightBlock"

WiFiClient espClient;
PubSubClient client(espClient);
//VARIABLES
int Button_State = 0; 
unsigned int Distance_LEFT    = 0;
unsigned int Distance_RIGHT   = 0;
//TIMERS
unsigned long loopTime = 0;                //TESTING LOOP TIMER
unsigned long startMillis;                 //TESTING LOOP TIMER
unsigned long currentMillis;               //TESTING LOOP TIMER
unsigned long completeLoopTime = 99;	   //TESTING
// WDT  
const int wdtTimeout = 15000;              //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

//WDT CALL BACK FUNCTION
void IRAM_ATTR resetModule() {
  ets_printf("\n");
  ets_printf("****************************************\n");
  ets_printf("******** WDT ACTIVATED @ 15 SECS *******\n");
  ets_printf("****************************************\n");
  ets_printf("\n");
  delay(10);
  digitalWrite(DONEPIN, LOW);   //SEND DONE TO TPL5110 TO SHUT DOWN
  delay (100);
  //esp_restart();
}

void setupWDT()  {                                         //SETUP WDT
  timer = timerBegin(0, 80, true);                         //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);         //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false);        //set time in us
  timerAlarmEnable(timer);                                 //enable interrupt
  timerWrite(timer, 0);                                    //feed watchdog
}
//INA219
float busvoltage   = 0;
float current_mA   = 0;

//FUNCTIONS
void reconnect() {

  while (!client.connected())
  {
    Serial.print        ("Connecting to MQTT broker ...");
    if (client.connect  ("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println    ("OK");
    } else {
      Serial.print      ("[Error] Not connected: ");
      Serial.print(client.state());
      Serial.println    ("Wait 0.5 seconds before retry.");
      delay(500);
    }
  }
}

void INA219() 
{
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
}

// Initialize I2C buses using TCA9548A I2C Multiplexer
void tcaselect(uint8_t i2c_bus) {
  if (i2c_bus > 7) return;
  Wire.beginTransmission(MUX_Address);
  Wire.write(1 << i2c_bus);
  Wire.endTransmission();
}

void MUXleft() {
  tcaselect(0);
  delay(150);                                        
  if (!left.begin())
  {
    Serial.println(F("LEFT  SENSOR FAILED"));
  }
  VL53L0X_RangingMeasurementData_t measure;
      left.rangingTest(&measure, false);             // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {
    Distance_LEFT    = (measure.RangeMilliMeter);
  } else {
    Serial.println(" LEFT out of range ");
  }
}

void MUXright() {
  tcaselect(1);
  delay(150);
  if (!right.begin())
  {
    Serial.println(F("RIGHT SENSOR FAILED"));
  }
  VL53L0X_RangingMeasurementData_t measure;
  right.rangingTest(&measure, false);				 // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {
    Distance_RIGHT  = (measure.RangeMilliMeter);
  } else {
    Serial.println(" RIGHT  out of range ");
  }
}


void sendMQTT() {									//SEND MQTT TO SERVER

	client.setServer(mqtt_server, 1883);
	if (!client.connected()) {
		reconnect();
	}

	client.publish(looptime_topic, String(completeLoopTime).c_str(), true);
	delay(MQTTdelay);
	if (DEBUG) {
		Serial.print(completeLoopTime);
		Serial.println("  RX looptime  sent to MQTT.");
		delay(MQTTdelay);
	}
	client.publish(battery_topic, String(busvoltage).c_str(), true);
	delay(MQTTdelay);
	if (DEBUG) {
		Serial.print(busvoltage);
		Serial.println("  VOLTAGE sent to MQTT.");
		delay(MQTTdelay);
	}
	client.publish(current_topic, String(current_mA).c_str(), true);
	delay(MQTTdelay);
	if (DEBUG) {
		Serial.print(current_mA);
		Serial.println("  CURRENT sent to MQTT.");
		delay(MQTTdelay);
	}
	client.publish(leftBlock_topic, String(Distance_LEFT).c_str(), true);
	delay(MQTTdelay);
	if (DEBUG) {
		Serial.print(Distance_LEFT);
		Serial.println("  LEFT sent to MQTT.");
		delay(MQTTdelay);
	}
	client.publish(rightBlock_topic, String(Distance_RIGHT).c_str(), true);
	delay(MQTTdelay);
	if (DEBUG) {
		Serial.print(Distance_RIGHT);
		Serial.println("  RIGHT sent to MQTT.");
		delay(MQTTdelay);
		Serial.println("MQTT Sent");
		
	}
	delay(2000);
}


void setup() {
  startMillis = millis();								
  pinMode(DONEPIN, OUTPUT);                                     //MUST BE FIRST TO KEEP POWER ON
  digitalWrite(DONEPIN, HIGH);
  pinMode(PROG_BUTTON, INPUT_PULLUP);
  pinMode(PROG_LED, OUTPUT);
  digitalWrite (PROG_LED, LOW);
  Button_State = digitalRead (PROG_BUTTON);
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  ina219.begin();                                               // INIT INA219
  ina219.setCalibration_16V_400mA();                            // SET CALIBRATION RANGE
  //WDT
  Serial.println("Starting WDT @ 15 Secs");
  setupWDT();
  //NVS - PREFERENCES
  preferences.begin("IAS_UPDATE", false);
  unsigned int counter = preferences.getUInt("counter", 0);
  Serial.printf("Initial counter value: %u\n", counter);
}


void loop() {

    INA219();                                                 //GET POWER MEASUREMENTS
    MUXleft();                                                //HAS TO BE HERE AS MISSES DATA ON FIRST PASS NO IDEA WHY?
    MUXright();												  //GET RIGHT BLOCK MEASUREMENT
    MUXleft();												  //GET LEFT BLOCK MEASUREMENT
	sendMQTT();												  //SEND MQTT TO SERVER
    currentMillis = millis();                                  //END LOOPTIME
    loopTime = (currentMillis - startMillis);
    if (DEBUG)Serial.print("looptime =  ");
    if (DEBUG)Serial.println(loopTime);
    if (DEBUG)Serial.println(" ");
    digitalWrite(DONEPIN, LOW);                                //SEND DONE TO NPN MOSFET TO SHUT DOWN POWER TO ESP32
    delay(10);
  }

