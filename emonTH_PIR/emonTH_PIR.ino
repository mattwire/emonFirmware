/*
  emonTx V3 Pulse example -----------------------------------------

  Many meters have pulse outputs, including electricity meters: single phase, 3-phase, 
  import, export.. Gas meters, Water flow meters etc

  The pulse output may be a flashing LED or a switching relay (usually solid state) or both.

  In the case of an electricity meter a pulse output corresponds to a certain amount of 
  energy passing through the meter (Kwhr/Wh). For single-phase domestic electricity meters
  (eg. Elster A100c) each pulse usually corresponds to 1 Wh (1000 pulses per kwh).  

  The code below detects the falling edge of each pulse and increment pulseCount
  
  It calculated the power by the calculating the time elapsed between pulses.
  
  Read more about pulse counting here:
  http://openenergymonitor.org/emon/buildingblocks/introduction-to-pulse-counting
 
 -----------------------------------------emonTH Hardware Connections-----------------------------
 
 Connect the pulse input into emonTH terminal block port 4 (IRQ 0 / Digital 2)
 See: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
 
 If your using an optical counter (e.g TSL256) you should connecting the power pin direct to the 3.3V (terminal block 2) or 5V (if running off 5V USB) (terminal port 1) and GND (terminal port 3)
 
 emonTx V3 Terminal block: 
 port 1: 5V
 port 2: 3.3V
 port 3: GND
 port 4: IRQ 0 / Dig2
 
 
 
 
 -----------------------------------------

  -----------------------------------------
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson, Trystan Lea
  Builds upon JeeLabs RF12 library and Arduino

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
 	- RFu JeeLib		https://github.com/openenergymonitor/rfu_jeelib

  Other files in project directory (should appear in the arduino tabs above)
	- emontx_lib.ino
*/

/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type- 
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes 
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------
*/

#define DEBUG 0                      //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ          // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 26;               // emonTx RFM12B node ID
const int networkGroup = 210;        // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD needs to be same as emonBase and emonGLCD

const int UNO = 1;                   // Set to 0 if your not using the UNO bootloader (i.e using Duemilanove) - All Atmega's shipped from OpenEnergyMonitor come with Arduino Uno bootloader
#include <avr/wdt.h>                 // the UNO bootloader 
#include <avr/power.h>
#include <avr/sleep.h>

#include <RFu_JeeLib.h>              // Download JeeLib: http://github.com/jcw/jeelib
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>                     // https://github.com/adafruit/DHT-sensor-library + https://github.com/adafruit/Adafruit_Sensor

ISR(WDT_vect) { Sleepy::watchdogEvent(); }
  
typedef struct {                     // RFM12B RF payload datastructure
          int temp;
          int temp_external;
          int humidity;    
          int battery;
          byte motion;
} Payload;
Payload emonth;

// Hardwired emonTH pin allocations 
const int DS18B20_PWR=5;
const int DHT22_PWR=6;
const int LED=9;
const int BATT_ADC=1;
const int ONE_WIRE_BUS=19;
const int DHTPIN=18;
const int IRQPIN=2;

// Humidity code adapted from ladyada' example                        // emonTh DHT22 data pin
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
boolean DHT22_status;                                                 // create flag variable to store presence of DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
boolean DS18B20;                                                      // create flag variable to store presence of DS18B20 

volatile byte pirValue = LOW;
volatile byte lastPirValue = LOW;

int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];  // 8 bytes per address

volatile int count = 0;
volatile bool changed = false;

//################################################################################################################################
//################################################################################################################################
void setup() {
  
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);                       // Status LED on
   
  rf12_initialize(nodeID, RF_freq, networkGroup);                       // Initialize RFM12B
  rf12_sleep(RF12_SLEEP);
  
#if (DEBUG == 1)
  Serial.begin(9600);
  Serial.println("emonTH"); 
  Serial.println("OpenEnergyMonitor.org");
  Serial.println("Version: V1.4.1");
  Serial.print("Node: "); 
  Serial.print(nodeID); 
  Serial.print(" Freq: "); 
  if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
  if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
  if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
  Serial.print(" Network: "); 
  Serial.println(networkGroup);
  Sleepy::loseSomeTime(100);
#endif
  
  pinMode(DHT22_PWR,OUTPUT);
  pinMode(DS18B20_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);
  digitalWrite(DHT22_PWR,LOW);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  #if (DEBUG==0)
  power_usart0_disable();   //disable serial UART
  #endif
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the DS18B20 library
  power_timer1_disable();
  power_spi_disable();
 
  //################################################################################################################################
  // Test for presence of DHT22
  //################################################################################################################################
  digitalWrite(DHT22_PWR,HIGH);
  Sleepy::loseSomeTime(2000);                                                        // wait 2s for DH22 to warm up
  dht.begin();
  float h = dht.readHumidity();                                         // Read Humidity
  float t = dht.readTemperature();                                      // Read Temperature
  digitalWrite(DHT22_PWR,LOW);                                          // Power down
  
  if (isnan(t) || isnan(h))                                             // check if returns are valid, if they are NaN (not a number) then something went wrong!
  {
    #if (DEBUG == 1)
    Serial.println(" - Unable to find DHT22 Sensor..trying agin");
    Sleepy::loseSomeTime(100);
    #endif
    
    Sleepy::loseSomeTime(1500); 
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h))   
    {
      #if (DEBUG == 1)
      Serial.println(" - Unable to find DHT22 Sensor for 2nd time..giving up");
      #endif
      
      DHT22_status=0;
    } 
  } 
  else 
  {
    DHT22_status=1;
    #if (DEBUG == 1)
    Serial.println("Detected DHT22 temp & humidity sensor");
    #endif
  }   
 
  //################################################################################################################################
  // Setup and for presence of DS18B20
  //################################################################################################################################
  digitalWrite(DS18B20_PWR, HIGH); Sleepy::loseSomeTime(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(DS18B20_PWR, LOW);
  
  if (numSensors==0)
  {
    #if (DEBUG == 1)
    Serial.println("No DS18B20 detected");
    #endif
    DS18B20=0; 
  } 
  else 
  {
    DS18B20=1; 
    #if (DEBUG == 1) 
      Serial.print("Detected "); Serial.print(numSensors); Serial.println(" DS18B20");
      if (DHT22_status==1) Serial.println("DS18B20 and DHT22 found, assuming DS18B20 is external sensor");
    #endif
    
  }
  
  //if (debug==1) delay(200);
  
  //################################################################################################################################
  
  // Serial.print(DS18B20); Serial.print(DHT22_status);
  // if (debug==1) delay(200);

  pinMode(IRQPIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IRQPIN), pirChange, CHANGE);                                 // KWH interrupt attached to IRQ 0  = Digita 2 - hardwired to emonTx V3 terminal block 
  //attachInterrupt(digitalPinToInterrupt(IRQPIN), pirOff, FALLING);                                 // KWH interrupt attached to IRQ 0  = Digita 2 - hardwired to emonTx V3 terminal block 

  Sleepy::loseSomeTime(100);
  digitalWrite(LED,LOW);

  //if (UNO) wdt_enable(WDTO_8S);
  #if (DEBUG ==1)
    Serial.println("Ready");
  #endif
} // end of setup

void loop() 
{
  digitalWrite(LED, pirValue);
  if (lastPirValue != pirValue)
  {
    lastPirValue = pirValue;
    #if (DEBUG == 1)
    Serial.print("Motion: ");
    Serial.print(pirValue);
    #endif
    emonth.motion = pirValue;
    changed = true;
  }

  Sleepy::loseSomeTime(250);
  count++;

  if (count == 40)
  {
    // Trigger DHT22 sensor read
    if (DHT22_status==1)
    { 
      digitalWrite(DHT22_PWR,HIGH);                              // Send the command to get temperatures
    }
  }
  else if (count > 48)
  {
    // Read DHT22 sensor
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humidity = ((dht.readHumidity()));
    emonth.humidity = (humidity*10);
  
    float temp=(dht.readTemperature());
    if ((temp<85.0) && (temp>-40.0)) emonth.temp = (temp*10);

    #if (DEBUG == 1)
    Serial.print(" Humidity: ");
    Serial.print(humidity);
    Serial.print("% Temperature: ");
    Serial.print(temp);
    Serial.println("C");
    #endif

    digitalWrite(DHT22_PWR,LOW);
    changed = true;
    count = 0;
  }
  if (changed == true)
  {
    changed = false;
    #if (DEBUG==1)
    Serial.println("send");
    #endif
    send_rf();
  }
}

void send_rf()
{
  power_spi_enable();
  rf12_sleep(RF12_WAKEUP);
  Sleepy::loseSomeTime(20);
  rf12_sendNow(0, &emonth, sizeof emonth);
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  power_spi_disable();
}


// The interrupt routine - runs each time a change is detected
void pirChange()                  
{
  pirValue = !pirValue;
}

