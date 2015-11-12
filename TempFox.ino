
/// 
/// @mainpage	DHT22 temperature and humidity sensor library
/// @details	DHT22 on LaunchPad
/// @n
/// @n		2012-06-15 First release
/// @n		2012-06-17 Arduino-related code wiped-out
/// @n		2012-06-17 int32_t only to avoid float and math.h
/// @n		2012-07-02 LaunchPad release
///
/// @n @a	Developed with [embedXcode](http://embedXcode.weebly.com)
/// 
/// @author	Rei VILO
/// @author	http://embeddedcomputing.weebly.com
/// @date	Jul 02, 2012
/// @version	2.01
/// 
/// @copyright	¬© Rei VILO, 2012
/// @copyright	CC = BY NC SA
///
/// @see	
/// @n		Based on http://www.ladyada.net/learn/sensors/dht.html
/// @n		written by Adafruit Industries, MIT license
/// @n		 
/// @b		LaunchPad implementation 
/// @n		by energia ¬ª Tue Jun 26, 2012 9:24 pm
/// @n		http://www.43oh.com/forum/viewtopic.php?p=20821#p20821
/// @n		As LaunchPad is faster than Arduino,
/// *		1. replace delayMicroseconds(1) with delayMicroseconds(3)
/// @n 	or
/// *		2. compare counter to a higher number
///

///
/// @file	DHT22_430_main.pde 
/// @brief	Main sketch
/// @details	DHT22 on LaunchPad
/// @n @a	Developed with [embedXcode](http://embedXcode.weebly.com)
/// 
/// @author	Rei VILO
/// @author	http://embeddedcomputing.weebly.com
/// @date	Jul 02, 2012
/// @version	2.01
/// 
/// @copyright	¬© Rei VILO, 2012
/// @copyright	CC = BY NC SA
///


// Core library - MCU-based
#if defined(__MSP430G2452__) || defined(__MSP430G2553__) || defined(__MSP430G2231__) // LaunchPad specific
#include "Energia.h"
#else // error
#error Platform not supported
#endif

// Include application, user and local libraries
#include "DHT22_430.h"


///
/// @brief	Pin for DHT22 signal
/// @n 		Connect 
/// *		pin 1 (on the left) of the sensor to +5V
/// *		pin 2 of the sensor to DHTPIN 
/// *		pin 4 (on the right) of the sensor to GROUND
/// @n		Place a 10k resistor between pin 2 (data) to pin 1 (power) of the sensor
///
//#define DHTPIN P1_4
#define DHTPIN P2_3
#define YELLOW_LED P1_0

#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

Enrf24 radio(P2_0, P2_1, P2_2);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
const char *str_on = "ON";
const char *str_off = "OFF";
void dump_radio_status_to_serialport(uint8_t);

DHT22 mySensor(DHTPIN);
boolean flag;

void setup() {
  pinMode(YELLOW_LED, OUTPUT);
  
  Serial.begin(9600);

  //setup SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
 
   radio.begin();
   dump_radio_status_to_serialport(radio.radioState());
   radio.setTXaddress((void*)txaddr);
 
  // setup DHT
  mySensor.begin();

 }

void loop() {
  digitalWrite(YELLOW_LED,LOW);
  delay(500);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  digitalWrite(YELLOW_LED,HIGH);  
  flag = mySensor.get();
  printDHT(flag, mySensor.humidityX10(), mySensor.temperatureX10());
  
  
  //flashLed(100,5);
  Serial.print("Sending packet: ");
  Serial.println(str_on);
  radio.print(str_on);
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(500);

  Serial.print("Sending packet: ");
  Serial.println(str_off);
  radio.print(str_off);
  radio.flush();  //
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  
}


void flashLed(int time, int NrOfFlash) {
  for (int i=0; i <= NrOfFlash; i++){
    digitalWrite(YELLOW_LED,LOW);  
    delay(time/2);
    digitalWrite(YELLOW_LED,HIGH);  
    delay(time/2);
   } 

}
int16_t GetBatteryVoltage()
{
  //TODO
  // measure batteryvoltage on ADC pin, report back.
  // see AnalogInput example,
  // use one of the 8 channels
}
void sendPacket() {
  Serial.print("Sending packet: ");
  Serial.println(str_on);
  radio.print(str_on);
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(200);

  Serial.print("Sending packet: ");
  Serial.println(str_off);
  radio.print(str_off);
  radio.flush();  //
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(200);
}

void dump_radio_status_to_serialport(uint8_t status)
{
  Serial.print("Enrf24 radio transceiver status: ");
  switch (status) {
    case ENRF24_STATE_NOTPRESENT:
      Serial.println("NO TRANSCEIVER PRESENT");
      break;

    case ENRF24_STATE_DEEPSLEEP:
      Serial.println("DEEP SLEEP <1uA power consumption");
      break;

    case ENRF24_STATE_IDLE:
      Serial.println("IDLE module powered up w/ oscillators running");
      break;

    case ENRF24_STATE_PTX:
      Serial.println("Actively Transmitting");
      break;

    case ENRF24_STATE_PRX:
      Serial.println("Receive Mode");
      break;

    default:
      Serial.println("UNKNOWN STATUS CODE");
  }
}


// print DHT data
void printDHT(boolean flag, int32_t humidity, int32_t temperature)
{
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (!flag) {
    Serial.println("Failed to read from DHT");
  } 
  else {
    Serial.print("RH% \t");
    Serial.print(humidity/10);
    Serial.print(".");
    Serial.print(humidity%10);
    Serial.println(" %\t");
    
    Serial.print("oC \t");
    Serial.print(temperature/10);
    Serial.print(".");
    Serial.print(temperature%10);
    Serial.println(" *C");    
  }
  
}
