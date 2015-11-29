
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

// include software I2C
#include "I2C_SoftwareLibrary.h"
#define SCL_PIN P2_4 ///< pin for SCL
#define SDA_PIN P2_3 ///< pin for SDA
SoftwareWire Wire(SDA_PIN, SCL_PIN); ///< Instantiate SoftwareWire
#define _address 0x18
uint16_t _reading;


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
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 10 };

const char *str_on = "ON";
const char *str_off = "OFF";
String tx_data_string, humidity_data_string;
int16_t humidity, temperature;

void dump_radio_status_to_serialport(uint8_t);
const byte Node_addr = 0x01;

DHT22 mySensor(DHTPIN);
boolean flag;

// Voltagemeasurement
#define ANALOG_HALFVCC_INPUT A3

void setup() {
  pinMode(YELLOW_LED, OUTPUT);
  
  Serial.begin(9600);

  analogReference(INTERNAL2V5);
  //setup SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
 
   radio.begin();
   dump_radio_status_to_serialport(radio.radioState());
   radio.setTXaddress((void*)txaddr);
   radio.setRXaddress((void*)rxaddr);
 
 
  // setup DHT
  mySensor.begin();
  
  // setup Software i2c
  //Wire.begin();


 }

void loop() {
  digitalWrite(YELLOW_LED,LOW);
  delay(500);
  
  // Measure Humidity/Temperature
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  digitalWrite(YELLOW_LED,HIGH);  
  flag = mySensor.get();
  humidity = mySensor.humidityX10() ;
  
  temperature = mySensor.temperatureX10();
  //printDHT(flag, mySensor.humidityX10(), mySensor.temperatureX10());
  
  // Measure Voltage
//  Serial.print("VCC value:");
  int voltage = getVCC();
//  Serial.println (voltage);
  delay(1000);
  
  // Send data via RF24
  //flashLed(100,5);
  // TODO change to separate defined strings, then concatenate them!
  tx_data_string = "";
  tx_data_string += "Node " ;
  tx_data_string += rxaddr[4];
  tx_data_string += " Humidity " ;
  tx_data_string += String(humidity/10);
//  tx_data_string += "-";
//  tx_data_string += String(humidity%10);
  tx_data_string += " % Temp ";
  tx_data_string += String(temperature/10);
//  tx_data_string += "-";
//  tx_data_string += String(temperature%10);
  tx_data_string += " degC Voltage ";
  tx_data_string += String(voltage);
  tx_data_string += " mV";
  
  Serial.print("Sending packet: ");
  Serial.println(tx_data_string);
  radio.print(tx_data_string);
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(500);
  
 
  // send I2C cmd
    //write
//    Wire.beginTransmission(_address);
//    Wire.write('A');
//    Wire.endTransmission();
//    // read
//    Wire.requestFrom(_address, 2);
//    while (Wire.available() < 2);
//    
//    flashLed(40,5);
//    _reading = Wire.read();
//    _reading = _reading << 8;
//    _reading += Wire.read();
//  Serial.print("I2C Read packet: ");
//    Serial.print(_reading/10, DEC);
//    Serial.print(".");
//    Serial.println(_reading%10, DEC);
//    

  
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
  delay(100);

  Serial.print("Sending packet: ");
  Serial.println(str_off);
  radio.print(str_off);
  radio.flush();  //
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(100);
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

// returns VCC in millivolts
int getVCC() {
  // start with the 1.5V internal reference
  analogReference(INTERNAL1V5);
  int data = analogRead(ANALOG_HALFVCC_INPUT);
  // if overflow, VCC is > 3V, switch to the 2.5V reference
  if (data==0x3ff) {
    analogReference(INTERNAL2V5);
    data = (int)map(analogRead(ANALOG_HALFVCC_INPUT), 0, 1023, 0, 5000);
  } else {
    data = (int)map(data, 0, 1023, 0, 3000);
  }
  return data; 
}
