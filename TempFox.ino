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
#include <SPI.h>
// set pin 8 as the slave select for the digital pot:
const int slaveSelectPin = SS;
const int chipSelectPin = P2_1; 

const byte READ = 0b11111100;   // SCP1000's read command
const byte WRITE = 0b00000010;  // SCP1000's write command

DHT22 mySensor(DHTPIN);
boolean flag;

void setup() {
  pinMode(YELLOW_LED, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("\n\n\n*** DHT22 test starts"); 
  Serial.println("PUSH2 to end"); 
  pinMode(PUSH2, INPUT_PULLUP);     
  
  mySensor.begin();
  // initialize SPI:
  SPI.begin(); 

}

void loop() {
  digitalWrite(YELLOW_LED,LOW);
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  digitalWrite(YELLOW_LED,HIGH);  
  flag = mySensor.get();
  
  printDHT(flag, mySensor.humidityX10(), mySensor.temperatureX10());
  
  writeSpiRegister(0x03, 0x0A);
  
  if (digitalRead(PUSH2)==LOW) {
    Serial.println("\n\n*** End"); 
    Serial.end();
    while(true); // endless loop
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

//Sends a write command to SCP1000

void writeSpiRegister(byte thisRegister, byte thisValue) {

  // SCP1000 expects the register address in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | WRITE;
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}

//Read from or write to register from the SCP1000:
unsigned int readSpiRegister(byte thisRegister, int bytesToRead ) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  Serial.print(thisRegister, BIN);
  Serial.print("\t");
  // SCP1000 expects the register name in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister & READ;
  Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return(result);
}

