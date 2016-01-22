

// Core library - MCU-based
#if defined(__MSP430G2452__) || defined(__MSP430G2553__) || defined(__MSP430G2231__) // LaunchPad specific
#include "Energia.h"
#else // error
#error Platform not supported
#endif

// Include application, user and local libraries
#include "DHT22_430.h"
#define YELLOW_LED P1_0

// include software I2C
#include "I2C_SoftwareLibrary.h"
#define SCL_PIN P2_4 ///< pin for SCL
#define SDA_PIN P2_3 ///< pin for SDA
SoftwareWire Wire(SDA_PIN, SCL_PIN); ///< Instantiate SoftwareWire
#define _address 0x18
uint16_t _reading;


#define DHTPIN P2_3

#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

Enrf24 radio(P2_0, P2_1, P2_2);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xff };

const int8_t node_addr = 1;
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, node_addr };

const int timeout = 30*1000; // time between the measurements
const char *str_on = "ON";
const char *str_off = "OFF";
String tx_data_string, humidity_data_string;
int16_t humidity, temperature;

void dump_radio_status_to_serialport(uint8_t);

DHT22 mySensor(DHTPIN);
boolean flag;

// Voltagemeasurement
#define ANALOG_HALFVCC_INPUT A3

void setup() {
  pinMode(YELLOW_LED, OUTPUT);
  digitalWrite(YELLOW_LED,HIGH);
  Serial.begin(9600);

  analogReference(INTERNAL2V5);
  //setup SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
 
  radio.begin();
   radio.setChannel(0x4c);
   radio.autoAck(true);
   radio.setAutoAckParams(15,15);
   radio.setSpeed(250000);
   
   radio.setTXaddress((void*)txaddr);
   radio.setRXaddress((void*)rxaddr);
   radio.enableRX();
   radio.deepsleep();
    
  dump_radio_status_to_serialport(radio.radioState());
   
  // setup DHT
  mySensor.begin();
  
  // setup Software i2c
  //Wire.begin();
  digitalWrite(YELLOW_LED,LOW);
 }

void loop() {
  char inbuf[33];


  sleep(node_addr*1000);
  if (true) {
    
    flashLed(5,1);    
    flag = mySensor.get();
    humidity = mySensor.humidityX10() ; 
    temperature = mySensor.temperatureX10();
    int voltage = getVCC();
    
    String node_str         = String(node_addr, HEX);
    String humidity_str     = String(humidity);
    String temperature_str  = String(temperature);
    String voltage_str      = String(voltage);
    String separator = "-";      
    
    tx_data_string = node_str  ;
    tx_data_string +=  separator ;
    tx_data_string +=  humidity_str  ;
    tx_data_string +=  separator ;
    tx_data_string +=  temperature_str  ;
    tx_data_string +=  separator ;
    tx_data_string +=  voltage  ;
    Serial.print(" tx_data_string: ");
    Serial.println(tx_data_string);
  
    radio.print(tx_data_string);
    radio.flush();  // Force transmit (don't wait for any more data)
    //dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
    radio.deepsleep();
    dump_radio_status_to_serialport(radio.radioState());  // Should report deepsleep

    //flashLed(50,10);
    
    // sleep for remaining to the max intervall
    sleep(timeout-node_addr);

   }
}

void flashLed(int time, int NrOfFlash) {
  for (int i=0; i <= NrOfFlash; i++){
    digitalWrite(YELLOW_LED,LOW);  
    delay(time/2);
    digitalWrite(YELLOW_LED,HIGH);  
    delay(time/2);
    digitalWrite(YELLOW_LED,LOW);  
    } 

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

  // map the measured voltage to the internal reference  
  data = (int)map(data, 0, 1023, 0, 1500);
  
  // apply resistor network multiplier 100k/20k = 6 to calculate the supply voltage
  data = data * 6;
  return data; 
}
