/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "AS5600.h"

#define CE_PIN 7
#define CSN_PIN 8
// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
AS5600 as5600;

const byte address[] = "2Node";
uint16_t payload[1] = { 0 };


void setup() {
  pinMode(5, OUTPUT);
  Serial.begin(115200);
  
  // START OF ENCODER
  Wire.begin();
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("encoder connected: ");
  Serial.println(as5600.isConnected());


  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
 

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(payload)); 
  radio.openWritingPipe(address);
  radio.stopListening();  // put radio in TX mode
  radio.setDataRate(RF24_250KBPS);
  ;radio.setAutoAck(false);
}  // setup


void loop() {
  payload[0] = as5600.readAngle();
  // This device is a TX node
  unsigned long start_timer = micros();                // start the timer
  bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
  unsigned long end_timer = micros();  
  if (report) {
    digitalWrite(5,HIGH);
  } else {
    digitalWrite(5,LOW);
  }
  // Serial.println(as5600.readAngle());   
  delay(30);             
}  // loop
