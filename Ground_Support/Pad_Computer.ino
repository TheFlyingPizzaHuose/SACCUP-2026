/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Pad Computer Main Code
Authors: Alleon Oxales

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
===FILL IN LATER===
*/
#include <RH_RF95.h> // Include RFM9X library
#include <SPI.h>               // SPI library

//RFM9x pin assignments
#define RFM95_CS 10
#define RFM95_INT 8
#define RFM95_RST 9
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS,RFM95_INT);
const int RFM95PWR = 23;
void setup() {
  //SPI.begin(); 
  Serial.begin(9600); // Start serial communication at 460800 baud rate

  uint32_t reset_reason = SRC_SRSR;  // Read reset status register

  Serial.print("Reset reason (raw value): 0x");
  Serial.println(reset_reason);
  pinMode(LED_BUILTIN, OUTPUT);  // Set the LED pin as an output
  //pinMode(RFM95_RST, OUTPUT);
  /*digitalWrite(RFM95_RST, LOW);
  //delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);*/
  if (!rf95.init()) {
    Serial.println("RFM9X_FAIL");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    Serial.println("RFM9X_INIT_SUCESS");
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(RFM95PWR, false);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
