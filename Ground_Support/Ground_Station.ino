/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Ground Station Main Code
Authors: Alleon Oxales

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/1AqIgfhQb1Wmkl7yFG0nSHRvpzL-viViXBCPfHbvtkt0/edit?usp=sharing
===FILL IN LATER===
*/
#include <SoftwareSerial.h>
#include <RH_RF95.h> // Include RFM9X library
#include <SPI.h>

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS    4  //
#define RFM95_INT   3  //
#define RFM95_RST   2  // "A"

// Change to 434.0 or other frequency, must match RX's freq!
#define GSE_FREQ 432.92
#define AV1_FREQ 433.0
#define AV2_FREQ 433.08

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const int RFM95PWR = 23;

//Command Message Class
u_int8 msg_class_01[13][10] = {{1,1,1}, //Ignition Abort
                              {1,1,1}, //Avionics & Pad Arm
                              {1,1,1}, //Ignition Sequence Start
                              {1,1,1}, //N2O Fill Valve Open
                              {1,1,1}, //N2O Fill Valve Closed
                              {1,1,1}, //N2 Fill Valve Open
                              {1,1,1}, //N2 Fill Valve Close
                              {1,1,1}, //Relief Valve Open
                              {1,1,1}, //Relief Valve Closed
                              {1,1,1}, //Disconnect Rocket Fill Line
                              {1,1,1}, //Open Cooling Clamshell
                              {1,1,1}, //Enable TX
                              {1,1,1}, //Disable TX
                              };

//Telemetry Message Class
u_int8 msg_class_02[3][10] = {{5,5,5,5,5,5,5,5,5,5}, //AV1 Telemetry
                              {5,5,5,5,5,5,5,5,5,5}, //AV2 Telemetry
                              {5,5,5,5,5,2,2,2,2,2}, //GSE Temps, Presses, Rocket Mass and Status
                              };
                              \
//Alert Message Class
u_int8 msg_class_03[7][10] = {{1,1,1}, //No Igniter Continuity
                              {1,1,1}, //Quick Disconnect Fail
                              {1,1,1}, //Clamshell Fail
                              {1,1,1}, //GPS Lock Fail
                              {4}, //Rocket List of Failed Sensors
                              {4}, //GSE List of Failed Sensors
                              {2}, //Rocket Computer Reset Register Value
                              };

void setup() {   
  Serial.begin(38400);
  Serial.print("Reset Register Value: ")
  Serial.println(''+SRC_SRSR);
  if(SRC_SRSR != 1){}// Read reset status register and PRGM_ERR if reset is not a power cycle

  pinMode(RFM69_RST, OUTPUT);
  Serial.println("Reset RFM9x");

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init()){
    Serial.println("RFM9x init failed");  
  }else{
    Serial.println("RFM0x init sucess");
    rf95.setFrequency(AV1_FREQ);
    rf95.setTxPower(RFM95_PWR, false);
  }
}

void loop() {
  if(Serial.available()){
    rf95.setFrequency(433.0);
    Serial.println("Set 433Mhz");
    uint8_t data[] = {Serial.read()};
    Serial.println("Sending  command");
    // Send a message to rf95_server
    rf95.send(data, sizeof(data));
    
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(3000))
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
    {
        Serial.print("got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("recv failed");
      }
    }
    else
    {
      Serial.println("No reply");
    }
    delay(400);
    rf95.setFrequency(434.0);
    Serial.println("Set 434Mhz");
  }
}