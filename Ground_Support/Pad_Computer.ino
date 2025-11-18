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
#define RF95_FREQ 434.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS,RFM95_INT);
const int RFM95PWR = 23;

bool valve_on = 0;

void setup() {
  pinMode(7, OUTPUT);
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");
  else{
    Serial.println("init sucess");
    rf95.setFrequency(433.0);
  }
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      if(buf[0] == 'A'){
        digitalWrite(7, HIGH);
      }
      if(buf[0] == 'B'){
        digitalWrite(7, LOW);
      }
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "ACK";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}


//==========RADIO CODE==========Alleon Oxales
void readRFM() {
  if (!errorCodes[RFM9X_FAIL] && rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.println(buf[0]);
      commands(buf[0]);
    }
  }
}
void sendRFM() {
  if ((micros() - RFM_LAST > RFM_RATE) && !errorCodes[RFM9X_FAIL]) {
    uint8_t* massage = reinterpret_cast<uint8_t*>(readyPacket());
    //itoa(packetnum++, radiopacket+13, 10);
    //Serial.print("Sending "); Serial.println(radiopacket);
    //radiopacket[19] = 0;

    //Serial.println("Sending...");
    //delay(10);
    rf95.send(massage, charArrayLegnth);

    //Serial.println("Waiting for packet to complete...");
    //delay(10);
    //rf95.waitPacketSent();
    RFM_LAST = micros();
  }
}
uint8_t radioChecksum(int* radioMSG, uint8_t msgLength) {//fletcher 8 checksum algorithm
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < msgLength; i++) {
      sum1 = (sum1 + radioMSG[i]) % 15;
      sum2 = (sum2 + sum1) % 15;
  }
  return (sum2 << 4) | sum1;
}  //end radioChecksum
char* float_to_byte(float num) {  //Alleon Oxales, Converts float bytes
  char* bits = reinterpret_cast<char*>(&num);
  return bits;
}  //end dec_to_binary
char* int_to_byte(int num) {//Aleon Oxales, Converts integeter to bytes
  char* bits = reinterpret_cast<char*>(&my_dec);
  return bits;
}
float byte_to_float(int my_bit_size, int* my_arr) { //WIP
  float my_sum = 0;
  int my_index = 0;
  for (int i = my_bit_size - 1; i >= 0; i--) {
    my_sum = pow(2, my_index) * *(my_arr + i) + my_sum;
    my_index++;
  }
  return my_sum;
}  //end binary to decimal
int* uint_to_binary(char character) { //Leiana Mendoza
  static int result[8];
  for (int i = 0; i <= 7; i++) {
    result[i] = (character & (1 << (7-i))) > 0;
  }
  return result;
}  //end uint_to_binary
//==============================