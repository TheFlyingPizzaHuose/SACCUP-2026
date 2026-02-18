//WIP: 
//1. Radio Checksum verification
//2. <100kHz radio bandwidth

/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Ground Station Main Code
Authors: Alleon Oxales

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/1AqIgfhQb1Wmkl7yFG0nSHRvpzL-viViXBCPfHbvtkt0/edit?usp=sharing
*/
#include <SoftwareSerial.h>
#include "RadioHead/RH_RF95.h" // Include RFM9X library
#include <SPI.h>              // SPI library
//#include <map>
//#include <string>

//===============Constant Variables===============
//RFM9x pin assignments
//Teensy #define RFM95_CS    4
#define RFM95_CS    10 //Arduino
#define RFM95_INT   3
#define RFM95_RST   4 //Arduino Nano

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const int RFM95_PWR = 23;

//Command Message Class
uint8_t msg_class_01[22][10] = {{1,1,1}, //Ignition Abort
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
                              {1,1,1}, //Tare Scale 1
                              {1,1,1}, //Tare Scale 2
                              {1,1,1}, //AV1 Inertial Align
                              {1,1,1}, //AV2 Inertial Align
                              {1,1,1,5}, //Set GSE Radio Frequency
                              {1,1,1,5}, //Set AV1 Radio Frequency
                              {1,1,1,5}, //Set AV2 Radio Frequency
                              {1,1,1}, //Set load cell channel A
                              {1,1,1} //Set load cell channel B
                              };

uint8_t msg_class_02[6][10] = {{5,5,5,5,5,5,5,5,5,5}, //AV1 Telemetry
                              {5,5,5,5,5,5,5,5,5,5}, //AV2 Telemetry
                              {5,5,5,5,5,5}, //GSE Temps, Presses, Supply and Rocket Mass
                              {4,5,5,5}, //AV1 Detected Flight Events
                              {4,5,5,5}, //AV2 Detected Flight Events
                              {4} //GSE States
                              };
                              
//Alert Message Class
uint8_t msg_class_03[7][10] = {{1,1,1}, //No Igniter Continuity
                                      {1,1,1}, //Quick Disconnect Fail
                                      {1,1,1}, //Clamshell Fail
                                      {1,1,1}, //GPS Lock Fail
                                      {4}, //Rocket List of Failed Sensors
                                      {4}, //GSE List of Failed Sensors
                                      {2}, //Rocket Computer Reset Register Value
                                      };

//Time between component usages in micros
const unsigned int RFM9x_rate = 50000;
const unsigned int SD_rate = 50000;
const unsigned int RFM9x_send_rate = 500000;
//const unsigned intPC_rate = 40000; <Not needed cuz PC telem gets sent everytime telem is recieved.

//===============Dynamic Variables===============

uint8_t freq_select_index = 0;

// Different Computer Frequencies
float GSE_FREQ = 432.875;
float AV1_FREQ = 433.0;
float AV2_FREQ = 433.125;

//Current RSSI's
float GSE_RSSI = 0;
float AV1_RSSI = 0;
float AV2_RSSI = 0;

//3 arrays w/ 100 bools store if previous RSSI greater than -50
//Indecies also present, used when writing and index get's reset at i=99
bool past_GSE_RSSI[100];
uint8_t GSE_RSSI_index = 0;
bool past_AV1_RSSI[100];
uint8_t AV1_RSSI_index = 0;
bool past_AV2_RSSI[100];
uint8_t AV2_RSSI_index = 0;

//Message parser output arrays
uint8_t out_flag[10]; 
uint8_t out_int8s[10];
uint16_t out_int16s[10];
uint32_t out_int32s[10];
float out_floats[10];

//Times of last component usages
unsigned int RFM9x_last_time = 0;
unsigned int RFM9x_send_last_time = 0;
unsigned int SD_last_time = 0;

//Frequency selection index 0:GSE, 1:AV1, 2:AV2 & time since last ACK
unsigned int GSE_ACK_last_time = 0;
unsigned int AV1_ACK_last_time = 0;
unsigned int AV2_ACK_last_time = 0;

//Ground station to PC telemetry variables
float pc_telem[33] = {
  0, //0 Position X
  0, //1 Position Y
  0, //2 Position Z
  0, //3 Velocity X
  0, //4 Velocity Y
  0, //5 Velocity Z
  0, //6 Orientation Theta 
  0, //7 Supply Tank Temp
  0, //8 Rocket Tank Temp
  0, //9 N2O Line Pressure
  0, //10 N2 Line Pressure
  0, //11 Supply Tank Load Cell Reading
  0, //12 Rocket Tank Load Cell Reading
  0, //13 Ambient Temperature
  0, //14 Ambient pressure
  0, //15 AV1 Ox-Tank pressure
  0, //16 AV2 CO2 pressure
  0, //17 AV1 Battery Voltage
  0, //18 AV2 Battery Voltage
  0, //19 N2O Valve State
  0, //20 N2 Valve State
  0, //21 Poppet Valve State
  0, //22 Quick Disconnect State
  0, //23 Clamshell State
  0, //24 AC Unit State
  0, //25 GSE RSSI
  0, //26 AV1 RSSI
  0, //27 AV2 RSSI
  0, //28 GSE ACK
  0, //29 AV1 ACK
  0, //30 AV2 ACK
  0, //31 Passive Vent Valve State
  0, //32 Dump Vent Valve State
};
size_t pc_telem_length = sizeof(pc_telem)/sizeof(pc_telem[0]);
//Message assembler input arrays
uint8_t in_flag[10] ; 
uint8_t in_int8s[10] ;
uint16_t in_int16s[10] ;
uint32_t in_int32s[10] ;
float in_floats[10] ;

//Message assembler buffer, message length, and message ready boolean, recieve ready boolean
uint8_t message_send_buf[43];
uint8_t message_send_len = 0;
uint8_t msg_ready = 0;
uint8_t recv_ready = 0;

void setup() {   
  Serial.begin(115200);

  // Save copy of Reset Status Register
  //uint32_t lastResetCause = SRC_SRSR;
  // Clear all Reset Status Register bits
  //SRC_SRSR = (uint32_t)0x1FF;

  // ... more setup steps, get USB ready ...
  //resetCause(lastResetCause);
  /*
  pinMode(RFM95_RST, OUTPUT);
  Serial.println("Reset RFM9x");

  //Manual reset of RFM9x
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);*/
  if (!rf95.init()){
    //Serial.println("RFM9x init failed");  
  }else{
    //Serial.println("RFM9x init sucess");
    rf95.setFrequency(GSE_FREQ);
    rf95.setTxPower(RFM95_PWR, false);
  }
  //Serial.println("Type \"h\" for a list of commands");
}

void loop() {
  send_command();
  read_RFM();
  send_RFM();
}
//==========COMMAND CODE==========Alleon Oxales
void send_command(){
  if(Serial.available()){
    char command = Serial.read();
    uint8_t command_id = 0;
    switch(command){//Switch case is best so it doesn't send a command if input is incorrect
      case 'h': print_command_list(); break;
      case '1': command_id = 0x01; break;
      case '2': command_id = 0x02; break;
      case '3': command_id = 0x03; break;
      case '4': command_id = 0x04; break;
      case '5': command_id = 0x05; break;
      case '6': command_id = 0x06; break;
      case '7': command_id = 0x07; break;
      case '8': command_id = 0x08; break;
      case '9': command_id = 0x09; break;
      case 'A': command_id = 0x0A; break;
      case 'B': command_id = 0x0B; break;
      case 'C': command_id = 0x0C; break;
      case 'D': command_id = 0x0D; break;
      case 'E': command_id = 0x0E; break;
      case 'F': command_id = 0x0F; break;
      case 'G': command_id = 0x10; break;
      case 'H': command_id = 0x11; break;
      case 'I':{
        command_id = 0x12; break;
        in_floats[3] = GSE_FREQ;
      } 
      case 'J':{
        command_id = 0x13; break;
        in_floats[3] = AV1_FREQ;
      } 
      case 'K':{
        command_id = 0x14; break;
        in_floats[3] = AV2_FREQ;
      } 
      case 'L': command_id = 0x15; break;
      case 'M': command_id = 0x16; break;
    }
    set_command_msg(command_id);
  }
}
void set_command_msg(uint8_t command_id){
  in_flag[0] = 0xFF;
  in_flag[1] = 0xFF;
  in_flag[2] = 0xFF;
  message_assembler(0x01, command_id);
  msg_ready = 10;
}
void print_command_list(){
  Serial.println("1: Ignition Abort");
  Serial.println("2: Launch Arm");
  Serial.println("3: Ignition Sequence Start");
  Serial.println("4: N2O Fill Valve Open");
  Serial.println("5: N2O Fill Valve Closed");
  Serial.println("6: N2 Fill Valve Open");
  Serial.println("7: N2 Fill Valve Closed");
  Serial.println("8: Relief Valve Open");
  Serial.println("9: Relief Valve Closed");
  Serial.println("A: Disconnect Rocket Fill Line");
  Serial.println("B: Open Cooling clamshell");
  Serial.println("C: Enable TX");
  Serial.println("D: Disable TX");
  Serial.println("E: Tare Supply Tank Load Cell");
  Serial.println("F: Tare Rocket Tank Load Cell");
  Serial.println("G: AV1 Inertial Align");
  Serial.println("H: AV2 Inertial Align");
  Serial.println("I: Set GSE Radio Frequency");
  Serial.println("J: Set AV1 Radio Frequency");
  Serial.println("K: Set AV2 Radio Frequency");
  Serial.println("L: Set load cell channel to A");
  Serial.println("M: Set load cell channel to B");
}
//==========TELEMETRY CODE==========Alleon Oxales
void read_telem(uint8_t msg_class, uint8_t msg_id){
  if(msg_class == 0x02){//Telemetry message class
    if(msg_id == 0x01){//AV1 Telemetry
      //Position & Velocity comes from AV1
      pc_telem[0] = out_floats[0];//Position X
      pc_telem[1] = out_floats[1];//Position Y
      pc_telem[2] = out_floats[2];//Position Z
      pc_telem[3] = out_floats[3];//Velocity X
      pc_telem[4] = out_floats[4];//Velocity Y
      pc_telem[5] = out_floats[5];//Velocity Z
    }
    if(msg_id == 0x02){//AV2 Telemetry
      //Quaternion comes from AV2
      pc_telem[6] = out_floats[6];//Orientation Theta
    }
    if(msg_id == 0x03){//GSE telemetry
      //pc_telem[7] = out_floats[0];//Supply Temp
      pc_telem[15] = out_floats[0];//Ox-tank pressure HACK
      pc_telem[8] = out_floats[1];//Rocket Temp
      pc_telem[9] = out_floats[2];//N2O Line Pres
      pc_telem[10] = out_floats[3];//N2 Line Pres
      pc_telem[11] = out_floats[4];//Supply Mass
      pc_telem[12] = out_floats[5];//Rocket Mass
      //Serial.println(out_floats[4]);
      //Serial.println(out_floats[5]);
    }
    if(msg_id == 0x04){//AV1 Events, Pressures, Voltage
      pc_telem[15] = out_floats[2];//AV1 Ox-tank pressure
      pc_telem[17] = out_floats[3];//AV1 Battery Voltage
    }
    if(msg_id == 0x05){//AV2 Events, Pressures, Voltage
      pc_telem[16] = out_floats[1];//AV2 CO2 pressure
      pc_telem[18] = out_floats[3];//AV2 Battery Voltage
    }
    if(msg_id == 0x6){
      pc_telem[19] = static_cast<float>((out_int32s[0] >> 0) & 1); //N2O Valve State
      pc_telem[20] = static_cast<float>((out_int32s[0] >> 1) & 1); //N2 Valve State
      pc_telem[21] = static_cast<float>((out_int32s[0] >> 2) & 1); //Poppet Valve State
      pc_telem[22] = static_cast<float>((out_int32s[0] >> 3) & 1); //Quick Disconnect State
      pc_telem[23] = static_cast<float>((out_int32s[0] >> 4) & 1); //Clamshell State
      pc_telem[24] = static_cast<float>((out_int32s[0] >> 5) & 1); //AC Unite State
    }
    pc_telem[25] = GSE_RSSI;
    pc_telem[26] = AV1_RSSI;
    pc_telem[27] = AV2_RSSI;
  }
  Serial.write((uint8_t*)&pc_telem[0], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[1], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[2], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[3], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[4], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[5], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[6], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[7], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[8], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[9], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[10], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[11], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[12], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[13], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[14], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[15], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[16], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[17], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[18], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[19], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[20], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[21], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[22], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[23], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[24], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[25], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[26], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[27], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[28], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[29], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[30], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[31], sizeof(pc_telem[0]));Serial.print(',');
  Serial.write((uint8_t*)&pc_telem[32], sizeof(pc_telem[0]));Serial.println();
  Serial.flush();
}

//==========RADIO CODE==========Alleon Oxales
void read_RFM() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      if(freq_select_index == 0){
        GSE_RSSI = static_cast<float>(rf95.lastRssi());
      }
      if(freq_select_index == 1){
        AV1_RSSI = static_cast<float>(rf95.lastRssi());
      }
      if(freq_select_index == 2){
        AV2_RSSI = static_cast<float>(rf95.lastRssi());
      }
      if(buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K'){
        if(buf[4] == '0'){
          GSE_ACK_last_time = micros();
        }//Show GSE has responded at this time
        if(buf[4] == '1'){AV1_ACK_last_time = micros();}//Show AV1 has responded at this time
        if(buf[4] == '2'){AV2_ACK_last_time = micros();}//Show AV2 has responded at this time
      }
      message_parser(buf);//Parse the message
      if(recv_ready){
        read_telem(buf[0], buf[1]);
        recv_ready = 0;
      }
      //No need for ACK code here :)
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}
void send_RFM() {
  if (micros()-RFM9x_send_last_time>RFM9x_send_rate && msg_ready > 0){
    rf95.send(message_send_buf, message_send_len);
    RFM9x_last_time = micros();
    msg_ready--;
  }
}
uint8_t radio_checksum(uint8_t* radioMSG, uint8_t msgLength) {//fletcher 8 checksum algorithm
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < msgLength; i++) {
      sum1 = (sum1 + radioMSG[i]) % 15;
      sum2 = (sum2 + sum1) % 15;
  }
  return (sum2 << 4) | sum1;
}
void message_parser(uint8_t* buf){
  //Identify message strucuture
  uint8_t msg_class = buf[0];
  uint8_t msg_id = buf[1]-1;
  uint8_t* msg_structure = nullptr;
  switch(msg_class){
    case 1:
      msg_structure = msg_class_01[msg_id];
      break;
    case 2:
      msg_structure = msg_class_02[msg_id];
      break;
    case 3:
      msg_structure = msg_class_03[msg_id];
      break;
  }

  uint8_t buffer_index = 2;
  //Parse using message structure
  for (uint8_t i = 0; i < 10; i++){
    switch(*(msg_structure+i)){//Obtains current datatype from message struct
      case 0:
        break;
      case 1:
        out_flag[i] = buf[buffer_index];
        buffer_index++;
        break;
      case 2:
        out_int8s[i] = buf[buffer_index];
        buffer_index++;
        break;
      case 3:
        out_int16s[i] = bytes_to_int_16((buf+buffer_index));
        buffer_index+=2;
        break;
      case 4:
        out_int32s[i] = bytes_to_int_32((buf+buffer_index));
        buffer_index+=4;
        break;
      case 5:
        out_floats[i] = bytes_to_float((buf+buffer_index));
        buffer_index+=4;
        break;
    }
  }
  if(buf[buffer_index] == radio_checksum(buf, buffer_index)){//Checksum verification
    recv_ready = 1;
  }
}
void message_assembler(uint8_t msg_class, uint8_t msg_id){
  message_send_buf[0] = msg_class;
  message_send_buf[1] = msg_id;
  uint8_t* msg_structure = nullptr;
  switch(msg_class){
    case 1:
      msg_structure = msg_class_01[msg_id-1];
      break;
    case 2:
      msg_structure = msg_class_02[msg_id-1];
      break;
    case 3:
      msg_structure = msg_class_03[msg_id-1];
      break;
  }

  uint8_t buffer_index = 2;
  //Parse using message structure
  for (uint8_t i = 0; i < 10; i++){
    switch(*(msg_structure+i)){//Obtains current datatype from message struct
      case 0:{
        break;
      }
      case 1:{
        uint8_t src = in_flag[i];
        memcpy(&message_send_buf[buffer_index], &src, sizeof(src));
        buffer_index++;
        break;
      }
      case 2:{
        uint8_t src = in_int8s[i];
        memcpy(&message_send_buf[buffer_index], &src, sizeof(src));
        buffer_index++;
        break;
      }
      case 3:{
        uint8_t src[2];
        int_16_to_bytes(in_int16s[i], src);
        memcpy(&message_send_buf[buffer_index], src, 2);
        buffer_index+=2;
        break;
      }
      case 4:{
        uint8_t src[4];
        int_32_to_bytes(in_int32s[i], src);
        memcpy(&message_send_buf[buffer_index], src, 4);
        buffer_index+=4;
        break;
      }
      case 5:{
        uint8_t src[4];
        float_to_bytes(in_floats[i], src);
        memcpy(&message_send_buf[buffer_index], src, 4);
        buffer_index+=4;
        break;
      }
    }
  }
  //Calculate checksum
  message_send_buf[buffer_index] = radio_checksum(message_send_buf, buffer_index);
  buffer_index++;
  message_send_len = buffer_index;
  msg_ready = 1;
}
//==========Data Type Conversion==========Alleon Oxales
//Passes pointers to character array and converts those bytes to another type
//and vise versa
void float_to_bytes(float value, uint8_t* out) {
    memcpy(out, &value, sizeof(value));
}

void int_16_to_bytes(uint16_t value, uint8_t* out) {
    memcpy(out, &value, sizeof(value));
}

void int_32_to_bytes(uint32_t value, uint8_t* out) {
    memcpy(out, &value, sizeof(value));
}

float bytes_to_float(uint8_t* data) { //Alleon Oxales, Converts bytes to float
  float num;
  memcpy(&num, data, sizeof(num));
  return num;
}
uint16_t bytes_to_int_16(uint8_t* data) { //Alleon Oxales, Converts bytes to int_16
  uint16_t num;
  memcpy(&num, data, sizeof(num));
  return num;
}
uint32_t bytes_to_int_32(uint8_t* data) { //Alleon Oxales, Converts bytes to int_32
  uint32_t num;
  memcpy(&num, data, sizeof(num));
  return num;
}
