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
#include <map>
#include <string>

//===============Constant Variables===============
//RFM9x pin assignments
#define RFM95_CS    4
#define RFM95_INT   3
#define RFM95_RST   2
// Different Computer Frequencies
#define GSE_FREQ 432.92
#define AV1_FREQ 433.0
#define AV2_FREQ 433.08

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const int RFM95_PWR = 23;

//Command Message Class
uint8_t msg_class_01[17][10] = {{1,1,1}, //Ignition Abort
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
                              };

//Telemetry Message Class
uint8_t msg_class_02[6][10] = {{5,5,5,5,5,5,5,5,5,5}, //AV1 Telemetry
                              {5,5,5,5,5,5,5,5,5,5}, //AV2 Telemetry
                              {5,5,5,5,5}, //GSE Temps, Presses, Supply and Rocket Mass
                              {4}, //AV1 Detected Flight Events
                              {4}, //AV2 Detected Flight Events
                              {4} //GSE States
                              };
                              
//Alert Message Class
uint8_t msg_class_03[7][10] = {{1,1,1}, //No Igniter Continuity
                              {1,1,1}, //Quick Disconnect Fail
                              {1,1,1}, //Clamshell Fail
                              {1,1,1}, //GPS Lock Fail
                              {4}, //Rocket List of Failed Sensors
                              {4}, //GSE List of Failed Sensors
                              {2} //Rocket Computer Reset Register Value
                              };

//Time between component usages in micros
const uint RFM9x_rate = 50000;
const uint SD_rate = 50000;
//const uint PC_rate = 40000; <Not needed cuz PC telem gets sent everytime telem is recieved.

//===============Dynamic Variables===============

//3 arrays w/ 100 bools store if previous RSSI greater than -50
//Indecies also present, used when writing and index get's reset at i=99
bool GSE_RSSI[100];
uint8_t GSE_RSSI_index = 0;
bool AV1_RSSI[100];
uint8_t AV1_RSSI_index = 0;
bool AV2_RSSI[100];
uint8_t AV2_RSSI_index = 0;

//Message parser output arrays
u_int8_t out_flag[10]; 
u_int8_t out_int8s[10];
u_int16_t out_int16s[10];
u_int32_t out_int32s[10];
float out_floats[10];

//Times of last component usages
const uint RFM9x_last_time = 0;
const uint SD_last_time = 0;

//Frequency selection index 0:GSE, 1:AV1, 2:AV2 & time since last ACK
uint GSE_ACK_last_time = 0;
uint AV1_ACK_last_time = 0;
uint AV2_ACK_last_time = 0;

std::map<std::string, bool> error_status = {
    {"PRGM_ERROR", false},
    {"ALT_OUT_RANGE", false},
    {"LAT_OUT_RANGE", false},
    {"VEL_OUT", false},
    {"ORIENT_X_OUT_RANGE", false},
    {"ORIENT_Y_OUT_RANGE", false},
    {"BMP280_ERR_DAT", false},
    {"SAMM8Q_ERR_DAT", false},
    {"MPU6050_ERR_DAT", false},
    {"BMP180_1_ERR_DAT", false},
    {"BMP180_2_ERR_DAT", false},
    {"AS5600_1_ERR_DAT", false},
    {"AS5600_2_ERR_DAT", false},
    {"TELEM_PWR_FAULT", false},
    {"VIDEO_PWR_FAULT", false},
    {"MAIN_PWR_FAULT", false},
    {"BMP280_FAIL", false},
    {"SAMM8Q_FAIL", false},
    {"MPU6050_FAIL", false},
    {"BMP180_1_FAIL", false},
    {"BMP180_2_FAIL", false},
    {"AS5600_1_FAIL", false},
    {"AS5600_2_FAIL", false},
    {"SD_FAIL", false},
    {"RFD900_FAIL", false},
    {"RFM9X_FAIL", false},
    {"ADXL375_FAIL", false},
    {"LSM9SD1_FAIL", false},
    {"INA219_FAIL", false},
    {"ACCEL_CALLIB", false},
    {"MAG_CALLIB", false}
};

//Ground station to PC telemetry variables
float pc_telem[21] = {
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
  0, //15 AV1 Battery Voltage
  0, //16 AV2 Battery Voltage
  0, //17 N2O Valve State
  0, //18 N2 Valve State
  0, //19 Quick Disconnect State
  0  //20 Clamshell State
};
size_t pc_telem_length = sizeof(pc_telem)/sizeof(pc_telem[0]);
//Message assembler input arrays
uint8_t in_flag[10] ; 
uint8_t in_int8s[10] ;
uint16_t in_int16s[10] ;
uint32_t in_int32s[10] ;
float in_floats[10] ;

//Message assembler buffer and message length
uint8_t message_send_buf[43];
uint8_t message_send_len = 0;

void setup() {   
  Serial.begin(38400);

  // Save copy of Reset Status Register
  uint32_t lastResetCause = SRC_SRSR;
  // Clear all Reset Status Register bits
  SRC_SRSR = (uint32_t)0x1FF;

  // ... more setup steps, get USB ready ...
  resetCause(lastResetCause);

  pinMode(RFM95_RST, OUTPUT);
  Serial.println("Reset RFM9x");

  //Manual reset of RFM9x
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  if (!rf95.init()){
    Serial.println("RFM9x init failed");  
  }else{
    Serial.println("RFM9x init sucess");
    rf95.setFrequency(GSE_FREQ);
    rf95.setTxPower(RFM95_PWR, false);
  }
}

void loop() {
  if(Serial.available()){
    uint8_t data[] = {'0'};
    Serial.println("Sending  command");
    // Send a message to rf95_server
    rf95.send(data, sizeof(data));
    
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(50))
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
    {
        Serial.print("got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else{Serial.println("No ACK");}
    }
    else{Serial.println("No ACK");}
  }
  read_RFM();
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
      pc_telem[6] = out_floats[6];//Quaternion X
      pc_telem[7] = out_floats[7];//Quaternion Y
      pc_telem[8] = out_floats[8];//Quaternion Z
      pc_telem[9] = out_floats[9];//Quaternion W
    }
    if(msg_id == 0x03){//GSE telemetry
      pc_telem[7] = out_floats[0];//Supply Temp
      pc_telem[8] = out_floats[1];//Rocket Temp
      pc_telem[9] = out_floats[2];//N2O Line Pres
      pc_telem[10] = out_floats[3];//N2 Line Pres
      pc_telem[11] = out_floats[4];//Supply Mass
      pc_telem[12] = out_floats[5];//Rocket Mass
    }
    if(msg_id == 0x6){
      pc_telem[17] = static_cast<float>((out_int32s[0] >> 0) & 1); //N2O Valve State
      pc_telem[18] = static_cast<float>((out_int32s[0] >> 1) & 1); //N2 Valve State
      pc_telem[19] = static_cast<float>((out_int32s[0] >> 2) & 1); //Quick Disconnect State
      pc_telem[20] = static_cast<float>((out_int32s[0] >> 3) & 1); //Clamshell State
    }
  }
  for(size_t i = 0; i < pc_telem_length; i++){
    Serial.print(pc_telem[i]);
    if(i+1 < pc_telem_length){
      Serial.print(',');
    }
  }
  Serial.println();
  Serial.flush();
}

//==========RADIO CODE==========Alleon Oxales
void read_RFM() {
  if (!error_status["RFM9X_FAIL"] && rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      if(buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K'){
        if(buf[4] == '0'){GSE_ACK_last_time = micros();}//Show GSE has responded at this time
        if(buf[4] == '1'){AV1_ACK_last_time = micros();}//Show AV1 has responded at this time
        if(buf[4] == '2'){AV2_ACK_last_time = micros();}//Show AV2 has responded at this time
      }
      message_parser(buf);//Parse the message
      read_telem(buf[0], buf[1]);
      //No need for ACK code here :)
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}
void send_RFM() {
  /*if ((micros() - RFM_LAST > RFM_RATE) && !errorCodes[RFM9X_FAIL]) {
    char* message = readyPacket();
    //itoa(packetnum++, radiopacket+13, 10);
    //Serial.print("Sending "); Serial.println(radiopacket);
    //radiopacket[19] = 0;

    //Serial.println("Sending...");
    //delay(10);
    rf95.send(message, charArrayLegnth);

    //Serial.println("Waiting for packet to complete...");
    //delay(10);
    //rf95.waitPacketSent();
    RFM_LAST = micros();
  }*/
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
  uint8_t msg_id = buf[1];
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
}
void message_assembler(uint8_t msg_class, uint8_t msg_id){
  message_send_buf[0] = msg_class;
  message_send_buf[1] = msg_id;
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

// i.MX RT1060 Processor Reference Manual, 21.8.3 SRC Reset Status Register
// Credit to jrw member of pjrc forum for code:
// https://forum.pjrc.com/index.php?threads/how-to-read-last-reset-reason-for-teensy-4-1.66654/#:~:text=/*%20======,R
void resetCause(uint32_t resetStatusReg) {
    bool info = false;

    if (resetStatusReg & SRC_SRSR_TEMPSENSE_RST_B) {
        Serial.println("Temperature Sensor Software Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_WDOG3_RST_B) {
        Serial.println("IC Watchdog3 Timeout Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_JTAG_SW_RST) {
        Serial.println("JTAG Software Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_JTAG_RST_B) {
        Serial.println("High-Z JTAG Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_WDOG_RST_B) {
        Serial.println("IC Watchdog Timeout Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_IPP_USER_RESET_B) {
        Serial.println("Power-up Sequence (Cold Reset Event)");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_CSU_RESET_B) {
        Serial.println("Central Security Unit Reset");
        info = true;
    }
    if (resetStatusReg & SRC_SRSR_LOCKUP_SYSRESETREQ) {
        Serial.println("CPU Lockup or Software Reset");
        info = true;
        /* Per datasheet: "SW needs to write a value to SRC_GPR5
         * before writing the SYSRESETREQ bit and use the SRC_GPR5
         * value to distinguish if the reset is caused by SYSRESETREQ
         * or CPU lockup."
         */
    }
    if (resetStatusReg & SRC_SRSR_IPP_RESET_B) {
        Serial.println("Power-up Sequence");
        info = true;
    }
    if (!info) {
        Serial.println("No status bits set in SRC Reset Status Register");
    }
}