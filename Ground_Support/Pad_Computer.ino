/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Pad Computer Main Code
Authors: Alleon Oxales

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/1AqIgfhQb1Wmkl7yFG0nSHRvpzL-viViXBCPfHbvtkt0/edit?usp=sharing
*/
#include <RH_RF95.h> // Include RFM9X library
#include <SPI.h>               // SPI library
#include <map>
#include <string>
#include <HX711.h>//https://github.com/bogde/HX711
#include <ADC.h>
#include <ADC_util.h>
#include <SD.h>                // SD card library

//===============Constant Variables===============

//SD card variables
#define SD_CS 5
File logfile;

//RFM9x pin assignments
#define RFM95_CS 10
#define RFM95_INT 8
#define RFM95_RST 9
#define GSE_FREQ 432.92
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS,RFM95_INT);
const int RFM95_PWR = 23;

//Valve MOSFET pin assignments
#define N2O_pin 7

//Command Message Class
u_int8_t msg_class_01[13][10] = {{1,1,1}, //Ignition Abort
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
u_int8_t msg_class_02[3][10] = {{5,5,5,5,5,5,5,5,5,5}, //AV1 Telemetry
                                      {5,5,5,5,5,5,5,5,5,5}, //AV2 Telemetry
                                      {5,5,5,5,5,2,2,2,2,2}, //GSE Temps, Presses, Rocket Mass and Status
                                      };
                              
//Alert Message Class
u_int8_t msg_class_03[7][10] = {{1,1,1}, //No Igniter Continuity
                                      {1,1,1}, //Quick Disconnect Fail
                                      {1,1,1}, //Clamshell Fail
                                      {1,1,1}, //GPS Lock Fail
                                      {4}, //Rocket List of Failed Sensors
                                      {4}, //GSE List of Failed Sensors
                                      {2}, //Rocket Computer Reset Register Value
                                      };

//Time between component usages in micros
const uint RFM9x_rate = 50000;
const uint SD_rate = 50000;
const uint Pres_rate = 50000;
const uint Temp_rate = 50000;
const uint HX711_rate = 100000;

//HX711 pin assignments
#define DOUT 3
#define CLK 2
const float load_cell_scale = 2280;
HX711 scale; //HX711 library object

ADC *adc = new ADC(); // adc object

#if defined(ADC_TEENSY_4) // Teensy 4
#define PINS 14
#define DIG_ADC_0_PINS 10
#define DIG_ADC_1_PINS 10
#define PINS_DIFF 0
uint8_t adc_pins_dig_ADC_0[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
uint8_t adc_pins_dig_ADC_1[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
uint8_t adc_pins_diff[] = {};

#elif defined(ADC_TEENSY_4_1) // Teensy 4.1
#define PINS 18
#define DIG_ADC_0_PINS 10
#define DIG_ADC_1_PINS 10
#define PINS_DIFF 0
uint8_t adc_pins_dig_ADC_0[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
uint8_t adc_pins_dig_ADC_1[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
uint8_t adc_pins_diff[] = {};
#endif // 

#ifdef ADC_TEENSY_4
const uint32_t num_samples = 10000;
#else
const uint32_t num_samples = 100;
#endif

const uint8_t pressure_1_pin = A1;
const float press_1_R1 = 680; //Ohm
const float press_1_R2 = 330; //Ohm
const float pressure_1_max = 2500; //PSI
const float press_1_ratio = pressure_1_max*(press_1_R1+press_1_R2)/press_1_R2;

const uint8_t pressure_2_pin = A2;
const float press_2_R1 = 680; //Ohm
const float press_2_R2 = 330; //Ohm
const float pressure_2_max = 2500; //PSI
const float press_2_ratio = pressure_1_max*(press_2_R1+press_2_R2)/press_2_R2;

const uint8_t temp_1_pin = A3;
const float temp_1_R = 47800; //Ohm

const uint8_t temp_2_pin = A4;
const float temp_2_R = 46500; //Ohm
//===============Dynamic Variables===============
char* log_file_name;

//Valve states
bool N2O_valve_state = 0;
bool N2_valve_state = 0;
bool Poppet_valve_state = 0;

//Message parser output arrays
u_int8_t out_flag[10]; 
u_int8_t out_int8s[10];
u_int16_t out_int16s[10];
u_int32_t out_int32s[10];
float out_floats[10];

//Times of last component usages
uint RFM9x_last_time = 0;
uint SD_last_time = 0;
uint HX711_last_time = 0;
uint Pressure_last_time = 0;
uint Temp_last_time = 0;

std::map<std::string, bool> error_status = {
    {"PRGM_ERROR", false},
    {"SAMM8Q_ERR_DAT", false},
    {"MAIN_PWR_FAULT", false},
    {"SAMM8Q_FAIL", false},
    {"SD_FAIL", false},
    {"RFM9X_FAIL", false},
    {"HX711_FAIL", false}
};

//Sensor outputs
float load_cell_output = 0;
float pressure_1_output = 0;
float pressure_2_output = 0;
float temp_1_output = 0;
float temp_2_output = 0;

//Message assembler input arrays
uint8_t in_flag[10] ; 
uint8_t in_int8s[10] ;
uint16_t in_int16s[10] ;
uint32_t in_int32s[10] ;
float in_floats[10] ;

//Message assembler buffer and message length
uint8_t message_send_buf[43];
uint8_t message_send_len = 0;

char* check_file_on_SD(bool mode = 0) {  //Alleon Oxales
  int fileExists = 1;
  int fileNum = 0;
  static char fileName[8] = "000.txt";
  while (fileExists) {
    fileName[0] = '0' + fileNum / 100;
    fileName[1] = '0' + (fileNum % 100) / 10;
    fileName[2] = '0' + fileNum % 10;
    File checkfile = SD.open(fileName);
    checkfile.seek(1 * 6);
    String content = checkfile.readStringUntil('\r');
    if (content != "") {
      fileNum++;
    } else {
      fileExists = 0;
    }
  }
  if (mode) {
    if (fileNum > 0) { fileNum--; }
    fileName[0] = '0' + fileNum / 100;
    fileName[1] = '0' + (fileNum % 100) / 10;
    fileName[2] = '0' + fileNum % 10;
  }
  return fileName;
}

void setup() {
  pinMode(N2O_pin, OUTPUT);
  Serial.begin(38400);
  Serial.print("Reset Register Value: ");
  Serial.println(SRC_SRSR);
  if(SRC_SRSR != 1){}// Read reset status register and PRGM_ERR if reset is not a power cycle
  //===SD Card INIT===
  if (!SD.begin(SD_CS)) { error_status["SD_FAIL"] = true; }  //Init SD, this comes first to be able to check the latest log file
  logfile = SD.open(check_file_on_SD(), FILE_WRITE);  //Opens new file with highest index
  /*
  //===RFM9x RADIO INIT===
  pinMode(RFM95_RST, OUTPUT);
  Serial.println("Reset RFM9x");

  //Manual reset of RFM9x
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  */
  if (!rf95.init()){
    Serial.println("init failed");
    error_status["RFM9X_FAIL"] = true; 
  }else{
    Serial.println("init sucess");
    rf95.setFrequency(GSE_FREQ);
    rf95.setTxPower(RFM95_PWR, false);
  }

  //===HX711 LOAD CELL AMP INIT===
  scale.begin(DOUT, CLK);
  scale.set_scale(load_cell_scale);
  scale.tare();

  //===ADC0 SETUP===
  adc->adc0->setAveraging(16);                                    // set number of averages
  adc->adc0->setResolution(10);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed
  adc->adc0->recalibrate();

  //===ADC1 SETUP===
  #ifdef ADC_DUAL_ADCS
    adc->adc1->setAveraging(16);                                    // set number of averages
    adc->adc1->setResolution(10);                                   // set bits of resolution
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed
    adc->adc1->recalibrate();
  #endif
}

void loop()
{
  read_RFM();
  read_load_cell();
  
}
//==========COMMAND CODE==========Alleon Oxales
void perform_command(uint8_t msg_class, uint8_t msg_id){
  if(msg_class == 1){
    switch(msg_id){
      case 0x0B:
        tare_load_cell();
        break;
      default:
        break;
    }
  }
}
//==========LOAD CELL CODE==========Alleon Oxales
void read_load_cell(){
  if(!error_status["HX711_FAIL"] && (micros()-HX711_last_time > HX711_rate)){
    load_cell_output = scale.get_units(10); //Take 10 average samples and submit weight
  }
  logfile.print(micros());
  logfile.print("|5|");
  logfile.println(load_cell_output);
  Serial.print("|5|");
  Serial.println(load_cell_output);
}
void tare_load_cell(){
  scale.tare();
}

//==========SD CARD CODE==========
void write_to_SD(){
  if (micros() - SD_last_time> SD_rate && !error_status["SD_FAIL"]){
        logfile.flush();
  }
}

//==========ANALOG READ + PRES + TEMP CODE==========
float read_voltage(uint8_t pin) {//Kartik Function to read true voltage from a voltage divider
  // Read raw ADC value from the pin
  int value = adc->adc0->analogRead(pin); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.

  // Convert ADC value to actual measured voltage at the pin
  float Vout = value*3.3/adc->adc0->getMaxValue();
  // 3.3 is VREF for teensy
  return Vout;
}
void read_pressure_1(){//Alleon Oxales
  float voltage = read_voltage(pressure_1_pin);
  pressure_1_output = press_1_ratio*voltage;
  logfile.print(micros());
  logfile.print("|1|");
  logfile.println(pressure_1_output);
  Serial.print("|1|");
  Serial.println(pressure_1_output);
}
void read_pressure_2(){//Alleon Oxales
  float voltage = read_voltage(pressure_2_pin);
  pressure_2_output = press_2_ratio*voltage;
  logfile.print(micros());
  logfile.print("|2|");
  logfile.println(pressure_2_output);
}
void read_temp_1(){//Alleon Oxales
  float voltage = read_voltage(temp_1_pin);
  float temp_resistance = (5*temp_1_R/voltage)*temp_1_R;
  temp_1_output = temp_resistance;
  logfile.print(micros());
  logfile.print("|3|");
  logfile.println(temp_1_output);
}
void read_temp_2(){//Alleon Oxales
  float voltage = read_voltage(temp_2_pin);
  float temp_resistance = (5*temp_2_R/voltage)*temp_2_R;
  temp_2_output = temp_resistance;
  logfile.print(micros());
  logfile.print("|4|");
  logfile.println(temp_2_output);
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
      message_parser(buf);//Parse the message
      perform_command(buf[0], buf[1]);

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
void send_RFM() {
  rf95.send(message_send_buf, message_send_len);
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