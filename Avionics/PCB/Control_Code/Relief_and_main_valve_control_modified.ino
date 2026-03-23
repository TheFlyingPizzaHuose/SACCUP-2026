/*
  Relief_and_main_valve_control_modified.ino

  Purpose:
  - Receive command packets from Ground_Station_Nano using the same message parser / assembler style
  - Execute valve commands locally
  - Read one pressure transducer
  - Send telemetry back as Msg Class 0x02, Msg ID 0x04

  Notes:
  - Command mapping used here:
      0x04 -> open main valve
      0x05 -> close main valve
      0x08 -> open relief valve
      0x09 -> close relief valve
  - Telemetry mapping used here:
      Msg 0x02-0x04 structure = {4,5,5,5}
      in_int32s[0] = valve status array
      in_floats[1] = unused placeholder (0.0f)
      in_floats[2] = pressure transducer output
      in_floats[3] = battery voltage placeholder (0.0f)

  IMPORTANT:
  - Verify PRESSURE_TRANSDUCER_pin against the actual PCB / connector before soldering.
  - This code assumes valves are active-HIGH: HIGH=open, LOW=closed.
*/

#include <RH_RF95.h>
#include <SPI.h>
#include <ADC.h>
#include <ADC_util.h>
#include <string.h>
#include <SD.h>

//================ CONSTANT VARIABLES ================

// Valve MOSFET pin assignments
#define MAIN_VALVE_pin   2
#define RELIEF_VALVE_pin 3
#define DUMP_VALVE_pin   4

// RFM9X pin assignments
#define RFM9X_G0_pin   9
#define RFM9X_SCK_pin  13
#define RFM9X_MISO_pin 12
#define RFM9X_MOSI_pin 11
#define RFM9X_CS       10
#define RFM9X_RST      26
#define RFM9X_PWR      23

// Radio frequency to match Ground_Station_Nano current setup
float CONTROL_FREQ = 433.0f;

bool send_telem_requested = false;

// Pressure transducer input
// Verify this pin before soldering the aviation connector.
const uint8_t PRESSURE_TRANSDUCER_pin = 15;

// ADC object (same style as pad computer code)
ADC *adc = new ADC();

// Time between component usages in microseconds
const unsigned int RFM9x_rate = 1000000;
const unsigned int Pres_rate  = 50000;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM9X_CS, RFM9X_G0_pin);

//================ MESSAGE STRUCTURES ================
// Kept intentionally close to Ground_Station_Nano / pad computer format.

// Command message class
uint8_t msg_class_01[22][10] = {
  {1,1,1},   // 0x01 Ignition Abort
  {1,1,1},   // 0x02 Avionics & Pad Arm
  {1,1,1},   // 0x03 Ignition Sequence Start
  {1,1,1},   // 0x04 Main Valve Open (repurposed locally)
  {1,1,1},   // 0x05 Main Valve Closed (repurposed locally)
  {1,1,1},   // 0x06 N2 Fill Valve Open
  {1,1,1},   // 0x07 N2 Fill Valve Closed
  {1,1,1},   // 0x08 Relief Valve Open
  {1,1,1},   // 0x09 Relief Valve Closed
  {1,1,1},   // 0x0A Disconnect Rocket Fill Line
  {1,1,1},   // 0x0B Open Cooling Clamshell
  {1,1,1},   // 0x0C Enable TX
  {1,1,1},   // 0x0D Disable TX
  {1,1,1},   // 0x0E Tare Scale 1
  {1,1,1},   // 0x0F Tare Scale 2
  {1,1,1},   // 0x10 AV1 Inertial Align
  {1,1,1},   // 0x11 AV2 Inertial Align
  {1,1,1,5}, // 0x12 Set GSE Radio Frequency
  {1,1,1,5}, // 0x13 Set AV1 Radio Frequency
  {1,1,1,5}, // 0x14 Set AV2 Radio Frequency
  {1,1,1},   // 0x15 Set load cell channel A
  {1,1,1}    // 0x16 Set load cell channel B
};

uint8_t msg_class_02[6][10] = {
  {5,5,5,5,5,5,5,5,5,5}, // 0x01 AV1 Telemetry
  {5,5,5,5,5,5,5,5,5,5}, // 0x02 AV2 Telemetry
  {5,5,5,5,5,5,4},       // 0x03 GSE Temps, Presses, Masses
  {4,5,5,5},             // 0x04 Status bit array, pressures, battery voltage
  {4,5,5,5},             // 0x05 Status bit array, pressures, battery voltage
  {4}                    // 0x06 State array
};

uint8_t msg_class_03[7][10] = {
  {1,1,1}, // 0x01 No Igniter Continuity
  {1,1,1}, // 0x02 Quick Disconnect Fail
  {1,1,1}, // 0x03 Clamshell Fail
  {1,1,1}, // 0x04 GPS Lock Fail
  {4},     // 0x05 Rocket failed sensors
  {4},     // 0x06 GSE failed sensors
  {2}      // 0x07 Reset register value
};

//================ DYNAMIC VARIABLES ================

bool RFM9X_FAIL = false;
bool TX_enabled = true;

float pressure_output = 0.0f;
float battery_voltage_output = 0.0f; // Placeholder until battery sense is added

// Message parser output arrays
uint8_t  out_flag[10]   = {0};
uint8_t  out_int8s[10]  = {0};
uint16_t out_int16s[10] = {0};
uint32_t out_int32s[10] = {0};
float    out_floats[10] = {0};

// Message assembler input arrays
uint8_t  in_flag[10]   = {0};
uint8_t  in_int8s[10]  = {0};
uint16_t in_int16s[10] = {0};
uint32_t in_int32s[10] = {0};
float    in_floats[10] = {0};

// Message assembler buffer and state
uint8_t message_send_buf[43] = {0};
uint8_t message_send_len = 0;
uint8_t msg_ready = 0;
uint8_t recv_ready = 0;

// Component usage timers
unsigned int RFM9x_last_time = 0;
unsigned int Pressure_last_time = 0;

//================ VALVE STRUCT ================
struct Valve {
  uint8_t pin;
  const char* name;
  uint8_t state; // 0 = closed, 1 = open
};

Valve MAIN_VALVE   = {MAIN_VALVE_pin,   "main valve",   0};
Valve RELIEF_VALVE = {RELIEF_VALVE_pin, "relief valve", 0};
Valve DUMP_VALVE   = {DUMP_VALVE_pin,   "dump valve",   0};

//================ FUNCTION DECLARATIONS ================
void VALVE_open(Valve& valve);
void VALVE_close(Valve& valve);
void read_RFM();
void send_RFM();
void read_pressure();
void prep_telemetry();
void perform_command(uint8_t msg_class, uint8_t msg_id);
uint8_t radio_checksum(uint8_t* radioMSG, uint8_t msgLength);
void message_parser(uint8_t* buf);
void message_assembler(uint8_t msg_class, uint8_t msg_id);
float read_voltage(uint8_t pin);
void float_to_bytes(float value, uint8_t* out);
void int_16_to_bytes(uint16_t value, uint8_t* out);
void int_32_to_bytes(uint32_t value, uint8_t* out);
float bytes_to_float(uint8_t* data);
uint16_t bytes_to_int_16(uint8_t* data);
uint32_t bytes_to_int_32(uint8_t* data);
uint32_t compute_valve_status_array();

void debug_menu();
void debug_serial();
void print_hex_buffer(const uint8_t* buf, uint8_t len);
void print_local_status();
void run_valve_self_test();
void inject_local_command(uint8_t msg_id);

//================ SETUP ================
void setup() {
  Serial.begin(115200);

  // Valve outputs
  pinMode(MAIN_VALVE.pin, OUTPUT);
  pinMode(RELIEF_VALVE.pin, OUTPUT);
  pinMode(DUMP_VALVE.pin, OUTPUT);

  // Start in a known safe state
  VALVE_close(MAIN_VALVE);
  VALVE_close(RELIEF_VALVE);
  VALVE_close(DUMP_VALVE);

  // Pressure transducer input
  pinMode(PRESSURE_TRANSDUCER_pin, INPUT);

  // ADC setup copied in the same style as pad computer code
  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(16);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc0->recalibrate();

  #ifdef ADC_DUAL_ADCS
    adc->adc1->setAveraging(16);
    adc->adc1->setResolution(10);
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
    adc->adc1->recalibrate();
  #endif

  // Reset RFM9X
  pinMode(RFM9X_RST, OUTPUT);
  digitalWrite(RFM9X_RST, HIGH);
  delay(10);
  digitalWrite(RFM9X_RST, LOW);
  delay(10);
  digitalWrite(RFM9X_RST, HIGH);
  delay(10);

  SPI.begin();

  if (!rf95.init()) {
    Serial.println("RFM9X init failed");
    RFM9X_FAIL = true;
  } else {
    Serial.println("RFM9X init success");

    if (!rf95.setFrequency(CONTROL_FREQ)) {
      Serial.println("RFM9X setFrequency failed");
      RFM9X_FAIL = true;
    }

    rf95.setTxPower(RFM9X_PWR, false);
  }

  if (!rf95.init()) {
  Serial.println("RFM9X init failed");
  RFM9X_FAIL = true;
} else {
  Serial.println("RFM9X init success");

  if (!rf95.setFrequency(CONTROL_FREQ)) {
    Serial.println("RFM9X setFrequency failed");
    RFM9X_FAIL = true;
  }

  rf95.setTxPower(RFM9X_PWR, false);
}

// SD Create header once
  /*logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("time_ms, pressure, MAIN VALVE ST, RELIEF VALVE ST");
    logFile.close();
    Serial.println("Header written");
  } else {
    Serial.println("Could not open log.csv");
  }*/

Serial.println("PCB controller booted");
debug_menu();
}

//================ MAIN LOOP ================
void loop() {
  if(!RFM9X_FAIL && rf95.available()){
    read_RFM();       // mostly listen
  }else{
    //debug_serial();   // manual debug from Serial Monitor
    read_pressure();  // keep pressure updated
    //save_to_sd();     // sd
    prep_telemetry();
    send_RFM();
  }

  /*if (send_telem_requested) { //is send requested
    send_telem_requested = false;
  }*/
}

//================ VALVES CONTROL ================
void VALVE_open(Valve& valve) {
  digitalWrite(valve.pin, HIGH);
  valve.state = 1;
  Serial.print(valve.name);
  Serial.println(" valve: OPEN");
}

void VALVE_close(Valve& valve) {
  digitalWrite(valve.pin, LOW);
  valve.state = 0;
  Serial.print(valve.name);
  Serial.println(" valve: CLOSED");
}

//================ ANALOG / PRESSURE CODE ================
// Same style as pad computer code.
float read_voltage(uint8_t pin) {
  int value = adc->adc0->analogRead(pin);
  float Vout = value * 3.3f / adc->adc0->getMaxValue();
  return Vout;
}

void read_pressure() {
  if (micros() - Pressure_last_time > Pres_rate) {
    float voltage = read_voltage(PRESSURE_TRANSDUCER_pin);

    // Calibration copied from pad computer code.
    // Replace with final calibration when available.
    pressure_output = 1.15 * 1000.0 * voltage - 425;

    Pressure_last_time = micros();
  }
}

//================ TELEMETRY CODE ================
uint32_t compute_valve_status_array() {
  // User-requested simplified encoding:
  // 0 = no valves open
  // 1 = relief valve open
  // 2 = main valve open
  // 3 = both open
  uint32_t status = 0;

  if (RELIEF_VALVE.state) {
    status |= 0x01;
  }
  if (MAIN_VALVE.state) {
    status |= 0x02;
  }

  return status;
}

void prep_telemetry() {
  // Match Msg 0x02-0x04 structure: {4,5,5,5}
  // index 0 -> int32 state array
  // index 1 -> float placeholder
  // index 2 -> pressure transducer output
  // index 3 -> battery voltage placeholder
  in_int32s[0] = compute_valve_status_array();
  in_floats[1] = 0.0f;
  in_floats[2] = pressure_output;
  in_floats[3] = battery_voltage_output;

  message_assembler(0x02, 0x04);
}

//================ RADIO CODE ================
// Kept close to Ground_Station_Nano
void read_RFM() {
  if (!RFM9X_FAIL && rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      message_parser(buf);

      
        perform_command(buf[0], buf[1]);
        print_local_status();
        send_telem_requested = true;
        recv_ready = 0;
      
    }
  }
}

void send_RFM() {
  if (!RFM9X_FAIL && micros()-RFM9x_last_time > RFM9x_rate && msg_ready) {
    rf95.send(message_send_buf, message_send_len);
    Serial.println("sent back");
    msg_ready = 0;
    RFM9x_last_time = micros();
  }
}

uint8_t radio_checksum(uint8_t* radioMSG, uint8_t msgLength) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < msgLength; i++) {
    sum1 = (sum1 + radioMSG[i]) % 15;
    sum2 = (sum2 + sum1) % 15;
  }
  return (sum2 << 4) | sum1;
}

void message_parser(uint8_t* buf) {
  uint8_t msg_class = buf[0];
  uint8_t msg_id_index = buf[1] - 1;
  uint8_t* msg_structure = nullptr;

  switch (msg_class) {
    case 1:
      if (msg_id_index < 22) msg_structure = msg_class_01[msg_id_index];
      break;
    case 2:
      if (msg_id_index < 6) msg_structure = msg_class_02[msg_id_index];
      break;
    case 3:
      if (msg_id_index < 7) msg_structure = msg_class_03[msg_id_index];
      break;
    default:
      break;
  }

  if (msg_structure == nullptr) {
    recv_ready = 0;
    return;
  }

  uint8_t buffer_index = 2;

  for (uint8_t i = 0; i < 10; i++) {
    switch (*(msg_structure + i)) {
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
        out_int16s[i] = bytes_to_int_16((buf + buffer_index));
        buffer_index += 2;
        break;
      case 4:
        out_int32s[i] = bytes_to_int_32((buf + buffer_index));
        buffer_index += 4;
        break;
      case 5:
        out_floats[i] = bytes_to_float((buf + buffer_index));
        buffer_index += 4;
        break;
    }
  }

  if (buf[buffer_index] == radio_checksum(buf, buffer_index)) {
    recv_ready = 1;
  } else {
    recv_ready = 0;
  }
}

void message_assembler(uint8_t msg_class, uint8_t msg_id) {
  message_send_buf[0] = msg_class;
  message_send_buf[1] = msg_id;
  uint8_t* msg_structure = nullptr;

  switch (msg_class) {
    case 1:
      if ((msg_id - 1) < 22) msg_structure = msg_class_01[msg_id - 1];
      break;
    case 2:
      if ((msg_id - 1) < 6) msg_structure = msg_class_02[msg_id - 1];
      break;
    case 3:
      if ((msg_id - 1) < 7) msg_structure = msg_class_03[msg_id - 1];
      break;
    default:
      break;
  }

  if (msg_structure == nullptr) {
    msg_ready = 0;
    return;
  }

  uint8_t buffer_index = 2;

  for (uint8_t i = 0; i < 10; i++) {
    switch (*(msg_structure + i)) {
      case 0: {
        break;
      }
      case 1: {
        uint8_t src = in_flag[i];
        memcpy(&message_send_buf[buffer_index], &src, sizeof(src));
        buffer_index++;
        break;
      }
      case 2: {
        uint8_t src = in_int8s[i];
        memcpy(&message_send_buf[buffer_index], &src, sizeof(src));
        buffer_index++;
        break;
      }
      case 3: {
        uint8_t src[2];
        int_16_to_bytes(in_int16s[i], src);
        memcpy(&message_send_buf[buffer_index], src, 2);
        buffer_index += 2;
        break;
      }
      case 4: {
        uint8_t src[4];
        int_32_to_bytes(in_int32s[i], src);
        memcpy(&message_send_buf[buffer_index], src, 4);
        buffer_index += 4;
        break;
      }
      case 5: {
        uint8_t src[4];
        float_to_bytes(in_floats[i], src);
        memcpy(&message_send_buf[buffer_index], src, 4);
        buffer_index += 4;
        break;
      }
    }
  }

  message_send_buf[buffer_index] = radio_checksum(message_send_buf, buffer_index);
  buffer_index++;
  message_send_len = buffer_index;
  msg_ready = 1;
}

//================ COMMAND EXECUTION ================
void perform_command(uint8_t msg_class, uint8_t msg_id) {
  if (msg_class == 0x01) {
    if (out_flag[0] == 0xFF && out_flag[1] == 0xFF && out_flag[2] == 0xFF) {
      switch (msg_id) {
        case 0x04: // open main valve
          VALVE_open(MAIN_VALVE);
          break;

        case 0x05: // close main valve
          VALVE_close(MAIN_VALVE);
          break;

        case 0x08: // open relief valve
          VALVE_open(RELIEF_VALVE);
          break;

        case 0x09: // close relief valve
          VALVE_close(RELIEF_VALVE);
          break;

        case 0x0C: // enable TX
          TX_enabled = true;
          break;

        case 0x0D: // disable TX
          TX_enabled = false;
          break;

        case 0x12: // set local radio frequency from float payload
          CONTROL_FREQ = out_floats[3];
          rf95.setFrequency(CONTROL_FREQ);
          break;

        default:
          break;
      }
    }
  }

  // Clear verify flags after command handling so they must be resent.
  out_flag[0] = 0;
  out_flag[1] = 0;
  out_flag[2] = 0;
}

//================ DATA TYPE CONVERSION ================
void float_to_bytes(float value, uint8_t* out) {
  memcpy(out, &value, sizeof(value));
}

void int_16_to_bytes(uint16_t value, uint8_t* out) {
  memcpy(out, &value, sizeof(value));
}

void int_32_to_bytes(uint32_t value, uint8_t* out) {
  memcpy(out, &value, sizeof(value));
}

float bytes_to_float(uint8_t* data) {
  float num;
  memcpy(&num, data, sizeof(num));
  return num;
}

uint16_t bytes_to_int_16(uint8_t* data) {
  uint16_t num;
  memcpy(&num, data, sizeof(num));
  return num;
}

uint32_t bytes_to_int_32(uint8_t* data) {
  uint32_t num;
  memcpy(&num, data, sizeof(num));
  return num;
}

//==================== SD ============================
void save_to_sd() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    File logFile = SD.open("log.csv", FILE_WRITE);

    if (logFile) {
      logFile.print(millis());
      logFile.print(" | ");
      logFile.print(pressure_output, 3);
      logFile.print(" | ");
      logFile.print(MAIN_VALVE.state);
      logFile.print(" | ");
      logFile.print(RELIEF_VALVE.state);
      logFile.close();
    } else {
      Serial.println("SD open failed");
    }
  }
}

// degub =======================================================
void debug_menu() {
  Serial.println();
  Serial.println("===== PCB DEBUG MENU =====");
  Serial.println("m = print this menu");
  Serial.println("s = print local status");
  Serial.println("v = run valve self-test");
  Serial.println("p = read and print pressure");
  Serial.println("t = build telemetry and print raw packet");
  Serial.println("4 = simulate command 0x04 (OPEN MAIN)");
  Serial.println("5 = simulate command 0x05 (CLOSE MAIN)");
  Serial.println("8 = simulate command 0x08 (OPEN RELIEF)");
  Serial.println("9 = simulate command 0x09 (CLOSE RELIEF)");
  Serial.println("==========================");
  Serial.println();
}

void print_hex_buffer(const uint8_t* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void print_local_status() {
  Serial.println();
  Serial.println("----- LOCAL STATUS -----");

  Serial.print("MAIN valve state   = ");
  Serial.println(MAIN_VALVE.state);

  Serial.print("RELIEF valve state = ");
  Serial.println(RELIEF_VALVE.state);

  Serial.print("DUMP valve state   = ");
  Serial.println(DUMP_VALVE.state);

  Serial.print("Pressure output    = ");
  Serial.println(pressure_output);

  Serial.print("Valve status array = ");
  Serial.println(compute_valve_status_array());

  Serial.print("TX enabled         = ");
  Serial.println(TX_enabled);

  Serial.print("RFM9X_FAIL         = ");
  Serial.println(RFM9X_FAIL);

  Serial.print("CONTROL_FREQ       = ");
  Serial.println(CONTROL_FREQ);

  Serial.println("------------------------");
  Serial.println();
}

void run_valve_self_test() {
  Serial.println("Running valve self-test...");

  VALVE_close(MAIN_VALVE);
  VALVE_close(RELIEF_VALVE);
  delay(500);

  Serial.println("Opening MAIN");
  VALVE_open(MAIN_VALVE);
  print_local_status();
  delay(1000);

  Serial.println("Closing MAIN");
  VALVE_close(MAIN_VALVE);
  print_local_status();
  delay(1000);

  Serial.println("Opening RELIEF");
  VALVE_open(RELIEF_VALVE);
  print_local_status();
  delay(1000);

  Serial.println("Closing RELIEF");
  VALVE_close(RELIEF_VALVE);
  print_local_status();
  delay(1000);

  Serial.println("Valve self-test complete.");
}

void inject_local_command(uint8_t msg_id) {
  // Simulate a valid decoded radio command locally
  out_flag[0] = 0xFF;
  out_flag[1] = 0xFF;
  out_flag[2] = 0xFF;

  Serial.print("Injecting local command 0x");
  Serial.println(msg_id, HEX);

  perform_command(0x01, msg_id);
  print_local_status();
}

void debug_serial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  switch (c) {
    case 'm':
      debug_menu();
      break;

    case 's':
      print_local_status();
      break;

    case 'v':
      run_valve_self_test();
      break;

    case 'p':
      read_pressure();
      Serial.print("Pressure output = ");
      Serial.println(pressure_output);
      break;

    case 't':
      prep_telemetry();
      Serial.print("Telemetry length = ");
      Serial.println(message_send_len);
      Serial.print("Telemetry packet = ");
      print_hex_buffer(message_send_buf, message_send_len);
      print_local_status();
      send_telem_requested = true;
      break;

    case '4':
      inject_local_command(0x04);
      break;

    case '5':
      inject_local_command(0x05);
      break;

    case '8':
      inject_local_command(0x08);
      break;

    case '9':
      inject_local_command(0x09);
      break;

    default:
      Serial.print("Unknown debug key: ");
      Serial.println(c);
      debug_menu();
      break;
  }
}


