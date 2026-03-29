/*
  SYS_Alan_V.3.2.ino

  Purpose:
  - Keep the existing message parser / assembler structure
  - Read 4 pressure transducers according to the pins at the top of the file
  - Read LSM6DSOX and ADXL375 locally (no radio for IMUs yet)
  - Keep radio working in the same parser / assembler style
  - Keep debug menu, local status, and manual / radio command functions
  - Add ability to send a raw check message such as "PCB online" to the ground station
  - Record a table log on the built-in SD card

  Telemetry used from the avionics datasheet:
  - Msg Class 0x02, Msg ID 0x03  -> pressures packet
  - Msg Class 0x02, Msg ID 0x06  -> valve states packet
*/
// Librarys
  #include <RH_RF95.h>
  #include <SPI.h>
  #include <ADC.h>
  #include <ADC_util.h>
  #include <string.h>
  #include <SD.h>
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_LSM6DSOX.h>
  #include <Adafruit_ADXL375.h>

//================ CONSTANT VARIABLES ================

  // Valve MOSFET pin assignments
  #define MAIN_VALVE_pin   2
  #define RELIEF_VALVE_pin 3
  #define DUMP_VALVE_pin   4

  // Pressure transducer inputs
  const uint8_t PRESSURE_TRANSDUCER_1_pin = 41;
  const uint8_t PRESSURE_TRANSDUCER_2_pin = 40;
  const uint8_t PRESSURE_TRANSDUCER_3_pin = 15;
  const uint8_t PRESSURE_TRANSDUCER_4_pin = 27;

  // RFM9X pin assignments
  #define RFM9X_G0_pin   9
  #define RFM9X_SCK_pin  13
  #define RFM9X_MISO_pin 12
  #define RFM9X_MOSI_pin 11
  #define RFM9X_CS       10
  #define RFM9X_RST      26
  #define RFM9X_PWR      23

  // LSM6DSOX pin assignments
  #define LSM6DSOX_SCL_pin 19
  #define LSM6DSOX_SDA_pin 18

  // ADXL375 pin assignments
  #define ADXL375_SCL_pin 16
  #define ADXL375_SDA_pin 16

  // DS1307 ass IGNORE THIS ONE FOR NOW
  #define DS1307_SDA_pin  18
  #define DS1307_SCL_pin  19

  // Radio frequency to match Ground_Station_Nano current setup
  float CONTROL_FREQ = 433.0f;

  // ADC object (same style as pad computer code)
  ADC *adc = new ADC();

  // SD file
  File logFile;

  // Time between component usages in microseconds
  const unsigned int RFM9x_rate    = 200000;
  const unsigned int Pres_rate     = 50000;
  const unsigned int LSM6DSOX_rate = 5000;
  const unsigned int ADXL375_rate  = 5000;
  const unsigned int SD_rate       = 100000;

  // Singleton instance of the radio driver
  RH_RF95 rf95(RFM9X_CS, RFM9X_G0_pin);

  // Sensor objects
  Adafruit_LSM6DSOX lsm6dsox;
  Adafruit_ADXL375  adxl375 = Adafruit_ADXL375(12345);

//================ MESSAGE STRUCTURES ================

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
    {5,5,5,5,5,5,4},       // 0x03 GSE Temps, Presses, Masses, Command count
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

  bool RFM9X_FAIL   = false;
  bool TX_enabled   = true;
  bool SD_FAIL      = false;
  bool LSM6DSOX_FAIL = false;
  bool ADXL375_FAIL  = false;

  float pressure_1_output = 0.0f;
  float pressure_2_output = 0.0f;
  float pressure_3_output = 0.0f;
  float pressure_4_output = 0.0f;
  float battery_voltage_output = 0.0f; // ??

  // LSM6DSOX outputs
    float lsm_ax = 0.0f;
    float lsm_ay = 0.0f;
    float lsm_az = 0.0f;
    float lsm_gx = 0.0f;
    float lsm_gy = 0.0f;
    float lsm_gz = 0.0f;
    float lsm_temp = 0.0f;

  // ADXL375 outputs
    float adxl_ax = 0.0f;
    float adxl_ay = 0.0f;
    float adxl_az = 0.0f;

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
    unsigned int RFM9x_last_time     = 0;
    unsigned int Pressure_last_time  = 0;
    unsigned int LSM6DSOX_last_time  = 0;
    unsigned int ADXL375_last_time   = 0;
    unsigned int SD_last_time        = 0;

  // Other state
    uint32_t commands_received = 0;
    uint8_t telemetry_select_index = 0;

    bool imu_stream_enabled = false;
    bool pressure_stream_enabled = false;

//================ VALVE STRUCT ================
  struct Valve {
    uint8_t pin;
    const char* name;
    uint8_t state; // 0 = closed, 1 = open
  };

  Valve MAIN_VALVE   = {MAIN_VALVE_pin,   "main valve",   0};
  Valve RELIEF_VALVE = {RELIEF_VALVE_pin, "relief valve", 0};
  Valve DUMP_VALVE   = {DUMP_VALVE_pin,   "dump valve",   0};


//================ FUNCTION DECLARATIONS==========

  // Valve control helpers
    void VALVE_open(Valve& valve);    // Set selected valve output to OPEN state
    void VALVE_close(Valve& valve);   // Set selected valve output to CLOSED state

  // Radio I/O
    void read_RFM();                  // Listen for incoming radio packets and process valid commands
    void send_RFM();                  // Send prepared radio packet when a message is ready

  // Sensor read functions
    void read_pressure();             // Read all pressure transducers and update pressure variables
    void read_LSM6DSOX();             // Read LSM6DSOX accelerometer/gyro data and store outputs
    void read_ADXL375();              // Read ADXL375 high-g accelerometer data and store outputs

  // Telemetry preparation
    void prep_telemetry();            // General telemetry prep wrapper if multiple telemetry packets are used
    void prep_telemetry_pressures();  // Build telemetry packet containing pressure sensor data
    void prep_telemetry_states();     // Build telemetry packet containing valve / state-array data

  // Command / message handling
    void perform_command(uint8_t msg_class, uint8_t msg_id);   // Execute action based on decoded message class and ID
    uint8_t radio_checksum(uint8_t* radioMSG, uint8_t msgLength); // Compute checksum for radio packet validation
    void message_parser(uint8_t* buf);                         // Decode received radio packet into output arrays
    void message_assembler(uint8_t msg_class, uint8_t msg_id); // Build radio packet from input arrays and message structure

  // Conversion helpers
    float read_voltage(uint8_t pin);          // Read analog pin and convert raw ADC value to voltage
    void float_to_bytes(float value, uint8_t* out);      // Convert float to 4-byte array
    void int_16_to_bytes(uint16_t value, uint8_t* out);  // Convert 16-bit integer to 2-byte array
    void int_32_to_bytes(uint32_t value, uint8_t* out);  // Convert 32-bit integer to 4-byte array
    float bytes_to_float(uint8_t* data);      // Convert 4 bytes into float
    uint16_t bytes_to_int_16(uint8_t* data);  // Convert 2 bytes into 16-bit integer
    uint32_t bytes_to_int_32(uint8_t* data);  // Convert 4 bytes into 32-bit integer

  // State-array helpers
    // uint32_t compute_valve_status_array();    // Return simplified valve status code: 0..3
    uint32_t compute_state_array();           // Return full state/status bit array for telemetry

  // Module initialization
    void init_LSM6DSOX();   // Initialize LSM6DSOX IMU and set fail flag if not found
    void init_ADXL375();    // Initialize ADXL375 accelerometer and set fail flag if not found
    void init_SD();         // Initialize SD card and create/open log file if needed
    void save_to_sd();      // Save current system data row to SD log file
    // void send_check_message(); // Send simple "PCB online" style check message over radio

  // Debug / local test helpers
    void debug_menu();                                     // Print available debug commands to Serial Monitor
    void debug_serial();                                   // Read Serial Monitor input and run matching debug action
    void print_hex_buffer(const uint8_t* buf, uint8_t len); // Print raw byte buffer in HEX format
    void print_local_status();                             // Print current local states, sensor values, and flags
    void print_active_pins();                              // Current pinout
    void run_valve_self_test();                            // Run local open/close sequence for valve testing
    void inject_local_command(uint8_t msg_id);             // Simulate incoming command locally without radio

//================ SETUP ================
  void setup() {
    Serial.begin(115200);
    delay(1000);

    // Valve outputs
      pinMode(MAIN_VALVE.pin, OUTPUT);
      pinMode(RELIEF_VALVE.pin, OUTPUT);
      pinMode(DUMP_VALVE.pin, OUTPUT);

    // Start in a known safe state
      VALVE_close(MAIN_VALVE);
      VALVE_close(RELIEF_VALVE);
      VALVE_close(DUMP_VALVE);

    // Pressure transducer inputs
      pinMode(PRESSURE_TRANSDUCER_1_pin, INPUT);
      pinMode(PRESSURE_TRANSDUCER_2_pin, INPUT);
      pinMode(PRESSURE_TRANSDUCER_3_pin, INPUT);
      pinMode(PRESSURE_TRANSDUCER_4_pin, INPUT);

    // ADC setup copied in the same style as pad computer code
      adc->adc0->setAveraging(16);
      adc->adc0->setResolution(16);
      adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
      adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
      adc->adc0->recalibrate();
      Serial.println("online: ADC");

      #ifdef ADC_DUAL_ADCS
        adc->adc1->setAveraging(16);
        adc->adc1->setResolution(10);
        adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
        adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
        adc->adc1->recalibrate();
      #endif

    // I2C buses for IMUs
      Wire.setSDA(LSM6DSOX_SDA_pin);
      Wire.setSCL(LSM6DSOX_SCL_pin);
      Wire.begin();
      init_ADXL375();

    #if ADXL375_SDA_pin == ADXL375_SCL_pin
      Wire1.setSDA(17); // fallback to Teensy Wire1 SDA if duplicated define was left at the top
    #else
      Wire1.setSDA(ADXL375_SDA_pin);
    #endif
    Wire1.setSCL(ADXL375_SCL_pin);
    Wire1.begin();

    init_LSM6DSOX();
    init_ADXL375();
    init_SD();

    // Reset RFM9X
      pinMode(RFM9X_RST, OUTPUT);
      digitalWrite(RFM9X_RST, HIGH);
      delay(10);
      digitalWrite(RFM9X_RST, LOW);
      delay(10);
      digitalWrite(RFM9X_RST, HIGH);
      delay(10);

    SPI.begin();


    // Radio reset
      if (!rf95.init()) {
        Serial.println("offline: RFM9X");
        RFM9X_FAIL = true;
      } else {
        if (!rf95.setFrequency(CONTROL_FREQ)) {
          Serial.println("offline: RFM9X frequency set");
          RFM9X_FAIL = true;
        } else {
          rf95.setTxPower(RFM9X_PWR, false);
          Serial.println("online: RFM9X");
        }
      }

    Serial.println("PCB - status: online");
    debug_menu();
  }

//================ MAIN LOOP ================
  void loop() {
    debug_serial();
    read_RFM();
    read_pressure();
    read_LSM6DSOX();
    read_ADXL375();

    if (pressure_stream_enabled) {
      Serial.print("P1 = "); Serial.print(pressure_1_output, 3);
      Serial.print(" | P2 = "); Serial.print(pressure_2_output, 3);
      Serial.print(" | P3 = "); Serial.print(pressure_3_output, 3);
      Serial.print(" | P4 = "); Serial.println(pressure_4_output, 3);
    }

    if (imu_stream_enabled) {
      Serial.print("LSM ACC = ");
      Serial.print(lsm_ax, 3); Serial.print(", ");
      Serial.print(lsm_ay, 3); Serial.print(", ");
      Serial.print(lsm_az, 3); Serial.print(" | GYRO = ");
      Serial.print(lsm_gx, 3); Serial.print(", ");
      Serial.print(lsm_gy, 3); Serial.print(", ");
      Serial.print(lsm_gz, 3); Serial.print(" | TEMP = ");
      Serial.print(lsm_temp, 3); Serial.print(" | ADXL = ");
      Serial.print(adxl_ax, 3); Serial.print(", ");
      Serial.print(adxl_ay, 3); Serial.print(", ");
      Serial.println(adxl_az, 3);
    }

    save_to_sd();
    prep_telemetry();
    send_RFM();
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
  float read_voltage(uint8_t pin) {
    int value = adc->adc0->analogRead(pin);
    float Vout = value * 3.3f / adc->adc0->getMaxValue();
    return Vout;
  }

  void read_pressure() {
    if (micros() - Pressure_last_time > Pres_rate) {
      float voltage = 0.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_1_pin);
      pressure_1_output = 0.8517f * 1000.0f * voltage - 370.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_2_pin);
      pressure_2_output = 0.8517f * 1000.0f * voltage - 370.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_3_pin);
      pressure_3_output = 0.8517f * 1000.0f * voltage - 370.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_4_pin);
      pressure_4_output = 0.8517f * 1000.0f * voltage - 370.0f;

      Pressure_last_time = micros();
    }
  }

//================ IMU CODE ================
  void init_LSM6DSOX() {
    if (!lsm6dsox.begin_I2C(0x6A, &Wire)) {
      Serial.println("offline: LSM6DSOX");
      LSM6DSOX_FAIL = true;
      return;
    }

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_208_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_208_HZ);
    Serial.println("online: LSM6DSOX");
  }

  void init_ADXL375() {
    if (!adxl375.begin(0x53)) {
      Serial.println("offline: ADXL375");
      ADXL375_FAIL = true;
      return;
    }
    Serial.println("online: ADXL375");
  }

  void read_LSM6DSOX() {
    if (LSM6DSOX_FAIL) return;

    if (micros() - LSM6DSOX_last_time > LSM6DSOX_rate) {
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;

      lsm6dsox.getEvent(&accel, &gyro, &temp);

      lsm_ax = accel.acceleration.x;
      lsm_ay = accel.acceleration.y;
      lsm_az = accel.acceleration.z;
      lsm_gx = gyro.gyro.x;
      lsm_gy = gyro.gyro.y;
      lsm_gz = gyro.gyro.z;
      lsm_temp = temp.temperature;

      LSM6DSOX_last_time = micros();
    }
  }

  void read_ADXL375() {
    if (ADXL375_FAIL) return;

    if (micros() - ADXL375_last_time > ADXL375_rate) {
      sensors_event_t event;
      adxl375.getEvent(&event);

      adxl_ax = event.acceleration.x;
      adxl_ay = event.acceleration.y;
      adxl_az = event.acceleration.z;

      ADXL375_last_time = micros();
    }
  }

//================ TELEMETRY CODE ================

  uint32_t compute_state_array() {
    // GSE-like state array packed into one uint32 for Msg 0x02-0x06
    // bit0 = MAIN, bit1 = RELIEF, bit2 = DUMP
    uint32_t state_array = 0;

    if (MAIN_VALVE.state)   state_array |= 0x01;
    if (RELIEF_VALVE.state) state_array |= 0x02;
    if (DUMP_VALVE.state)   state_array |= 0x04;

    return state_array;
  }

  void prep_telemetry_pressures() {
    // Msg 0x02-0x03 structure: {5,5,5,5,5,5,4}
    // Use the 4 pressures in floats[0..3]
    in_floats[0] = pressure_1_output;
    in_floats[1] = pressure_2_output;
    in_floats[2] = pressure_3_output;
    in_floats[3] = pressure_4_output;
    in_floats[4] = 0.0f;
    in_floats[5] = 0.0f;
    in_int32s[6] = commands_received;

    message_assembler(0x02, 0x03);
  }

  void prep_telemetry_states() {
    // Msg 0x02-0x06 structure: {4}
    in_int32s[0] = commands_received;
    message_assembler(0x02, 0x06);
  }

  void prep_telemetry() {
    // Alternate packets so the radio keeps the same basic behavior,
    // but all 4 pressures and valve states get transmitted.
    if (telemetry_select_index == 0) {
      prep_telemetry_pressures();
      telemetry_select_index = 1;
    } else {
      prep_telemetry_states();
      telemetry_select_index = 0;
    }
  }

//================ RADIO CODE ================
  void read_RFM() {
    if (!RFM9X_FAIL && rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {
        message_parser(buf);

        if (recv_ready) {
          perform_command(buf[0], buf[1]);
          print_local_status();
          recv_ready = 0;
        }
      }
    }
  }

  void send_RFM() {
    if (!RFM9X_FAIL && TX_enabled && (micros() - RFM9x_last_time > RFM9x_rate) && msg_ready) {
      rf95.send(message_send_buf, message_send_len);
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
      commands_received++;

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

//================ SD CARD CODE ================
  void init_SD() {
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("offline: SD");
      SD_FAIL = true;
      return;
    }

    if (!SD.exists("log.csv")) {
      logFile = SD.open("log.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("time_ms,commands_received,main_state,relief_state,dump_state,pressure_1,pressure_2,pressure_3,pressure_4,lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,adxl_ax,adxl_ay,adxl_az");
        logFile.close();
      }
    }

    Serial.println("online: SD");
  }

  void save_to_sd() {
    if (SD_FAIL) return;

    if (micros() - SD_last_time > SD_rate) {
      logFile = SD.open("log.csv", FILE_WRITE);
      if (logFile) {
        logFile.print(millis());
        logFile.print(',');
        logFile.print(commands_received);
        logFile.print(',');
        logFile.print(MAIN_VALVE.state);
        logFile.print(',');
        logFile.print(RELIEF_VALVE.state);
        logFile.print(',');
        logFile.print(DUMP_VALVE.state);
        logFile.print(',');
        logFile.print(pressure_1_output, 3);
        logFile.print(',');
        logFile.print(pressure_2_output, 3);
        logFile.print(',');
        logFile.print(pressure_3_output, 3);
        logFile.print(',');
        logFile.print(pressure_4_output, 3);
        logFile.print(',');
        logFile.print(lsm_ax, 3);
        logFile.print(',');
        logFile.print(lsm_ay, 3);
        logFile.print(',');
        logFile.print(lsm_az, 3);
        logFile.print(',');
        logFile.print(lsm_gx, 3);
        logFile.print(',');
        logFile.print(lsm_gy, 3);
        logFile.print(',');
        logFile.print(lsm_gz, 3);
        logFile.print(',');
        logFile.print(lsm_temp, 3);
        logFile.print(',');
        logFile.print(adxl_ax, 3);
        logFile.print(',');
        logFile.print(adxl_ay, 3);
        logFile.print(',');
        logFile.println(adxl_az, 3);
        logFile.close();
      }
      SD_last_time = micros();
    }
  }

//================ DEBUG & MANUAL CONTROL ================
  void debug_menu() {
    Serial.println();
    Serial.println("===== PCB DEBUG MENU =====");
    Serial.println("m = print this menu");
    Serial.println("s = print local status");
    Serial.println("a = print active pins");
    Serial.println("v = run valve self-test");
    Serial.println("p = toggle continuous pressure stream");
    Serial.println("i = toggle continuous IMU stream");
    Serial.println("t = build telemetry and print raw packet");
    Serial.println("e = enable TX");
    Serial.println("d = disable TX");
    Serial.println("f = set radio frequency from serial input");
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

    Serial.print("Pressure 1 output  = ");
    Serial.println(pressure_1_output);

    Serial.print("Pressure 2 output  = ");
    Serial.println(pressure_2_output);

    Serial.print("Pressure 3 output  = ");
    Serial.println(pressure_3_output);

    Serial.print("Pressure 4 output  = ");
    Serial.println(pressure_4_output);

    Serial.print("Commands received  = ");
    Serial.println(commands_received);

    Serial.print("State array        = ");
    Serial.println(compute_state_array());

    Serial.print("TX enabled         = ");
    Serial.println(TX_enabled);

    Serial.print("RFM9X_FAIL         = ");
    Serial.println(RFM9X_FAIL);

    Serial.print("CONTROL_FREQ       = ");
    Serial.println(CONTROL_FREQ, 3);

    Serial.println("------------------------");
    Serial.println();
  }

  void run_valve_self_test() {
    Serial.println("Running valve self-test...");

    VALVE_close(MAIN_VALVE);
    VALVE_close(RELIEF_VALVE);
    VALVE_close(DUMP_VALVE);
    delay(1500);

    Serial.println("Opening MAIN");
    VALVE_open(MAIN_VALVE);
    print_local_status();
    delay(2000);

    Serial.println("Closing MAIN");
    VALVE_close(MAIN_VALVE);
    print_local_status();
    delay(2000);

    Serial.println("Opening RELIEF");
    VALVE_open(RELIEF_VALVE);
    print_local_status();
    delay(2000);

    Serial.println("Closing RELIEF");
    VALVE_close(RELIEF_VALVE);
    print_local_status();
    delay(2000);

    Serial.println("Valve self-test complete.");
  }

  void inject_local_command(uint8_t msg_id) {
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

      case 'a':
        print_active_pins();
        break;

      case 'p':
        pressure_stream_enabled = !pressure_stream_enabled;
        Serial.print("Pressure stream ");
        Serial.println(pressure_stream_enabled ? "ENABLED" : "DISABLED");
        break;

      case 'i':
        imu_stream_enabled = !imu_stream_enabled;
        Serial.print("IMU stream ");
        Serial.println(imu_stream_enabled ? "ENABLED" : "DISABLED");
        break;

      case 't':
        prep_telemetry();
        Serial.print("Telemetry length = ");
        Serial.println(message_send_len);
        Serial.print("Telemetry packet = ");
        print_hex_buffer(message_send_buf, message_send_len);
        break;

      case 'e':
        TX_enabled = true;
        Serial.println("TX enabled");
        break;

      case 'd':
        TX_enabled = false;
        Serial.println("TX disabled");
        break;

      case 'f': {
        Serial.println("Enter frequency MHz:");
        while (!Serial.available()) {
        }
        float new_freq = Serial.parseFloat();
        CONTROL_FREQ = new_freq;
        rf95.setFrequency(CONTROL_FREQ);
        Serial.print("CONTROL_FREQ set to ");
        Serial.println(CONTROL_FREQ, 3);
        break;
      }

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

  void print_active_pins() {
    Serial.println();
    Serial.println("----- ACTIVE PINS -----");

    Serial.print("MAIN_VALVE_pin          = "); Serial.println(MAIN_VALVE_pin);
    Serial.print("RELIEF_VALVE_pin        = "); Serial.println(RELIEF_VALVE_pin);
    Serial.print("DUMP_VALVE_pin          = "); Serial.println(DUMP_VALVE_pin);

    Serial.print("PRESSURE_TRANSDUCER_1   = "); Serial.println(PRESSURE_TRANSDUCER_1_pin);
    Serial.print("PRESSURE_TRANSDUCER_2   = "); Serial.println(PRESSURE_TRANSDUCER_2_pin);
    Serial.print("PRESSURE_TRANSDUCER_3   = "); Serial.println(PRESSURE_TRANSDUCER_3_pin);
    Serial.print("PRESSURE_TRANSDUCER_4   = "); Serial.println(PRESSURE_TRANSDUCER_4_pin);

    Serial.print("RFM9X_G0_pin            = "); Serial.println(RFM9X_G0_pin);
    Serial.print("RFM9X_CS                = "); Serial.println(RFM9X_CS);
    Serial.print("RFM9X_RST               = "); Serial.println(RFM9X_RST);

    Serial.print("LSM6DSOX_SDA_pin        = "); Serial.println(LSM6DSOX_SDA_pin);
    Serial.print("LSM6DSOX_SCL_pin        = "); Serial.println(LSM6DSOX_SCL_pin);
    Serial.print("ADXL375_SDA_pin         = "); Serial.println(ADXL375_SDA_pin);
    Serial.print("ADXL375_SCL_pin         = "); Serial.println(ADXL375_SCL_pin);

    Serial.println("-----------------------");
    Serial.println();
  }