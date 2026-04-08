/*
  SYS_Alan_V.4.1.ino

  ORACLE

  Pointing from gyro

  calibration for gyro from gravity

*/


// START -- START -- START -- START -- START -- START -- START -- START -- START
//================ LIBRARIES ==============
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
  #include <Adafruit_BME680.h>
  #include <SparkFun_u-blox_GNSS_Arduino_Library.h>
  #include <uRTCLib.h>
  #include <cmath>

//================ CONSTANT VARIABLES ================

  // long loops = 0;

  // Valve MOSFET pin ass
    #define MAIN_VALVE_pin   2
    #define RELIEF_VALVE_pin 3
    #define DUMP_VALVE_pin   4

  // Pressure transducer inputs
    const uint8_t PRESSURE_TRANSDUCER_1_pin = 41;
    const uint8_t PRESSURE_TRANSDUCER_2_pin = 40;
    const uint8_t PRESSURE_TRANSDUCER_3_pin = 15;
    const uint8_t PRESSURE_TRANSDUCER_4_pin = 27;

  // RFM9X pin ass
    #define RFM9X_G0_pin   9
    #define RFM9X_SCK_pin  13
    #define RFM9X_MISO_pin 12
    #define RFM9X_MOSI_pin 11
    #define RFM9X_CS       10
    #define RFM9X_RST      26
    #define RFM9X_PWR      23

  // LSM6DSOX pin ass
    #define LSM6DSOX_SCL_pin 19
    #define LSM6DSOX_SDA_pin 18

  // ADXL375 pin ass
    #define ADXL375_SCL_pin 16
    #define ADXL375_SDA_pin 16

  // BME680
    #define BME680_SCK_pin 19
    #define BME680_SDI_pin 18

    uint32_t bme_ready_time = 0;
    bool bme_busy = false;
  
  // GPS sparkfun SAM-M8Q
    #define GPS_RX_pin 7
    #define GPS_TX_pin 8
    #define GPS_PPS_pin 34

  // DS1307 RTC ass
    #define DS1307_SDA_pin  18
    #define DS1307_SCL_pin  19

  // Radio frequency to match Ground_Station_Nano current setup
    float CONTROL_FREQ = 433.0f;

  // ADC object (same style as pad computer code)
    ADC *adc = new ADC();

  // SD file
    File Black_Box;

  // Time between component usages in microseconds
    const unsigned int RFM9x_rate    = 200000;
    const unsigned int Pres_rate     = 50000;
    const unsigned int LSM6DSOX_rate = 1000;
    const unsigned int ADXL375_rate  = 500;
    const unsigned int SD_rate       = 100000;
    //const unsigned int BME680_rate = 100000; BME already takes too long to measure stuff so lets read it whenever its ready to be read
    const unsigned int GPS_rate      = 200000;
    const unsigned int DS1307_rate   = 5000;

  // Singleton instance of the radio driver
    RH_RF95 rf95(RFM9X_CS, RFM9X_G0_pin);

  // Sensor objects
    Adafruit_LSM6DSOX lsm6dsox;
    Adafruit_ADXL375  adxl375 = Adafruit_ADXL375(12345);
    Adafruit_BME680 BME680;
    SFE_UBLOX_GNSS myGNSS;
    uRTCLib rtc(0x68, URTCLIB_MODEL_DS1307);
    #define GPS_PORT Serial2
  // Geometry structures

    struct Vector3 {
    double x, y, z;
    };

    struct Matrix3 {
        double m[3][3];
    };

    struct LocalAxes {
        Vector3 x_axis_global;
        Vector3 y_axis_global;
        Vector3 z_axis_global;
    };

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
  // FAIL
    bool RFM9X_FAIL    = false;
    bool TX_enabled    = true;
    bool SD_FAIL       = false;
    bool LSM6DSOX_FAIL = false;
    bool ADXL375_FAIL  = false;
    bool BME680_FAIL   = false;
    bool GPS_FAIL      = false;
    bool DS1307_FAIL   = false;

  // Press out
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

  // BME680 outputs (using current Adafruit_BME680 object)
    float BME680_temp = 0.0f;
    float BME680_humidity = 0.0f;
    float BME680_pressure = 0.0f;
    float BME680_gas_kohm = 0.0f;

  // GPS outputs
    double gps_lat = 0.0;
    double gps_lon = 0.0;
    float gps_alt_m = 0.0f;
    uint8_t gps_siv = 0;
    uint8_t gps_fix_type = 0;
    bool gps_pps_state = false;

  // DS1307 outputs
    uint8_t rtc_year = 0;
    uint8_t rtc_month = 0;
    uint8_t rtc_day = 0;
    uint8_t rtc_hour = 0;
    uint8_t rtc_minute = 0;
    uint8_t rtc_second = 0;

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
    unsigned int BME680_last_time    = 0;
    unsigned int GPS_last_time       = 0;
    unsigned int DS1307_last_time    = 0;
    unsigned int SD_last_time        = 0;

  // Other state
    uint32_t commands_received = 0;
    uint8_t telemetry_select_index = 0;

  // Stream
    bool imu_stream_enabled = false;
    bool pressure_stream_enabled = false;
    bool bme_stream_enabled = false;
    bool gps_stream_enabled = false;
    bool rtc_stream_enabled = false;
    bool calib_imu_stream_enabled = false;
    bool Orient_Teensy = false;

  // ORACLE
    //Gyro
      float Gyro_meas_start = 0.0f;
      float Gyro_meas_fin = 0.0f;
      double dt_Gyro = 0.0f;

      float gyro_x_offset = 0.0f;
      float gyro_y_offset = 0.0f;
      float gyro_z_offset = 0.0f;

      float Calibrated_Gyro_x = 0;
      float Calibrated_Gyro_y = 0;
      float Calibrated_Gyro_z = 0;

      double Teensy_dx = 0;
      double Teensy_dy = 0;
      double Teensy_dz = 0;

      // Pitch, Yaw, Roll local in rad
        double Local_dx = 0; // Pitch
        double Local_dy = 0; // Yaw
        double Local_dz = 0; // Roll
    
    // acc/vel/loc
      Vector3 Teensy_acc_local = {0.0, 0.0, 0.0};
      
      Vector3 Teensy_vel_global = {0.0, 0.0, 0.0};

      Vector3 Teensy_loc_global = {0.0, 0.0, 0.0};

    // Local in Global world axes
      // Vector3 Local_X_in_Global = {1.0, 0.0, 0.0};
      // Vector3 Local_Y_in_Global = {0.0, 1.0, 0.0};
      // Vector3 Local_Z_in_Global = {0.0, 0.0, 1.0};

      Vector3 Local_X_in_Global_n = {1.0, 0.0, 0.0};
      Vector3 Local_Y_in_Global_n = {0.0, 1.0, 0.0};
      Vector3 Local_Z_in_Global_n = {0.0, 0.0, 1.0};


//================ VALVE STRUCT ================
  struct Valve {
  uint8_t pin;
  const char* name;
  bool state; 
  bool active_low;   // true = LOW opens valve, false = HIGH opens valve
  };

  Valve MAIN_VALVE   = {MAIN_VALVE_pin,   "main valve",   false, false};
  Valve RELIEF_VALVE = {RELIEF_VALVE_pin, "relief valve", false, true};
  Valve DUMP_VALVE   = {DUMP_VALVE_pin,   "dump valve",   false, false};


//================ FUNCTION DECLARATIONS==========

  // Valve control helpers
    void VALVE_open(Valve& valve);    // Set selected valve output to OPEN state
    void VALVE_close(Valve& valve);   // Set selected valve output to CLOSED state

  // Radio I/O
    void read_RFM();                  // Listen for incoming radio packets and process valid commands
    void send_RFM();                  // Send prepared radio packet when a message is ready

  // Sensor read functions
    void read_pressure(); 
    void read_LSM6DSOX();
    void read_ADXL375();

    // void read_BME680();

    void start_BME680_read();
    void finish_BME680_read();

    void read_GPS();
    void read_DS1307();

    void init_BME680();
    void init_GPS();
    void init_DS1307();

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
    void print_start_banner();                             // print in the beginning BANNER
    void debug_menu();                                     // Print available debug commands to Serial Monitor
    void debug_serial();                                   // Read Serial Monitor input and run matching debug action
    void print_hex_buffer(const uint8_t* buf, uint8_t len); // Print raw byte buffer in HEX format
    void print_local_status();                             // Print current local states, sensor values, and flags
    void print_active_pins();                              // Current pinout
    void run_valve_self_test();                            // Run local open/close sequence for valve testing
    void inject_local_command(uint8_t msg_id);             // Simulate incoming command locally without radio
    void streams();

  // Oracle
    void Calibrated_Gyro();
    void calibrateGyro();
    Matrix3 multiplyMatrices(const Matrix3& left_matrix, const Matrix3& right_matrix);
    LocalAxes rotateLocalAxes(
        const Vector3& initial_x_axis_global,
        const Vector3& initial_y_axis_global,
        const Vector3& initial_z_axis_global,
        double local_pitch_x_rad,
        double local_yaw_y_rad,
        double local_roll_z_rad
    );
    void Orientation_of_local_in_global();
    void LSM_to_Teensy_orientation_and_acc();
    void orient_teensy_reset();
    void auto_calib();
    void build_local_axes_from_gravity_projection();


//================ SETUP ================
  void setup() {
    Serial.begin(115200);
    delay(1000);

    print_start_banner();
    delay(3000);

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

    // Shared I2C bus on pins 18/19 for LSM6DSOX, BME680, and DS1307
      Wire.setSDA(18);
      Wire.setSCL(19);
      Wire.begin();

      // Separate I2C bus for ADXL375
      #if ADXL375_SDA_pin == ADXL375_SCL_pin
        Wire1.setSDA(17); // fallback if top-of-file define is still duplicated
      #else
        Wire1.setSDA(ADXL375_SDA_pin);
      #endif
      Wire1.setSCL(ADXL375_SCL_pin);
      Wire1.begin();

      // GPS PPS input
      pinMode(GPS_PPS_pin, INPUT);

      // Initialize sensor modules
      init_LSM6DSOX();
      init_ADXL375();
      init_BME680();
      init_GPS();
      init_DS1307();
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

    Serial.println("ALAN - status: online");
    debug_menu();

    //resetOrientationReference();
    auto_calib();

    
  }

//================ MAIN LOOP ================
  void loop() {
    //loops++;
    //Serial.println(vectorMagnitude(lsm_ax, lsm_ay, lsm_az));
    debug_serial();
    read_RFM();
    read_pressure();

    read_LSM6DSOX();
    Calibrated_Gyro();
    LSM_to_Teensy_orientation_and_acc();
    Orientation_of_local_in_global();
    

    //  updatePCBPointingAngles(Calibrated_Gyro_y, -Calibrated_Gyro_z, Calibrated_Gyro_x);
    //  printPointingData();
 
    read_ADXL375();
    
    start_BME680_read();
    finish_BME680_read();

    read_GPS();
    read_DS1307();

    streams();
    save_to_sd();
    prep_telemetry();
    send_RFM();
  }

//================ VALVES CONTROL ================
  void VALVE_open(Valve& valve) {
    digitalWrite(valve.pin, valve.active_low ? LOW : HIGH);
    valve.state = true;
    Serial.print(valve.name);
    Serial.println(" valve: OPEN");
  }

  void VALVE_close(Valve& valve) {
    digitalWrite(valve.pin, valve.active_low ? HIGH : LOW);
    valve.state = false;
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
      Gyro_meas_start = micros();

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
      //Serial.println(lsm_gz);

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

  void init_BME680() {
    if (!BME680.begin(0x77, &Wire) && !BME680.begin(0x76, &Wire)) {
      Serial.println("offline: BME680");
      BME680_FAIL = true;
      return;
    }

    BME680.setTemperatureOversampling(BME680_OS_8X);
    //BME680.setHumidityOversampling(BME680_OS_NONE);
    BME680.setPressureOversampling(BME680_OS_4X);
    //BME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //BME680.setGasHeater(0, 0);

    Serial.println("online: BME680");
  }

  void start_BME680_read() {
    if (!bme_busy) {
      bme_ready_time = BME680.beginReading();
      bme_busy = true;
    }
  }

  void finish_BME680_read() {
    if (bme_busy && BME680.remainingReadingMillis() == 0) {
      if (BME680.endReading()) {
        BME680_temp = BME680.temperature;
        BME680_pressure = BME680.pressure / 100.0f;
      }
      bme_busy = false;
    }
  }
  /*OLD BME680 CODE
    void read_BME680() {
      if (BME680_FAIL) return;

      if (micros() - BME680_last_time > BME680_rate) {
        if (BME680.performReading()) {
          BME680_temp = BME680.temperature;
          //BME680_humidity = BME680.humidity;
          BME680_pressure = BME680.pressure / 100.0f;
          //BME680_gas_kohm = BME680.gas_resistance / 1000.0f;
        }

        BME680_last_time = micros();
      }
    }
  */

  void init_GPS() {
    GPS_PORT.begin(9600);
    GPS_PORT.setRX(GPS_RX_pin);
    GPS_PORT.setTX(GPS_TX_pin);

    if (!myGNSS.begin(GPS_PORT)) {
      Serial.println("offline: GPS");
      GPS_FAIL = true;
      return;
    }

    Serial.println("online: GPS");
  }

  void read_GPS() {
    if (GPS_FAIL) return;

    if (micros() - GPS_last_time > GPS_rate) {
      gps_lat = myGNSS.getLatitude() / 10000000.0;
      gps_lon = myGNSS.getLongitude() / 10000000.0;
      gps_alt_m = myGNSS.getAltitude() / 1000.0f;
      gps_siv = myGNSS.getSIV();
      gps_fix_type = myGNSS.getFixType();
      gps_pps_state = digitalRead(GPS_PPS_pin);

      GPS_last_time = micros();
    }
  }

  void init_DS1307() {
    rtc.refresh();
    Serial.println("online: DS1307");
  }

  void read_DS1307() {
    if (micros() - DS1307_last_time > DS1307_rate) {
      rtc.refresh();

      rtc_year = rtc.year();
      rtc_month = rtc.month();
      rtc_day = rtc.day();
      rtc_hour = rtc.hour();
      rtc_minute = rtc.minute();
      rtc_second = rtc.second();

      DS1307_last_time = micros();
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
    // change send here
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
      Black_Box = SD.open("log.csv", FILE_WRITE);
      if (Black_Box) {
        Black_Box.println("time_ms,commands_received,main_state,relief_state,dump_state,pressure_1,pressure_2,pressure_3,pressure_4,lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,adxl_ax,adxl_ay,adxl_az,bme_temp,bme_humidity,bme_pressure,bme_gas_kohm,gps_lat,gps_lon,gps_alt_m,gps_siv,gps_fix_type,gps_pps,rtc_year,rtc_month,rtc_day,rtc_hour,rtc_minute,rtc_second");
        Black_Box.close();
      }
    }

    Serial.println("online: SD");
  }

  void save_to_sd() {
    if (SD_FAIL) return;

    if (micros() - SD_last_time > SD_rate) {
      Black_Box = SD.open("log.csv", FILE_WRITE);
      if (Black_Box) {
        Black_Box.print(millis());
        Black_Box.print(',');
        Black_Box.print(commands_received);
        Black_Box.print(',');
        Black_Box.print(MAIN_VALVE.state);
        Black_Box.print(',');
        Black_Box.print(RELIEF_VALVE.state);
        Black_Box.print(',');
        Black_Box.print(DUMP_VALVE.state);
        Black_Box.print(',');
        Black_Box.print(pressure_1_output, 3);
        Black_Box.print(',');
        Black_Box.print(pressure_2_output, 3);
        Black_Box.print(',');
        Black_Box.print(pressure_3_output, 3);
        Black_Box.print(',');
        Black_Box.print(pressure_4_output, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_ax, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_ay, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_az, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_gx, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_gy, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_gz, 3);
        Black_Box.print(',');
        Black_Box.print(lsm_temp, 3);
        Black_Box.print(',');
        Black_Box.print(adxl_ax, 3);
        Black_Box.print(',');
        Black_Box.print(adxl_ay, 3);
        Black_Box.print(',');
        Black_Box.print(adxl_az, 3);
        Black_Box.print(',');
        Black_Box.print(BME680_temp, 3);
        Black_Box.print(',');
        Black_Box.print(BME680_humidity, 3);
        Black_Box.print(',');
        Black_Box.print(BME680_pressure, 3);
        Black_Box.print(',');
        Black_Box.print(BME680_gas_kohm, 3);
        Black_Box.print(',');
        Black_Box.print(gps_lat, 7);
        Black_Box.print(',');
        Black_Box.print(gps_lon, 7);
        Black_Box.print(',');
        Black_Box.print(gps_alt_m, 3);
        Black_Box.print(',');
        Black_Box.print(gps_siv);
        Black_Box.print(',');
        Black_Box.print(gps_fix_type);
        Black_Box.print(',');
        Black_Box.print(gps_pps_state);
        Black_Box.print(',');
        Black_Box.print(rtc_year);
        Black_Box.print(',');
        Black_Box.print(rtc_month);
        Black_Box.print(',');
        Black_Box.print(rtc_day);
        Black_Box.print(',');
        Black_Box.print(rtc_hour);
        Black_Box.print(',');
        Black_Box.print(rtc_minute);
        Black_Box.print(',');
        Black_Box.println(rtc_second);

        Black_Box.close();
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
    Serial.println("i calib = toggle continuous calibrated IMU stream");
    Serial.println("calib i = calibrate IMU");
    Serial.println("teensy or = Orientation of PCB in unit vectors global");
    Serial.println("or reset = Orientation of PCB in unit vectors global gravity down");

    Serial.println("b = toggle continuous BME stream");
    Serial.println("g = toggle continuous GPS stream");
    Serial.println("r = toggle continuous RTC stream");
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

    Serial.print("BME temp          = ");
    Serial.println(BME680_temp);

    Serial.print("BME humidity      = ");
    Serial.println(BME680_humidity);

    Serial.print("BME pressure      = ");
    Serial.println(BME680_pressure);

    Serial.print("BME gas kOhm      = ");
    Serial.println(BME680_gas_kohm);

    Serial.print("GPS lat           = ");
    Serial.println(gps_lat, 7);

    Serial.print("GPS lon           = ");
    Serial.println(gps_lon, 7);

    Serial.print("GPS alt m         = ");
    Serial.println(gps_alt_m, 3);

    Serial.print("GPS SIV           = ");
    Serial.println(gps_siv);

    Serial.print("GPS fix type      = ");
    Serial.println(gps_fix_type);

    Serial.print("GPS PPS state     = ");
    Serial.println(gps_pps_state);

    Serial.print("RTC date          = ");
    Serial.print(rtc_year); Serial.print("-");
    Serial.print(rtc_month); Serial.print("-");
    Serial.println(rtc_day);

    Serial.print("RTC time          = ");
    Serial.print(rtc_hour); Serial.print(":");
    Serial.print(rtc_minute); Serial.print(":");
    Serial.println(rtc_second);

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

    //char c = Serial.read();
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "m") {
      debug_menu();
    }

    else if (cmd == "s") {
      print_local_status();
    }

    else if (cmd == "v") {
      run_valve_self_test();
    }

    else if (cmd == "a") {
      print_active_pins();
    }

    else if (cmd == "p") {
      pressure_stream_enabled = !pressure_stream_enabled;
      Serial.print("Pressure stream ");
      Serial.println(pressure_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "i") {
      imu_stream_enabled = !imu_stream_enabled;
      Serial.print("IMU stream ");
      Serial.println(imu_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "i calib") {
      calib_imu_stream_enabled = !calib_imu_stream_enabled;
      Serial.print("IMU calibrated stream ");
      Serial.println(calib_imu_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "calib i") {
      calibrateGyro();
      Serial.println("IMU calibrated");
    }

    else if (cmd == "teensy or") {
      Orient_Teensy = !Orient_Teensy;
      Serial.print("Teensy orientation in unit vect global ");
      Serial.println(Orient_Teensy ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "or reset") {
      orient_teensy_reset();
      Serial.print("Teensy orientation has been reset");
    }

    else if (cmd == "t") {
      prep_telemetry();
      Serial.print("Telemetry length = ");
      Serial.println(message_send_len);
      Serial.print("Telemetry packet = ");
      print_hex_buffer(message_send_buf, message_send_len);
    }

    else if (cmd == "b") {
      bme_stream_enabled = !bme_stream_enabled;
      Serial.print("BME stream ");
      Serial.println(bme_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "g") {
      gps_stream_enabled = !gps_stream_enabled;
      Serial.print("GPS stream ");
      Serial.println(gps_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "r") {
      rtc_stream_enabled = !rtc_stream_enabled;
      Serial.print("RTC stream ");
      Serial.println(rtc_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "e") {
      TX_enabled = true;
      Serial.println("TX enabled");
    }

    else if (cmd == "d") {
      TX_enabled = false;
      Serial.println("TX disabled");
    }

    else if (cmd == "f") {
      Serial.println("Enter frequency MHz:");
      while (!Serial.available()) {
      }
      float new_freq = Serial.parseFloat();
      CONTROL_FREQ = new_freq;
      rf95.setFrequency(CONTROL_FREQ);
      Serial.print("CONTROL_FREQ set to ");
      Serial.println(CONTROL_FREQ, 3);
    }

    else if (cmd == "4") {
      inject_local_command(0x04);
    }

    else if (cmd == "5") {
      inject_local_command(0x05);
    }

    else if (cmd == "8") {
      inject_local_command(0x08);
    }

    else if (cmd == "9") {
      inject_local_command(0x09);
    }

    else {
      Serial.print("Unknown debug key: ");
      Serial.println(cmd);
      debug_menu();
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

    Serial.print("BME680_SDI_pin          = "); Serial.println(BME680_SDI_pin);
    Serial.print("BME680_SCK_pin          = "); Serial.println(BME680_SCK_pin);

    Serial.print("GPS_RX_pin              = "); Serial.println(GPS_RX_pin);
    Serial.print("GPS_TX_pin              = "); Serial.println(GPS_TX_pin);
    Serial.print("GPS_PPS_pin             = "); Serial.println(GPS_PPS_pin);

    Serial.print("DS1307_SDA_pin          = "); Serial.println(DS1307_SDA_pin);
    Serial.print("DS1307_SCL_pin          = "); Serial.println(DS1307_SCL_pin);

    Serial.println("-----------------------");
    Serial.println();
  }

  void streams(){
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
        Serial.print(lsm_az, 5); Serial.print(" | GYRO = ");
        Serial.print(lsm_gx, 3); Serial.print(", ");
        Serial.print(lsm_gy, 3); Serial.print(", ");
        Serial.print(lsm_gz, 3); Serial.print(" | TEMP = ");
        Serial.print(lsm_temp, 3); Serial.print(" | ADXL = ");
        Serial.print(adxl_ax, 3); Serial.print(", ");
        Serial.print(adxl_ay, 3); Serial.print(", ");
        Serial.println(adxl_az, 3);
      }

      if (calib_imu_stream_enabled) {
        Serial.print("LSM ACC = ");
        Serial.print(lsm_ax, 3); Serial.print(", ");
        Serial.print(lsm_ay, 3); Serial.print(", ");
        Serial.print(lsm_az, 3); Serial.print(" | GYRO = ");
        Serial.print(Calibrated_Gyro_x, 3); Serial.print(", ");
        Serial.print(Calibrated_Gyro_y, 3); Serial.print(", ");
        Serial.print(Calibrated_Gyro_z, 3); Serial.print(" | TEMP = ");
        Serial.print(lsm_temp, 3); Serial.print(" | ADXL = ");
        Serial.print(adxl_ax, 3); Serial.print(", ");
        Serial.print(adxl_ay, 3); Serial.print(", ");
        Serial.println(adxl_az, 3);
      }

      if (Orient_Teensy) {
        Serial.print("Orientation in global for local:");

        Serial.print("|x: (");
        Serial.print(Local_X_in_Global_n.x); Serial.print(", ");
        Serial.print(Local_X_in_Global_n.y); Serial.print(", ");
        Serial.print(Local_X_in_Global_n.z); Serial.print(") |");

        Serial.print("y: (");
        Serial.print(Local_Y_in_Global_n.x); Serial.print(", ");
        Serial.print(Local_Y_in_Global_n.y); Serial.print(", ");
        Serial.print(Local_Y_in_Global_n.z); Serial.print(") |");

        Serial.print("z: (");
        Serial.print(Local_Z_in_Global_n.x); Serial.print(", ");
        Serial.print(Local_Z_in_Global_n.y); Serial.print(", ");
        Serial.print(Local_Z_in_Global_n.z); Serial.println(") |");
      }

      if (bme_stream_enabled) {
        Serial.print("BME T = "); Serial.print(BME680_temp, 3);
        Serial.print(" | H = "); Serial.print(BME680_humidity, 3);
        Serial.print(" | P = "); Serial.print(BME680_pressure, 3);
        Serial.print(" | GAS = "); Serial.println(BME680_gas_kohm, 3);
      }

      if (gps_stream_enabled) {
        Serial.print("GPS LAT = "); Serial.print(gps_lat, 7);
        Serial.print(" | LON = "); Serial.print(gps_lon, 7);
        Serial.print(" | ALT = "); Serial.print(gps_alt_m, 3);
        Serial.print(" | SIV = "); Serial.print(gps_siv);
        Serial.print(" | FIX = "); Serial.print(gps_fix_type);
        Serial.print(" | PPS = "); Serial.println(gps_pps_state);
      }

      if (rtc_stream_enabled) {
        Serial.print("RTC = ");
        Serial.print(rtc_year); Serial.print("-");
        Serial.print(rtc_month); Serial.print("-");
        Serial.print(rtc_day); Serial.print(" ");
        Serial.print(rtc_hour); Serial.print(":");
        Serial.print(rtc_minute); Serial.print(":");
        Serial.println(rtc_second);
      }


  }

  void print_start_banner() {
  Serial.println(R"PCB(
                                                                                              +                                                         
                                                                                             +                                                          
                                                                                            +                                                           
                                                                                           ++                                                           
                                                                                         +++                                                            
                                                                                        +π+                                                             
                                                                                          +                                                             
                                                                                         +                                                              
                                                                                        +                                                               
                ++++++++++++++    ×+++++++          +++++++      +++++++          +++++÷ ++++++++++     +++++++++++++++++   ++++++++                    
              +++÷++++++++++++    ++≠≠≠÷+          ∞+=≠≠≠++     ++≠≠=≈++         ×+++   +++++≠≠≠=÷++   ++≠≠=≈+++++-≠≠≠=÷+=  +-≠≠≠-+                     
             ++==≠+               +÷===++          ++====+      +÷==≠-+          +++ = +    +≠×===+    +×===++    +=====+  ++×===+≈                     
             +×÷==+              ++===≠+           +÷==÷++     ++===÷+∞         +++ ∞ ++    +===≠++   ++÷=÷=+    ÷+==÷=++  ++==÷÷+                      
            ++÷∞≈∞-++++++++≈     +===≠++          ++==÷≠+      ++===-+          ++ + ++++++++≠==++    +÷===≠++++++×=≠÷++  ++===≠++                      
             -++++++++-==≠++    ++====+           +===≠-+     ++===≠++         + ÷ ×++=÷÷÷÷=+++++    ++===≠++++++++++++   +-===×+                       
                      +==≠++    +÷===-+          ++==÷÷+      +====×+          ≠  ++×=÷÷===÷+        +×===-+             √+====+×                       
                     +-==++    ++====-π          +÷===≠++   +++÷==-+≠        ×+ +++×+++++==÷++      ++====+π             ++===≠+                        
          ++++++++++++++++     +÷÷÷==-+++++++++  +++÷===+++++-×+++++       ++ +++=≈÷+  =++==≠+++    +÷===×+             ++×====++++++++++               
         √+++++++++++++       ++++++++++++++++    ∞+++++++++++++=       ×++  ++++++++    ++++++++  ++++++++             ++++++++++++++++                
                           +√                                        π++                                                                                
                              ++                                  +++                                                                                   
                                  +++                        ++++                                                                                       
                                       -+++++++++++++++++++÷                                                                                            
  )PCB");
  }
// --------------- math -------------
    float vectorMagnitude(float x, float y, float z) {
      return sqrt(x * x + y * y + z * z);
    }

    Matrix3 multiplyMatrices(const Matrix3& left_matrix, const Matrix3& right_matrix) {
        Matrix3 result{};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                for (int k = 0; k < 3; ++k)
                    result.m[row][col] += left_matrix.m[row][k] * right_matrix.m[k][col];
        return result;
    }
    double vectorMagnitude__double(double x, double y, double z) {
      return std::sqrt(x * x + y * y + z * z);
    }

// --------------> ORACLE <-------------- 
  // Explanation
    /**/

  // --------------- Constants ---------------
    const int Mach_to_Cd_size = 200;
    // _______________ TABLES __________________
      float Mach_collum_in_Table_M_vs_Cd[Mach_to_Cd_size] = {
        0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f, 0.09f, 0.1f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.2f,
        0.21f, 0.22f, 0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.3f, 0.31f, 0.32f, 0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.4f,
        0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f, 0.49f, 0.5f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f, 0.57f, 0.58f, 0.59f, 0.6f,
        0.61f, 0.62f, 0.63f, 0.64f, 0.65f, 0.66f, 0.67f, 0.68f, 0.69f, 0.7f, 0.71f, 0.72f, 0.73f, 0.74f, 0.75f, 0.76f, 0.77f, 0.78f, 0.79f, 0.8f,
        0.81f, 0.82f, 0.83f, 0.84f, 0.85f, 0.86f, 0.87f, 0.88f, 0.89f, 0.9f, 0.91f, 0.92f, 0.93f, 0.94f, 0.95f, 0.96f, 0.97f, 0.98f, 0.99f, 1.0f,
        1.01f, 1.02f, 1.03f, 1.04f, 1.05f, 1.06f, 1.07f, 1.08f, 1.09f, 1.1f, 1.11f, 1.12f, 1.13f, 1.14f, 1.15f, 1.16f, 1.17f, 1.18f, 1.19f, 1.2f,
        1.21f, 1.22f, 1.23f, 1.24f, 1.25f, 1.26f, 1.27f, 1.28f, 1.29f, 1.3f, 1.31f, 1.32f, 1.33f, 1.34f, 1.35f, 1.36f, 1.37f, 1.38f, 1.39f, 1.4f,
        1.41f, 1.42f, 1.43f, 1.44f, 1.45f, 1.46f, 1.47f, 1.48f, 1.49f, 1.5f, 1.51f, 1.52f, 1.53f, 1.54f, 1.55f, 1.56f, 1.57f, 1.58f, 1.59f, 1.6f,
        1.61f, 1.62f, 1.63f, 1.64f, 1.65f, 1.66f, 1.67f, 1.68f, 1.69f, 1.7f, 1.71f, 1.72f, 1.73f, 1.74f, 1.75f, 1.76f, 1.77f, 1.78f, 1.79f, 1.8f,
        1.81f, 1.82f, 1.83f, 1.84f, 1.85f, 1.86f, 1.87f, 1.88f, 1.89f, 1.9f, 1.91f, 1.92f, 1.93f, 1.94f, 1.95f, 1.96f, 1.97f, 1.98f, 1.99f, 2.0f
      };

      float Cd_collum_in_Table_M_vs_Cd[Mach_to_Cd_size] = {
        0.506505f, 0.501224f, 0.485555f, 0.471806f, 0.460341f, 0.450702f, 0.442469f, 0.435329f, 0.429053f, 0.423474f, 0.418465f, 0.413932f, 0.41115f, 0.411898f, 0.412308f, 0.412462f, 0.412419f, 0.412224f, 0.411912f, 0.411508f,
        0.411032f, 0.4105f, 0.409926f, 0.409317f, 0.408684f, 0.408161f, 0.407627f, 0.407085f, 0.406538f, 0.40599f, 0.405442f, 0.404895f, 0.404352f, 0.403812f, 0.403278f, 0.402749f, 0.402226f, 0.40171f, 0.401201f, 0.400699f,
        0.400204f, 0.399717f, 0.399238f, 0.398766f, 0.398302f, 0.397846f, 0.397397f, 0.396956f, 0.396522f, 0.396096f, 0.395677f, 0.395266f, 0.394861f, 0.394464f, 0.394073f, 0.393689f, 0.393312f, 0.392942f, 0.392578f, 0.39222f,
        0.392126f, 0.392029f, 0.391938f, 0.391853f, 0.391773f, 0.391699f, 0.391631f, 0.391567f, 0.391509f, 0.391456f, 0.391408f, 0.391364f, 0.391325f, 0.391291f, 0.391261f, 0.391235f, 0.391214f, 0.391197f, 0.391183f, 0.391174f,
        0.391169f, 0.391167f, 0.391169f, 0.391175f, 0.391185f, 0.391197f, 0.391214f, 0.391233f, 0.391256f, 0.391282f, 0.393942f, 0.401925f, 0.413099f, 0.423474f, 0.433849f, 0.444224f, 0.454599f, 0.464974f, 0.47535f, 0.485725f,
        0.4961f, 0.506475f, 0.51685f, 0.527225f, 0.537601f, 0.535759f, 0.533941f, 0.532145f, 0.530372f, 0.528621f, 0.52689f, 0.52518f, 0.523489f, 0.521818f, 0.520168f, 0.518247f, 0.516357f, 0.514502f, 0.512434f, 0.509754f,
        0.507112f, 0.504506f, 0.501935f, 0.499398f, 0.496895f, 0.494425f, 0.491988f, 0.489582f, 0.487208f, 0.484864f, 0.482551f, 0.480268f, 0.478015f, 0.475791f, 0.473595f, 0.471429f, 0.46929f, 0.46718f, 0.465097f, 0.463041f,
        0.461012f, 0.459011f, 0.457036f, 0.455087f, 0.453164f, 0.451267f, 0.449396f, 0.44755f, 0.44573f, 0.443935f, 0.442164f, 0.440418f, 0.438697f, 0.437f, 0.435328f, 0.433679f, 0.432054f, 0.430453f, 0.428876f, 0.427322f,
        0.425791f, 0.424284f, 0.4228f, 0.421338f, 0.4199f, 0.418484f, 0.41709f, 0.41572f, 0.414371f, 0.413045f, 0.411741f, 0.410459f, 0.409199f, 0.407961f, 0.406745f, 0.40555f, 0.404377f, 0.403225f, 0.402095f, 0.400987f,
        0.399899f, 0.398833f, 0.397788f, 0.396764f, 0.395761f, 0.394779f, 0.393818f, 0.392878f, 0.391958f, 0.391059f, 0.39018f, 0.389323f, 0.388027f, 0.386167f, 0.38432f, 0.382486f, 0.380665f, 0.378857f, 0.377061f, 0.375278f
      };

  // --------------- Properties, Missle Knows Where It Is Functions ---------------
    float interpolateFromTables(const float xTable[], const float yTable[], int size, float xValue) {
      /* explanation
        Finds the y-value for a given x-value using two tables.

        xTable = known input values
        yTable = known output values

        The function finds where xValue falls in xTable, then linearly
        interpolates between the two surrounding yTable values.

        If xValue is below or above the table range, it returns the
        nearest end value.
      */
      // Safety checks
      if (size < 2) {
        return 0.0f;
      }

      // If xValue is below table range, return first y value
      if (xValue <= xTable[0]) {
        return yTable[0];
      }

      // If xValue is above table range, return last y value
      if (xValue >= xTable[size - 1]) {
        return yTable[size - 1];
      }

      // Find interval and interpolate
      for (int i = 0; i < size - 1; i++) {
        if (xValue >= xTable[i] && xValue <= xTable[i + 1]) {
          float x0 = xTable[i];
          float x1 = xTable[i + 1];
          float y0 = yTable[i];
          float y1 = yTable[i + 1];

          // Linear interpolation
          return y0 + (xValue - x0) * (y1 - y0) / (x1 - x0);
        }
      }

      // Fallback, should normally never reach here
      return yTable[size - 1];
    }

    LocalAxes rotateLocalAxes(
        const Vector3& initial_x_axis_global,
        const Vector3& initial_y_axis_global,
        const Vector3& initial_z_axis_global,
        double local_pitch_x_rad,
        double local_yaw_y_rad,
        double local_roll_z_rad
    ) {
        double cos_pitch_x = std::cos(local_pitch_x_rad);
        double sin_pitch_x = std::sin(local_pitch_x_rad);

        double cos_yaw_y = std::cos(local_yaw_y_rad);
        double sin_yaw_y = std::sin(local_yaw_y_rad);

        double cos_roll_z = std::cos(local_roll_z_rad);
        double sin_roll_z = std::sin(local_roll_z_rad);

        Matrix3 initial_local_axes_in_global{{
            {initial_x_axis_global.x, initial_y_axis_global.x, initial_z_axis_global.x},
            {initial_x_axis_global.y, initial_y_axis_global.y, initial_z_axis_global.y},
            {initial_x_axis_global.z, initial_y_axis_global.z, initial_z_axis_global.z}
        }};

        Matrix3 rotate_about_local_z{{
            {cos_roll_z, -sin_roll_z, 0},
            {sin_roll_z,  cos_roll_z, 0},
            {0,           0,          1}
        }};

        Matrix3 rotate_about_local_x{{
            {1, 0,           0},
            {0, cos_pitch_x, -sin_pitch_x},
            {0, sin_pitch_x,  cos_pitch_x}
        }};

        Matrix3 rotate_about_local_y{{
            { cos_yaw_y, 0, sin_yaw_y},
            { 0,         1, 0},
            {-sin_yaw_y, 0, cos_yaw_y}
        }};

        Matrix3 final_local_axes_in_global = multiplyMatrices(
            initial_local_axes_in_global,
            multiplyMatrices(
                rotate_about_local_z,
                multiplyMatrices(rotate_about_local_x, rotate_about_local_y)
            )
        ); // local order: z roll -> x pitch -> y yaw

        return {
            {final_local_axes_in_global.m[0][0], final_local_axes_in_global.m[1][0], final_local_axes_in_global.m[2][0]},
            {final_local_axes_in_global.m[0][1], final_local_axes_in_global.m[1][1], final_local_axes_in_global.m[2][1]},
            {final_local_axes_in_global.m[0][2], final_local_axes_in_global.m[1][2], final_local_axes_in_global.m[2][2]}
        };
    }

    void LSM_to_Teensy_orientation_and_acc(){
      Teensy_acc_local.x = -lsm_ay;
      Teensy_acc_local.y = lsm_az;
      Teensy_acc_local.z = -lsm_ax;

      Teensy_dx = Calibrated_Gyro_y;
      Teensy_dy = -Calibrated_Gyro_z;
      Teensy_dz = Calibrated_Gyro_x;
    }

    void Orientation_of_local_in_global(){
      Gyro_meas_fin = micros();
      dt_Gyro = (Gyro_meas_fin - Gyro_meas_start)/1000000.0;
      
      
      Local_dx = Teensy_dx*dt_Gyro;
      Local_dy = Teensy_dy*dt_Gyro;
      Local_dz = Teensy_dz*dt_Gyro;

      if (Local_dx < 0.005) {
        Local_dx = 0;
      }
      if (Local_dy < 0.005) {
        Local_dy = 0;
      }
      if (Local_dz < 0.005) {
        Local_dz = 0;
      }

      //Serial.println(Local_dz,10);

      LocalAxes result = rotateLocalAxes(
        Local_X_in_Global_n, Local_Y_in_Global_n, Local_Z_in_Global_n,
        Local_dx, Local_dy, Local_dz
      );

      Local_X_in_Global_n.x = result.x_axis_global.x;
      Local_X_in_Global_n.y = result.x_axis_global.y;
      Local_X_in_Global_n.z = result.x_axis_global.z;

      Local_Y_in_Global_n.x = result.y_axis_global.x;
      Local_Y_in_Global_n.y = result.y_axis_global.y;
      Local_Y_in_Global_n.z = result.y_axis_global.z;

      Local_Z_in_Global_n.x = result.z_axis_global.x;
      Local_Z_in_Global_n.y = result.z_axis_global.y;
      Local_Z_in_Global_n.z = result.z_axis_global.z;
      
    }

  //  -------------- Calibration Functions ---------------
    void calibrateGyro() {
      read_LSM6DSOX();
      
      gyro_x_offset = lsm_gx;
      gyro_y_offset = lsm_gy;
      gyro_z_offset = lsm_gz;
    }

    void Calibrated_Gyro(){
      Calibrated_Gyro_x = lsm_gx - gyro_x_offset;
      Calibrated_Gyro_y = lsm_gy - gyro_y_offset;
      Calibrated_Gyro_z = lsm_gz - gyro_z_offset;
    }

    void orient_teensy_reset(){
      read_LSM6DSOX();
      LSM_to_Teensy_orientation_and_acc();
      build_local_axes_from_gravity_projection(Teensy_acc_local.x, Teensy_acc_local.y, Teensy_acc_local.z, M_PI / 2.0, Local_X_in_Global_n, Local_Y_in_Global_n, Local_Z_in_Global_n);
    }

    void auto_calib(){
      read_LSM6DSOX();
      calibrateGyro();
      orient_teensy_reset();
    }

    void build_local_axes_from_gravity_projection(
        double xg,
        double yg,
        double zg,
        double phi,
        Vector3& Local_X_in_Global_n,
        Vector3& Local_Y_in_Global_n,
        Vector3& Local_Z_in_Global_n
    ) {
        const double g = vectorMagnitude__double(xg, yg, zg);
        const double eps = 1e-12;

        // If gravity magnitude is too small, return default global axes
        if (g < eps) {
            Local_X_in_Global_n = {1.0, 0.0, 0.0};
            Local_Y_in_Global_n = {0.0, 1.0, 0.0};
            Local_Z_in_Global_n = {0.0, 0.0, 1.0};
            return;
        }

        // Normalized gravity components in local frame
        const double a = xg / g;
        const double b = yg / g;
        const double c = zg / g;

        const double h = std::sqrt(a * a + b * b);

        const double cos_phi = std::cos(phi);
        const double sin_phi = std::sin(phi);

        // Special case: gravity is exactly along local Z
        if (h < eps) {
            Local_X_in_Global_n = {
                cos_phi,
                sin_phi,
                0.0
            };

            Local_Y_in_Global_n = {
                c * sin_phi,
                -c * cos_phi,
                0.0
            };

            Local_Z_in_Global_n = {
                0.0,
                0.0,
                -c
            };

            return;
        }

        // General case
        Local_X_in_Global_n = {
            (b * cos_phi - a * c * sin_phi) / h,
            (-b * sin_phi - a * c * cos_phi) / h,
            -a
        };

        Local_Y_in_Global_n = {
            (-a * cos_phi - b * c * sin_phi) / h,
            ( a * sin_phi - b * c * cos_phi) / h,
            -b
        };

        Local_Z_in_Global_n = {
            h * sin_phi,
            h * cos_phi,
            -c
        };
    }

// END -- END -- END -- END -- END -- END -- END -- END -- END -- END -- END