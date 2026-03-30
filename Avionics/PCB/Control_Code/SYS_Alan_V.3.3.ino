/*
  SYS_Alan_V.3.3.ino

Added support for a BME680-style environmental sensor with initialization and read functions, without adding its data to radio telemetry.
Added support for a SparkFun SAM-M8Q GPS with initialization and read functions, including PPS pin monitoring, without adding its data to radio telemetry.
Added support for a DS1307 RTC using uRTCLib, with initialization and read functions, without adding its data to radio telemetry.
Updated the main loop so it now continuously reads the new BME, GPS, and RTC sensors alongside the existing pressure and IMU sensors.
Expanded the SD logging format so the log file now includes BME data, GPS data, and RTC date/time fields in addition to the previously logged valve, pressure, and IMU data.
Expanded the debug system so it can now stream BME, GPS, and RTC data through Serial Monitor.
Updated the active-pins debug output to include the new BME, GPS, and DS1307 pin definitions.
Kept the radio logic unchanged

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


//================ CONSTANT VARIABLES ================

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

  // BME600
  #define BME600_SCK_pin 19
  #define BME600_SDI_pin 18
  
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
  const unsigned int LSM6DSOX_rate = 5000;
  const unsigned int ADXL375_rate  = 5000;
  const unsigned int SD_rate       = 100000;
  const unsigned int BME600_rate   = 10000;
  const unsigned int GPS_rate      = 200000;
  const unsigned int DS1307_rate   = 500000;

  // Singleton instance of the radio driver
  RH_RF95 rf95(RFM9X_CS, RFM9X_G0_pin);

  // Sensor objects
  Adafruit_LSM6DSOX lsm6dsox;
  Adafruit_ADXL375  adxl375 = Adafruit_ADXL375(12345);
  Adafruit_BME680 bme600;
  SFE_UBLOX_GNSS myGNSS;
  uRTCLib rtc(0x68, URTCLIB_MODEL_DS1307);
  #define GPS_PORT Serial2



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

  bool RFM9X_FAIL    = false;
  bool TX_enabled    = true;
  bool SD_FAIL       = false;
  bool LSM6DSOX_FAIL = false;
  bool ADXL375_FAIL  = false;
  bool BME600_FAIL   = false;
  bool GPS_FAIL      = false;
  bool DS1307_FAIL   = false;

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

  // BME600 outputs (using current Adafruit_BME680 object)
    float bme600_temp = 0.0f;
    float bme600_humidity = 0.0f;
    float bme600_pressure = 0.0f;
    float bme600_gas_kohm = 0.0f;

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
    unsigned int BME600_last_time    = 0;
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
    void read_BME600();
    void read_GPS();
    void read_DS1307();

    void init_BME600();
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

    // Shared I2C bus on pins 18/19 for LSM6DSOX, BME600, and DS1307
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
      init_BME600();
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
  }

//================ MAIN LOOP ================
  void loop() {
    debug_serial();
    read_RFM();
    read_pressure();
    read_LSM6DSOX();
    read_ADXL375();
    read_BME600();
    read_GPS();
    read_DS1307();

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

    if (bme_stream_enabled) {
      Serial.print("BME T = "); Serial.print(bme600_temp, 3);
      Serial.print(" | H = "); Serial.print(bme600_humidity, 3);
      Serial.print(" | P = "); Serial.print(bme600_pressure, 3);
      Serial.print(" | GAS = "); Serial.println(bme600_gas_kohm, 3);
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

  void init_BME600() {
    if (!bme600.begin(0x77, &Wire) && !bme600.begin(0x76, &Wire)) {
      Serial.println("offline: BME600");
      BME600_FAIL = true;
      return;
    }

    bme600.setTemperatureOversampling(BME680_OS_8X);
    bme600.setHumidityOversampling(BME680_OS_2X);
    bme600.setPressureOversampling(BME680_OS_4X);
    bme600.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme600.setGasHeater(320, 150);

    Serial.println("online: BME600");
  }

  void read_BME600() {
    if (BME600_FAIL) return;

    if (micros() - BME600_last_time > BME600_rate) {
      if (bme600.performReading()) {
        bme600_temp = bme600.temperature;
        bme600_humidity = bme600.humidity;
        bme600_pressure = bme600.pressure / 100.0f;
        bme600_gas_kohm = bme600.gas_resistance / 1000.0f;
      }

      BME600_last_time = micros();
    }
  }

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
        Black_Box.print(bme600_temp, 3);
        Black_Box.print(',');
        Black_Box.print(bme600_humidity, 3);
        Black_Box.print(',');
        Black_Box.print(bme600_pressure, 3);
        Black_Box.print(',');
        Black_Box.print(bme600_gas_kohm, 3);
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
    Serial.println(bme600_temp);

    Serial.print("BME humidity      = ");
    Serial.println(bme600_humidity);

    Serial.print("BME pressure      = ");
    Serial.println(bme600_pressure);

    Serial.print("BME gas kOhm      = ");
    Serial.println(bme600_gas_kohm);

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

      case 'b':
        bme_stream_enabled = !bme_stream_enabled;
        Serial.print("BME stream ");
        Serial.println(bme_stream_enabled ? "ENABLED" : "DISABLED");
        break;

      case 'g':
        gps_stream_enabled = !gps_stream_enabled;
        Serial.print("GPS stream ");
        Serial.println(gps_stream_enabled ? "ENABLED" : "DISABLED");
        break;

      case 'r':
        rtc_stream_enabled = !rtc_stream_enabled;
        Serial.print("RTC stream ");
        Serial.println(rtc_stream_enabled ? "ENABLED" : "DISABLED");
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

    Serial.print("BME600_SDI_pin          = "); Serial.println(BME600_SDI_pin);
    Serial.print("BME600_SCK_pin          = "); Serial.println(BME600_SCK_pin);

    Serial.print("GPS_RX_pin              = "); Serial.println(GPS_RX_pin);
    Serial.print("GPS_TX_pin              = "); Serial.println(GPS_TX_pin);
    Serial.print("GPS_PPS_pin             = "); Serial.println(GPS_PPS_pin);

    Serial.print("DS1307_SDA_pin          = "); Serial.println(DS1307_SDA_pin);
    Serial.print("DS1307_SCL_pin          = "); Serial.println(DS1307_SCL_pin);

    Serial.println("-----------------------");
    Serial.println();
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

// END -- END -- END -- END -- END -- END -- END -- END -- END -- END -- END