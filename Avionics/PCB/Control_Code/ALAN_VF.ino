/*
  ALAN VF not checked

  to check
    ORACLE
    GATHER FLIGHT STAGE
    PRESS INTERPOL
    MAIN LOOP ALL ORDER SEQUENCE CKHECK
    not whole list yet

  ALL IN METRIC IF NOT SAID OTHERWISE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/


// ============================ LIBRARIES ==========================

    

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

  #include <SoftwareSerial.h>
  #include <TinyGPSPlus.h>

  //#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
  #include <uRTCLib.h>
  #include <cmath>
// ======================= HARDWARE CONNECTIONS ====================

  // ---------------- PCB output pins: valve control -----------------
    // New PCB / diagram:
    // 1 - pin 2 - Main valve
    // 2 - pin 3 - Relief valve
    // 3 - pin 4 - Dump valve
    #define MAIN_VALVE_pin    2
    #define RELIEF_VALVE_pin  3
    #define DUMP_VALVE_pin    4
    
    // Valve connection voltage check pins
    #define MAIN_VALVE_CONNECTION_pin    38
    #define RELIEF_VALVE_CONNECTION_pin  37
    #define DUMP_VALVE_CONNECTION_pin    36

  // ---------------- PCB input pins: pressure transducers ------------
    // New PCB / diagram:
    // 1 - pin 20 - N2O/fuel tank
    // 2 - pin 40 - extra
    // 3 - pin 15 - combustion tank
    // 4 - pin 27 - CO2 pressure level
    const uint8_t PRESSURE_TRANSDUCER_1_pin = 20;
    const uint8_t PRESSURE_TRANSDUCER_2_pin = 40;
    const uint8_t PRESSURE_TRANSDUCER_3_pin = 15;
    const uint8_t PRESSURE_TRANSDUCER_4_pin = 27;

  // ---------------- Battery voltage input --------------------------
    // TODO: assign actual battery voltage divider/read pin.
    const int BATTERY_VOLTAGE_pin = -1;

  // ---------------- RFM9X radio pins -------------------------------
    #define RFM9X_G0_pin    9
    #define RFM9X_SCK_pin   13
    #define RFM9X_MISO_pin  12
    #define RFM9X_MOSI_pin  11
    #define RFM9X_CS_pin    10
    #define RFM9X_RST_pin   26
    #define RFM9X_PWR_pin   23

  // ---------------- Shared I2C bus LSM6DSOX + BME680 + DS1307 ------
    #define I2C_MAIN_SDA_pin 18
    #define I2C_MAIN_SCL_pin 19

  // ---------------- ADXL375 separate I2C bus ------------------------
    #define ADXL375_SDA_pin 17
    #define ADXL375_SCL_pin 16

  // ---------------- GPS SparkFun SAM-M8Q ----------------------------
    #define GPS_RX_pin   8   // receive GPS data from the pin where activity exists
    #define GPS_TX_pin   7
    #define GPS_PPS_pin  34

    SoftwareSerial GPS_PORT(GPS_RX_pin, GPS_TX_pin); // RX, TX
    TinyGPSPlus gps;
    //SoftwareSerial GPS_SOFT_SERIAL(GPS_RX_pin, GPS_TX_pin);
    //#define GPS_PORT GPS_SOFT_SERIAL

  // ---------------- Future debug display system ---------------------
    // TODO: display/debug screen pins are not known yet.
    // Reserved placeholders for future display SDA/SCL or display controller interface.
    const int DEBUG_DISPLAY_SDA_pin = -1;
    const int DEBUG_DISPLAY_SCL_pin = -1;

// ============================ CONSTANTS ==========================

  // Valve connection check
    const float VALVE_CONNECTION_MIN_VOLTAGE = 1.00f;

  // Radio frequency from latest SYS_Alan_V.4.1 reference
    float CONTROL_FREQ = 433.0f;
    const int RFM9X_TX_POWER_DBM = 23;

  // Time between component usages in microseconds
    const unsigned int RFM9x_rate    = 200000;
    const unsigned int Pres_rate     = 50000;
    const unsigned int LSM6DSOX_rate = 1000;
    const unsigned int ADXL375_rate  = 500;
    // const unsigned int BME680_rate   = 50000;
    const unsigned int GPS_rate      = 200000;
    const unsigned int DS1307_rate   = 5000;
    const unsigned int SD_rate       = 100000;

  // Pressure calibration placeholders from old code style.
    const float PRESSURE_1_SLOPE  = 1.0f;
    const float PRESSURE_1_OFFSET = 0.0f;
    const float PRESSURE_2_SLOPE  = 1.0f;
    const float PRESSURE_2_OFFSET = 0.0f;
    const float PRESSURE_3_SLOPE  = 1.0f;
    const float PRESSURE_3_OFFSET = 0.0f;
    const float PRESSURE_4_SLOPE  = 1.0f;
    const float PRESSURE_4_OFFSET = 0.0f;

    // ---------------- Pressure sensor calibration tables ----------------
      // Input = sensor output voltage after divider / ADC conversion
      // Output = pressure in PSI

      const int COMBUST_PRESSURE_TABLE_SIZE = 7;

      const float combust_voltage_table[COMBUST_PRESSURE_TABLE_SIZE] = {
        0.40f,
        0.49f,
        0.56f,
        0.67f,
        0.76f,
        1.01f,
        1.23f
      };

      const float combust_pressure_psi_table[COMBUST_PRESSURE_TABLE_SIZE] = {
        0.0f,
        200.0f,
        300.0f,
        500.0f,
        700.0f,
        1100.0f,
        1500.0f
      };


      const int N2O_PRESSURE_TABLE_SIZE = 7;

      const float n2o_voltage_table[N2O_PRESSURE_TABLE_SIZE] = {
        0.39f,
        0.68f,
        0.90f,
        1.18f,
        1.28f,
        1.29f,
        1.30f
      };

      const float n2o_pressure_psi_table[N2O_PRESSURE_TABLE_SIZE] = {
        0.0f,
        200.0f,
        500.0f,
        600.0f,
        700.0f,
        900.0f,
        1200.0f
      };


      const int CO2_PRESSURE_TABLE_SIZE = 7;

      const float co2_voltage_table[CO2_PRESSURE_TABLE_SIZE] = {
        0.27f,
        0.47f,
        0.63f,
        0.70f,
        0.81f,
        0.85f,
        1.16f
      };

      const float co2_pressure_psi_table[CO2_PRESSURE_TABLE_SIZE] = {
        0.0f,
        400.0f,
        700.0f,
        800.0f,
        1000.0f,
        1100.0f,
        1600.0f
      };

    const float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

  // Old ignition-system reference values.
    // These are kept as building blocks, not as final flight-stage logic.
    const uint32_t MAIN_VALVE_OPEN_AFTER_START_MS  = 28000;
    const uint32_t MAIN_VALVE_AUTO_CLOSE_DELAY_MS  = 4100; // ~11000ft
    const uint32_t MAX_ENGINE_ON_TIME_MS = MAIN_VALVE_AUTO_CLOSE_DELAY_MS;
    const uint32_t MIN_ENGINE_ON_TIME_MS = 3500; // ~9000ft
  
  // Oracle
    // tables
      const float Cd_from_V[342] = {
        0.50608240f, 0.50458680f, 0.50458457f, 0.50458116f, 0.49749070f, 0.51169724f,
        0.51941569f, 0.52226290f, 0.52316236f, 0.52264935f, 0.52118880f, 0.51935563f,
        0.51748371f, 0.51508699f, 0.51254705f, 0.50998470f, 0.46952259f, 0.50548293f,
        0.50299827f, 0.50057976f, 0.49858020f, 0.49230488f, 0.49396207f, 0.49199073f,
        0.49010486f, 0.48795614f, 0.48590470f, 0.48409090f, 0.48249905f, 0.48131982f,
        0.48063839f, 0.48004104f, 0.47932868f, 0.45217220f, 0.47803022f, 0.47736318f,
        0.47674797f, 0.47615915f, 0.47550953f, 0.45996762f, 0.47428329f, 0.47360679f,
        0.47296465f, 0.47240918f, 0.47174349f, 0.47113716f, 0.47055454f, 0.46998756f,
        0.46949175f, 0.46884482f, 0.46832728f, 0.46772486f, 0.46720088f, 0.46662900f,
        0.46610817f, 0.46555654f, 0.46503900f, 0.46450594f, 0.46399570f, 0.46351287f,
        0.46300246f, 0.46253179f, 0.46207101f, 0.46156408f, 0.46111888f, 0.46060592f,
        0.46017974f, 0.45965851f, 0.45920370f, 0.45880050f, 0.45833391f, 0.45791388f,
        0.45747401f, 0.45698447f, 0.45657868f, 0.45617910f, 0.45574739f, 0.45530083f,
        0.45490884f, 0.45452145f, 0.45412584f, 0.45375720f, 0.45341763f, 0.43195053f,
        0.45272610f, 0.45239902f, 0.45205788f, 0.45171986f, 0.45139656f, 0.45106843f,
        0.43024096f, 0.45043167f, 0.45011752f, 0.44980593f, 0.44949944f, 0.44919183f,
        0.44889425f, 0.44858818f, 0.44827987f, 0.44798957f, 0.43472654f, 0.44743515f,
        0.44713225f, 0.44687027f, 0.44656175f, 0.44627734f, 0.44605052f, 0.44574376f,
        0.44544236f, 0.44524020f, 0.44494430f, 0.44464591f, 0.44443379f, 0.44418700f,
        0.44392298f, 0.44361337f, 0.44339443f, 0.44308466f, 0.43620657f, 0.44258348f,
        0.44240524f, 0.44208996f, 0.44186205f, 0.44164453f, 0.44136293f, 0.44119218f,
        0.43364849f, 0.44066036f, 0.44045499f, 0.44024360f, 0.44003038f, 0.43981751f,
        0.43960718f, 0.43940351f, 0.43922333f, 0.43907611f, 0.43861582f, 0.43845801f,
        0.43815960f, 0.43812343f, 0.43772436f, 0.43764965f, 0.43741678f, 0.43708053f,
        0.43711456f, 0.43669909f, 0.41189370f, 0.43638893f, 0.43612080f, 0.43617428f,
        0.43561763f, 0.43588790f, 0.43526244f, 0.43542459f, 0.43480371f, 0.43511737f,
        0.43457127f, 0.43420256f, 0.43408812f, 0.43391513f, 0.43420507f, 0.43365507f,
        0.43326868f, 0.43359709f, 0.43290059f, 0.43263343f, 0.43258588f, 0.43291254f,
        0.43260783f, 0.43186452f, 0.43167364f, 0.43163819f, 0.43150925f, 0.43146355f,
        0.43146367f, 0.43143873f, 0.43056434f, 0.43102257f, 0.43016272f, 0.43059176f,
        0.42999837f, 0.42960180f, 0.42941531f, 0.42922900f, 0.42956721f, 0.42963085f,
        0.42867105f, 0.42884615f, 0.42829950f, 0.42811384f, 0.42792804f, 0.42774230f,
        0.42755645f, 0.42826694f, 0.41173178f, 0.42802330f, 0.42698348f, 0.42759245f,
        0.42704591f, 0.42660721f, 0.42634103f, 0.42664204f, 0.42611772f, 0.42788053f,
        0.41476277f, 0.42676773f, 0.42562147f, 0.42549864f, 0.42537477f, 0.42653542f,
        0.42512378f, 0.42499648f, 0.42486785f, 0.42473779f, 0.42677347f, 0.42447315f,
        0.42433828f, 0.42420164f, 0.41274582f, 0.42392247f, 0.42377964f, 0.42014837f,
        0.42348671f, 0.42333637f, 0.42318309f, 0.42302680f, 0.42493182f, 0.42631547f,
        0.42253722f, 0.42236624f, 0.42219086f, 0.42201071f, 0.42182536f, 0.42163440f,
        0.42143733f, 0.42220161f, 0.42102240f, 0.42080311f, 0.42057489f, 0.42033652f,
        0.42050919f, 0.41982466f, 0.42304764f, 0.41925423f, 0.41899964f, 0.41860514f,
        0.41824125f, 0.41812402f, 0.41818329f, 0.41823715f, 0.41828321f, 0.41831684f,
        0.41832438f, 0.42564126f, 0.42560434f, 0.42556797f, 0.42553200f, 0.41257913f,
        0.42546139f, 0.42542679f, 0.42539264f, 0.42535894f, 0.42532567f, 0.42529282f,
        0.40855132f, 0.41837848f, 0.42519690f, 0.42516578f, 0.42513516f, 0.42510485f,
        0.41697603f, 0.40700242f, 0.40764821f, 0.41808173f, 0.42495980f, 0.42493203f,
        0.42490462f, 0.42487765f, 0.42485108f, 0.42482492f, 0.42479909f, 0.41372034f,
        0.40552422f, 0.40415545f, 0.40647322f, 0.42467611f, 0.41317629f, 0.42462958f,
        0.41732346f, 0.42458461f, 0.42226970f, 0.42454107f, 0.42451986f, 0.42449904f,
        0.42447858f, 0.42445846f, 0.42431225f, 0.42441990f, 0.42462319f, 0.42529715f,
        0.42644266f, 0.42805967f, 0.42650057f, 0.42494129f, 0.42402250f, 0.42333445f,
        0.42197308f, 0.40064964f, 0.43070126f, 0.42876534f, 0.42880988f, 0.46099660f,
        0.46136878f, 0.46822406f, 0.47183768f, 0.47545138f, 0.44911075f, 0.42658263f,
        0.43461759f, 0.46762541f, 0.49351983f, 0.49713367f, 0.50074733f, 0.47210666f,
        0.50797533f, 0.51158924f, 0.51520339f, 0.51881763f, 0.50027590f, 0.49250367f,
        0.49567541f, 0.46630557f, 0.50202053f, 0.50519324f, 0.47865536f, 0.51153932f,
        0.50755861f, 0.49216170f, 0.52106049f, 0.50473782f, 0.52740946f, 0.51808735f
      };

      const float rho_from_altitude[202] = {
        1.22500002f, 1.21913063f, 1.21328278f, 1.20745642f, 1.20165148f, 1.19586792f,
        1.19010568f, 1.18436471f, 1.17864494f, 1.17294633f, 1.16726883f, 1.16161237f,
        1.15597691f, 1.15036238f, 1.14476874f, 1.13919594f, 1.13364391f, 1.12811260f,
        1.12260197f, 1.11711195f, 1.11164250f, 1.10619356f, 1.10076507f, 1.09535699f,
        1.08996926f, 1.08460182f, 1.07925463f, 1.07392764f, 1.06862077f, 1.06333400f,
        1.05806726f, 1.05282050f, 1.04759366f, 1.04238670f, 1.03719957f, 1.03203220f,
        1.02688455f, 1.02175657f, 1.01664820f, 1.01155939f, 1.00649010f, 1.00144026f,
        0.99640983f, 0.99139875f, 0.98640697f, 0.98143445f, 0.97648112f, 0.97154694f,
        0.96663186f, 0.96173583f, 0.95685879f, 0.95200069f, 0.94716148f, 0.94234112f,
        0.93753955f, 0.93275671f, 0.92799257f, 0.92324706f, 0.91852014f, 0.91381176f,
        0.90912186f, 0.90445040f, 0.89979733f, 0.89516259f, 0.89054614f, 0.88594792f,
        0.88136789f, 0.87680599f, 0.87226219f, 0.86773641f, 0.86322863f, 0.85873878f,
        0.85426681f, 0.84981269f, 0.84537635f, 0.84095776f, 0.83655685f, 0.83217359f,
        0.82780791f, 0.82345978f, 0.81912915f, 0.81481596f, 0.81052016f, 0.80624172f,
        0.80198057f, 0.79773667f, 0.79350998f, 0.78930044f, 0.78510800f, 0.78093263f,
        0.77677426f, 0.77263285f, 0.76850836f, 0.76440073f, 0.76030992f, 0.75623588f,
        0.75217856f, 0.74813791f, 0.74411389f, 0.74010645f, 0.73611555f, 0.73214112f,
        0.72818314f, 0.72424154f, 0.72031629f, 0.71640734f, 0.71251463f, 0.70863813f,
        0.70477778f, 0.70093354f, 0.69710536f, 0.69329320f, 0.68949701f, 0.68571674f,
        0.68195234f, 0.67820378f, 0.67447100f, 0.67075396f, 0.66705261f, 0.66336690f,
        0.65969680f, 0.65604225f, 0.65240321f, 0.64877963f, 0.64517147f, 0.64157868f,
        0.63800121f, 0.63443903f, 0.63089209f, 0.62736033f, 0.62384372f, 0.62034221f,
        0.61685575f, 0.61338431f, 0.60992783f, 0.60648627f, 0.60305958f, 0.59964773f,
        0.59625067f, 0.59286835f, 0.58950072f, 0.58614775f, 0.58280939f, 0.57948559f,
        0.57617632f, 0.57288152f, 0.56960116f, 0.56633518f, 0.56308355f, 0.55984623f,
        0.55662316f, 0.55341431f, 0.55021962f, 0.54703907f, 0.54387260f, 0.54072017f,
        0.53758174f, 0.53445726f, 0.53134669f, 0.52825000f, 0.52516713f, 0.52209804f,
        0.51904269f, 0.51600105f, 0.51297305f, 0.50995867f, 0.50695786f, 0.50397057f,
        0.50099678f, 0.49803642f, 0.49508947f, 0.49215587f, 0.48923559f, 0.48632859f,
        0.48343482f, 0.48055424f, 0.47768681f, 0.47483249f, 0.47199123f, 0.46916300f,
        0.46634775f, 0.46354545f, 0.46075604f, 0.45797949f, 0.45521577f, 0.45246482f,
        0.44972660f, 0.44700109f, 0.44428822f, 0.44158797f, 0.43890030f, 0.43622515f,
        0.43356250f, 0.43091230f, 0.42827451f, 0.42564909f, 0.42303600f, 0.42043521f,
        0.41784666f, 0.41527032f, 0.41270615f, 0.41015412f
      };

      int APOGEE_V_MAX_FOR_TABLE = 340;
      const float RHO_ALTITUDE_STEP_INV = 0.02f;
      const float RHO_MAX_ALTITUDE_FOR_TABLE = 10000.0f;
      const float APOGEE_V_STEP_INV = 1.0f;



    float manual_zero_alt = 0.0f;
    int desired_apogee = 10000; // in ft need to convert to meters
    float desired_apogee_m = desired_apogee * 0.3048f;
    float no_fuel_rocket_mass = 36.287;
    float initial_fuel_mass = 3.6f + 8.528f;
    float fuel_mass = initial_fuel_mass;
    // float distance_from_PCB_to_stratalogger_along_the_rocket = 10; // in ft
    

// ============================ OBJECTS ============================

  ADC *adc = new ADC();
  File Black_Box;

  char black_box_filename[13] = "BLACK000.CSV";

  RH_RF95 rf95(RFM9X_CS_pin, RFM9X_G0_pin);

  Adafruit_LSM6DSOX lsm6dsox;
  Adafruit_ADXL375 adxl375 = Adafruit_ADXL375(12345, &Wire1);
  Adafruit_BME680 BME680;
  //SFE_UBLOX_GNSS myGNSS;
  uRTCLib rtc(0x68, URTCLIB_MODEL_DS1307);

// ======================= MESSAGE STRUCTURES ======================

  // Command message class
  uint8_t msg_class_01[22][10] = {
    {1,1,1},   // 0x01 Ignition Abort
    {1,1,1},   // 0x02 Avionics & Pad Arm
    {1,1,1},   // 0x03 Ignition Sequence Start
    {1,1,1},   // 0x04 Main Valve Open
    {1,1,1},   // 0x05 Main Valve Closed
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

  uint8_t msg_class_02[7][10] = {
    {5,5,5,5,5,5,5,5,5,5}, // 0x01 AV1 Telemetry
    {5,5,5,5,5,5,5,5,5,5}, // 0x02 AV2 Telemetry
    {5,5,5,5,5,5,4},       // 0x03 Pressures/battery/altitude/command count
    {4,5,5,5},             // 0x04 Status bit array, pressures, battery voltage
    {4,5,5,5},             // 0x05 Status bit array, pressures, battery voltage
    {4}                    // 0x06 State array
    // 0x07 GPS location packet
    // Sends: gps_lat, gps_lon, gps_alt_m
    // Types: float, float, float
    {5,5,5}                // 0x07 GPS location
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


// ======================== DYNAMIC VARIABLES ======================

  // ---------------- Fail flags -------------------------------------
    bool RFM9X_FAIL    = false;
    bool TX_enabled    = true;
    bool SD_FAIL       = false;
    bool LSM6DSOX_FAIL = false;
    bool ADXL375_FAIL  = false;
    bool BME680_FAIL   = false;
    bool GPS_FAIL      = false;
    bool DS1307_FAIL   = false;
    bool DISPLAY_FAIL  = true;   // display system is not defined yet

  // ---------------- Old ignition-system state -----------------------
    bool avionics_armed = false;
    bool ignition_sequence_started = false;
    bool main_valve_can_close_by_radio = true;
    bool main_valve_ignition_hold_active = false;
    uint32_t main_valve_ignition_start_time_ms = 0;
    int8_t countdown = -20;

    // ARM stage sequence state
      bool arm_sequence_active = false;
      bool arm_sequence_started_once = false;
      uint32_t arm_sequence_start_time_ms = 0;
      int8_t arm_countdown = -20;

    // Time from ARM stage start to launch ignition sequence stage
    // This follows old ignition timing idea.
      const uint32_t ARM_SEQUENCE_DURATION_MS = 28000;  

  // ---------------- Pressure / battery outputs ----------------------
    float pressure_1_output = 0.0f;
    float pressure_2_output = 0.0f;
    float pressure_3_output = 0.0f;
    float pressure_4_output = 0.0f;
    float battery_voltage_output = 0.0f;

  // ---------------- LSM6DSOX outputs --------------------------------
    float lsm_ax = 0.0f;
    float lsm_ay = 0.0f;
    float lsm_az = 0.0f;
    float lsm_gx = 0.0f;
    float lsm_gy = 0.0f;
    float lsm_gz = 0.0f;
    float lsm_temp = 0.0f;

  // ---------------- ADXL375 outputs ---------------------------------
    float adxl_ax = 0.0f;
    float adxl_ay = 0.0f;
    float adxl_az = 0.0f;

  // ---------------- BME680 outputs ----------------------------------
    float BME680_temp = 0.0f;
    float BME680_humidity = 0.0f;
    float BME680_pressure_hpa = 0.0f;
    float BME680_gas_kohm = 0.0f;
    float BME680_altitude_m = 0.0f;

    bool BME680_new_data = false;
    uint32_t BME680_last_new_data_us = 0;
    float BME680_update_hz = 0.0f;

    float BME680_filtered_pressure_hpa = 0.0f;
    float BME680_pressure_threshold_hpa = 0.0f;
    float BME680_calibration_duration_s = 5.0f;
    bool BME680_filter_started = false;

  // ---------------- GPS outputs -------------------------------------
    double gps_lat = 0.0;
    double gps_lon = 0.0;
    float gps_alt_m = 0.0f;
    uint8_t gps_siv = 0;
    uint8_t gps_fix_type = 0;
    bool gps_pps_state = false;

  // ---------------- DS1307 outputs ----------------------------------
    uint8_t rtc_year = 0;
    uint8_t rtc_month = 0;
    uint8_t rtc_day = 0;
    uint8_t rtc_hour = 0;
    uint8_t rtc_minute = 0;
    uint8_t rtc_second = 0;

  // ---------------- Message parser output arrays --------------------
    uint8_t  out_flag[10]   = {0};
    uint8_t  out_int8s[10]  = {0};
    uint16_t out_int16s[10] = {0};
    uint32_t out_int32s[10] = {0};
    float    out_floats[10] = {0};


  // ---------------- Message assembler input arrays ------------------
    uint8_t  in_flag[10]   = {0};
    uint8_t  in_int8s[10]  = {0};
    uint16_t in_int16s[10] = {0};
    uint32_t in_int32s[10] = {0};
    float    in_floats[10] = {0};

  // ---------------- Message assembler buffer/state ------------------
    uint8_t message_send_buf[43] = {0};
    uint8_t message_send_len = 0;
    uint8_t msg_ready = 0;
    uint8_t recv_ready = 0;

    uint32_t commands_received = 0;
    uint8_t telemetry_select_index = 0;

  // ---------------- Timers ------------------------------------------
    unsigned int RFM9x_last_time    = 0;
    unsigned int Pressure_last_time = 0;
    unsigned int LSM6DSOX_last_time = 0;
    unsigned int ADXL375_last_time  = 0;
    unsigned int BME680_last_time   = 0;
    unsigned int GPS_last_time      = 0;
    unsigned int DS1307_last_time   = 0;
    unsigned int SD_last_time       = 0;

    uint32_t bme_ready_time = 0;
    bool bme_busy = false;

  // ---------------- Debug stream flags ------------------------------
    bool imu_stream_enabled = false;
    bool pressure_stream_enabled = false;
    bool bme_stream_enabled = false;
    bool gps_stream_enabled = false;
    bool rtc_stream_enabled = false;
    bool valve_connection_stream_enabled = false;

  // ---------------- Stages ------------------------------------------
    enum StageId : uint8_t {
      STAGE_WAITING = 1,
      STAGE_ARM = 2,
      STAGE_LAUNCH_IGNITION_SEQUENCE = 3,
      STAGE_ENGINE_WORKING = 4,
      STAGE_DATA_GATHERING_FLIGHT = 5
    };

    uint8_t stage = STAGE_WAITING;
    uint8_t previous_stage = STAGE_WAITING;
    uint32_t stage_entry_time_ms = 0;

    bool no_return_flight_started = false;
    bool main_valve_opened_for_flight = false;
    uint32_t flight_start_time_ms = 0;

    // ---------------- Stage 5 / post-engine apogee relief logic --------
      bool apogee_detected = false;
      bool relief_opened_after_apogee = false;

      float previous_kalman_vertical_speed_mps = 0.0f;
      bool previous_vertical_speed_valid = false;

      float apogee_detection_vz_threshold_mps = 0.5f;

      uint32_t apogee_detected_time_ms = 0;

      // Relief valve timing after engine cutoff / Stage 5 entry
      const uint32_t MIN_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS = 1000;   // do not open before this
      const uint32_t MAX_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS = 15000;  // force open after this
      const uint32_t RELIEF_OPEN_DELAY_AFTER_APOGEE_MS = 0;             // optional delay after apogee

      // GPS location radio send rate
      uint32_t GPS_location_send_last_time_us = 0;
      uint32_t GPS_location_send_rate_us = 1000000; // 1 Hz

  // ---------------- All math for ORACLE and varibles ----------------
    

    struct Vector3f {
      float x;
      float y;
      float z;
    };

    struct Matrix3f {
      float m[3][3];
    };

    struct LocationXYZ {
      float x_m;
      float y_m;
      float z_m;
    };

    struct Quaternionf {
      float w;
      float x;
      float y;
      float z;
    };

    // Acceleration of PCB in global/world frame.
      // Created from LSM6DSOX + ADXL375, then rotated from local PCB frame to global frame.
      Vector3f PCB_global_acc_vector = {0.0f, 0.0f, 0.0f};
      Vector3f PCB_local_specific_force_vector = {0.0f, 0.0f, 0.0f};

      // PCB angular velocity / rotation rate.
      // From LSM6DSOX gyro, converted to PCB coordinate convention.
      float PCB_gx = 0.0f;
      float PCB_gy = 0.0f;
      float PCB_gz = 0.0f;

      Vector3f PCB_global_gyro_vector = {0.0f, 0.0f, 0.0f};

      float gyro_x_offset = 0.0f;
      float gyro_y_offset = 0.0f;
      float gyro_z_offset = 0.0f;

      float Calibrated_Gyro_x = 0;
      float Calibrated_Gyro_y = 0;
      float Calibrated_Gyro_z = 0;

      float BME_alt_offset = 0;

      // Change of PCB orientation relative to previous orientation.
      // This can later be calculated from PCB_gx, PCB_gy, PCB_gz and delta time.
      Vector3f local_direction_change = {0.0f, 0.0f, 0.0f};

      float PCB_acc_filter_threshold = 0.05f; // m/s^2, ignore smaller changes

      float last_PCB_acc_x = 0.0f;
      float last_PCB_acc_y = 0.0f;
      float last_PCB_acc_z = 0.0f;

      bool PCB_acc_filter_started = false;

      Quaternionf PCB_orientation_quat = {1.0f, 0.0f, 0.0f, 0.0f};

      bool PCB_orientation_quat_valid = false;
      bool gyro_orientation_debug_enabled = false;

      float PCB_gyro_deadband_rad_s = 0.002f; // ignore tiny gyro noise
      float PCB_gyro_mag = 0.0f;

      uint32_t PCB_orientation_last_update_us = 0;
      float PCB_orientation_dt_s = 0.0f;

      float rocket_tilt_angle_rad = 0.0f;
      float rocket_vertical_cos = 1.0f;
      float rocket_tilt_angle_deg = 0.0f;


    // PCB local coordinate axes expressed in global/world frame.
      // expl comm
        // global X = Horizontal Axis 1
        // global Y = Horizontal Axis 2
        // global Z = Up

        // Row/column meaning:
        // x_axis_global = local_directions_axis_vectors_in_global.m[0]
        // y_axis_global = local_directions_axis_vectors_in_global.m[1]
        // z_axis_global = local_directions_axis_vectors_in_global.m[2]
      Matrix3f local_directions_axis_vectors_in_global = {
        {
          {1.0f, 0.0f, 0.0f},
          {0.0f, 1.0f, 0.0f},
          {0.0f, 0.0f, 1.0f}
        }
      };

      // expl comm
        // Current estimated velocity of PCB/rocket in global/world frame.
        // Later: integration of acceleration + correction from BME/GPS/engine model.
      Vector3f PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};

      // expl comm
        // Current estimated location of PCB/rocket.
        // GPS gives lat/lon/alt; BME can help altitude; velocity integration can fill between GPS updates.
      LocationXYZ global_location = {0.0, 0.0, 0.0f};

      // Optional extra state values for future integration.
      Vector3f PCB_local_acc_vector = {0.0f, 0.0f, 0.0f};
      Vector3f PCB_local_gyro_vector = {0.0f, 0.0f, 0.0f};

      Vector3f gravity = {0.0f, 0.0f, 0.0f}; // will be determent by acc modules and used later for integration

      Vector3f previous_PCB_global_acc_vector = {0.0f, 0.0f, 0.0f};
      Vector3f previous_PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};

      bool gravity_anchor_valid = false;
      bool oracle_debug_enabled = false;

      float gravity_anchor_acc_mag = 0.0f;

      Vector3f global_up_vector = {0.0f, 0.0f, 1.0f};
      Vector3f global_gravity_vector = {0.0f, 0.0f, -9.80665f};

      // This is the acceleration after gravity is removed.
      // real_global_acc = PCB_global_acc_vector + global_gravity_vector
      Vector3f PCB_real_global_acc_vector = {0.0f, 0.0f, 0.0f};

    uint32_t oracle_last_update_us = 0;
    float oracle_delta_time_s = 0.0f;

    // module red flags
      bool GPS_makes_sense = true;
      bool BME_makes_sense = true;
      bool ADXL_makes_sense = true;
      bool LSM_makes_sense = true;
      
    
    //PCB_location_next_step_simulation
    //PCB_location_last_step_simulation

    

    // vis
      bool orientation_visualizer_enabled = false;
      unsigned int orientation_visualizer_last_time = 0;
      const unsigned int ORIENTATION_VISUALIZER_RATE_US = 5000;

    bool rocket_tilt_stream_enabled = false;

    // Barometer altitude Kalman filter
      float kalman_altitude_m = 0.0f;
      float kalman_vertical_speed_mps = 0.0f;

      // Covariance matrix P
      float kalman_P00 = 1.0f;
      float kalman_P01 = 0.0f;
      float kalman_P10 = 0.0f;
      float kalman_P11 = 1.0f;

      // Tuning values
      float kalman_R_altitude = 1.0f;   // measurement noise, m^2
      float kalman_Q_altitude = 0.05f;  // model noise for altitude
      float kalman_Q_velocity = 2.0f;   // model noise for velocity

      uint32_t kalman_last_update_us = 0;
      float kalman_dt_s = 0.0f;

      bool kalman_altitude_initialized = false;

      float rocket_speed_mps = 0.0f;
      bool rocket_speed_valid = false;

      float min_vertical_cos_for_speed = 0.30f;

      float raw_vertical_speed_mps = 0.0f;
      float previous_raw_altitude_m = 0.0f;
      uint32_t raw_vertical_speed_last_update_us = 0;
      bool raw_vertical_speed_initialized = false;

      float rocket_speed_raw_mps = 0.0f;
      bool rocket_speed_raw_valid = false;

      bool rocket_speed_stream_enabled = false;

    // alt prediction
      int stratologger_apogee_prediction = 0;

      float rocket_mass = no_fuel_rocket_mass + fuel_mass;

      const float PSI_TO_PA = 6894.757f;

      float oxidizer_mass_used_kg = 0.0f;
      float oxidizer_mass_flow_kg_s = 0.0f;

      bool use_mass_flow_integration_for_rocket_mass = true;
      bool oxidizer_mass_integration_enabled = false;

      float nitrous_density_kg_m3 = 750.0f;
      float oxidizer_flow_coefficient_area_m2 = 1.11e-4f;
      float oxidizer_flow_multiplier = 0.5f;

      
      float predicted_apogee_m = 0.0f;
      uint32_t apogee_prediction_time_us = 0;
      bool apogee_prediction_valid = false;

      float apogee_prediction_dt_s = 0.01f;
      float apogee_prediction_max_time_s = 50.0f;

      float rocket_cross_sectional_area_m2 = 0.0182f;

      int apogee_prediction_step_count = 0;


// ============================ VALVE STRUCT =======================

  struct Valve {
    uint8_t id;
    uint8_t control_pin;
    uint8_t connection_pin;
    const char* name;
    bool state;
    bool active_low;
    float connection_voltage;
    bool connection_ok;
  };

  Valve MAIN_VALVE   = {1, MAIN_VALVE_pin,   MAIN_VALVE_CONNECTION_pin,   "main",   false, false, 0.0f, false};
  Valve RELIEF_VALVE = {2, RELIEF_VALVE_pin, RELIEF_VALVE_CONNECTION_pin, "relief", false, true,  0.0f, false};
  Valve DUMP_VALVE   = {3, DUMP_VALVE_pin,   DUMP_VALVE_CONNECTION_pin,   "dump",   false, false, 0.0f, false};

  enum CommandSource : uint8_t {
    COMMAND_FROM_RADIO = 0,
    COMMAND_FROM_DEBUG = 1
  };


// ======================= FUNCTION DECLARATIONS ===================

  // ---------------- Setup / system init ----------------------------
    void setup_valves();
    void setup_inputs();
    void setup_adc();
    void setup_i2c();
    void setup_radio();
    void print_start_banner();
    void print_setup_summary();
    void setup_valve_connection_check();

  // ---------------- Input / sensor read section --------------------
    float read_voltage(uint8_t pin);
    void read_battery_voltage();
    void read_pressure();

    float interpolate_voltage_to_pressure_psi(
      float voltage,
      const float voltage_table[],
      const float pressure_table[],
      int table_size
    );

    float combust_voltage_to_pressure_psi(float voltage);
    float n2o_voltage_to_pressure_psi(float voltage);
    float co2_voltage_to_pressure_psi(float voltage);

    void init_LSM6DSOX();
    void read_LSM6DSOX();

    void init_ADXL375();
    void read_ADXL375();

    void init_BME680();
    // void read_BME680();

    void start_BME680_read();
    void finish_BME680_read();
    void calibrate_BME680_pressure_threshold();
    void apply_BME680_pressure_deadband();
    

    void init_GPS();
    void read_GPS();
    void raw_GPS_UART_test();

    void init_DS1307();
    void read_DS1307();

   

  // ---------------- PCB output / control section -------------------
    void VALVE_open(Valve& valve);
    void VALVE_close(Valve& valve);
    
    void close_all_valves();
    void update_main_valve_ignition_hold();
    void update_debug_display();
    void update_laptop_debug();

  // ---------------- Stage ------------------------------------------
    void run_stage_loop();
    void set_stage(uint8_t new_stage);
    const char* stage_name(uint8_t stage_id);
    bool stage_can_return_to_waiting(uint8_t stage_id);

    void stage_waiting_loop();
    void stage_arm_loop();
    void stage_launch_ignition_sequence_loop();
    void stage_engine_working_loop();
    void stage_data_gathering_flight_loop();

  // ---------------- ARM stage --------------------------------------
    void start_arm_sequence();
    void update_arm_sequence();
    void reset_arm_sequence();
    void print_arm_sequence_status();

  // ---------------- ignition stage ---------------------------------
    void enter_no_return_flight();
    bool stage_transition_allowed(uint8_t current_stage, uint8_t new_stage);

  // ---------------- Valve connection -------------------------------
    Valve* get_valve_by_id(uint8_t valve_id);
    void read_valve_connection();
    void valve_connection_ckeck_all();
    bool valve_connection_ckeck(uint8_t valve_id);
    bool valve_connection_check(uint8_t valve_id);
    float valve_connection_voltage_read(uint8_t valve_id);
    void print_valve_connection_status();

  // ---------------- Radio / telemetry / message section ------------
    void read_RFM();
    void send_RFM();
    void send_ACK_packet(uint8_t computer_id);
    void prep_telemetry();
    void prep_telemetry_pressures();
    void prep_telemetry_states();
    uint32_t compute_state_array();
    uint8_t radio_checksum(uint8_t* radioMSG, uint8_t msgLength);
    void message_parser(uint8_t* buf);
    void message_assembler(uint8_t msg_class, uint8_t msg_id);

  // ---------------- Command execution section ----------------------
    bool command_flags_confirmed();
    bool execute_radio_message(uint8_t msg_class, uint8_t msg_id);
    bool execute_debug_command(uint8_t msg_id);
    bool execute_command_id(uint8_t msg_id, CommandSource source);
    const char* command_source_name(CommandSource source);
    void clear_command_flags();

    // Old compatibility wrapper
    bool perform_command(uint8_t msg_class, uint8_t msg_id, bool from_radio);

  // ---------------- SD log section ---------------------------------
    void init_SD();
    void create_new_black_box_file();
    void write_sd_header();
    void save_to_sd();

  // ----------------------- ORACLE + math ---------------------------
    void ORACLE_update();

    void calibrate_all();
    void set_to_start_location();

    void LSM_to_PCB_swithch_over(); // LSM orientation to rocket orientation
    void ADXL_to_PCB_swithch_over();
    void current_rocket_mass();
    void local_PCB_direction_determination();

    void calculate_rocket_tilt_from_orientation();

    void global_PCB_vel_vertor();
    void global_PCB_location_vector();

    void calculate_rocket_speed_from_vertical_speed();
    void update_raw_vertical_speed();
    void stream_rocket_speed_status();

    void apogee_prediction();

    float lookup_Cd_from_speed(float speed_mps);
    float lookup_rho_from_altitude(float altitude_m);

    float rocket_apogee_fast_cpp(float tilt_angle_rad,
                                float rocket_mass_kg,
                                float vertical_velocity_mps,
                                float altitude_m,
                                float dt_s);

    void print_apogee_prediction_status();

    void gravity_anchor();
    

    Vector3f vector_add(Vector3f a, Vector3f b);
    Vector3f vector_sub(Vector3f a, Vector3f b);
    Vector3f vector_scale(Vector3f a, float s);
    float vector_dot(Vector3f a, Vector3f b);
    Vector3f vector_cross(Vector3f a, Vector3f b);
    float vector_magnitude(Vector3f a);
    Vector3f vector_normalize(Vector3f a);

    Matrix3f matrix_identity();
    Matrix3f matrix_add(Matrix3f A, Matrix3f B);
    Matrix3f matrix_scale(Matrix3f A, float s);
    Matrix3f matrix_multiply(Matrix3f A, Matrix3f B);
    Matrix3f skew_matrix(Vector3f v);
    Matrix3f rotation_matrix_from_vector_to_vector(Vector3f from_vec, Vector3f to_vec);

    Vector3f local_to_global_vector(Vector3f local_vec);
    void update_real_global_acc_vector();

    void print_orientation_for_visualizer();
    void update_orientation_visualizer_output();

    Quaternionf quaternion_identity();
    Quaternionf quaternion_normalize(Quaternionf q);
    Quaternionf quaternion_multiply(Quaternionf a, Quaternionf b);
    Quaternionf quaternion_from_rotation_vector(Vector3f rotation_vector);
    Vector3f quaternion_rotate_vector(Quaternionf q, Vector3f v);

    void quaternion_to_local_direction_matrix();
    void local_direction_matrix_to_quaternion();

    void local_PCB_direction_determination();
    void print_gyro_orientation_status();

    void update_altitude_kalman_filter();
    void reset_altitude_kalman_filter();
    void print_altitude_kalman_status();

  // ---------------- Stage 5 / post-engine apogee relief logic --------

    void reset_data_gathering_flight_flags();
    void detect_apogee_after_engine_cutoff();
    void update_relief_after_apogee();
    void prep_current_location_message();
    void send_current_location();


  // ---------------- Data conversion / helper section ---------------
    void float_to_bytes(float value, uint8_t* out);
    void int_16_to_bytes(uint16_t value, uint8_t* out);
    void int_32_to_bytes(uint32_t value, uint8_t* out);
    float bytes_to_float(uint8_t* data);
    uint16_t bytes_to_int_16(uint8_t* data);
    uint32_t bytes_to_int_32(uint8_t* data);

  // ---------------- Debug / local test section ---------------------
    void debug_menu();
    void debug_serial();
    void print_hex_buffer(const uint8_t* buf, uint8_t len);
    void print_local_status();
    void print_ignition_control_status();
    void print_active_pins();
    void run_valve_self_test();
    void inject_local_command(uint8_t msg_id);
    void debug_arm_av1();
    void debug_abort_av1();
    void debug_start_ignition_sequence();
    void run_full_local_ignition_test();
    void streams();
    void print_load_progress(int prog);
    void print_GPS_status();
    void print_RTC_status();
    int read_serial_int(const char* prompt);
    void set_DS1307_from_serial();

    void print_BME680_pressure_filter_status();
    
    void print_gravity_anchor_status();
    void print_gyro_orientation_status();

    void debug_laptop();


// ============================ SETUP ==============================

  void setup() {
    Serial.begin(115200);
    delay(1000);

    print_start_banner();
    delay(1000);

    print_load_progress(0);
    
    setup_valves();
    setup_valve_connection_check();
    setup_inputs();
    setup_adc();
    setup_i2c();

    print_load_progress(30);
    
    init_LSM6DSOX();
    print_load_progress(40);
    init_ADXL375();
    print_load_progress(50);
    init_BME680();
    calibrate_BME680_pressure_threshold();
    print_load_progress(70);
    init_GPS();
    print_load_progress(80);
    init_DS1307();
    init_SD();
    print_load_progress(90);
    setup_radio();
    print_load_progress(100);

    read_valve_connection();
    print_setup_summary();
    debug_menu();

    calibrate_all();
    set_to_start_location();

    Serial.println("ALAN V5 base setup complete");
  }

// ============================ MAIN LOOP ==========================

  void loop() {
    update_laptop_debug();
    run_stage_loop();
    //stage_engine_working_loop();
    save_to_sd();
    //update_orientation_visualizer_output();
  }
// ============================== STAGES ===========================
  void run_stage_loop() {
    switch (stage) {
      case STAGE_WAITING:
        stage_waiting_loop();
        break;

      case STAGE_ARM:
        stage_arm_loop();
        break;

      case STAGE_LAUNCH_IGNITION_SEQUENCE:
        stage_launch_ignition_sequence_loop();
        break;

      case STAGE_ENGINE_WORKING:
        stage_engine_working_loop();
        break;

      case STAGE_DATA_GATHERING_FLIGHT:
        stage_data_gathering_flight_loop();
        break;

      default:
        Serial.println("Unknown stage. Returning to waiting stage.");
        set_stage(STAGE_WAITING);
        break;
    }
  }

  void set_stage(uint8_t new_stage) {
    if (new_stage == stage) {
      return;
    }

    if (!stage_transition_allowed(stage, new_stage)) {
      Serial.print("BLOCKED STAGE CHANGE: ");
      Serial.print(stage_name(stage));
      Serial.print(" -> ");
      Serial.println(stage_name(new_stage));
      return;
    }

    previous_stage = stage;
    stage = new_stage;
    stage_entry_time_ms = millis();

    if (new_stage == STAGE_DATA_GATHERING_FLIGHT) {
      reset_data_gathering_flight_flags();
    }

    Serial.print("Stage changed: ");
    Serial.print(stage_name(previous_stage));
    Serial.print(" -> ");
    Serial.println(stage_name(stage));
  }

  bool stage_transition_allowed(uint8_t current_stage, uint8_t new_stage) {
    // After Stage 4 starts, code can never return to stages 1, 2, or 3.
    if (no_return_flight_started && new_stage < STAGE_ENGINE_WORKING) {
      return false;
    }

    // Stage 4 and 5 are no-return flight stages.
    // They cannot go backwards to pre-flight / ignition stages.
    if (current_stage >= STAGE_ENGINE_WORKING && new_stage < STAGE_ENGINE_WORKING) {
      return false;
    }

    // Stage 5 should not return to Stage 4 unless we explicitly decide otherwise later.
    if (current_stage == STAGE_DATA_GATHERING_FLIGHT && new_stage != STAGE_DATA_GATHERING_FLIGHT) {
      return false;
    }

    return true;
  }

  const char* stage_name(uint8_t stage_id) {
    switch (stage_id) {
      case STAGE_WAITING:
        return "waiting";

      case STAGE_ARM:
        return "arm";

      case STAGE_LAUNCH_IGNITION_SEQUENCE:
        return "launch ignition sequence";

      case STAGE_ENGINE_WORKING:
        return "engine working";

      case STAGE_DATA_GATHERING_FLIGHT:
        return "data gathering flight";

      default:
        return "unknown";
    }
  }

  bool stage_can_return_to_waiting(uint8_t stage_id) {
    return stage_id == STAGE_WAITING || stage_id == STAGE_ARM;
  }

  void enter_no_return_flight() {
    if (no_return_flight_started) {
      return;
    }

    no_return_flight_started = true;
    main_valve_opened_for_flight = true;
    flight_start_time_ms = millis();

    set_stage(STAGE_ENGINE_WORKING);

    Serial.println("NO-RETURN FLIGHT STARTED");
    Serial.println("Main valve opened. Stage locked to flight stages only.");
  }

  void stage_waiting_loop() {
    read_battery_voltage();

    read_valve_connection();

    read_pressure();
    read_GPS();
    read_DS1307();

    read_ADXL375();
    //read_BME680();
    start_BME680_read();
    finish_BME680_read();
    read_LSM6DSOX();

    read_RFM();

    save_to_sd();

    prep_telemetry();
    send_RFM();

    update_debug_display();
  }

  void stage_arm_loop() {
    // Stage 2: ARM
    // If ARM command/state is no longer active, return to waiting.

    if (!avionics_armed) {
      reset_arm_sequence();
      set_stage(STAGE_WAITING);
      return;
    }

    // Stage 2 still reads only needed systems.
    read_battery_voltage();
    read_valve_connection();
    read_pressure();
    read_GPS();
    read_DS1307();

    read_RFM();

    update_arm_sequence();

    save_to_sd();
    prep_telemetry();
    send_RFM();

    debug_laptop();
  }

  void stage_launch_ignition_sequence_loop() {
    if (!ignition_sequence_started) {
      if (!no_return_flight_started) {
        set_stage(STAGE_WAITING);
      }
      return;
    }

    read_pressure();
    read_LSM6DSOX();
    read_ADXL375();
    read_RFM();

    update_main_valve_ignition_hold();

    save_to_sd();
    prep_telemetry();
    send_RFM();
  }

  void stage_engine_working_loop() {
    // Stage 4: engine working / no-return flight stage.
    // No return to waiting, arm, or ignition sequence is allowed after this point.

    read_pressure();

    read_ADXL375();

    //read_BME680();
    start_BME680_read();
    finish_BME680_read();

    read_LSM6DSOX();

    ORACLE_update();

    if ((millis() - flight_start_time_ms) >= MIN_ENGINE_ON_TIME_MS) {
      //stratologger_apogee_prediction = apogee_prediction();
      if (predicted_apogee_m >= desired_apogee_m) {
        VALVE_close(MAIN_VALVE);
        set_stage(STAGE_DATA_GATHERING_FLIGHT);
      }
    }

    if ((millis() - flight_start_time_ms) >= MAX_ENGINE_ON_TIME_MS) {
      VALVE_close(MAIN_VALVE);
      set_stage(STAGE_DATA_GATHERING_FLIGHT);
    }

    
  }

  void stage_data_gathering_flight_loop() {
    read_battery_voltage();

    read_valve_connection();

    read_pressure();

    read_GPS();
    read_DS1307();

    read_ADXL375();

    start_BME680_read();
    finish_BME680_read();

    read_LSM6DSOX();

    ORACLE_update();

    detect_apogee_after_engine_cutoff();
    update_relief_after_apogee();

    read_RFM();

    send_current_location();

    // save_to_sd() is already called in main loop.
  }

// ======================= SETUP / SYSTEM INIT =====================

  void setup_valves() {
    pinMode(MAIN_VALVE.control_pin, OUTPUT);
    pinMode(RELIEF_VALVE.control_pin, OUTPUT);
    pinMode(DUMP_VALVE.control_pin, OUTPUT);

    close_all_valves();
  }

  void setup_valve_connection_check() {
    pinMode(MAIN_VALVE.connection_pin, INPUT);
    pinMode(RELIEF_VALVE.connection_pin, INPUT);
    pinMode(DUMP_VALVE.connection_pin, INPUT);
  }

  void setup_inputs() {
    pinMode(PRESSURE_TRANSDUCER_1_pin, INPUT);
    pinMode(PRESSURE_TRANSDUCER_2_pin, INPUT);
    pinMode(PRESSURE_TRANSDUCER_3_pin, INPUT);
    pinMode(PRESSURE_TRANSDUCER_4_pin, INPUT);
    pinMode(GPS_PPS_pin, INPUT);

    // TODO: enable after battery voltage pin is assigned.
    // pinMode(BATTERY_VOLTAGE_pin, INPUT);
  }

  void setup_adc() {
    adc->adc0->setAveraging(16);
    adc->adc0->setResolution(16);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
    adc->adc0->recalibrate();
    Serial.println("online: ADC0");

    #ifdef ADC_DUAL_ADCS
      adc->adc1->setAveraging(16);
      adc->adc1->setResolution(10);
      adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
      adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
      adc->adc1->recalibrate();
      Serial.println("online: ADC1");
    #endif
  }

  void setup_i2c() {
    Wire.setSDA(I2C_MAIN_SDA_pin);
    Wire.setSCL(I2C_MAIN_SCL_pin);
    Wire.begin();
    Serial.println("online: main I2C Wire");

    Wire1.setSDA(ADXL375_SDA_pin);
    Wire1.setSCL(ADXL375_SCL_pin);
    Wire1.begin();
    Serial.println("online: ADXL375 I2C Wire1");

    // TODO: display/debug screen setup will go here after display pins/module are known.
  }

  void setup_radio() {
    pinMode(RFM9X_PWR_pin, OUTPUT);
    digitalWrite(RFM9X_PWR_pin, HIGH);

    pinMode(RFM9X_RST_pin, OUTPUT);
    digitalWrite(RFM9X_RST_pin, HIGH);
    delay(10);
    digitalWrite(RFM9X_RST_pin, LOW);
    delay(10);
    digitalWrite(RFM9X_RST_pin, HIGH);
    delay(10);

    SPI.begin();

    if (!rf95.init()) {
      Serial.println("offline: RFM9X");
      RFM9X_FAIL = true;
      return;
    }

    if (!rf95.setFrequency(CONTROL_FREQ)) {
      Serial.println("offline: RFM9X frequency set");
      RFM9X_FAIL = true;
      return;
    }

    rf95.setTxPower(RFM9X_TX_POWER_DBM, false);
    Serial.println("online: RFM9X");
  }

  void print_setup_summary() {
    Serial.println();
    Serial.println("===== SETUP SUMMARY =====");
    Serial.print("RFM9X_FAIL    = "); Serial.println(RFM9X_FAIL);
    Serial.print("SD_FAIL       = "); Serial.println(SD_FAIL);
    Serial.print("LSM6DSOX_FAIL = "); Serial.println(LSM6DSOX_FAIL);
    Serial.print("ADXL375_FAIL  = "); Serial.println(ADXL375_FAIL);
    Serial.print("BME680_FAIL   = "); Serial.println(BME680_FAIL);
    Serial.print("GPS_FAIL      = "); Serial.println(GPS_FAIL);
    Serial.print("DS1307_FAIL   = "); Serial.println(DS1307_FAIL);
    Serial.print("DISPLAY_FAIL  = "); Serial.println(DISPLAY_FAIL);
    Serial.println("=========================");
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
// ============================== ARM ==============================
  void start_arm_sequence() {
    arm_sequence_active = true;
    arm_sequence_started_once = true;
    arm_sequence_start_time_ms = millis();
    arm_countdown = -20;

    ignition_sequence_started = false;

    main_valve_can_close_by_radio = false;
    main_valve_ignition_hold_active = false;
    main_valve_ignition_start_time_ms = 0;

    VALVE_close(RELIEF_VALVE);
    VALVE_close(DUMP_VALVE);
    VALVE_close(MAIN_VALVE);

    Serial.println("ARM sequence started");
    Serial.println("Relief, dump, and main valves closed");
  }

  void update_arm_sequence() {
    if (!arm_sequence_active) {
      start_arm_sequence();
    }

    uint32_t elapsed_ms = millis() - arm_sequence_start_time_ms;

    if (elapsed_ms / 1000 > (uint32_t)(arm_countdown + 20)) {
      arm_countdown++;

      Serial.print("ARM countdown = ");
      Serial.println(arm_countdown);
    }

    if (elapsed_ms >= ARM_SEQUENCE_DURATION_MS) {
      arm_sequence_active = false;

      ignition_sequence_started = true;
      main_valve_ignition_hold_active = true;
      main_valve_ignition_start_time_ms = millis();

      set_stage(STAGE_LAUNCH_IGNITION_SEQUENCE);

      Serial.println("ARM sequence complete");
      Serial.println("Moving to STAGE_LAUNCH_IGNITION_SEQUENCE");
    }
  }

  void reset_arm_sequence() {
    arm_sequence_active = false;
    arm_sequence_started_once = false;
    arm_sequence_start_time_ms = 0;
    arm_countdown = -20;
  }

  void print_arm_sequence_status() {
    Serial.println();
    Serial.println("ARM SEQUENCE STATUS");

    Serial.print("avionics_armed = ");
    Serial.println(avionics_armed);

    Serial.print("arm_sequence_active = ");
    Serial.println(arm_sequence_active);

    Serial.print("arm_sequence_started_once = ");
    Serial.println(arm_sequence_started_once);

    Serial.print("arm_countdown = ");
    Serial.println(arm_countdown);

    if (arm_sequence_active) {
      Serial.print("elapsed_ms = ");
      Serial.println(millis() - arm_sequence_start_time_ms);
    }

    Serial.println();
  }

// ==================== INPUT / SENSOR READ SECTION ================

  float read_voltage(uint8_t pin) {
    int value = adc->adc0->analogRead(pin);
    float Vout = value * 3.3f / adc->adc0->getMaxValue();
    return Vout;
  }

  void read_battery_voltage() {
    // TODO: needs battery voltage divider pin and divider ratio.
    if (BATTERY_VOLTAGE_pin < 0) return;

    float voltage = read_voltage((uint8_t)BATTERY_VOLTAGE_pin);
    battery_voltage_output = voltage; // TODO: multiply by divider ratio.
  }

  void read_pressure() {
    if (micros() - Pressure_last_time > Pres_rate) {
      float voltage = 0.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_1_pin); // 1 - pin 20 - N2O/fuel tank
      pressure_1_output = n2o_voltage_to_pressure_psi(voltage);

      voltage = read_voltage(PRESSURE_TRANSDUCER_2_pin); // 2 - pin 40 - extra / currently unused
      pressure_2_output = PRESSURE_2_SLOPE * voltage + PRESSURE_2_OFFSET;

      voltage = read_voltage(PRESSURE_TRANSDUCER_3_pin); // 3 - pin 15 - combustion chamber
      pressure_3_output = combust_voltage_to_pressure_psi(voltage);

      voltage = read_voltage(PRESSURE_TRANSDUCER_4_pin); // 4 - pin 27 - CO2 pressure level
      pressure_4_output = co2_voltage_to_pressure_psi(voltage);

      Pressure_last_time = micros();
    }
  }

  void init_LSM6DSOX() {
    if (!lsm6dsox.begin_I2C(0x6A, &Wire)) {
      Serial.println("offline: LSM6DSOX");
      LSM6DSOX_FAIL = true;
      return;
    }

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6dsox.setAccelDataRate(LSM6DS_RATE_208_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_208_HZ);

    Serial.println("online: LSM6DSOX");
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

  void init_ADXL375() {
    if (!adxl375.begin(0x53)) {
      Serial.println("offline: ADXL375");
      ADXL375_FAIL = true;
      return;
    }

    Serial.println("online: ADXL375");
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
  // old BME
    //   void init_BME680() {
    //     if (!BME680.begin(0x77, &Wire) && !BME680.begin(0x76, &Wire)) {
    //       Serial.println("offline: BME680");
    //       BME680_FAIL = true;
    //       return;
    //     }

    //     BME680.setTemperatureOversampling(BME680_OS_8X);
    //     BME680.setHumidityOversampling(BME680_OS_2X);
    //     BME680.setPressureOversampling(BME680_OS_4X);
    //     BME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    //     BME680.setGasHeater(320, 150);

    //     Serial.println("online: BME680");
    // }

    // void read_BME680() {
    //   if (BME680_FAIL) return;

    //   if (micros() - BME680_last_time > BME680_rate) {
    //     if (BME680.performReading()) {
    //       BME680_temp = BME680.temperature;
    //       BME680_humidity = BME680.humidity;
    //       BME680_pressure_hpa = BME680.pressure / 100.0f;
    //       BME680_gas_kohm = BME680.gas_resistance / 1000.0f;
    //       BME680_altitude_m = 44330.0f * (1.0f - pow(BME680_pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));
    //     }

    //     BME680_last_time = micros();
    //   }
    // }

  void init_BME680() {
    if (!BME680.begin(0x77, &Wire) && !BME680.begin(0x76, &Wire)) {
      Serial.println("offline: BME680");
      BME680_FAIL = true;
      return;
    }

    BME680.setTemperatureOversampling(BME680_OS_4X);
    //BME680.setHumidityOversampling(BME680_OS_NONE);
    BME680.setPressureOversampling(BME680_OS_2X);
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
    BME680_new_data = false;
    if (bme_busy && BME680.remainingReadingMillis() == 0) {
      if (BME680.endReading()) {
        BME680_temp = BME680.temperature;
        BME680_pressure_hpa = BME680.pressure / 100.0f;
        apply_BME680_pressure_deadband();
        BME680_new_data = true;
      }
      bme_busy = false;
    }
  }

  void calibrate_BME680_pressure_threshold() {
    if (BME680_FAIL) {
      Serial.println("BME680 calibration failed: sensor offline");
      return;
    }

    Serial.println();
    Serial.println("BME680 pressure threshold calibration started");

    uint32_t calibration_start_ms = millis();
    uint32_t calibration_duration_ms = (uint32_t)(BME680_calibration_duration_s * 1000.0f);

    bool first_sample = true;
    float min_pressure_hpa = 0.0f;
    float max_pressure_hpa = 0.0f;

    while (millis() - calibration_start_ms < calibration_duration_ms) {
      if (BME680.performReading()) {
        float current_pressure_hpa = BME680.pressure / 100.0f;

        if (first_sample) {
          min_pressure_hpa = current_pressure_hpa;
          max_pressure_hpa = current_pressure_hpa;
          first_sample = false;
        }
        else {
          if (current_pressure_hpa < min_pressure_hpa) {
            min_pressure_hpa = current_pressure_hpa;
          }

          if (current_pressure_hpa > max_pressure_hpa) {
            max_pressure_hpa = current_pressure_hpa;
          }
        }
      }
    }

    if (first_sample) {
      Serial.println("BME680 calibration failed: no valid samples");
      return;
    }

    BME680_pressure_threshold_hpa = max_pressure_hpa - min_pressure_hpa;

    BME680_filtered_pressure_hpa = BME680_pressure_hpa;
    BME680_filter_started = true;

    Serial.println("BME680 pressure threshold calibration complete");

    Serial.print("min pressure hPa = ");
    Serial.println(min_pressure_hpa, 4);

    Serial.print("max pressure hPa = ");
    Serial.println(max_pressure_hpa, 4);

    Serial.print("pressure threshold hPa = ");
    Serial.println(BME680_pressure_threshold_hpa, 4);

    Serial.print("used range = +/- ");
    Serial.println(BME680_pressure_threshold_hpa * 0.5f, 4);

    Serial.println();
  }

  void apply_BME680_pressure_deadband() {
    if (!BME680_filter_started) {
      BME680_filtered_pressure_hpa = BME680_pressure_hpa;
      BME680_filter_started = true;
      return;
    }

    float half_threshold_hpa = BME680_pressure_threshold_hpa * 0.4f;
    float pressure_difference_hpa = fabsf(BME680_pressure_hpa - BME680_filtered_pressure_hpa);

    if (pressure_difference_hpa > half_threshold_hpa) {
      BME680_filtered_pressure_hpa = BME680_pressure_hpa;
    }

    // From here, the normal altitude variable uses filtered pressure.
    BME680_pressure_hpa = BME680_filtered_pressure_hpa;

    BME680_altitude_m = 44330.0f * (1.0f - pow(BME680_pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));
  }

  void init_GPS() {
    GPS_PORT.begin(9600);

    pinMode(GPS_PPS_pin, INPUT);

    GPS_FAIL = false;

    Serial.println("online: GPS SoftwareSerial NMEA");
    Serial.println("GPS RX software pin = 8");
    Serial.println("GPS TX software pin = 7");
    Serial.println("GPS PPS pin = 34");
  }

  void read_GPS() {
    if (GPS_FAIL) return;

    while (GPS_PORT.available()) {
      gps.encode(GPS_PORT.read());
    }

    if (gps.location.isValid()) {
      gps_lat = gps.location.lat();
      gps_lon = gps.location.lng();
      gps_fix_type = 1;
    } else {
      gps_fix_type = 0;
    }

    if (gps.altitude.isValid()) {
      gps_alt_m = gps.altitude.meters();
    }

    if (gps.satellites.isValid()) {
      gps_siv = gps.satellites.value();
    }

    gps_pps_state = digitalRead(GPS_PPS_pin);
  }

  void init_DS1307() {
    Wire.beginTransmission(0x68);
    byte error = Wire.endTransmission();

    if (error != 0) {
      Serial.println("offline: DS1307 RTC");
      DS1307_FAIL = true;
      return;
    }

    rtc.refresh();

    DS1307_FAIL = false;

    Serial.println("online: DS1307 RTC");
  }

  void read_DS1307() {
    if (DS1307_FAIL) return;

    if (micros() - DS1307_last_time <= DS1307_rate) return;

    rtc.refresh();

    rtc_year = rtc.year();
    rtc_month = rtc.month();
    rtc_day = rtc.day();
    rtc_hour = rtc.hour();
    rtc_minute = rtc.minute();
    rtc_second = rtc.second();

    DS1307_last_time = micros();
  }

// ================= PCB OUTPUT / CONTROL SECTION ==================

  void VALVE_open(Valve& valve) {
    digitalWrite(valve.control_pin, valve.active_low ? LOW : HIGH);
    valve.state = true;

    Serial.print(valve.name);
    Serial.println(" valve: OPEN");
  }

  void VALVE_close(Valve& valve) {
    digitalWrite(valve.control_pin, valve.active_low ? HIGH : LOW);
    valve.state = false;

    Serial.print(valve.name);
    Serial.println(" valve: CLOSED");
  }

  void close_all_valves() {
    VALVE_close(MAIN_VALVE);
    VALVE_close(RELIEF_VALVE);
    VALVE_close(DUMP_VALVE);
  }

  void update_main_valve_ignition_hold() {
    if (!main_valve_ignition_hold_active) {
      countdown = -20;
      return;
    }

    uint32_t elapsed_ms = millis() - main_valve_ignition_start_time_ms;

    if (elapsed_ms >= MAIN_VALVE_OPEN_AFTER_START_MS - 500 && !main_valve_opened_for_flight) {
      calibrate_all();
      set_to_start_location();
      return;
    }


    if (elapsed_ms >= MAIN_VALVE_OPEN_AFTER_START_MS && !main_valve_opened_for_flight) {
      // start of flight
      enter_no_return_flight();
      set_to_start_location();
      oxidizer_mass_integration_enabled = true;
      VALVE_open(MAIN_VALVE);
      return;
    }

    if (elapsed_ms / 1000 > (uint32_t)(countdown + 20)) {
      countdown++;
      Serial.println(countdown);
    }
  }

  void update_debug_display() {
    // TODO: future display/debug screen output.
  }

  void update_laptop_debug() {
    debug_serial();
    streams();
  }

// ==================== VALVE CONNECTION CHECK SECTION =============

  Valve* get_valve_by_id(uint8_t valve_id) {
    switch (valve_id) {
      case 1:
        return &MAIN_VALVE;

      case 2:
        return &RELIEF_VALVE;

      case 3:
        return &DUMP_VALVE;

      default:
        return nullptr;
    }
  }

  void read_valve_connection() {
    valve_connection_ckeck_all();
  }

  void valve_connection_ckeck_all() {
    valve_connection_ckeck(1);
    valve_connection_ckeck(2);
    valve_connection_ckeck(3);
  }

  bool valve_connection_ckeck(uint8_t valve_id) {
    Valve* valve = get_valve_by_id(valve_id);

    if (valve == nullptr) {
      return false;
    }

    valve->connection_voltage = valve_connection_voltage_read(valve_id);
    valve->connection_ok = valve->connection_voltage >= VALVE_CONNECTION_MIN_VOLTAGE;

    return valve->connection_ok;
  }

  bool valve_connection_check(uint8_t valve_id) {
    return valve_connection_ckeck(valve_id);
  }

  float valve_connection_voltage_read(uint8_t valve_id) {
    Valve* valve = get_valve_by_id(valve_id);

    if (valve == nullptr) {
      return 0.0f;
    }

    return read_voltage(valve->connection_pin);
  }

  void print_valve_connection_status() {
    Serial.println("VALVE CONNECTION STATUS");

    Serial.print("Main connection V   = ");
    Serial.print(MAIN_VALVE.connection_voltage, 3);
    Serial.print(" | OK = ");
    Serial.println(MAIN_VALVE.connection_ok);

    Serial.print("Relief connection V = ");
    Serial.print(RELIEF_VALVE.connection_voltage, 3);
    Serial.print(" | OK = ");
    Serial.println(RELIEF_VALVE.connection_ok);

    Serial.print("Dump connection V   = ");
    Serial.print(DUMP_VALVE.connection_voltage, 3);
    Serial.print(" | OK = ");
    Serial.println(DUMP_VALVE.connection_ok);
  }

// =============== RADIO / TELEMETRY / MESSAGE SECTION =============

  uint32_t compute_state_array() {
    uint32_t state_array = 0;

    // bits 0-2: valve states
    if (MAIN_VALVE.state)   state_array |= 0x00000001;
    if (RELIEF_VALVE.state) state_array |= 0x00000002;
    if (DUMP_VALVE.state)   state_array |= 0x00000004;

    // old ignition-reference states
    if (avionics_armed)                  state_array |= 0x00000008;
    if (ignition_sequence_started)       state_array |= 0x00000010;
    if (main_valve_ignition_hold_active) state_array |= 0x00000020;

    // fail flags
    if (RFM9X_FAIL)    state_array |= 0x00000100;
    if (SD_FAIL)       state_array |= 0x00000200;
    if (LSM6DSOX_FAIL) state_array |= 0x00000400;
    if (ADXL375_FAIL)  state_array |= 0x00000800;
    if (BME680_FAIL)   state_array |= 0x00001000;
    if (GPS_FAIL)      state_array |= 0x00002000;
    if (DS1307_FAIL)   state_array |= 0x00004000;
    if (DISPLAY_FAIL)  state_array |= 0x00008000;

    return state_array;
  }

  void prep_telemetry_pressures() {
    // Msg 0x02-0x03 structure: {5,5,5,5,5,5,4}
    in_floats[0] = pressure_1_output;
    in_floats[1] = pressure_2_output;
    in_floats[2] = pressure_3_output;
    in_floats[3] = pressure_4_output;
    in_floats[4] = battery_voltage_output;
    in_floats[5] = BME680_altitude_m;
    in_int32s[6] = commands_received;

    message_assembler(0x02, 0x03);
  }

  void prep_telemetry_states() {
    // Msg 0x02-0x06 structure: {4}
    in_int32s[0] = compute_state_array();
    message_assembler(0x02, 0x06);
  }

  void prep_telemetry() {
    // Called by whichever future phase needs telemetry.
    // It does not read sensors by itself.
    if (telemetry_select_index == 0) {
      prep_telemetry_pressures();
      telemetry_select_index = 1;
    } else {
      prep_telemetry_states();
      telemetry_select_index = 0;
    }
  }

  void read_RFM() {
    if (!RFM9X_FAIL && rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {
        message_parser(buf);

        if (recv_ready) {
          bool command_accepted = execute_radio_message(buf[0], buf[1]);

          // Ground station expects ACK1 from AV1 for accepted class 0x01 commands.
          if (command_accepted && buf[0] == 0x01) {
            send_ACK_packet(1);
          }

          print_local_status();
          recv_ready = 0;
        }
      }
    }
  }

  void send_ACK_packet(uint8_t computer_id) {
    if (RFM9X_FAIL) return;

    uint8_t ack_msg[4] = {'A', 'C', 'K', (uint8_t)('0' + computer_id)};
    rf95.send(ack_msg, sizeof(ack_msg));
    rf95.waitPacketSent();
  }

  void send_RFM() {
    if (!RFM9X_FAIL && TX_enabled && msg_ready && (micros() - RFM9x_last_time > RFM9x_rate)) {
      rf95.send(message_send_buf, message_send_len);
      rf95.waitPacketSent();
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
        if (msg_id_index < 7) msg_structure = msg_class_02[msg_id_index];
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
        if ((msg_id - 1) < 7) msg_structure = msg_class_02[msg_id - 1];
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
        case 0:
          break;

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
// ===================== COMMAND EXECUTION SECTION =================
  bool command_flags_confirmed() {
    return out_flag[0] == 0xFF &&
          out_flag[1] == 0xFF &&
          out_flag[2] == 0xFF;
  }

  bool execute_radio_message(uint8_t msg_class, uint8_t msg_id) {
    if (msg_class != 0x01) {
      clear_command_flags();
      return false;
    }

    if (!command_flags_confirmed()) {
      clear_command_flags();
      return false;
    }

    bool command_handled = execute_command_id(msg_id, COMMAND_FROM_RADIO);
    clear_command_flags();
    return command_handled;
  }

  bool execute_debug_command(uint8_t msg_id) {
    bool command_handled = execute_command_id(msg_id, COMMAND_FROM_DEBUG);
    clear_command_flags();
    return command_handled;
  }

  bool execute_command_id(uint8_t msg_id, CommandSource source) {
    bool command_handled = false;

    switch (msg_id) {
      case 0x01: // ignition abort
        commands_received++;

        avionics_armed = false;
        ignition_sequence_started = false;

        main_valve_can_close_by_radio = true;
        main_valve_ignition_hold_active = false;
        main_valve_ignition_start_time_ms = 0;

        reset_arm_sequence();

        close_all_valves();

        if (stage_can_return_to_waiting(stage)) {
          set_stage(STAGE_WAITING);
        }

        Serial.print("ABORT from ");
        Serial.println(command_source_name(source));

        command_handled = true;
        break;

      case 0x02: // avionics arm
        commands_received++;

        avionics_armed = true;
        ignition_sequence_started = false;

        reset_arm_sequence();
        set_stage(STAGE_ARM);

        Serial.print("ARM from ");
        Serial.println(command_source_name(source));

        command_handled = true;
        break;

      case 0x03: // ignition sequence start
        if (avionics_armed) {
          commands_received++;

          ignition_sequence_started = true;
          main_valve_can_close_by_radio = false;
          main_valve_ignition_hold_active = true;
          main_valve_ignition_start_time_ms = millis();

          VALVE_close(RELIEF_VALVE);
          VALVE_close(DUMP_VALVE);

          set_stage(STAGE_LAUNCH_IGNITION_SEQUENCE);

          command_handled = true;

          Serial.print("IGNITION START ACCEPTED from ");
          Serial.println(command_source_name(source));
        } else {
          command_handled = true;
          Serial.println("IGNITION START IGNORED: AV1 not armed");
        }
        break;

      case 0x04: // open main valve
        commands_received++;
        VALVE_open(MAIN_VALVE);
        command_handled = true;
        break;

      case 0x05: // close main valve
        commands_received++;

        if (source == COMMAND_FROM_RADIO && !main_valve_can_close_by_radio) {
          Serial.println("MAIN valve close by radio ignored: ignition hold active");
        } else {
          VALVE_close(MAIN_VALVE);

          if (no_return_flight_started) {
            set_stage(STAGE_DATA_GATHERING_FLIGHT);
            Serial.println("MAIN valve closed after no-return: data gathering stage");
          } else {
            main_valve_can_close_by_radio = true;
            main_valve_ignition_hold_active = false;
            main_valve_ignition_start_time_ms = 0;
            avionics_armed = false;
            ignition_sequence_started = false;

            Serial.println("MAIN valve closed: AV1 returned to unarmed state");
          }
        }

        command_handled = true;
        break;

      case 0x08: // open relief valve
        commands_received++;
        VALVE_open(RELIEF_VALVE);
        command_handled = true;
        break;

      case 0x09: // close relief valve
        commands_received++;
        VALVE_close(RELIEF_VALVE);
        command_handled = true;
        break;

      case 0x0C: // enable TX
        commands_received++;
        TX_enabled = true;
        command_handled = true;
        break;

      case 0x0D: // disable TX
        commands_received++;
        TX_enabled = false;
        command_handled = true;
        break;

      case 0x13: // set AV1 radio frequency
        commands_received++;
        CONTROL_FREQ = out_floats[3];
        rf95.setFrequency(CONTROL_FREQ);
        command_handled = true;
        break;

      default:
        Serial.print("Unknown command ID: 0x");
        Serial.println(msg_id, HEX);
        break;
    }

    return command_handled;
  }

  const char* command_source_name(CommandSource source) {
    switch (source) {
      case COMMAND_FROM_RADIO:
        return "radio";

      case COMMAND_FROM_DEBUG:
        return "debug";

      default:
        return "unknown";
    }
  }

  void clear_command_flags() {
    out_flag[0] = 0;
    out_flag[1] = 0;
    out_flag[2] = 0;
  }

  // Compatibility wrapper.
  // New code should use execute_radio_message() or execute_debug_command().
  bool perform_command(uint8_t msg_class, uint8_t msg_id, bool from_radio) {
    if (from_radio) {
      return execute_radio_message(msg_class, msg_id);
    }

    return execute_debug_command(msg_id);
  }

// ============================ SD LOG SECTION =====================


  void init_SD() {
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("offline: SD");
      SD_FAIL = true;
      return;
    }

    create_new_black_box_file();

    Serial.print("online: SD, logging to ");
    Serial.println(black_box_filename);
  }

  void create_new_black_box_file() {
    for (int file_number = 0; file_number <= 999; file_number++) {
      sprintf(black_box_filename, "BLACK%03d.CSV", file_number);

      if (!SD.exists(black_box_filename)) {
        write_sd_header();
        return;
      }
    }

    Serial.println("SD ERROR: no free BLACK###.CSV filename");
    SD_FAIL = true;
  }

  void write_sd_header() {
    Black_Box = SD.open(black_box_filename, FILE_WRITE);
      if (Black_Box) {
        Black_Box.println(
          "time_ms,"
          "stage,"
          "commands_received,"
          "main_state,"
          "relief_state,"
          "dump_state,"
          "main_connection_v,"
          "main_connection_ok,"
          "relief_connection_v,"
          "relief_connection_ok,"
          "dump_connection_v,"
          "dump_connection_ok,"
          "pressure_1,"
          "pressure_2,"
          "pressure_3,"
          "pressure_4,"
          "battery_voltage,"
          "lsm_ax,"
          "lsm_ay,"
          "lsm_az,"
          "lsm_gx,"
          "lsm_gy,"
          "lsm_gz,"
          "lsm_temp,"
          "adxl_ax,"
          "adxl_ay,"
          "adxl_az,"
          "BME680_temp,"
          "BME680_pressure_hpa,"
          "BME680_altitude_m,"
          "gps_lat,"
          "gps_lon,"
          "gps_alt_m,"
          "gps_siv,"
          "gps_fix_type,"
          "gps_pps,"
          "rtc_year,"
          "rtc_month,"
          "rtc_day,"
          "rtc_hour,"
          "rtc_minute,"
          "rtc_second,"
          "rocket_tilt_angle_deg,"
          "kalman_vertical_speed_mps,"
          "raw_vertical_speed_mps,"
          "rocket_speed_mps,"
          "rocket_speed_raw_mps,"
          "rocket_speed_valid,"
          "state_array"
        );

      Black_Box.close();
    }
  }

  void save_to_sd() {
    if (SD_FAIL) {
      return;
    }

    if (micros() - SD_last_time > SD_rate) {
      Black_Box = SD.open(black_box_filename, FILE_WRITE);

      if (Black_Box) {
        Black_Box.print(millis()); Black_Box.print(',');
        Black_Box.print(stage); Black_Box.print(',');
        Black_Box.print(commands_received); Black_Box.print(',');

        Black_Box.print(MAIN_VALVE.state); Black_Box.print(',');
        Black_Box.print(RELIEF_VALVE.state); Black_Box.print(',');
        Black_Box.print(DUMP_VALVE.state); Black_Box.print(',');

        Black_Box.print(MAIN_VALVE.connection_voltage, 3); Black_Box.print(',');
        Black_Box.print(MAIN_VALVE.connection_ok); Black_Box.print(',');
        Black_Box.print(RELIEF_VALVE.connection_voltage, 3); Black_Box.print(',');
        Black_Box.print(RELIEF_VALVE.connection_ok); Black_Box.print(',');
        Black_Box.print(DUMP_VALVE.connection_voltage, 3); Black_Box.print(',');
        Black_Box.print(DUMP_VALVE.connection_ok); Black_Box.print(',');

        Black_Box.print(pressure_1_output, 3); Black_Box.print(',');
        Black_Box.print(pressure_2_output, 3); Black_Box.print(',');
        Black_Box.print(pressure_3_output, 3); Black_Box.print(',');
        Black_Box.print(pressure_4_output, 3); Black_Box.print(',');
        Black_Box.print(battery_voltage_output, 3); Black_Box.print(',');

        Black_Box.print(lsm_ax, 3); Black_Box.print(',');
        Black_Box.print(lsm_ay, 3); Black_Box.print(',');
        Black_Box.print(lsm_az, 3); Black_Box.print(',');
        Black_Box.print(lsm_gx, 3); Black_Box.print(',');
        Black_Box.print(lsm_gy, 3); Black_Box.print(',');
        Black_Box.print(lsm_gz, 3); Black_Box.print(',');
        Black_Box.print(lsm_temp, 3); Black_Box.print(',');

        Black_Box.print(adxl_ax, 3); Black_Box.print(',');
        Black_Box.print(adxl_ay, 3); Black_Box.print(',');
        Black_Box.print(adxl_az, 3); Black_Box.print(',');

        Black_Box.print(BME680_temp, 3); Black_Box.print(',');
        Black_Box.print(BME680_pressure_hpa, 3); Black_Box.print(',');
        Black_Box.print(BME680_altitude_m, 3); Black_Box.print(',');

        Black_Box.print(gps_lat, 7); Black_Box.print(',');
        Black_Box.print(gps_lon, 7); Black_Box.print(',');
        Black_Box.print(gps_alt_m, 3); Black_Box.print(',');
        Black_Box.print(gps_siv); Black_Box.print(',');
        Black_Box.print(gps_fix_type); Black_Box.print(',');
        Black_Box.print(gps_pps_state); Black_Box.print(',');

        Black_Box.print(rtc_year); Black_Box.print(',');
        Black_Box.print(rtc_month); Black_Box.print(',');
        Black_Box.print(rtc_day); Black_Box.print(',');
        Black_Box.print(rtc_hour); Black_Box.print(',');
        Black_Box.print(rtc_minute); Black_Box.print(',');
        Black_Box.print(rtc_second); Black_Box.print(',');

        Black_Box.print(rocket_tilt_angle_deg, 3); Black_Box.print(',');
        Black_Box.print(kalman_vertical_speed_mps, 3); Black_Box.print(',');
        Black_Box.print(raw_vertical_speed_mps, 3); Black_Box.print(',');
        Black_Box.print(rocket_speed_mps, 3); Black_Box.print(',');
        Black_Box.print(rocket_speed_raw_mps, 3); Black_Box.print(',');
        Black_Box.print(rocket_speed_valid); Black_Box.print(',');

        Black_Box.println(compute_state_array());

        Black_Box.close();
      }

      SD_last_time = micros();
    }
  }

// MISSLE KNOWS WHERE IT IS & MISSLE KNOWS WHERE IT WILL BE (ORACLE) 
  void set_to_start_location() {

    read_LSM6DSOX();
    LSM_to_PCB_swithch_over();

    gravity_anchor();

    if (GPS_makes_sense) {
      global_location = {0.0f, 0.0f, gps_alt_m};
    }
    else if (BME_makes_sense) {
      global_location = {0.0f, 0.0f, BME680_altitude_m - BME_alt_offset};
    }
    else {
      global_location = {0.0f, 0.0f, manual_zero_alt};
    }

    PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};
    previous_PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};

    oracle_last_update_us = micros();
    oracle_delta_time_s = 0.0f;
  }

  void ORACLE_update() { // will determine location and vel vectors for next simulation
    LSM_to_PCB_swithch_over(); // accelem orientation to rocket orientation
    //ADXL_to_PCB_swithch_over();

    local_PCB_direction_determination();

    current_rocket_mass();

    calculate_rocket_tilt_from_orientation();

    update_altitude_kalman_filter();
    calculate_rocket_speed_from_vertical_speed();

    apogee_prediction();
  }
 
  void LSM_to_PCB_swithch_over(){ // accelerometers orientation to rocket orientation
    float new_PCB_acc_x = -lsm_ay;
    float new_PCB_acc_y =  lsm_az;
    float new_PCB_acc_z = -lsm_ax;

    if (!PCB_acc_filter_started) {
      PCB_local_acc_vector.x = new_PCB_acc_x;
      PCB_local_acc_vector.y = new_PCB_acc_y;
      PCB_local_acc_vector.z = new_PCB_acc_z;

      last_PCB_acc_x = new_PCB_acc_x;
      last_PCB_acc_y = new_PCB_acc_y;
      last_PCB_acc_z = new_PCB_acc_z;

      PCB_acc_filter_started = true;
    }
    else {
      if (fabs(new_PCB_acc_x - last_PCB_acc_x) >= PCB_acc_filter_threshold) {
        PCB_local_acc_vector.x = new_PCB_acc_x;
        last_PCB_acc_x = new_PCB_acc_x;
      }

      if (fabs(new_PCB_acc_y - last_PCB_acc_y) >= PCB_acc_filter_threshold) {
        PCB_local_acc_vector.y = new_PCB_acc_y;
        last_PCB_acc_y = new_PCB_acc_y;
      }

      if (fabs(new_PCB_acc_z - last_PCB_acc_z) >= PCB_acc_filter_threshold) {
        PCB_local_acc_vector.z = new_PCB_acc_z;
        last_PCB_acc_z = new_PCB_acc_z;
      }
    }

    // Gyro calibration
    Calibrated_Gyro_x = lsm_gx - gyro_x_offset;
    Calibrated_Gyro_y = lsm_gy - gyro_y_offset;
    Calibrated_Gyro_z = lsm_gz - gyro_z_offset;

    // // Gyro orientation to PCB orientation initial
    // PCB_gx = Calibrated_Gyro_y;
    // PCB_gy = -Calibrated_Gyro_z;
    // PCB_gz = Calibrated_Gyro_x;

    // Gyro orientation to PCB orientation // working one

      PCB_gx = -Calibrated_Gyro_y;
      PCB_gy = Calibrated_Gyro_z;
      PCB_gz = -Calibrated_Gyro_x;


  }; 

  void ADXL_to_PCB_swithch_over(){
    // empty for now
    return;
  };

  void current_rocket_mass() {
    float dt_s = oracle_delta_time_s;

    if (dt_s <= 0.0f || dt_s > 0.5f) {
      rocket_mass = no_fuel_rocket_mass + fuel_mass;
      return;
    }

    if (use_mass_flow_integration_for_rocket_mass) {
      if (oxidizer_mass_integration_enabled) {
        float P_tank_pa = pressure_1_output * PSI_TO_PA;
        float Pc_pa     = pressure_3_output * PSI_TO_PA;

        float deltaP_pa = P_tank_pa - Pc_pa;

        if (deltaP_pa > 0.0f) {
          oxidizer_mass_flow_kg_s =
            oxidizer_flow_multiplier *
            oxidizer_flow_coefficient_area_m2 *
            sqrtf(2.0f * nitrous_density_kg_m3 * deltaP_pa);

          oxidizer_mass_used_kg += oxidizer_mass_flow_kg_s * dt_s;

          if (oxidizer_mass_used_kg > initial_fuel_mass) {
            oxidizer_mass_used_kg = initial_fuel_mass;
          }

          fuel_mass = initial_fuel_mass - oxidizer_mass_used_kg;
        }
        else {
          oxidizer_mass_flow_kg_s = 0.0f;
        }
      }
      else {
        oxidizer_mass_flow_kg_s = 0.0f;
      }
    }
    else {
      // TODO:
      // fuel_mass = mass_as_function_of_flight_time();
      // empty for now
    }

    if (fuel_mass < 0.0f) {
      fuel_mass = 0.0f;
    }

    rocket_mass = no_fuel_rocket_mass + fuel_mass;
  }

  void local_PCB_direction_determination() {
    // Uses LSM gyro converted to PCB local gyro:
    // PCB_gx, PCB_gy, PCB_gz
    //
    // Gyro convention:
    // +PCB_gx = right-hand rotation around PCB local X
    // +PCB_gy = right-hand rotation around PCB local Y
    // +PCB_gz = right-hand rotation around PCB local Z
    //
    // Quaternion convention:
    // PCB_orientation_quat rotates PCB local vectors into global vectors.
    // Since gyro is measured in local/body frame:
    // q_new = q_old * dq_local

    if (!gravity_anchor_valid || !PCB_orientation_quat_valid) {
      return;
    }

    uint32_t now_us = micros();

    if (PCB_orientation_last_update_us == 0) {
      PCB_orientation_last_update_us = now_us;
      return;
    }

    uint32_t dt_us = now_us - PCB_orientation_last_update_us;
    PCB_orientation_last_update_us = now_us;

    PCB_orientation_dt_s = dt_us / 1000000.0f;
    oracle_delta_time_s = PCB_orientation_dt_s;

    if (PCB_orientation_dt_s <= 0.0f || PCB_orientation_dt_s > 0.25f) {
      return;
    }

    PCB_local_gyro_vector.x = PCB_gx;
    PCB_local_gyro_vector.y = PCB_gy;
    PCB_local_gyro_vector.z = PCB_gz;

    // Simple gyro deadband to ignore small gyro noise.
    if (fabsf(PCB_local_gyro_vector.x) < PCB_gyro_deadband_rad_s) PCB_local_gyro_vector.x = 0.0f;
    if (fabsf(PCB_local_gyro_vector.y) < PCB_gyro_deadband_rad_s) PCB_local_gyro_vector.y = 0.0f;
    if (fabsf(PCB_local_gyro_vector.z) < PCB_gyro_deadband_rad_s) PCB_local_gyro_vector.z = 0.0f;

    PCB_gyro_mag = vector_magnitude(PCB_local_gyro_vector);

    local_direction_change.x = PCB_local_gyro_vector.x * PCB_orientation_dt_s;
    local_direction_change.y = PCB_local_gyro_vector.y * PCB_orientation_dt_s;
    local_direction_change.z = PCB_local_gyro_vector.z * PCB_orientation_dt_s;

    Quaternionf dq_local = quaternion_from_rotation_vector(local_direction_change);

    // Body/local-frame gyro update.
    PCB_orientation_quat = quaternion_multiply(PCB_orientation_quat, dq_local);
    PCB_orientation_quat = quaternion_normalize(PCB_orientation_quat);

    // Update 3x3 matrix from quaternion.
    quaternion_to_local_direction_matrix();

    // Debug global gyro vector only; not used for integration.
    PCB_global_gyro_vector = local_to_global_vector(PCB_local_gyro_vector);

    if (gyro_orientation_debug_enabled) {
      print_gyro_orientation_status();
    }
  }

  void calculate_rocket_tilt_from_orientation() {
    // Fastest tilt calculation.
    // Assumption: rocket longitudinal axis = PCB local +Z.
    // Matrix row 2 = PCB local Z axis in global coordinates.
    // Dot with global Z {0,0,1} = only the Z component.

    rocket_vertical_cos = local_directions_axis_vectors_in_global.m[2][2];

    // Protect acosf() from tiny floating point errors.
    if (rocket_vertical_cos > 1.0f) {
      rocket_vertical_cos = 1.0f;
    }
    else if (rocket_vertical_cos < -1.0f) {
      rocket_vertical_cos = -1.0f;
    }

    rocket_tilt_angle_rad = acosf(rocket_vertical_cos);

    // Optional. Remove if you do not need degrees.
    rocket_tilt_angle_deg = rocket_tilt_angle_rad * 57.2957795f;
  }

  void quaternion_to_local_direction_matrix() {
    Vector3f local_x_global = quaternion_rotate_vector(PCB_orientation_quat, {1.0f, 0.0f, 0.0f});
    Vector3f local_y_global = quaternion_rotate_vector(PCB_orientation_quat, {0.0f, 1.0f, 0.0f});
    Vector3f local_z_global = quaternion_rotate_vector(PCB_orientation_quat, {0.0f, 0.0f, 1.0f});

    local_directions_axis_vectors_in_global.m[0][0] = local_x_global.x;
    local_directions_axis_vectors_in_global.m[0][1] = local_x_global.y;
    local_directions_axis_vectors_in_global.m[0][2] = local_x_global.z;

    local_directions_axis_vectors_in_global.m[1][0] = local_y_global.x;
    local_directions_axis_vectors_in_global.m[1][1] = local_y_global.y;
    local_directions_axis_vectors_in_global.m[1][2] = local_y_global.z;

    local_directions_axis_vectors_in_global.m[2][0] = local_z_global.x;
    local_directions_axis_vectors_in_global.m[2][1] = local_z_global.y;
    local_directions_axis_vectors_in_global.m[2][2] = local_z_global.z;
  }

  void local_direction_matrix_to_quaternion() {
    // Your matrix stores local axes as rows.
    // Standard rotation matrix uses those same axes as columns.
    // So:
    // R00 = m[0][0], R01 = m[1][0], R02 = m[2][0]
    // R10 = m[0][1], R11 = m[1][1], R12 = m[2][1]
    // R20 = m[0][2], R21 = m[1][2], R22 = m[2][2]

    float R00 = local_directions_axis_vectors_in_global.m[0][0];
    float R01 = local_directions_axis_vectors_in_global.m[1][0];
    float R02 = local_directions_axis_vectors_in_global.m[2][0];

    float R10 = local_directions_axis_vectors_in_global.m[0][1];
    float R11 = local_directions_axis_vectors_in_global.m[1][1];
    float R12 = local_directions_axis_vectors_in_global.m[2][1];

    float R20 = local_directions_axis_vectors_in_global.m[0][2];
    float R21 = local_directions_axis_vectors_in_global.m[1][2];
    float R22 = local_directions_axis_vectors_in_global.m[2][2];

    Quaternionf q;
    float trace = R00 + R11 + R22;

    if (trace > 0.0f) {
      float s = sqrtf(trace + 1.0f) * 2.0f;
      q.w = 0.25f * s;
      q.x = (R21 - R12) / s;
      q.y = (R02 - R20) / s;
      q.z = (R10 - R01) / s;
    }
    else if ((R00 > R11) && (R00 > R22)) {
      float s = sqrtf(1.0f + R00 - R11 - R22) * 2.0f;
      q.w = (R21 - R12) / s;
      q.x = 0.25f * s;
      q.y = (R01 + R10) / s;
      q.z = (R02 + R20) / s;
    }
    else if (R11 > R22) {
      float s = sqrtf(1.0f + R11 - R00 - R22) * 2.0f;
      q.w = (R02 - R20) / s;
      q.x = (R01 + R10) / s;
      q.y = 0.25f * s;
      q.z = (R12 + R21) / s;
    }
    else {
      float s = sqrtf(1.0f + R22 - R00 - R11) * 2.0f;
      q.w = (R10 - R01) / s;
      q.x = (R02 + R20) / s;
      q.y = (R12 + R21) / s;
      q.z = 0.25f * s;
    }

    PCB_orientation_quat = quaternion_normalize(q);
    PCB_orientation_quat_valid = true;
  }

  void calibrate_all(){
    // LSM
      read_LSM6DSOX();

      gyro_x_offset = lsm_gx;
      gyro_y_offset = lsm_gy;
      gyro_z_offset = lsm_gz;

    // ADXL
    // BME
      //BME_alt_offset

  };

  void reset_altitude_kalman_filter() {
    kalman_altitude_m = 0.0f;
    kalman_vertical_speed_mps = 0.0f;

    kalman_P00 = 1.0f;
    kalman_P01 = 0.0f;
    kalman_P10 = 0.0f;
    kalman_P11 = 1.0f;

    kalman_last_update_us = 0;
    kalman_dt_s = 0.0f;

    kalman_altitude_initialized = false;
  }

  void update_altitude_kalman_filter() {
    // Only update filter when barometer has a new real measurement.
    if (!BME680_new_data) {
      return;
    }

    float measured_altitude_m = BME680_altitude_m;
    uint32_t now_us = micros();

    if (!kalman_altitude_initialized) {
      kalman_altitude_m = measured_altitude_m;
      kalman_vertical_speed_mps = 0.0f;

      kalman_P00 = 1.0f;
      kalman_P01 = 0.0f;
      kalman_P10 = 0.0f;
      kalman_P11 = 1.0f;

      kalman_last_update_us = now_us;
      kalman_dt_s = 0.0f;

      kalman_altitude_initialized = true;
      return;
    }

    uint32_t dt_us = now_us - kalman_last_update_us;
    kalman_last_update_us = now_us;

    kalman_dt_s = dt_us / 1000000.0f;

    if (kalman_dt_s <= 0.0f || kalman_dt_s > 2.0f) {
      return;
    }

    float dt = kalman_dt_s;

    // Prediction:
    // altitude = altitude + velocity * dt
    // velocity = velocity
    float altitude_pred = kalman_altitude_m + kalman_vertical_speed_mps * dt;
    float velocity_pred = kalman_vertical_speed_mps;

    // Covariance prediction:
    // P = F * P * F^T + Q
    // F = [1 dt]
    //     [0  1]
    float P00_pred = kalman_P00 + dt * (kalman_P10 + kalman_P01) + dt * dt * kalman_P11 + kalman_Q_altitude;
    float P01_pred = kalman_P01 + dt * kalman_P11;
    float P10_pred = kalman_P10 + dt * kalman_P11;
    float P11_pred = kalman_P11 + kalman_Q_velocity;

    // Measurement:
    // z = altitude
    // H = [1 0]
    float innovation = measured_altitude_m - altitude_pred;

    float S = P00_pred + kalman_R_altitude;

    if (S <= 0.000001f) {
      return;
    }

    // Kalman gain
    float K0 = P00_pred / S;
    float K1 = P10_pred / S;

    // State update
    kalman_altitude_m = altitude_pred + K0 * innovation;
    kalman_vertical_speed_mps = velocity_pred + K1 * innovation;

    // Covariance update
    kalman_P00 = (1.0f - K0) * P00_pred;
    kalman_P01 = (1.0f - K0) * P01_pred;
    kalman_P10 = P10_pred - K1 * P00_pred;
    kalman_P11 = P11_pred - K1 * P01_pred;
  }

  void calculate_rocket_speed_from_vertical_speed() {

    update_raw_vertical_speed();

    rocket_speed_valid = false;
    rocket_speed_raw_valid = false;

    // if (fabsf(rocket_vertical_cos) < min_vertical_cos_for_speed) {
    //   rocket_speed_mps = 0.0f;
    //   rocket_speed_raw_mps = 0.0f;
    //   return;
    // }

    if (kalman_altitude_initialized) {
      rocket_speed_mps = kalman_vertical_speed_mps / rocket_vertical_cos;
      rocket_speed_valid = true;
    }
    else {
      rocket_speed_mps = 0.0f;
    }

    if (raw_vertical_speed_initialized) {
      rocket_speed_raw_mps = raw_vertical_speed_mps / rocket_vertical_cos;
      rocket_speed_raw_valid = true;
    }
    else {
      rocket_speed_raw_mps = 0.0f;
    }
  }

  void gravity_anchor() {
    // Uses stationary accelerometer reading as UP.
    // Builds initial local-to-global orientation.
    // Convention:
    // global Z positive = up
    // gravity = negative Z
    // real_global_acc = PCB_global_acc_vector + {0, 0, -9.81}

    // If PCB local Z looks almost up, you should get matrix row 3 approximately:
    // [ 0, 0, +1 ]
    // If PCB local Y looks almost up, row 2 should be:
    // [ 0, 0, +1 ]
    // If PCB local X looks almost up, row 1 should be:
    // [ 0, 0, +1 ]

    // And stationary real acceleration should still be:

    // PCB_real_global_acc_vector ≈ 0, 0, 0

    gravity_anchor_valid = false;

    PCB_local_specific_force_vector = vector_scale(PCB_local_acc_vector, -1.0f);

    gravity_anchor_acc_mag = vector_magnitude(PCB_local_specific_force_vector);

    if (gravity_anchor_acc_mag < 6.0f || gravity_anchor_acc_mag > 13.0f) {
      LSM_makes_sense = false;
      gravity_anchor_valid = false;

      Serial.println("GRAVITY ANCHOR FAILED: acceleration magnitude out of range");
      Serial.print("gravity_anchor_acc_mag = ");
      Serial.println(gravity_anchor_acc_mag, 4);

      return;
    }

    LSM_makes_sense = true;

    Vector3f local_up = vector_normalize(PCB_local_specific_force_vector);
    Matrix3f R_standard = rotation_matrix_from_vector_to_vector(local_up, global_up_vector);

    // R_standard is conventional: global = R * local.
    // Your storage convention is rows = local axes in global.
    // Therefore store transpose of R_standard.
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        local_directions_axis_vectors_in_global.m[r][c] = R_standard.m[c][r];
      }
    }

    gravity = {0.0f, 0.0f, -gravity_anchor_acc_mag};
    global_gravity_vector = gravity;

    gravity_anchor_valid = true;

    local_direction_matrix_to_quaternion();

    PCB_orientation_last_update_us = micros();
    PCB_orientation_dt_s = 0.0f;

    if (oracle_debug_enabled) {
      print_gravity_anchor_status();
    }
  }

  Vector3f local_to_global_vector(Vector3f local_vec) {
    Vector3f global_vec;

    global_vec.x =
      local_vec.x * local_directions_axis_vectors_in_global.m[0][0] +
      local_vec.y * local_directions_axis_vectors_in_global.m[1][0] +
      local_vec.z * local_directions_axis_vectors_in_global.m[2][0];

    global_vec.y =
      local_vec.x * local_directions_axis_vectors_in_global.m[0][1] +
      local_vec.y * local_directions_axis_vectors_in_global.m[1][1] +
      local_vec.z * local_directions_axis_vectors_in_global.m[2][1];

    global_vec.z =
      local_vec.x * local_directions_axis_vectors_in_global.m[0][2] +
      local_vec.y * local_directions_axis_vectors_in_global.m[1][2] +
      local_vec.z * local_directions_axis_vectors_in_global.m[2][2];

    return global_vec;
  }
  
  void print_orientation_for_visualizer() {
    Serial.print("ORI,");

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        Serial.print(local_directions_axis_vectors_in_global.m[r][c], 5);

        if (!(r == 2 && c == 2)) {
          Serial.print(",");
        }
      }
    }

    Serial.println();
  }

  void print_altitude_kalman_status() {
    Serial.println();
    Serial.println("----- ALTITUDE KALMAN STATUS -----");

    Serial.print("BME680_altitude_m = ");
    Serial.println(BME680_altitude_m, 4);

    Serial.print("kalman_altitude_m = ");
    Serial.println(kalman_altitude_m, 4);

    Serial.print("kalman_vertical_speed_mps = ");
    Serial.println(kalman_vertical_speed_mps, 4);

    Serial.print("kalman_dt_s = ");
    Serial.println(kalman_dt_s, 6);

    Serial.print("kalman_R_altitude = ");
    Serial.println(kalman_R_altitude, 4);

    Serial.print("kalman_Q_altitude = ");
    Serial.println(kalman_Q_altitude, 4);

    Serial.print("kalman_Q_velocity = ");
    Serial.println(kalman_Q_velocity, 4);

    Serial.print("kalman initialized = ");
    Serial.println(kalman_altitude_initialized);

    Serial.println("----------------------------------");
    Serial.println();
  }

  void update_orientation_visualizer_output() {
    

    if (!orientation_visualizer_enabled) {
      return;
    }

    if (micros() - orientation_visualizer_last_time <= ORIENTATION_VISUALIZER_RATE_US) {
      return;
    }

    read_ADXL375();

    //read_BME680();
    start_BME680_read();
    finish_BME680_read();

    read_LSM6DSOX();


    ORACLE_update();
    

    print_orientation_for_visualizer();
    orientation_visualizer_last_time = micros();
  }

  void update_raw_vertical_speed() {
    if (!BME680_new_data) {
      return;
    }

    uint32_t now_us = micros();

    if (!raw_vertical_speed_initialized) {
      previous_raw_altitude_m = BME680_altitude_m;
      raw_vertical_speed_last_update_us = now_us;
      raw_vertical_speed_mps = 0.0f;
      raw_vertical_speed_initialized = true;
      return;
    }

    uint32_t dt_us = now_us - raw_vertical_speed_last_update_us;
    raw_vertical_speed_last_update_us = now_us;

    float dt_s = dt_us / 1000000.0f;

    if (dt_s <= 0.0f || dt_s > 2.0f) {
      return;
    }

    raw_vertical_speed_mps = (BME680_altitude_m - previous_raw_altitude_m) / dt_s;
    previous_raw_altitude_m = BME680_altitude_m;
  }

  float lookup_Cd_from_speed(float speed_mps) {
    if (speed_mps <= 0.0f) {
      return Cd_from_V[0];
    }

    if (speed_mps >= APOGEE_V_MAX_FOR_TABLE) {
      return Cd_from_V[APOGEE_V_MAX_FOR_TABLE];
    }

    float q = speed_mps * APOGEE_V_STEP_INV;
    int i = (int)floorf(q);
    float f = q - (float)i;

    return Cd_from_V[i] * (1.0f - f) + Cd_from_V[i + 1] * f;
  }

  float lookup_rho_from_altitude(float altitude_m) {
    if (altitude_m <= 0.0f) {
      return rho_from_altitude[0];
    }

    if (altitude_m >= RHO_MAX_ALTITUDE_FOR_TABLE) {
      return rho_from_altitude[200];
    }

    float q = altitude_m * RHO_ALTITUDE_STEP_INV;
    int i = (int)floorf(q);
    float f = q - (float)i;

    return rho_from_altitude[i] * (1.0f - f) + rho_from_altitude[i + 1] * f;
  }

  float rocket_apogee_fast_cpp(float tilt_angle_rad,
                             float rocket_mass_kg,
                             float vertical_velocity_mps,
                             float altitude_m,
                             float dt_s) {
    apogee_prediction_step_count = 0;

    if (rocket_mass_kg <= 0.0f) {
      return altitude_m;
    }

    if (dt_s <= 0.0f) {
      return altitude_m;
    }

    if (vertical_velocity_mps <= 0.0f) {
      return altitude_m;
    }

    float prediction_gravity_mps2 = 9.80665f;

    if (gravity_anchor_valid) {
      prediction_gravity_mps2 = vector_magnitude(global_gravity_vector);

      if (prediction_gravity_mps2 < 6.0f || prediction_gravity_mps2 > 13.0f) {
        prediction_gravity_mps2 = 9.80665f;
      }
    }

    float c = cosf(tilt_angle_rad);

    // Do not divide by tiny cos(). This means rocket is too tilted for this estimate.
    if (fabsf(c) < min_vertical_cos_for_speed) {
      return altitude_m;
    }

    float total_velocity = vertical_velocity_mps / c;

    float x = 0.0f;
    float y = altitude_m;

    float vx = total_velocity * sinf(tilt_angle_rad);
    float vy = vertical_velocity_mps;

    float t = 0.0f;

    int max_steps = (int)ceilf(apogee_prediction_max_time_s / dt_s) + 1;

    float y_previous = y;
    float vy_previous = vy;

    while (vy > 0.0f && apogee_prediction_step_count < max_steps) {
      y_previous = y;
      vy_previous = vy;

      float speed = sqrtf(vx * vx + vy * vy);

      float Cd = lookup_Cd_from_speed(speed);
      float rho = lookup_rho_from_altitude(y);

      float Fd = 0.5f * rho * Cd * rocket_cross_sectional_area_m2 * speed * speed;

      float ax = 0.0f;
      float ay = -prediction_gravity_mps2;

      if (speed > 0.0f) {
        ax = -(Fd * vx / speed) / rocket_mass_kg;
        ay = -(Fd * vy / speed) / rocket_mass_kg - prediction_gravity_mps2;
      }

      // Semi-implicit Euler, same logic as MATLAB.
      vx += ax * dt_s;
      vy += ay * dt_s;

      x += vx * dt_s;
      y += vy * dt_s;

      t += dt_s;
      apogee_prediction_step_count++;
    }

    if (vy <= 0.0f) {
      float denom = vy_previous - vy;

      if (fabsf(denom) > 0.000001f) {
        float f_apogee = vy_previous / denom;
        return y_previous + f_apogee * (y - y_previous);
      }
    }

    return y; // + distance_from_PCB_to_stratalogger_along_the_rocket * vy / sqrt(vy*vy + vx*vx);
  }

  void apogee_prediction() {
    uint32_t start_us = micros();

    apogee_prediction_valid = false;

    if (!kalman_altitude_initialized) {
      predicted_apogee_m = BME680_altitude_m;
      apogee_prediction_time_us = micros() - start_us;
      return;
    }

    if (kalman_vertical_speed_mps <= 0.0f) {
      predicted_apogee_m = kalman_altitude_m;
      apogee_prediction_time_us = micros() - start_us;
      apogee_prediction_valid = true;
      return;
    }

    predicted_apogee_m = rocket_apogee_fast_cpp(
      rocket_tilt_angle_rad,
      rocket_mass,
      kalman_vertical_speed_mps,
      kalman_altitude_m,
      apogee_prediction_dt_s
    );

    apogee_prediction_time_us = micros() - start_us;
    apogee_prediction_valid = true;
  }



// ======================= ORACLE MATH dont open ===================
  Vector3f vector_add(Vector3f a, Vector3f b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
  }

  Vector3f vector_sub(Vector3f a, Vector3f b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
  }

  Vector3f vector_scale(Vector3f a, float s) {
    return {a.x * s, a.y * s, a.z * s};
  }

  float vector_dot(Vector3f a, Vector3f b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  Vector3f vector_cross(Vector3f a, Vector3f b) {
    return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    };
  }

  float vector_magnitude(Vector3f a) {
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
  }

  Vector3f vector_normalize(Vector3f a) {
    float mag = vector_magnitude(a);

    if (mag < 0.000001f) {
      return {0.0f, 0.0f, 0.0f};
    }

    return {a.x / mag, a.y / mag, a.z / mag};
  }

  Matrix3f matrix_identity() {
    Matrix3f I = {
      {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
      }
    };

    return I;
  }

  Matrix3f matrix_add(Matrix3f A, Matrix3f B) {
    Matrix3f C;

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        C.m[r][c] = A.m[r][c] + B.m[r][c];
      }
    }

    return C;
  }

  Matrix3f matrix_scale(Matrix3f A, float s) {
    Matrix3f B;

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        B.m[r][c] = A.m[r][c] * s;
      }
    }

    return B;
  }

  Matrix3f matrix_multiply(Matrix3f A, Matrix3f B) {
    Matrix3f C;

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        C.m[r][c] = 0.0f;

        for (int k = 0; k < 3; k++) {
          C.m[r][c] += A.m[r][k] * B.m[k][c];
        }
      }
    }

    return C;
  }

  Matrix3f skew_matrix(Vector3f v) {
    Matrix3f K = {
      {
        {0.0f, -v.z,  v.y},
        {v.z,   0.0f, -v.x},
        {-v.y,  v.x,  0.0f}
      }
    };

    return K;
  }

  Matrix3f rotation_matrix_from_vector_to_vector(Vector3f from_vec, Vector3f to_vec) {
    Vector3f a = vector_normalize(from_vec);
    Vector3f b = vector_normalize(to_vec);

    float c = vector_dot(a, b);

    if (c > 0.9999f) {
      return matrix_identity();
    }

    if (c < -0.9999f) {
      // 180 degree case.
      // Pick an arbitrary axis perpendicular to a.
      Vector3f temp_axis = {1.0f, 0.0f, 0.0f};

      if (fabsf(a.x) > 0.9f) {
        temp_axis = {0.0f, 1.0f, 0.0f};
      }

      Vector3f v = vector_normalize(vector_cross(a, temp_axis));

      Matrix3f K = skew_matrix(v);
      Matrix3f K2 = matrix_multiply(K, K);

      // For 180 deg: R = I + 2*K^2
      return matrix_add(matrix_identity(), matrix_scale(K2, 2.0f));
    }

    Vector3f v = vector_cross(a, b);
    float s = vector_magnitude(v);

    Matrix3f K = skew_matrix(v);
    Matrix3f K2 = matrix_multiply(K, K);

    float factor = (1.0f - c) / (s * s);

    Matrix3f R = matrix_add(
      matrix_add(matrix_identity(), K),
      matrix_scale(K2, factor)
    );

    return R;
  }

  Quaternionf quaternion_identity() {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }

  Quaternionf quaternion_normalize(Quaternionf q) {
    float mag = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (mag < 0.000001f) {
      return quaternion_identity();
    }

    q.w /= mag;
    q.x /= mag;
    q.y /= mag;
    q.z /= mag;

    return q;
  }

  Quaternionf quaternion_multiply(Quaternionf a, Quaternionf b) {
    Quaternionf q;

    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return q;
  }

  Quaternionf quaternion_from_rotation_vector(Vector3f rotation_vector) {
    // rotation_vector = axis * angle, in radians.
    // Example:
    // x = PCB_gx * dt
    // y = PCB_gy * dt
    // z = PCB_gz * dt

    float angle = vector_magnitude(rotation_vector);

    if (angle < 0.000001f) {
      return quaternion_identity();
    }

    Vector3f axis = vector_scale(rotation_vector, 1.0f / angle);

    float half_angle = angle * 0.5f;
    float s = sinf(half_angle);

    Quaternionf q;
    q.w = cosf(half_angle);
    q.x = axis.x * s;
    q.y = axis.y * s;
    q.z = axis.z * s;

    return quaternion_normalize(q);
  }

  Vector3f quaternion_rotate_vector(Quaternionf q, Vector3f v) {
    // Rotates local vector v into global frame using q.

    Quaternionf p = {0.0f, v.x, v.y, v.z};
    Quaternionf q_conj = {q.w, -q.x, -q.y, -q.z};

    Quaternionf result = quaternion_multiply(
      quaternion_multiply(q, p),
      q_conj
    );

    return {result.x, result.y, result.z};
  }

// =============== ALONE IN THE DESERT (last stage) ================
  void send_current_location() {
    if (RFM9X_FAIL || !TX_enabled) {
      return;
    }

    if (micros() - GPS_location_send_last_time_us < GPS_location_send_rate_us) {
      return;
    }

    prep_current_location_message();
    send_RFM();

    GPS_location_send_last_time_us = micros();
  }

  void reset_data_gathering_flight_flags() {
    apogee_detected = false;
    relief_opened_after_apogee = false;

    previous_kalman_vertical_speed_mps = 0.0f;
    previous_vertical_speed_valid = false;

    apogee_detected_time_ms = 0;
    GPS_location_send_last_time_us = 0;
  }

  void detect_apogee_after_engine_cutoff() {
    if (apogee_detected) {
      return;
    }

    if (!kalman_altitude_initialized) {
      return;
    }

    float current_vz = kalman_vertical_speed_mps;

    if (!previous_vertical_speed_valid) {
      previous_kalman_vertical_speed_mps = current_vz;
      previous_vertical_speed_valid = true;
      return;
    }

    bool crossed_apogee =
      previous_kalman_vertical_speed_mps > apogee_detection_vz_threshold_mps &&
      current_vz <= -apogee_detection_vz_threshold_mps;

    if (crossed_apogee) {
      apogee_detected = true;
      apogee_detected_time_ms = millis();

      Serial.println("APOGEE DETECTED");
    }

    previous_kalman_vertical_speed_mps = current_vz;
  }

  void update_relief_after_apogee() {
    if (relief_opened_after_apogee) {
      return;
    }

    uint32_t time_since_stage_entry_ms = millis() - stage_entry_time_ms;

    bool min_time_passed =
      time_since_stage_entry_ms >= MIN_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS;

    bool max_time_passed =
      time_since_stage_entry_ms >= MAX_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS;

    bool apogee_delay_passed = false;

    if (apogee_detected) {
      apogee_delay_passed =
        (millis() - apogee_detected_time_ms) >= RELIEF_OPEN_DELAY_AFTER_APOGEE_MS;
    }

    if ((apogee_detected && apogee_delay_passed && min_time_passed) || max_time_passed) {
      VALVE_open(RELIEF_VALVE);
      relief_opened_after_apogee = true;

      Serial.println("RELIEF VALVE OPENED AFTER APOGEE / MAX TIME");
    }
  }

  void prep_current_location_message() {
    // Msg 0x02-0x07 structure: {5,5,5}
    // in_floats[0] = GPS latitude
    // in_floats[1] = GPS longitude
    // in_floats[2] = GPS altitude, m

    in_floats[0] = (float)gps_lat;
    in_floats[1] = (float)gps_lon;
    in_floats[2] = gps_alt_m;

    message_assembler(0x02, 0x07);
  }

// ================= DATA CONVERSION / HELPER SECTION ==============


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

  float interpolate_voltage_to_pressure_psi(
    float voltage,
    const float voltage_table[],
    const float pressure_table[],
    int table_size
  ) {
    if (table_size < 2) {
      return 0.0f;
    }

    // Clamp below calibrated range
    if (voltage <= voltage_table[0]) {
      return pressure_table[0];
    }

    // Clamp above calibrated range
    if (voltage >= voltage_table[table_size - 1]) {
      return pressure_table[table_size - 1];
    }

    for (int i = 0; i < table_size - 1; i++) {
      float v0 = voltage_table[i];
      float v1 = voltage_table[i + 1];

      if (voltage >= v0 && voltage <= v1) {
        float p0 = pressure_table[i];
        float p1 = pressure_table[i + 1];

        float f = (voltage - v0) / (v1 - v0);

        return p0 + f * (p1 - p0);
      }
    }



    return pressure_table[table_size - 1];
  }

  float combust_voltage_to_pressure_psi(float voltage) {
    return interpolate_voltage_to_pressure_psi(
      voltage,
      combust_voltage_table,
      combust_pressure_psi_table,
      COMBUST_PRESSURE_TABLE_SIZE
    );
  }

  float n2o_voltage_to_pressure_psi(float voltage) {
    return interpolate_voltage_to_pressure_psi(
      voltage,
      n2o_voltage_table,
      n2o_pressure_psi_table,
      N2O_PRESSURE_TABLE_SIZE
    );
  }

  float co2_voltage_to_pressure_psi(float voltage) {
    return interpolate_voltage_to_pressure_psi(
      voltage,
      co2_voltage_table,
      co2_pressure_psi_table,
      CO2_PRESSURE_TABLE_SIZE
    );
  }

// ====================== DEBUG / LOCAL TEST SECTION ===============


  void debug_menu() {
    Serial.println();
    Serial.println("===== ALAN V5 BASE DEBUG MENU =====");
    Serial.println("m = print this menu");
    Serial.println("s = print local status");
    Serial.println("k = print ignition control status");
    Serial.println("a = print active pins");
    Serial.println("v = run valve self-test");
    Serial.println("p = toggle pressure stream flag");
    Serial.println("i = toggle IMU stream flag");
    Serial.println("b = toggle BME680 stream flag");
    Serial.println("g = toggle GPS stream flag");
    Serial.println("r = toggle RTC stream flag");
    Serial.println("t = build telemetry and print raw packet");
    Serial.println("e = enable TX");
    Serial.println("d = disable TX");
    Serial.println("f = set radio frequency from Serial input");
    Serial.println("U = raw GPS UART test");
    Serial.println("P = GPS pin activity test");
    Serial.println("Y = GPS coordinates");
    Serial.println("R = print RTC status");
    Serial.println("T = RTC time set");
    Serial.println("K = print ARM sequence status");
    Serial.println("O = run gravity anchor and print ORACLE orientation status");
    Serial.println("o = toggle ORACLE debug");
    Serial.println("V = toggle orientation visualizer output");
    Serial.println("Q = print gyro/quaternion orientation status");
    Serial.println("q = toggle gyro/quaternion debug stream");
    Serial.println("M = toggle rocket tilt angle stream");
    Serial.println("B = calibrate BME680 pressure threshold");
    Serial.println("N = print BME680 pressure filter status");
    Serial.println("n = print altitude Kalman status");
    Serial.println("0 = reset altitude Kalman filter");
    Serial.println("G = toggle rocket speed stream");
    Serial.println("j = print apogee prediction status");

    Serial.println("1 = local ABORT");
    Serial.println("2 = local ARM");
    Serial.println("3 = local IGNITION START");
    Serial.println("4 = local OPEN MAIN");
    Serial.println("5 = local CLOSE MAIN");
    Serial.println("8 = local OPEN RELIEF");
    Serial.println("9 = local CLOSE RELIEF");

    Serial.println("y = full local ignition sequence test");
    Serial.println("c = print valve connection status");
    Serial.println("C = toggle valve connection stream");
    Serial.println("x = print current stage");
    Serial.println("w = set stage WAITING");
    Serial.println("A = set stage ARM");
    Serial.println("L = set stage LAUNCH IGNITION SEQUENCE");
    Serial.println("E = set stage ENGINE WORKING");
    Serial.println("F = set stage DATA GATHERING FLIGHT");
    
    Serial.println("===============================");
    Serial.println();
  }

  void debug_serial() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "m") {
      debug_menu();
    }
    else if (cmd == "s") {
      print_local_status();
    }
    else if (cmd == "k") {
      print_ignition_control_status();
    }
    else if (cmd == "a") {
      print_active_pins();
    }
    else if (cmd == "v") {
      run_valve_self_test();
    }
    else if (cmd == "p") {
      pressure_stream_enabled = !pressure_stream_enabled;
      Serial.print("Pressure stream flag ");
      Serial.println(pressure_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "i") {
      imu_stream_enabled = !imu_stream_enabled;
      Serial.print("IMU stream flag ");
      Serial.println(imu_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "b") {
      bme_stream_enabled = !bme_stream_enabled;
      Serial.print("BME680 stream flag ");
      Serial.println(bme_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "g") {
      gps_stream_enabled = !gps_stream_enabled;
      Serial.print("GPS stream flag ");
      Serial.println(gps_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "r") {
      rtc_stream_enabled = !rtc_stream_enabled;
      Serial.print("RTC stream flag ");
      Serial.println(rtc_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "t") {
      prep_telemetry();
      Serial.print("Telemetry length = ");
      Serial.println(message_send_len);
      Serial.print("Telemetry packet = ");
      print_hex_buffer(message_send_buf, message_send_len);
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
    else if (cmd == "U") {
      raw_GPS_UART_test();
    }
    else if (cmd == "P") {
      GPS_pin_activity_test();
    }
    else if (cmd == "Y") {
      print_GPS_status();
    }
    else if (cmd == "R") {
      print_RTC_status();
    }
    else if (cmd == "T") {
      set_DS1307_from_serial();
    }
    else if (cmd == "K") {
      print_arm_sequence_status();
    }
    else if (cmd == "O") {
      read_LSM6DSOX();
      LSM_to_PCB_swithch_over();
      gravity_anchor();
      print_gravity_anchor_status();
    }
    else if (cmd == "o") {
      oracle_debug_enabled = !oracle_debug_enabled;
      Serial.print("ORACLE debug ");
      Serial.println(oracle_debug_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "V") {
      orientation_visualizer_enabled = !orientation_visualizer_enabled;

      Serial.print("Orientation visualizer ");
      Serial.println(orientation_visualizer_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "Q") {
      print_gyro_orientation_status();
    }
    else if (cmd == "q") {
      gyro_orientation_debug_enabled = !gyro_orientation_debug_enabled;

      Serial.print("Gyro orientation debug ");
      Serial.println(gyro_orientation_debug_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "M") {
      rocket_tilt_stream_enabled = !rocket_tilt_stream_enabled;

      Serial.print("Rocket tilt stream ");
      Serial.println(rocket_tilt_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "B") {
      calibrate_BME680_pressure_threshold();
    }
    else if (cmd == "N") {
      print_BME680_pressure_filter_status();
    }
    else if (cmd == "n") {
      print_altitude_kalman_status();
    }
    else if (cmd == "0") {
      reset_altitude_kalman_filter();
      Serial.println("Altitude Kalman filter reset");
    }
    else if (cmd == "G") {
      rocket_speed_stream_enabled = !rocket_speed_stream_enabled;

      Serial.print("Rocket speed stream ");
      Serial.println(rocket_speed_stream_enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd == "j") {
      print_apogee_prediction_status();
    }



    else if (cmd == "1") {
      debug_abort_av1();
    }
    else if (cmd == "2") {
      debug_arm_av1();
    }
    else if (cmd == "3") {
      debug_start_ignition_sequence();
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
    else if (cmd == "y") {
      run_full_local_ignition_test();
    }
    else if (cmd == "c") {
      read_valve_connection();
      print_valve_connection_status();
    }
    else if (cmd == "C") {
      valve_connection_stream_enabled = !valve_connection_stream_enabled;

      Serial.print("Valve connection stream ");
      Serial.println(valve_connection_stream_enabled ? "ENABLED" : "DISABLED");
    }

    else if (cmd == "x") {
      Serial.print("Current stage = ");
      Serial.print(stage);
      Serial.print(" / ");
      Serial.println(stage_name(stage));
    }

    else if (cmd == "w") {
      set_stage(STAGE_WAITING);
    }

    else if (cmd == "A") {
      set_stage(STAGE_ARM);
    }

    else if (cmd == "L") {
      set_stage(STAGE_LAUNCH_IGNITION_SEQUENCE);
    }

    else if (cmd == "E") {
      set_stage(STAGE_ENGINE_WORKING);
    }

    else if (cmd == "F") {
      set_stage(STAGE_DATA_GATHERING_FLIGHT);
    }
    else {
      Serial.print("Unknown debug command: ");
      Serial.println(cmd);
      debug_menu();
    }
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
    Serial.print("Stage                   = "); Serial.print(stage); Serial.print(" / "); Serial.println(stage_name(stage));
    Serial.print("MAIN valve state        = "); Serial.println(MAIN_VALVE.state);
    Serial.print("RELIEF valve state      = "); Serial.println(RELIEF_VALVE.state);
    Serial.print("DUMP valve state        = "); Serial.println(DUMP_VALVE.state);
    print_valve_connection_status();
    Serial.print("Pressure 1 output       = "); Serial.println(pressure_1_output, 3);
    Serial.print("Pressure 2 output       = "); Serial.println(pressure_2_output, 3);
    Serial.print("Pressure 3 output       = "); Serial.println(pressure_3_output, 3);
    Serial.print("Pressure 4 output       = "); Serial.println(pressure_4_output, 3);
    Serial.print("Battery voltage         = "); Serial.println(battery_voltage_output, 3);
    Serial.print("Commands received       = "); Serial.println(commands_received);
    Serial.print("State array             = "); Serial.println(compute_state_array());
    Serial.print("TX enabled              = "); Serial.println(TX_enabled);
    Serial.print("RFM9X_FAIL              = "); Serial.println(RFM9X_FAIL);
    Serial.print("CONTROL_FREQ            = "); Serial.println(CONTROL_FREQ, 3);
    Serial.print("BME680 temp             = "); Serial.println(BME680_temp, 3);
    Serial.print("BME680 pressure hPa     = "); Serial.println(BME680_pressure_hpa, 3);
    Serial.print("BME680 altitude m       = "); Serial.println(BME680_altitude_m, 3);
    Serial.print("GPS lat                 = "); Serial.println(gps_lat, 7);
    Serial.print("GPS lon                 = "); Serial.println(gps_lon, 7);
    Serial.print("GPS alt m               = "); Serial.println(gps_alt_m, 3);
    Serial.print("GPS SIV                 = "); Serial.println(gps_siv);
    Serial.print("GPS fix type            = "); Serial.println(gps_fix_type);
    Serial.print("GPS PPS state           = "); Serial.println(gps_pps_state);
    Serial.print("RTC date                = ");
    Serial.print(rtc_year); Serial.print('-');
    Serial.print(rtc_month); Serial.print('-');
    Serial.println(rtc_day);
    Serial.print("RTC time                = ");
    Serial.print(rtc_hour); Serial.print(':');
    Serial.print(rtc_minute); Serial.print(':');
    Serial.println(rtc_second);
    Serial.println("------------------------");
    Serial.println();
  }

  void print_ignition_control_status() {
    Serial.println();
    Serial.println("----- IGNITION CONTROL STATUS -----");
    Serial.print("AVIONICS armed              = "); Serial.println(avionics_armed);
    Serial.print("Ignition sequence started   = "); Serial.println(ignition_sequence_started);
    Serial.print("MAIN close by radio allowed = "); Serial.println(main_valve_can_close_by_radio);
    Serial.print("MAIN ignition hold active   = "); Serial.println(main_valve_ignition_hold_active);

    if (main_valve_ignition_hold_active) {
      uint32_t elapsed_ms = millis() - main_valve_ignition_start_time_ms;
      Serial.print("Ignition elapsed ms         = "); Serial.println(elapsed_ms);
      Serial.print("Countdown                   = "); Serial.println(countdown);
    }

    Serial.println("-----------------------------------");
    Serial.println();
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
    Serial.print("RFM9X_CS_pin            = "); Serial.println(RFM9X_CS_pin);
    Serial.print("RFM9X_RST_pin           = "); Serial.println(RFM9X_RST_pin);
    Serial.print("RFM9X_PWR_pin           = "); Serial.println(RFM9X_PWR_pin);
    Serial.print("I2C_MAIN_SDA_pin        = "); Serial.println(I2C_MAIN_SDA_pin);
    Serial.print("I2C_MAIN_SCL_pin        = "); Serial.println(I2C_MAIN_SCL_pin);
    Serial.print("ADXL375_SDA_pin         = "); Serial.println(ADXL375_SDA_pin);
    Serial.print("ADXL375_SCL_pin         = "); Serial.println(ADXL375_SCL_pin);
    Serial.print("GPS_RX_pin              = "); Serial.println(GPS_RX_pin);
    Serial.print("GPS_TX_pin              = "); Serial.println(GPS_TX_pin);
    Serial.print("GPS_PPS_pin             = "); Serial.println(GPS_PPS_pin);
    Serial.print("DEBUG_DISPLAY_SDA_pin   = "); Serial.println(DEBUG_DISPLAY_SDA_pin);
    Serial.print("DEBUG_DISPLAY_SCL_pin   = "); Serial.println(DEBUG_DISPLAY_SCL_pin);
    Serial.println("-----------------------");
    Serial.println();
  }

  void run_valve_self_test() {
    Serial.println("Running valve self-test...");

    close_all_valves();
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

    Serial.println("Opening DUMP");
    VALVE_open(DUMP_VALVE);
    print_local_status();
    delay(2000);

    Serial.println("Closing DUMP");
    VALVE_close(DUMP_VALVE);
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

    execute_debug_command(msg_id);
    print_local_status();
  }

  void debug_arm_av1() {
    Serial.println("DEBUG: local ARM");
    inject_local_command(0x02);
  }

  void debug_abort_av1() {
    Serial.println("DEBUG: local ABORT");
    inject_local_command(0x01);
  }

  void debug_start_ignition_sequence() {
    if (!avionics_armed) {
      Serial.println("DEBUG: ignition start blocked, AV1 not armed");
      print_ignition_control_status();
      return;
    }

    if (main_valve_ignition_hold_active) {
      Serial.println("DEBUG: ignition already active");
      print_ignition_control_status();
      return;
    }

    Serial.println("DEBUG: local IGNITION START");
    inject_local_command(0x03);
  }

  void run_full_local_ignition_test() {
    Serial.println();
    Serial.println("===== FULL LOCAL IGNITION TEST =====");

    Serial.println("STEP 1: ABORT / RESET");
    debug_abort_av1();
    delay(300);

    Serial.println("STEP 2: ARM");
    debug_arm_av1();
    delay(300);

    Serial.println("STEP 3: IGNITION START");
    debug_start_ignition_sequence();
    delay(300);

    Serial.println("STEP 4: WAIT FOR AUTO-CLOSE");
    uint32_t last_print_ms = 0;

    while (main_valve_ignition_hold_active) {
      update_main_valve_ignition_hold();

      if (millis() - last_print_ms >= 250) {
        print_ignition_control_status();
        last_print_ms = millis();
      }

      delay(10);
    }

    Serial.println("STEP 5: FINAL STATUS");
    print_ignition_control_status();
    print_local_status();

    Serial.println("===== FULL LOCAL IGNITION TEST COMPLETE =====");
    Serial.println();
  }

  void streams() {
    // This function only prints already-stored values.
    // It does not read sensors by itself.

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
      Serial.print("BME680 T = ");
      Serial.print(BME680_temp, 3);

      Serial.print(" | P hPa = ");
      Serial.print(BME680_pressure_hpa, 3);

      Serial.print(" | ALT m = ");
      Serial.print(BME680_altitude_m, 3);

      Serial.print(" | Hz = ");
      Serial.println(BME680_update_hz, 2);
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
      Serial.print(rtc_year); Serial.print('-');
      Serial.print(rtc_month); Serial.print('-');
      Serial.print(rtc_day); Serial.print(' ');
      Serial.print(rtc_hour); Serial.print(':');
      Serial.print(rtc_minute); Serial.print(':');
      Serial.println(rtc_second);
    }

    if (valve_connection_stream_enabled) {
      read_valve_connection();

      Serial.print("VALVE CONNECTION | Main = ");
      Serial.print(MAIN_VALVE.connection_voltage, 3);
      Serial.print(" V / ");
      Serial.print(MAIN_VALVE.connection_ok ? "OK" : "NO");

      Serial.print(" | Relief = ");
      Serial.print(RELIEF_VALVE.connection_voltage, 3);
      Serial.print(" V / ");
      Serial.print(RELIEF_VALVE.connection_ok ? "OK" : "NO");

      Serial.print(" | Dump = ");
      Serial.print(DUMP_VALVE.connection_voltage, 3);
      Serial.print(" V / ");
      Serial.println(DUMP_VALVE.connection_ok ? "OK" : "NO");
    }

    if (oracle_debug_enabled) {
      Serial.print(PCB_real_global_acc_vector.x);
      Serial.print(" ");
      Serial.print(PCB_real_global_acc_vector.y);
      Serial.print(" ");
      Serial.print(PCB_real_global_acc_vector.z);
      Serial.println("");
    }

    if (rocket_tilt_stream_enabled) {
      calculate_rocket_tilt_from_orientation();

      Serial.print("TILT deg = ");
      Serial.print(rocket_tilt_angle_deg, 3);
      Serial.print(" | cos = ");
      Serial.println(rocket_vertical_cos, 5);
    }
    
    if (rocket_speed_stream_enabled) {
      stream_rocket_speed_status();
    }
    
  }

  void stream_rocket_speed_status() {
      Serial.print("TILT deg = ");
      Serial.print(rocket_tilt_angle_deg, 3);

      Serial.print(" | Vz Kalman = ");
      Serial.print(kalman_vertical_speed_mps, 3);

      Serial.print(" | Vz Raw = ");
      Serial.print(raw_vertical_speed_mps, 3);

      Serial.print(" | Rocket Speed Kalman = ");
      Serial.print(rocket_speed_mps, 3);

      Serial.print(" | Rocket Speed Raw = ");
      Serial.print(rocket_speed_raw_mps, 3);

      Serial.print(" | Kalman valid = ");
      Serial.print(rocket_speed_valid);

      Serial.print(" | Raw valid = ");
      Serial.println(rocket_speed_raw_valid);
    }

  void print_load_progress(int prog) {
     Serial.print("Load progress: "); Serial.print(prog); Serial.print(" |");
     for (int i = 0; i <= round(prog/2); i++) {
        Serial.print("="); 
     }
     Serial.print(">");
     for (int i = 0; i < 100/2 - round(prog/2); i++) {
        Serial.print("_"); 
     }
     Serial.println("|");
  }

  void raw_GPS_UART_test() {
    Serial.println();
    Serial.println("----- RAW GPS SOFTWARE SERIAL TEST -----");

    GPS_PORT.begin(9600);
    delay(500);

    uint32_t start_ms = millis();
    uint32_t byte_count = 0;

    while (millis() - start_ms < 10000) {
      while (GPS_PORT.available()) {
        char c = GPS_PORT.read();
        Serial.write(c);
        byte_count++;
      }
    }

    Serial.println();
    Serial.print("SoftwareSerial GPS bytes received: ");
    Serial.println(byte_count);
    Serial.println("----------------------------------------");
    Serial.println();
  }

  void GPS_pin_activity_test() {
    Serial.println();
    Serial.println("----- GPS PIN ACTIVITY TEST -----");

    pinMode(GPS_RX_pin, INPUT); // Teensy pin 7
    pinMode(GPS_TX_pin, INPUT); // Teensy pin 8, temporarily input for test only

    uint32_t start_ms = millis();

    uint32_t pin7_changes = 0;
    uint32_t pin8_changes = 0;

    int last7 = digitalRead(GPS_RX_pin);
    int last8 = digitalRead(GPS_TX_pin);

    while (millis() - start_ms < 5000) {
      int now7 = digitalRead(GPS_RX_pin);
      int now8 = digitalRead(GPS_TX_pin);

    if (now7 != last7) {
      pin7_changes++;
      last7 = now7;
    }

    if (now8 != last8) {
      pin8_changes++;
      last8 = now8;
    }
  }

    Serial.print("Pin 7 / RX2 changes = ");
    Serial.println(pin7_changes);

    Serial.print("Pin 8 / TX2 changes = ");
    Serial.println(pin8_changes);

    Serial.print("Pin 7 final state = ");
    Serial.println(digitalRead(GPS_RX_pin));

    Serial.print("Pin 8 final state = ");
    Serial.println(digitalRead(GPS_TX_pin));

    Serial.println("-------------------------------");
    Serial.println();

    //GPS_PORT.setRX(GPS_RX_pin);
    //GPS_PORT.setTX(GPS_TX_pin);
    GPS_PORT.begin(9600);
  }

  void print_GPS_status() {
    Serial.print("GPS LAT = ");
    Serial.print(gps_lat, 7);

    Serial.print(" | LON = ");
    Serial.print(gps_lon, 7);

    Serial.print(" | ALT = ");
    Serial.print(gps_alt_m, 3);

    Serial.print(" | SIV = ");
    Serial.print(gps_siv);

    Serial.print(" | FIX = ");
    Serial.print(gps_fix_type);

    Serial.print(" | PPS = ");
    Serial.println(gps_pps_state);
  }

  void print_RTC_status() {
    read_DS1307();

    Serial.print("RTC = ");
    Serial.print(rtc_year);
    Serial.print("-");
    Serial.print(rtc_month);
    Serial.print("-");
    Serial.print(rtc_day);
    Serial.print(" ");
    Serial.print(rtc_hour);
    Serial.print(":");
    Serial.print(rtc_minute);
    Serial.print(":");
    Serial.println(rtc_second);

    Serial.print("DS1307_FAIL = ");
    Serial.println(DS1307_FAIL);
  }

  int read_serial_int(const char* prompt) {
    Serial.println(prompt);

    while (Serial.available()) {
      Serial.read();  // clear old leftover characters
    }

    while (!Serial.available()) {
      // wait for user input
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    return input.toInt();
  }

  void set_DS1307_from_serial() {
    Serial.println();
    Serial.println("Set DS1307 RTC time");
    Serial.println("Enter one value, press Enter, then wait for next question.");

    int year = read_serial_int("Year, last two digits, example 26 for 2026:");
    int month = read_serial_int("Month, 1-12:");
    int day = read_serial_int("Day of month, 1-31:");
    int day_of_week = read_serial_int("Day of week, 1-7:");
    int hour = read_serial_int("Hour, 0-23:");
    int minute = read_serial_int("Minute, 0-59:");
    int second = read_serial_int("Second, 0-59:");

    rtc.set(
      second,
      minute,
      hour,
      day_of_week,
      day,
      month,
      year
    );

    delay(200);
    rtc.refresh();

    Serial.println();
    Serial.println("RTC time set.");

    Serial.print("RTC = 20");
    Serial.print(rtc.year());
    Serial.print("-");
    Serial.print(rtc.month());
    Serial.print("-");
    Serial.print(rtc.day());
    Serial.print(" ");
    Serial.print(rtc.hour());
    Serial.print(":");
    Serial.print(rtc.minute());
    Serial.print(":");
    Serial.println(rtc.second());
  }

  void print_BME680_pressure_filter_status() {
    Serial.println();
    Serial.println("----- BME680 PRESSURE FILTER STATUS -----");

    Serial.print("BME680_calibration_duration_s = ");
    Serial.println(BME680_calibration_duration_s, 2);

    Serial.print("BME680_pressure_threshold_hpa = ");
    Serial.println(BME680_pressure_threshold_hpa, 5);

    Serial.print("half threshold hPa = ");
    Serial.println(BME680_pressure_threshold_hpa * 0.5f, 5);

    Serial.print("BME680_filtered_pressure_hpa = ");
    Serial.println(BME680_filtered_pressure_hpa, 5);

    Serial.print("BME680_pressure_hpa current = ");
    Serial.println(BME680_pressure_hpa, 5);

    Serial.print("BME680_altitude_m current = ");
    Serial.println(BME680_altitude_m, 5);

    Serial.print("BME680_filter_started = ");
    Serial.println(BME680_filter_started);

    Serial.println("-----------------------------------------");
    Serial.println();
  }

  void debug_laptop(){
    
  };



  // oracle
    void print_gravity_anchor_status() {
      Serial.println();
      Serial.println("----- GRAVITY ANCHOR STATUS -----");

      Serial.print("gravity_anchor_valid = ");
      Serial.println(gravity_anchor_valid);

      Serial.print("gravity_anchor_acc_mag = ");
      Serial.println(gravity_anchor_acc_mag, 4);

      Serial.print("PCB_local_acc_vector = ");
      Serial.print(PCB_local_acc_vector.x, 4);
      Serial.print(", ");
      Serial.print(PCB_local_acc_vector.y, 4);
      Serial.print(", ");
      Serial.println(PCB_local_acc_vector.z, 4);

      Serial.print("PCB_global_acc_vector = ");
      Serial.print(PCB_global_acc_vector.x, 4);
      Serial.print(", ");
      Serial.print(PCB_global_acc_vector.y, 4);
      Serial.print(", ");
      Serial.println(PCB_global_acc_vector.z, 4);

      Serial.print("global_gravity_vector = ");
      Serial.print(global_gravity_vector.x, 4);
      Serial.print(", ");
      Serial.print(global_gravity_vector.y, 4);
      Serial.print(", ");
      Serial.println(global_gravity_vector.z, 4);

      Serial.print("PCB_real_global_acc_vector = ");
      Serial.print(PCB_real_global_acc_vector.x, 4);
      Serial.print(", ");
      Serial.print(PCB_real_global_acc_vector.y, 4);
      Serial.print(", ");
      Serial.println(PCB_real_global_acc_vector.z, 4);

      Serial.println("local_directions_axis_vectors_in_global:");

      for (int r = 0; r < 3; r++) {
        Serial.print("[ ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][0], 5);
        Serial.print(", ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][1], 5);
        Serial.print(", ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][2], 5);
        Serial.println(" ]");
      }

      Serial.println("---------------------------------");
      Serial.println();
    }

    void print_gyro_orientation_status() {
      Serial.println();
      Serial.println("----- GYRO ORIENTATION STATUS -----");

      Serial.print("PCB_orientation_dt_s = ");
      Serial.println(PCB_orientation_dt_s, 6);

      Serial.print("PCB gyro local rad/s = ");
      Serial.print(PCB_local_gyro_vector.x, 6);
      Serial.print(", ");
      Serial.print(PCB_local_gyro_vector.y, 6);
      Serial.print(", ");
      Serial.println(PCB_local_gyro_vector.z, 6);

      Serial.print("local_direction_change rad = ");
      Serial.print(local_direction_change.x, 6);
      Serial.print(", ");
      Serial.print(local_direction_change.y, 6);
      Serial.print(", ");
      Serial.println(local_direction_change.z, 6);

      Serial.print("PCB_orientation_quat = ");
      Serial.print(PCB_orientation_quat.w, 6);
      Serial.print(", ");
      Serial.print(PCB_orientation_quat.x, 6);
      Serial.print(", ");
      Serial.print(PCB_orientation_quat.y, 6);
      Serial.print(", ");
      Serial.println(PCB_orientation_quat.z, 6);

      Serial.println("local_directions_axis_vectors_in_global:");
      for (int r = 0; r < 3; r++) {
        Serial.print("[ ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][0], 5);
        Serial.print(", ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][1], 5);
        Serial.print(", ");
        Serial.print(local_directions_axis_vectors_in_global.m[r][2], 5);
        Serial.println(" ]");
      }

      Serial.println("-----------------------------------");
      Serial.println();
    }

    void print_apogee_prediction_status() {
      Serial.println();
      Serial.println("----- APOGEE PREDICTION STATUS -----");

      Serial.print("predicted_apogee_m = ");
      Serial.println(predicted_apogee_m, 3);

      Serial.print("prediction_valid = ");
      Serial.println(apogee_prediction_valid);

      Serial.print("prediction_time_us = ");
      Serial.println(apogee_prediction_time_us);

      Serial.print("prediction_step_count = ");
      Serial.println(apogee_prediction_step_count);

      Serial.print("prediction_dt_s = ");
      Serial.println(apogee_prediction_dt_s, 4);

      Serial.print("rocket_mass kg = ");
      Serial.println(rocket_mass, 3);

      Serial.print("cross_sectional_area_m2 = ");
      Serial.println(rocket_cross_sectional_area_m2, 6);

      Serial.print("altitude used m = ");
      Serial.println(kalman_altitude_m, 3);

      Serial.print("vertical_velocity used m/s = ");
      Serial.println(kalman_vertical_speed_mps, 3);

      Serial.print("tilt used deg = ");
      Serial.println(rocket_tilt_angle_deg, 3);

      Serial.println("------------------------------------");
      Serial.println();
    }



// ============================= END ===============================