/*
  ALAN VF F Mykhailo

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
  //#include <Adafruit_BME280.h>
  #include <Adafruit_BMP3XX.h>

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
    // 1 - pin 41 - N2O/fuel tank
    // 2 - pin 40 - extra
    // 3 - pin 15 - combustion tank
    // 4 - pin 27 - CO2 pressure level
    const uint8_t PRESSURE_TRANSDUCER_1_pin = 41;
    const uint8_t PRESSURE_TRANSDUCER_2_pin = 40;
    const uint8_t PRESSURE_TRANSDUCER_3_pin = 15;
    const uint8_t PRESSURE_TRANSDUCER_4_pin = 27;


    // // temp 2 <- N2O
    // // temp 3 <- CO2 
    // const uint8_t PRESSURE_TRANSDUCER_1_pin = 40;
    // const uint8_t PRESSURE_TRANSDUCER_2_pin = 41;
    // const uint8_t PRESSURE_TRANSDUCER_3_pin = 27;
    // const uint8_t PRESSURE_TRANSDUCER_4_pin = 15;

    

  // ---------------- Battery voltage input --------------------------
    // TODO: assign actual battery voltage divider/read pin.
    const int BATTERY_VOLTAGE_pin = 14;

  // ---------------- RFM9X radio pins -------------------------------
    #define RFM9X_G0_pin    9
    #define RFM9X_SCK_pin   13
    #define RFM9X_MISO_pin  12
    #define RFM9X_MOSI_pin  11
    #define RFM9X_CS_pin    10
    #define RFM9X_RST_pin   26
    #define RFM9X_PWR_pin   23

  // ---------------- Shared I2C bus LSM6DSOX + BME390 + DS1307 ------
    #define I2C_MAIN_SDA_pin 18
    #define I2C_MAIN_SCL_pin 19

    #define BMP390_I2C_ADDRESS 0x77

    #define BME680_I2C_ADDRESS 0x76
    //#define BME280_I2C_ADDRESS 0x77

  // ---------------- ADXL375 separate I2C bus ------------------------
    #define ADXL375_SDA_pin 17
    #define ADXL375_SCL_pin 16

  // ---------------- GPS SparkFun SAM-M8Q ----------------------------
    #define GPS_RX_pin   7   // receive GPS data from the pin where activity exists
    #define GPS_TX_pin   8
    #define GPS_PPS_pin  34

    // SoftwareSerial GPS_PORT(GPS_RX_pin, GPS_TX_pin); // RX, TX
    // TinyGPSPlus gps;

    #define GPS_PORT Serial2
    TinyGPSPlus gps;
    //SoftwareSerial GPS_SOFT_SERIAL(GPS_RX_pin, GPS_TX_pin);
    //#define GPS_PORT GPS_SOFT_SERIAL

  // ---------------- extra SDA, SCL (BME 680) ---------------------
    const int extra_SDA_pin = 25;
    const int extra_SCL_pin = 24;

// ============================ CONSTANTS ==========================

  // Valve connection check
    const float VALVE_CONNECTION_MIN_VOLTAGE = 0.5f;

  // Radio frequency from SYS_Alan_V.4.1
    float CONTROL_FREQ = 446.37f;
    const int RFM9X_TX_POWER_DBM = 23;

  // Time between component usages in microseconds
    const unsigned int RFM9x_rate    = 200000;
    const unsigned int Pres_rate     = 50000;
    const unsigned int LSM6DSOX_rate = 10000;
    const unsigned int ADXL375_rate  = 10000;
    const uint32_t BMP390_rate       = 10000;
    // const unsigned int BME680_rate   = 50000; delete
    // const unsigned int GPS_rate      = 200000; delete
    const unsigned int DS1307_rate   = 100000;
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

      // ---------------- Pressure sensor calibration tables ----------------
        // Input = sensor output voltage after divider / ADC conversion
        // Output = pressure in PSI
        //
        // New calibration from Calibration_for_press.xlsx
        //
        // Sensor mapping:
        // Pressure 1 / pin 41 = N2O / fuel tank
        // Pressure 3 / pin 15 = combustion chamber
        // Pressure 4 / pin 27 = CO2

        const int COMBUST_PRESSURE_TABLE_SIZE = 9;

        const float combust_voltage_table[COMBUST_PRESSURE_TABLE_SIZE] = {
          0.390f,
          0.490f,
          0.590f,
          0.660f,
          0.726f,
          0.892f,
          0.940f,
          1.005f,
          1.065f
        };

        const float combust_pressure_psi_table[COMBUST_PRESSURE_TABLE_SIZE] = {
          0.0f,
          200.0f,
          400.0f,
          500.0f,
          600.0f,
          800.0f,
          1000.0f,
          1100.0f,
          1200.0f
        };


        const int N2O_PRESSURE_TABLE_SIZE = 8;

        const float n2o_voltage_table[N2O_PRESSURE_TABLE_SIZE] = {
          0.372f,
          0.600f,
          0.765f,
          0.834f,
          1.230f,
          1.270f,
          1.274f,
          1.280f
        };

        const float n2o_pressure_psi_table[N2O_PRESSURE_TABLE_SIZE] = {
          0.0f,
          200.0f,
          300.0f,
          400.0f,
          700.0f,
          900.0f,
          1000.0f,
          1200.0f
        };


        const int CO2_PRESSURE_TABLE_SIZE = 14;

        const float co2_voltage_table[CO2_PRESSURE_TABLE_SIZE] = {
          0.260f,
          0.271f,
          0.274f,
          0.277f,
          0.282f,
          0.284f,
          0.287f,
          0.290f,
          0.294f,
          0.297f,
          0.300f,
          0.303f,
          0.600f,
          0.980f
        };

        const float co2_pressure_psi_table[CO2_PRESSURE_TABLE_SIZE] = {
          0.0f,
          20.0f,
          25.0f,
          30.0f,
          35.0f,
          40.0f,
          45.0f,
          50.0f,
          55.0f,
          60.0f,
          65.0f,
          70.0f,
          600.0f,
          1300.0f
        };

      const float SEA_LEVEL_PRESSURE_HPA = 1013.25f;

  // Old ignition-system reference values.
    // These are kept as building blocks, not as final flight-stage logic.
    const uint32_t MAIN_VALVE_OPEN_AFTER_START_MS  = 28000;

    uint32_t MAIN_VALVE_AUTO_CLOSE_DELAY_MS = 4100; // ~11000ft
    uint32_t MAX_ENGINE_ON_TIME_MS = 4100;
    uint32_t MIN_ENGINE_ON_TIME_MS = 3500; // ~9000ft
  
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

      const int ENGINE_MDOT_FALLBACK_TABLE_SIZE = 229;
      const float ENGINE_MDOT_FALLBACK_MAX_TIME_S = 11.400f;

      float fallback_engine_time_s = 0.0f;

      const float engine_mdot_fallback_time_s[ENGINE_MDOT_FALLBACK_TABLE_SIZE] = {
        0.000f, 0.050f, 0.100f, 0.150f, 0.200f, 0.250f, 0.300f, 0.350f,
        0.400f, 0.450f, 0.500f, 0.550f, 0.600f, 0.650f, 0.700f, 0.750f,
        0.800f, 0.850f, 0.900f, 0.950f, 1.000f, 1.050f, 1.100f, 1.150f,
        1.200f, 1.250f, 1.300f, 1.350f, 1.400f, 1.450f, 1.500f, 1.550f,
        1.600f, 1.650f, 1.700f, 1.750f, 1.800f, 1.850f, 1.900f, 1.950f,
        2.000f, 2.050f, 2.100f, 2.150f, 2.200f, 2.250f, 2.300f, 2.350f,
        2.400f, 2.450f, 2.500f, 2.550f, 2.600f, 2.650f, 2.700f, 2.750f,
        2.800f, 2.850f, 2.900f, 2.950f, 3.000f, 3.050f, 3.100f, 3.150f,
        3.200f, 3.250f, 3.300f, 3.350f, 3.400f, 3.450f, 3.500f, 3.550f,
        3.600f, 3.650f, 3.700f, 3.750f, 3.800f, 3.850f, 3.900f, 3.950f,
        4.000f, 4.050f, 4.100f, 4.150f, 4.200f, 4.250f, 4.300f, 4.350f,
        4.400f, 4.450f, 4.500f, 4.550f, 4.600f, 4.650f, 4.700f, 4.750f,
        4.800f, 4.850f, 4.900f, 4.950f, 5.000f, 5.050f, 5.100f, 5.150f,
        5.200f, 5.250f, 5.300f, 5.350f, 5.400f, 5.450f, 5.500f, 5.550f,
        5.600f, 5.650f, 5.700f, 5.750f, 5.800f, 5.850f, 5.900f, 5.950f,
        6.000f, 6.050f, 6.100f, 6.150f, 6.200f, 6.250f, 6.300f, 6.350f,
        6.400f, 6.450f, 6.500f, 6.550f, 6.600f, 6.650f, 6.700f, 6.750f,
        6.800f, 6.850f, 6.900f, 6.950f, 7.000f, 7.050f, 7.100f, 7.150f,
        7.200f, 7.250f, 7.300f, 7.350f, 7.400f, 7.450f, 7.500f, 7.550f,
        7.600f, 7.650f, 7.700f, 7.750f, 7.800f, 7.850f, 7.900f, 7.950f,
        8.000f, 8.050f, 8.100f, 8.150f, 8.200f, 8.250f, 8.300f, 8.350f,
        8.400f, 8.450f, 8.500f, 8.550f, 8.600f, 8.650f, 8.700f, 8.750f,
        8.800f, 8.850f, 8.900f, 8.950f, 9.000f, 9.050f, 9.100f, 9.150f,
        9.200f, 9.250f, 9.300f, 9.350f, 9.400f, 9.450f, 9.500f, 9.550f,
        9.600f, 9.650f, 9.700f, 9.750f, 9.800f, 9.850f, 9.900f, 9.950f,
        10.000f, 10.050f, 10.100f, 10.150f, 10.200f, 10.250f, 10.300f, 10.350f,
        10.400f, 10.450f, 10.500f, 10.550f, 10.600f, 10.650f, 10.700f, 10.750f,
        10.800f, 10.850f, 10.900f, 10.950f, 11.000f, 11.050f, 11.100f, 11.150f,
        11.200f, 11.250f, 11.300f, 11.350f, 11.400f
      };

      const float engine_mdot_fallback_kg_s[ENGINE_MDOT_FALLBACK_TABLE_SIZE] = {
        0.000000f, 1.466172f, 1.740965f, 1.753105f, 1.850220f, 1.896571f, 1.801662f, 1.842495f,
        1.718893f, 1.707858f, 1.708961f, 1.740965f, 1.685786f, 1.680268f, 1.702339f, 1.657093f,
        1.662610f, 1.615156f, 1.580945f, 1.637228f, 1.619570f, 1.567702f, 1.605224f, 1.572116f,
        1.555562f, 1.578738f, 1.578738f, 1.575427f, 1.534594f, 1.525765f, 1.502590f, 1.504797f,
        1.545630f, 1.492658f, 1.498176f, 1.510315f, 1.508108f, 1.467275f, 1.465068f, 1.475000f,
        1.461757f, 1.477208f, 1.451825f, 1.483829f, 1.412096f, 1.426442f, 1.423132f, 1.447411f,
        1.409889f, 1.419821f, 1.422028f, 1.408785f, 1.361331f, 1.376781f, 1.358020f, 1.306151f,
        1.342570f, 1.320498f, 1.324913f, 1.266422f, 1.255386f, 1.324913f, 1.313877f, 1.267526f,
        1.273044f, 1.271940f, 1.317187f, 1.301737f, 1.290701f, 1.267526f, 1.260905f, 1.256490f,
        1.239936f, 1.252076f, 1.190275f, 1.245454f, 1.237729f, 1.223382f, 1.232211f, 1.263111f,
        1.247661f, 1.224486f, 1.218968f, 1.239936f, 1.200207f, 1.205725f, 1.250972f, 1.203518f,
        1.213450f, 1.185860f, 1.199103f, 1.175928f, 1.200207f, 1.193585f, 1.165996f, 1.199103f,
        1.178135f, 1.198000f, 1.125163f, 1.124059f, 1.135095f, 1.157167f, 1.160478f, 1.121852f,
        1.098677f, 1.000457f, 0.937553f, 0.886788f, 0.776429f, 0.757668f, 0.690349f, 0.655035f,
        0.597648f, 0.570058f, 0.546883f, 0.514879f, 0.518189f, 0.501636f, 0.482875f, 0.479564f,
        0.454182f, 0.446457f, 0.432110f, 0.411142f, 0.405624f, 0.390174f, 0.385759f, 0.340512f,
        0.364791f, 0.348237f, 0.340512f, 0.306301f, 0.325062f, 0.316233f, 0.298575f, 0.297472f,
        0.291954f, 0.282022f, 0.270986f, 0.269883f, 0.259950f, 0.248914f, 0.240086f, 0.241189f,
        0.237878f, 0.230153f, 0.215807f, 0.220221f, 0.212496f, 0.198149f, 0.188217f, 0.198149f,
        0.190424f, 0.193735f, 0.182699f, 0.173870f, 0.172767f, 0.172767f, 0.167249f, 0.166145f,
        0.156213f, 0.154006f, 0.148488f, 0.144073f, 0.146281f, 0.115380f, 0.124209f, 0.126416f,
        0.113173f, 0.117587f, 0.117587f, 0.122001f, 0.116484f, 0.110966f, 0.094412f, 0.096619f,
        0.095515f, 0.092205f, 0.088894f, 0.076755f, 0.081169f, 0.066822f, 0.060201f, 0.040336f,
        0.038129f, 0.061304f, 0.059097f, 0.030404f, 0.045854f, 0.053579f, 0.049165f, 0.041440f,
        0.046958f, 0.043647f, 0.040336f, 0.038129f, 0.041440f, 0.035922f, 0.028197f, 0.033715f,
        0.037025f, 0.030404f, 0.008332f, 0.027093f, 0.028197f, 0.022679f, 0.005021f, 0.025990f,
        0.022679f, 0.018264f, 0.019368f, 0.017161f, 0.017161f, 0.012747f, 0.016057f, 0.014954f,
        0.010539f, 0.014954f, 0.011643f, 0.008332f, 0.010539f, 0.000000f, 0.007229f, 0.006125f,
        0.007229f, 0.000000f, 0.003918f, 0.001710f, 0.005021f
      };

    float manual_zero_alt = 0.0f;
    int desired_apogee = 10000; // in ft need to convert to meters
    float desired_apogee_m = desired_apogee * 0.3048f;
    float initial_SL_altitude_m = 0.0f;
    float real_desired_apogee_m = desired_apogee_m;

    float no_fuel_rocket_mass = 36.287;
    float initial_fuel_mass = 3.6f + 8.528f;
    float fuel_mass = initial_fuel_mass;
    // float distance_from_PCB_to_stratalogger_along_the_rocket = 10; // in ft delete

// ============================ OBJECTS ============================

  ADC *adc = new ADC();
  File Black_Box;

  char black_box_filename[13] = "BLACK000.CSV";

  RH_RF95 rf95(RFM9X_CS_pin, RFM9X_G0_pin);

  Adafruit_LSM6DSOX lsm6dsox;
  Adafruit_ADXL375 adxl375 = Adafruit_ADXL375(12345, &Wire1);
  Adafruit_BME680 BME680(&Wire2);
  Adafruit_BMP3XX BMP390;
  //Adafruit_BME280 BME280(&Wire2);
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
    {4},                   // 0x06 State array
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
    bool BMP390_FAIL   = false;
    //bool BME280_FAIL   = false;
    bool GPS_FAIL      = false;
    bool DS1307_FAIL   = false;
    bool DISPLAY_FAIL  = true;   // display system is not defined yet

  // ---------------- Old ignition-system state -----------------------
    bool avionics_armed = false;
    bool ignition_sequence_started = false;
    bool main_valve_can_close_by_radio = true;
    bool main_valve_ignition_hold_active = false;
    uint32_t main_valve_ignition_start_time_ms = 0;

    const int8_t IGNITION_COUNTDOWN_START_S = -48;
    const int8_t ARM_COUNTDOWN_START_S = 0;

    bool pre_launch_calibration_done = false;

    // ARM stage sequence state
      bool arm_sequence_active = false;
      bool arm_sequence_started_once = false;
      uint32_t arm_sequence_start_time_ms = 0;
      int8_t countdown = IGNITION_COUNTDOWN_START_S;
      int8_t arm_countdown = ARM_COUNTDOWN_START_S;

      bool arm_sequence_complete = false;

    // Time from ARM stage start to launch ignition sequence stage
    // This follows old ignition timing idea.
      const uint32_t ARM_SEQUENCE_DURATION_MS = 10;  

  // ---------------- Pressure / battery outputs ----------------------
    float pressure_1_output = 0.0f;
    float pressure_2_output = 0.0f;
    float pressure_3_output = 0.0f;
    float pressure_4_output = 0.0f;
    float battery_voltage_output = 0.0f;

    uint32_t valve_connection_last_time = 0;
    const uint32_t valve_connection_rate_us = 100000;  // 10 Hz 

    uint32_t battery_last_time = 0;
    const uint32_t battery_rate_us = 200000;  // 5 Hz

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
    float BME680_temp = 0.0f; // delete
    // float BME680_humidity = 0.0f; delete
    float BME680_pressure_hpa = 0.0f;
    float BME680_gas_kohm = 0.0f;
    float BME680_altitude_m = 0.0f;

    bool BME680_new_data = false;
    // uint32_t BME680_last_new_data_us = 0; delete
    // float BME680_update_hz = 0.0f; delete

    float BME_filtered_pressure_hpa = 0.0f;
    float BME_pressure_threshold_hpa = 0.0f;
    float BME_calibration_duration_s = 6.0f;
    bool BME_filter_started = false;

    // float BME280_temp = 0.0f;
    // float BME280_pressure_hpa = 0.0f;
    // float BME280_humidity = 0.0f;
    // float BME280_altitude_m = 0.0f;
    // bool BME280_new_data = false;

  // ---------------- BMP390 outputs ----------------------------------
    float BMP390_temp = 0.0f;
    float BMP390_pressure_hpa = 0.0f;
    float BMP390_altitude_m = 0.0f;
    float old_BMP_reading = 0.0f;

    bool BMP390_new_data = false;

    bool bmp390_busy = false;
    uint32_t bmp390_ready_time_us = 0;
    const uint32_t BMP390_READ_DELAY_US = 5000;  // 5 ms for 200 Hz target

    uint32_t loop_counter = 0;
    uint32_t bmp390_new_counter = 0;
    uint32_t freq_last_ms = 0;

    float code_loop_hz = 0.0f;
    float bmp390_read_hz = 0.0f;

    void update_frequency_counters() {
      uint32_t now_ms = millis();

      if (now_ms - freq_last_ms >= 1000) {
        float dt_s = (now_ms - freq_last_ms) / 1000.0f;

        code_loop_hz = loop_counter / dt_s;
        bmp390_read_hz = bmp390_new_counter / dt_s;

        loop_counter = 0;
        bmp390_new_counter = 0;
        freq_last_ms = now_ms;
      }
    }

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
    uint32_t BMP390_last_time = 0;
    // unsigned int BME680_last_time   = 0; delete
    // unsigned int GPS_last_time      = 0; delete
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

      uint32_t negative_vertical_speed_start_ms = 0;
      bool negative_vertical_speed_timer_active = false;

      uint32_t APOGEE_NEGATIVE_VELOCITY_CONFIRM_TIME_MS = 500;

      // Relief valve timing after engine cutoff / Stage 5 entry
      const uint32_t MIN_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS = 1000;   // do not open before this
      const uint32_t MAX_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS = 40000;  // force open after this
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
      // float PCB_gyro_mag = 0.0f; delete

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

    // uint32_t oracle_last_update_us = 0; delete
    float oracle_delta_time_s = 0.0f;

    // module red flags
      bool GPS_makes_sense = true;
      bool BME_makes_sense = true;
      bool ADXL_makes_sense = true;
      bool LSM_makes_sense = true;

      bool fuel_tank_pressure_makes_sense = true;
      bool combustion_chamber_pressure_makes_sense = true;

      // ---------------- Sensor MakeSense thresholds ----------------

        // BME / barometer
        float BME_ALTITUDE_MIN_M = 0.0f;
        float BME_ALTITUDE_MAX_M = 10000.0f;
        float BME_ALTITUDE_MAX_RATE_MPS = 400.0f;
        uint32_t BME_MAKES_SENSE_FAIL_TIME_MS = 1000;

        // GPS
        float GPS_ALTITUDE_MIN_M = 0.0f;
        float GPS_ALTITUDE_MAX_M = 10000.0f;
        float GPS_MAX_ALTITUDE_RATE_MPS = 300.0f;
        uint8_t GPS_MIN_SIV = 4;
        uint32_t GPS_MAKES_SENSE_FAIL_TIME_MS = 1000;

        // ---------------- GPS allowed mission area ----------------
        // GPS is considered valid only inside this coordinate rectangle.
        // Set launch/test location.

        double GPS_ALLOWED_LAT_MIN = 37.9000000;
        double GPS_ALLOWED_LAT_MAX = 38.1000000;

        double GPS_ALLOWED_LON_MIN = -90.1000000;
        double GPS_ALLOWED_LON_MAX = -89.9000000;

        bool GPS_USE_ALLOWED_AREA_CHECK = true;

        // LSM6DSOX
        float LSM_ACCEL_MAG_MAX_MPS2 = 250.0f;
        float LSM_GYRO_MAG_MAX_RAD_S = 10.0f;
        uint32_t LSM_MAKES_SENSE_FAIL_TIME_MS = 500;

        // ADXL375
        float ADXL_ACCEL_MAG_MAX_MPS2 = 2500.0f;
        uint32_t ADXL_MAKES_SENSE_FAIL_TIME_MS = 500;

        // Fuel tank pressure / N2O tank
        float FUEL_TANK_PRESSURE_MIN_PSI = 0.0f;
        float FUEL_TANK_PRESSURE_MAX_PSI = 1500.0f;
        float FUEL_TANK_PRESSURE_MAX_RATE_PSI_S = 500.0f;
        uint32_t FUEL_TANK_PRESSURE_FAIL_TIME_MS = 1000;

        // Combustion chamber pressure
        float CHAMBER_PRESSURE_MIN_PSI = 0.0f;
        float CHAMBER_PRESSURE_MAX_PSI = 2000.0f;
        float CHAMBER_PRESSURE_MAX_RATE_PSI_S = 500.0f;
        uint32_t CHAMBER_PRESSURE_FAIL_TIME_MS = 500;


        // ---------------- Sensor MakeSense bad timers ----------------
          float manual_rocket_tilt_angle_deg = 0.0f;
          float manual_rocket_tilt_angle_rad = manual_rocket_tilt_angle_deg * 0.017453f;

          bool rocket_tilt_using_LSM = true;
          bool rocket_tilt_using_manual_set = false;

          bool rocket_mass_using_pressure_flow = true;
          bool altitude_kalman_using_BME = true;

          uint32_t BME_bad_start_ms = 0;
          uint32_t GPS_bad_start_ms = 0;
          uint32_t LSM_bad_start_ms = 0;
          uint32_t ADXL_bad_start_ms = 0;
          uint32_t fuel_tank_pressure_bad_start_ms = 0;
          uint32_t combustion_chamber_pressure_bad_start_ms = 0;


        // ---------------- Previous values for rate checks ----------------

          float previous_BME_altitude_for_sense_m = 0.0f;
          uint32_t previous_BME_sense_time_ms = 0;
          bool previous_BME_sense_valid = false;

          float previous_GPS_altitude_for_sense_m = 0.0f;
          uint32_t previous_GPS_sense_time_ms = 0;
          bool previous_GPS_sense_valid = false;

          float previous_fuel_tank_pressure_for_sense_psi = 0.0f;
          uint32_t previous_fuel_tank_pressure_sense_time_ms = 0;
          bool previous_fuel_tank_pressure_sense_valid = false;

          float previous_chamber_pressure_for_sense_psi = 0.0f;
          uint32_t previous_chamber_pressure_sense_time_ms = 0;
          bool previous_chamber_pressure_sense_valid = false;
      
    


    

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
      // int stratologger_apogee_prediction = 0; delete

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
      float current_max_altitude_reached_m = 0.0f;

  // OMG NEW DEBUG ****
    uint32_t stage_loop_time_us = 0;


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
    //void init_BME280();
    // void read_BME680();

    // void start_BME680_read();
    // void finish_BME680_read();
    
    void init_BMP390();
    void read_BMP390();

    void start_BMP390_read();
    void finish_BMP390_read();

    void calibrate_primary_baro_pressure_threshold();
    void apply_primary_baro_pressure_deadband();

    

    void init_GPS();
    void read_GPS();
    void raw_GPS_UART_test();

    void init_DS1307();
    void read_DS1307();

    void update_sensor_makes_sense();

    void update_makes_sense_flag(bool value_is_bad,
                                bool& makes_sense_flag,
                                uint32_t& bad_start_ms,
                                uint32_t fail_time_ms);

    bool value_outside_range_or_not_finite(float value, float min_value, float max_value);
    bool value_rate_too_high(float current_value,
                            float previous_value,
                            uint32_t current_time_ms,
                            uint32_t previous_time_ms,
                            bool previous_valid,
                            float max_rate_per_s);

    void print_sensor_makes_sense_status();

    bool gps_inside_allowed_area(double lat, double lon);

   

  // ---------------- PCB output / control section -------------------
    void VALVE_open(Valve& valve);
    void VALVE_close(Valve& valve);
    
    void close_all_valves();
    void update_main_valve_ignition_hold();
    void update_debug_display();
    void in_every_loop();

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
    void valve_connection_check_all();
    bool valve_connection_check(uint8_t valve_id);
    // bool valve_connection_check(uint8_t valve_id); delete
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

    void LSM_to_PCB_switch_over(); // LSM orientation to rocket orientation
    void ADXL_to_PCB_swithch_over();
    void current_rocket_mass();

    float lookup_fallback_oxidizer_mdot_from_engine_time(float engine_time_s);
    void local_PCB_direction_determination();

    void calculate_rocket_tilt_from_orientation();

    void calculate_rocket_tilt_from_manual_set();
    void set_manual_rocket_tilt_angle_deg(float angle_deg);

    void global_PCB_vel_vertor();
    void global_PCB_location_vector();

    void current_rocket_mass_from_time_function();

    // void calculate_rocket_tilt_from_fallback(); delete

    void update_altitude_kalman_filter_from_fallback();



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

    void print_primary_baro_pressure_filter_status();
    
    void print_gravity_anchor_status();
    void print_gyro_orientation_status();

    void debug_laptop();

// ======= NEW RADIO CODE OMG PLEASE WORK ITS FUCKING 5 AM =========
  // ---------------- New compact radio protocol ----------------
    bool override_mode_is_valid();
    int32_t compute_radio_timeline_ms();

    const uint8_t MSG_CLASS_TELEMETRY = 0x02;

    const uint8_t TEL_FAST_STATUS = 0x01;
    const uint8_t TEL_GPS         = 0x02;
    const uint8_t TEL_MODULES     = 0x03;
    const uint8_t TEL_COMMAND_ACK = 0x04;

    const uint8_t MSG_CLASS_COMMAND = 0x01;

    const uint8_t CMD_ABORT         = 0x01;
    const uint8_t CMD_ARM           = 0x02;
    const uint8_t CMD_IGNITION      = 0x03;

    const uint8_t CMD_OPEN_MAIN     = 0x04;
    const uint8_t CMD_CLOSE_MAIN    = 0x05;
    const uint8_t CMD_OPEN_DUMP     = 0x06;
    const uint8_t CMD_CLOSE_DUMP    = 0x07;
    const uint8_t CMD_OPEN_RELIEF   = 0x08;
    const uint8_t CMD_CLOSE_RELIEF  = 0x09;

    const uint8_t CMD_GET_STATUS    = 0x0A;
    const uint8_t CMD_GET_GPS       = 0x0B;

    const uint8_t CMD_GRAVITY       = 0x0C;
    const uint8_t CMD_BME_CAL       = 0x0D;

    const uint8_t CMD_OVERRIDE      = 0x0E;
    const uint8_t CMD_EXIT_OVERRIDE = 0x0F;

    const uint8_t CMD_SET_TIME      = 0x10;
    const uint8_t CMD_SET_STAGE     = 0x11;

    const uint8_t CMD_SET_ENGINE_TIMING = 0x12;

    uint8_t last_command_id_executed = 0;

  // ------------- helper functions declarations

    void pack_u8(uint8_t* b, uint8_t& i, uint8_t v);
    void pack_i8(uint8_t* b, uint8_t& i, int8_t v);
    void pack_u16(uint8_t* b, uint8_t& i, uint16_t v);
    void pack_i16(uint8_t* b, uint8_t& i, int16_t v);
    void pack_u32(uint8_t* b, uint8_t& i, uint32_t v);
    void pack_i32(uint8_t* b, uint8_t& i, int32_t v);

    uint8_t build_fast_flags1();
    uint8_t build_fast_flags2();

    void send_fast_telemetry_packet();
    void update_radio_telemetry();

  // ------------- helper functions

    void pack_u8(uint8_t* b, uint8_t& i, uint8_t v) {
      b[i++] = v;
    }

    void pack_i8(uint8_t* b, uint8_t& i, int8_t v) {
      b[i++] = (uint8_t)v;
    }

    void pack_u16(uint8_t* b, uint8_t& i, uint16_t v) {
      b[i++] = (uint8_t)(v & 0xFF);
      b[i++] = (uint8_t)((v >> 8) & 0xFF);
    }

    void pack_i16(uint8_t* b, uint8_t& i, int16_t v) {
      pack_u16(b, i, (uint16_t)v);
    }

    void pack_u32(uint8_t* b, uint8_t& i, uint32_t v) {
      b[i++] = (uint8_t)(v & 0xFF);
      b[i++] = (uint8_t)((v >> 8) & 0xFF);
      b[i++] = (uint8_t)((v >> 16) & 0xFF);
      b[i++] = (uint8_t)((v >> 24) & 0xFF);
    }

    void pack_i32(uint8_t* b, uint8_t& i, int32_t v) {
      pack_u32(b, i, (uint32_t)v);
    }

    uint8_t build_fast_flags1() {
      uint8_t flags = 0;

      if (MAIN_VALVE.state)            flags |= (1 << 0);
      if (RELIEF_VALVE.state)          flags |= (1 << 1);
      if (DUMP_VALVE.state)            flags |= (1 << 2);
      if (avionics_armed)              flags |= (1 << 3);
      if (no_return_flight_started)    flags |= (1 << 4);
      if (apogee_detected)             flags |= (1 << 5);

      if (fuel_tank_pressure_makes_sense &&
          combustion_chamber_pressure_makes_sense) {
        flags |= (1 << 6);
      }

      if (override_mode_is_valid()) flags |= (1 << 7);

      return flags;
    }

    uint8_t build_fast_flags2() {
      uint8_t flags = 0;

      if (!SD_FAIL)       flags |= (1 << 0);
      if (!BMP390_FAIL)   flags |= (1 << 1);
      if (!LSM6DSOX_FAIL) flags |= (1 << 2);
      if (!ADXL375_FAIL)  flags |= (1 << 3);
      if (!GPS_FAIL)      flags |= (1 << 4);
      if (!DS1307_FAIL)   flags |= (1 << 5);

      if (MAIN_VALVE.connection_ok)   flags |= (1 << 6);
      if (RELIEF_VALVE.connection_ok) flags |= (1 << 7);

      return flags;
    }



  // ---------------- tel send
      uint8_t build_module_flags();

    uint8_t build_makes_sense_flags();
    uint8_t build_valve_connection_flags();
    uint8_t build_state_flags_for_status();

    void print_packet_hex(const char* tag, uint8_t* buf, uint8_t len) {
  Serial.print(tag);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" data=");

  for (uint8_t k = 0; k < len; k++) {
    if (buf[k] < 16) Serial.print("0");
    Serial.print(buf[k], HEX);
    if (k < len - 1) Serial.print(" ");
  }

  Serial.println();
}

    void send_fast_telemetry_packet() {
      float current_altitude_above_launch_m = kalman_altitude_m - initial_SL_altitude_m;

      if (current_altitude_above_launch_m < 0.0f) {
        current_altitude_above_launch_m = 0.0f;
      }

      if (current_altitude_above_launch_m > current_max_altitude_reached_m) {
        current_max_altitude_reached_m = current_altitude_above_launch_m;
      }

      uint8_t buf[43];
      uint8_t i = 0;

      int32_t altitude_cm =
        (int32_t)(kalman_altitude_m * 100.0f);

      int16_t vertical_velocity_cm_s =
        (int16_t)(kalman_vertical_speed_mps * 100.0f);

      int16_t tilt_deg_x100 =
        (int16_t)(rocket_tilt_angle_deg * 100.0f);

      int32_t predicted_apogee_cm =
        (int32_t)(predicted_apogee_m * 100.0f);

      uint16_t battery_mv =
        (uint16_t)(battery_voltage_output * 1000.0f);

      uint16_t fuel_tank_pressure_psi_x10 =
        (uint16_t)(pressure_1_output * 10.0f);

      uint16_t chamber_pressure_psi_x10 =
        (uint16_t)(pressure_3_output * 10.0f);

      uint16_t co2_pressure_psi_x10 =
        (uint16_t)(pressure_4_output * 10.0f);

      uint16_t current_max_altitude_10m =
        (uint16_t)(current_max_altitude_reached_m / 10.0f + 0.5f); // Decode logic на Nano/HTML: current_max_altitude_m = current_max_altitude_10m * 10

      if (current_max_altitude_10m > 512) {
        current_max_altitude_10m = 512;
      }

      uint8_t module_status_flags = build_module_flags();

      uint8_t makes_sense_flags = build_makes_sense_flags();

      int32_t timeline_ms = compute_radio_timeline_ms();

      int8_t radio_db = 0;

      // lastRssi() is only meaningful after receiving a packet.
      // Still useful as control-link RSSI estimate.
      radio_db = (int8_t)rf95.lastRssi();

      pack_u8(buf, i, MSG_CLASS_TELEMETRY);
      pack_u8(buf, i, TEL_FAST_STATUS);

      pack_i32(buf, i, altitude_cm);
      pack_i16(buf, i, vertical_velocity_cm_s);
      pack_i16(buf, i, tilt_deg_x100);
      pack_i32(buf, i, predicted_apogee_cm);

      pack_u8(buf, i, stage);
      pack_u8(buf, i, build_fast_flags1());
      pack_u8(buf, i, build_fast_flags2());

      pack_u16(buf, i, battery_mv);

      pack_u16(buf, i, fuel_tank_pressure_psi_x10);
      pack_u16(buf, i, chamber_pressure_psi_x10);
      pack_u16(buf, i, co2_pressure_psi_x10);

      pack_u16(buf, i, current_max_altitude_10m);
      pack_u8(buf, i, module_status_flags);
      pack_u8(buf, i, makes_sense_flags);
      
      pack_i32(buf, i, timeline_ms);

      pack_i8(buf, i, radio_db);
      pack_u16(buf, i, (uint16_t)commands_received);
      pack_u8(buf, i, last_command_id_executed);

      buf[i] = radio_checksum(buf, i);
      i++;

      rf95.send(buf, i);
      rf95.waitPacketSent();
    }

    void update_radio_telemetry() {
      if (RFM9X_FAIL || !TX_enabled) {
        return;
      }

      if ((micros() - RFM9x_last_time) < RFM9x_rate) {
        return;
      }

      send_fast_telemetry_packet();

      RFM9x_last_time = micros();
    }

    int32_t compute_radio_timeline_ms() {
      uint32_t now_ms = millis();

      if (stage == STAGE_ARM) {
        if (arm_sequence_active && arm_sequence_start_time_ms > 0) {
          uint32_t elapsed_ms = now_ms - arm_sequence_start_time_ms;

          if (elapsed_ms < ARM_SEQUENCE_DURATION_MS) {
            return -(int32_t)(ARM_SEQUENCE_DURATION_MS - elapsed_ms);
          }

          return 0;
        }

        if (arm_sequence_complete) {
          return 0;
        }

        return 0;
      }

      if (stage == STAGE_LAUNCH_IGNITION_SEQUENCE) {
        if (main_valve_ignition_hold_active &&
            main_valve_ignition_start_time_ms > 0) {

          uint32_t elapsed_ms = now_ms - main_valve_ignition_start_time_ms;

          return (int32_t)elapsed_ms + ((int32_t)IGNITION_COUNTDOWN_START_S * 1000);
        }

        return 0;
      }

      if (stage == STAGE_ENGINE_WORKING ||
          stage == STAGE_DATA_GATHERING_FLIGHT) {

        if (main_valve_ignition_start_time_ms > 0) {
          uint32_t elapsed_ms = now_ms - main_valve_ignition_start_time_ms;
          return (int32_t)elapsed_ms + ((int32_t)IGNITION_COUNTDOWN_START_S * 1000);
        }

        return 0;
      }

      return 0;
    }

  // ----------------- for receiving
    bool parse_new_control_packet(uint8_t* buf, uint8_t len);
    bool verify_packet_checksum(uint8_t* buf, uint8_t len);

    void send_command_ack_packet(uint8_t command_id, uint8_t accepted, uint8_t reject_reason);

    uint8_t execute_new_radio_command(uint8_t command_id, uint8_t* payload, uint8_t payload_len);

    const uint8_t CMD_ACCEPTED = 0;

    const uint8_t CMD_REJECT_UNKNOWN           = 1;
    const uint8_t CMD_REJECT_BAD_CHECKSUM      = 2;
    const uint8_t CMD_REJECT_OVERRIDE_REQUIRED = 3;
    const uint8_t CMD_REJECT_BAD_PASSWORD      = 4;
    const uint8_t CMD_REJECT_BAD_STAGE         = 5;
    const uint8_t CMD_REJECT_NOT_ARMED         = 6;
    const uint8_t CMD_REJECT_ARM_NOT_COMPLETE  = 7;
    const uint8_t CMD_REJECT_BAD_PAYLOAD       = 8;

    uint8_t last_command_accepted = 0;
    uint8_t last_command_reject_reason = 0;

    bool verify_packet_checksum(uint8_t* buf, uint8_t len) {
      if (len < 3) {
        return false;
      }

      uint8_t expected = radio_checksum(buf, len - 1);
      uint8_t received = buf[len - 1];

      return expected == received;
    }

    void send_command_ack_packet(uint8_t command_id, uint8_t accepted, uint8_t reject_reason) {
      if (RFM9X_FAIL) {
        return;
      }

      uint8_t buf[16];
      uint8_t i = 0;

      pack_u8(buf, i, MSG_CLASS_TELEMETRY);
      pack_u8(buf, i, TEL_COMMAND_ACK);

      pack_u16(buf, i, (uint16_t)commands_received);
      pack_u8(buf, i, command_id);
      pack_u8(buf, i, accepted);
      pack_u8(buf, i, reject_reason);

      buf[i] = radio_checksum(buf, i);
      i++;

      rf95.send(buf, i);
      rf95.waitPacketSent();
    }

    bool parse_new_control_packet(uint8_t* buf, uint8_t len) {
      if (len < 3) {
        return false;
      }

      if (buf[0] != MSG_CLASS_COMMAND) {
        return false;
      }

      uint8_t command_id = buf[1];

      if (!verify_packet_checksum(buf, len)) {
        last_command_id_executed = command_id;
        last_command_accepted = 0;
        last_command_reject_reason = CMD_REJECT_BAD_CHECKSUM;

        send_command_ack_packet(command_id, 0, CMD_REJECT_BAD_CHECKSUM);
        return true;
      }

      uint8_t* payload = buf + 2;
      uint8_t payload_len = len - 3;

      uint8_t reject_reason =
        execute_new_radio_command(command_id, payload, payload_len);

      uint8_t accepted = (reject_reason == CMD_ACCEPTED) ? 1 : 0;

      last_command_id_executed = command_id;
      last_command_accepted = accepted;
      last_command_reject_reason = reject_reason;

      send_command_ack_packet(command_id, accepted, reject_reason);

      return true;
    }

    void read_RFM() {
      if (RFM9X_FAIL) {
        return;
      }

      if (!rf95.available()) {
        return;
      }

      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (!rf95.recv(buf, &len)) {
        return;
      }

      // New compact command protocol first.
      if (parse_new_control_packet(buf, len)) {
        return;
      }

      // Old compatibility parser.
      // Keep this for now until old assembler/parser is deleted.
      message_parser(buf);

      if (recv_ready) {
        bool command_accepted = execute_radio_message(buf[0], buf[1]);

        if (command_accepted && buf[0] == 0x01) {
          send_ACK_packet(1);
        }

        print_local_status();
        recv_ready = 0;
      }
    }
  // statuses send to CC by request

    void send_gps_response_packet();
    void send_module_status_packet();




    uint8_t build_module_flags() {
      uint8_t flags = 0;

      if (!RFM9X_FAIL)    flags |= (1 << 0);
      if (!SD_FAIL)       flags |= (1 << 1);
      if (!LSM6DSOX_FAIL) flags |= (1 << 2);
      if (!ADXL375_FAIL)  flags |= (1 << 3);
      if (!BMP390_FAIL)   flags |= (1 << 4);
      if (!GPS_FAIL)      flags |= (1 << 5);
      if (!DS1307_FAIL)   flags |= (1 << 6);

      return flags;
    }

    uint8_t build_makes_sense_flags() {
      uint8_t flags = 0;

      if (GPS_makes_sense)                         flags |= (1 << 0);
      if (BME_makes_sense)                         flags |= (1 << 1);
      if (ADXL_makes_sense)                        flags |= (1 << 2);
      if (LSM_makes_sense)                         flags |= (1 << 3);
      if (fuel_tank_pressure_makes_sense)          flags |= (1 << 4);
      if (combustion_chamber_pressure_makes_sense) flags |= (1 << 5);

      return flags;
    }

    uint8_t build_valve_connection_flags() {
      uint8_t flags = 0;

      if (MAIN_VALVE.state)          flags |= (1 << 0);
      if (RELIEF_VALVE.state)        flags |= (1 << 1);
      if (DUMP_VALVE.state)          flags |= (1 << 2);

      if (MAIN_VALVE.connection_ok)  flags |= (1 << 3);
      if (RELIEF_VALVE.connection_ok)flags |= (1 << 4);
      if (DUMP_VALVE.connection_ok)  flags |= (1 << 5);

      return flags;
    }

    uint8_t build_state_flags_for_status() {
      uint8_t flags = 0;

      if (avionics_armed)           flags |= (1 << 0);
      if (arm_sequence_active)      flags |= (1 << 1);
      if (arm_sequence_complete)    flags |= (1 << 2);
      if (ignition_sequence_started)flags |= (1 << 3);
      if (no_return_flight_started) flags |= (1 << 4);
      if (apogee_detected)          flags |= (1 << 5);
      if (TX_enabled)               flags |= (1 << 6);
      if (override_mode_is_valid()) flags |= (1 << 7);

      return flags;
    }

    void send_gps_response_packet() {
      if (RFM9X_FAIL) {
        return;
      }
      if (!DS1307_FAIL) {
        rtc.refresh();
        rtc_year   = rtc.year();
        rtc_month  = rtc.month();
        rtc_day    = rtc.day();
        rtc_hour   = rtc.hour();
        rtc_minute = rtc.minute();
        rtc_second = rtc.second();
      }

      uint8_t buf[43];
      uint8_t i = 0;
      

      int32_t lat_e7 = (int32_t)(gps_lat * 10000000.0);
      int32_t lon_e7 = (int32_t)(gps_lon * 10000000.0);
      int32_t alt_cm = (int32_t)(gps_alt_m * 100.0f);

      pack_u8(buf, i, MSG_CLASS_TELEMETRY);
      pack_u8(buf, i, TEL_GPS);

      pack_i32(buf, i, lat_e7);
      pack_i32(buf, i, lon_e7);
      pack_i32(buf, i, alt_cm);

      pack_u8(buf, i, gps_siv);
      pack_u8(buf, i, gps_pps_state ? 1 : 0);

      pack_u16(buf, i, (uint16_t)commands_received);
      pack_u8(buf, i, last_command_id_executed);

      // Extra DS1307 time payload appended to the end of TEL_GPS.
      // Order: yy, month, day, hour, minute, second. One byte each.
      // Old GPS payload was 17 bytes; new GPS payload is 23 bytes.

      pack_u8(buf, i, rtc_year);
      pack_u8(buf, i, rtc_month);
      pack_u8(buf, i, rtc_day);
      pack_u8(buf, i, rtc_hour);
      pack_u8(buf, i, rtc_minute);
      pack_u8(buf, i, rtc_second);

      // Engine timings sent in 10 ms units.
      // Example: 4100 ms -> 410, 3500 ms -> 350.
      uint16_t main_auto_close_10ms =
        (uint16_t)(MAIN_VALVE_AUTO_CLOSE_DELAY_MS / 10);

      uint16_t min_engine_on_10ms =
        (uint16_t)(MIN_ENGINE_ON_TIME_MS / 10);

      pack_u16(buf, i, main_auto_close_10ms);
      pack_u16(buf, i, min_engine_on_10ms);

      buf[i] = radio_checksum(buf, i);
      i++;

      rf95.send(buf, i);
      rf95.waitPacketSent();
    }

    void send_module_status_packet() {
      if (RFM9X_FAIL) {
        return;
      }

      uint8_t buf[24];
      uint8_t i = 0;

      pack_u8(buf, i, MSG_CLASS_TELEMETRY);
      pack_u8(buf, i, TEL_MODULES);

      pack_u8(buf, i, build_module_flags());
      pack_u8(buf, i, build_makes_sense_flags());
      pack_u8(buf, i, build_valve_connection_flags());

      pack_u16(buf, i, (uint16_t)commands_received);
      pack_u8(buf, i, last_command_id_executed);

      pack_u8(buf, i, stage);
      pack_u8(buf, i, build_state_flags_for_status());

      buf[i] = radio_checksum(buf, i);
      i++;

      rf95.send(buf, i);
      rf95.waitPacketSent();
    }

  // override protocol

    bool override_password_matches(uint8_t* payload, uint8_t payload_len);
    uint8_t require_override_or_reject();

    void set_DS1307_from_radio_payload(uint8_t* payload, uint8_t payload_len);

    // ---------------- Override mode ----------------
    const char OVERRIDE_PASSWORD[9] = "12345678";

    bool override_mode_active = false;
    uint32_t override_mode_started_ms = 0;
    const uint32_t OVERRIDE_MODE_TIMEOUT_MS = 300000; // 5 mins

    bool override_mode_is_valid() {
      if (!override_mode_active) {
        return false;
      }

      if ((millis() - override_mode_started_ms) > OVERRIDE_MODE_TIMEOUT_MS) {
        override_mode_active = false;
        return false;
      }

      return true;
    }

    uint8_t require_override_or_reject() {
      if (override_mode_is_valid()) {
        return CMD_ACCEPTED;
      }

      return CMD_REJECT_OVERRIDE_REQUIRED;
    }

    bool override_password_matches(uint8_t* payload, uint8_t payload_len) {
      if (payload_len < 11) {
        return false;
      }

      // payload[0..2] = 0xFF safety bytes from Nano
      // payload[3..10] = 8-byte ASCII password
      for (uint8_t k = 0; k < 8; k++) {
        if ((char)payload[3 + k] != OVERRIDE_PASSWORD[k]) {
          return false;
        }
      }

      return true;
    }

    void set_DS1307_from_radio_payload(uint8_t* payload, uint8_t payload_len) {
      if (payload_len < 10) {
        return;
      }

      uint8_t yy   = payload[3];
      uint8_t mo   = payload[4];
      uint8_t dd   = payload[5];
      uint8_t hh   = payload[6];
      uint8_t mm   = payload[7];
      uint8_t ss   = payload[8];
      uint8_t dow  = payload[9];

      rtc.set(
        ss,
        mm,
        hh,
        dow,
        dd,
        mo,
        yy
      );

      delay(50);
      rtc.refresh();
    }

    uint32_t radio_payload_read_u32(uint8_t* payload, uint8_t index) {
      return ((uint32_t)payload[index]) |
            ((uint32_t)payload[index + 1] << 8) |
            ((uint32_t)payload[index + 2] << 16) |
            ((uint32_t)payload[index + 3] << 24);
    }

    uint8_t set_engine_timing_from_radio_payload(uint8_t* payload, uint8_t payload_len) {
      // payload[0..2] = FF FF FF safety bytes
      // payload[3..6] = MAIN_VALVE_AUTO_CLOSE_DELAY_MS
      // payload[7..10] = MIN_ENGINE_ON_TIME_MS
      // Serial.println('fff');

      if (payload_len < 11) {
        return CMD_REJECT_BAD_PAYLOAD;
      }

      uint32_t auto_close_delay_ms = radio_payload_read_u32(payload, 3);
      uint32_t min_engine_on_ms    = radio_payload_read_u32(payload, 7);

      if (auto_close_delay_ms < 100 || auto_close_delay_ms > 120000) {
        return CMD_REJECT_BAD_PAYLOAD;
      }

      if (min_engine_on_ms > auto_close_delay_ms) {
        return CMD_REJECT_BAD_PAYLOAD;
      }

      MAIN_VALVE_AUTO_CLOSE_DELAY_MS = auto_close_delay_ms;
      MAX_ENGINE_ON_TIME_MS = auto_close_delay_ms;
      MIN_ENGINE_ON_TIME_MS = min_engine_on_ms;

      return CMD_ACCEPTED;
    }



  // command execution goes last for easier declar
      uint8_t execute_new_radio_command(uint8_t command_id, uint8_t* payload, uint8_t payload_len) {
        switch (command_id) {
          case CMD_ABORT:
            if (execute_command_id(0x01, COMMAND_FROM_RADIO)) {
              return CMD_ACCEPTED;
            }
            return CMD_REJECT_UNKNOWN;

          case CMD_ARM:
            if (execute_command_id(0x02, COMMAND_FROM_RADIO)) {
              return CMD_ACCEPTED;
            }
            return CMD_REJECT_UNKNOWN;

          case CMD_IGNITION:
            if (stage != STAGE_ARM) {
              return CMD_REJECT_BAD_STAGE;
            }

            if (!avionics_armed) {
              return CMD_REJECT_NOT_ARMED;
            }

            if (!arm_sequence_complete) {
              return CMD_REJECT_ARM_NOT_COMPLETE;
            }

            if (execute_command_id(0x03, COMMAND_FROM_RADIO)) {
              return CMD_ACCEPTED;
            }

            return CMD_REJECT_UNKNOWN;

          case CMD_GET_STATUS:
            commands_received++;
            send_module_status_packet();
            return CMD_ACCEPTED;

          case CMD_GET_GPS:
            commands_received++;
            send_gps_response_packet();
            return CMD_ACCEPTED;

          case CMD_GRAVITY:
            gravity_anchor();
            commands_received++;
            // Serial.println('gravity anchor executed');
            return CMD_ACCEPTED;

          case CMD_BME_CAL:
            calibrate_primary_baro_pressure_threshold();
            commands_received++;
            return CMD_ACCEPTED;

          case CMD_OVERRIDE:
            if (!override_password_matches(payload, payload_len)) {
              return CMD_REJECT_BAD_PASSWORD;
            }

            override_mode_active = true;
            override_mode_started_ms = millis();
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_EXIT_OVERRIDE:
            override_mode_active = false;
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_OPEN_MAIN:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_open(MAIN_VALVE);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_CLOSE_MAIN:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_close(MAIN_VALVE);
            commands_received++;

            if (no_return_flight_started) {
              set_stage(STAGE_DATA_GATHERING_FLIGHT);
            }

            return CMD_ACCEPTED;


          case CMD_OPEN_DUMP:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_open(DUMP_VALVE);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_CLOSE_DUMP:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_close(DUMP_VALVE);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_OPEN_RELIEF:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_open(RELIEF_VALVE);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_CLOSE_RELIEF:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            VALVE_close(RELIEF_VALVE);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_SET_STAGE:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            if (payload_len < 4) {
              return CMD_REJECT_BAD_PAYLOAD;
            }

            set_stage(payload[3]);
            commands_received++;
            return CMD_ACCEPTED;


          case CMD_SET_TIME:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            if (payload_len < 10) {
              return CMD_REJECT_BAD_PAYLOAD;
            }

            set_DS1307_from_radio_payload(payload, payload_len);
            commands_received++;
            return CMD_ACCEPTED;
          case CMD_SET_ENGINE_TIMING:
            if (require_override_or_reject() != CMD_ACCEPTED) {
              return CMD_REJECT_OVERRIDE_REQUIRED;
            }

            {
              uint8_t result = set_engine_timing_from_radio_payload(payload, payload_len);

              if (result != CMD_ACCEPTED) {
                return result;
              }
            }

            commands_received++;
            return CMD_ACCEPTED;

          default:
            return CMD_REJECT_UNKNOWN;
        }
      }

    




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

    //scan_Wire2_I2C();

    // read_Wire2_chip_id(0x76);
    // read_Wire2_chip_id(0x77);

    print_load_progress(30);
    
    init_LSM6DSOX();
    print_load_progress(40);
    init_ADXL375();
    print_load_progress(50);
    //init_BME680();
    init_BMP390();

    //init_BME280();
    calibrate_primary_baro_pressure_threshold();

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
    loop_counter++;
    //Serial.println(micros());
    in_every_loop();
    ORACLE_update();
    run_stage_loop();
    update_radio_telemetry();
  } 


// ============================== STAGES ===========================
  void run_stage_loop() {
    uint32_t stage_loop_start_us = micros();

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

    stage_loop_time_us = micros() - stage_loop_start_us;
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
    read_RFM();
  }

  void stage_arm_loop() {
    // Stage 2: ARM
    // If ARM command/state is no longer active, return to waiting.

    if (!avionics_armed) {
      reset_arm_sequence();
      set_stage(STAGE_WAITING);
      return;
    }

    read_RFM();
    update_arm_sequence();
  }

  void stage_launch_ignition_sequence_loop() {
    if (!ignition_sequence_started) {
      if (!no_return_flight_started) {
        set_stage(STAGE_WAITING);
      }
      return;
    }
    read_RFM();

    update_main_valve_ignition_hold();
  }

  void stage_engine_working_loop() {
    // Stage 4: engine working / no-return flight stage.
    // No return to waiting, arm, or ignition sequence is allowed after this point.

    if ((millis() - flight_start_time_ms) >= MIN_ENGINE_ON_TIME_MS) {
      //stratologger_apogee_prediction = apogee_prediction();
      if (predicted_apogee_m >= real_desired_apogee_m) {
        Serial.print("desired apogee predicted: ");
        Serial.print(predicted_apogee_m);
        Serial.println(" m");
        Serial.println("closing main valve");
        VALVE_close(MAIN_VALVE);
        set_stage(STAGE_DATA_GATHERING_FLIGHT);
      }
    }

    if ((millis() - flight_start_time_ms) >= MAX_ENGINE_ON_TIME_MS) {
      Serial.println("max engine work time reached: closing main valve");
      VALVE_close(MAIN_VALVE);
      set_stage(STAGE_DATA_GATHERING_FLIGHT);
    }

  }

  void stage_data_gathering_flight_loop() {
    detect_apogee_after_engine_cutoff();
    update_relief_after_apogee();

    read_RFM();

    //send_current_location();
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
    Wire.setClock(1000000);   // 400 kHz I2C for BMP390
    Serial.println("online: main I2C Wire");

    Wire1.setSDA(ADXL375_SDA_pin);
    Wire1.setSCL(ADXL375_SCL_pin);
    Wire1.begin();
    Serial.println("online: ADXL375 I2C Wire1");

    Wire2.setSDA(extra_SDA_pin);
    Wire2.setSCL(extra_SCL_pin);
    Wire2.begin();
    Serial.println("online: extra I2C Wire2");
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
    Serial.print("BMP390_FAIL   = "); Serial.println(BMP390_FAIL);
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
    arm_countdown = ARM_COUNTDOWN_START_S;

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
    if (arm_sequence_complete) {
      return;
    }

    if (!arm_sequence_active) {
      start_arm_sequence();
    }

    uint32_t elapsed_ms = millis() - arm_sequence_start_time_ms;

    int32_t elapsed_s = elapsed_ms / 1000;
    int32_t seconds_since_countdown_start = arm_countdown - ARM_COUNTDOWN_START_S;

    if (elapsed_s > seconds_since_countdown_start) {
      arm_countdown++;

      Serial.print("ARM countdown = ");
      Serial.println(arm_countdown);
    }

    if (elapsed_ms >= ARM_SEQUENCE_DURATION_MS) {
      arm_sequence_active = false;
      arm_sequence_complete = true;

      Serial.println("ARM sequence complete");
      Serial.println("Waiting for RADIO IGNITION START command");
    }
  }

  void reset_arm_sequence() {
    arm_sequence_active = false;
    arm_sequence_started_once = false;

    arm_sequence_start_time_ms = 0;
    arm_countdown = ARM_COUNTDOWN_START_S;
    arm_sequence_complete = false;
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

    int value = 0;
    int max_value = 0;

    if (pin == MAIN_VALVE_CONNECTION_pin ||
        pin == RELIEF_VALVE_CONNECTION_pin ||
        pin == DUMP_VALVE_CONNECTION_pin ||
        pin == PRESSURE_TRANSDUCER_4_pin) {

      value = analogRead(pin);
      max_value = 1023;   // because analogReadResolution(10)
    }
    else {
      value = adc->adc0->analogRead(pin);
      max_value = adc->adc0->getMaxValue();
    }

    float Vout = value * 3.3f / max_value;

    return Vout;
  }

  void read_battery_voltage() {
    if ((uint32_t)(micros() - battery_last_time) >= battery_rate_us) {
        battery_last_time = micros();
      if (BATTERY_VOLTAGE_pin < 0) return;

      float voltage = read_voltage((uint8_t)BATTERY_VOLTAGE_pin);
      battery_voltage_output = voltage*4.8f; // TODO: multiply by divider ratio.
    }
  }

  void read_pressure() {

    if (micros() - Pressure_last_time > Pres_rate) {
      float voltage = 0.0f;

      voltage = read_voltage(PRESSURE_TRANSDUCER_1_pin); // 1 - pin 41 - N2O/fuel tank
      pressure_1_output = n2o_voltage_to_pressure_psi(voltage);
      //pressure_1_output = PRESSURE_1_SLOPE * voltage + PRESSURE_1_OFFSET;

      voltage = read_voltage(PRESSURE_TRANSDUCER_2_pin); // 2 - pin 40 - extra / currently unused
      pressure_2_output = PRESSURE_2_SLOPE * voltage + PRESSURE_2_OFFSET;

      voltage = read_voltage(PRESSURE_TRANSDUCER_3_pin); // 3 - pin 15 - combustion chamber
      pressure_3_output = combust_voltage_to_pressure_psi(voltage);
      //pressure_3_output = PRESSURE_3_SLOPE * voltage + PRESSURE_3_OFFSET;

      voltage = read_voltage(PRESSURE_TRANSDUCER_4_pin); // 4 - pin 27 - CO2 pressure level
      pressure_4_output = co2_voltage_to_pressure_psi(voltage);
      //pressure_4_output = PRESSURE_4_SLOPE * voltage + PRESSURE_4_OFFSET;

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
  

  void init_BME680() {
    if (!BME680.begin(BME680_I2C_ADDRESS)) {
      Serial.println("offline: BME680");
      BME680_FAIL = true;
      return;
    }

    BME680.setTemperatureOversampling(BME680_OS_4X);
    BME680.setPressureOversampling(BME680_OS_2X);

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
        apply_primary_baro_pressure_deadband();
        BME680_new_data = true;
      }
      bme_busy = false;
    }
  }

 

  void calibrate_primary_baro_pressure_threshold() {
    if (BMP390_FAIL) {
      Serial.println("BMP390 calibration failed: sensor offline");
      return;
    }

    Serial.println();
    Serial.println("BMP390 pressure threshold calibration started");

    uint32_t Icalibration_start_ms = millis();
    uint32_t Icalibration_duration_ms = (uint32_t)(BME_calibration_duration_s * 1.0f / 4.0f * 1000.0f);

    while (millis() - Icalibration_start_ms < Icalibration_duration_ms){
      read_BMP390();
    }

    uint32_t calibration_start_ms = millis();
    uint32_t calibration_duration_ms = (uint32_t)(BME_calibration_duration_s * 3.0f / 4.0f * 1000.0f);

    bool first_sample = true;
    float min_pressure_hpa = 0.0f;
    float max_pressure_hpa = 0.0f;

    while (millis() - calibration_start_ms < calibration_duration_ms) {
      read_BMP390();

      if (BMP390_new_data) {
        float current_pressure_hpa = BMP390_pressure_hpa;

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
      Serial.println("BMP390 calibration failed: no valid samples");
      return;
    }

    BME_pressure_threshold_hpa = max_pressure_hpa - min_pressure_hpa;

    BMP390_pressure_hpa = (min_pressure_hpa + max_pressure_hpa) * 0.5f;
    BME_filtered_pressure_hpa = BMP390_pressure_hpa;
    BME_filter_started = true;

    Serial.println("BMP390 pressure threshold calibration complete");

    Serial.print("min pressure hPa = ");
    Serial.println(min_pressure_hpa, 4);

    Serial.print("max pressure hPa = ");
    Serial.println(max_pressure_hpa, 4);

    Serial.print("pressure threshold hPa = ");
    Serial.println(BME_pressure_threshold_hpa, 4);

    Serial.print("used range = +/- ");
    Serial.println(BME_pressure_threshold_hpa * 0.5f, 4);

    Serial.println();
  }

  

  void apply_primary_baro_pressure_deadband() {

    float BMP_reading_difference = abs(old_BMP_reading - BMP390_pressure_hpa);

    if (BMP_reading_difference < BME_pressure_threshold_hpa) {
      return;
    }

    old_BMP_reading = BMP390_pressure_hpa;

    BMP390_altitude_m = 44330.0f * (1.0f - pow(BMP390_pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));

  }

  // void init_BME280() {
  //   if (!BME280.begin(BME280_I2C_ADDRESS, &Wire2)) {
  //     Serial.println("offline: BME280");
  //     BME280_FAIL = true;
  //     return;
  //   }

  //   BME280.setSampling(
  //     Adafruit_BME280::MODE_NORMAL,
  //     Adafruit_BME280::SAMPLING_X2,   // temperature
  //     Adafruit_BME280::SAMPLING_X8,   // pressure
  //     Adafruit_BME280::SAMPLING_X1,   // humidity
  //     Adafruit_BME280::FILTER_X4,
  //     Adafruit_BME280::STANDBY_MS_125
  //   );

  //   Serial.println("online: BME280");
  // }

  // void read_BME280() {
  //   BME280_new_data = false;

  //   if (BME280_FAIL) {
  //     return;
  //   }

  //   //BME280_temp = BME280.readTemperature();
  //   BME280_pressure_hpa = BME280.readPressure() / 100.0f;
  //   //BME280_humidity = BME280.readHumidity();

  //   BME280_altitude_m =
  //     44330.0f * (1.0f - pow(BME280_pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));

  //   BME280_new_data = true;
  // }
  uint8_t bmp390_read8(uint8_t reg) {
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(BMP390_I2C_ADDRESS, (uint8_t)1);

    if (Wire.available()) {
      return Wire.read();
    }

    return 0xFF;
  }

  void bmp390_read_burst(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(BMP390_I2C_ADDRESS, len);

    for (uint8_t i = 0; i < len; i++) {
      if (Wire.available()) {
        data[i] = Wire.read();
      } else {
        data[i] = 0xFF;
      }
    }
  }

  void bmp390_write8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  void print_BMP390_raw_debug() {
    uint8_t status = bmp390_read8(0x03);
    uint8_t pwr    = bmp390_read8(0x1B);
    uint8_t osr    = bmp390_read8(0x1C);
    uint8_t odr    = bmp390_read8(0x1D);
    uint8_t config = bmp390_read8(0x1F);

    uint8_t data[6];
    bmp390_read_burst(0x04, data, 6);

    Serial.print("BMP390 RAW | STATUS=0x");
    if (status < 16) Serial.print("0");
    Serial.print(status, HEX);

    Serial.print(" | PWR=0x");
    if (pwr < 16) Serial.print("0");
    Serial.print(pwr, HEX);

    Serial.print(" | OSR=0x");
    if (osr < 16) Serial.print("0");
    Serial.print(osr, HEX);

    Serial.print(" | ODR=0x");
    if (odr < 16) Serial.print("0");
    Serial.print(odr, HEX);

    Serial.print(" | CONFIG=0x");
    if (config < 16) Serial.print("0");
    Serial.print(config, HEX);

    Serial.print(" | DATA=");
    for (uint8_t i = 0; i < 6; i++) {
      if (data[i] < 16) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }

  void init_BMP390() {
    if (!BMP390.begin_I2C(BMP390_I2C_ADDRESS, &Wire)) {
      Serial.println("offline: BMP390");
      BMP390_FAIL = true;
      return;
    }

    bool ok = BMP390.startNormalModeFast(
      BMP3_NO_OVERSAMPLING,        // pressure OS = 1x
      BMP3_NO_OVERSAMPLING,        // temperature OS = 1x
      BMP3_IIR_FILTER_DISABLE,     // or BMP3_IIR_FILTER_COEFF_1 / 3
      BMP3_ODR_200_HZ,
      true                         // keep temp enabled for pressure compensation
    );

    if (!ok) {
      Serial.println("offline: BMP390 normal mode");
      BMP390_FAIL = true;
      return;
    }

    // Direct register backup configuration:
// OSR 1x pressure, 1x temperature
bmp390_write8(0x1C, 0x00);

// ODR 200 Hz
bmp390_write8(0x1D, 0x00);

// IIR filter off
bmp390_write8(0x1F, 0x00);

// PWR_CTRL: pressure enable + temp enable + normal mode
// bit0 = press_en, bit1 = temp_en, mode = 0b11 in mode bits
bmp390_write8(0x1B, 0x33);

    Serial.println("online: BMP390 normal mode 200Hz");
  }

  void read_BMP390() {
    // start_BMP390_read();
    // finish_BMP390_read();

    
      BMP390_new_data = false;

      if (BMP390_FAIL) {
        return;
      }

      static uint32_t last_poll_us = 0;
      const uint32_t BMP390_POLL_RATE_US = 8000; 

      uint32_t now_us = micros();

      if ((uint32_t)(now_us - last_poll_us) < BMP390_POLL_RATE_US) {
        return;
      }

      last_poll_us = now_us;

      if (!BMP390.readNormalModeData(true, true)) {
        return;
      }

      BMP390_temp = BMP390.temperature;
      BMP390_pressure_hpa = BMP390.pressure / 100.0f;
      BMP390_altitude_m =
        44330.0f * (1.0f - pow(BMP390_pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));

      //apply_primary_baro_pressure_deadband();

      BMP390_new_data = true;
      bmp390_new_counter++;
    
  }

  void start_BMP390_read() {
    if (BMP390_FAIL) {
      bmp390_busy = false;
      BMP390_new_data = false;
      return;
    }

    if (bmp390_busy) {
      return;
    }

    uint32_t now_us = micros();

    if ((uint32_t)(now_us - BMP390_last_time) < BMP390_rate) {
      return;
    }

    bmp390_busy = true;

    // Important: this is the time the read cycle starts, not finishes.
    BMP390_last_time = now_us;

    bmp390_ready_time_us = now_us + BMP390_READ_DELAY_US;
  }
  
  void finish_BMP390_read() {
    BMP390_new_data = false;

    if (BMP390_FAIL) {
      bmp390_busy = false;
      return;
    }

    if (!bmp390_busy) {
      return;
    }

    if ((int32_t)(micros() - bmp390_ready_time_us) < 0) {
      return;
    }

    bmp390_busy = false;

    if (!BMP390.performReading()) {
      BMP390_new_data = false;
      return;
    }

    BMP390_temp = BMP390.temperature;
    BMP390_pressure_hpa = BMP390.pressure / 100.0f;

    apply_primary_baro_pressure_deadband();

    BMP390_new_data = true;
    bmp390_new_counter++;
  }

  void init_GPS() {
    GPS_PORT.begin(9600);

    pinMode(GPS_PPS_pin, INPUT);

    GPS_FAIL = false;

    Serial.println("online: GPS SoftwareSerial NMEA");
    Serial.println("GPS RX software pin = 7");
    Serial.println("GPS TX software pin = 8");
    Serial.println("GPS PPS pin = 34");
  }

  void read_GPS() {
    if (GPS_FAIL) return;

    static uint32_t last_rx_ms = 0;

    const uint16_t GPS_MAX_BYTES_PER_LOOP = 64;
    const uint32_t GPS_MAX_READ_TIME_US = 500;

    uint16_t bytes_read = 0;
    uint32_t start_us = micros();

    while (GPS_PORT.available() &&
          bytes_read < GPS_MAX_BYTES_PER_LOOP &&
          (micros() - start_us) < GPS_MAX_READ_TIME_US) {

      char c = GPS_PORT.read();
      gps.encode(c);

      last_rx_ms = millis();
      bytes_read++;
    }

    bool location_fresh =
      gps.location.isValid() &&
      gps.location.age() < 5000;

    if (location_fresh) {
      gps_lat = gps.location.lat();
      gps_lon = gps.location.lng();
    }

    if (gps.altitude.isValid() && gps.altitude.age() < 5000) {
      gps_alt_m = gps.altitude.meters();
    }

    if (gps.satellites.isValid() && gps.satellites.age() < 5000) {
      gps_siv = gps.satellites.value();
    } else {
      gps_siv = 0;
    }

    bool satellites_ok = gps_siv >= 4;

    if (location_fresh && satellites_ok) {
      gps_fix_type = 1;
    } else {
      gps_fix_type = 0;
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
      countdown = IGNITION_COUNTDOWN_START_S;
      return;
    }

    uint32_t elapsed_ms = millis() - main_valve_ignition_start_time_ms;

    static uint32_t last_ignition_status_print_ms = 0;

    if (millis() - last_ignition_status_print_ms >= 1000) {
      Serial.print("IGNITION elapsed ms = ");
      Serial.print(elapsed_ms);
      Serial.print(" / open at ms = ");
      Serial.println(MAIN_VALVE_OPEN_AFTER_START_MS);

      last_ignition_status_print_ms = millis();
    }

    // Run calibration exactly once, 0.5 seconds before main valve opens.
    if (elapsed_ms >= MAIN_VALVE_OPEN_AFTER_START_MS - 500 && !main_valve_opened_for_flight && !pre_launch_calibration_done) {
      Serial.println("PRE-LAUNCH CALIBRATION STARTED");
      calibrate_all();
      set_to_start_location();
      pre_launch_calibration_done = true;
      Serial.println("PRE-LAUNCH CALIBRATION COMPLETE");
    }

    // Open main valve at launch time.
    if (elapsed_ms >= MAIN_VALVE_OPEN_AFTER_START_MS && !main_valve_opened_for_flight) {

      enter_no_return_flight();
      set_to_start_location();

      oxidizer_mass_integration_enabled = true;

      VALVE_open(MAIN_VALVE);

      main_valve_opened_for_flight = true;
      main_valve_ignition_hold_active = false;

      Serial.println("MAIN VALVE OPENED FOR FLIGHT");

      return;
    }

    int32_t elapsed_s = elapsed_ms / 1000;
    int32_t seconds_since_countdown_start = countdown - IGNITION_COUNTDOWN_START_S;

    if (elapsed_s > seconds_since_countdown_start) {
      countdown++;
      Serial.println(countdown);
    }
  }

  void in_every_loop() {
    read_BMP390();
    read_LSM6DSOX();
    read_pressure();

    read_GPS();
    read_DS1307();
    read_ADXL375();

    read_valve_connection();
    read_battery_voltage();

    save_to_sd();

    update_frequency_counters();

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
    if ((uint32_t)(micros() - valve_connection_last_time) >= valve_connection_rate_us) {
      valve_connection_last_time = micros();
      valve_connection_check_all();
    }
  }

  void valve_connection_check_all() {
    valve_connection_check(1);
    valve_connection_check(2);
    valve_connection_check(3);
  }

  bool valve_connection_check(uint8_t valve_id) {
    Valve* valve = get_valve_by_id(valve_id);

    if (valve == nullptr) {
      return false;
    }

    valve->connection_voltage = valve_connection_voltage_read(valve_id);
    valve->connection_ok = valve->connection_voltage >= VALVE_CONNECTION_MIN_VOLTAGE;

    return valve->connection_ok;
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
    if (BMP390_FAIL)   state_array |= 0x00001000;
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
    in_floats[5] = BMP390_altitude_m;
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
  // old read radio
    // void read_RFM() {
    //   if (!RFM9X_FAIL && rf95.available()) {
    //     uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    //     uint8_t len = sizeof(buf);

    //     if (rf95.recv(buf, &len)) {
    //       message_parser(buf);

    //       if (recv_ready) {
    //         bool command_accepted = execute_radio_message(buf[0], buf[1]);

    //         // Ground station expects ACK1 from AV1 for accepted class 0x01 commands.
    //         if (command_accepted && buf[0] == 0x01) {
    //           send_ACK_packet(1);
    //         }

    //         print_local_status();
    //         recv_ready = 0;
    //       }
    //     }
    //   }
    // }

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
        command_handled = true;

        if (source != COMMAND_FROM_RADIO) {
          Serial.println("IGNITION START IGNORED: command must come from RADIO");
          break;
        }

        if (!avionics_armed) {
          Serial.println("IGNITION START IGNORED: AV1 not armed");
          break;
        }

        if (stage != STAGE_ARM) {
          Serial.println("IGNITION START IGNORED: rocket is not in ARM stage");
          break;
        }

        if (!arm_sequence_complete) {
          Serial.println("IGNITION START IGNORED: ARM sequence not complete");
          break;
        }

        commands_received++;

        ignition_sequence_started = true;
        main_valve_can_close_by_radio = false;
        main_valve_ignition_hold_active = true;
        main_valve_ignition_start_time_ms = millis();
        pre_launch_calibration_done = false;

        VALVE_close(RELIEF_VALVE);
        VALVE_close(DUMP_VALVE);

        set_stage(STAGE_LAUNCH_IGNITION_SEQUENCE);

        Serial.print("IGNITION START ACCEPTED from ");
        Serial.println(command_source_name(source));

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
    if (command_handled) {
      last_command_id_executed = msg_id;
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
          "BMP390_temp,"
          "BMP390_pressure_hpa,"
          "BMP390_altitude_m,"
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

        Black_Box.print(BMP390_temp, 3); Black_Box.print(',');
        Black_Box.print(BMP390_pressure_hpa, 3); Black_Box.print(',');
        Black_Box.print(BMP390_altitude_m, 3); Black_Box.print(',');

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
    LSM_to_PCB_switch_over();

    gravity_anchor();

    if (BME_makes_sense) {
      global_location = {0.0f, 0.0f, BMP390_altitude_m - BME_alt_offset};
    }
    else if (GPS_makes_sense) {
      global_location = {0.0f, 0.0f, gps_alt_m};
    }
    else {
      global_location = {0.0f, 0.0f, manual_zero_alt};
    }

    initial_SL_altitude_m = global_location.z_m;
    real_desired_apogee_m = desired_apogee_m + initial_SL_altitude_m;
    current_max_altitude_reached_m = 0.0f;

    PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};
    previous_PCB_global_vel_vector = {0.0f, 0.0f, 0.0f};

    // oracle_last_update_us = micros(); delete
    oracle_delta_time_s = 0.0f;
  }

  void ORACLE_update() { // will determine location and vel vectors for next simulation
    LSM_to_PCB_switch_over(); 

    update_sensor_makes_sense();

    local_PCB_direction_determination();

    if (fuel_tank_pressure_makes_sense && combustion_chamber_pressure_makes_sense) {
      current_rocket_mass();
    }
    else {
      current_rocket_mass_from_time_function();
    }

    if (LSM_makes_sense) {
      calculate_rocket_tilt_from_orientation();
    }
    else {
       calculate_rocket_tilt_from_manual_set();
    }

    if (BME_makes_sense) {
      update_altitude_kalman_filter();
    }
    else {
      update_altitude_kalman_filter_from_fallback();
    }

    calculate_rocket_speed_from_vertical_speed();

    apogee_prediction();
    //Serial.println(predicted_apogee_m);
  }
 
  void LSM_to_PCB_switch_over(){ // accelerometers orientation to rocket orientation
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
    rocket_mass_using_pressure_flow = true;

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

    // PCB_gyro_mag = vector_magnitude(PCB_local_gyro_vector); delete

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
    rocket_tilt_using_manual_set = false;
    rocket_tilt_using_LSM = true;

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

  void set_manual_rocket_tilt_angle_deg(float angle_deg) {
    if (!isfinite(angle_deg)) {
      angle_deg = 0.0f;
    }

    if (angle_deg < 0.0f) {
      angle_deg = 0.0f;
    }

    if (angle_deg > 180.0f) {
      angle_deg = 180.0f;
    }

    manual_rocket_tilt_angle_deg = angle_deg;
    manual_rocket_tilt_angle_rad = manual_rocket_tilt_angle_deg * 0.01745329252f;
  }

  void calculate_rocket_tilt_from_manual_set() {
    rocket_tilt_using_LSM = false;
    rocket_tilt_using_manual_set = true;

    rocket_tilt_angle_deg = manual_rocket_tilt_angle_deg;
    rocket_tilt_angle_rad = manual_rocket_tilt_angle_rad;

    rocket_vertical_cos = cosf(rocket_tilt_angle_rad);

    if (rocket_vertical_cos > 1.0f) {
      rocket_vertical_cos = 1.0f;
    }

    if (rocket_vertical_cos < -1.0f) {
      rocket_vertical_cos = -1.0f;
    }
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
    altitude_kalman_using_BME = true;

    // Only update filter when barometer has a new real measurement.
    if (!BMP390_new_data) {
      return;
    }

    float measured_altitude_m = BMP390_altitude_m;
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

    Serial.print("BME_altitude_m = ");
    Serial.println(BMP390_altitude_m, 4);

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
    // start_BME680_read();
    // finish_BME680_read();
    read_BMP390();

    read_LSM6DSOX();


    ORACLE_update();
    

    print_orientation_for_visualizer();
    orientation_visualizer_last_time = micros();
  }

  void update_raw_vertical_speed() {
    if (!BMP390_new_data) {
      return;
    }

    uint32_t now_us = micros();

    if (!raw_vertical_speed_initialized) {
      previous_raw_altitude_m = BMP390_altitude_m;
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

    raw_vertical_speed_mps = (BMP390_altitude_m - previous_raw_altitude_m) / dt_s;
    previous_raw_altitude_m = BMP390_altitude_m;
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
      predicted_apogee_m = BMP390_altitude_m;
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



  // red flags
    void update_sensor_makes_sense() {
      uint32_t now_ms = millis();

      // ---------------- BME / barometer ----------------
      if (BMP390_FAIL) {
        BME_makes_sense = false;
        BME_bad_start_ms = now_ms;
      }
      else {
        bool bme_bad = false;

        bme_bad |= value_outside_range_or_not_finite(
          BMP390_altitude_m,
          BME_ALTITUDE_MIN_M,
          BME_ALTITUDE_MAX_M
        );

        bme_bad |= value_rate_too_high(
          BMP390_altitude_m,
          previous_BME_altitude_for_sense_m,
          now_ms,
          previous_BME_sense_time_ms,
          previous_BME_sense_valid,
          BME_ALTITUDE_MAX_RATE_MPS
        );

        update_makes_sense_flag(
          bme_bad,
          BME_makes_sense,
          BME_bad_start_ms,
          BME_MAKES_SENSE_FAIL_TIME_MS
        );

        if (!bme_bad) {
          previous_BME_altitude_for_sense_m = BMP390_altitude_m;
          previous_BME_sense_time_ms = now_ms;
          previous_BME_sense_valid = true;
        }
      }


      // ---------------- GPS ----------------
      if (GPS_FAIL) {
        GPS_makes_sense = false;
        GPS_bad_start_ms = now_ms;
      }
      else {
        bool gps_bad = false;

        gps_bad |= (gps_fix_type == 0);
        gps_bad |= (gps_siv < GPS_MIN_SIV);

        gps_bad |= !isfinite(gps_lat);
        gps_bad |= !isfinite(gps_lon);
        gps_bad |= !isfinite(gps_alt_m);

        // Basic Earth-valid coordinate check
        gps_bad |= (gps_lat < -90.0 || gps_lat > 90.0);
        gps_bad |= (gps_lon < -180.0 || gps_lon > 180.0);

        // Mission area / launch field box check
        gps_bad |= !gps_inside_allowed_area(gps_lat, gps_lon);

        gps_bad |= value_outside_range_or_not_finite(
          gps_alt_m,
          GPS_ALTITUDE_MIN_M,
          GPS_ALTITUDE_MAX_M
        );

        gps_bad |= value_rate_too_high(
          gps_alt_m,
          previous_GPS_altitude_for_sense_m,
          now_ms,
          previous_GPS_sense_time_ms,
          previous_GPS_sense_valid,
          GPS_MAX_ALTITUDE_RATE_MPS
        );

        update_makes_sense_flag(
          gps_bad,
          GPS_makes_sense,
          GPS_bad_start_ms,
          GPS_MAKES_SENSE_FAIL_TIME_MS
        );

        if (!gps_bad) {
          previous_GPS_altitude_for_sense_m = gps_alt_m;
          previous_GPS_sense_time_ms = now_ms;
          previous_GPS_sense_valid = true;
        }
      }


      // ---------------- LSM6DSOX ----------------
      if (LSM6DSOX_FAIL) {
        LSM_makes_sense = false;
        LSM_bad_start_ms = now_ms;
      }
      else {
        bool lsm_bad = false;

        float lsm_acc_mag = sqrtf(
          PCB_local_acc_vector.x * PCB_local_acc_vector.x +
          PCB_local_acc_vector.y * PCB_local_acc_vector.y +
          PCB_local_acc_vector.z * PCB_local_acc_vector.z
        );

        float lsm_gyro_mag = sqrtf(
          PCB_gx * PCB_gx +
          PCB_gy * PCB_gy +
          PCB_gz * PCB_gz
        );

        lsm_bad |= !isfinite(PCB_local_acc_vector.x);
        lsm_bad |= !isfinite(PCB_local_acc_vector.y);
        lsm_bad |= !isfinite(PCB_local_acc_vector.z);

        lsm_bad |= !isfinite(PCB_gx);
        lsm_bad |= !isfinite(PCB_gy);
        lsm_bad |= !isfinite(PCB_gz);

        lsm_bad |= !isfinite(lsm_acc_mag);
        lsm_bad |= !isfinite(lsm_gyro_mag);

        lsm_bad |= (lsm_acc_mag > LSM_ACCEL_MAG_MAX_MPS2);
        lsm_bad |= (lsm_gyro_mag > LSM_GYRO_MAG_MAX_RAD_S);

        update_makes_sense_flag(
          lsm_bad,
          LSM_makes_sense,
          LSM_bad_start_ms,
          LSM_MAKES_SENSE_FAIL_TIME_MS
        );
      }

      


      // ---------------- ADXL375 ----------------
      if (ADXL375_FAIL) {
        ADXL_makes_sense = false;
        ADXL_bad_start_ms = now_ms;
      }
      else {
        bool adxl_bad = false;

        float adxl_acc_mag = sqrtf(
          adxl_ax * adxl_ax +
          adxl_ay * adxl_ay +
          adxl_az * adxl_az
        );

        adxl_bad |= !isfinite(adxl_ax);
        adxl_bad |= !isfinite(adxl_ay);
        adxl_bad |= !isfinite(adxl_az);

        adxl_bad |= !isfinite(adxl_acc_mag);

        adxl_bad |= (adxl_acc_mag > ADXL_ACCEL_MAG_MAX_MPS2);

        update_makes_sense_flag(
          adxl_bad,
          ADXL_makes_sense,
          ADXL_bad_start_ms,
          ADXL_MAKES_SENSE_FAIL_TIME_MS
        );
      }

      // ---------------- Fuel tank pressure / pressure_1_output ----------------
      {
        bool tank_pressure_bad = false;

        tank_pressure_bad |= value_outside_range_or_not_finite(
          pressure_1_output,
          FUEL_TANK_PRESSURE_MIN_PSI,
          FUEL_TANK_PRESSURE_MAX_PSI
        );

        tank_pressure_bad |= value_rate_too_high(
          pressure_1_output,
          previous_fuel_tank_pressure_for_sense_psi,
          now_ms,
          previous_fuel_tank_pressure_sense_time_ms,
          previous_fuel_tank_pressure_sense_valid,
          FUEL_TANK_PRESSURE_MAX_RATE_PSI_S
        );

        update_makes_sense_flag(
          tank_pressure_bad,
          fuel_tank_pressure_makes_sense,
          fuel_tank_pressure_bad_start_ms,
          FUEL_TANK_PRESSURE_FAIL_TIME_MS
        );

        if (!tank_pressure_bad) {
          previous_fuel_tank_pressure_for_sense_psi = pressure_1_output;
          previous_fuel_tank_pressure_sense_time_ms = now_ms;
          previous_fuel_tank_pressure_sense_valid = true;
        }
      }


      // ---------------- Combustion chamber pressure / pressure_3_output ----------------
      {
        bool chamber_pressure_bad = false;

        chamber_pressure_bad |= value_outside_range_or_not_finite(
          pressure_3_output,
          CHAMBER_PRESSURE_MIN_PSI,
          CHAMBER_PRESSURE_MAX_PSI
        );

        chamber_pressure_bad |= value_rate_too_high(
          pressure_3_output,
          previous_chamber_pressure_for_sense_psi,
          now_ms,
          previous_chamber_pressure_sense_time_ms,
          previous_chamber_pressure_sense_valid,
          CHAMBER_PRESSURE_MAX_RATE_PSI_S
        );

        update_makes_sense_flag(
          chamber_pressure_bad,
          combustion_chamber_pressure_makes_sense,
          combustion_chamber_pressure_bad_start_ms,
          CHAMBER_PRESSURE_FAIL_TIME_MS
        );

        if (!chamber_pressure_bad) {
          previous_chamber_pressure_for_sense_psi = pressure_3_output;
          previous_chamber_pressure_sense_time_ms = now_ms;
          previous_chamber_pressure_sense_valid = true;
        }
      }
    }

    bool gps_inside_allowed_area(double lat, double lon) {
      if (!GPS_USE_ALLOWED_AREA_CHECK) {
        return true;
      }

      if (!isfinite(lat) || !isfinite(lon)) {
        return false;
      }

      if (lat < GPS_ALLOWED_LAT_MIN || lat > GPS_ALLOWED_LAT_MAX) {
        return false;
      }

      if (lon < GPS_ALLOWED_LON_MIN || lon > GPS_ALLOWED_LON_MAX) {
        return false;
      }

      return true;
    }

    void current_rocket_mass_from_time_function() {
      rocket_mass_using_pressure_flow = false;

      float dt_s = oracle_delta_time_s;

      if (dt_s <= 0.0f || dt_s > 0.5f) {
        rocket_mass = no_fuel_rocket_mass + fuel_mass;
        return;
      }

      if (!oxidizer_mass_integration_enabled || flight_start_time_ms == 0) {
        oxidizer_mass_flow_kg_s = 0.0f;
        rocket_mass = no_fuel_rocket_mass + fuel_mass;
        return;
      }

      fallback_engine_time_s = (millis() - flight_start_time_ms) / 1000.0f;

      oxidizer_mass_flow_kg_s = lookup_fallback_oxidizer_mdot_from_engine_time(fallback_engine_time_s);

      oxidizer_mass_used_kg += oxidizer_mass_flow_kg_s * dt_s;

      if (oxidizer_mass_used_kg > initial_fuel_mass) {
        oxidizer_mass_used_kg = initial_fuel_mass;
      }

      fuel_mass = initial_fuel_mass - oxidizer_mass_used_kg;

      if (fuel_mass < 0.0f) {
        fuel_mass = 0.0f;
      }

      rocket_mass = no_fuel_rocket_mass + fuel_mass;
    }


    float lookup_fallback_oxidizer_mdot_from_engine_time(float engine_time_s) {
      if (engine_time_s <= engine_mdot_fallback_time_s[0]) {
        return engine_mdot_fallback_kg_s[0];
      }

      if (engine_time_s >= ENGINE_MDOT_FALLBACK_MAX_TIME_S) {
        return 0.0f;
      }

      // Table step is 0.05 s, so direct index lookup is faster than loop.
      int i = (int)floorf(engine_time_s / 0.05f);

      if (i < 0) {
        i = 0;
      }

      if (i >= ENGINE_MDOT_FALLBACK_TABLE_SIZE - 1) {
        i = ENGINE_MDOT_FALLBACK_TABLE_SIZE - 2;
      }

      float t0 = engine_mdot_fallback_time_s[i];
      float t1 = engine_mdot_fallback_time_s[i + 1];

      float m0 = engine_mdot_fallback_kg_s[i];
      float m1 = engine_mdot_fallback_kg_s[i + 1];

      float f = 0.0f;

      if ((t1 - t0) > 0.000001f) {
        f = (engine_time_s - t0) / (t1 - t0);
      }

      return m0 + f * (m1 - m0);
    }


    // void calculate_rocket_tilt_from_fallback() { delete
    //   rocket_tilt_using_LSM = false; delete

    //   // TODO: delete
    //   // Later add orientation/tilt fallback if LSM does not make sense. delete
    //   // delete
    //   // Possible future options: delete
    //   // 1. freeze last good tilt delete
    //   // 2. use conservative tilt angle delete
    //   // 3. mark apogee prediction unreliable delete
    //   // 4. use last good orientation matrix delete

    //   // For now: keep previous rocket_tilt_angle_rad, delete
    //   // rocket_tilt_angle_deg, and rocket_vertical_cos unchanged. delete
    // } delete

    void update_altitude_kalman_filter_from_fallback() {
      altitude_kalman_using_BME = false;

      // TODO:
      // Later add altitude / vertical speed fallback if BME does not make sense.
      //
      // Possible future options:
      // 1. freeze last good Kalman altitude and vertical speed
      // 2. use GPS altitude if GPS makes sense
      // 3. use coast model propagation
      // 4. make ORACLE prediction more conservative

      // For now: keep previous kalman_altitude_m and
      // kalman_vertical_speed_mps unchanged.
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

  bool value_outside_range_or_not_finite(float value, float min_value, float max_value) {
    if (!isfinite(value)) {
      return true;
    }

    if (value < min_value || value > max_value) {
      return true;
    }

    return false;
  }

  bool value_rate_too_high(float current_value,
                          float previous_value,
                          uint32_t current_time_ms,
                          uint32_t previous_time_ms,
                          bool previous_valid,
                          float max_rate_per_s) {
    if (!previous_valid) {
      return false;
    }

    uint32_t dt_ms = current_time_ms - previous_time_ms;

    if (dt_ms == 0) {
      return false;
    }

    float dt_s = dt_ms / 1000.0f;

    if (dt_s <= 0.0f) {
      return false;
    }

    float rate = fabsf(current_value - previous_value) / dt_s;

    return rate > max_rate_per_s;
  }

  void update_makes_sense_flag(bool value_is_bad,
                              bool& makes_sense_flag,
                              uint32_t& bad_start_ms,
                              uint32_t fail_time_ms) {
    uint32_t now_ms = millis();

    if (!value_is_bad) {
      makes_sense_flag = true;
      bad_start_ms = 0;
      return;
    }

    if (bad_start_ms == 0) {
      bad_start_ms = now_ms;
    }

    if ((now_ms - bad_start_ms) >= fail_time_ms) {
      makes_sense_flag = false;
    }
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
    //send_RFM();

    GPS_location_send_last_time_us = micros();
  }

  void reset_data_gathering_flight_flags() {
    apogee_detected = false;
    relief_opened_after_apogee = false;

    previous_kalman_vertical_speed_mps = 0.0f;
    previous_vertical_speed_valid = false;

    negative_vertical_speed_start_ms = 0;
    negative_vertical_speed_timer_active = false;

    apogee_detected_time_ms = 0;
    GPS_location_send_last_time_us = 0;
  }

  void detect_apogee_after_engine_cutoff() {
    if (apogee_detected) {
      return;
    }

    if (!kalman_altitude_initialized) {
      negative_vertical_speed_start_ms = 0;
      negative_vertical_speed_timer_active = false;
      return;
    }

    float current_vz = kalman_vertical_speed_mps;

    if (!isfinite(current_vz)) {
      negative_vertical_speed_start_ms = 0;
      negative_vertical_speed_timer_active = false;
      return;
    }

    bool vertical_speed_is_negative =
      current_vz <= -apogee_detection_vz_threshold_mps;

    if (vertical_speed_is_negative) {
      if (!negative_vertical_speed_timer_active) {
        negative_vertical_speed_timer_active = true;
        negative_vertical_speed_start_ms = millis();

        Serial.println("APOGEE CHECK: negative vertical speed started");
      }

      uint32_t negative_vz_duration_ms = millis() - negative_vertical_speed_start_ms;

      if (negative_vz_duration_ms >= APOGEE_NEGATIVE_VELOCITY_CONFIRM_TIME_MS) {
        apogee_detected = true;
        apogee_detected_time_ms = millis();

        Serial.println("APOGEE DETECTED: negative vertical speed confirmed");
      }
    }
    else {
      negative_vertical_speed_start_ms = 0;
      negative_vertical_speed_timer_active = false;
    }

    previous_kalman_vertical_speed_mps = current_vz;
    previous_vertical_speed_valid = true;
  }

  void update_relief_after_apogee() {
    if (relief_opened_after_apogee) {
      return;
    }

    uint32_t time_since_stage_entry_ms = millis() - stage_entry_time_ms;

    bool min_time_passed = time_since_stage_entry_ms >= MIN_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS;

    bool max_time_passed = time_since_stage_entry_ms >= MAX_RELIEF_OPEN_TIME_AFTER_ENGINE_OFF_MS;

    bool apogee_delay_passed = false;

    if (apogee_detected) {
      apogee_delay_passed = (millis() - apogee_detected_time_ms) >= RELIEF_OPEN_DELAY_AFTER_APOGEE_MS;
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
    Serial.println("b = toggle BME stream flag");
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
    Serial.println("B = calibrate BME pressure threshold");
    Serial.println("N = print BME pressure filter status");
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
      Serial.print("BME stream flag ");
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
      LSM_to_PCB_switch_over();
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
      calibrate_primary_baro_pressure_threshold();
    }
    else if (cmd == "N") {
      print_primary_baro_pressure_filter_status();
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
    Serial.print("BME temp                = "); Serial.println(BMP390_temp, 3);
    Serial.print("BME pressure hPa        = "); Serial.println(BMP390_pressure_hpa, 3);
    Serial.print("BME altitude m          = "); Serial.println(BMP390_altitude_m, 3);
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
    Serial.print("extra_SDA_pin   = "); Serial.println(extra_SDA_pin);
    Serial.print("extra_SCL_pin   = "); Serial.println(extra_SCL_pin);
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
    Serial.println("DEBUG ignition start disabled.");
    Serial.println("Ignition must be started by RADIO command 0x03 after ARM sequence complete.");
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
      static uint32_t last_baro_stream_ms = 0;

      if (millis() - last_baro_stream_ms >= 1000) {  // 10 Hz print only
        last_baro_stream_ms = millis();

        Serial.print(" | CODE Hz = ");
        Serial.print(code_loop_hz, 1);

        Serial.print(" | BMP Hz = ");
        Serial.print(bmp390_read_hz, 1);

        Serial.print(" || BMP390 | T = ");
        Serial.print(BMP390_temp, 3);

        Serial.print(" C | P = ");
        Serial.print(BMP390_pressure_hpa, 3);

        Serial.print(" hPa | ALT = ");
        Serial.print(BMP390_altitude_m, 3);

        Serial.print(" m | FAIL = ");
        Serial.print(BMP390_FAIL);

        Serial.print(" | NEW = ");
        Serial.println(BMP390_new_data);
      }
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

  void print_primary_baro_pressure_filter_status() {
    Serial.println("----- BMP390 PRESSURE FILTER STATUS -----");

    Serial.print("BMP390 calibration duration s = ");
    Serial.println(BME_calibration_duration_s, 2);

    Serial.print("BMP390 pressure threshold hPa = ");
    Serial.println(BME_pressure_threshold_hpa, 5);

    Serial.print("half threshold hPa = ");
    Serial.println(BME_pressure_threshold_hpa * 0.5f, 5);

    Serial.print("filtered pressure hPa = ");
    Serial.println(BME_filtered_pressure_hpa, 5);

    Serial.print("BMP390 pressure hPa current = ");
    Serial.println(BMP390_pressure_hpa, 5);

    Serial.print("BMP390 altitude m current = ");
    Serial.println(BMP390_altitude_m, 5);

    Serial.print("filter started = ");
    Serial.println(BME_filter_started);
  }




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