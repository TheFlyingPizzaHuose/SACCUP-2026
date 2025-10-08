/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)
Avionics Main Code
Authors: Leiana Mendoza,

All pertinent information for this code is in document(s) linked below.
This includes all references and credits to the autors of code that is
used in this program such as libraries.

*/

#include <SoftwareSerial.h>   //UART Library
#include <Wire.h>             // I2C library
#include <SPI.h>              // SPI library
#include <SD.h>               // SD card library
#include <Adafruit_BMP085.h>  // BMP 085 or 180 pressure sensor library
#include <Adafruit_BMP280.h>  // BMP 280 pressure sensor library
#include <Adafruit_MPU6050.h> // MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h>  // Adafruit unified sensor library
#include <Adafruit_LSM9DS1.h> // Include LSM9DS1 library
#include <Adafruit_ADXL375.h> // Include ADXL375 library
#include <Adafruit_INA219.h>  // Include INA 219 library
#include <RH_RF95.h>          // Include RFM9X library
#include <iostream>
#include <cmath>
#include <map>
#include <EEPROM.h>
#include <uRTCLib.h> //Include Real Time Clock Library
#include <time.h>
using namespace std;

// Defining Variables
#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C
#define BMP_SCL 13
#define BMP_SDO 12
#define BMP_SDA 11

// UART Serial Objects
#define rfSerial Serial1
#define gpsSerial Serial2

// RFM9x pin assignments
#define RFM95_CS 10
#define RFM95_INT 9
#define RFM95_RST 26
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RMF95_INT);
const int RFM9X_PWR = 23;
// Pin assignments
const int video_pin = 2;
const int shutdown_pin = 3;
// SD card variables
const int sdSelect = BUILTIN_SDCARD;
char *logFileName;
File logfile;

cost bool callibration_mode = 0, debug = 0;
int16_t packetnum = 0; // packet counter, we increment per xmission

// Sensor objects
Adafruit_INA219 ina219;       // INA 219 Object
Adafruit_MPU6050 mpu;         // Accelerometer
Adafruit_BMP280 bmp280(&Wire0);  // Pitot Tube Pressure Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire);
Adafruit_ADXL275 adxl = Adafruit_ADXL375(12345, &Wire1);

static sensors_event_t LSM_acc, LSM_gyro, LSM_mag, LSM_temp;

// GPS Variables
int gpsVersion = 2; //SAM-M8Q
uint gpsStats, glonasStats, gallileoStats = 0, 0, 0;
uint8_t[] setGSVON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; // Calculates checkSum
uint8_t[] setGSVOFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};

// AV Modes
bool lowPower, videoPower, pwrGPSStatusPoll = 0, 0, 1;
int timeVidOff = 0;
int[] shutdownCheck[3] = {0, 0, 0} // All three elements must be >0 to activate shutdown
int[] restartCheck[3] = {0, 0, 0}  // All three elements must be >0 to activate restart

map<string, bool> errorStatus;
errorStatus.insert({"PRGM_ERROR", false},
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
                   {"MAG_CALLIB", false});

// Component cycle time miscros
uint ADXL345RATE = 350,
     AS5600RATE = 150,
     BMP180RATE = 5500, // Ultra low: 5.5, Low: 7.5, Standard: 11.5, High: 19.5, Ultra high: 37.5
     BMP280RATE = 5500, // Ultra low: 3, Standard: 5, High: 9, Ultra high: 17, Adv. High: 51
     DYNSTARTRATE = 100000,
     LSMRATE = 1500,
     MPU6050RATE = 125,
     RFDRATE = 44983,   // Prime number near 45ms
     RFMRATE = 50000,
     SAMM8QRATE = 100000,
     SDRATE = 10000,
     STATUSRATE = 10000;

// Component last cycle time micros
uint ADXL245LAST = 0,
     AS5600LAST = 0,
     BMP280LAST = 0,
     BMP180LAST = 0,
     DYNSTARTLAST = 0,
     SAMM8QLAST = 0,
     LSMLAST = 0,
     MPU6050 = 0,
     RFDLAST = 0,
     RFMLAST = 0,
     SDLAST = 0;

// Time variables
uint8_t timeLaunch = 0,
        timeMillis = 0,
        lastMiscros = 0;
float timeSinceLaunch = 0;

// Start variables


void printErr()
{
    for (auto const &[key, value] : errorStatus)
    {
        if (value == true)
        {
            Serial.print(key);
        }
    }
}

char *checkFile(bool mode = 0)
{
    int fileExists = 1;
    int fileNum = 0;
    static char filename[8] = "000.txt";
    while (fileExists)
    {
        fileName[0] = '0' + fileNum / 100;
        fileName[1] = '0' + (fileName % 100) / 10;
        fileName[2] = '0' + fileName % 10;
        File checkFile = SD.open(fileName);
        checkfile.seek(1 * 6);
        String content = checkFile.readStringUntil('\r');
    }
}

char *readyPacket()
{
    int bitArray[bitArrayLength] = {};
    static char charArray[charArrayLength] = {};
    int bitIndex = 0;
    int charIndex = 0;

    bitArray[1] = sender_indent;
    bitIndex++;

    for (int i = 0; i < 10; i++)
    {
        while (i > 8)
        {
            if (i == 0)
            {
                bitsPtr = time_since_launch;
            }
            else if (i == 2)
            {
                bitsPtr = BMP280_ALT;
            }
            else if (i == 3)
            {
                bitsPtr = position[1];
            }
            else if (i == 4)
            {
                bitsPtr = position[0];
            }
            else if (i == 5)
            {
                bitsPtr = sqrt(pow(velocity[0], 2) + pow(velocity[1], 2) + pow(velocity[2], 2));
            }
            else if (i == 6 || i == 7)
            {
                bitsPtr = 255;
            }
            for (int x = 0; x < dataLength; x++)
            {
                bitArray[bitIndex] = *(intoBinary(bitsPtr) + dataLength - 1 - x);
                bitIndex++;
            }
        }
        if (i == 8)
        {
            for (int x = 0; x < dataLength; x++)
            {
                bitArray[bitIndex] = errorCodes[x];
                bitIndex++;
            }
        }
        else if (i == 9)
        {
            charArray[charArrayLength - 3] = radioChecksum(bitArray, bitArrayLength);
            charArray[charArrayLength - 2] = 'R';
            charArray[charArrayLength - 1] = 'L';
        }
    }
    for (int i = 0; i < bitArrayLength; i += 8)
    {
        char result = 0;
        for (int j = 0; j <= 7; j++)
        {
            result |= (bitArray[i + j] << (7 - j));
        }
        charArray[charIndex] = result;
        charIndex++;
    }
    return charArray;
}

template <typename R>
string bitsOfUnsigned(R x, int width)
{
    string s(width, '0');
    for (int i = 0; i < width; i++)
    {
        s[width - 1 - i] = ((x >> i) & 1) ? "1" : "0";
    }
    return s;
}
temple<typename T>
    T intoBinary(T placeHolder)
{
    if constexpr (is_integral_v<placeHolder>)
    {
        using R = make_unsigned_t<T>; // unsigned is better for binary
        return bitsOfUnsigned((R)value, sizeof(placeHolder) * 8);
    }
    else if constexpr (is_floating_point_v<placeHolder>)
    {
        using R = conditional_t<sizeof(placeHolder) == 4, uint32_t,
                                conditional_t<sizeof(placeHolder) == 8, unit64_t,
                                              __uint128_t>>;
        R bits = 0;
        memcpy(&bits, &value, sizeof(placeHolder));
        return bitsOfUnsigned(bits, sizeof(placeHolder) * 8);
    }
    else
    {
        Serial.println("Array requested too large");
        setErr(PRGM_ERR);
    }
}
