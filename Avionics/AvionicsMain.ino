  /*
  Saint Louis University Rocket Propulsion Laboratory (SLURPL)
  Avionics Main Code
  Authors: Leiana Mendoza,

  All pertinent information for this code is in document(s) linked below.
  This includes all references and credits to the autors of code that is
  used in this program such as libraries.
  
  */

  #include <SoftwareSerial.h>
  #include <iostream>
  #include "Adafruit_BMP085.h"
  #include <Adafruit_BMP280.h>
  #include <iostream>
  #include <map>
  using namespace std;


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

void printErr()
{
    for (auto const& [key, value] : errorStatus)
    {
        if (value == true)
        {
            Serial.print(key);
        }
    }
}

char* readyPacket()
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
                bitsPtr  = position[0];
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
        if (i == 8){
            for (int x = 0; x < dataLength; x++) {
                bitArray[bitIndex] = errorCodes[x];
                bitIndex++;
            }
        }
        else if (i == 9) {
            charArray[charArrayLength - 3] = radioChecksum(bitArray, bitArrayLength);
            charArray[charArrayLength - 2] = 'R';
            charArray[charArrayLength - 1] = 'L';
        }
    }
    for (int i = 0; i < bitArrayLength; i += 8) {
        char result = 0;
        for (int j = 0; j <= 7; j++) {
            result |= (bitArray[i + j] << (7-j));
        }
        charArray[charIndex] = result;
        charIndex++;
    }
    return charArray;
}

template <typename R>
string bitsOfUnsigned(R x, int width) {
    string s(width, '0');
    for (int i = 0; i < width; i++) {
        s[width - 1 - i] = ((x >> i) & 1) ? "1" : "0";
    }
    return s;
}
temple <typename T>
T intoBinary(T placeHolder) {
    if constexpr (is_integral_v<placeHolder>) {
        using R = make_unsigned_t<T>; //unsigned is better for binary
        return bitsOfUnsigned((R)value, sizeof(placeHolder) * 8);
    } else if constexpr (is_floating_point_v<placeHolder>) {
        using R = conditional_t<sizeof(placeHolder) == 4, uint32_t,
                conditional_t<sizeof(placeHolder) == 8, unit64_t,
                __uint128_t>>;
        R bits = 0;
        memcpy(&bits, &value, sizeof(placeHolder));
        return bitsOfUnsigned(bits, sizeof(placeHolder) * 8);
    } else {
        Serial.println("Array requested too large");
        setErr(PRGM_ERR);
    }
}
