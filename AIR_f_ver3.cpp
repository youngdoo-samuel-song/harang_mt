/********************************************************************
2023 Harang-dentity Avionics Code
Target board : Raspberry Pi 4 (version 3 or less is NOT acceptable)
Library : wiringPi V2.7.0
V0.0.0 2023.03.21 ê°íì°, ê¹ë¯¼í, ê¹ì ë¹, ìµì¬ì
V1.0.0 2023.03.23 ê°íì°, ì´í¸ì§
TODO: imu ë°ì´í°ê° ìì¤í¤ ì½ëë¼ì íì¬ë ë¬¸ìì´ë¡ ì½ë ìíë¡ ì ì¥ì ê°ë¥íë ì½ëì íì© ë¶ê°.
      ","ë¥¼ ê¸°ì¤ì¼ë¡ ë¬¸ìì´ë¡ ë ê° ê°ì doubleíì¼ë¡ ê° ë³ìì ì ì¥íì¬ Aì pitch rateë¥¼ ì¬ì¶ ìëë¦¬ì¤ì ì¬ì©í  ì ìì´ì¼ íë¤.
********************************************************************/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
using namespace std;

#define MAX_NMEA0183_LEN 82 // GPS ë°ì´í° ê¸¸ì´
#define MAX_IMU_LEN 200     // IMU ë°ì´í° ê¸¸ì´
#define init_count 20       // ìµì´ê³ ë 20ë² ì¸¡ì 
#define drogue_port 17      // drogue ì¬ì¶ ì í¸ í¬í¸
#define main_port 23        // main ì¬ì¶ ì í¸ í¬í¸
#define I2C_address 0x76    // BMP390 I2C ì£¼ì
#define P0 1013.25          // Sealevel pressure, hPa

float init_temperature = 0;
float init_altitude = 0;
float max_altitude = 0;
int step, on = 0;       // ì¬ì¶ ì í¸ ì ì¥ì© step, loop íì±íì© on
//float A = 0;            // Accleration Vector, g
//float pitch_rate = 0;   // dps

// ì¼ì ìë ¥
float temperature = 0;           // BMP390 ì¨ë, C
float pressure = 0;              // BMP390 ìë ¥, hPa
float altitude = 0;              // BMP390 ê³ ë, m
/* ìë°ì´í¸ íì
double yaw, pitch, roll = 0;     // IMU ìì¸ ë°ì´í°, deg
double ax, ay, az = 0;           // IMU ê°ìë ë°ì´í°, g
double gx, gy, gz = 0;           // IMU ê°ìë ë°ì´í°, dps
double mx, my, mz = 0;           // IMU ì§ìê¸° ë°ì´í°, uT
*/
unsigned int imu_time_stamp = 0;     // ms
unsigned int gps_time_stamp = 0;
unsigned int baro_time_stamp = 0;


// bmp390 í¨ì
struct {                   // The BMP390 compensation trim parameters (coefficients)
    uint16_t param_T1;
    uint16_t param_T2;
    int8_t   param_T3;
    int16_t  param_P1;
    int16_t  param_P2;
    int8_t   param_P3;
    int8_t   param_P4;
    uint16_t param_P5;
    uint16_t param_P6;
    int8_t   param_P7;
    int8_t   param_P8;
    int16_t  param_P9;
    int8_t   param_P10;
    int8_t   param_P11;
}params;

struct FloatParams {       // The BMP390 float point compensation trim parameters
    float param_T1;
    float param_T2;
    float param_T3;
    float param_P1;
    float param_P2;
    float param_P3;
    float param_P4;
    float param_P5;
    float param_P6;
    float param_P7;
    float param_P8;
    float param_P9;
    float param_P10;
    float param_P11;
} floatParams;

int init_bmp390(void) {
    int fp = wiringPiI2CSetup(I2C_address); // I2C íµì  ì´ê¸°í
    if (fp < 0) return fp;

    while (!(wiringPiI2CReadReg8(fp, 0x0C) & 0b00000001));

    wiringPiI2CWriteReg8(fp, 0x1B, 0b00110011);  // normal mode, temp&pres on
    wiringPiI2CWriteReg8(fp, 0x1C, 0b00000001);  // *2 oversampling
    wiringPiI2CWriteReg8(fp, 0x1D, 0b00000001);  // 100Hzë¡ ì¤ì 
    wiringPiI2CWriteReg8(fp, 0x1F, 0b00000010);  // filter coeff. = 3
    wiringPiI2CWriteReg8(fp, 0x17, 0b00000000);

    params.param_T1 = (wiringPiI2CReadReg8(fp, 0x32) << 8) + wiringPiI2CReadReg8(fp, 0x31);
    params.param_T2 = (wiringPiI2CReadReg8(fp, 0x34) << 8) + wiringPiI2CReadReg8(fp, 0x33);
    params.param_T3 = wiringPiI2CReadReg8(fp, 0x35);
    params.param_P1 = (wiringPiI2CReadReg8(fp, 0x37) << 8) + wiringPiI2CReadReg8(fp, 0x36);
    params.param_P2 = (wiringPiI2CReadReg8(fp, 0x39) << 8) + wiringPiI2CReadReg8(fp, 0x38);
    params.param_P3 = wiringPiI2CReadReg8(fp, 0x3A);
    params.param_P4 = wiringPiI2CReadReg8(fp, 0x3B);
    params.param_P5 = (wiringPiI2CReadReg8(fp, 0x3D) << 8) + wiringPiI2CReadReg8(fp, 0x3C);
    params.param_P6 = (wiringPiI2CReadReg8(fp, 0x3F) << 8) + wiringPiI2CReadReg8(fp, 0x3E);
    params.param_P7 = wiringPiI2CReadReg8(fp, 0x40);
    params.param_P8 = wiringPiI2CReadReg8(fp, 0x41);
    params.param_P9 = (wiringPiI2CReadReg8(fp, 0x43) << 8) + wiringPiI2CReadReg8(fp, 0x42);
    params.param_P10 = wiringPiI2CReadReg8(fp, 0x44);
    params.param_P11 = wiringPiI2CReadReg8(fp, 0x45);

    // Calculate the floating point trim parameters
    floatParams.param_T1 = (float)params.param_T1 / powf(2.0f, -8.0f);
    floatParams.param_T2 = (float)params.param_T2 / powf(2.0f, 30.0f);
    floatParams.param_T3 = (float)params.param_T3 / powf(2.0f, 48.0f);
    floatParams.param_P1 = ((float)params.param_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    floatParams.param_P2 = ((float)params.param_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    floatParams.param_P3 = (float)params.param_P3 / powf(2.0f, 32.0f);
    floatParams.param_P4 = (float)params.param_P4 / powf(2.0f, 37.0f);
    floatParams.param_P5 = (float)params.param_P5 / powf(2.0f, -3.0f);
    floatParams.param_P6 = (float)params.param_P6 / powf(2.0f, 6.0f);
    floatParams.param_P7 = (float)params.param_P7 / powf(2.0f, 8.0f);
    floatParams.param_P8 = (float)params.param_P8 / powf(2.0f, 15.0f);
    floatParams.param_P9 = (float)params.param_P9 / powf(2.0f, 48.0f);
    floatParams.param_P10 = (float)params.param_P10 / powf(2.0f, 48.0f);
    floatParams.param_P11 = (float)params.param_P11 / powf(2.0f, 65.0f);

    return fp;
}

int get_bmp390_error(int fp) {
    return wiringPiI2CReadReg8(fp, 0x02);
}

int32_t get_bmp390_pres(int fp) {  //8+24 bit pressure data
    int32_t adcPres = (wiringPiI2CReadReg8(fp, 0x06) << 16) + (wiringPiI2CReadReg8(fp, 0x05) << 8) + wiringPiI2CReadReg8(fp, 0x04);
    return adcPres;
}

int32_t get_bmp390_temp(int fp) {  //8+24 bit temperature data
    int32_t adcTemp = (wiringPiI2CReadReg8(fp, 0x09) << 16) + (wiringPiI2CReadReg8(fp, 0x08) << 8) + wiringPiI2CReadReg8(fp, 0x07);
    return adcTemp;
}

float get_bmp390_altitude(float p0, float pressure) {
    return 44330 * (1 - powf(pressure / p0, 1 / 5.255));
}

// Compensate algorithm from BMP390 datasheet
float bmp390_compensate_temp(float uncomp_temp) {
    float partial_data1 = uncomp_temp - floatParams.param_T1;
    float partial_data2 = partial_data1 * floatParams.param_T2;
    return partial_data2 + partial_data1 * partial_data1 * floatParams.param_T3;
}

float bmp390_compensate_pres(float uncomp_press, float t_lin) {
    float partial_data1 = floatParams.param_P6 * t_lin;
    float partial_data2 = floatParams.param_P7 * t_lin * t_lin;
    float partial_data3 = floatParams.param_P8 * t_lin * t_lin * t_lin;
    float partial_out1 = floatParams.param_P5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = floatParams.param_P2 * t_lin;
    partial_data2 = floatParams.param_P3 * t_lin * t_lin;
    partial_data3 = floatParams.param_P4 * t_lin * t_lin * t_lin;
    float partial_out2 = uncomp_press * (floatParams.param_P1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = floatParams.param_P9 + floatParams.param_P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + uncomp_press * uncomp_press * uncomp_press * floatParams.param_P11;
    return partial_out1 + partial_out2 + partial_data4;
}

// ë©í° ì¤ë ë
PI_THREAD(GPS_THREAD) {  // GPS ë°ì´í°ë¥¼ sdì¹´ëì ì ì¥
    int gps_serial_port;
    char gps_data[MAX_NMEA0183_LEN];

    if ((gps_serial_port = serialOpen("/dev/ttyAMA1", 9600)) < 0) {
        fprintf(stderr, "Unable to open serial device (GPS): %s\n", strerror(errno));
    }

    ofstream gpsout;
    gpsout.open("GPS.txt");

    while(1){
	    gps_time_stamp = millis();
        if (serialDataAvail(gps_serial_port)) {
            for (int i = 0; i < MAX_NMEA0183_LEN; i++) {
                gps_data[i] = serialGetchar(gps_serial_port);
            }
            gpsout << gps_time_stamp << endl << gps_data << endl << endl;
            cout << gps_time_stamp << endl << gps_data << endl;
        }
    }

    gpsout.close();
}

PI_THREAD(MAIN_THREAD) {  // ë©ì¸ í¨ì, ëíì° ì¬ì¶ ìí, ì¬ì¶ ê´ë ¨ ë°ì´í°ë¥¼ sdì¹´ëì ì ì¥
    int fp = init_bmp390();

    // ì´ê¸° ê³ ë & ì¨ë ì¸¡ì 
    cout << "Barometer initializing..." << endl;
    for (int i = 0; i < init_count; i++) {
        temperature = bmp390_compensate_temp(get_bmp390_temp(fp));
        pressure = bmp390_compensate_pres(get_bmp390_pres(fp), temperature);
        pressure /= 100.0f;  // Calculate the pressure in hPa
        altitude = get_bmp390_altitude(P0, pressure);

        init_temperature += temperature;
        init_altitude += altitude;

        delay(floor(1000 / init_count));
    }
    init_altitude /= init_count;
    init_temperature /= init_count;

    ofstream fout;
    fout.open("AIR.txt");

    fout << "init temperature = " << init_temperature << " C, init altitude = " << init_altitude << endl;
    cout << "init temperature = " << init_temperature << " C, init altitude = " << init_altitude << endl;

    while(1) {
        temperature = bmp390_compensate_temp(get_bmp390_temp(fp));
        pressure = bmp390_compensate_pres(get_bmp390_pres(fp), temperature);
        pressure /= 100.0f;  // Calculate the pressure in hPa
        altitude = get_bmp390_altitude(P0, pressure) - init_altitude;  // ì´ê¸° ê³ ëë¥¼ 0më¡ ì¤ì 

        //A = sqrt(ax * ax + ay * ay + az * az);
        //pitch_rate = sqrt(gy * gy + gz * gz);

        // ëª¨í°ê° ì íëë©´ loop íì±í
        //if (A > 1.5 && on == 0) on = 1;
        if (altitude > 2 && on == 0) on = 1;

        // ì¬ì¶ ìëë¦¬ì¤
        if (on) {
            // ìì¹íë ëì ìµê³ ê³ ë ê°±ì 
            if (step == 0 && altitude > max_altitude) max_altitude = altitude;

            // ìµê³ ê³ ëìì 1.5m íê°íë©´ drogue ëíì° ì¬ì¶
            if (step == 0 && max_altitude - altitude > 1.5) {
                digitalWrite(drogue_port, HIGH);
                delay(100);
                //digitalWrite(drogue_port, LOW);
                step = 1;
            }

            // ìµê³ ê³ ëìì 50m íê°íë©´ main ëíì° ì¬ì¶
            if (step == 1 && max_altitude - altitude > 50) {
                digitalWrite(main_port, HIGH);
                delay(100);
                digitalWrite(main_port, LOW);
                step = 2;
            }
        }
	baro_time_stamp = millis();
        // SDì¹´ë ì ì¥
        fout << baro_time_stamp << ' ' << on << ' ' << altitude << ' ' << max_altitude << ' ' << step << endl;

        // ë°ì´í° ì¶ë ¥
        cout << baro_time_stamp << ' ' << on << ' ' << altitude << ' ' << max_altitude << ' ' << step << endl;

        delay(20);  // ê³ ë íì¸ 50Hz
    }

    fout.close();
}
/*
PI_THREAD(IMU_THREAD) {  // IMU ë°ì´í°ë¥¼ sdì¹´ëì ì ì¥
    int imu_serial_port;
    char imu_data[MAX_IMU_LEN];

    if ((imu_serial_port = serialOpen("/dev/ttyAMA4", 115200)) < 0) {
        fprintf(stderr, "Unable to open serial device (IMU): %s\n", strerror(errno));
    }

    ofstream imuout;
    imuout.open("IMU.txt");

    while(1) {
        if (serialDataAvail(imu_serial_port)) {
            for (int i = 0; i < MAX_IMU_LEN; i++) {
                imu_data[i] = serialGetchar(imu_serial_port);
                if (imu_data[i] == 10) break;  // ASCII code of "/n" is 10
            }
            imuout << "\b" << time_stamp << "," << imu_data;
            cout << "\b" << time_stamp << "," << imu_data;
        }
    }

    imuout.close();
}
*/

int main() {
    // ì´ê¸° ì¤ì 
    if (wiringPiSetup() == -1) {
        cout << "wiringPi setup Error" << endl;
        return -1;
    }
    pinMode(drogue_port, OUTPUT);
    pinMode(main_port, OUTPUT);

    int x = piThreadCreate(MAIN_THREAD);
    if (x != 0)
        printf("MAIN_THREAD ERROR!");
   /* int y = piThreadCreate(GPS_THREAD);
    if (y != 0)
        printf("GPS_THREAD ERROR!");
    int z = piThreadCreate(IMU_THREAD);
    if (z != 0)
        printf("IMU_THREAD ERROR!");
    */
    
    int imu_serial_port;
    char imu_data[MAX_IMU_LEN];

    if ((imu_serial_port = serialOpen("/dev/ttyAMA4", 9600)) < 0) {
        fprintf(stderr, "Unable to open serial device (IMU): %s\n", strerror(errno));
    }

    ofstream imuout;
    imuout.open("IMU.txt");
    
    while(1) {
	imu_time_stamp = millis();
        if (serialDataAvail(imu_serial_port)) {
            cout << imu_time_stamp << "/b";
	    for (int i = 0; i < MAX_IMU_LEN; i++) {
                imu_data[i] = serialGetchar(imu_serial_port);
                //if (imu_data[i] == 10) break;  // ASCII code of "/n" is 10
           	cout << int(imu_data[i]) << "\b";
	    }
	    cout << endl;
            imuout << "\b" << imu_time_stamp << "," << imu_data;
            //cout << "\b" << time_stamp << "," << imu_data;
        }
	//delayMicroseconds(100);
    }

    imuout.close();


    /*
    while (1) {
        time_stamp = millis();
        delayMicroseconds(100);  // us
    }
    */
    
    return 0;
}
