#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string.h>
#include <errno.h>
#include <chrono>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
using namespace std;
using namespace chrono;

#define MAX_NMEA0183_LEN 82 // GPS 데이터 길이
#define init_count 20       // 최초고도 20번 측정
#define drogue_port 17      // drogue 사출 신호 포트
#define main_port 27        // main 사출 신호 포트

int init_press = 0;
float max_altitude = 0;
int step, on = 0;      // 사출 신호 저장용 step, loop 활성화용 on
float A = 0;           // Accleration Vector, g
float pitch_rate = 0;  // dps

// 센서 입력
float altitude = 0;              // 고도계 데이터
double ax, ay, az = 0;           // IMU 가속도 데이터, g
double gx, gy, gz = 0;           // IMU 각속도 데이터, dps
double mx, my, mz = 0;           // IMU 지자기 데이터, uT
long time_stamp = 0;             // ms

/*
센서 데이터 읽어오는 함수를 일단 대충 지어내서 코딩했으니 나중에 수정해야 함
bmp.altitude()  고도계 고도
imu.accel()     IMU 가속도
imu.gyro()      IMU 각속도
imu.mag()       IMU 지자기
gps.disp()      gps 변위
gps.time()      gps real time
*/

// 타임스탬프
static long getNowTimeStamp() {
    milliseconds Ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return Ms.count();
}

// bmp390 함수
int init_bmp390(void) {
    int fp = wiringPiI2CSetup(0x76);
    if (fp < 0) return fp;

    while (!(wiringPiI2CReadReg8(fp, 0x0C) & 0b00000001));

    wiringPiI2CWriteReg8(fp, 0x1B, 0b00110011);
    wiringPiI2CWriteReg8(fp, 0x1C, 0b00000001);
    wiringPiI2CWriteReg8(fp, 0x1D, 0b00000001);
    wiringPiI2CWriteReg8(fp, 0x1F, 0b00000010);
    wiringPiI2CWriteReg8(fp, 0x17, 0b00000000);

    return fp;
}

int get_bmp390_error(int fp) {
    return wiringPiI2CReadReg8(fp, 0x02);
}

int get_bmp390_press(int fp) {
    return (wiringPiI2CReadReg8(fp, 0x06) << 16) + (wiringPiI2CReadReg8(fp, 0x05) << 8) + wiringPiI2CReadReg8(fp, 0x04);
}

int get_bmp390_temp(int fp) {
    return (wiringPiI2CReadReg8(fp, 0x09) << 16) + (wiringPiI2CReadReg8(fp, 0x08) << 8) + wiringPiI2CReadReg8(fp, 0x07);
}

float get_bmp390_altitude(int fp, int p0) {
    int press;
    press = get_bmp390_press(fp);
    return 44330 * (1 - pow((float)press / p0, 1 / 5.255));
}

int main() {
    // 초기 설정
    if (wiringPiSetup() == -1) {
        cout << "wiringPi setup Error" << endl;
        return -1;
    }
    pinMode(drogue_port, OUTPUT);
    pinMode(main_port, OUTPUT);

    int fp = init_bmp390();

    int gps_serial_port;
    char gps_data[MAX_NMEA0183_LEN];
    if ((gps_serial_port = serialOpen("/dev/ttyAMA1", 9600)) < 0) {
        fprintf(stderr, "Unable to open serial device (GPS): %s\n", strerror(errno));
        return 1;
    }

    long init_time = getNowTimeStamp();

    ofstream fout;
    fout.open("AIR.txt");

    // 초기 기압 측정
    int pressure;
    cout << "Barometer initializing..." << endl;
    for (int i = 0; i < init_count; i++) {
        pressure = get_bmp390_press(fp);
        init_press += pressure;
        delay(floor(1000 / init_count));
    }
    init_press /= init_count;

    while (1) {
        // 센서 데이터 read
        time_stamp = getNowTimeStamp() - init_time;
        altitude = get_bmp390_altitude(fp, init_press);
        /*
        ax = imu.accel(x);
        ay = imu.accel(y);
        az = imu.accel(z);
        gx = imu.gyro(x);
        gy = imu.gyro(y);
        gz = imu.gyro(z);
        mx = imu.mag(x);
        my = imu.mag(y);
        mz = imu.mag(z);
        */
        if (serialDataAvail(gps_serial_port)) {
            for (int i = 0; i < MAX_NMEA0183_LEN; i++) {
                gps_data[i] = serialGetchar(gps_serial_port);
            }
        }

        A = sqrt(ax * ax + ay * ay + az * az);
        pitch_rate = sqrt(gx * gx + gy * gy);

        // 모터가 점화되면 loop 활성화
        if (A > 1.5 && on == 0) on = 1;

        // 사출 시나리오
        if (on) {
            // 상승하는 동안 최고고도 갱신
            if (step == 0 && altitude > max_altitude) max_altitude = altitude;
            
            // 최고고도에서 1.5m 하강하면 drogue 낙하산 사출
            if (step == 0 && max_altitude - altitude > 1.5) {
                digitalWrite(drogue_port, HIGH);
                step = 1;
            }

            // drogue 전개 후 고도 100m 이하 및 pitch_rate 20dps 이하면 main 낙하산 사출
            if (step == 1 && altitude < 100 && pitch_rate < 20) {
                digitalWrite(main_port, HIGH);
                step = 2;
            }
        }

        // SD카드 저장
        fout << time_stamp << on << altitude << ax << ay << az << gz << gy << gz << mx << my << mz << gps_data << A << pitch_rate << step << endl;

        // 데이터 출력
        cout << time_stamp << on << altitude << ax << ay << az << gz << gy << gz << mx << my << mz << gps_data << A << pitch_rate << step << endl;
    }

    fout.close();
    return 0;
}
