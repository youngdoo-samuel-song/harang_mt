#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <fstream>
#include <iostream>


#define MAX_IMU_LEN 10

using namespace std;


int main()
{
	int imu_serial_port;
	char imu_data[MAX_IMU_LEN];
	int time_stamp;

	if ((imu_serial_port = serialOpen("/dev/ttyAMA4", 9600)) < 0)		
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
	}
		
	ofstream imuout;
	imuout.open("IMU.txt");
	
	while(1){	
		time_stamp = millis();
		if(serialDataAvail(imu_serial_port)){
			cout << time_stamp << "    ";
			for(int i=0; i<MAX_IMU_LEN; ++i){
				imu_data[i] = serialGetchar(imu_serial_port);
				cout << int(imu_data[i]) << "    " ;
			}
			cout << endl;
			//cout << time_stamp << endl << *imu_data << endl;
			imuout << time_stamp << endl << *imu_data << endl;		
		}	
	}
	
	imuout.close();
}
