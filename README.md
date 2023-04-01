# harang_mt




ver2 가 전체가 다 들어가 있느 코드이고 

ver3 과 imu 코드가 삽질해보려고 제작한 코드입니다. 



g++ -o air AIR_f_ver2.cpp -lwiringPi -lm 

wiringPi 

git clone https://github.com/WiringPi/WiringPi.git
cd wiringPi 
./build 

inside cpp file 
#include <wiringPi.h>
g++ flags -> -lwiringPi 
