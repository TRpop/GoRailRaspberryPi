/*
	MPU6050 Interfacing with Raspberry Pi
	http://www.electronicwings.com
*/

#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>

//Arduino Command
#define GET_ENCODER             0x01
#define GO_FRONT                0x02
#define GO_BACK                 0x04
#define STOP                    0x08
#define OPEN_VALVE              0x10
#define CLOSE_VALVE             0x20

#define Device_Address 42 	/*Device Address/Identifier for MPU6050*/
int fd;

unsigned short read_raw_data(int addr){
	return wiringPiI2CReadReg16(fd, addr);
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}

int main(){
	
	fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/

	int temp;

	int i = 0;

	while(1)
	{
		if(i == 0){
			temp = read_raw_data(GET_ENCODER);//Temperature conversion

                	printf("encoder : %d\t", temp);
		}
		if(i == 1){
			temp = read_raw_data(GO_FRONT);//Temperature conversion

                	printf("go front : 2 : %d\t", temp);
		}
		if(i == 2){
			temp = read_raw_data(STOP);//Temperature conversion

                	printf("stop : 8 : %d\n", temp);
		}

		delay(100);
		
		i++;
		i = i%3;
	}
	return 0;
}
