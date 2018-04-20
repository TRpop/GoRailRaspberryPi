/*
	MPU6050 Interfacing with Raspberry Pi
	http://www.electronicwings.com
*/

#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>

#define MLX90614_TA 		0x06
#define MLX90614_TOBJ1 		0x07

#define Device_Address 0x5A	/*Device Address/Identifier for MPU6050*/

int fd;

short read_raw_data(int addr){
	return wiringPiI2CReadReg16(fd, addr);
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}

int main(){
	
	fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/

	double temp;

	while(1)
	{
		temp = read_raw_data(MLX90614_TOBJ1)*0.02-273.15;//Temperature conversion

		printf("temperature : %g\n", temp);

		delay(500);
		
	}
	return 0;
}
