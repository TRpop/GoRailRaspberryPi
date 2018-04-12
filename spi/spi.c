/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <sys/mman.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int  mem_fd;
void *gpio_map;
static int spiFds[2];

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#define SPI_CHANNEL 0
#define SPI_SPEED   100000  // 1MHz

#define TARGET_CMD      0xA0                    // 대상 온도 커맨드
#define SENSOR_CMD      0xA1                    // 센서 온도 커맨드
#define LASER_CMD       0xA4                    // 레이저 ON 커맨드 

#define CS(a) (5+a)

uint8_t T_high_byte;
uint8_t T_low_byte;
uint8_t cEratio;  // 방사율 저장 변수
int  iTARGET, iSENSOR;  // 부호 2byte 온도 저장 변수 
volatile uint8_t Laser_Flag=0;


static void pabort(const char *s)
{
        perror(s);
        abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 3;             ////   TODO     //////////// Find right mode
static uint8_t bits = 8;
static uint32_t speed = SPI_SPEED;
static uint16_t delay = 10;

void setup_io();
int SEND_COMMAND(int, uint8_t, int);

static void transfer(int fd, uint8_t* tx, int len)
{
        int ret;

        //uint8_t rx[len] = {0, };
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)tx,
                .len = len,
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
        };

        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
                pabort("can't send spi message");
/*
        for (ret = 0; ret < len; ret++) {
                if (!(ret % 6))
                        puts("");
                printf("%.2X ", tx[ret]);
        }
        puts("");
*/
}

int main(int argc, char *argv[])
{
        int ret = 0;
        int fd;

        //parse_opts(argc, argv);

        fd = open(device, O_RDWR);
	if (fd < 0)
                pabort("can't open device");

        /*
         * spi mode
         */
        ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1)
                pabort("can't set spi mode");

        ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
        if (ret == -1)
                pabort("can't get spi mode");

        printf("mode : %d\n", ret);

        /*
         * bits per word
         */
        ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't set bits per word");

        ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't get bits per word");

        /*
         * max speed hz
         */
        ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1)
                pabort("can't set max speed hz");

        ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
        if (ret == -1)
                pabort("can't get max speed hz");

        printf("spi mode: %d\n", mode);
        printf("bits per word: %d\n", bits);
        printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	printf("setting gpio\n");
	setup_io();

	INP_GPIO(CS(0));
	OUT_GPIO(CS(0));

	INP_GPIO(CS(1));
	OUT_GPIO(CS(1));

	while(1){
                printf("Channel 0 : Target : %g, Sensor : %g\t",
			SEND_COMMAND(fd, TARGET_CMD, 0)/10.0,
			SEND_COMMAND(fd, SENSOR_CMD, 0)/10.0);
		
		printf("Channel 1 : Target : %g, Sensor : %g\n",
                        SEND_COMMAND(fd, TARGET_CMD, 1)/10.0,
                        SEND_COMMAND(fd, SENSOR_CMD, 1)/10.0);

		//sleep(1);
                //SEND_COMMAND(TARGET_CMD)
                //sleep(2);
                //SEND_COMMAND(fd, LASER_CMD);
                //sleep(1);
        }

	/*
        while(1){
                transfer(fd);
        }
	*/
	uint8_t tx[] = {0,1,2,3,4,5,6};
        transfer(fd, tx, ARRAY_SIZE(tx));
	sleep(1);

        close(fd);

        return ret;
}

void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }
 
   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );
 
   close(mem_fd); //No need to keep mem_fd open after mmap
 
   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }
 
   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;

} // setup_io

int SEND_COMMAND(int fd, uint8_t cCMD, int channel)    // 온도 READ 함수
{
    uint8_t buff[] = {cCMD, 0x22, 0x22};

	channel &= 1;

    GPIO_CLR = 1<<CS(channel);
	transfer(fd, buff, ARRAY_SIZE(buff));

    //SPIDataRW(0, buff, 4);
    T_low_byte = buff[1];
    T_high_byte = buff[2];

	T_low_byte &= 0xFF;
	T_high_byte &= 0xFF;

    GPIO_SET = 1<<CS(channel);
	usleep(20);
    return (int)(T_high_byte<<8 | T_low_byte);       // 상위, 하위 바이트 연산 
}
