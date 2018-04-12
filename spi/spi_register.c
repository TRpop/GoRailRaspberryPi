//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013
 
 
// Access from ARM Running Linux
 
#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
 
 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
 
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 
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
#define SPI_SPEED   1000000  // 1MHz

#define TARGET_CMD	0xA0			// 대상 온도 커맨드
#define SENSOR_CMD	0xA1			// 센서 온도 커맨드
#define LASER_CMD       0xA4                    // 레이저 ON 커맨드 

#define CS 5

unsigned char T_high_byte;
unsigned char T_low_byte;
unsigned char cEratio;  // 방사율 저장 변수
int  iTARGET, iSENSOR;	// 부호 2byte 온도 저장 변수 
volatile unsigned char Laser_Flag=0;

static unsigned char     spiMode   = 0 ;
static unsigned char     spiBPW    = 8 ;
static unsigned short    spiDelay  = 10;
static unsigned int    spiSpeeds [2] ;
 
void setup_io();
void setup_spi();
int SPIDataRW(int, unsigned char*, int);
int SEND_COMMAND(unsigned char);

 
int main(int argc, char **argv)
{ 
  // Set up gpi pointer for direct register access
  printf("setting gpio\n");
  setup_io();

  // Set up spi
  printf("setting spi\n");
  setup_spi();
 
  // Switch GPIO 7..11 to output mode
 
 /************************************************************************\
  * You are about to change the GPIO settings of your computer.          *
  * Mess this up and it will stop working!                               *
  * It might be a good idea to 'sync' before running this program        *
  * so at least you still have your code changes written to the SD-card! *
 \************************************************************************/
 
  // Set GPIO pins 7-11 to output
  
printf("pin%d setting\n", CS);
INP_GPIO(CS);
OUT_GPIO(CS);
printf("entering while\n");

	while(1){
		printf("Target : %g, Sensor : %g\n", SEND_COMMAND(TARGET_CMD)/10.0, SEND_COMMAND(SENSOR_CMD)/10.0);
		//SEND_COMMAND(TARGET_CMD)
                sleep(2);
		SEND_COMMAND(LASER_CMD);
		sleep(1);
	}
  printf("while skiped\n");
  return 0;
 
} // main
 
 
//
// Set up a memory regions to access GPIO
//
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

void setup_spi(){
	if ((spiFds[0] = open ("/dev/spidev0.0", O_RDWR)) < 0)
	{
		printf("can't open /dev/spidev0.0\n");
        	exit(-1);
     	}
}

int SPIDataRW (int channel, unsigned char *data, int len)
 {	 //SPI ����� spi_ioc_transfer ����ü��� �ϰ��� �������̽���
	 //���� SPI Master/Slave �� ����� �Ѵ�.
     struct spi_ioc_transfer spi ;

     channel &= 1 ;
	 // data�� ���� ������ ä���� ���� �о������ �Ѵ�.
	 // tx_buf�� MCP_3204�� Timing Diagram�� D0, D1, D2�� �ش��Ѵ�.
	 // tx_buf�� �״�� �̿��Ͽ� rx_buf�� 12bit ADC�� ���� �޴´�
     spi.tx_buf        = (unsigned long)data ;
     spi.rx_buf        = (unsigned long)data ;
     spi.len           = len ;
     spi.delay_usecs   = spiDelay ;
     spi.speed_hz      = SPI_SPEED;
     spi.bits_per_word = spiBPW ;
	 //  spi_ioc_transfer ����ü�� configuration�� �Ѵ�.

	 // SPI_IOC_MESSAGE(1) Command�� �����͸� ��ִ� ��ũ���� _IOW�� 
	 // ġȯ�ȴ�. �ڼ��� ������ Linux/include/uapi/linux/spi/spidev.h�� ����
	 // ioctl �Լ��� ���� ���� ��ɰ� ������ �ּҸ� ����̹��� ������.
     return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ;
 }

int SEND_COMMAND(unsigned char cCMD)    // 온도 READ 함수
{
	printf("SEND_COMMAND Entered\n");
    unsigned char buff[4] = {cCMD, 0x22, 0x22, 0x00};

    //digitalWrite(chipSelectPin , LOW);  // CS Low Level
    GPIO_CLR = 1<<CS;
    //delayMicroseconds(10);              // delay(10us)
    //SPI.transfer(cCMD);                // Send 1st Byte
	printf("Calling SPIDataRW\n");
    SPIDataRW(0, buff, 4);
	printf("SPIDataRW Done\n");
    //delayMicroseconds(10);                          // delay(10us)          
    //T_low_byte = SPI.transfer(0x22);   // Send 2nd Byte
    //delayMicroseconds(10);                          //delay(10us)  
    //T_high_byte = SPI.transfer(0x22);  // Send 3rd Byte
    //digitalWrite(chipSelectPin , HIGH); // CS High Level 
    T_low_byte = buff[1] & 0xFF;
    T_high_byte = buff[2] & 0xFF;

    GPIO_SET = 1<<CS;
	printf("returning Temp\n");
    return (T_high_byte<<8 | T_low_byte);	// 상위, 하위 바이트 연산 
}
