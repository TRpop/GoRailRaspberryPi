 #include <stdio.h>
 #include <string.h>
 #include <errno.h>
 #include <linux/spi/spidev.h>
 #include <fcntl.h>
 #include <sys/mman.h>

 ////////////////

 #include <stdint.h>
 #include <stdlib.h>
 #include <ctype.h>
 #include <poll.h>
 #include <unistd.h>
 #include <errno.h>
 #include <string.h>
 #include <time.h>
 #include <pthread.h>
 #include <sys/time.h>
 #include <sys/stat.h>
 #include <sys/wait.h>
 #include <sys/ioctl.h>

 ///////////////


 #define INPUT            0
 #define OUTPUT           1
 #define LOW              0
 #define HIGH             1


 #define BLOCK_SIZE      (4*1024)
 #define BCM2708_PERI_BASE 0x20000000
 #define GPIO_BASE       (BCM2708_PERI_BASE + 0x00200000)

 #define SPI_CHANNEL 0
 #define SPI_SPEED   1000000  // 1MHza

 static unsigned char     spiMode   = 0 ;
 static unsigned char     spiBPW    = 8 ;
 static unsigned short    spiDelay  = 0;
 static unsigned int    spiSpeeds [2] ;

 static volatile unsigned int *gpio ;
 static int         spiFds [2] ;

 // 25�� GPIO ���� Fuction Select�� �Ѵ�.
 void pin_25_Mode( int mode);
 // Write���� Fuction Select�� 25�� GPIO��
 // HIGH �Ǵ� LOW ���� �Է��Ѵ�.
 void digitalWrite_pin_25(int value);
 // ADC�� ���ڷ� �ѱ� ä�η� ���� 12bit ���� �о�´�.
 int read_mcp3204_adc(unsigned char adcChannel);
 // read_mcp_3204_adc �Լ� ���ο��� �θ��鼭 ���������� SPI ����� �ϴ� �Լ�
 int SPIDataRW (int channel, unsigned char *data, int len);
 
 int main (void)
 {
     int adcChannel = 0;
     int adcValue   = 0;
     int fd;

     if ((fd = open ("/dev/mem", O_RDWR | O_SYNC) ) < 0)
     {
          return -1 ;
     }
	// 25�� GPIO�� MCP_3204�� SPI Master���� Chip Select ��ȣ�� ���̸�
	// ��� ����ȭ�� ���ߴ� ������ �Ѵ�.MCP_3204�� Timing Diagram�� ����/
     gpio = (unsigned int *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE,
             MAP_SHARED, fd, GPIO_BASE) ;

     if ((int)gpio == -1)
     {
         return -1 ;
     }
 
	// ���������� ��� �ֺ���ġ(Peripheral)�� ���Ϸ� �����ȴ�.
	// spi�� ���õ� ������ open�ϰ� �ʿ��� �帮�̹� API�� ����ϰ� �ȴ�.
     if ((fd = open ("/dev/spidev0.0", O_RDWR)) < 0)
     {
         printf("/dev/spidev0.0\n");
         return 1 ;
     }

     spiFds    [0] = fd ;

     pin_25_Mode(OUTPUT); // CS ��ȣ�� ����ϱ� ���ؼ� OUTPUT���� 
						  // Fuction Set �Ѵ�.

     while(1)
     {
		 // ���� 1�� ���� �о� �帰��. �̶� 300 - ((adcValue*500)>>10
		 // �� �µ� ���� ������ü���� �����ϴ� �����ϸ��� ���̴�.
         adcValue = read_mcp3204_adc(adcChannel);
         printf("adc0 Value = %u\n", 300 - ((adcValue*500)>>10));
         sleep(1);
     }

     return 0;

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
     spi.speed_hz      = 1000000;
     spi.bits_per_word = spiBPW ;
	 //  spi_ioc_transfer ����ü�� configuration�� �Ѵ�.

	 // SPI_IOC_MESSAGE(1) Command�� �����͸� ��ִ� ��ũ���� _IOW�� 
	 // ġȯ�ȴ�. �ڼ��� ������ Linux/include/uapi/linux/spi/spidev.h�� ����
	 // ioctl �Լ��� ���� ���� ��ɰ� ������ �ּҸ� ����̹��� ������.
     return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ;
 }

 int read_mcp3204_adc(unsigned char adcChannel)
 {
     unsigned char buff[3];
     int adcValue = 0;

	 // MCP3204���� ����� ä�� ��ȣ�� buff[0]�� buff[1]��
	 // ���� �����Ѵ�.
     buff[0] = 0x06 | ((adcChannel & 0x07) >> 2 );
     buff[1] = ((adcChannel & 0x07) << 6);
     buff[2] = 0x00;

     digitalWrite_pin_25(0);  // Low : CS Active

     SPIDataRW(SPI_CHANNEL, buff, 3);

     buff[1] = 0x0F & buff[1];
     adcValue = ( buff[1] << 8) | buff[2];

     digitalWrite_pin_25(1);  // High : CS Inactive

     return adcValue;
 }



 void pin_25_Mode(int mode)
 {
     int fSel, shift, alt ;

       fSel    = 2;
       shift   = 15;

	 if (mode == INPUT){
			*(gpio + fSel ) = (*(gpio + fSel) & ~(7 << shift)) ; // Sets bits to     zero = input
     }
	 else if (mode == OUTPUT){
			*(gpio + fSel ) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift) ;
			printf("OUTPUT\n");
	 }
 }

 void digitalWrite_pin_25(int value)
 {
     int pin =25;
     int gpCLR = 10;
     int gpSET = 7;

     if (value == LOW)
         *(gpio + gpCLR) = 1 << (pin & 31) ;
     else
         *(gpio + gpSET) = 1 << (pin & 31) ;
}
