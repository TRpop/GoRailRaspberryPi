#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <termios.h>
//------------------------------------------------------------------------------
// 설명 : 시리얼포트를 연다.
// 주의 : RTS/CTS 를 제어하지 않는다.
// 시리얼포트를 열고 이전의 포트설정상태를 저장하지 않았다.
//------------------------------------------------------------------------------
int open_serial( char *dev_name, int baud, int vtime, int vmin )
{
    int fd;
    struct termios newtio;
    // 시리얼포트를 연다.
    fd = open( dev_name, O_RDWR | O_NOCTTY );
	printf("open");
    if ( fd < 0 )
     {
         // 화일 열기 실패
         printf( "Device OPEN FAIL %s\n", dev_name );
         return -1;
     }
     // 시리얼 포트 환경을 설정한다.
     memset(&newtio, 0, sizeof(newtio));
     newtio.c_iflag = IGNPAR; // non-parity
     newtio.c_oflag = 0;
     newtio.c_cflag = CS8 | CLOCAL | CREAD; // NO-rts/cts 
     switch( baud )
     {
         case 115200 : newtio.c_cflag |= B115200; break;
         case 57600 : newtio.c_cflag |= B57600; break;
         case 38400 : newtio.c_cflag |= B38400; break;
         case 19200 : newtio.c_cflag |= B19200; break;
         case 9600 : newtio.c_cflag |= B9600; break;
         case 4800 : newtio.c_cflag |= B4800; break;
         case 2400 : newtio.c_cflag |= B2400; break;
         default : newtio.c_cflag |= B115200; break;
     }
    //set input mode (non-canonical, no echo,.....)
     newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime; // timeout 0.1초 단위
     newtio.c_cc[VMIN] = vmin; // 최소 n 문자 받을 때까진 대기
     tcflush ( fd, TCIFLUSH );
     tcsetattr( fd, TCSANOW, &newtio );
 
     return fd;
}
//------------------------------------------------------------------------------
// 설명 : 시리얼포트를 닫는다.
// 주의 :
//------------------------------------------------------------------------------
void close_serial( int fd )
{
     close( fd );
}
//------------------------------------------------------------------------------
// 설명 : main
// 32바이트의 데이타를 보낸후 데이타가 들어오는지를 1초동안 감시한다.
// 데이타가 일정시간(1초)동안 없으면 다시 데이타를 전송한다.
// 주의 :
//------------------------------------------------------------------------------
int main( int argc, char **argv )
{
     int fd; // 시리얼포트 파일핸들
     int baud; // 전송속도
     char dev_name[128]; // 시리얼포트 노드파일 이름
     char cc, buf[128]; // 데이타 버퍼
     int rdcnt;
     if ( argc != 3 )
     {
         printf( " sample_serial [device] [baud]\n" \
         " device : /dev/ttySAC0 ...\n" \
         " baud : 2400 ... 115200\n" );
         return -1;
     }
     printf( " Serial test start... (%s)\n", __DATE__ );
 
 // 인자를 얻어온다.
     strcpy( dev_name, argv[1] ); // 노드파일 이름
     baud = strtoul( argv[2], NULL, 10 ); // 전송속도
 // 시리얼 포트를 연다
 // 시리얼포트를 1초동안 대기하거나 32바이트 이상의 데이타가 들어오면
 // 깨어나도록 설정한다.
	printf("asdf\n");
     fd = open_serial( dev_name, baud, 10, 32 );
     if ( fd < 0 ) return -2;
	printf("opened");
     for ( cc='A'; cc<='z'; cc++ )
     {
         // 32바이트 데이타를 전송한다.
         memset( buf, cc, 2 );
         write( fd, buf, 2 );

	 usleep(1000); 
         // 데이타를 읽어온다.
         rdcnt = read( fd, buf, sizeof(buf) );
         if ( rdcnt > 0 )
         {
             buf[rdcnt] = '\0';
             printf( "<%s rd=%2d> %s\n", dev_name, rdcnt, buf );
         }
 
         // 테스트를 위한 지연
         //sleep(1);
	if(cc == 'z') cc = 'A';
     }
     // 시리얼 포트를 닫는다.
     close_serial( fd );
 
     printf( " Serial test end\n" );
 
     return 0;
}
