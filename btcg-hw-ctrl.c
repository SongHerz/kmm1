#include <assert.h>
#include <stdint.h> // For various data types
#include <termios.h>

///////////////////////////////////////////////////
// SPI related hardware control
///////////////////////////////////////////////////

// SPI commands
#define CMD_CK    (uint8_t)(0)              // 2'b00xx_xxxx
#define CMD_RD    (uint8_t)(0x40)           // 2'b01xx_xxxx
#define CMD_WR    (uint8_t)(0x80)           // 2'b10xx_xxxx
#define CMD_RST   (uint8_t)(0x40 | 0x80)    // 2'b11xx_xxxx

// PLL frequency
// frequence in MHZ
#define CLK_OSC 20
#define CLK_CORE_MAX    400
#define CLK_CORE_MIN    200
#define PLL_CONF_MAX    0x7F    // 2'b0111_1111
uint8_t pll_conf( int clk_core) {
    // CLK_CORE = CLK_OSC * (F6:F0 + 1) / 2
    // => F6:F0 = ( CLK_CORE * 2 / CLK_OSC) - 1
    assert( clk_core >= CLK_CORE_MIN && clk_core <= CLK_CORE_MAX);
    int f6f0 = clk_core * 2 / CLK_OSC - 1;
    assert( f6f0 <= PLL_CONF_MAX);
    return f6f0;
}


///////////////////////////////////////////////////
// UART related hardware control
///////////////////////////////////////////////////
static int fp_uart;

static bool set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) 
    { 
        //perror("SetupSerial 1");
        return false;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD; 
    newtio.c_cflag &= ~CSIZE; 

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':                     //odd parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //even parity
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    //no parity
        newtio.c_cflag &= ~PARENB;
        break;
    }

switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        return false;
    }
    return true;
}

bool chip_selector_init() {
    fp_uart = open("/dev/ttyAMA0", O_RDWR);
    return fp_uart >= 0 && set_opt( fp_uart, 115200, 8, 'N', 1);
}

bool chip_select(uint8_t n) {
	usleep(1500);
    int ret = write(fp_uart , &n , 1);
    usleep( 800);
    return ret == 1;
}
