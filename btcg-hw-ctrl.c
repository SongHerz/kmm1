#include <assert.h>
#include <stdint.h> // For various data types
#include <termios.h>

#include "spi-context.h"

///////////////////////////////////////////////////
// SPI related hardware control
///////////////////////////////////////////////////

// SPI commands
#define CMD_CK  (uint8_t)(0)              // 2'b00xx_xxxx
#define CMD_RD  (uint8_t)(0x40)           // 2'b01xx_xxxx
#define CMD_WR  (uint8_t)(0x80)           // 2'b10xx_xxxx
#define CMD_RST (uint8_t)(0x40 | 0x80)    // 2'b11xx_xxxx

// SPI registers
#define PLL_ADDR    (uint8_t)(45)
#define STATUS_ADDR (uint8_t)(63)

// PLL frequency
// frequence in MHZ
#define CLK_OSC 20
#define CLK_CORE_MAX    400
#define CLK_CORE_MIN    200
#define PLL_CONF_MAX    0x7F    // 2'b0111_1111

static uint8_t pll_conf( int clk_core) {
    // CLK_CORE = CLK_OSC * (F6:F0 + 1) / 2
    // => F6:F0 = ( CLK_CORE * 2 / CLK_OSC) - 1
    assert( clk_core >= CLK_CORE_MIN && clk_core <= CLK_CORE_MAX);
    int f6f0 = clk_core * 2 / CLK_OSC - 1;
    assert( f6f0 <= PLL_CONF_MAX);
    return f6f0;
}

static bool __chip_sw_reset(struct spi_ctx *ctx) {
    uint8_t tx = CMD_RST;
    uint8_t dummy;

    return spi_transfer(ctx, &tx, &dummy, 1);
}

static bool __chip_set_pll(struct spi_ctx *ctx, int clk_core) {
    uint8_t tx[2];
    uint8_t dummy[2];

    tx[0] = CMD_WR | PLL_ADDR;
    tx[1] = 0x80 | pll_conf(clk_core);
    if (!spi_transfer(ctx, tx, dummy, sizeof(tx))) {
        return false;
    }

    tx[1] = 0x00 | pll_conf(clk_core);
    if (!spi_transfer(ctx, tx, dummy, sizeof(tx))) {
        return false;
    }
    // sleep 0.3 ms, after setting PLL
    usleep(300);
    return true;
}

bool chip_reset(struct spi_ctx *ctx) {
    return __chip_sw_reset(ctx) && __chip_set_pll(ctx, 200);
}

#define STATUS_W_ALLOW(status)  ((status) & 0x1)
#define STATUS_R_READY(status)  ((status) & 0x2)
#define STATUS_NONCE_GRP0_RDY(status)   (((status) >> 2) & 0x01)
#define STATUS_NONCE_GRP1_RDY(status)   (((status) >> 3) & 0x01)
#define STATUS_NONCE_GRP2_RDY(status)   (((status) >> 4) & 0x01)
#define STATUS_NONCE_GRP3_RDY(status)   (((status) >> 5) & 0x01)

bool chip_status(struct spi_ctx *ctx, int *status) {
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = CMD_WR | STATUS_ADDR;
    tx[1] = 0xff;   // any data

    if (!spi_transfer(ctx, tx, rx, sizeof(tx))) {
        return false;
    }
    *status = rx[1];
    return true;
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
