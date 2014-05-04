#include <assert.h>
#include <stdint.h> // For various data types
#include <termios.h>

#include "spi-context.h"

#if 1
#include <pthread.h>
#include "logging.h"
#endif

///////////////////////////////////////////////////
// SPI related hardware control
///////////////////////////////////////////////////

// SPI commands
#define CMD_CK  (uint8_t)(0)              // 2'b00xx_xxxx
#define CMD_RD  (uint8_t)(0x40)           // 2'b01xx_xxxx
#define CMD_WR  (uint8_t)(0x80)           // 2'b10xx_xxxx
#define CMD_RST (uint8_t)(0x40 | 0x80)    // 2'b11xx_xxxx

// SPI registers
#define MIDSTATE_BASE_ADDR  (uint8_t)(0)    // 0 ~ 31
#define DATA_BASE_ADDR      (uint8_t)(32)   // 32 ~ 43
#define START_WORK_ADDR     (uint8_t)(44)
#define PLL_ADDR            (uint8_t)(45)
#define NONCE_GRP_BASE_ADDR(n)  (uint8_t)(46 + (n)*4)
#define CLEAN_ADDR  (uint8_t)(62)
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
#define STATUS_BUSY(status)     (((status) & 0x3) == 0)
#define STATUS_NONCE_GRP0_RDY(status)   (((status) >> 2) & 0x01)
#define STATUS_NONCE_GRP1_RDY(status)   (((status) >> 3) & 0x01)
#define STATUS_NONCE_GRP2_RDY(status)   (((status) >> 4) & 0x01)
#define STATUS_NONCE_GRP3_RDY(status)   (((status) >> 5) & 0x01)
#define STATUS_NONCE_NO_GRP_RDY(status) ((((status) >> 2) & 0x0f) == 0)

bool chip_status(struct spi_ctx *ctx, uint8_t *status) {
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = CMD_RD | STATUS_ADDR;
    tx[1] = 0xff;   // any data

    if (!spi_transfer(ctx, tx, rx, sizeof(tx))) {
        return false;
    }
    *status = rx[1];
    return true;
}


#define JOB_LENGTH 90
/*
 * midstate: a 32-byte array
 * wdata: a 12-byte array
 */
static uint8_t *__create_job( const uint8_t *midstate, const uint8_t *wdata)
{
	static uint8_t job[JOB_LENGTH];

    // FIXME: What about diff field in the work
	// p1[4] = get_diff(work->sdiff);
    int i;
    int j;
    for ( i = 0; i < 32; ++i) {
        job[2*i] = CMD_WR | (MIDSTATE_BASE_ADDR + i);
        job[2*i + 1] = *(midstate + i);
    }
    assert( 2*i == 64);

    for ( j = 0; j < 12; ++j) {
        job[2*i + 2*j] = CMD_WR | (DATA_BASE_ADDR + j);
        job[2*i + 2*j + 1] = *(wdata + j);
    }
    assert( 2*i + 2*j == 88);
    job[88] = CMD_WR | START_WORK_ADDR;
    job[89] = 0xff;   // any value is fine.

    return job;
}


bool chip_write_job(struct spi_ctx *ctx, const uint8_t *midstate, const uint8_t *wdata) {
#if 1
    applog(LOG_ERR, "pthread id: %x", pthread_self());
#endif
    uint8_t *tx = __create_job( midstate, wdata);
    assert( tx);
    uint8_t dummy[JOB_LENGTH];
#if 0
    return spi_transfer(ctx, tx, dummy, JOB_LENGTH);
#else
    size_t i = 0;
    bool succ = true;
    for (i = 0; i < JOB_LENGTH; i += 2) {
        succ &= spi_transfer( ctx, tx + i, dummy, 2);
#if 1
        unsigned diffs = 0;
        if ( i < 88) {
            // Make sure what written is actually written.
            // read back the byte just written
            uint8_t itx[2] = { (0x3f & tx[i]) | CMD_RD, 0xff};
            uint8_t irx[2];
            bool r = spi_transfer( ctx, itx, irx, 2);
            assert( r) ;
            if ( tx[i + 1] != irx[1]) {
                applog(LOG_ERR, "diff pos: %zd", i);
                diffs += 1;
            }
        }
        if ( diffs != 0) {
            applog(LOG_ERR, "job diffs: %d", diffs);
        }
#endif
    }
    return succ;
#endif
}

// Read one nonce, according to group number.
// grp: 0, 1, 2, 3
// Write nonce to *nonce
static bool __chip_read_nonce(struct spi_ctx *ctx, const unsigned int grp, uint32_t *nonce) {
    assert( ctx);
    assert( nonce);
    assert( grp >= 0 && grp <= 3);

    uint8_t tx[8];
    uint8_t rx[8];

    memset( tx, 0, sizeof( tx));

    const uint8_t base = NONCE_GRP_BASE_ADDR(grp);
#if 0
    applog(LOG_ERR, "Grp base addr %u", base);
#endif
    tx[0] = CMD_RD | base;
    tx[2] = CMD_RD | base + 1;
    tx[4] = CMD_RD | base + 2;
    tx[6] = CMD_RD | base + 3;

#if 0
    if (!spi_transfer( ctx, tx, rx, sizeof(tx))) {
        return false;
    }
#else
    size_t i = 0;
    bool succ = true;
    for ( i = 0; i < 8; i += 2) {
        succ &= spi_transfer( ctx, tx + i, rx + i, 2);
    }
    if (!succ) {
        return false;
    }
#endif

    uint8_t *p = (uint8_t*)nonce;
    *p = rx[1];
    *(p + 1) = rx[3];
    *(p + 2) = rx[5];
    *(p + 3) = rx[7];

    return true;
}

bool chip_read_nonce(struct spi_ctx *ctx, const unsigned int grp, uint32_t *nonce) {
    uint32_t nonce0;
    uint32_t nonce1;

    if ( !__chip_read_nonce( ctx, grp, &nonce0)
            || !__chip_read_nonce( ctx, grp, &nonce1)) {
        return false;
    }

    if ( nonce0 == nonce1) {
        *nonce = nonce0;
        return true;
    }

    uint32_t nonce2;
    if ( !__chip_read_nonce( ctx, grp, &nonce2)) {
        return false;
    }

    if ( nonce0 == nonce2 || nonce1 == nonce2) {
        *nonce = nonce2;
        return true;
    }
    return false;
}

bool chip_clean(struct spi_ctx *ctx) {
    assert( ctx);

    uint8_t tx[2] = { CMD_RD | CLEAN_ADDR, 0xff /* any value*/ };
    uint8_t dummy[2];

    return spi_transfer( ctx, tx, dummy, sizeof( tx));
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
