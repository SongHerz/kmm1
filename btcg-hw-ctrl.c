#ifndef BTCG_SPI_DEFINES_H
#define BTCG_SPI_DEFINES_H

#include <assert.h>
#include <stdint.h> // For various data types

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
    f6f0 = clk_core * 2 / CLK_OSC - 1;
    assert( f6f0 <= PLL_CONF_MAX);
    return f6f0;
}

#endif
