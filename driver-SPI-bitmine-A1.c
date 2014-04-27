/*
 * cgminer SPI driver for Bitmine.ch A1 devices
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"


FILE *fp_file;

int fp_uart;

///////////////////////////////////////////////////////////////////////////


/*
 * TODO: yes, we include a C file for now until we set up a framework
 * to support different variants of A1 products
 */

#define MAX_KM_IC	4
 
#define MAX_BOARDS 1
bool a1_board_selector_init() { return true; }
void a1_board_selector_exit() {}
void a1_board_selector_reset_all_boards() {}
void a1_board_selector_select_board(uint8_t b) {};
void unlock_board_selector() {}

///////////////////////////////////////////////////////////////////////////


/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL){
		applog(LOG_ERR, "queue nont init");
		return NULL;
	}
	if (wq->num_elems == 0){
		applog(LOG_ERR, "queue length is 0");
		return NULL;
	}
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}


struct work work_pool[32];
struct work *work_pool_p = work_pool;
unsigned work_state[32] ={
	0x000,0x00,0x00,0x00,0x000,0x00,0x00,0x00,
	0x000,0x00,0x00,0x00,0x000,0x00,0x00,0x00,
	0x000,0x00,0x00,0x00,0x000,0x00,0x00,0x00,
	0x000,0x00,0x00,0x00,0x000,0x00,0x00,0x00
};






/********** chip and chain context structures */
/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD	7

/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */
#define WRITE_JOB_LENGTH	58
#define MAX_CHAIN_LENGTH	64
/*
 * For commands to traverse the chain, we need to issue dummy writes to
 * keep SPI clock running. To reach the last chip in the chain, we need to
 * write the command, followed by chain-length words to pass it through the
 * chain and another chain-length words to get the ACK back to host
 */
#define MAX_CMD_LENGTH		(WRITE_JOB_LENGTH + MAX_CHAIN_LENGTH * 2 * 2)

struct A1_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[32];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

struct A1_chain {
	int board_id;
	struct cgpu_info *cgpu;
	int num_chips;
	int num_cores;
	int num_active_chips;
	int chain_skew;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *spi_ctx;
	struct A1_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;
};

enum A1_command {
	A1_BIST_START		= 0x01,
	A1_BIST_FIX		= 0x03,
	A1_RESET		= 0x04,
	A1_WRITE_JOB		= 0x07,
	A1_READ_RESULT		= 0x40,
	A1_WRITE_REG		= 0x80,
	A1_READ_REG		= 0x40,
	A1_READ_REG_RESP	= 0x1a,
};

/********** config paramters */
struct A1_config_options {
	int ref_clk_khz;
	int sys_clk_khz;
	int spi_clk_khz;
	/* limit chip chain to this number of chips (testing only) */
	int override_chip_num;
};

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
static struct A1_config_options config_options = {
	.ref_clk_khz = 16000, .sys_clk_khz = 800000, .spi_clk_khz = 2000,
};

/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_DEBUG, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

static void flush_spi(struct A1_chain *a1)
{
	/*memset(a1->spi_tx, 0, 64);
	applog(LOG_ERR, "spi tp7");
	spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 64);
	*/
}


/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct A1_chain *a1,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t len,
			  uint8_t resp_len)
{
	int tx_len = 4 + len;
	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = cmd;
	a1->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(a1->spi_tx + 2, data, len);
	applog(LOG_ERR, "spi tp8");
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);

	int poll_len = resp_len;
	if (chip_id == 0) {
		if (a1->num_chips == 0) {
			applog(LOG_ERR, "%d: unknown chips in chain, assuming 8",
			       a1->board_id);
			poll_len += 32;
		}
		poll_len += 4 * a1->num_chips;
	}
	else {
		poll_len += 4 * chip_id - 2;
	}
	applog(LOG_ERR, "spi tp9");
	assert(spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);
	int ack_len = tx_len + resp_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", a1->spi_rx + ack_pos, ack_len - 2);

	return (a1->spi_rx + ack_pos);
}


/********** A1 SPI commands */
static uint8_t *cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	uint8_t *ret = exec_cmd(a1, A1_BIST_FIX, 0x00, NULL, 0, 0);
	return ret;
}

static uint8_t *cmd_RESET_BCAST(struct A1_chain *a1, uint8_t strategy)
{
	static uint8_t s[2];
	s[0] = strategy;
	s[1] = strategy;
	uint8_t *ret = exec_cmd(a1, A1_RESET, 0x00, s, 2, 0);
	if (ret == NULL || (ret[0] != A1_RESET && a1->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", a1->board_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_READ_RESULT_BCAST(struct A1_chain *a1)
{
	
	int tx_len = 2;
	int tx_len2 = 8;
	int chip_num;
	char mask;
	//applog(LOG_ERR, "cmd_READ_RESULT_BCAST");
	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = 0x40|63;		//read statue reg
	a1->spi_tx[1] = 0xff;
	a1->spi_tx[3] = 0xff;
	a1->spi_tx[5] = 0xff;
	a1->spi_tx[7] = 0xff;
	a1->spi_tx[9] = 0xff;
	//applog(LOG_ERR, "spi transfer");
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	
	
	//applog(LOG_ERR, "read result is %x , %x " ,a1->spi_rx[0] , a1->spi_rx[1]);
	if((a1->spi_rx[1]&0x01) == 0x01){		//can write data to chip
		//applog(LOG_ERR, "can write data to chip" );
		a1->spi_rx[0] = chip_num;				//set a1->spi_rx[0] = chip ID
		return a1->spi_rx;						//return chip ID
	}
	else if((a1->spi_rx[1]&0x02) == 0x02){
		//applog(LOG_ERR, "nonce ready" );
		mask = (a1->spi_rx[1] >> 2);
		//applog(LOG_ERR, "mask is %x " , mask );
		if( ( (mask>>3) & 0x01) == 1 ){
			mask = 3;
		}else	if( ( (mask>>2) & 0x01) == 1 ){
			mask = 2;
		}else 	if( ( (mask>>1) & 0x01) == 1 ){
			mask = 1;
		}else	if( ( (mask>>0) & 0x01) == 1 ){
			mask = 0;
		}else {
			//time out
			a1->spi_rx;
		}
		//applog(LOG_ERR, "mask is %x " , mask );
		
		a1->spi_tx[2] = 0x40|(46+mask*4);
		a1->spi_tx[4] = 0x40|(47+mask*4);
		a1->spi_tx[6] = 0x40|(48+mask*4);
		a1->spi_tx[8] = 0x40|(49+mask*4);
		//applog(LOG_ERR, "address is %x %x %x %x" , a1->spi_tx[2] , a1->spi_tx[4] , a1->spi_tx[6] , a1->spi_tx[8] );
		bool retval = spi_transfer(a1->spi_ctx, (a1->spi_tx+2), (a1->spi_rx+2), tx_len);
		retval = spi_transfer(a1->spi_ctx, (a1->spi_tx+4), (a1->spi_rx+4), tx_len);
		retval = spi_transfer(a1->spi_ctx, (a1->spi_tx+6), (a1->spi_rx+6), tx_len);
		retval = spi_transfer(a1->spi_ctx, (a1->spi_tx+8), (a1->spi_rx+8), tx_len);
		//applog(LOG_ERR, "nonce is %x %x %x %x" , a1->spi_rx[3] , a1->spi_rx[5] , a1->spi_rx[7] , a1->spi_rx[9] );
		a1->spi_rx[0] = chip_num;				//nonce ready
		char cmdrst_tx[2];
		char cmdrst_rx[2];
		
		
		//sw reset here 
		cmdrst_tx[0] = 0xff;
		cmdrst_tx[1] = 0xff;
		int delay;
		int flag;
		
		//
		
		spi_transfer(a1->spi_ctx, cmdrst_tx, cmdrst_rx , 2);
		//config pll 
		cmdrst_tx[0] = 0xad;
		cmdrst_tx[1] = 0x9d;
	


		spi_transfer(a1->spi_ctx, cmdrst_tx, cmdrst_rx , 2);
		
		
		//wait for pll
		for(delay = 0 ; delay < 100000; delay ++){
			flag++;
			if( flag > 999 )
				break;
		}
		cmdrst_tx[0] = 0xad;
		cmdrst_tx[1] = 0x1d;

			spi_transfer(a1->spi_ctx, cmdrst_tx, cmdrst_rx , 2);
		
		//wait for pll
		for(delay = 0 ; delay < 100000; delay ++){
			flag++;
			if( flag > 999 )
				break;
		}
		
		
		return a1->spi_rx;						//read 4 bytes nonce
	} else {									//chip busy
		return a1->spi_rx;
	}
	
	/*
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);

	int poll_len = tx_len + 4 * a1->num_chips;
	retval = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len);
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);
	
	uint8_t *scan = a1->spi_rx;
	int i;
	for (i = 0; i < poll_len; i += 2) {
		if ((scan[i] & 0x0f) == A1_READ_RESULT) {
			return scan + i;
		}
	}
	*/
	//applog(LOG_ERR, "%d: cmd_READ_RESULT_BCAST failed", a1->board_id);
	//return NULL;
}

static uint8_t *cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	uint8_t *ret = exec_cmd(a1, A1_WRITE_REG, chip, reg, 6, 0);
	if (ret == NULL || ret[0] != A1_WRITE_REG) {
		applog(LOG_ERR, "%d: cmd_WRITE_REG failed", a1->board_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	uint8_t *ret = exec_cmd(a1, A1_READ_REG, chip, NULL, 0, 6);
	if (ret == NULL || ret[0] != A1_READ_REG_RESP || ret[1] != chip) {
		applog(LOG_ERR, "%d: cmd_READ_REG chip %d failed",
		       a1->board_id, chip);
		return NULL;
	}
	memcpy(a1->spi_rx, ret, 8);
	return ret;
}



static uint8_t *cmd_WRITE_JOB(struct A1_chain *a1, uint8_t chip_id,
			      uint8_t *job)
{

	//uint8_t cmd = (a1->spi_tx[0] << 8) | a1->spi_tx[1];
	/* ensure we push the SPI command to the last chip in chain */
	int tx_len = 90;// + 2;
	memcpy(a1->spi_tx, job, 90);
	//memset(a1->spi_tx + WRITE_JOB_LENGTH, 0, tx_len - WRITE_JOB_LENGTH);
	//applog(LOG_ERR, "spi tp3");
	
	
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	char test_tx[2];
	char test_rx[2];
	int i;
	/*
	for( i = 0; i <= 44 ; i++){
		test_tx[0] = 0x40|i;
		test_tx[1] = 0xaa;
		retval = spi_transfer(a1->spi_ctx, test_tx, test_rx, 2);
		applog(LOG_ERR, "read reg %d is %x %x" , i ,  test_rx[0] , test_rx[1]);
	}
	*/
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);
	int poll_len = 4 * chip_id - 2;
	int ack_len = tx_len;
	int ack_pos = tx_len + poll_len - ack_len;
	uint8_t *ret = a1->spi_rx + ack_pos;
	/*
	int poll_len = 4 * chip_id - 2;

	retval = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len);
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);

	int ack_len = tx_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", a1->spi_rx + ack_pos, tx_len);

	uint8_t *ret = a1->spi_rx + ack_pos;
	if (ret[0] != a1->spi_tx[0] || ret[1] != a1->spi_tx[1]){
		applog(LOG_ERR, "%d: cmd_WRITE_JOB failed: "
			"0x%02x%02x/0x%02x%02x", a1->board_id,
			ret[0], ret[1], a1->spi_tx[0], a1->spi_tx[1]);
		//return NULL;
	}
	*/
	return ret;
	
}

/********** A1 low level functions */
static bool A1_hw_reset(void)
{
	/*
	 * TODO: issue cold reset
	 *
	 * NOTE: suggested sequence
	 * a) reset the RSTN pin for at least 1s
	 * b) release the RSTN pin
	 * c) wait at least 1s before sending the first CMD
	 */
	if (!a1_board_selector_init()) {
		applog(LOG_ERR, "Failed to init board selector");
		a1_board_selector_exit();
		return false;
	}
	a1_board_selector_reset_all_boards();
	return true;
}

#define MAX_PLL_WAIT_CYCLES 25
#define PLL_CYCLE_WAIT_TIME 40
static bool check_chip_pll_lock(struct A1_chain *a1, int chip_id, uint8_t *wr)
{
	int n;
	for (n = 0; n < MAX_PLL_WAIT_CYCLES; n++) {
		/* check for PLL lock status */
		if (cmd_READ_REG(a1, chip_id) && (a1->spi_rx[4] & 1) == 1)
			/* double check that we read back what we set before */
			return wr[0] == a1->spi_rx[2] && wr[1] == a1->spi_rx[3];

		cgsleep_ms(PLL_CYCLE_WAIT_TIME);
	}
	applog(LOG_ERR, "%d: chip %d failed PLL lock", a1->board_id, chip_id);
	return false;
}

static uint8_t *get_pll_reg(struct A1_chain *a1, int ref_clock_khz,
			    int sys_clock_khz)
{
	/*
	 * TODO: this is only an initial approach with binary adjusted
	 * dividers and thus not exploiting the whole divider range.
	 *
	 * If required, the algorithm can be adapted to find the PLL
	 * parameters after:
	 *
	 * sys_clk = (ref_clk * pll_fbdiv) / (pll_prediv * 2^(pll_postdiv - 1))
	 *
	 * with a higher pll_postdiv being desired over a higher pll_prediv
	 */

	static uint8_t writereg[6] = { 0x00, 0x00, 0x21, 0x84, };
	uint8_t pre_div = 1;
	uint8_t post_div = 1;
	uint32_t fb_div = sys_clock_khz / ref_clock_khz;
	int bid = a1->board_id;

	applog(LOG_WARNING, "%d: Setting PLL: CLK_REF=%dMHz, SYS_CLK=%dMHz",
	       bid, ref_clock_khz / 1000, sys_clock_khz / 1000);

	while (fb_div > 511) {
		if (post_div < 4)
			post_div++;
		else
			pre_div <<= 1;
		fb_div >>= 1;
	}
	if (pre_div > 31) {
		applog(LOG_WARNING, "%d: can't set PLL parameters", bid);
		return NULL;
	}
	writereg[0] = (post_div << 6) | (pre_div << 1) | (fb_div >> 8);
	writereg[1] = fb_div & 0xff;
	applog(LOG_WARNING, "%d: setting PLL: pre_div=%d, post_div=%d, fb_div=%d"
	       ": 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", bid,
	       pre_div, post_div, fb_div,
	       writereg[0], writereg[1], writereg[2],
	       writereg[3], writereg[4], writereg[5]);
	return writereg;
}

static bool set_pll_config(struct A1_chain *a1, int chip_id,
			   int ref_clock_khz, int sys_clock_khz)
{
	uint8_t *writereg = get_pll_reg(a1, ref_clock_khz, sys_clock_khz);
	if (writereg == NULL)
		return false;
	if (!cmd_WRITE_REG(a1, chip_id, writereg))
		return false;

	int from, to;
	if (chip_id == 0) {
		from = 0;
		to = a1->num_active_chips;
	} else {
		from = chip_id - 1;
		to = chip_id - 1;
	}
	int i;
	for (i = from; i < to; i++) {
		int cid = i + 1;
		if (!check_chip_pll_lock(a1, chip_id, writereg)) {
			applog(LOG_ERR, "%d: chip %d failed PLL lock",
			       a1->board_id, cid);
			return false;
		}
	}
	return true;
}

#define WEAK_CHIP_THRESHOLD	30
#define BROKEN_CHIP_THRESHOLD	26
#define WEAK_CHIP_SYS_CLK	(600 * 1000)
#define BROKEN_CHIP_SYS_CLK	(400 * 1000)
static bool check_chip(struct A1_chain *a1, int i)
{
	int chip_id = i + 1;
	int bid = a1->board_id;
	
	
	
	/*if (!cmd_READ_REG(a1, chip_id)) {
		applog(LOG_WARNING, "%d: Failed to read register for "
		       "chip %d -> disabling", bid, chip_id);
		a1->chips[i].num_cores = 0;
		a1->chips[i].disabled = 1;
		return false;;
	}
	
	a1->chips[i].num_cores = a1->spi_rx[7];
	a1->num_cores += a1->chips[i].num_cores;
	applog(LOG_WARNING, "%d: Found chip %d with %d active cores",
	       bid, chip_id, a1->chips[i].num_cores);
	*/
	/*jump check chiip
	if (a1->chips[i].num_cores < BROKEN_CHIP_THRESHOLD) {
		applog(LOG_WARNING, "%d: broken chip %d with %d active "
		       "cores (threshold = %d)", bid, chip_id,
		       a1->chips[i].num_cores, BROKEN_CHIP_THRESHOLD);
		set_pll_config(a1, chip_id, config_options.ref_clk_khz,
				BROKEN_CHIP_SYS_CLK);
		cmd_READ_REG(a1, chip_id);
		hexdump_error("new.PLL", a1->spi_rx, 8);
		a1->chips[i].disabled = true;
		a1->num_cores -= a1->chips[i].num_cores;
		return false;
	}

	if (a1->chips[i].num_cores < WEAK_CHIP_THRESHOLD) {
		applog(LOG_WARNING, "%d: weak chip %d with %d active "
		       "cores (threshold = %d)", bid,
		       chip_id, a1->chips[i].num_cores, WEAK_CHIP_THRESHOLD);
		set_pll_config(a1, chip_id, config_options.ref_clk_khz,
			       WEAK_CHIP_SYS_CLK);
		cmd_READ_REG(a1, chip_id);
		hexdump_error("new.PLL", a1->spi_rx, 8);
		return false;
	}
	*/
	return true;
}

/*
 * BIST_START works only once after HW reset, on subsequent calls it
 * returns 0 as number of chips.
 */
static int chain_detect(struct A1_chain *a1)
{
	int tx_len = 6;

	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = A1_BIST_START;
	a1->spi_tx[1] = 0;
	applog(LOG_ERR, "spi tp4");
	if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len))
		return 0;
	hexdump("TX", a1->spi_tx, 6);
	hexdump("RX", a1->spi_rx, 6);

	int i;
	int bid = a1->board_id;
	int max_poll_words = MAX_CHAIN_LENGTH * 2;
	for(i = 1; i < max_poll_words; i++) {
		if (a1->spi_rx[0] == A1_BIST_START && a1->spi_rx[1] == 0) {
			applog(LOG_ERR, "spi tp6");
			spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
			hexdump("RX", a1->spi_rx, 2);
			uint8_t n = a1->spi_rx[1];
			a1->num_chips = (i / 2) + 1;
			if (a1->num_chips != n) {
				applog(LOG_ERR, "%d: enumeration: %d <-> %d",
				       bid, a1->num_chips, n);
				if (n != 0)
					a1->num_chips = n;
			}
			applog(LOG_WARNING, "%d: detected %d chips",
			       bid, a1->num_chips);
			return a1->num_chips;
		}
		applog(LOG_ERR, "spi tp5");
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
			return 0;
	}
	applog(LOG_WARNING, "%d: no A1 chip-chain detected", bid);
	return 0;
}

/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id)
{
	struct A1_chip *chip = &a1->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct A1_chain *a1, uint8_t chip_id)
{
	flush_spi(a1);
	struct A1_chip *chip = &a1->chips[chip_id - 1];
	int bid = a1->board_id;
	if (is_chip_disabled(a1, chip_id)) {
		applog(LOG_WARNING, "%d: chip %d already disabled",
		       bid, chip_id);
		return;
	}
	applog(LOG_WARNING, "%d: temporary disabling chip %d", bid, chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct A1_chain *a1)
{
	int i;
	int bid = a1->board_id;
	for (i = 0; i < a1->num_active_chips; i++) {
		int chip_id = i + 1;
		struct A1_chip *chip = &a1->chips[i];
		if (!is_chip_disabled(a1, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;
		/*if (!cmd_READ_REG(a1, chip_id)) {
			chip->fail_count++;
			applog(LOG_WARNING, "%d: chip %d not yet working - %d",
			       bid, chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) {
				applog(LOG_WARNING,
				       "%d: completely disabling chip %d at %d",
				       bid, chip_id, chip->fail_count);
				chip->disabled = true;
				a1->num_cores -= chip->num_cores;
				continue;
			}
			
			chip->cooldown_begin = get_current_ms();
			continue;
		}
		*/
		applog(LOG_WARNING, "%d: chip %d is working again",
		       bid, chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
	}
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	int i,j;
	static uint8_t job_2[90];
	static uint8_t job[WRITE_JOB_LENGTH] = {
		/* command */
		0x00, 0x00,
		/* midstate */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* wdata */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty 1 */
		0xff, 0xff, 0x00, 0x1d,
		/* end nonce */
		0xff, 0xff, 0xff, 0xff,
	};
	uint8_t *midstate = work->midstate;
	uint8_t *wdata = work->data + 64;
		
	//uint8_t *midstate = work->midstate;
	//uint8_t *wdata = work->data + 64;

	uint32_t *p1 = (uint32_t *) &job[34];
	uint32_t *p2 = (uint32_t *) wdata;

	job[0] = (job_id << 4) | A1_WRITE_JOB;
	job[1] = chip_id;

	//applog(LOG_ERR, " bef swap %x %x %x %x", *midstate,*(midstate+1),*(midstate+2),*(midstate+3));
	swab256(job + 2, midstate);
	p1[0] = bswap_32(p2[0]);
	p1[1] = bswap_32(p2[1]);
	p1[2] = bswap_32(p2[2]);
	p1[4] = get_diff(work->sdiff);
	//applog(LOG_ERR, " aft swap %x %x %x %x",job[30],job[31],job[32],job[33]);
	
	for( i = 0,j=0; i < 44,j<88 ; i++,j=j+2 ){
		job_2[j] = 0x80|i;
		if(i<32) 
			job_2[j+1] = *(midstate+i);
		else 
			job_2[j+1] = *(wdata+i-32);//job[2+32+i];
		//applog(LOG_ERR, " job is %x ",job_2[j+1]);
	}
	
	//applog(LOG_ERR, " job is %x %x %x %x",job_2[1],job_2[3],job_2[5],job_2[7]);
	job_2[88] = 0x80|44;
	job_2[89] = 0x80|44;
	
	
	//fprintf(fp_file , "set:job\r\n" );
	//fprintf(fp_file , "address\r\n" );
	//for(i=0;i<45;i++){
		//fprintf(fp_file , " %02x " ,job_2[i*2] );
	//}
	//fprintf(fp_file , "\r\n" );
	//fprintf(fp_file , "reg data \r\n" );
	//for(i=0;i<45;i++){
		//fprintf(fp_file , " %02x " ,job_2[i*2+1]);
	//}
	//fprintf(fp_file , "\r\n" );
	return job_2;
	
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work,
		     uint8_t queue_states)
{
	struct A1_chip *chip = &a1->chips[2 - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;
	uint8_t *jobdata = create_job(chip_id, job_id, work);
	
	//applog(LOG_ERR, " calc work is %x %x , %x ，%x",  *(jobdata+1), *(jobdata+3) , *(jobdata+5) , *(jobdata+7));
	cmd_WRITE_JOB(a1, chip_id, jobdata);
	return retval;
}

static char get_nonce(struct A1_chain *a1, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_READ_RESULT_BCAST(a1);
	//applog(LOG_ERR, " ret is %x ", ret[1]);
	if (ret == NULL)
		return false;
	if ((ret[1]&0x03) == 0x00) {
		//applog(LOG_ERR, "busy");
		return 0;
	}
	if ( (ret[1]&0x01) == 0x01 )
	{
		//applog(LOG_ERR, "allow and send work");
		return 1;
	}
	if ( (ret[1]&0x02) == 0x02 ){
		//applog(LOG_ERR, "read nonce");
		
	
		*chip = 2; //read from uart and so on..
		*job_id = *chip - 1;
		//applog(LOG_ERR, "job id is %d\n" , (*job_id));
		//add read nonce here
		//memcpy(nonce, ret + 2, 4);
		*nonce = *(ret+9);
		*(nonce + 1) = *(ret+7);
		*(nonce + 2) = *(ret+5);
		*(nonce + 3) = *(ret+3);
		//fprintf(fp_file, " calc nonoce is %x %x %x %x" , (*nonce),*(nonce+1),*(nonce+2),*(nonce+3) );
		return 2;
	}
	return -1;
}

/* reset input work queues in chip chain */
static bool abort_work(struct A1_chain *a1)
{
	/* drop jobs already queued: reset strategy 0xed */
	return cmd_RESET_BCAST(a1, 0xed);
}

/********** driver interface */
void exit_A1_chain(struct A1_chain *a1)
{
	if (a1 == NULL)
		return;
	free(a1->chips);
	a1->chips = NULL;
	spi_exit(a1->spi_ctx);
	a1->spi_ctx = NULL;
	free(a1);
}

struct A1_chain *init_A1_chain(struct spi_ctx *ctx, int board_id)
{
	int i;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	assert(a1 != NULL);

	applog(LOG_DEBUG, "%d: A1 init chain", a1->board_id);
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->board_id = board_id;

	//a1->num_chips = chain_detect(a1);
	a1->num_chips = 32;
	if (a1->num_chips == 0)
		goto failure;
	//set pll here by maych
	for(i=0;i<a1->num_chips;i++)
	{	
		//set pll point 1
	}
	
	
	
	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->board_id, a1->num_chips);
	//jump set pll
	//if (!set_pll_config(a1, 0, config_options.ref_clk_khz,
	//		    config_options.sys_clk_khz))
	//	goto failure;

	/* override max number of active chips if requested */
	a1->num_active_chips = a1->num_chips;
	if (config_options.override_chip_num > 0 &&
	    a1->num_chips > config_options.override_chip_num) {
		a1->num_active_chips = config_options.override_chip_num;
		applog(LOG_WARNING, "%d: limiting chain to %d chips",
		       a1->board_id, a1->num_active_chips);
	}

	a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
	assert (a1->chips != NULL);

	//if (!cmd_BIST_FIX_BCAST(a1))
	//	goto failure;

	for (i = 0; i < a1->num_active_chips; i++)
		check_chip(a1, i);

	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       a1->board_id, a1->num_active_chips, a1->num_cores);

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:
	exit_A1_chain(a1);
	return NULL;
}

static bool A1_detect_one_chain(struct spi_config *cfg)
{
	struct cgpu_info *cgpu;
	int board_id;

	for (board_id = 0; board_id < MAX_BOARDS; board_id++) {
		struct spi_ctx *ctx = spi_init(cfg);

		if (ctx == NULL)
			return false;

		applog(LOG_WARNING, "checking board %d...", board_id);
//		a1_board_selector_select_board(board_id);
//		a1_board_selector_reset_board();

		struct A1_chain *a1 = init_A1_chain(ctx, board_id);
		unlock_board_selector();
		if (a1 == NULL)
			continue;

		cgpu = malloc(sizeof(*cgpu));
		assert(cgpu != NULL);

		memset(cgpu, 0, sizeof(*cgpu));
		cgpu->drv = &bitmineA1_drv;
		cgpu->name = "BitmineA1";
		cgpu->threads = 1;

		cgpu->device_data = a1;

		a1->cgpu = cgpu;
		add_cgpu(cgpu);
	}

	return true;
}

#define MAX_SPI_BUS	1
#define MAX_SPI_CS	1
/* Probe SPI channel and register chip chain */
void A1_detect(bool hotplug)
{
	int bus;
	int cs_line;
	int board_id;

	/* no hotplug support for now */
	if (hotplug)
		return;

	if (opt_bitmine_a1_options != NULL && parsed_config_options == NULL) {
		int ref_clk = 0;
		int sys_clk = 0;
		int spi_clk = 0;
		int override_chip_num = 0;

		sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d",
		       &ref_clk, &sys_clk, &spi_clk,  &override_chip_num);
		if (ref_clk != 0)
			config_options.ref_clk_khz = ref_clk;
		if (sys_clk != 0)
			config_options.sys_clk_khz = sys_clk;
		if (spi_clk != 0)
			config_options.spi_clk_khz = spi_clk;
		if (override_chip_num != 0)
			config_options.override_chip_num = override_chip_num;

		/* config options are global, scan them once */
		parsed_config_options = &config_options;
	}

	applog(LOG_ERR, "A1 detect");
	A1_hw_reset();
	
	for (bus = 0; bus < MAX_SPI_BUS; bus++) {
		for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
			struct spi_config cfg = default_spi_config;
			cfg.mode = SPI_MODE_1;
			cfg.speed = config_options.spi_clk_khz * 1000;
			cfg.bus = bus;
			cfg.cs_line = cs_line;
			A1_detect_one_chain(&cfg);
		}
	}
	
	//config all pll here
	int i;
	for(i=0;i<32;i++){
		
	
	}
	
	
	
}





static bool ctrl_chip_cs( unsigned char chip )
{
	//uart send chip number
	int i;
	int j;
	pthread_mutex_t lock;
	//mutex_lock(&lock);
	j=0;
	usleep(1500);
	if( write(fp_uart , &chip , 1) ){
		//mutex_unlock(&lock);
		usleep(800);
		return true;
	}
	else {
		//mutex_unlock(&lock);
		usleep(800);
		return false;
	}
}





static int64_t A1_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;
	int32_t nonce_ranges_processed = 0;

	//a1_board_selector_select_board(a1->board_id);
	fp_file = fopen("a.txt","ab+");
	applog(LOG_DEBUG, "A1 running scanwork");
	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	//uint8_t job_id;
	bool work_updated = false;

	mutex_lock(&a1->lock);

	int bid = a1->board_id;
	int res;
	uint8_t c = i;
	uint8_t *wdata;// work->data + 64;
	
	chip_id = 2;
	applog(LOG_ERR, "start");
	uint8_t qstate = a1->spi_rx[1] & 3;
	uint8_t qbuff = a1->spi_rx[6];
	struct work *work;
	struct A1_chip *chip = &a1->chips[2 - 1];
	applog(LOG_ERR, "SPI fd2 is  %x", a1->spi_ctx->fd);
	while(1)
	{
			//work_pool_p = &work_pool[0];
re_req:		
			//fil work here
			for(i=0;i< MAX_KM_IC;i++){
				work_pool_p = &work_pool[i];
				//applog(LOG_ERR, "check work %d state ID %d  buf address is %x", i , work_state[i] , work_pool_p);
				if( work_state[i] == 0 ){
					work = get_work(thr, thr->id);
					work_pool[i] = *work;
					work_state[i] = 1;
					ctrl_chip_cs(i);
					set_work(a1, c, (work_pool_p) , qbuff);	
					applog(LOG_ERR, "get and set work ID %d state %d  buf address is %x data is %x, %x , %x , %x", i , work_state[i] , work_pool_p , 
						*(work_pool_p->data),*(work_pool_p->data+1),*(work_pool_p->data+2),*(work_pool_p->data+3));
				} 
			}		
			int try_time =0;
			int j;
			for(j=0;j<MAX_KM_IC;j++){
				ctrl_chip_cs(j);
				work_pool_p = &work_pool[j];
				//applog(LOG_ERR, "get nonce %d state %d  buf address is %x", j , work_state[j] , work_pool_p);
				res	= get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id);				
				switch(res) {
				case 3: //inv state
					goto rec_out;
					break;
				case 2:			//ready
				if( nonce == 0x00)
				{
					applog(LOG_ERR, "time out");
					work_state[j] = 0;
				} else {
					nonce = bswap_32(nonce);
					nonce = nonce + 1;
					if( (work != NULL) && (nonce!=NULL)){
						for(i=0;i<4;i++){
							if(!submit_nonce(thr, work_pool_p, nonce)){
								applog(LOG_ERR, "get nonce ID %d state %d  buf address is %x", j , work_state[j] , work_pool_p);
								applog(LOG_ERR, "hw err nonce is %x" , nonce);
								nonce = nonce + 1;
							} else{
								applog(LOG_ERR, " submit nonce ok ID is %d nonce is %x addr is %x" , j ,nonce , work_pool_p);
								work_state[j] = 0;
								applog(LOG_ERR, "change 2 state %d 0" , j);
								//goto rec_out;
								goto submit_end;
								break;
							}
						}
						nonce = nonce  - 8;
						for(i=0;i<4;i++){
							if(!submit_nonce(thr, work_pool_p, nonce)){
								applog(LOG_ERR, "get nonce ID %d state %d  buf address is %x", j , work_state[j] , work_pool_p);
								applog(LOG_ERR, "hw err nonce is %x" , nonce);
								nonce = nonce + 1;
							} else {
								work_state[j] = 0;
								applog(LOG_ERR, "change 2 state %d 0" , j);
								applog(LOG_ERR, " submit nonce ok nonce ID is %d is %x addr is " , j , nonce , work_pool_p);
								//goto rec_out;
								break;
							}
						}
							work_state[j] = 0;
							applog(LOG_ERR, "change 1 state %d 0" , j);
							applog(LOG_ERR, "submit nonce end");
						} else {
							applog(LOG_ERR, "submit end");
							goto rec_out;
						}
submit_end:				
					break;
				case 0:
					//applog(LOG_ERR, " chip busy");
					break;
				case 1:		//allow change chip state
					applog(LOG_ERR, "allow send");
					work_state[j] = 0;
					applog(LOG_ERR, "change state %d 0" , j);
					break;
				default:
					applog(LOG_ERR, "error state");
					break;
		
				}
			//ctrl_chip_cs(3);
			}
		}
	}	
	
rec_out:
	mutex_unlock(&a1->lock);
	if (nonce_ranges_processed < 0)
		nonce_ranges_processed = 0;
	if (nonce_ranges_processed != 0) {
		applog(LOG_DEBUG, "%d, nonces processed %d",
		       bid, nonce_ranges_processed);
	}
	/* in case of no progress, prevent busy looping */
	if (!work_updated)
		cgsleep_ms(160);
	
	return (int64_t)nonce_ranges_processed << 32;
}


/* queue two work items per chip in chain */
static bool A1_queue_full(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int queue_full = false;
	struct work *work;

	mutex_lock(&a1->lock);
	applog(LOG_DEBUG, "%d, A1 running queue_full: %d/%d",
	       a1->board_id, a1->active_wq.num_elems, a1->num_active_chips);

	if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
		queue_full = true;
	else{
		//push queue
		applog(LOG_ERR," queue elem add and num is %d",a1->active_wq.num_elems);
		wq_enqueue(&a1->active_wq, get_queued(cgpu));
	}
	mutex_unlock(&a1->lock);

	return queue_full;
}

static void A1_flush_work(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int bid = a1->board_id;
	a1_board_selector_select_board(bid);

	applog(LOG_DEBUG, "%d: A1 running flushwork", bid);

	int i;

	mutex_lock(&a1->lock);
	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", bid);
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < a1->num_active_chips; i++) {
		int j;
		struct A1_chip *chip = &a1->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
			       bid, i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
		chip->last_queued_id = 0;
	}
	/* flush queued work */
	applog(LOG_DEBUG, "%d: flushing queued work...", bid);
	while (a1->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&a1->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&a1->lock);

	unlock_board_selector();
}

static void A1_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	tailsprintf(buf, len, " %2d:%2d/%3d ",
		    a1->board_id, a1->num_active_chips, a1->num_cores);
}

struct device_drv bitmineA1_drv = {
	.drv_id = DRIVER_bitmineA1,
	.dname = "BitmineA1",
	.name = "BA1",
	.drv_detect = A1_detect,

	.hash_work = hash_queued_work,
	.scanwork = A1_scanwork,
	.queue_full = A1_queue_full,
	.flush_work = A1_flush_work,
	.get_statline_before = A1_get_statline_before,
};
