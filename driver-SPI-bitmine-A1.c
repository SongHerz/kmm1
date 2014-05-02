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


//////////////////////////////////////////////
// TODO: Now, a C file is included
/////////////////////////////////////////////
#include "btcg-hw-ctrl.c"

///////////////////////////////////////////////////////////////////////////


#define MAX_KM_IC	2
 
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


/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
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


/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct A1_chain *a1, struct work *work)
{
	bool retval = false;

    if (!chip_write_job( a1->spi_ctx, work->midstate, work->data + 64)) {
        applog( LOG_ERR, "Failed to write job to chip");
    }
	return retval;
}

static char get_1st_nonce(struct A1_chain *a1, uint32_t *nonce) {
    uint8_t status;
    bool ret = chip_status( a1->spi_ctx, &status);
    assert( ret);

    if ( STATUS_R_READY( status)) {
        unsigned int grp = 0;
        if ( STATUS_NONCE_GRP3_RDY( status)) {
            grp = 3;
        }
        else if ( STATUS_NONCE_GRP2_RDY( status)) {
            grp = 2;
        }
        else if ( STATUS_NONCE_GRP1_RDY( status)) {
            grp = 1;
        }
        else {
            assert( STATUS_NONCE_GRP0_RDY( status));
            grp = 0;
        }

        bool ret2 = chip_read_nonce( a1->spi_ctx, grp, nonce);
        assert( ret2);
        bool ret3 = chip_reset( a1->spi_ctx);
        assert( ret3);
        return 2;
    } else if ( STATUS_W_ALLOW( status)) {
        return 1;
    }
    else {  // chip busy
        return 0;
    }
}

/* reset input work queues in chip chain */
static bool abort_work(struct A1_chain *a1)
{
	/* drop jobs already queued: reset strategy 0xed */
	// return cmd_RESET_BCAST(a1, 0xed);
    return true;
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

struct A1_chain *init_A1_chain(struct spi_ctx *ctx)
{
    const int board_id = 0;
	int i;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	assert(a1 != NULL);

	applog(LOG_DEBUG, "%d: A1 init chain", a1->board_id);
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->board_id = board_id;

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

	/* override max number of active chips if requested */
	a1->num_active_chips = a1->num_chips;

	a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
	assert (a1->chips != NULL);

	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       a1->board_id, a1->num_active_chips, a1->num_cores);

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:
	exit_A1_chain(a1);
	return NULL;
}

static bool submit_a_nonce(struct thr_info *thr, struct work *work, uint32_t nonce) {
    const uint32_t nonce_candies[] = {
       nonce + 1, nonce + 2, nonce + 3, nonce + 4,
       nonce - 3, nonce - 2, nonce - 1, nonce};

    size_t i;
    for ( i = 0; i < sizeof( nonce_candies) / sizeof( nonce_candies[0]); ++i) {
        const uint32_t a_nonce = nonce_candies[i];
        if (!test_nonce( work, a_nonce)) {
            continue;
        }
        if (submit_nonce( thr, work, a_nonce)) {
            return true;
        }
    }
    inc_hw_errors( thr);
    return false;
}

static bool A1_detect_one_chain(struct spi_config *cfg)
{
	struct cgpu_info *cgpu;

    if (!chip_selector_init()) {
        applog(LOG_ERR, "Failed to initialize chip selector");
        return false;
    }
    struct spi_ctx *ctx = spi_init(cfg);

    if (ctx == NULL)
        return false;

    if (!chip_reset(ctx)) {
        applog(LOG_ERR, "Failed to reset chip");
        return false;
    }

    struct A1_chain *a1 = init_A1_chain(ctx);
    if (a1 == NULL)
        return false;

    cgpu = malloc(sizeof(*cgpu));
    assert(cgpu != NULL);

    memset(cgpu, 0, sizeof(*cgpu));
    cgpu->drv = &bitmineA1_drv;
    cgpu->name = "BitmineA1";
    cgpu->threads = 1;

    cgpu->device_data = a1;

    a1->cgpu = cgpu;
    add_cgpu(cgpu);

	return true;
}

/*
 * id: chip id
 */
static void init_one_chip( struct spi_ctx *ctx, unsigned int id) {
    /* TODO: SHOULD RETURN A STRUCTURE THAT PRESENTS A CHIP. */
    if ( !chip_reset( ctx)) {
        applog(LOG_ERR, "Failed to reset chip %u", id);
        return;
    }
}

/* Probe SPI channel and register chip chain */
void A1_detect(bool hotplug)
{
	/* no hotplug support for now */
	if (hotplug)
		return;
 
    if (!chip_selector_init()) {
        applog(LOG_ERR, "Failed to initialize chip selector");
        return;
    }
	
    /* SPI configuration */
    /* TODO: Use options to control spi clk */
    struct spi_config cfg = default_spi_config;
    cfg.mode = SPI_MODE_0;
    cfg.speed = 500 * 1000;
    cfg.delay = 10;         // TODO: may use default value

    struct spi_ctx *ctx = spi_init(&cfg);
    if (ctx == NULL) {
        applog(LOG_ERR, "Failed to initialize SPI");
        return;
    }
	
    struct A1_chain *a1 = init_A1_chain(ctx);
    if (a1 == NULL)
        return;

    struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
    assert(cgpu != NULL);

    memset(cgpu, 0, sizeof(*cgpu));
    cgpu->drv = &bitmineA1_drv;
    cgpu->name = "BitmineA1";
    cgpu->threads = 1;
    cgpu->device_data = a1;

    a1->cgpu = cgpu;

    size_t i;
    for ( i = 0; i < MAX_KM_IC; ++i) {
        init_one_chip( ctx, i);
    }


    // Finally, add the cgpu
    add_cgpu(cgpu);
}







static int64_t A1_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;

	applog(LOG_DEBUG, "A1 running scanwork");
	uint32_t nonce;
	bool work_updated = false;

	mutex_lock(&a1->lock);

	int res;
	
	applog(LOG_ERR, "start");
	struct work *work;
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
                    if ( !chip_select(i)) {
                        applog(LOG_ERR, "Failed to select chip %d", i);
                        continue;
                    }
					set_work(a1, work_pool_p);	
					applog(LOG_ERR, "get and set work ID %d state %d  buf address is %x data is %x, %x , %x , %x", i , work_state[i] , work_pool_p , 
						*(work_pool_p->data),*(work_pool_p->data+1),*(work_pool_p->data+2),*(work_pool_p->data+3));
				} 
			}		
			int try_time =0;
			int j;
			for(j=0;j<MAX_KM_IC;j++){
				if ( !chip_select(j)) {
                    applog(LOG_ERR, "Failed to select chip %d", j);
                    continue;
                }
				work_pool_p = &work_pool[j];
				//applog(LOG_ERR, "get nonce %d state %d  buf address is %x", j , work_state[j] , work_pool_p);
				res	= get_1st_nonce(a1, &nonce);				
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
					if( (work != NULL) && (nonce!=NULL)){
                        if (submit_a_nonce( thr, work_pool_p, nonce)) {
                            applog(LOG_ERR, " submit nonce ok ID is %d nonce is %x addr is %x" , j ,nonce , work_pool_p);
                            work_state[j] = 0;
                            applog(LOG_ERR, "change 2 state %d 0" , j);
                            //goto rec_out;
                            goto submit_end;
                            break;
                        }
                        else {
                            applog(LOG_ERR, "get nonce ID %d state %d  buf address is %x", j , work_state[j] , work_pool_p);
                            applog(LOG_ERR, "hw err nonce is %x, line: %d" , nonce, __LINE__);
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
			}
		}
	}	
	
rec_out:
	mutex_unlock(&a1->lock);
	/* in case of no progress, prevent busy looping */
	if (!work_updated)
		cgsleep_ms(160);
	
    // TODO: SHOULD RETURN (int64_t)(number of hashes done)
	return 0;
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
