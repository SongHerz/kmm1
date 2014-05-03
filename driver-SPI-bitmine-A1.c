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


struct A1_chip {
    unsigned int id;
	struct work *work;
	/* stats */
	int hw_errors;
	int nonces_found;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

struct A1_chain {
	struct cgpu_info *cgpu;
	int num_chips;
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
    // Reset all chips
    uint8_t i;
    for( i = 0; i < a1->num_chips; ++i) {
        if ( !chip_select( i) || !chip_reset( a1->spi_ctx)) {
            return false;
        }
    }
    return true;
}

/********** driver interface */

/*
 * id: chip id
 */
static bool init_one_chip( struct A1_chip *chip, struct spi_ctx *ctx, unsigned int id) {
    if ( !chip_reset( ctx)) {
        applog(LOG_ERR, "Failed to reset chip %u", id);
        return false;
    }
    memset(chip, 0, sizeof(*chip));
    chip->id = id;
    return true;
}

static struct A1_chain *init_A1_chain( struct cgpu_info *cgpu, struct spi_ctx *ctx)
{
	struct A1_chain *a1 = malloc(sizeof(*a1));
	assert(a1 != NULL);

	applog(LOG_DEBUG, "A1 init chain");
	memset(a1, 0, sizeof(*a1));
    a1->cgpu = cgpu;
	a1->num_chips = MAX_KM_IC;
	if (a1->num_chips == 0)
		goto failure;
	a1->spi_ctx = ctx;

	applog(LOG_WARNING, "spidev%d.%d: Found %d A1 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->num_chips);

	a1->chips = calloc(a1->num_chips, sizeof(struct A1_chip));
	assert (a1->chips != NULL);

	applog(LOG_WARNING, "found %d chips", a1->num_chips);

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

    size_t i;
    for ( i = 0; i < a1->num_chips; ++i) {
        if (!init_one_chip( a1->chips + i, ctx, i)) {
            goto failure;
        }
    }

	return a1;

failure:
    if (a1) {
        if (a1->chips) {
            free(a1->chips);
        }
        free(a1);
    }
    return NULL;
}

static bool submit_a_nonce(struct thr_info *thr, struct work *work,
        const uint32_t nonce, uint32_t *actual_nonce) {
    assert( actual_nonce);
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
            *actual_nonce = a_nonce;
            return true;
        }
    }
    inc_hw_errors( thr);
    return false;
}

static bool may_submit_may_get_work(struct cgpu_info *cgpu, struct thr_info *thr,
        struct A1_chain *chain, unsigned int id) {
    assert( id < chain->num_chips);
    if ( !chip_select(id)) {
        applog(LOG_ERR, "Failed to select chip %d", id);
        return false;
    }

    struct spi_ctx *ctx = chain->spi_ctx;
    struct A1_chip *chip = chain->chips + id;
    assert( chip);

#define RETURN_FALSE    do {    \
    chip->hw_errors++;          \
    return false;               \
} while(0)


    if ( chip->work) {
        uint8_t status;
        if ( !chip_status( ctx, &status)) {
            applog(LOG_ERR, "Failed to get chip status %d", id);
            RETURN_FALSE;
        }

        if (STATUS_BUSY( status)) {
            return true;
        }

        if (STATUS_R_READY( status)) {
            // READ 1st nonce
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

            uint32_t nonce;
            if (!chip_read_nonce( ctx, grp, &nonce)) {
                applog(LOG_ERR, "Failed to get nonce from chip %u", id);
                RETURN_FALSE;
            }
            if (!chip_reset( ctx)) {
                applog(LOG_ERR, "Failed to reset, after get 1st nonce for chip %u", id);
                RETURN_FALSE;
            }
            // submit nonce
            if (nonce == 0) {
                // FIXME: ASK myc, why nonce == 0 means time out
                applog(LOG_ERR, "time out");
                RETURN_FALSE;
            }
            uint32_t actual_nonce;
            if (submit_a_nonce( thr, chip->work, nonce, &actual_nonce)) {
                applog(LOG_ERR, "submit nonce ok, nonce %u, actual nonce %u, for chip %u",
                        nonce, actual_nonce, id);
            }
            else {
                applog(LOG_ERR, "Failed to submit nonce %u, for chip %u", nonce, id);
                RETURN_FALSE;
            }

            // FIXME: MAY BE WORK_COMPLETED SHOULD BE CALLED
            chip->work = NULL;
        }

        if (STATUS_W_ALLOW(status)) {
            // FIXME: MAY BE WORK_COMPLETED SHOULD BE CALLED
            // Empty the work for the chip, and a new work will be assigned to
            // the chip later.
            chip->work = NULL;
        }
    }

    if ( chip->work == NULL) {
        chip->work = wq_dequeue(&chain->active_wq);
        if (chip->work == NULL) {
            applog(LOG_ERR, "queue under flow");
            return false;
        }
        if (!chip_write_job( ctx, chip->work->midstate, chip->work->data + 64)) {
            // give back job
            work_completed( cgpu, chip->work);
            chip->work = NULL;
            applog( LOG_ERR, "Failed to write job to chip %u", id);
            RETURN_FALSE;
        }
    }
    return true;
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
	
    struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
    assert(cgpu != NULL);

    struct A1_chain *a1 = init_A1_chain(cgpu, ctx);
    if (a1 == NULL)
        return;

    memset(cgpu, 0, sizeof(*cgpu));
    cgpu->drv = &bitmineA1_drv;
    cgpu->name = "BitmineA1";
    cgpu->threads = 1;
    cgpu->device_data = a1;

    // Finally, add the cgpu
    add_cgpu(cgpu);
}




static int64_t A1_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;

	applog(LOG_DEBUG, "A1 running scanwork");
	mutex_lock(&a1->lock);

	struct work *work;

    size_t k;
	for (k = 0; k < 10; k++) {
        int id;
        for(id = 0;id < a1->num_chips; id++){
            may_submit_may_get_work(cgpu, thr, a1, id);
            //applog(LOG_ERR, "check work %d state ID %d  buf address is %x", i , work_state[i] , work_pool_p);
        }		
	}	
	
	mutex_unlock(&a1->lock);
	
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
	applog(LOG_DEBUG, "A1 running queue_full: %d/%d",
            a1->active_wq.num_elems, a1->num_chips);

	if (a1->active_wq.num_elems >= a1->num_chips * 2)
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

	applog(LOG_DEBUG, "A1 running flushwork");

	size_t i;

	mutex_lock(&a1->lock);
	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "failed to abort work in chip chain!");
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < a1->num_chips; i++) {
		struct A1_chip *chip = &a1->chips[i];
        struct work *work = chip->work;
        if (work == NULL) {
            continue;
        }
        applog(LOG_DEBUG, "flushing chip %d, work: 0x%p", i, work);
        work_completed( cgpu, work);
        chip->work = NULL;
    }
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
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
	tailsprintf(buf, len, "%2d ", a1->num_chips);
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
