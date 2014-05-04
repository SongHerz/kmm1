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


#define MAX_KM_IC	14
 
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
    
    /********************************/
    /* data relates to current work */
    /********************************/
	struct work *work;
    struct timeval this_work_deadline;

    /*********************/
	/* global statistics */
    /*********************/
	uint32_t total_nonces;

    /* consecutive errors */
    uint32_t consec_errs;

	uint32_t hw_errs;
    float ave_hw_errs;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

/********************************************/
/* MACROS that operate on A1_chip structure */
/********************************************/
#define __CHIP_INC_AVE(val)     do {    \
    val = 0.5 + (val) / 2.0;            \
} while(0)

#define __CHIP_DEC_AVE(val)     do {    \
    val = 0.0 + (val) / 2.0;            \
} while(0)

/* Operations on nonces_found */
#define CHIP_INC_NONCE(chip)    do {    \
    chip->total_nonces += 1;            \
} while(0)

/* Operations on err */
#define CHIP_INC_ERR(chip)  do {        \
    chip->consec_errs += 1;             \
    chip->hw_errs += 1;                 \
    __CHIP_INC_AVE(chip->ave_hw_errs);  \
} while(0)

#define CHIP_NO_ERR(chip)   do {    \
    chip->consec_errs = 0;          \
    __CHIP_DEC_AVE(chip->hw_errs);  \
} while(0)

/* Operations on work */
/* Get a future time which is 'ms' milliseconds after current time.
 * The future time is stored to tv.
 */
static void __future_time(unsigned ms, struct timeval *tv) {
    struct timeval incremental;
    incremental.tv_sec = ms / 1000;
    incremental.tv_usec = (ms % 1000) * 1000;

    struct timeval curtime;
    cgtime( &curtime);

    timeradd( &curtime, &incremental, tv);
}

static void CHIP_NEW_WORK(struct cgpu_info *cgpu, struct A1_chip *chip, struct work *newwork) {
    if (chip->work) {
        work_completed( cgpu, chip->work);
    }
    chip->work = newwork;
    if (newwork) {
        // Now, set it to 15 minutes.
        __future_time( 15 * 60 * 1000, &chip->this_work_deadline);
    }
    else {
        timerclear( &chip->this_work_deadline);
    }
}

static inline bool CHIP_IS_WORK_TIMEOUT( const struct A1_chip *chip) {
    assert( chip->work != NULL);
    struct timeval curtime;
    cgtime( &curtime);
    return timercmp( &chip->this_work_deadline, &curtime, <);
}


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
       nonce - 4, nonce - 3, nonce - 2, nonce - 1, nonce};

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


/*
 * Read valid nonces from a chip.
 */
static bool submit_ready_nonces( struct thr_info *thr, struct A1_chip *chip, const uint8_t status) {
#if 0
    // int start = get_current_ms();
    applog( LOG_ERR, "thr = %p", thr);
    // int end = get_current_ms();
    // applog( LOG_ERR, "MS: %d", end - start);
#else
    usleep(10000);
#endif

    assert( STATUS_R_READY( status));
    unsigned int grps[4];
    size_t num_grps = 0;

    if ( STATUS_NONCE_GRP0_RDY( status)) {
        grps[ num_grps++] = 0;
    }
    if ( STATUS_NONCE_GRP1_RDY( status)) {
        grps[ num_grps++] = 1;
    }
    if ( STATUS_NONCE_GRP2_RDY( status)) {
        grps[ num_grps++] = 2;
    }
    if ( STATUS_NONCE_GRP3_RDY( status)) {
        grps[ num_grps++] = 3;
    }

    if ( num_grps == 0) {
        applog(LOG_ERR, "R_READY is high, but no group ready for chip %u", chip->id);
        return false;
    }
#if 1
    {
        size_t j;
        applog(LOG_ERR, "================================ for chip %u", chip->id);
        char s[1024];
        int n = snprintf( s, sizeof(s), "grps = ");
        for ( j = 0; j < num_grps; ++j) {
            n += snprintf( s + n, sizeof(s) - n, "%u, ", grps[j]);
        }
        applog(LOG_ERR, "%sfor chip %u", s, chip->id);
    } 
#endif

    // Get ready groups
    struct A1_chain *chain = thr->cgpu->device_data;
    struct spi_ctx *ctx = chain->spi_ctx;

    bool all_submit_succ = true;
    size_t i;
    for ( i = 0; i < num_grps; ++i) {
        uint32_t nonce;
        if (!chip_read_nonce( ctx, grps[i], &nonce)) {
            applog(LOG_ERR, "Failed to get nonce from chip %u", chip->id);
            return false;
        }

        // submit nonce
        uint32_t actual_nonce;
        if (!submit_a_nonce( thr, chip->work, nonce, &actual_nonce)) {
            applog(LOG_ERR, "Failed to submit nonce %u, work %p, for chip %u",
                    nonce, chip->work, chip->id);
            all_submit_succ = false;
        }
        else {
            CHIP_INC_NONCE(chip);
            applog(LOG_ERR, "submit nonce ok, nonce %u, actual nonce %u, work %p, for chip %u",
                    nonce, actual_nonce, chip->work, chip->id);
        }
    }
    return all_submit_succ;
}

static void may_submit_may_get_work(struct thr_info *thr, unsigned int id) {
#if 0
    applog( LOG_ERR, "%s for chip %u", __func__, id);
#endif
    struct cgpu_info *cgpu = thr->cgpu;
    struct A1_chain *chain = cgpu->device_data;

    assert( id < chain->num_chips);

    if ( !chip_select(id)) {
        applog(LOG_ERR, "Failed to select chip %u", id);
        return;
    }

    struct spi_ctx *ctx = chain->spi_ctx;
    struct A1_chip *chip = chain->chips + id;
    assert( chip);


#define __RESET_AND_GIVE_BACK_WORK()  do {      \
    (void)chip_reset( ctx);                     \
    applog(LOG_ERR, "Reset chip %u", chip->id); \
    CHIP_NEW_WORK( cgpu, chip, NULL);           \
} while(0)

#define FIX_CHIP_ERR_AND_RETURN do {    \
    CHIP_INC_ERR(chip);                 \
    __RESET_AND_GIVE_BACK_WORK();       \
    return;                             \
} while(0)


    if ( chip->work) {
#if 0
        applog(LOG_ERR, "WE ARE HERE 11111111");
#endif
        uint8_t status;
        if ( !chip_status( ctx, &status)) {
            applog(LOG_ERR, "Failed to get status for chip %u", id);
            FIX_CHIP_ERR_AND_RETURN;
        }

        /* The chip status check order is important.
         * Do NOT change the order without strong reason.
         */
        if (STATUS_R_READY( status)) {
#if 0
        applog(LOG_ERR, "CHIP R_READY");
#endif
            // READ nonces
            const bool submit_succ =  submit_ready_nonces( thr, chip, status);

            /* DO always clean chip status */
#if 1
            applog(LOG_ERR, "chip_clean() for chip %u", id);
#endif
            if ( !chip_clean( ctx)) {
                applog(LOG_ERR, "Failed to clean status from chip %u", id);
                FIX_CHIP_ERR_AND_RETURN;
            }
            if ( !submit_succ) {
                applog(LOG_ERR, "Failed to submit nonce for chip %u", id);
                FIX_CHIP_ERR_AND_RETURN;
            }
        }
        /* FIXME: For performance, I think I should write
         * 'if (STATUS_W_ALLOW...)' but not 'else if (STATUS_W_ALLOW...)'
         */
        else if (STATUS_W_ALLOW(status)) {
            assert( chip->work);
#if 0
            applog(LOG_ERR, "CHIP W_ALLOW for chip %u", id);
#endif
            CHIP_NO_ERR( chip);
            CHIP_NEW_WORK( cgpu, chip, NULL);
        }
        else if ( CHIP_IS_WORK_TIMEOUT( chip)) {
            // check w_allow timeout
            applog(LOG_ERR, "Work time out for chip %u", id);
            FIX_CHIP_ERR_AND_RETURN;
        }
        /* FIXME: I think no else is needed here */
        else if (STATUS_BUSY( status)) {
            return;
        }
    }

    if ( chip->work == NULL) {
#if 0
        applog(LOG_ERR, "WE ARE HERE 222222");
#endif
        struct work *new_work = wq_dequeue(&chain->active_wq);
        if (new_work == NULL) {
            applog(LOG_ERR, "queue under flow");
            return;
        }
        CHIP_NEW_WORK( cgpu, chip, new_work);
#if 1
        applog(LOG_ERR, "new work %p for chip %u", chip->work, id);
#endif
        if (!chip_write_job( ctx, chip->work->midstate, chip->work->data + 64)) {
            // give back job
            applog( LOG_ERR, "Failed to write job to chip %u", id);
            FIX_CHIP_ERR_AND_RETURN;
        }
    }
    return;
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
    cfg.speed = 200 * 1000;
    cfg.delay = 30;         // TODO: may use default value

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
	struct A1_chain *chain = thr->cgpu->device_data;

	applog(LOG_DEBUG, "A1 running scanwork");
	mutex_lock(&chain->lock);

	struct work *work;

    size_t k;
	for (k = 0; k < 10; k++) {
        int id;
        for(id = 0;id < chain->num_chips; id++){
            if (id != 0 && id != 1 && id != 6 && id != 7 && id != 12 && id != 13) {
                continue;
            }
            may_submit_may_get_work(thr, id);
        }		
	}	
	
	mutex_unlock(&chain->lock);
	
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
