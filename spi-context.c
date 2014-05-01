/*
 * generic SPI functions
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "spi-context.h"

#include "logging.h"
#include "miner.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>

struct spi_ctx *spi_init(struct spi_config *config)
{
	char dev_fname[PATH_MAX];
	
	unsigned char cmdrst_tx[2];
	
	struct spi_ctx *ctx;

	if (config == NULL)
		return NULL;

	sprintf(dev_fname, SPI_DEVICE_TEMPLATE, config->bus, config->cs_line);

	int fd = open(dev_fname, O_RDWR);
	if (fd < 0) {
		applog(LOG_ERR, "SPI: Can not open SPI device %s", dev_fname);
		return NULL;
	}

	if ((ioctl(fd, SPI_IOC_WR_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MODE, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) {
		applog(LOG_ERR, "SPI: ioctl error on SPI device %s", dev_fname);
		close(fd);
		return NULL;
	}
	struct spi_ioc_transfer spi_send_data;
	
	unsigned char send_data_tx[2];
	unsigned char send_data_rx[2];
	
	//read reg 63
	send_data_tx[0] = 0x7f;		// 'b10111111&address  
	send_data_tx[1] = 0x01;	
	
	spi_send_data.tx_buf = (unsigned long)send_data_tx;
    spi_send_data.rx_buf = (unsigned long)send_data_rx;
    spi_send_data.len = 2;
    spi_send_data.delay_usecs = 10;
    spi_send_data.speed_hz = config->speed;
    spi_send_data.bits_per_word = config->bits;
	ioctl(fd, SPI_IOC_MESSAGE(1), &spi_send_data);

	
	applog(LOG_ERR, "SPI send %x", send_data_tx[0]);
	applog(LOG_ERR, "SPI send %x", send_data_tx[1]);
	
	ctx = malloc(sizeof(*ctx));
	assert(ctx != NULL);

	ctx->fd = fd;
	ctx->config = *config;
	ctx->config.mode = 0;
	applog(LOG_WARNING, "SPI '%s': mode=%hhu, bits=%hhu, speed=%u",
	       dev_fname, ctx->config.mode, ctx->config.bits,
	       ctx->config.speed);
	applog(LOG_ERR, "SPI fd is  %x", ctx->fd);
	
	return ctx;
}

extern void spi_exit(struct spi_ctx *ctx)
{
	if (NULL == ctx)
		return;

	close(ctx->fd);
	free(ctx);
}

extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len)
{
	struct spi_ioc_transfer xfr;
	int ret;

	if (rxbuf != NULL)
		memset(rxbuf, 0xff, len);

	ret = len;

	xfr.tx_buf = (unsigned long)txbuf;
	xfr.rx_buf = (unsigned long)rxbuf;
	xfr.len = len;
	xfr.speed_hz = ctx->config.speed;
	xfr.delay_usecs = ctx->config.delay;
	xfr.bits_per_word = ctx->config.bits;
	xfr.cs_change = 0;
	xfr.pad = 0;

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(1), &xfr);
	if (ret < 1)
		applog(LOG_ERR, "SPI: ioctl error on SPI device: %d", ret);

	return ret > 0;
}
