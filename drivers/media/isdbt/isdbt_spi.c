/*
 * mtv_spi.c
 *
 * RAONTECH MTV spi driver.
 *
 * Copyright (C) (2013, RAONTECH)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "mtv23x.h"
#include "mtv23x_internal.h"

#include "isdbt.h"

#define SPI_DEV_NAME	"isdbt"

#define SPI_SPEED_NORMAL_Hz	10000000//6000000//15000000 //12000000
#define SPI_SPEED_HIGH_Hz	32000000//45000000 //12000000

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)

unsigned char isdbt_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
	u8 out_buf[4], in_buf[4];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};

#ifdef RTV_SPI_HIGH_SPEED_ENABLE
	if (g_bRtvSpiHighSpeed == TRUE)
		msg_xfer.speed_hz = SPI_SPEED_HIGH_Hz;
	else
		msg_xfer.speed_hz = SPI_SPEED_NORMAL_Hz;
#endif

	spi_message_init(&msg);
	out_buf[0] = 0x90 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* Read size */

	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(isdbt_cb_ptr->spi_ptr, &msg);
	if (ret) {
		DMBERR("error: %d\n", ret);
		return 0xFF;
	}

#if 0
	DMBMSG("0x%02X 0x%02X 0x%02X 0x%02X\n",
			in_buf[0], in_buf[1], in_buf[2], in_buf[3]);
#endif

	return in_buf[MTV23X_SPI_CMD_SIZE];
}

void isdbt_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
	u8 out_buf[MTV23X_SPI_CMD_SIZE];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = buf,
		.len = MTV23X_SPI_CMD_SIZE,
		.cs_change = 0,
		.delay_usecs = 0
	};

	struct spi_transfer msg_xfer1 = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0
	};

	spi_message_init(&msg);
	if (page > 15) { /* 0 ~ 15: not SPI memory */
		out_buf[0] = 0xA0; /* Memory read */
		out_buf[1] = 0x00;
		out_buf[2] = 188; /* Fix */
	} else {
		out_buf[0] = 0x90 | page; /* Register read */
		out_buf[1] = reg;
		out_buf[2] = size;
	}

#ifdef RTV_SPI_HIGH_SPEED_ENABLE
	if (g_bRtvSpiHighSpeed == TRUE)
		msg_xfer.speed_hz = SPI_SPEED_HIGH_Hz;
	else
		msg_xfer.speed_hz = SPI_SPEED_NORMAL_Hz;

	if (g_bRtvSpiHighSpeed == TRUE)
		msg_xfer1.speed_hz = SPI_SPEED_HIGH_Hz;
	else
		msg_xfer1.speed_hz = SPI_SPEED_NORMAL_Hz;
#endif

	spi_message_add_tail(&msg_xfer, &msg);
	ret = spi_sync(isdbt_cb_ptr->spi_ptr, &msg);
	if (ret) {
		DMBERR("0 error: %d\n", ret);
		return;
	}

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer1, &msg);
	ret = spi_sync(isdbt_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("1 error: %d\n", ret);	
}

void isdbt_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
	u8 out_buf[4];
	u8 in_buf[4];
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};
	int ret;

#ifdef RTV_SPI_HIGH_SPEED_ENABLE
	if (g_bRtvSpiHighSpeed == TRUE)
		msg_xfer.speed_hz = SPI_SPEED_HIGH_Hz;
	else
		msg_xfer.speed_hz = SPI_SPEED_NORMAL_Hz;
#endif

	spi_message_init(&msg);
	out_buf[0] = 0x80 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* size */
	out_buf[3] = val;
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(isdbt_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);
}

void isdbt_spi_recover(unsigned char *buf, unsigned int size)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	memset(buf, 0xFF, size);

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(isdbt_cb_ptr->spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);
}

static int isdbt_spi_probe(struct spi_device *spi)
{
	int ret;

	DMBMSG("ENTERED!!!!!!!!!!!!!!\n");

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;

	ret = spi_setup(spi);
	if (ret)
		return ret;

	isdbt_cb_ptr->spi_ptr = spi;

	return 0;
}


static int isdbt_spi_remove(struct spi_device *spi)
{
	return 0;
}

static const struct of_device_id isdbt_spi_match_table[] = {
	{   .compatible = "isdbt_spi_comp",
	},
	{}
};

struct spi_driver isdbt_spi_driver = {
	.driver = {
		.name = SPI_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = isdbt_spi_match_table,
	},

	.probe = isdbt_spi_probe,
	.suspend = NULL,
	.resume	= NULL,
	.remove	= __devexit_p(isdbt_spi_remove),
};

#endif /* #if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx) */

