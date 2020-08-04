/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <mach/gpiomux.h>

#include <media/isdbt_pdata.h>

#include <linux/clk.h>
#include <mach/gpio.h>

#if defined(CONFIG_ISDBT_ANT_DET)
#include <linux/wakelock.h>
#include <linux/input.h>
#endif

#include "fc8300.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"

struct ISDBT_INIT_INFO_T *hInit;
#define FEATURE_GLOBAL_MEM

#ifdef FEATURE_GLOBAL_MEM
struct ISDBT_OPEN_INFO_T hOpen_Val;
#define RING_BUFFER_SIZE	(1024*1024)
u8 ringbuffer[RING_BUFFER_SIZE];
#else
#define RING_BUFFER_SIZE	(188 * 320 * 50)
#endif

#define USE_THREADED_IRQ
#define FCI_DEBUG
/* GPIO(RESET & INTRRUPT) Setting */
#define FC8300_NAME		"isdbt"


#define GPIO_ISDBT_EN2		110 
#define GPIO_ISDBT_RST_N	67
#define GPIO_ISDBT_EN		36 
#define GPIO_ISDBT_INT		49
#define GPIO_ISDBT_IRQ		gpio_to_irq(GPIO_ISDBT_INT)

#define GPIO_ISDBT_IRQ_FC8150 		GPIO_ISDBT_INT
#define GPIO_ISDBT_PWR_EN_FC8150 	GPIO_ISDBT_EN
#define GPIO_ISDBT_RST_FC8150 	GPIO_ISDBT_RST_N

#define GPIO_ISDBT_MOSI                 20
#define GPIO_ISDBT_MISO                 21
#define GPIO_ISDBT_CS                   22
#define GPIO_ISDBT_CLK                  23

#if 0
//u8 static_ringbuffer[RING_BUFFER_SIZE];
#define FEATURE_TS_CHECK
#endif

#ifdef FEATURE_TS_CHECK
u32 check_cnt_size;

#define MAX_DEMUX           2

/*
 * Sync Byte 0xb8
 */
#define SYNC_BYTE_INVERSION

struct pid_info {
	unsigned long count;
	unsigned long discontinuity;
	unsigned long continuity;
};

struct demux_info {
	struct pid_info  pids[8192];

	unsigned long    ts_packet_c;
	unsigned long    malformed_packet_c;
	unsigned long    tot_scraped_sz;
	unsigned long    packet_no;
	unsigned long    sync_err;
	unsigned long 	   sync_err_set;
};

static int is_sync(unsigned char *p)
{
	int syncword = p[0];
#ifdef SYNC_BYTE_INVERSION
	if(0x47 == syncword || 0xb8 == syncword)
		return 1;
#else
	if(0x47 == syncword)
		return 1;
#endif
	return 0;
}
static struct demux_info demux[MAX_DEMUX];

void print_pkt_log(void)
{
	unsigned long i=0;

	print_log(NULL, "\nPKT_TOT : %d, SYNC_ERR : %d, SYNC_ERR_BIT : %d \
		, ERR_PKT : %d \n"
		, demux[0].ts_packet_c, demux[0].sync_err
		, demux[0].sync_err_set, demux[0].malformed_packet_c);

	for (i = 0; i < 8192; i++) {
		if(demux[0].pids[i].count>0)
			print_log(NULL, "PID : %d, TOT_PKT : %d\
							, DISCONTINUITY : %d \n"
			, i, demux[0].pids[i].count
			, demux[0].pids[i].discontinuity);
	}
}

int put_ts_packet(int no, unsigned char *packet, int sz)
{
	unsigned char* p;
	int transport_error_indicator, pid, payload_unit_start_indicator;
	int continuity_counter, last_continuity_counter;
	int i;
	if((sz % 188)) {
		print_log(NULL, "L : %d", sz);
	} else {
		for(i = 0; i < sz; i += 188) {
			p = packet + i;

			pid = ((p[1] & 0x1f) << 8) + p[2];

			demux[no].ts_packet_c++;
			if(!is_sync(packet + i)) {
				print_log(NULL, "S     ");
				demux[no].sync_err++;
				if(0x80==(p[1] & 0x80))
					demux[no].sync_err_set++;
				print_log(NULL, "0x%x, 0x%x, 0x%x, 0x%x \n"
					, *p, *(p+1),  *(p+2), *(p+3));
				continue;
			}

			transport_error_indicator = (p[1] & 0x80) >> 7;
			if(1 == transport_error_indicator) {
				demux[no].malformed_packet_c++;
				continue;
			}

			payload_unit_start_indicator = (p[1] & 0x40) >> 6;

			demux[no].pids[pid].count++;

			continuity_counter = p[3] & 0x0f;

			if(demux[no].pids[pid].continuity == -1) {
				demux[no].pids[pid].continuity
					= continuity_counter;
			} else {
				last_continuity_counter
					= demux[no].pids[pid].continuity;

				demux[no].pids[pid].continuity
					= continuity_counter;

				if (((last_continuity_counter + 1) & 0x0f)
					!= continuity_counter) {
					demux[no].pids[pid].discontinuity++;
				}
			}
		}
	}
	return 0;
}


void create_tspacket_anal(void)
{
	int n, i;

	for(n = 0; n < MAX_DEMUX; n++) {
		memset((void*)&demux[n], 0, sizeof(demux[n]));

		for (i = 0; i < 8192; i++)
			demux[n].pids[i].continuity = -1;
		}

}
#endif

enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
static DEFINE_MUTEX(ringbuffer_lock);

static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);

static u8 isdbt_isr_sig;
//static struct task_struct *isdbt_kthread;
static struct isdbt_platform_data gpio_cfg;

//static struct clk *isdbt_clk=NULL;
//static unsigned long clkrate = 0;


#ifdef USE_THREADED_IRQ
static irqreturn_t isdbt_threaded_irq(int irq, void *dev_id)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)dev_id;

	if (driver_mode == ISDBT_POWERON) {
		driver_mode = ISDBT_DATAREAD;
		bbm_com_isr(hInit);
		driver_mode = ISDBT_POWERON;
	}

	return IRQ_HANDLED;
}
#else
static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	isdbt_isr_sig = 1;
	wake_up_interruptible(&isdbt_isr_wait);
	return IRQ_HANDLED;
}
#endif

int isdbt_hw_setting(HANDLE hDevice)
{
	int err;
#ifdef USE_THREADED_IRQ
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;
#endif	
	print_log(0, "isdbt_hw_setting \n");

	err = gpio_request(GPIO_ISDBT_EN2, "isdbt_en2");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en2\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_EN2, 0);

	err = gpio_request(GPIO_ISDBT_PWR_EN_FC8150, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN_FC8150, 0);

	err = 	gpio_request(GPIO_ISDBT_RST_FC8150, "isdbt_rst");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_rst\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST_FC8150, 0);

	err = 	gpio_request(GPIO_ISDBT_IRQ_FC8150, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_input(GPIO_ISDBT_IRQ_FC8150);

#ifdef USE_THREADED_IRQ
	err = request_threaded_irq(gpio_to_irq(GPIO_ISDBT_IRQ_FC8150), NULL
	, isdbt_threaded_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING
	, FC8300_NAME, hInit);
#else
	err = request_irq(gpio_to_irq(GPIO_ISDBT_IRQ_FC8150), isdbt_irq
		, IRQF_DISABLED | IRQF_TRIGGER_RISING, FC8300_NAME, NULL);
#endif

	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: couldn't request gpio	\
			interrupt %d reason(%d)\n"
			, gpio_to_irq(GPIO_ISDBT_IRQ_FC8150), err);
	goto request_isdbt_irq;
	}
	return 0;
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ_FC8150);
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN_FC8150);
gpio_isdbt_en:
	return err;
}

static struct gpiomux_setting gpio_spi_gpio_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gpio_spi_gpio_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{
	int i = 0;
	int pwr_en, pwr_en2, spi_mosi, spi_miso, spi_cs, spi_clk;

	while (driver_mode == ISDBT_DATAREAD) {
		msWait(100);
		if (i++ > 5)
			break;
	}

	print_log(0, "isdbt_hw_init \n");

	pwr_en = GPIO_ISDBT_EN;
	pwr_en2 = GPIO_ISDBT_EN2;
	spi_mosi = GPIO_ISDBT_MOSI;
	spi_miso = GPIO_ISDBT_MISO;
	spi_cs = GPIO_ISDBT_CS;
	spi_clk = GPIO_ISDBT_CLK;

	if(msm_gpiomux_write(spi_mosi, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		print_log(0,"spi_mosi Port request error!!!\n");
	if(msm_gpiomux_write(spi_miso, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		print_log(0,"spi_miso Port request error!!!\n");
	if(msm_gpiomux_write(spi_cs, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		print_log(0,"spi_cs Port request error!!!\n");
	if(msm_gpiomux_write(spi_clk, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		print_log(0,"spi_clk Port request error!!!\n");

	gpio_direction_output(pwr_en2, 1);
	mdelay(50);	
	gpio_direction_output(pwr_en, 1);

	gpio_set_value(pwr_en2, 1);
	mdelay(50);
	gpio_set_value(pwr_en, 1);
	mdelay(50);
	gpio_set_value(GPIO_ISDBT_RST_FC8150, 1);
	mdelay(5);
	driver_mode = ISDBT_POWERON;

}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{
	int pwr_en, pwr_en2, spi_mosi, spi_miso, spi_cs, spi_clk;

	print_log(0, "isdbt_hw_deinit \n");
	driver_mode = ISDBT_POWEROFF;
	gpio_set_value(GPIO_ISDBT_PWR_EN_FC8150, 0);
	mdelay(5);	
	gpio_set_value(GPIO_ISDBT_EN2, 0);
	gpio_set_value(GPIO_ISDBT_RST_N, 0);

	pwr_en = GPIO_ISDBT_EN;
	pwr_en2 = GPIO_ISDBT_EN2;
	spi_mosi = GPIO_ISDBT_MOSI;
	spi_miso = GPIO_ISDBT_MISO;
	spi_cs = GPIO_ISDBT_CS;
	spi_clk = GPIO_ISDBT_CLK;
	
	gpio_direction_input(pwr_en);
	gpio_direction_input(pwr_en2);

	if(msm_gpiomux_write(spi_mosi, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		print_log(0,"spi_mosi Port request error!!!\n");
	if(msm_gpiomux_write(spi_miso, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		print_log(0,"spi_miso Port request error!!!\n");
	if(msm_gpiomux_write(spi_cs, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		print_log(0,"spi_cs Port request error!!!\n");
	if(msm_gpiomux_write(spi_clk, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		print_log(0,"spi_clk Port request error!!!\n");	
}

int data_callback(u32 hDevice, u8 bufid, u8 *data, int len)
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;
	
	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			mutex_lock(&ringbuffer_lock);
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len) {
				/*print_log(hDevice, "f"); */
				/* return 0 */;
				print_log(hInit, "fci_ringbuffer_free is full\n");
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);
			}

			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);

			wake_up_interruptible(&(hOpen->RingBuffer.queue));

			mutex_unlock(&ringbuffer_lock);

#ifdef FEATURE_TS_CHECK
			if(!(len%188)) {
				put_ts_packet(0, data, len);
				check_cnt_size+=len;

				if (check_cnt_size > 188*320*40) {
					print_pkt_log();
					check_cnt_size=0;
				}
			}
#endif			
		}
	}

	return 0;
}

#ifndef USE_THREADED_IRQ
static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	set_user_nice(current, -20);

	print_log(hInit, "isdbt_kthread enter\n");

	bbm_com_ts_callback_register((u32)hInit, data_callback);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait,
			isdbt_isr_sig || kthread_should_stop());

		if (driver_mode == ISDBT_POWERON) {
			driver_mode = ISDBT_DATAREAD;
			bbm_com_isr(hInit);
			driver_mode = ISDBT_POWERON;
		}

			isdbt_isr_sig = 0;

		if (kthread_should_stop())
			break;			
	}

	bbm_com_ts_callback_deregister();

	print_log(hInit, "isdbt_kthread exit\n");

	return 0;
}
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

static struct miscdevice fc8300_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = FC8300_NAME,
    .fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt open\n");

#ifdef FEATURE_GLOBAL_MEM
	hOpen = &hOpen_Val;
	hOpen->buf = &ringbuffer[0];
#else
	hOpen = kmalloc(sizeof(struct ISDBT_OPEN_INFO_T), GFP_KERNEL);
	hOpen->buf = kmalloc(RING_BUFFER_SIZE, GFP_KERNEL);
#endif	
	/*kmalloc(RING_BUFFER_SIZE, GFP_KERNEL);*/
	hOpen->isdbttype = 0;

	list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if (hOpen->buf == NULL) {
		print_log(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	s32 non_blocking = filp->f_flags & O_NONBLOCK;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;

	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}

	if (non_blocking && (fci_ringbuffer_empty(cibuf)))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}

	if (wait_event_interruptible(cibuf->queue,
		!fci_ringbuffer_empty(cibuf))) {
		print_log(hInit, "return ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	mutex_lock(&ringbuffer_lock);

	avail = fci_ringbuffer_avail(cibuf);

	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);

	read_len = fci_ringbuffer_read_user(cibuf, buf, len);

	mutex_unlock(&ringbuffer_lock);

	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt_release\n");

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;

	list_del(&(hOpen->hList));
#ifndef FEATURE_GLOBAL_MEM
		kfree(hOpen->buf);
		kfree(hOpen);
#endif

	return 0;
}

void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, DIV_BROADCAST, BBM_BUF_INT_ENABLE, 0x00);

	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;

		msWait(10);
		isr_time++;
	}

}

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	s32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);

	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		res = bbm_com_reset(hInit, DIV_BROADCAST);
		print_log(hInit, "[FC8300] IOCTL_ISDBT_RESET \n");
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		if (res) {
			print_log(hInit
				, "FC8300 bbm_com_i2c_init Initialize Fail \n");
			break;
		}		
		res |= bbm_com_probe(hInit, DIV_BROADCAST);
		if (res) {
			print_log(hInit
				, "FC8300  bbm_com_probe Initialize Fail \n");
			break;
		}
		res |= bbm_com_init(hInit, DIV_BROADCAST);
		res |= bbm_com_tuner_select(hInit
			, DIV_BROADCAST, FC8300_TUNER, ISDBT_13SEG);
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u16 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u32 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8)info.buff[1]);
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u16)info.buff[1]);
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u32)info.buff[1]);
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_bulk_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_read(hInit, DIV_BROADCAST, (u8)info.buff[0]
			, (u8)info.buff[1],  (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_write(hInit, DIV_BROADCAST, (u8)info.buff[0]
			, (u8)info.buff[1], (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			err = copy_from_user((void *)&info, (void *)arg, size);

			f_rf = (u32)info.buff[0] ;
			isdbt_isr_check(hInit);
			res = bbm_com_tuner_set_freq(hInit
				, DIV_MASTER, f_rf, 0x15);
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit
				, DIV_BROADCAST, BBM_BUF_INT_ENABLE, 0x01);
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hInit
			, DIV_BROADCAST, (u32)info.buff[0], (u32)info.buff[1]);
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TUNER_SELECT \n");
		break;
	case IOCTL_ISDBT_TS_START:
#ifdef FEATURE_TS_CHECK
				create_tspacket_anal();
				check_cnt_size = 0;
#endif		
		hOpen->isdbttype = TS_TYPE;
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_START \n");
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_STOP \n");
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_setting(hInit);		
		
		isdbt_hw_init();
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_ON \n");
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_OFF \n");
		break;
	case IOCTL_ISDBT_SCAN_STATUS:
		res = bbm_com_scan_status(hInit, DIV_BROADCAST);
		print_log(hInit
			, "[FC8300] IOCTL_ISDBT_SCAN_STATUS : %d\n", res);
		break;
	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit
			, DIV_BROADCAST, (s32 *)&info.buff[0]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

static int isdbt_probe(struct platform_device *pdev)
{
	int res;
	struct isdbt_platform_data *p = pdev->dev.platform_data;
	memcpy(&gpio_cfg, p, sizeof(struct isdbt_platform_data));

	res = misc_register(&fc8300_misc_device);

	if (res < 0) {
		print_log(hInit, "isdbt init fail : %d\n", res);
		return res;
	}


	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);

//	isdbt_hw_setting(hInit);
	
#ifdef USE_THREADED_IRQ
	bbm_com_ts_callback_register((u32)hInit, data_callback);
#endif

	res = bbm_com_hostif_select(hInit, BBM_SPI);

	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");
	
	print_log(hInit, "isdbt isdbt_probe\n");

#ifndef USE_THREADED_IRQ
	if (!isdbt_kthread)	{
		print_log(hInit, "kthread run\n");
		isdbt_kthread = kthread_run(isdbt_thread
			, (void *)hInit, "isdbt_thread");
	}
#endif

	INIT_LIST_HEAD(&(hInit->hHead));
#if defined(CONFIG_ISDBT_ANT_DET)
	wake_lock_init(&isdbt_ant_wlock, WAKE_LOCK_SUSPEND, "isdbt_ant_wlock");

	if (!isdbt_ant_det_reg_input(pdev))
		goto err_reg_input;
	if (!isdbt_ant_det_create_wq())
		goto free_reg_input;
	if (!isdbt_ant_det_irq_set(true))
		goto free_ant_det_wq;

	return 0;
free_ant_det_wq:
	isdbt_ant_det_destroy_wq();
free_reg_input:
	isdbt_ant_det_unreg_input();
err_reg_input:
	return -EFAULT;
#else
	return 0;
#endif

}

static int isdbt_remove(struct platform_device *pdev)
{
	print_log(hInit, "ISDBT remove\n");
#if defined(CONFIG_ISDBT_ANT_DET)
	isdbt_ant_det_unreg_input();
	isdbt_ant_det_destroy_wq();
	isdbt_ant_det_irq_set(false);
	wake_lock_destroy(&isdbt_ant_wlock);
#endif
	return 0;
}

static int isdbt_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int isdbt_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver isdbt_driver = {
			.probe	= isdbt_probe,
			.remove = isdbt_remove,
			.suspend = isdbt_suspend,
			.resume = isdbt_resume,
			.driver = {
					.owner	= THIS_MODULE,
					.name = "isdbt"
				},
};

int isdbt_init(void)
{
	s32 res;

#ifdef FCI_DEBUG	
	print_log(hInit, "isdbt_init\n");
#endif
	res = platform_driver_register(&isdbt_driver);
	if (res < 0) {
		print_log(hInit, "isdbt init fail : %d\n", res);
		return res;
	}
	return 0;	

}
void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit \n");

	free_irq(gpio_to_irq(GPIO_ISDBT_IRQ_FC8150), NULL);
	gpio_free(GPIO_ISDBT_IRQ_FC8150);
	gpio_free(GPIO_ISDBT_RST_FC8150);

#ifdef USE_THREADED_IRQ
	bbm_com_ts_callback_deregister();
#else
	if (isdbt_kthread != NULL)
	kthread_stop(isdbt_kthread);
	isdbt_kthread = NULL;
#endif

	bbm_com_hostif_deselect(hInit);
	gpio_free(GPIO_ISDBT_PWR_EN_FC8150);

	isdbt_hw_deinit();
	platform_driver_unregister(&isdbt_driver);
	misc_deregister(&fc8300_misc_device);

	kfree(hInit);
}

module_init(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");

