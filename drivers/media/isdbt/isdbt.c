/*
 * isdbt.c
 *
 * Driver for ISDB-T.
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

#include <linux/version.h>
#include <linux/of_gpio.h>
#include <mach/gpiomux.h>
#include <linux/gpio.h>

#include "mtv23x.h" /* Place before isdbt_ioctl.h ! */
#include "mtv23x_internal.h"

#include "isdbt.h"
#include "isdbt_debug.h"
#include "isdbt_gpio.h"
#include "isdbt_ioctl.h"
#include "isdbt_ioctl_func.h"

#define ISDBT_DEV_MAJOR		225
#define ISDBT_DEV_MINOR		0

#if defined(RTV_IF_SPI)
	// Select time of ksmem allocation.
	#define _MAKE_KSHMEM_ALLOC_FROM_BOOT

	#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
		#define MAX_TSB_DESC_SIZE 	PAGE_ALIGN(sizeof(struct ISDBT_KSHMEM_DESC_INFO))
		#define MAX_TSB_SEG_SIZE	PAGE_ALIGN(188 * 212)

		#define TOTAL_KSMEM_MAPPING_SIZE\
			(MAX_TSB_DESC_SIZE\
				+ ISDBT_MAX_NUM_TSB_SEG*MAX_TSB_SEG_SIZE)
	#endif /* _MAKE_KSHMEM_ALLOC_FROM_BOOT */	
#endif

static struct class *isdbt_class = NULL;

struct ISDBT_CB *isdbt_cb_ptr = NULL;

#ifndef _ISDBT_DEVICE_TREE_UNUSED
static struct isdbt_dt_platform_data *dt_pdata;
#endif

static const char *build_date = __DATE__;
static const char *build_time = __TIME__;

#ifdef CONFIG_ISDBT_CAMIF
extern int camif_isdbt_stop(void);
extern int camif_isdbt_start(unsigned int num_tsp);
extern int camif_isdbt_close(void);
extern int camif_isdbt_open(void);
#endif

#if defined(RTV_IF_SPI)
	extern irqreturn_t isdbt_irq_handler(int irq, void *param);

	#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
		extern int isdbt_isr_thread(void *data);
	#else
		extern void isdbt_isr_handler(struct work_struct *work);
	#endif

static int isdbt_create_isr(void)
{
	int ret = 0;

#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
	if (isdbt_cb_ptr->isr_thread_cb == NULL) {
		atomic_set(&isdbt_cb_ptr->isr_cnt, 0); /* Reset */
		init_waitqueue_head(&isdbt_cb_ptr->isr_wq);
 
		isdbt_cb_ptr->isr_thread_cb
			= kthread_run(isdbt_isr_thread, NULL, ISDBT_DEV_NAME);
		if (IS_ERR(isdbt_cb_ptr->isr_thread_cb)) {
			WARN(1, KERN_ERR "Create ISR thread error\n");
			ret = PTR_ERR(isdbt_cb_ptr->isr_thread_cb);
			isdbt_cb_ptr->isr_thread_cb = NULL;
		}
	}
#else
	INIT_WORK(&isdbt_cb_ptr->isr_work, isdbt_isr_handler);
	isdbt_cb_ptr->isr_workqueue
			= create_singlethread_workqueue(ISDBT_DEV_NAME);
	if (isdbt_cb_ptr->isr_workqueue == NULL) {
		DMBERR("Couldn't create workqueue\n");
		ret = -ENOMEM;
	}
#endif
	return ret;
}

static void isdbt_destory_isr(void)
{
#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
	if (isdbt_cb_ptr->isr_thread_cb) {
		kthread_stop(isdbt_cb_ptr->isr_thread_cb);
		isdbt_cb_ptr->isr_thread_cb = NULL;
	}
#else
	if (isdbt_cb_ptr->isr_workqueue) {
		cancel_work_sync(&isdbt_cb_ptr->isr_work);
		//flush_workqueue(isdbt_cb_ptr->isr_workqueue); 
		destroy_workqueue(isdbt_cb_ptr->isr_workqueue);
		isdbt_cb_ptr->isr_workqueue = NULL;
	}
#endif
}

unsigned int isdbt_get_enqueued_tsp_count(void)
{
	int readi, writei;
	unsigned int num_tsp = 0;
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;

	if (!kshmem->tsbd) {
		DMBERR("Not memory mapped\n");
		return 0;
	}

	if (kshmem->tsbd->op_enabled) {
		readi = kshmem->tsbd->read_idx;
		writei = kshmem->tsbd->write_idx;

		if (writei > readi)
			num_tsp = writei - readi;
		else if (writei < readi)
			num_tsp = kshmem->num_total_tsp - (readi - writei);
		else
			num_tsp = 0; /* Empty */
	}

	return num_tsp;
}

U8 *isdbt_get_kshmem_buf(void)
{
	int readi;
	int nwi; /* Next index of tsp buffer to be write. */
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;
	unsigned char *tspb = NULL;

	if (!kshmem->tsbd) {
		DMBERR("Not memory mapped\n");
		return NULL;
	}

	if (kshmem->tsbd->op_enabled) {
		if (kshmem->num_tsp_per_seg) {
			readi = kshmem->tsbd->read_idx;

			/* Get the next avaliable index of segment to be write in the next time. */
			nwi = kshmem->avail_write_tspb_idx + kshmem->num_tsp_per_seg;
			if (nwi >= kshmem->num_total_tsp)
				nwi = 0;

			if ((readi < nwi) || (readi >= (nwi + kshmem->num_tsp_per_seg))) {
				tspb = kshmem->seg_buf[kshmem->avail_seg_idx];
				if (tspb == NULL) {
					DMBERR("Abnormal mmap configured!\n");
					goto exit_get;
				}
				
				/* Update the writting index of tsp buffer. */
				kshmem->avail_write_tspb_idx = nwi;

				/* Update the avaliable index of segment to be write in the next time. */
				if (++kshmem->avail_seg_idx >= kshmem->num_total_seg)
					kshmem->avail_seg_idx = 0;

				//DMBMSG("@@ readi(%d), next_writei(%d), avail_seg_idx(%d), tspb(0x%08lX)\n",
				//		readi, nwi, kshmem->avail_seg_idx, (unsigned long)tspb);
			} else
				DMBERR("Full tsp buffer.\n");
		} else
			DMBERR("SHMEM was not configured!\n");
	}

exit_get:
	return tspb;
}

void isdbt_enqueue_kshmem_buf(unsigned char *ts_chunk)
{
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;

	if (!kshmem->tsbd) {
		DMBERR("Not memory mapped\n");
		return;
	}

	if (kshmem->tsbd->op_enabled) {
		/* Check if the specified tspb is the allocated tspb? */
		if (ts_chunk == kshmem->seg_buf[kshmem->enqueue_seg_idx]) {
			/* Update the next index of write-tsp. */
			kshmem->tsbd->write_idx = kshmem->avail_write_tspb_idx;

			/* Update the next index of segment. */
			kshmem->enqueue_seg_idx = kshmem->avail_seg_idx;
		} else
			DMBERR("Invalid the enqueuing chunk address!\n");		
	}
}

static inline void free_kshmem_mapping_area(void)
{
	int i;
	unsigned int order;
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;

	order = get_order(kshmem->seg_size);
	for (i = 0; i < kshmem->num_total_seg; i++) {
		if (kshmem->seg_buf[i]) {
			//DMBMSG("SEG[%d]: seg_buf(0x%lX)\n", i, (unsigned long)kshmem->seg_buf[i]);
			free_pages((unsigned long)kshmem->seg_buf[i], order);
			kshmem->seg_buf[i] = NULL;
		}
	}

	kshmem->seg_bufs_allocated = false;
	kshmem->seg_size = 0;
	kshmem->num_total_seg = 0;

	if (kshmem->tsbd) {
		order = get_order(kshmem->desc_size);
		free_pages((unsigned long)kshmem->tsbd, order);

		kshmem->tsbd = NULL;
		kshmem->desc_size = 0;
	}
}

#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
static inline int alloc_kshmem_mapping_area(unsigned int desc_size,
										unsigned int seg_size, int num_seg)
{
	int i, ret;
	unsigned int order;
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;

	/* Allocate the TSB descriptor. */
	order = get_order(desc_size);
	kshmem->tsbd
		= (struct ISDBT_KSHMEM_DESC_INFO *)__get_dma_pages(GFP_KERNEL, order);
	if (!kshmem->tsbd) {
		DMBERR("DESC allocation error\n");
		return -ENOMEM;
	}

	/* Allocate the TSB segments. */
	order = get_order(seg_size);
	DMBMSG("SEG order(%u)\n", order);

	if (order > MAX_ORDER) {
		DMBERR("Invalid page order value of segment (%u)\n", order);
		ret = -ENOMEM;
		goto free_tsb;
	}

	for (i = 0; i < num_seg; i++) {
		kshmem->seg_buf[i] = (U8 *)__get_dma_pages(GFP_KERNEL, order);
		if (!kshmem->seg_buf[i]) {
			DMBERR("SEG[%u] allocation error\n", i);
			ret = -ENOMEM;
			goto free_tsb;
		}
	}

	kshmem->seg_bufs_allocated = true;

	DMBMSG("Success\n");

	return 0;

free_tsb:
	free_kshmem_mapping_area();

	return ret;
}
#endif /* #ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT */

static void isdbt_mmap_close(struct vm_area_struct *vma)
{
	DMBMSG("Entered. mmap_completed(%d)\n", isdbt_cb_ptr->kshmem.mmap_completed);

#ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	if (isdbt_cb_ptr->kshmem.mmap_completed == true)
		free_kshmem_mapping_area();
#endif

	isdbt_cb_ptr->kshmem.mmap_completed = false;

	DMBMSG("Leaved...\n");
}

static const struct vm_operations_struct isdbt_mmap_ops = {
	.close = isdbt_mmap_close,
};

static int isdbt_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret, num_total_seg;
	unsigned int i, mmap_size, desc_size, seg_size;
	unsigned long pfn;
	unsigned long start = vma->vm_start;
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;
#ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	unsigned int desc_order, seg_order;
#endif

	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &isdbt_mmap_ops;

	mmap_size = vma->vm_end - vma->vm_start;

#if 0
	DMBMSG("mmap_size(0x%X), vm_start(0x%lX), vm_page_prot(0x%lX)\n",
				mmap_size, vma->vm_start, vma->vm_page_prot);
#endif

	if (mmap_size & (~PAGE_MASK)) {
		DMBERR("Must align with PAGE size\n");
		return -EINVAL;
	}

	if (kshmem->mmap_completed == true) {
		DMBERR("Already mapped!\n");
		return 0;
	}

	seg_size = vma->vm_pgoff << PAGE_SHIFT;
	num_total_seg = (mmap_size - PAGE_SIZE) / seg_size;

	desc_size = mmap_size - (num_total_seg * seg_size);

	/* Save */
	kshmem->desc_size = desc_size;
	kshmem->seg_size = seg_size;
	kshmem->num_total_seg = num_total_seg;

#if 1
	DMBMSG("mmap_size(%u), seg_size(%u) #seg(%d), desc_size(%u)\n",
				mmap_size, seg_size, num_total_seg, desc_size);
#endif

	if (num_total_seg > ISDBT_MAX_NUM_TSB_SEG) {
		DMBERR("Too large request #seg! kernel(%u), req(%d)\n",
			ISDBT_MAX_NUM_TSB_SEG, num_total_seg);
		return -ENOMEM;
	}

#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	if (desc_size > MAX_TSB_DESC_SIZE) {
		DMBERR("Too large request desc size! kernel(%u), req(%u)\n",
			MAX_TSB_DESC_SIZE, desc_size);
		return -ENOMEM;
	}

	if (seg_size > MAX_TSB_SEG_SIZE) {
		DMBERR("Too large request seg size! kernel(%u), req(%u)\n",
			MAX_TSB_SEG_SIZE, seg_size);
		return -ENOMEM;
	}

	if (!kshmem->tsbd) {
		DMBERR("Kshmem DESC was NOT allocated!\n");
		return -ENOMEM;
	}

	if (kshmem->seg_bufs_allocated == false) {
		DMBERR("Kshmem SEG are NOT allocated!\n");
		return -ENOMEM;
	}
#else
	seg_order = get_order(seg_size);
	DMBMSG("SEG order(%u)\n", seg_order);
	
	if (seg_order > MAX_ORDER) {
		DMBERR("Invalid page order value of segment (%u)\n", seg_order);
		return -ENOMEM;
	}

	/* Allocate the TSB descriptor. */
	desc_order = get_order(desc_size);
	kshmem->tsbd
		= (struct ISDBT_KSHMEM_DESC_INFO *)__get_dma_pages(GFP_KERNEL, desc_order);
	if (!kshmem->tsbd) {
		DMBERR("DESC allocation error\n");
		return -ENOMEM;
	}
#endif

	/* Map the shared informations. */
	pfn = virt_to_phys(kshmem->tsbd) >> PAGE_SHIFT;
	if (remap_pfn_range(vma, vma->vm_start, pfn, desc_size, vma->vm_page_prot)) {
		DMBERR("HDR remap_pfn_range() error!\n");
		ret = -EAGAIN;
		goto out;
	}

	/* Init descriptor except the addres of segments */
	kshmem->tsbd->op_enabled = 0;
	kshmem->tsbd->read_idx = 0;
	kshmem->tsbd->write_idx = 0;

#if 0
	DMBMSG("tsbd(0x%lX), pfn(0x%lX), start(0x%lX)\n",
		(unsigned long)kshmem->tsbd, pfn, start);
#endif

	start += desc_size; /* Avdance VMA. */

	/* Allocate and map the TSP buffer segments. */
	for (i = 0; i < num_total_seg; i++) {
#ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT
		kshmem->seg_buf[i] = (U8 *)__get_dma_pages(GFP_KERNEL, seg_order);
		if (!kshmem->seg_buf[i]) {
			DMBERR("SEG[%u] allocation error\n", i);
			ret = -ENOMEM;
			goto out;
		}
#endif

		pfn = virt_to_phys(kshmem->seg_buf[i]) >> PAGE_SHIFT;

#if 1
		DMBMSG("SEG[%d]: seg_buf(0x%lX) pfn(0x%lX) start(0x%lX)\n",
				i, (unsigned long)kshmem->seg_buf[i], pfn, start);
#endif

		if (remap_pfn_range(vma, start, pfn, seg_size, vma->vm_page_prot)) {
			DMBERR("SEG[%u] remap_pfn_range() error!\n", i);
			ret = -EAGAIN;
			goto out;
		}

		kshmem->tsbd->seg_base[i] = start;
		start += seg_size;
	}

#ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	kshmem->seg_bufs_allocated = true;
#endif
	kshmem->mmap_completed = true;

	return 0;

out:
#ifndef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	/* Free kernel mapped memory */
	free_kshmem_mapping_area();
#endif

	return ret;
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

static void isdbt_gpio_on(void)
{
	int pwr_en, pwr_en2, spi_mosi, spi_miso, spi_cs, spi_clk;
	
	DMBMSG("isdbt_gpio_on\n");	

#ifndef _ISDBT_DEVICE_TREE_UNUSED
	pwr_en = dt_pdata->isdbt_pwr_en;
	pwr_en2 = dt_pdata->isdbt_pwr_en2;
	spi_mosi = dt_pdata->isdbt_spi_mosi;
	spi_miso = dt_pdata->isdbt_spi_miso;
	spi_cs = dt_pdata->isdbt_spi_cs;
	spi_clk = dt_pdata->isdbt_spi_clk;
#else
	pwr_en = MTV_PWR_EN;
	pwr_en2 = MTV_PWR_EN2;
	spi_mosi = GPIO_ISDBT_MOSI;
	spi_miso = GPIO_ISDBT_MISO;
	spi_cs = GPIO_ISDBT_CS;
	spi_clk = GPIO_ISDBT_CLK;
#endif

	gpio_direction_output(pwr_en, 1);
	gpio_direction_output(pwr_en2, 1);

	if(msm_gpiomux_write(spi_mosi, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		DMBMSG("spi_mosi Port request error!!!\n");
	if(msm_gpiomux_write(spi_miso, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		DMBMSG("spi_miso Port request error!!!\n");
	if(msm_gpiomux_write(spi_cs, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		DMBMSG("spi_cs Port request error!!!\n");
	if(msm_gpiomux_write(spi_clk, GPIOMUX_ACTIVE, &gpio_spi_gpio_active_config, NULL) < 0)
		DMBMSG("spi_clk Port request error!!!\n");

	usleep_range(1, 1000);
}

static void isdbt_gpio_off(void)
{
	int pwr_en, pwr_en2, spi_mosi, spi_miso, spi_cs, spi_clk;
	
	DMBMSG("isdbt_gpio_off\n");

#ifndef _ISDBT_DEVICE_TREE_UNUSED
	pwr_en = dt_pdata->isdbt_pwr_en;
	pwr_en2 = dt_pdata->isdbt_pwr_en2;
	spi_mosi = dt_pdata->isdbt_spi_mosi;
	spi_miso = dt_pdata->isdbt_spi_miso;
	spi_cs = dt_pdata->isdbt_spi_cs;
	spi_clk = dt_pdata->isdbt_spi_clk;
#else
	pwr_en = MTV_PWR_EN;
	pwr_en2 = MTV_PWR_EN2;
	spi_mosi = GPIO_ISDBT_MOSI;
	spi_miso = GPIO_ISDBT_MISO;
	spi_cs = GPIO_ISDBT_CS;
	spi_clk = GPIO_ISDBT_CLK;
#endif

	gpio_direction_input(pwr_en);
	gpio_direction_input(pwr_en2);

	if(msm_gpiomux_write(spi_mosi, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		DMBMSG("spi_mosi Port request error!!!\n");
	if(msm_gpiomux_write(spi_miso, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		DMBMSG("spi_miso Port request error!!!\n");
	if(msm_gpiomux_write(spi_cs, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		DMBMSG("spi_cs Port request error!!!\n");
	if(msm_gpiomux_write(spi_clk, GPIOMUX_SUSPENDED, &gpio_spi_gpio_suspend_config, NULL) < 0)
		DMBMSG("spi_clk Port request error!!!\n");

	usleep_range(1, 1000);
}

#endif /* #if defined(RTV_IF_SPI) */

static int isdbt_power_off(void)
{
	int ret = 0;

	if (isdbt_cb_ptr->is_power_on == FALSE)
		return 0;

	isdbt_cb_ptr->is_power_on = FALSE;
	isdbt_cb_ptr->tsout_enabled = false;

	DMBMSG("START\n");

	isdbt_disable_ts_out();

#if defined(RTV_IF_SPI)
	disable_irq(isdbt_cb_ptr->irq);

	isdbt_destory_isr();
#endif

	ret = __isdbt_deinit();

	rtvOEM_PowerOn(0);

#if defined(RTV_IF_SPI)
	isdbt_gpio_off();
#endif
	DMBMSG("END\n");

	return ret;
}

static int isdbt_power_on(unsigned long arg)
{
	int ret = 0;

	if (isdbt_cb_ptr->is_power_on == TRUE)	
		return 0;

	DMBMSG("Start\n");

#if defined(RTV_IF_SPI)
	isdbt_gpio_on();
#endif
	rtvOEM_PowerOn(1);
	isdbt_cb_ptr->is_power_on = TRUE;

	ret = __isdbt_power_on(arg);
	if (ret != 0) {
		return ret;
	}

#if defined(RTV_IF_SPI)
	ret = isdbt_create_isr();
	if (ret != 0)
		return ret;

	enable_irq(isdbt_cb_ptr->irq); /* After DMB init */
#endif

	DMBMSG("End\n");

	return ret;
}

static ssize_t isdbt_read(struct file *filp, char *buf,
				size_t count, loff_t *pos)
{
	DMBMSG("Empty function\n");
	return 0;
}

static long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	mutex_lock(&isdbt_cb_ptr->ioctl_lock);
	
	switch (cmd) {
	case IOCTL_ISDBT_POWER_ON:
		ret = isdbt_power_on(arg);
		if (ret)
			ret = isdbt_power_off();
		break;

	case IOCTL_ISDBT_POWER_OFF:
		isdbt_power_off();
		break;

	case IOCTL_ISDBT_SCAN_CHANNEL:
		ret = isdbt_scan_channel(arg);
		break;
            
	case IOCTL_ISDBT_SET_CHANNEL:
		ret = isdbt_set_channel(arg);		
		break;

	case IOCTL_ISDBT_START_TS:
		ret = isdbt_enable_ts_out();
		break;

	case IOCTL_ISDBT_STOP_TS:
		isdbt_disable_ts_out();
		break;			

	case IOCTL_ISDBT_GET_LOCK_STATUS:
		ret = isdbt_get_lock_status(arg);
		break;

	case IOCTL_ISDBT_GET_SIGNAL_INFO:
		ret = isdbt_get_signal_info(arg);
		break;

	case IOCTL_ISDBT_SUSPEND:
		isdbt_enable_standby_mode(arg);
		break;

	case IOCTL_ISDBT_RESUME:
		isdbt_disable_standby_mode(arg);
		break;

	case IOCTL_ISDBT_GET_BER_PER_INFO:
		ret = isdbt_get_ber_per_info(arg);
		break;

	case IOCTL_ISDBT_GET_RSSI:
		ret = isdbt_get_rssi(arg);
		break;

	case IOCTL_ISDBT_GET_CNR:
		ret = isdbt_get_cnr(arg);
		break;

	case IOCTL_ISDBT_GET_SIGNAL_QUAL_INFO:
		ret = isdbt_get_signal_qual_info(arg);
		break;

	/* Test IO command */
	case IOCTL_TEST_GPIO_SET:
	case IOCTL_TEST_GPIO_GET:
		ret = test_gpio(arg, cmd);
		break;

	case IOCTL_TEST_MTV_POWER_ON:	
	case IOCTL_TEST_MTV_POWER_OFF:
		test_power_on_off(cmd);
		break;		

	case IOCTL_TEST_REG_SINGLE_READ:
	case IOCTL_TEST_REG_BURST_READ:
	case IOCTL_TEST_REG_WRITE:
	case IOCTL_TEST_REG_SPI_MEM_READ:
	case IOCTL_TEST_REG_ONLY_SPI_MEM_READ:
		ret = test_register_io(arg, cmd);
		break;
	
	default:
		DMBERR("Invalid ioctl command: 0x%X\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&isdbt_cb_ptr->ioctl_lock);

	return ret;
}

static int isdbt_close(struct inode *inode, struct file *filp)
{
	DMBMSG("START\n");

	mutex_lock(&isdbt_cb_ptr->ioctl_lock);
	isdbt_power_off();
	mutex_unlock(&isdbt_cb_ptr->ioctl_lock);
	mutex_destroy(&isdbt_cb_ptr->ioctl_lock);

	/* Allow the DMB applicaton to entering suspend. */
	wake_unlock(&isdbt_cb_ptr->wake_lock);

	atomic_set(&isdbt_cb_ptr->open_flag, 0);

	DMBMSG("END\n");

	return 0;
}

static int isdbt_open(struct inode *inode, struct file *filp)
{
	/* Check if the device is already opened ? */
	if (atomic_cmpxchg(&isdbt_cb_ptr->open_flag, 0, 1)) {
		DMBERR("%s driver was already opened!\n", ISDBT_DEV_NAME);
		return -EBUSY;
	}

	isdbt_cb_ptr->is_power_on = FALSE;

	mutex_init(&isdbt_cb_ptr->ioctl_lock);

	/* Prevents the DMB applicaton from entering suspend. */
	wake_lock(&isdbt_cb_ptr->wake_lock);

	DMBMSG("Open OK\n");

	return 0;
}

static void isdbt_exit_bus(void)
{
#if defined(RTV_IF_TSIF_0) || defined(RTV_IF_TSIF_1) || defined(RTV_IF_SPI_SLAVE)
	#ifndef RTV_IF_SPI_TSIFx
	i2c_del_driver(&isdbt_i2c_driver);
	#endif
#endif

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
	spi_unregister_driver(&isdbt_spi_driver);
#endif
}

static int isdbt_init_bus(void)
{
	int ret;

#if defined(RTV_IF_TSIF_0) || defined(RTV_IF_TSIF_1) || defined(RTV_IF_SPI_SLAVE)
	#ifndef RTV_IF_SPI_TSIFx
	ret = i2c_add_driver(&isdbt_i2c_driver);
	if (ret) {
		DMBERR("I2C driver register failed\n");
		return ret;
	}
	#endif
#endif

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
	ret = spi_register_driver(&isdbt_spi_driver);
	if (ret) {
		DMBERR("SPI driver register failed\n");
		return ret;
	}
#endif

	return 0;
}

static const struct file_operations isdbt_ctl_fops = {
	.owner          = THIS_MODULE,
	.open           = isdbt_open,
	.read           = isdbt_read,
	.unlocked_ioctl  = isdbt_ioctl,

#if defined(RTV_IF_SPI)
	.mmap           = isdbt_mmap,
#endif
	.release	    = isdbt_close,
	.llseek         = no_llseek,
};

#ifndef _ISDBT_DEVICE_TREE_UNUSED
static struct isdbt_dt_platform_data *get_isdbt_dt_pdata(struct device *dev)
{
	struct isdbt_dt_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct isdbt_dt_platform_data), GFP_KERNEL);
	if (!pdata) {
		DMBMSG("%s : could not allocate memory for platform data\n", __func__);
		goto err;
	}
	pdata->isdbt_pwr_en = of_get_named_gpio(dev->of_node, "isdbt_pwr_en", 0);
	if (pdata->isdbt_pwr_en < 0) {
		DMBMSG("%s : can not find the isdbt_pwr_en\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_pwr_en2 = of_get_named_gpio(dev->of_node, "isdbt_pwr_en2", 0);
	if (pdata->isdbt_pwr_en2 < 0) {
		DMBMSG("%s : can not find the isdbt_pwr_en2\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_irq = of_get_named_gpio(dev->of_node, "isdbt_irq", 0);
	if (pdata->isdbt_irq < 0) {
		DMBMSG("%s : can not find the isdbt_irq\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_spi_mosi = of_get_named_gpio(dev->of_node, "isdbt_spi_mosi", 0);
	if (pdata->isdbt_spi_mosi < 0) {
		DMBMSG("%s : can not find the isdbt_spi_mosi\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_spi_miso = of_get_named_gpio(dev->of_node, "isdbt_spi_miso", 0);
	if (pdata->isdbt_spi_miso < 0) {
		DMBMSG("%s : can not find the isdbt_spi_miso\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_spi_cs = of_get_named_gpio(dev->of_node, "isdbt_spi_cs", 0);
	if (pdata->isdbt_spi_cs < 0) {
		DMBMSG("%s : can not find the isdbt_spi_cs\n", __func__);
		goto alloc_err;
	}
	pdata->isdbt_spi_clk = of_get_named_gpio(dev->of_node, "isdbt_spi_clk", 0);
	if (pdata->isdbt_spi_clk < 0) {
		DMBMSG("%s : can not find the isdbt_spi_clk\n", __func__);
		goto alloc_err;
	}
	return pdata;
alloc_err:
	devm_kfree(dev, pdata);
err:
	return NULL;
}
#endif

static int isdbt_probe(struct platform_device *pdev)
{
	int ret;
	int irq_pin_no;

	printk(KERN_INFO "\t=============== ISDB-T ========================\n");
	printk(KERN_INFO "\t[%s] Module Build Date/Time: %s, %s\n",
				ISDBT_DEV_NAME, build_date, build_time);
	printk(KERN_INFO "\t===============================================\n");

#ifndef _ISDBT_DEVICE_TREE_UNUSED
	dt_pdata = get_isdbt_dt_pdata(&pdev->dev);
	if (!dt_pdata) {
		pr_err("%s : isdbt_dt_pdata is NULL.\n", __func__);
		return -ENODEV;
	}
#endif
	
//	isdbt_configure_gpio();

	ret = isdbt_init_bus();
	if (ret)
		return ret;

#if defined(RTV_IF_SPI) // || defined(RTV_IF_EBI)
	#ifndef _ISDBT_DEVICE_TREE_UNUSED
	irq_pin_no = dt_pdata->isdbt_irq;
	#else
	irq_pin_no = GPIO_ISDBT_IN;
	#endif

	isdbt_cb_ptr->irq = gpio_to_irq(irq_pin_no);

	ret = request_irq(isdbt_cb_ptr->irq, isdbt_irq_handler,
	#if defined(RTV_INTR_POLARITY_LOW_ACTIVE)
					IRQ_TYPE_EDGE_FALLING,
	#elif defined(RTV_INTR_POLARITY_HIGH_ACTIVE)
					IRQ_TYPE_EDGE_RISING,
	#endif
					ISDBT_DEV_NAME, NULL);
	if (ret != 0) {
		DMBERR("Failed to install irq (%d)\n", ret);
		isdbt_exit_bus();
		return ret;
	}
	disable_irq(isdbt_cb_ptr->irq); /* Must disabled */
#endif

	atomic_set(&isdbt_cb_ptr->open_flag, 0);

	wake_lock_init(&isdbt_cb_ptr->wake_lock,
			WAKE_LOCK_SUSPEND, ISDBT_DEV_NAME);

	return 0;
}

static int isdbt_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&isdbt_cb_ptr->wake_lock);

#if defined(RTV_IF_SPI) // || defined(RTV_IF_EBI)
	free_irq(isdbt_cb_ptr->irq, NULL);
#endif

	isdbt_exit_bus();

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

#ifndef _ISDBT_DEVICE_TREE_UNUSED
static const struct of_device_id isdbt_match_table[] = {
	{   .compatible = "isdbt_pdata",
	},
	{}
};
#endif
static struct platform_driver isdbt_driver = {
	.probe	= isdbt_probe,
	.remove = isdbt_remove,
	.suspend = isdbt_suspend,
	.resume = isdbt_resume,
	.driver = {
		.owner	= THIS_MODULE,
		.name = "isdbt",
#ifndef _ISDBT_DEVICE_TREE_UNUSED
		.of_match_table = isdbt_match_table,
#endif
	},
};

static int __init isdbt_init(void)
{
   	int ret;
	struct device *isdbt_dev;

	DMBMSG("Module init\n");

	isdbt_cb_ptr = kzalloc(sizeof(struct ISDBT_CB), GFP_KERNEL);
	if (!isdbt_cb_ptr) {
		DMBERR("ISDBT_CB allocating error\n");
		return -ENOMEM;
	}

#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	ret = alloc_kshmem_mapping_area(MAX_TSB_DESC_SIZE, MAX_TSB_SEG_SIZE,
									ISDBT_MAX_NUM_TSB_SEG);
	if (ret)
		goto free_cb;
#endif

	ret = register_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME, &isdbt_ctl_fops);
	if (ret < 0) {
		DMBERR("Failed to register chrdev\n");
		goto before_free_ksmem;
	}

	isdbt_class = class_create(THIS_MODULE, ISDBT_DEV_NAME);
	if (IS_ERR(isdbt_class)) {
		DMBERR("class_create failed!\n");
		ret = -EFAULT;
		goto unreg_chrdev;
	}

	isdbt_dev = device_create(isdbt_class, NULL,
				MKDEV(ISDBT_DEV_MAJOR, ISDBT_DEV_MINOR),
				NULL, ISDBT_DEV_NAME);
	if (IS_ERR(isdbt_dev)) {
		ret = -EFAULT;
		DMBERR("device_create failed!\n");
		goto destory_class;
	}

	ret = platform_driver_register(&isdbt_driver);
	if (ret) {
		DMBERR("platform_driver_register failed!\n");
		goto destory_device;
	}

	return 0;

destory_device:
	device_destroy(isdbt_class, MKDEV(ISDBT_DEV_MAJOR, ISDBT_DEV_MINOR));

destory_class:
	class_destroy(isdbt_class);

unreg_chrdev:
	unregister_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME);

before_free_ksmem:
#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	free_kshmem_mapping_area();
#endif

free_cb:
	kfree(isdbt_cb_ptr);
	isdbt_cb_ptr = NULL;

	return ret;
}

static void __exit isdbt_exit(void)
{
	DMBMSG("Module exit\n");

	platform_driver_unregister(&isdbt_driver);

	device_destroy(isdbt_class, MKDEV(ISDBT_DEV_MAJOR, ISDBT_DEV_MINOR));

	class_destroy(isdbt_class);

	unregister_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME);

#ifdef _MAKE_KSHMEM_ALLOC_FROM_BOOT
	free_kshmem_mapping_area();
#endif
	
	kfree(isdbt_cb_ptr);
	isdbt_cb_ptr = NULL;
}

module_init(isdbt_init);
module_exit(isdbt_exit);
MODULE_DESCRIPTION("ISDBT driver");
MODULE_LICENSE("GPL v2");

