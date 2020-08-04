/*
 * File name: isdbt_isr.c
 *
 * Description: ISDBT ISR for SPI interface.
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
 */

#include "mtv23x.h"
#include "mtv23x_internal.h"
#include "isdbt.h"
#include "isdbt_debug.h"

UINT total_int_cnt = 0;

#if defined(RTV_IF_SPI)
static INLINE void print_tsp(U8 *tspb, UINT size)
{
#if 0
	UINT i;
	const U8 *tsp_buf_ptr = (const U8 *)tspb;	
	
	for (i = 0; i < size/188; i++, tsp_buf_ptr += 188) {
		DMBMSG("[%d] 0x%02X 0x%02X 0x%02X 0x%02X | 0x%02X\n",
			i, tsp_buf_ptr[0], tsp_buf_ptr[1],
			tsp_buf_ptr[2], tsp_buf_ptr[3],
			tsp_buf_ptr[187]);
	}
#endif
}

#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
void isdbt_isr_handler(void)
#else
void isdbt_isr_handler(struct work_struct *work)
#endif
{
	U8 *tspb = NULL; /* reset */
	UINT intr_size;
	U8 istatus;
	//U8 ifreg;
	//static U8 recover_buf[MTV23X_SPI_CMD_SIZE + 435*188];

	RTV_GUARD_LOCK;

	RTV_REG_MAP_SEL(SPI_CTRL_PAGE);

	intr_size = rtvMTV23x_GetInterruptSize();

#if 0
	ifreg = RTV_REG_GET(0x55);
	if (ifreg != 0xAA) {
		isdbt_spi_recover(recover_buf, MTV23X_SPI_CMD_SIZE + intr_size);
		DMBMSG("Interface error 1\n");
		DMB_INV_INTR_INC;
	}
#endif

	/* Read the register of interrupt status. */
	istatus = RTV_REG_GET(0x10);
	//DMBMSG("$$istatus(0x%02X)\n", istatus);

#if 0
	if (istatus & (U8)(~SPI_INTR_BITS)) {
		isdbt_spi_recover(recover_buf, MTV23X_SPI_CMD_SIZE + intr_size);
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		RTV_GUARD_FREE;
		DMBMSG("Interface error 2 (0x%02X)\n", istatus);
		return;
	}	

	if (istatus & SPI_UNDERFLOW_INTR) {
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		DMB_UNF_INTR_INC;
		RTV_GUARD_FREE;
		DMBMSG("UNF: 0x%02X\n", istatus);
		return;
	}
#endif

	if (istatus & SPI_THRESHOLD_INTR) {
		/* Allocate a TS buffer from shared memory. */
		tspb = isdbt_get_kshmem_buf();
		if (tspb) {
			RTV_REG_MAP_SEL(SPI_MEM_PAGE);
			RTV_REG_BURST_GET(0x10, tspb, intr_size); 
	
			print_tsp(tspb, intr_size);
	
			/* Enqueue */
			isdbt_enqueue_kshmem_buf(tspb);

			DMB_LEVEL_INTR_INC;

			if (istatus & SPI_OVERFLOW_INTR) {
				DMB_OVF_INTR_INC;
				DMBMSG("OVF: 0x%02X\n", istatus);
			}
		} else {
			RTV_REG_SET(0x2A, 1); /* SRAM init */
			RTV_REG_SET(0x2A, 0);
			isdbt_cb_ptr->alloc_tspb_err_cnt++;
			DMBERR("No more TSP buffer from pool.\n");
		}
	} else {
		RTV_REG_MAP_SEL(SPI_CTRL_PAGE);
		RTV_REG_SET(0x2A, 1);
		RTV_REG_SET(0x2A, 0);
		DMBMSG("No data interrupt (0x%02X)\n", istatus);
	}
	
	RTV_GUARD_FREE;
}

#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD

#define SCHED_FIFO_USE

int isdbt_isr_thread(void *data)
{
#ifdef SCHED_FIFO_USE
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	sched_setscheduler(current, SCHED_FIFO, &param);
#else
	set_user_nice(current, -20);
#endif

	DMBMSG("Start...\n");

	while (!kthread_should_stop()) {
		wait_event_interruptible(isdbt_cb_ptr->isr_wq,
			kthread_should_stop() || atomic_read(&isdbt_cb_ptr->isr_cnt));

		if (kthread_should_stop())
			break;

		isdbt_isr_handler();

		atomic_dec(&isdbt_cb_ptr->isr_cnt);
	}

	DMBMSG("Exit.\n");

	return 0;
}
#endif /* ISDBT_SPI_ISR_HANDLER_IS_KTHREAD */

irqreturn_t isdbt_irq_handler(int irq, void *param)
{
#ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
	if (isdbt_cb_ptr->isr_thread_cb) {
		if (isdbt_cb_ptr->tsout_enabled) {
			atomic_inc(&isdbt_cb_ptr->isr_cnt);

			wake_up_interruptible(&isdbt_cb_ptr->isr_wq);
			//total_int_cnt++;
		}
	}
#else
	if (isdbt_cb_ptr->isr_workqueue) {
		if (isdbt_cb_ptr->tsout_enabled) {
			atomic_inc(&isdbt_cb_ptr->isr_cnt);

		if (!queue_work(isdbt_cb_ptr->isr_workqueue,
				&isdbt_cb_ptr->isr_work)))
			DMBERR("Failed to queue notify ISR work\n");
	}
	}
#endif

	return IRQ_HANDLED;
}
#endif /* #if defined(RTV_IF_SPI) */


