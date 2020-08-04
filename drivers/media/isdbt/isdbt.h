
#ifndef __ISDBT_H__
#define __ISDBT_H__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/atomic.h>
//#include <asm/atomic.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/list.h> 
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>

#if defined(CONFIG_ISDBT_MTV23x)
	#include "mtv23x.h"

#elif defined(CONFIG_ISDBT_MTV222)
	#include "raontv.h"
#else
	#error "Code not present"
#endif

//#define _ISDBT_DEVICE_TREE_UNUSED
#define DMB_DEBUG_MSG_ENABLE

#if defined(RTV_IF_SPI)
	/* Select debug options */
	#define DEBUG_INTERRUPT
	#define DEBUG_TSP_BUF
#endif


#define DMBERR(fmt, args...) \
	printk(KERN_ERR "ISDBT: %s(): " fmt, __func__, ## args)

#ifdef DMB_DEBUG_MSG_ENABLE
	#define DMBMSG(fmt, args...) \
		printk(KERN_INFO "ISDBT: %s(): " fmt, __func__, ## args)
#else 
	#define DMBMSG(x...)  do {} while (0)
#endif 


#if defined(RTV_IF_SPI)
/* Select SPI ISR handler is kthread or workqueue.
   Kthread if defined,
   Workqueue if not defined. */
#define ISDBT_SPI_ISR_HANDLER_IS_KTHREAD

#define ISDBT_MAX_NUM_TSB_SEG		60
/* KSHMEM descriptor informations. */
struct ISDBT_KSHMEM_DESC_INFO {
	/* Flag of operation enabled or not. */
	volatile int op_enabled; /* 0: disabled, 1: enabled */

	/* TSP buffer index which updated when read operation by App. */
	volatile int read_idx;

	/* TSP buffer index which update when write operation by Kernel. */
	volatile int write_idx;

	/* Mapping base address of TS buffer segments.	
	The number of allocating elements was configured by application. */
	unsigned long seg_base[ISDBT_MAX_NUM_TSB_SEG];
};

/* KSHMEM control block. */
struct ISDBT_KSHMEM_CB {
	/* Index of available tsp segment to be write. */
	int avail_seg_idx;

	/* Index of available tsp buffer to be write. */
	int avail_write_tspb_idx;

	/* Index of tsp segment to be enqueued. */
	int enqueue_seg_idx;

	/* Number of buffering TSPs per segment configured by App. */
	int num_tsp_per_seg;

	/* Number of buffering TSPs in the kernel shared memory
	configured by App. */
	int num_total_tsp;

	/* Number of shared memory segments. */
	int num_total_seg;

	unsigned int desc_size;
	unsigned int seg_size;

	/* The pointer to the address of TSB descriptor
	which shared informations to be allocated. */
	struct ISDBT_KSHMEM_DESC_INFO *tsbd;

	bool seg_bufs_allocated;

	bool mmap_completed;

	/* The pointer to the address of TS buffer segments to be allocated. */
	unsigned char *seg_buf[ISDBT_MAX_NUM_TSB_SEG];
};
#endif /* #if defined(RTV_IF_SPI) */

/* ISDBT drvier Control Block */
struct ISDBT_CB {
#if defined(RTV_IF_TSIF_0) || defined(RTV_IF_TSIF_1) || defined(RTV_IF_SPI_SLAVE)
	#ifndef RTV_IF_SPI_TSIFx
	struct i2c_client *i2c_client_ptr;
	struct i2c_adapter *i2c_adapter_ptr;
	#endif

	#ifdef CONFIG_ISDBT_CAMIF
	struct ISDBT_KSHMEM_CB kshmem;
	unsigned int intr_size[MAX_NUM_RTV_SERVICE]; /* Interrupt size */
	unsigned int cfged_tsp_chunk_size; /* Configured TSP chunk size */
	#endif /* CONFIG_ISDBT_CAMIF */
#endif

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
	struct spi_device *spi_ptr;

	#ifndef RTV_IF_SPI_TSIFx
	int irq; /* IRQ number */
	struct ISDBT_KSHMEM_CB kshmem;
	unsigned int intr_size[MAX_NUM_RTV_SERVICE]; /* Interrupt size */
	unsigned int cfged_tsp_chunk_size; /* Configured TSP chunk size */

  #ifdef ISDBT_SPI_ISR_HANDLER_IS_KTHREAD
	struct task_struct *isr_thread_cb;
	wait_queue_head_t isr_wq;
	atomic_t isr_cnt;
  #else
	struct workqueue_struct *isr_workqueue;
	struct work_struct isr_work;
  #endif
	#endif /* #ifndef RTV_IF_SPI_TSIFx */
#endif


#if defined(RTV_IF_SPI) || defined(CONFIG_ISDBT_EBI)\
	|| defined(CONFIG_ISDBT_CAMIF)

#endif

	struct device *dev;
	atomic_t open_flag; /* to open only once */
	struct mutex ioctl_lock;
	struct wake_lock wake_lock;

	volatile bool is_power_on;
	unsigned int freq_khz;

	volatile bool tsout_enabled;
	enum E_RTV_SERVICE_TYPE cfged_svc; /* Configured service type */
	
#ifdef DEBUG_INTERRUPT
	unsigned long invalid_intr_cnt;
	unsigned long level_intr_cnt;
	unsigned long ovf_intr_cnt;
	unsigned long udf_intr_cnt;
#endif

#ifdef DEBUG_TSP_BUF
	unsigned int max_alloc_tspb_cnt;
	unsigned int max_enqueued_tspb_cnt;
	unsigned long alloc_tspb_err_cnt;
#endif
};

#ifndef _ISDBT_DEVICE_TREE_UNUSED
struct isdbt_dt_platform_data {
	int isdbt_irq;
	int isdbt_pwr_en;
	int isdbt_pwr_en2;
	int isdbt_spi_mosi;
	int isdbt_spi_miso;
	int isdbt_spi_cs;
	int isdbt_spi_clk;
};
#endif

extern struct ISDBT_CB *isdbt_cb_ptr;

#if defined(RTV_IF_TSIF_0) || defined(RTV_IF_TSIF_1) || defined(RTV_IF_SPI_SLAVE)
	#ifndef RTV_IF_SPI_TSIFx
extern struct i2c_driver isdbt_i2c_driver;
	#endif
#endif

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
extern struct spi_driver isdbt_spi_driver;
#endif

#if defined(RTV_IF_SPI)
unsigned int isdbt_get_enqueued_tsp_count(void);
U8 *isdbt_get_kshmem_buf(void);
void isdbt_enqueue_kshmem_buf(unsigned char *ts_chunk);
#endif  /*#if defined(RTV_IF_SPI) */

#endif /* __ISDBT_H__*/

