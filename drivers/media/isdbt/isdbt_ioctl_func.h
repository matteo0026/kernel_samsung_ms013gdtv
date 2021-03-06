/*
 * File name: isdbt_ioctl_func.h
 *
 * Description: ISDBT IO control functions header file.
 *                   Only used in the isdbt.c
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

#include "isdbt.h"

#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
static unsigned long diff_jiffies_1st, diff_jiffies0, hours_cnt;
#endif

#if defined(CONFIG_ISDBT_MTV23x)
#define ISDBT_VALID_SVC_MASK\
	((1<<RTV_SERVICE_UHF_ISDBT_1seg)|(1<<RTV_SERVICE_UHF_ISDBT_13seg)\
	|(1<<RTV_SERVICE_VHF_ISDBTmm_1seg)|(1<<RTV_SERVICE_VHF_ISDBTmm_13seg)\
	|(1<<RTV_SERVICE_VHF_ISDBTsb_1seg)|(1<<RTV_SERVICE_VHF_ISDBTsb_3seg))
#elif defined(CONFIG_ISDBT_MTV222)
	#define ISDBT_VALID_SVC_MASK	(1<<RTV_SERVICE_UHF_ISDBT_1seg)
#else
	#error "Code not present"
#endif


/*============================================================================
 * Test IO control commands(0 ~ 10)
 *==========================================================================*/
static int test_register_io(unsigned long arg, unsigned int cmd)
{
	int ret = 0;
	unsigned int page, addr, write_data, read_cnt, i;
	U8 value;
#if defined(RTV_IF_SPI) || defined(RTV_IF_SPI_TSIFx)
	unsigned long diff_jiffies1;
	unsigned int elapsed_ms;
	unsigned long param1;
#endif
	U8 *reg_read_buf;
	U8 *src_ptr, *dst_ptr;
	IOCTL_REG_ACCESS_INFO __user *argp
				= (IOCTL_REG_ACCESS_INFO __user *)arg;

	if (isdbt_cb_ptr->is_power_on == FALSE) {			
		DMBMSG("[mtv] Power Down state!Must Power ON\n");
		return -EFAULT;
	}

	if (get_user(page, &argp->page))
		return -EFAULT;

	if (get_user(addr, &argp->addr))
		return -EFAULT;

	RTV_GUARD_LOCK;

	switch (cmd) {
	case IOCTL_TEST_REG_SINGLE_READ:
		RTV_REG_MAP_SEL(page);
		value = RTV_REG_GET(addr);
		if (put_user(value, &argp->read_data[0])) {
			ret = -EFAULT;
			goto regio_exit;
		}
		break;

	case IOCTL_TEST_REG_BURST_READ:
		if (get_user(read_cnt, &argp->read_cnt)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		reg_read_buf = kmalloc(MAX_NUM_MTV_REG_READ_BUF, GFP_KERNEL);
		if (reg_read_buf == NULL) {
			DMBERR("Register buffer allocation error\n");
			ret = -ENOMEM;
			goto regio_exit;
		}

		RTV_REG_MAP_SEL(page);
		RTV_REG_BURST_GET(addr, reg_read_buf, read_cnt);
		src_ptr = &reg_read_buf[0];
		dst_ptr = argp->read_data;

		for (i = 0; i< read_cnt; i++, src_ptr++, dst_ptr++) {
			if(put_user(*src_ptr, dst_ptr)) {
				ret = -EFAULT;
				break;
			}
		}

		kfree(reg_read_buf);
		break;

	case IOCTL_TEST_REG_WRITE:
		if (get_user(write_data, &argp->write_data)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		RTV_REG_MAP_SEL(page);
		RTV_REG_SET(addr, write_data);
		break;

	case IOCTL_TEST_REG_SPI_MEM_READ:
	#if defined(RTV_IF_SPI)
		if (get_user(write_data, &argp->write_data)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		if (get_user(read_cnt, &argp->read_cnt)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		if (get_user(param1, &argp->param1)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		reg_read_buf = kmalloc(MAX_NUM_MTV_REG_READ_BUF, GFP_KERNEL);
		if (reg_read_buf == NULL) {
			DMBERR("Register buffer allocation error\n");
			ret = -ENOMEM;
			goto regio_exit;
		}

		if (param1 == 0) {
			diff_jiffies_1st = diff_jiffies0 = get_jiffies_64();
			hours_cnt = 0;
			DMBMSG("START [AGING SPI Memory Test with Single IO]\n");
		}

		RTV_REG_MAP_SEL(page);
		RTV_REG_SET(addr, write_data);

		RTV_REG_MAP_SEL(SPI_MEM_PAGE);
		RTV_REG_BURST_GET(0x10, reg_read_buf, read_cnt);

		RTV_REG_MAP_SEL(page);
		value = RTV_REG_GET(addr);

		diff_jiffies1 = get_jiffies_64();
		elapsed_ms = jiffies_to_msecs(diff_jiffies1-diff_jiffies0);
		if (elapsed_ms >= (1000 * 60 * 60)) {
			diff_jiffies0 = get_jiffies_64(); /* Re-start */
			hours_cnt++;
			DMBMSG("\t %lu hours elaspesed...\n", hours_cnt);
		}

		if (write_data != value) {
			unsigned int min, sec;
			elapsed_ms = jiffies_to_msecs(diff_jiffies1-diff_jiffies_1st);
			sec = elapsed_ms / 1000;
			min = sec / 60;			
			DMBMSG("END [AGING SPI Memory Test with Single IO]\n");
			DMBMSG("Total minutes: %u\n", min);
		}

		if (put_user(value, &argp->read_data[0]))
			ret = -EFAULT;

		kfree(reg_read_buf);
	#else
		DMBERR("Not SPI interface\n");
	#endif
		break;

	case IOCTL_TEST_REG_ONLY_SPI_MEM_READ:
	#if defined(RTV_IF_SPI)
		if (get_user(read_cnt, &argp->read_cnt)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		if (get_user(write_data, &argp->write_data)) {
			ret = -EFAULT;
			goto regio_exit;
		}

		reg_read_buf = kmalloc(MAX_NUM_MTV_REG_READ_BUF, GFP_KERNEL);
		if (reg_read_buf == NULL) {
			DMBERR("Register buffer allocation error\n");
			ret = -ENOMEM;
			goto regio_exit;
		}

		if (write_data == 0) /* only one-time page selection */
			RTV_REG_MAP_SEL(page);

		RTV_REG_BURST_GET(addr, reg_read_buf, read_cnt);
	
		src_ptr = reg_read_buf;
		dst_ptr = argp->read_data;

		for (i = 0; i< read_cnt; i++, src_ptr++, dst_ptr++) {
			if(put_user(*src_ptr, dst_ptr)) {
				ret = -EFAULT;
				break;
			}
		}
		kfree(reg_read_buf);
	#else
		DMBERR("Not SPI interface\n");
	#endif
		break;

	default:
		break;
	}

regio_exit:
	RTV_GUARD_FREE;

	return 0;
}

static int test_gpio(unsigned long arg, unsigned int cmd)
{
	unsigned int pin, value;
	IOCTL_GPIO_ACCESS_INFO __user *argp = (IOCTL_GPIO_ACCESS_INFO __user *)arg;

	if (get_user(pin, &argp->pin))
		return -EFAULT;

	switch (cmd) {
	case IOCTL_TEST_GPIO_SET:
		if(get_user(value, &argp->value))
			return -EFAULT;

		gpio_set_value(pin, value);
		break;

	case IOCTL_TEST_GPIO_GET:
		value = gpio_get_value(pin);
		if(put_user(value, &argp->value))
			return -EFAULT;
	}

	return 0;
}

static void test_power_on_off(unsigned int cmd)
{
	switch (cmd) {
	case IOCTL_TEST_MTV_POWER_ON:
		DMBMSG("IOCTL_TEST_MTV_POWER_ON\n");

		if (isdbt_cb_ptr->is_power_on == FALSE) {
			rtvOEM_PowerOn(1);

			/* Only for test command to confirm. */
			RTV_DELAY_MS(100);

			rtvMTV23x_Initialize(RTV_BW_MODE_571KHZ);

			isdbt_cb_ptr->is_power_on = TRUE;
		}
		break;

	case IOCTL_TEST_MTV_POWER_OFF:	
		if(isdbt_cb_ptr->is_power_on == TRUE) {
			rtvOEM_PowerOn(0);
			isdbt_cb_ptr->is_power_on = FALSE;
		}
		break;	
	}
}


/*==============================================================================
 * TDMB IO control commands(30 ~ 49)
 *============================================================================*/ 
static INLINE int isdbt_get_signal_qual_info(unsigned long arg)
{
	IOCTL_ISDBT_SIGNAL_QUAL_INFO sig;
	void __user *argp = (void __user *)arg;

	sig.lock_mask = rtvMTV23x_GetLockStatus();	
	sig.rssi = rtvMTV23x_GetRSSI();

	sig.ber_layer_A = rtvMTV23x_GetBER();
	sig.ber_layer_B = rtvMTV23x_GetBER2();

	sig.per_layer_A = rtvMTV23x_GetPER();
	sig.per_layer_B = rtvMTV23x_GetPER2();

	sig.cnr_layer_A = rtvMTV23x_GetCNR_LayerA();
	sig.ant_level_layer_A = rtvMTV23x_GetAntennaLevel_1seg(sig.cnr_layer_A);

	sig.cnr_layer_B = rtvMTV23x_GetCNR_LayerB();
	sig.ant_level_layer_B = rtvMTV23x_GetAntennaLevel(sig.cnr_layer_B);

	if (copy_to_user(argp, &sig, sizeof(IOCTL_ISDBT_SIGNAL_QUAL_INFO)))
		return -EFAULT;

	SHOW_TDMB_DEBUG_STAT;

	return 0;
}

static INLINE int isdbt_get_cnr(unsigned long arg)
{
	int cnr = (int)rtvMTV23x_GetCNR();

	if (put_user(cnr, (int *)arg))
		return -EFAULT;

	SHOW_TDMB_DEBUG_STAT;

	return 0;
}

static INLINE int isdbt_get_rssi(unsigned long arg)
{
	int rssi = rtvMTV23x_GetRSSI();

	if (put_user(rssi, (int *)arg))
		return -EFAULT;

	SHOW_TDMB_DEBUG_STAT;

	return 0;
}

static INLINE int isdbt_get_ber_per_info(unsigned long arg)
{
	IOCTL_ISDBT_BER_PER_INFO info;
	void __user *argp = (void __user *)arg;

	info.ber_layer_A = rtvMTV23x_GetBER();
	info.ber_layer_B = rtvMTV23x_GetBER2();

	info.per_layer_A = rtvMTV23x_GetPER();
	info.per_layer_B = rtvMTV23x_GetPER2();

	if (copy_to_user(argp, &info, sizeof(IOCTL_ISDBT_BER_PER_INFO)))
		return -EFAULT;

	SHOW_TDMB_DEBUG_STAT;

	return 0;
}

static INLINE void isdbt_disable_standby_mode(unsigned long arg)
{
	rtvMTV23x_StandbyMode(0);
}

static INLINE void isdbt_enable_standby_mode(unsigned long arg)
{
	rtvMTV23x_StandbyMode(1);

#if defined(RTV_IF_SPI)
	atomic_set(&isdbt_cb_ptr->isr_cnt, 0); /* Reset */
#endif
}

static INLINE int isdbt_get_lock_status(unsigned long arg)
{
	unsigned int lock_mask = rtvMTV23x_GetLockStatus();

	if (put_user(lock_mask, (unsigned int *)arg))
		return -EFAULT;

	return 0;
}

static INLINE int isdbt_get_signal_info(unsigned long arg)
{
	IOCTL_ISDBT_SIGNAL_INFO sig;
	void __user *argp = (void __user *)arg;

	sig.lock_mask = rtvMTV23x_GetLockStatus();	
	sig.ber = rtvMTV23x_GetBER();	 
	sig.cnr = rtvMTV23x_GetCNR(); 
	sig.per = rtvMTV23x_GetPER(); 
	sig.rssi = rtvMTV23x_GetRSSI();
	sig.ant_level = rtvMTV23x_GetAntennaLevel(sig.cnr);

	if (copy_to_user(argp, &sig, sizeof(IOCTL_ISDBT_SIGNAL_INFO)))
		return -EFAULT;

	SHOW_TDMB_DEBUG_STAT;

	return 0;
}

static INLINE int isdbt_set_channel(unsigned long arg)
{
	int ret = 0;
	unsigned int freq_khz, subch_id, intr_size;
	enum E_RTV_SERVICE_TYPE svc_type;
	enum E_RTV_BANDWIDTH_TYPE bw;
	IOCTL_ISDBT_SET_CH_INFO __user *argp
				= (IOCTL_ISDBT_SET_CH_INFO __user *)arg;

	if (get_user(freq_khz, &argp->freq_khz))
		return -EFAULT;

	if (get_user(subch_id, &argp->subch_id))
		return -EFAULT;

	if (get_user(svc_type, &argp->svc_type))
		return -EFAULT;

	if (get_user(bw, &argp->bandwidth))
		return -EFAULT;

	if (!((1<<svc_type) & ISDBT_VALID_SVC_MASK)) {
		DMBERR("Invaild service type: %d\n", svc_type);
		isdbt_cb_ptr->cfged_svc = RTV_SERVICE_INVALID;
		return -EINVAL;
	}

	DMBMSG("freq_khz(%u), subch_id(%u), svc_type(%u), bandwidth(%d)\n",
			freq_khz, subch_id, svc_type, bw);

#if defined(RTV_IF_SPI)
	intr_size = isdbt_cb_ptr->intr_size[svc_type];
#else
	intr_size = 0;
#endif

		isdbt_cb_ptr->cfged_svc = svc_type;

	ret = rtvMTV23x_SetFrequency(freq_khz, subch_id, svc_type, bw, intr_size);
	if (ret != RTV_SUCCESS) {
		DMBERR("failed: %d\n", ret);
		if (put_user(ret, &argp->tuner_err_code))
			return -EFAULT;

		return -EIO;
	}

	DMBMSG("Leave...\n");

	return 0;
}

static INLINE int isdbt_scan_channel(unsigned long arg)
{
	int ret;
	unsigned int freq_khz, subch_id, intr_size;
	enum E_RTV_SERVICE_TYPE svc_type;
	enum E_RTV_BANDWIDTH_TYPE bw;
	IOCTL_ISDBT_SCAN_INFO __user *argp
				= (IOCTL_ISDBT_SCAN_INFO __user *)arg;

	if (get_user(freq_khz, &argp->freq_khz))
		return -EFAULT;

	if (get_user(subch_id, &argp->subch_id))
		return -EFAULT;

	if (get_user(svc_type, &argp->svc_type))
		return -EFAULT;

	if (get_user(bw, &argp->bandwidth))
		return -EFAULT;

	if (!((1<<svc_type) & ISDBT_VALID_SVC_MASK)) {
		DMBERR("Invaild service type: %d\n", svc_type);
		isdbt_cb_ptr->cfged_svc = RTV_SERVICE_INVALID;
		return -EINVAL;
	}

#if defined(RTV_IF_SPI)
	intr_size = isdbt_cb_ptr->intr_size[svc_type];
#else
	intr_size = 0;
#endif

	isdbt_cb_ptr->cfged_svc = svc_type;

	ret = rtvMTV23x_ScanFrequency(freq_khz, subch_id, svc_type, bw, intr_size);
	if (ret == RTV_SUCCESS)
		return 0;
	else {
		if(ret != RTV_CHANNEL_NOT_DETECTED)
			DMBERR("Device error: %d\n", ret);

		/* Copy the tuner error-code to application */
		if (put_user(ret, &argp->tuner_err_code))
			return -EFAULT;

		return -EINVAL;
	}
}


#ifdef RTV_IF_CSI656_RAW_8BIT_ENABLE
extern int camif_isdbt_stop(void);
extern int camif_isdbt_start(unsigned int num_tsp);
extern int camif_isdbt_close(void);
extern int camif_isdbt_open(void);
#endif

static INLINE int isdbt_disable_ts_out(void)
{
	int ret = 0;

	DMBMSG("Enter\n");

	if (!isdbt_cb_ptr->tsout_enabled) {
		DMBMSG("Already TS out Disabled\n");
		return 0;
	}

	isdbt_cb_ptr->tsout_enabled = false;

	rtvMTV23x_DisableStreamOut();

#if defined(RTV_IF_SPI)
	atomic_set(&isdbt_cb_ptr->isr_cnt, 0); /* Reset */
#endif

#if defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
	ret = camif_isdbt_stop();
	if (ret < 0) {
		DMBERR("CAMIF stop error. ret(%d)", ret);
		return ret;
	}
#endif

	DMBMSG("Leave\n");

	return ret;
}

static INLINE int isdbt_enable_ts_out(void)
{
	int ret = 0;
#if defined(RTV_IF_SPI) || defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
	struct ISDBT_KSHMEM_CB *kshmem = &isdbt_cb_ptr->kshmem;
#endif

	//DMBMSG("ENTER\n");

	if (isdbt_cb_ptr->tsout_enabled) {
		DMBMSG("Already TS out Enabled\n");
		return 0;
	}

	RESET_DEBUG_INTR_STAT;
	RESET_DEBUG_TSPB_STAT;

	if (!((1<<isdbt_cb_ptr->cfged_svc) & ISDBT_VALID_SVC_MASK)) {
		DMBERR("Invaild configured service type: %d\n",
				isdbt_cb_ptr->cfged_svc);
		isdbt_cb_ptr->cfged_svc = RTV_SERVICE_INVALID;
		return -EINVAL;
	}

#if defined(RTV_IF_SPI) || defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
	/* Setup the kshmem stuff to process interrupt. */
	kshmem->avail_seg_idx = 0;
	kshmem->avail_write_tspb_idx = 0;
	kshmem->enqueue_seg_idx = 0;

	if (isdbt_cb_ptr->intr_size[isdbt_cb_ptr->cfged_svc]
			!= isdbt_cb_ptr->cfged_tsp_chunk_size) {
		isdbt_cb_ptr->cfged_tsp_chunk_size
			= isdbt_cb_ptr->intr_size[isdbt_cb_ptr->cfged_svc];

		kshmem->num_tsp_per_seg
			= isdbt_cb_ptr->intr_size[isdbt_cb_ptr->cfged_svc]
					/ RTV_TSP_XFER_SIZE;

		kshmem->num_total_tsp
			= kshmem->num_tsp_per_seg * kshmem->num_total_seg;
	}

	DMBMSG("svc_type(%d), #tsp_per_seg(%u), #seg(%u), #total_tsp(%u)\n",
		isdbt_cb_ptr->cfged_svc, kshmem->num_tsp_per_seg,
		kshmem->num_total_seg, kshmem->num_total_tsp);
#endif

#if defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
	ret = camif_isdbt_start(kshmem->num_tsp_per_seg);
	if (ret < 0) {
		DMBERR("CAMIF start error. ret(%d)", ret);
		return ret;
	}
#endif

	isdbt_cb_ptr->tsout_enabled = true;

	rtvMTV23x_EnableStreamOut();	

	//DMBMSG("END\n");

	return ret;
}

static INLINE int __isdbt_deinit(void)
{
    int ret = 0;

	isdbt_disable_ts_out();
 
#if defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
    ret = camif_isdbt_close();
    if (ret < 0)
        DMBERR("CAMIF close error. ret(%d)", ret);
#endif

    return ret;
}

static INLINE int __isdbt_power_on(unsigned long arg)
{
    int ret;
	enum E_RTV_BANDWIDTH_TYPE bandwidth;
	IOCTL_ISDBT_POWER_ON_INFO __user *argp
		= (IOCTL_ISDBT_POWER_ON_INFO __user *)arg;
#if defined(RTV_IF_SPI) || defined(RTV_IF_CSI656_RAW_8BIT_ENABLE)
	int i;

	for (i = 0; i < MAX_NUM_RTV_SERVICE; i++) {
		if (get_user(isdbt_cb_ptr->intr_size[i],
				&argp->spi_intr_size[i]))
			return -EFAULT;

		DMBMSG("intr_size[%d]: %u\n", i, isdbt_cb_ptr->intr_size[i]);
	}

	isdbt_cb_ptr->cfged_tsp_chunk_size = 0;
#endif

	if (get_user(bandwidth,	&argp->bandwidth))
		return -EFAULT;

	isdbt_cb_ptr->tsout_enabled = false;
	isdbt_cb_ptr->cfged_svc = RTV_SERVICE_INVALID;

	ret = rtvMTV23x_Initialize(bandwidth);
	if (ret != RTV_SUCCESS) {
		DMBERR("Tuner initialization failed: %d\n", ret);
		if (put_user(ret, &argp->tuner_err_code))
			return -EFAULT;

		return -EIO; /* error occurred during the open() */
	}

#ifdef RTV_IF_CSI656_RAW_8BIT_ENABLE
	ret = camif_isdbt_open();
	if (ret < 0) {
		DMBERR("CAMIF open error. ret(%d)", ret);
		return ret;
	}
#endif

	return ret;
}


