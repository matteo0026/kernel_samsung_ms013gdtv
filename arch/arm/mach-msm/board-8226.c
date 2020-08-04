/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/memory.h>
#include <linux/regulator/qpnp-regulator.h>
#include <linux/msm_tsens.h>
#include <linux/export.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/restart.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#ifdef CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#include <linux/persistent_ram.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/socinfo.h>
#include <mach/board.h>
#include <mach/clk-provider.h>
#include <mach/msm_smd.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/msm_smem.h>
#include <linux/msm_thermal.h>
#include "board-dt.h"
#include "clock.h"
#include "platsmp.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#ifdef CONFIG_ISDBT_FC8300
#include <media/isdbt_pdata.h>
#endif

#ifdef CONFIG_PROC_AVC
#include <linux/proc_avc.h>
#endif

#if defined(CONFIG_MACH_MS01) || defined(CONFIG_MACH_CRATERVE) || defined(CONFIG_MACH_CT01) || \
	defined(CONFIG_MACH_S3VE) || defined(CONFIG_MACH_S3VECTC) || defined(CONFIG_MACH_MS01_LTE) || \
	defined(CONFIG_MACH_CS03_SGLTE) || defined(CONFIG_MACH_CRATERQ) || defined(CONFIG_MACH_MS01_LTE_KOR) || \
	defined(CONFIG_MACH_MS01_CHN_CMCC_3G) || defined(CONFIG_MACH_BAFFIN2_SGLTE) 	
#include <mach/msm8x26-thermistor.h>
#endif

static struct memtype_reserve msm8226_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
/* CONFIG_SEC_DEBUG reserving memory for persistent RAM*/
#define RAMCONSOLE_PHYS_ADDR 0x1FB00000
static struct persistent_ram_descriptor per_ram_descs[] __initdata = {
       {
               .name = "ram_console",
               .size = SZ_1M,
       }
};

static struct persistent_ram per_ram __initdata = {
       .descs = per_ram_descs,
       .num_descs = ARRAY_SIZE(per_ram_descs),
       .start = RAMCONSOLE_PHYS_ADDR,
       .size = SZ_1M
};
#endif
static int msm8226_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#if defined(CONFIG_ISDBT_FC8300)
#define GPIO_ISDBT_RST_N	67
#define GPIO_ISDBT_EN		110
#define GPIO_ISDBT_INT		49
#define GPIO_ISDBT_IRQ		gpio_to_irq(GPIO_ISDBT_INT)

#define GPIO_ISDBT_DET_IRQ		gpio_to_irq(GPIO_ISDBT_INT)


#define GPIO_ISDBT_EBI2_ADDR0	121
#define GPIO_ISDBT_EBI2_ADDR1	120


#define GPIO_LEVEL_HIGH		1
#define GPIO_LEVEL_LOW		0

static void isdbt_set_config_poweron(void)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_ISDBT_EN, GPIO_LEVEL_LOW);


	gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_ISDBT_RST_N, GPIO_LEVEL_LOW);

	// ROY_LTN_DTV, set GPIO_ISDBT_INT
	gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

}
static void isdbt_set_config_poweroff(void)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_ISDBT_EN, GPIO_LEVEL_LOW);

	gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_ISDBT_RST_N, GPIO_LEVEL_LOW);

	gpio_set_value(GPIO_ISDBT_INT, GPIO_LEVEL_LOW);


}

static void isdbt_gpio_on(void)
{
	isdbt_set_config_poweron();

	gpio_set_value(GPIO_ISDBT_EN, GPIO_LEVEL_LOW);
	msleep(1000); // usleep_range(1000, 1000);
	gpio_set_value(GPIO_ISDBT_EN, GPIO_LEVEL_HIGH);

	msleep(1000); // usleep_range(1000, 1000);
	gpio_set_value(GPIO_ISDBT_RST_N, GPIO_LEVEL_LOW);
	msleep(2000); //usleep_range(2000, 2000);
	gpio_set_value(GPIO_ISDBT_RST_N, GPIO_LEVEL_HIGH);
	msleep(1000); //usleep_range(1000, 1000);

}

static void isdbt_gpio_off(void)
{
	isdbt_set_config_poweroff();

	gpio_set_value(GPIO_ISDBT_RST_N, GPIO_LEVEL_LOW);
	msleep(1000); //usleep_range(1000, 1000);

	gpio_set_value(GPIO_ISDBT_EN, GPIO_LEVEL_LOW);
}

static struct isdbt_platform_data isdbt_pdata = {
	.gpio_on = isdbt_gpio_on,
	.gpio_off = isdbt_gpio_off,
};

static struct platform_device isdbt_device = {
	.name			= "isdbt",
	.id				= -1,
	.dev			= {
		.platform_data = &isdbt_pdata,
	},
};

static int __init isdbt_dev_init(void)
{

	isdbt_set_config_poweroff();
	// s5p_register_gpio_interrupt(GPIO_ISDBT_INT);
	isdbt_pdata.irq = GPIO_ISDBT_IRQ;
	platform_device_register(&isdbt_device);

	printk(KERN_ERR "isdbt_dev_init\n");

	return 0;
}
#endif

static struct of_dev_auxdata msm8226_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	{}
};

static struct reserve_info msm8226_reserve_info __initdata = {
	.memtype_reserve_table = msm8226_reserve_table,
	.paddr_to_memtype = msm8226_paddr_to_memtype,
};

static void __init msm8226_early_memory(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, msm8226_reserve_table);
}

static void __init msm8226_reserve(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8226_reserve_table);
	msm_reserve();
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
	persistent_ram_early_init(&per_ram);
#endif
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8226_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	qpnp_regulator_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8226_rumi_clock_init_data);
	else
		msm_clock_init(&msm8226_clock_init_data);
	tsens_tm_init_driver();
#if defined(CONFIG_MACH_MS01) || defined(CONFIG_MACH_CRATERVE) || defined(CONFIG_MACH_CT01) || \
	defined(CONFIG_MACH_S3VE) || defined(CONFIG_MACH_S3VECTC) || defined(CONFIG_MACH_MS01_LTE) || \
	defined(CONFIG_MACH_CS03_SGLTE) || defined(CONFIG_MACH_CRATERQ) || defined(CONFIG_MACH_MS01_LTE_KOR) || \
	defined(CONFIG_MACH_MS01_CHN_CMCC_3G) || defined(CONFIG_MACH_BAFFIN2_SGLTE)
	platform_device_register(&sec_device_thermistor);
#endif
	msm_thermal_device_init();
}

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static void samsung_sys_class_init(void)
{
	pr_info("samsung sys class init.\n");

	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class)) {
		pr_err("Failed to create class(sec)!\n");
		return;
	}

	pr_info("samsung sys class end.\n");
};

#if defined(CONFIG_BATTERY_SAMSUNG)
extern void __init samsung_init_battery(void);
#endif

void __init msm8226_init(void)
{
	struct of_dev_auxdata *adata = msm8226_auxdata_lookup;

#ifdef CONFIG_SEC_DEBUG
	sec_debug_init();
#endif

#ifdef CONFIG_PROC_AVC
	sec_avc_log_init();
#endif

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8226_init_gpiomux();
	board_dt_populate(adata);
	samsung_sys_class_init();
	msm8226_add_drivers();
#if defined(CONFIG_BATTERY_SAMSUNG)
	samsung_init_battery();
#endif

#ifdef CONFIG_ISDBT_FC8300 
	isdbt_dev_init();
	pr_info("msm8226_init.\n");

#endif

}

static const char *msm8226_dt_match[] __initconst = {
	"qcom,msm8226",
	"qcom,msm8926",
	"qcom,apq8026",
	NULL
};

DT_MACHINE_START(MSM8226_DT, "Qualcomm MSM 8226 (Flattened Device Tree)")
	.map_io = msm_map_msm8226_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8226_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8226_dt_match,
	.reserve = msm8226_reserve,
	.init_very_early = msm8226_early_memory,
	.restart = msm_restart,
	.smp = &arm_smp_ops,
MACHINE_END
