#ifndef __ISDBT_GPIO_H__
#define __ISDBT_GPIO_H__


#ifdef __cplusplus 
extern "C"{ 
#endif  

#if defined(CONFIG_ARCH_S5PV310)// for Hardkernel
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>

#define PORT_CFG_OUTPUT                 1
#define MTV_PWR_EN                      S5PV310_GPD0(3)         
#define MTV_PWR_EN_CFG_VAL              S3C_GPIO_SFN(PORT_CFG_OUTPUT)
#define ISDBT_IRQ_INT                  S5PV310_GPX0(2)

static inline int isdbt_configure_gpio(void)
{
	if (gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
		DMBMSG("MTV_PWR_EN Port request error!!!\n");
	else {
		// MTV_EN
		s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
		s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
		gpio_direction_output(MTV_PWR_EN, 0); // power down
	}

	return 0;
}

#elif defined(CONFIG_ARCH_S5PV210)//for MV210
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>

#define S5PV210_GPD0_PWM_TOUT_0         (0x1 << 0)
#define MTV_PWR_EN                      S5PV210_GPD0(0) 
#define MTV_PWR_EN_CFG_VAL              S5PV210_GPD0_PWM_TOUT_0 

#define ISDBT_IRQ_INT                  IRQ_EINT6
#ifdef gpio_to_irq
	#undef gpio_to_irq
	#define	gpio_to_irq(x)		(x)
#endif

static inline int isdbt_configure_gpio(void)
{
	if (gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
		DMBMSG("MTV_PWR_EN Port request error!!!\n");
	else {
		// MTV_EN
		s3c_gpio_cfgpin(MTV_PWR_EN, MTV_PWR_EN_CFG_VAL);
		s3c_gpio_setpull(MTV_PWR_EN, S3C_GPIO_PULL_NONE);
		gpio_direction_output(MTV_PWR_EN, 0); // power down
	}

	return 0;
}

#elif defined(CONFIG_MACH_MS01DTV_LTN)//for msm8226

#define MTV_PWR_EN                      110
#define MTV_PWR_EN2                     36
#ifdef _ISDBT_DEVICE_TREE_UNUSED
#define GPIO_ISDBT_IN                   49
#define GPIO_ISDBT_MOSI                 20
#define GPIO_ISDBT_MISO                 21
#define GPIO_ISDBT_CS                   22
#define GPIO_ISDBT_CLK                  23
#endif

/*
#ifdef gpio_to_irq
	#undef gpio_to_irq
	#define	gpio_to_irq(x)		(x)
#endif
*/
#if 0
static inline int isdbt_configure_gpio(void)
{
	return 0;
	}
#endif

#else
	#error "code not present"
#endif

#ifdef __cplusplus 
} 
#endif 

#endif /* __ISDBT_GPIO_H__*/
