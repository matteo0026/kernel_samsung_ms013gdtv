/******************************************************************************
* (c) COPYRIGHT 2013 RAONTECH, Inc. ALL RIGHTS RESERVED.
*
* This software is the property of RAONTECH and is furnished under license
* by RAONTECH.
* This software may be used only in accordance with the terms of said license.
* This copyright notice may not be removed, modified or obliterated without
* the prior written permission of RAONTECH, Inc.
*
* This software may not be copied, transmitted, provided to or otherwise
* made available to any other person, company, corporation or other entity
* except as specified in the terms of said license.
*
* No right, title, ownership or other interest in the software is hereby
* granted or transferred.
*
* The information contained herein is subject to change without notice
* and should not be construed as a commitment by RAONTECH, Inc.
*
* TITLE		: MTV23x porting source file.
*
* FILENAME	: mtv23x_port.c
*
* DESCRIPTION	: User-supplied Routines for RAONTECH TV Services.
*
******************************************************************************/
/******************************************************************************
* REVISION HISTORY
*
*    DATE         NAME          REMARKS
* ----------  -------------    ------------------------------------------------
* 07/12/2013  Ko, Kevin        Created.
******************************************************************************/

#include "mtv23x.h"
#include "mtv23x_internal.h"


/* Declares a variable of gurad object if neccessry. */
#if defined(RTV_IF_SPI) || defined(RTV_IF_EBI2)
	#if defined(__KERNEL__)
	struct mutex raontv_guard;
	#elif defined(WINCE) || defined(WINDOWS) || defined(WIN32)
        CRITICAL_SECTION raontv_guard;
    #else
	/* non-OS and RTOS. */
	#endif
#endif

#include "isdbt.h"
#include "isdbt_gpio.h"

void rtvOEM_PowerOn(int on)
{
	if (on) {
		/* Set the GPIO of MTV_EN pin to low. */
		gpio_set_value(MTV_PWR_EN, 0);
#if defined(CONFIG_MACH_MS01DTV_LTN)//for msm8226
		gpio_set_value(MTV_PWR_EN2, 0);
#endif
		RTV_DELAY_MS(1);

		/* Set the GPIO of MTV_EN pin to high. */
		gpio_set_value(MTV_PWR_EN, 1);
#if defined(CONFIG_MACH_MS01DTV_LTN)//for msm8226
		gpio_set_value(MTV_PWR_EN2, 1);
#endif
		RTV_DELAY_MS(1);
		RTV_GUARD_INIT;
	} else {
		/* Set the GPIO of MTV_EN pin to low. */
#if defined(CONFIG_MACH_MS01DTV_LTN)//for msm8226		
		gpio_set_value(MTV_PWR_EN2, 0);
#endif
		gpio_set_value(MTV_PWR_EN, 0);
		RTV_DELAY_MS(1);

		RTV_GUARD_DEINIT;
	}
}

