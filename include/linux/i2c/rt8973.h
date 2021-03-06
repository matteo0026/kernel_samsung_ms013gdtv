/*
 * Copyright (C) 2013 Samsung Electronics
 * Jeongrae Kim <jryu.kim@samsung.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _RT8973_H_
#define _RT8973_H_

//#include "fsa880.h"
#include "tsu6721.h"

enum {
	FSA880_DETACHED,
	FSA880_ATTACHED
};

struct rt8973_platform_data {
	void (*callback)(enum cable_type_t cable_type, int attached);
	void (*mhl_sel) (bool onoff);
	int	(*dock_init) (void);
	int gpio_int;
	u32 irq_gpio_flags;
	int gpio_sda;
	u32 sda_gpio_flags;
	int gpio_scl;
	u32 scl_gpio_flags;
};

extern struct rt8973_platform_data rt8973_pdata;

extern void rt8973_callback(enum cable_type_t cable_type, int attached);
extern int rt8973_dock_init(void);

#endif /* _RT8973_H_ */
