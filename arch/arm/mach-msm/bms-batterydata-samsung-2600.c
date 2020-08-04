/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20, 0, 25, 40, 60},
	.y		= {2589, 2589, 2587, 2577, 2564},
	.cols	= 5
};

static struct single_row_lut fcc_sf = {
	.x		= {0},
	.y		= {100},
	.cols	= 1
};

static struct sf_lut rbatt_sf = {
	.rows		= 30,
	.cols		= 5,
	.row_entries		= {-20, 0, 25, 40, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 16, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1},
	.sf		= {
				{232, 232, 100, 84, 78},
				{232, 232, 100, 84, 78},
				{234, 234, 101, 85, 78},
				{238, 238, 102, 86, 79},
				{245, 245, 105, 89, 81},
				{260, 260, 112, 94, 82},
				{236, 236, 111, 93, 85},
				{230, 230, 120, 97, 86},
				{222, 222, 125, 102, 90},
				{211, 211, 117, 104, 92},
				{209, 209, 100, 86, 80},
				{208, 208, 97, 84, 79},
				{212, 212, 98, 86, 81},
				{223, 223, 99, 88, 83},
				{240, 240, 102, 88, 82},
				{262, 262, 105, 86, 79},
				{280, 280, 104, 86, 79},
				{287, 287, 102, 87, 80},
				{273, 273, 99, 85, 78},
				{279, 279, 97, 84, 78},
				{288, 288, 99, 85, 80},
				{296, 296, 101, 86, 81},
				{308, 308, 104, 88, 82},
				{324, 324, 106, 90, 85},
				{342, 342, 110, 93, 88},
				{365, 365, 113, 96, 91},
				{378, 378, 110, 94, 83},
				{397, 397, 109, 90, 84},
				{440, 440, 116, 96, 88},
				{604, 604, 137, 116, 106},
	}
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20, 0, 25, 40, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 16, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4326, 4326, 4322, 4317, 4310},
				{4222, 4244, 4249, 4247, 4243},
				{4158, 4188, 4192, 4190, 4187},
				{4102, 4136, 4139, 4137, 4134},
				{4052, 4088, 4089, 4087, 4084},
				{3963, 4041, 4045, 4042, 4036},
				{3914, 3968, 3984, 3989, 3993},
				{3875, 3926, 3955, 3957, 3954},
				{3845, 3891, 3921, 3922, 3919},
				{3826, 3856, 3878, 3883, 3882},
				{3812, 3829, 3837, 3838, 3838},
				{3800, 3807, 3813, 3814, 3813},
				{3787, 3792, 3795, 3796, 3795},
				{3773, 3783, 3781, 3782, 3781},
				{3758, 3774, 3772, 3769, 3765},
				{3743, 3765, 3764, 3756, 3744},
				{3727, 3747, 3747, 3738, 3724},
				{3714, 3723, 3724, 3716, 3703},
				{3703, 3703, 3696, 3689, 3676},
				{3692, 3695, 3688, 3681, 3669},
				{3687, 3693, 3687, 3680, 3668},
				{3680, 3691, 3686, 3679, 3667},
				{3672, 3688, 3685, 3678, 3667},
				{3662, 3686, 3684, 3677, 3665},
				{3647, 3683, 3681, 3674, 3663},
				{3625, 3676, 3675, 3668, 3655},
				{3592, 3653, 3648, 3643, 3625},
				{3548, 3604, 3594, 3592, 3571},
				{3487, 3532, 3517, 3519, 3498},
				{3388, 3422, 3404, 3415, 3395},
				{3200, 3200, 3200, 3200, 3200}
	}
};

struct bms_battery_data Samsung_8x26_2600mAh_data = {
	.fcc				= 2600,
	.fcc_temp_lut			= &fcc_temp,
	.fcc_sf_lut				= &fcc_sf,
	.pc_temp_ocv_lut		= &pc_temp_ocv,
	.rbatt_sf_lut			= &rbatt_sf,
	.default_rbatt_mohm	= 175
};
