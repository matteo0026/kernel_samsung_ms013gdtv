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
	.y		= {2050, 2067, 2063, 2063, 2055},
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
				{1198, 361, 100, 77, 72},
				{1064, 361, 101, 78, 73},
				{1070, 361, 101, 78, 73},
				{1021, 359, 104, 80, 74},
				{926, 372, 107, 82, 75},
				{899, 329, 109, 84, 76},
				{886, 317, 116, 86, 78},
				{877, 319, 122, 91, 81},
				{888, 313, 114, 97, 86},
				{912, 309, 100, 82, 77},
				{939, 308, 97, 78, 74},
				{968, 320, 98, 79, 75},
				{1000, 342, 100, 82, 77},
				{1034, 368, 101, 83, 79},
				{1073, 400, 104, 83, 77},
				{1138, 436, 106, 82, 75},
				{1290, 466, 108, 82, 76},
				{1487, 508, 108, 81, 75},
				{1700, 550, 109, 81, 75},
				{1889, 585, 112, 82, 76},
				{1976, 605, 114, 83, 77},
				{2050, 590, 116, 85, 78},
				{1995, 590, 117, 86, 80},
				{2128, 613, 121, 88, 82},
				{2365, 632, 127, 91, 83},
				{2694, 667, 128, 88, 79},
				{3151, 698, 128, 87, 78},
				{3870, 750, 135, 92, 83},
				{5005, 938, 150, 103, 96},
				{12568, 63595, 223, 165, 3079},
	}
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20, 0, 25, 40, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 16, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4297, 4297, 4293, 4289, 4281},
				{4192, 4216, 4225, 4225, 4221},
				{4124, 4160, 4170, 4170, 4166},
				{4080, 4107, 4119, 4118, 4114},
				{3982, 4069, 4071, 4069, 4065},
				{3932, 3989, 4016, 4021, 4020},
				{3890, 3939, 3977, 3981, 3978},
				{3854, 3904, 3943, 3945, 3942},
				{3831, 3869, 3899, 3907, 3906},
				{3817, 3839, 3856, 3859, 3859},
				{3804, 3811, 3827, 3829, 3828},
				{3791, 3795, 3807, 3808, 3807},
				{3779, 3785, 3790, 3791, 3790},
				{3765, 3776, 3777, 3777, 3777},
				{3751, 3765, 3769, 3764, 3758},
				{3736, 3750, 3758, 3750, 3738},
				{3719, 3727, 3738, 3731, 3717},
				{3703, 3712, 3710, 3703, 3690},
				{3687, 3702, 3692, 3685, 3672},
				{3673, 3695, 3686, 3680, 3668},
				{3665, 3692, 3684, 3679, 3668},
				{3655, 3688, 3683, 3678, 3667},
				{3643, 3683, 3682, 3676, 3665},
				{3628, 3675, 3680, 3675, 3663},
				{3610, 3659, 3675, 3669, 3655},
				{3584, 3633, 3657, 3647, 3628},
				{3550, 3590, 3614, 3601, 3580},
				{3504, 3525, 3551, 3537, 3515},
				{3438, 3424, 3462, 3449, 3426},
				{3330, 3259, 3325, 3308, 3275},
				{3000, 3000, 3000, 3000, 3000}
	}
};

struct bms_battery_data Samsung_8x26_S3LITE_2100mAh_data = {
	.fcc				= 2100,
	.fcc_temp_lut			= &fcc_temp,
	.fcc_sf_lut				= &fcc_sf,
	.pc_temp_ocv_lut		= &pc_temp_ocv,
	.rbatt_sf_lut			= &rbatt_sf,
	.default_rbatt_mohm	= 208
};
