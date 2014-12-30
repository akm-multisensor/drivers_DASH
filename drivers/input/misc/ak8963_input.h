/* drivers/input/misc/ak8963_input.h - AK8963 compass driver
 *
 * Copyright (C) 2012 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _AK8963_INPUT_H_
#define _AK8963_INPUT_H_

struct device;
struct ak8963_data;

struct ak8963_bus_ops {
	u16 bustype;
	int (*rxdata)(struct device *dev, unsigned char *rxdata, int length);
	int (*txdata)(struct device *dev, unsigned char *txdata, int length);
};

int ak8963_suspend(struct ak8963_data *akm);
int ak8963_resume(struct ak8963_data *akm);
struct ak8963_data *ak8963_probe(struct device *dev, int irq,
		const struct ak8963_bus_ops *bops);
int ak8963_remove(struct ak8963_data *akm);

#endif
