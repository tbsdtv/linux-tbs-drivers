/*
 * Silicon Labs Si2146/2147/2148/2157/2158 silicon tuner driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef SI2157_H
#define SI2157_H

#include <linux/kconfig.h>
#include "dvb_frontend.h"

#define SI2157_CHIPTYPE_SI2157 0
#define SI2157_CHIPTYPE_SI2146 1

struct si2157_config {
	u8 i2c_addr;

	u8 chiptype;
	/*
	 * Spectral Inversion
	 */
	bool inversion;

	/*
	 * Port selection
	 * Select the RF interface to use (pins 9+11 or 12+13)
	 */
	u8 if_port;
};

#if defined(CONFIG_MEDIA_TUNER_SI2157) || \
	(defined(CONFIG_MEDIA_TUNER_SI2157_MODULE) && defined(MODULE))
extern struct dvb_frontend *si2157_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c, struct si2157_config *cfg);
#else
static inline struct dvb_frontend *si2157_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c, struct si2157_config *cfg)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif
