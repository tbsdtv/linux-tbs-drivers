/*
 * Conexant CX24120/CX24118 - DVB-S/S2 demod/tuner driver
 *
 * Copyright (C) 2008 Patrick Boettcher <pb@linuxtv.org>
 * Copyright (C) 2009 Sergey Tyurin <forum.free-x.de>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef CX24120_H
#define CX24120_H

#include <linux/dvb/frontend.h>

struct firmware;
struct dvb_frontend;
struct i2c_adapter;

struct cx24120_config
{
	u8 i2c_addr;
	int (*request_firmware)(struct dvb_frontend *fe, const struct firmware **fw, char *name);
	void (*stream_control)(struct dvb_frontend *fe, u8 onoff);
};

#if defined(CONFIG_DVB_CX24120) || \
	(defined(CONFIG_DVB_CX24120_MODULE) && defined(MODULE))
extern struct dvb_frontend *cx24120_attach(const struct cx24120_config *config,
		struct i2c_adapter *i2c);
extern int cx24120_reset(struct dvb_frontend *fe);
#else
static inline
struct dvb_frontend *cx24120_attach(const struct cx24120_config *config,
		struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
static inline int cx24120_reset(struct dvb_frontend *fe)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return -ENODEV;
}
#endif

#endif
