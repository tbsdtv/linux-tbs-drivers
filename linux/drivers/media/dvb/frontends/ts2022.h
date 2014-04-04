  /*
     Driver for Montage TS2022 DVBS/S2 Silicon tuner

     Copyright (C) 2012 Tomazzo Muzumici

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the

     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

  */

#ifndef TS2022_H
#define TS2022_H

#include <linux/dvb/frontend.h>

/**
 * Attach a ts2022 tuner to the supplied frontend structure.
 *
 * @param fe Frontend to attach to.
 * @param addr i2c address of the tuner.
 * @param i2c i2c adapter to use.
 * @return FE pointer on success, NULL on failure.
 */
#if defined(CONFIG_DVB_TS2022) || \
	(defined(CONFIG_DVB_TS2022_MODULE) && defined(MODULE))
extern struct dvb_frontend *ts2022_attach(struct dvb_frontend *fe, int addr,
					   struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *ts2022_attach(struct dvb_frontend *fe,
						  int addr,
						  struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_TS2022 */

#endif /* TS2022_H */
