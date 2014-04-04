  /*
     Driver for Montage ts2022 DVBS/S2 Silicon tuner

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

#include "dvb_frontend.h"
#include "ts2022.h"

static int debug;
#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "ts2022: " args); \
	} while (0)

#define TS2022_XTAL_FREQ   27000 /* in kHz */

struct ts2022_priv {
	/* i2c details */
	int i2c_address;
	struct i2c_adapter *i2c;
	u32 frequency;
};

static int ts2022_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static int ts2022_writereg(struct dvb_frontend *fe, int reg, int data)
{
	struct ts2022_priv *priv = fe->tuner_priv;
	u8 buf[] = { reg, data };
	struct i2c_msg msg[] = {
		{
			.addr = priv->i2c_address,
			.flags = 0,
			.buf = buf,
			.len = 2
		}
	};
	int err;

	dprintk("%s: write reg 0x%02x, value 0x%02x\n", __func__, reg, data);
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	err = i2c_transfer(priv->i2c, msg, 1);
	if (err != 1) {
		printk("%s: writereg error(err == %i, reg == 0x%02x,"
		" value == 0x%02x)\n", __func__, err, reg, data);
		return -EREMOTEIO;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;
}

static int ts2022_readreg(struct dvb_frontend *fe, u8 reg)
{
	struct ts2022_priv *priv = fe->tuner_priv;
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = priv->i2c_address,
			.flags = 0,
			.buf = b0,
			.len = 1
		}, {
			.addr = priv->i2c_address,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = i2c_transfer(priv->i2c, msg, 2);
	
	if (ret != 2) {
		printk(KERN_ERR "%s: reg=0x%x(error=%d)\n", __func__, reg, ret);
		return ret;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	dprintk("%s: read reg 0x%02x, value 0x%02x\n", __func__, reg, b1[0]);
	
	return b1[0];
}

static int ts2022_sleep(struct dvb_frontend *fe)
{
	struct ts2022_priv *priv = fe->tuner_priv;
	int ret;
	u8 buf[] = { 10, 0 };
	struct i2c_msg msg = {
		.addr = priv->i2c_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	dprintk("%s:\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret != 1)
		dprintk("%s: i2c error\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return (ret == 1) ? 0 : ret;
}

static int ts2022_set_params(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct ts2022_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 mlpf, mlpf_new, mlpf_max, mlpf_min, nlpf, div4;
	u16 value, ndiv;
	u32 f3db;

	dprintk("%s:\n", __func__);

	ts2022_writereg(fe, 0x10, 0x0b);
	ts2022_writereg(fe, 0x11, 0x40);
	div4 = 0;
	if (c->frequency < 1103000) {
		ts2022_writereg(fe, 0x10, 0x1b);
		div4 = 1;
		ndiv = (c->frequency * (6 + 8) * 4)/TS2022_XTAL_FREQ ;
	} else
		ndiv = (c->frequency * (6 + 8) * 2)/TS2022_XTAL_FREQ ;

	ndiv = ndiv + ndiv %2 ;
	if (ndiv < 4095)
		value = ndiv - 1024;
	else if (ndiv < 6143 )
		value = ndiv + 1024;
	else
		value = ndiv + 3072;

	ts2022_writereg(fe, 0x01, (value & 0x3f00) >> 8);
	ts2022_writereg(fe, 0x02, value & 0x00ff);
	ts2022_writereg(fe, 0x03, 0x06);
	ts2022_writereg(fe, 0x51, 0x0f);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x10);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);

	value =  ts2022_readreg(fe, 0x14);
	value &=0x7f;
	if (value < 64 ) {
		value =  ts2022_readreg(fe, 0x10);
		value |= 0x80;
		ts2022_writereg(fe, 0x10, value);
		ts2022_writereg(fe, 0x11, 0x6f);

		ts2022_writereg(fe, 0x51, 0x0f);
		ts2022_writereg(fe, 0x51, 0x1f);
		ts2022_writereg(fe, 0x50, 0x10);
		ts2022_writereg(fe, 0x50, 0x00);
	}
	msleep(5);
	value =  ts2022_readreg(fe, 0x14);
	value &=0x1f;
	if (value > 19) {
		value =  ts2022_readreg(fe, 0x10);
		value &= 0x1d;
		ts2022_writereg(fe, 0x10, value);
	}
	/*set the RF gain*/
	ts2022_writereg(fe, 0x60, 0x79);

	ts2022_writereg(fe, 0x51, 0x17);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x08);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);

	ts2022_writereg(fe, 0x25, 0x00);
	ts2022_writereg(fe, 0x27, 0x70);
	ts2022_writereg(fe, 0x41, 0x09);

	ts2022_writereg(fe, 0x08, 0x0b);
	ts2022_writereg(fe, 0x04, 0x2e);
	ts2022_writereg(fe, 0x51, 0x1b);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x04);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);

	f3db = ((c->symbol_rate / 1000) * 135) / 200 + 2000;
	if ((c->symbol_rate / 1000) < 5000)
		f3db += 3000;
	if (f3db < 7000)
		f3db = 7000;
	if (f3db > 40000)
		f3db = 40000;

	value = ts2022_readreg(fe, 0x26);
	value &= 0x3f ;

	ts2022_writereg(fe, 0x41, 0x0d);

	ts2022_writereg(fe, 0x51, 0x1b);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x04);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);
	value = (value + (ts2022_readreg(fe, 0x26) & 0x3f)) / 2;
	mlpf = 0x2e * 207 / ((value << 1) + 151);
	mlpf_max = mlpf * 135 / 100;
	mlpf_min = mlpf * 78 / 100;
	if (mlpf_max > 63)
		mlpf_max = 63;


		value = 3200;
	nlpf = ((mlpf * f3db * 1000) + (value * TS2022_XTAL_FREQ / 2))
			/ (value * TS2022_XTAL_FREQ);

	if (nlpf > 23)
		nlpf = 23;
	if (nlpf < 1)
		nlpf = 1;

	/* rounded to the closest integer */
	mlpf_new = ((TS2022_XTAL_FREQ * nlpf * value) +
			(1000 * f3db / 2)) / (1000 * f3db);

	if (mlpf_new < mlpf_min) {
		nlpf++;
		mlpf_new = ((TS2022_XTAL_FREQ * nlpf * value) +
				(1000 * f3db / 2)) / (1000 * f3db);
	}

	if (mlpf_new > mlpf_max)
		mlpf_new = mlpf_max;

	ts2022_writereg(fe, 0x04, mlpf_new);
	ts2022_writereg(fe, 0x06, nlpf);
	ts2022_writereg(fe, 0x51, 0x1b);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x04);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);

	value = ts2022_readreg(fe, 0x26);
	value &= 0x3f;
	ts2022_writereg(fe, 0x41, 0x09);

	ts2022_writereg(fe, 0x51, 0x1b);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x04);
	ts2022_writereg(fe, 0x50, 0x00);
	msleep(5);
	value = (value + (ts2022_readreg(fe, 0x26)&0x3f))/2;

	value |= 0x80;
	ts2022_writereg(fe, 0x25, value);
	ts2022_writereg(fe, 0x27, 0x30);
	ts2022_writereg(fe, 0x08, 0x09);
	ts2022_writereg(fe, 0x51, 0x1e);
	ts2022_writereg(fe, 0x51, 0x1f);
	ts2022_writereg(fe, 0x50, 0x01);
	ts2022_writereg(fe, 0x50, 0x00);

	msleep(60);

	priv->frequency = (u32)(ndiv * TS2022_XTAL_FREQ / (6 + 8) / (div4 + 1) / 2);

	printk("%s: offset %dkhz\n", __func__, priv->frequency - c->frequency);
	printk("%s:  %dkhz  %dkhz\n", __func__, c->frequency, priv->frequency);

	return 0;
}

static int ts2022_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct ts2022_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency;
	return 0;
}

static int ts2022_init(struct dvb_frontend *fe)
{
	ts2022_writereg(fe, 0x62, 0xec);
	ts2022_writereg(fe, 0x42, 0x6c);
	
	ts2022_writereg(fe, 0x7d, 0x9d);
	ts2022_writereg(fe, 0x7c, 0x9a);
	ts2022_writereg(fe, 0x7a, 0x76);
	
	ts2022_writereg(fe, 0x3b, 0x01);
	ts2022_writereg(fe, 0x63, 0x88);
	
	ts2022_writereg(fe, 0x61, 0x85);
	ts2022_writereg(fe, 0x22, 0x30);
	ts2022_writereg(fe, 0x30, 0x40);
	ts2022_writereg(fe, 0x20, 0x23);
	ts2022_writereg(fe, 0x24, 0x02);
	ts2022_writereg(fe, 0x12, 0xa0);

	return 0;
}

static int ts2022_read_signal_strength(struct dvb_frontend *fe,
				       u16 *signal_strength)
{
	int sig_reading = 0; 
	u8 rfgain, bbgain, nngain;
	u8 rfagc;
	u32 gain = 0;
	dprintk("%s()\n", __func__);
	
	rfgain = ts2022_readreg(fe, 0x3d) & 0x1f;
	bbgain = ts2022_readreg(fe, 0x21) & 0x1f;
	rfagc = ts2022_readreg(fe, 0x3f);
	sig_reading = rfagc * 16 -670;
	if (sig_reading<0)
		sig_reading =0;
	nngain =ts2022_readreg(fe, 0x66);
	nngain = (nngain >> 3) & 0x07;
	
	if (rfgain < 0)
		rfgain = 0;
	if (rfgain > 15)
		rfgain = 15;
	if (bbgain < 2)
		bbgain = 2;
	if (bbgain > 16)
		bbgain = 16;
	if (nngain < 0)
		nngain = 0;
	if (nngain > 6)
		nngain = 6;
	
	if (sig_reading < 600)
		sig_reading = 600;
	if (sig_reading > 1600)
		sig_reading = 1600;
	
	gain = (u16) rfgain * 265 + (u16) bbgain * 338 + (u16) nngain * 285 + sig_reading * 176 / 100 - 3000;
	
	
	*signal_strength = gain*100;
	
	dprintk("%s: raw / cooked = 0x%04x / 0x%04x\n", __func__,
		sig_reading, *signal_strength);
	
	return 0;
}

static struct dvb_tuner_ops ts2022_tuner_ops = {
	.info = {
		.name = "TS2022",
		.frequency_min = 950000,
		.frequency_max = 2150000
	},
	.init = ts2022_init,
	.release = ts2022_release,
	.sleep = ts2022_sleep,
	.set_params = ts2022_set_params,
	.get_frequency = ts2022_get_frequency,
	.get_rf_strength = ts2022_read_signal_strength,
};

struct dvb_frontend *ts2022_attach(struct dvb_frontend *fe, int addr,
						struct i2c_adapter *i2c)
{
	struct ts2022_priv *priv = NULL;
	u8 buf;

	dprintk("%s:\n", __func__);

	priv = kzalloc(sizeof(struct ts2022_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->i2c_address = addr;
	priv->i2c = i2c;
	fe->tuner_priv = priv;

	/* Wake Up the tuner */
	buf = ts2022_readreg(fe, 0x00);
	buf &= 0x03;
	
	if (buf == 0x00) {
		ts2022_writereg(fe, 0x00, 0x01);
		msleep(2);
	}

	ts2022_writereg(fe, 0x00, 0x03);
	msleep(2);
	
	/* Check the tuner version */
	buf = ts2022_readreg(fe, 0x00);
	if ((buf == 0xc3)|| (buf == 0x83))
		dprintk(KERN_INFO "%s: Find tuner TS2022!\n", __func__);
	else {
		dprintk(KERN_ERR "%s: Read tuner reg[0] = %d\n", __func__, buf);
		kfree(priv);
		return NULL;
	}

	memcpy(&fe->ops.tuner_ops, &ts2022_tuner_ops,
				sizeof(struct dvb_tuner_ops));
	fe->ops.read_signal_strength = fe->ops.tuner_ops.get_rf_strength;

	return fe;
}
EXPORT_SYMBOL(ts2022_attach);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("DVB ts2022 driver");
MODULE_AUTHOR("Tomazzo Muzumici");
MODULE_LICENSE("GPL");
