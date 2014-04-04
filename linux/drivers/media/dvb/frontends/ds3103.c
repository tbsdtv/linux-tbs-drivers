/*
    Montage Technology DS3103 - DVBS/S2 Demodulator driver

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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/firmware.h>

#include "dvb_frontend.h"
#include "ds3103.h"

static int debug;

#define dprintk(args...) \
	do { \
		if (debug) \
			printk(args); \
	} while (0)

#define DS3103_DEFAULT_FIRMWARE "dvb-fe-ds3103.fw"

static u8 ds310x_dvbs_init_tab[] = {
	0x23, 0x07,
	0x08, 0x03,
	0x0c, 0x02,
	0x21, 0x54,
	0x25, 0x82,
	0x27, 0x31,
	0x30, 0x08,
	0x31, 0x40,
	0x32, 0x32,
	0x33, 0x35,
	0x35, 0xff,
	0x3a, 0x00,
	0x37, 0x10,
	0x38, 0x10,
	0x39, 0x02,
	0x42, 0x60,
	0x4a, 0x80,
	0x4b, 0x04,
	0x4d, 0x91,
	0x5d, 0xc8,
	0x50, 0x36,
	0x51, 0x36,
	0x52, 0x36,
	0x53, 0x36,
	0x63, 0x0f,
	0x64, 0x30,
	0x65, 0x40,
	0x68, 0x26,
	0x69, 0x4c,
	0x70, 0x20,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x40,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x60,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x80,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0xa0,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x1f,
	0x76, 0x38,
	0x77, 0xa6,
	0x78, 0x0c,
	0x79, 0x80,
	0x7f, 0x14,
	0x7c, 0x00,
	0xae, 0x82,
	0x80, 0x64,
	0x81, 0x66,
	0x82, 0x44,
	0x85, 0x04,
	0xcd, 0xf4,
	0x90, 0x33,
	0xa0, 0x44,
	0xc0, 0x08,
	0xc3, 0x10,
	0xc4, 0x08,
	0xc5, 0xf0,
	0xc6, 0xff,
	0xc7, 0x00,
	0xc8, 0x1a,
	0xc9, 0x80,
	0xe0, 0xf8,
	0xe6, 0x8b,
	0xd0, 0x40,
	0xf8, 0x20,
	0xfa, 0x0f,
	0x00, 0x00,
	0xbd, 0x01,
	0xb8, 0x00
};

static u8 ds310x_dvbs2_init_tab[] = {
	0x23, 0x07,
	0x08, 0x07,
	0x0c, 0x02,
	0x21, 0x54,
	0x25, 0x82,
	0x27, 0x31,
	0x30, 0x08,
	0x32, 0x32,
	0x33, 0x35,
	0x35, 0xff,
	0x3a, 0x00,
	0x37, 0x10,
	0x38, 0x10,
	0x39, 0x02,
	0x42, 0x60,
	0x4a, 0x80,
	0x4b, 0x04,
	0x4d, 0x91,
	0x5d, 0xc8,
	0x50, 0x36,
	0x51, 0x36,
	0x52, 0x36,
	0x53, 0x36,
	0x63, 0x0f,
	0x64, 0x10,
	0x65, 0x20,
	0x68, 0x46,
	0x69, 0xcd,
	0x70, 0x20,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x40,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x60,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x80,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0xa0,
	0x71, 0x70,
	0x72, 0x04,
	0x73, 0x00,
	0x70, 0x1f,
	0x76, 0x38,
	0x77, 0xa6,
	0x78, 0x0c,
	0x79, 0x80,
	0x7f, 0x14,
	0x85, 0x08,
	0xcd, 0xf4,
	0x90, 0x33,
	0x86, 0x00,
	0x87, 0x0f,
	0x89, 0x00,
	0x8b, 0x44,
	0x8c, 0x66,
	0x9d, 0xc1,
	0x8a, 0x10,
	0xad, 0x40,
	0xa0, 0x44,
	0xc0, 0x08,
	0xc1, 0x10,
	0xc2, 0x08,
	0xc3, 0x10,
	0xc4, 0x08,
	0xc5, 0xf0,
	0xc6, 0xff,
	0xc7, 0x00,
	0xc8, 0x1a,
	0xc9, 0x80,
	0xca, 0x23,
	0xcb, 0x24,
	0xcc, 0xf4,
	0xce, 0x74,
	0x00, 0x00,
	0xbd, 0x01,
	0xb8, 0x00
};

struct ds3103_state {
	struct i2c_adapter *i2c;
	const struct ds3103_config *config;
	struct dvb_frontend frontend;
	/* previous uncorrected block counter for DVB-S2 */
	u16 prevUCBS2;
};

static int ds3103_writereg(struct ds3103_state *state, int reg, int data)
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .addr = state->config->demod_address,
		.flags = 0, .buf = buf, .len = 2 };
	int err;

	dprintk("%s: write reg 0x%02x, value 0x%02x\n", __func__, reg, data);

	err = i2c_transfer(state->i2c, &msg, 1);
	if (err != 1) {
		printk(KERN_ERR "%s: writereg error(err == %i, reg == 0x%02x,"
			 " value == 0x%02x)\n", __func__, err, reg, data);
		return -EREMOTEIO;
	}

	return 0;
}

/* I2C write for 8k firmware load */
static int ds3103_writeFW(struct ds3103_state *state, int reg,
				const u8 *data, u16 len)
{
	int i, ret = -EREMOTEIO;
	struct i2c_msg msg;
	u8 *buf;

	buf = kmalloc(33, GFP_KERNEL);
	if (buf == NULL) {
		printk(KERN_ERR "Unable to kmalloc\n");
		ret = -ENOMEM;
		goto error;
	}

	*(buf) = reg;

	msg.addr = state->config->demod_address;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = 33;
   
	for (i = 0; i < len; i += 32) {
		memcpy(buf + 1, data + i, 32);

		dprintk("%s: write reg 0x%02x, len = %d\n", __func__, reg, len);

		ret = i2c_transfer(state->i2c, &msg, 1);
		if (ret != 1) {
			printk(KERN_ERR "%s: write error(err == %i, "
				"reg == 0x%02x\n", __func__, ret, reg);
			ret = -EREMOTEIO;
		}
	}

error:
	kfree(buf);

	return ret;
}

static int ds3103_readreg(struct ds3103_state *state, u8 reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = state->config->demod_address,
			.flags = 0,
			.buf = b0,
			.len = 1
		}, {
			.addr = state->config->demod_address,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};

	ret = i2c_transfer(state->i2c, msg, 2);

	if (ret != 2) {
		printk(KERN_ERR "%s: reg=0x%x(error=%d)\n", __func__, reg, ret);
		return ret;
	}

	dprintk("%s: read reg 0x%02x, value 0x%02x\n", __func__, reg, b1[0]);

	return b1[0];
}

static int ds3103_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ds3103_state *state = fe->demodulator_priv;
	
	if (enable)
		ds3103_writereg(state, 0x03, 0x12);
	else
		ds3103_writereg(state, 0x03, 0x02);
	
	return 0;
}
static int ds3103_load_firmware(struct dvb_frontend *fe,
					const struct firmware *fw);

static int ds3103_firmware_ondemand(struct dvb_frontend *fe)
{
	struct ds3103_state *state = fe->demodulator_priv;
	const struct firmware *fw;
	int ret = 0;

	dprintk("%s()\n", __func__);

	if (ds3103_readreg(state, 0xb2) <= 0)
		return ret;

	/* global reset, global diseqc reset, global fec reset */
	ds3103_writereg(state, 0x07, 0xe0);
	ds3103_writereg(state, 0x07, 0x00);

	/* request the firmware, this will block until someone uploads it */
	printk(KERN_INFO "%s: Waiting for firmware upload (%s)...\n", __func__,
				DS3103_DEFAULT_FIRMWARE);
	ret = request_firmware(&fw, DS3103_DEFAULT_FIRMWARE,
				state->i2c->dev.parent);
	printk(KERN_INFO "%s: Waiting for firmware upload(2)...\n", __func__);
	if (ret) {
		printk(KERN_ERR "%s: No firmware uploaded (timeout or file not "
				"found?)\n", __func__);
		return ret;
	}

	ret = ds3103_load_firmware(fe, fw);
	if (ret)
		printk("%s: Writing firmware to device failed\n", __func__);

	release_firmware(fw);

	dprintk("%s: Firmware upload %s\n", __func__,
			ret == 0 ? "complete" : "failed");

	return ret;
}

static int ds3103_load_firmware(struct dvb_frontend *fe,
					const struct firmware *fw)
{
	struct ds3103_state *state = fe->demodulator_priv;

	dprintk("%s\n", __func__);
	dprintk("Firmware is %zu bytes (%02x %02x .. %02x %02x)\n",
			fw->size,
			fw->data[0],
			fw->data[1],
			fw->data[fw->size - 2],
			fw->data[fw->size - 1]);

	/* Begin the firmware load process */
	ds3103_writereg(state, 0xb2, 0x01);
	/* write the entire firmware */
	ds3103_writeFW(state, 0xb0, fw->data, fw->size);
	ds3103_writereg(state, 0xb2, 0x00);

	return 0;
}

static int ds3103_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct ds3103_state *state = fe->demodulator_priv;
	u8 data;

	dprintk("%s(%d)\n", __func__, voltage);

	data = ds3103_readreg(state, 0xa2);
	data |= 0x03; /* bit0 V/H, bit1 off/on */

	switch (voltage) {
	case SEC_VOLTAGE_18:
		data &= ~0x03;
		break;
	case SEC_VOLTAGE_13:
		data &= ~0x03;
		data |= 0x01;
		break;
	case SEC_VOLTAGE_OFF:
		break;
	}

	ds3103_writereg(state, 0xa2, data);

	return 0;
}

static int ds3103_read_status(struct dvb_frontend *fe, fe_status_t* status)
{
	struct ds3103_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int lock;

	*status = 0;

	switch (c->delivery_system) {
	case SYS_DVBS:
		lock = ds3103_readreg(state, 0xd1);
		if ((lock & 0x07) == 0x07)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				FE_HAS_VITERBI | FE_HAS_SYNC |
				FE_HAS_LOCK;

		break;
	case SYS_DVBS2:
		lock = ds3103_readreg(state, 0x0d);
		if ((lock & 0x8f) == 0x8f)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				FE_HAS_VITERBI | FE_HAS_SYNC |
				FE_HAS_LOCK;

		break;
	default:
		return 1;
	}

	if (state->config->set_lock_led)
		state->config->set_lock_led(fe, *status == 0 ? 0 : 1);

	dprintk("%s: status = 0x%02x\n", __func__, lock);

	return 0;
}

/* read DS3103 BER value */
static int ds3103_read_ber(struct dvb_frontend *fe, u32* ber)
{
	struct ds3103_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 data;
	u32 ber_reading, lpdc_frames;

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		/* set the number of bytes checked during
		BER estimation */
		ds3103_writereg(state, 0xf9, 0x04);
		/* read BER estimation status */
		data = ds3103_readreg(state, 0xf8);
		/* check if BER estimation is ready */
		if ((data & 0x10) == 0) {
			/* this is the number of error bits,
			to calculate the bit error rate
			divide to 8388608 */
			*ber = (ds3103_readreg(state, 0xf7) << 8) |
				ds3103_readreg(state, 0xf6);
			/* start counting error bits */
			/* need to be set twice
			otherwise it fails sometimes */
			data |= 0x10;
			ds3103_writereg(state, 0xf8, data);
			ds3103_writereg(state, 0xf8, data);
		} else
			/* used to indicate that BER estimation
			is not ready, i.e. BER is unknown */
			*ber = 0xffffffff;
		break;
	case SYS_DVBS2:
		/* read the number of LPDC decoded frames */
		lpdc_frames = (ds3103_readreg(state, 0xd7) << 16) |
				(ds3103_readreg(state, 0xd6) << 8) |
				ds3103_readreg(state, 0xd5);
		/* read the number of packets with bad CRC */
		ber_reading = (ds3103_readreg(state, 0xf8) << 8) |
				ds3103_readreg(state, 0xf7);
		if (lpdc_frames > 750) {
			/* clear LPDC frame counters */
			ds3103_writereg(state, 0xd1, 0x01);
			/* clear bad packets counter */
			ds3103_writereg(state, 0xf9, 0x01);
			/* enable bad packets counter */
			ds3103_writereg(state, 0xf9, 0x00);
			/* enable LPDC frame counters */
			ds3103_writereg(state, 0xd1, 0x00);
			*ber = ber_reading;
		} else
			/* used to indicate that BER estimation is not ready,
			i.e. BER is unknown */
			*ber = 0xffffffff;
		break;
	default:
		return 1;
	}

	return 0;
}

/* calculate DS3103 snr value in dB */
static int ds3103_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct ds3103_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 snr_reading, snr_value;
	u32 dvbs2_signal_reading, dvbs2_noise_reading, tmp;
	static const u16 dvbs_snr_tab[] = { /* 20 x Table (rounded up) */
		0x0000, 0x1b13, 0x2aea, 0x3627, 0x3ede, 0x45fe, 0x4c03,
		0x513a, 0x55d4, 0x59f2, 0x5dab, 0x6111, 0x6431, 0x6717,
		0x69c9, 0x6c4e, 0x6eac, 0x70e8, 0x7304, 0x7505
	};
	static const u16 dvbs2_snr_tab[] = { /* 80 x Table (rounded up) */
		0x0000, 0x0bc2, 0x12a3, 0x1785, 0x1b4e, 0x1e65, 0x2103,
		0x2347, 0x2546, 0x2710, 0x28ae, 0x2a28, 0x2b83, 0x2cc5,
		0x2df1, 0x2f09, 0x3010, 0x3109, 0x31f4, 0x32d2, 0x33a6,
		0x3470, 0x3531, 0x35ea, 0x369b, 0x3746, 0x37ea, 0x3888,
		0x3920, 0x39b3, 0x3a42, 0x3acc, 0x3b51, 0x3bd3, 0x3c51,
		0x3ccb, 0x3d42, 0x3db6, 0x3e27, 0x3e95, 0x3f00, 0x3f68,
		0x3fcf, 0x4033, 0x4094, 0x40f4, 0x4151, 0x41ac, 0x4206,
		0x425e, 0x42b4, 0x4308, 0x435b, 0x43ac, 0x43fc, 0x444a,
		0x4497, 0x44e2, 0x452d, 0x4576, 0x45bd, 0x4604, 0x4649,
		0x468e, 0x46d1, 0x4713, 0x4755, 0x4795, 0x47d4, 0x4813,
		0x4851, 0x488d, 0x48c9, 0x4904, 0x493f, 0x4978, 0x49b1,
		0x49e9, 0x4a20, 0x4a57
	};

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		snr_reading = ds3103_readreg(state, 0xff);
		snr_reading /= 8;
		if (snr_reading == 0)
			*snr = 0x0000;
		else {
			if (snr_reading > 20)
				snr_reading = 20;
			snr_value = dvbs_snr_tab[snr_reading - 1] * 10 / 23026;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = snr_value * 8 * 655;
		}
		dprintk("%s: raw / cooked = 0x%02x / 0x%04x\n", __func__,
				snr_reading, *snr);
		break;
	case SYS_DVBS2:
		dvbs2_noise_reading = (ds3103_readreg(state, 0x8c) & 0x3f) +
				(ds3103_readreg(state, 0x8d) << 4);
		dvbs2_signal_reading = ds3103_readreg(state, 0x8e);
		tmp = dvbs2_signal_reading * dvbs2_signal_reading >> 1;
		if (tmp == 0) {
			*snr = 0x0000;
			return 0;
		}
		if (dvbs2_noise_reading == 0) {
			snr_value = 0x0013;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = 0xffff;
			return 0;
		}
		if (tmp > dvbs2_noise_reading) {
			snr_reading = tmp / dvbs2_noise_reading;
			if (snr_reading > 80)
				snr_reading = 80;
			snr_value = dvbs2_snr_tab[snr_reading - 1] / 1000;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = snr_value * 5 * 655;
		} else {
			snr_reading = dvbs2_noise_reading / tmp;
			if (snr_reading > 80)
				snr_reading = 80;
			*snr = -(dvbs2_snr_tab[snr_reading] / 1000);
		}
		dprintk("%s: raw / cooked = 0x%02x / 0x%04x\n", __func__,
				snr_reading, *snr);
		break;
	default:
		return 1;
	}

	return 0;
}

/* read DS3103 uncorrected blocks */
static int ds3103_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct ds3103_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 data;
	u16 _ucblocks;

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		*ucblocks = (ds3103_readreg(state, 0xf5) << 8) |
				ds3103_readreg(state, 0xf4);
		data = ds3103_readreg(state, 0xf8);
		/* clear packet counters */
		data &= ~0x20;
		ds3103_writereg(state, 0xf8, data);
		/* enable packet counters */
		data |= 0x20;
		ds3103_writereg(state, 0xf8, data);
		break;
	case SYS_DVBS2:
		_ucblocks = (ds3103_readreg(state, 0xe2) << 8) |
				ds3103_readreg(state, 0xe1);
		if (_ucblocks > state->prevUCBS2)
			*ucblocks = _ucblocks - state->prevUCBS2;
		else
			*ucblocks = state->prevUCBS2 - _ucblocks;
		state->prevUCBS2 = _ucblocks;
		break;
	default:
		return 1;
	}

	return 0;
}

static int ds3103_set_tone(struct dvb_frontend *fe, fe_sec_tone_mode_t tone)
{
	struct ds3103_state *state = fe->demodulator_priv;
	u8 data;

	dprintk("%s(%d)\n", __func__, tone);
	if ((tone != SEC_TONE_ON) && (tone != SEC_TONE_OFF)) {
		printk(KERN_ERR "%s: Invalid, tone=%d\n", __func__, tone);
		return -EINVAL;
	}

	data = ds3103_readreg(state, 0xa2);
	data &= ~0xc0;
	ds3103_writereg(state, 0xa2, data);

	switch (tone) {
	case SEC_TONE_ON:
		dprintk("%s: setting tone on\n", __func__);
		data = ds3103_readreg(state, 0xa1);
		data &= ~0x43;
		data |= 0x04;
		ds3103_writereg(state, 0xa1, data);
		break;
	case SEC_TONE_OFF:
		dprintk("%s: setting tone off\n", __func__);
		data = ds3103_readreg(state, 0xa2);
		data |= 0x80;
		ds3103_writereg(state, 0xa2, data);
		break;
	}

	return 0;
}

static int ds3103_send_diseqc_msg(struct dvb_frontend *fe,
				struct dvb_diseqc_master_cmd *d)
{
	struct ds3103_state *state = fe->demodulator_priv;
	int i;
	u8 data;

	/* Dump DiSEqC message */
	dprintk("%s(", __func__);
	for (i = 0 ; i < d->msg_len;) {
		dprintk("0x%02x", d->msg[i]);
		if (++i < d->msg_len)
			dprintk(", ");
	}

	/* enable DiSEqC message send pin */
	data = ds3103_readreg(state, 0xa2);
	data &= ~0xc0;
	data &= ~0x20;
	ds3103_writereg(state, 0xa2, data);

	/* DiSEqC message */
	for (i = 0; i < d->msg_len; i++)
		ds3103_writereg(state, 0xa3 + i, d->msg[i]);

	data = ds3103_readreg(state, 0xa1);
	/* clear DiSEqC message length and status,
	enable DiSEqC message send */
	data &= ~0xf8;
	/* set DiSEqC mode, modulation active during 33 pulses,
	set DiSEqC message length */
	data |= ((d->msg_len - 1) << 3) | 0x07;
	ds3103_writereg(state, 0xa1, data);

	/* wait up to 150ms for DiSEqC transmission to complete */
	for (i = 0; i < 15; i++) {
		data = ds3103_readreg(state, 0xa1);
		if ((data & 0x40) == 0)
			break;
		msleep(10);
	}

	/* DiSEqC timeout after 150ms */
	if (i == 15) {
		data = ds3103_readreg(state, 0xa1);
		data &= ~0x80;
		data |= 0x40;
		ds3103_writereg(state, 0xa1, data);

		data = ds3103_readreg(state, 0xa2);
		data &= ~0xc0;
		data |= 0x80;
		ds3103_writereg(state, 0xa2, data);

		return 1;
	}

	data = ds3103_readreg(state, 0xa2);
	data &= ~0xc0;
	data |= 0x80;
	ds3103_writereg(state, 0xa2, data);

	return 0;
}

/* Send DiSEqC burst */
static int ds3103_diseqc_send_burst(struct dvb_frontend *fe,
					fe_sec_mini_cmd_t burst)
{
	struct ds3103_state *state = fe->demodulator_priv;
	int i;
	u8 data;

	dprintk("%s()\n", __func__);

	data = ds3103_readreg(state, 0xa2);
	data &= ~0xc0;
	data &= ~0x20;
	ds3103_writereg(state, 0xa2, data);

	/* DiSEqC burst */
	if (burst == SEC_MINI_A)
		/* Unmodulated tone burst */
		ds3103_writereg(state, 0xa1, 0x02);
	else if (burst == SEC_MINI_B)
		/* Modulated tone burst */
		ds3103_writereg(state, 0xa1, 0x01);
	else
		return -EINVAL;

	msleep(13);
	for (i = 0; i < 5; i++) {
		data = ds3103_readreg(state, 0xa1);
		if ((data & 0x40) == 0)
			break;
		msleep(1);
	}

	if (i == 5) {
		data = ds3103_readreg(state, 0xa1);
		data &= ~0x80;
		data |= 0x40;
		ds3103_writereg(state, 0xa1, data);

		data = ds3103_readreg(state, 0xa2);
		data &= ~0xc0;
		data |= 0x80;
		ds3103_writereg(state, 0xa2, data);

		return 1;
	}

	data = ds3103_readreg(state, 0xa2);
	data &= ~0xc0;
	data |= 0x80;
	ds3103_writereg(state, 0xa2, data);

	return 0;
}

static void ds3103_release(struct dvb_frontend *fe)
{
	struct ds3103_state *state = fe->demodulator_priv;

	if (state->config->set_lock_led)
		state->config->set_lock_led(fe, 0);

	dprintk("%s\n", __func__);
	kfree(state);
}

static struct dvb_frontend_ops ds3103_ops;

struct dvb_frontend *ds3103_attach(const struct ds3103_config *config,
				    struct i2c_adapter *i2c)
{
	struct ds3103_state *state = NULL;
	int ret;
	u8 val_01, val_02, val_b2;


	dprintk("%s\n", __func__);

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct ds3103_state), GFP_KERNEL);
	if (state == NULL) {
		printk(KERN_ERR "Unable to kmalloc\n");
		goto error2;
	}

	state->config = config;
	state->i2c = i2c;
	state->prevUCBS2 = 0;

	/* check if the demod is present */
	ret = ds3103_readreg(state, 0x00) & 0xfe;
	if (ret != 0xe0) {
		printk(KERN_ERR "Invalid probe, probably not a DS3x0x\n");
		goto error3;
	}

	/* check demod chip ID */
	val_01 = ds3103_readreg(state, 0x01);
	val_02 = ds3103_readreg(state, 0x02);
	val_b2 = ds3103_readreg(state, 0xb2);
	if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0xC0)) {
		printk("\tChip ID = [DS3103]!\n");
	} else if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0x00)) {
		printk("\tChip ID = [DS3002B]!\n");
	} else if ((val_02 == 0x00) && (val_01 == 0xC0)) {
		printk("\tChip ID = [DS300X]! Not supported by this module\n");
		goto error3;
	} else {
		printk("\tChip ID = unknow!\n");
		goto error3;
	}

	printk(KERN_INFO "DS3103 chip version: %d.%d attached.\n", val_02, val_01);

	memcpy(&state->frontend.ops, &ds3103_ops,
			sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;
	return &state->frontend;

error3:
	kfree(state);
error2:
	return NULL;
}
EXPORT_SYMBOL(ds3103_attach);

static int ds3103_set_carrier_offset(struct dvb_frontend *fe,
					s32 carrier_offset_khz,
					u32 target_mclk)
{
	struct ds3103_state *state = fe->demodulator_priv;
	s32 tmp;

	tmp = carrier_offset_khz;
	tmp *= 65536;
	tmp = (2 * tmp + target_mclk) / (2 * target_mclk);

	if (tmp < 0)
		tmp += 65536;

	ds3103_writereg(state, 0x5f, tmp >> 8);
	ds3103_writereg(state, 0x5e, tmp & 0xff);

	return 0;
}
static int ds3103_set_ts_div(struct ds3103_state *state, u8 tmp1, u8 tmp2)
{
	u8 buf;
	tmp1 -= 1;
	tmp2 -= 1;

	tmp1 &= 0x3f;
	tmp2 &= 0x3f;

	buf = ds3103_readreg(state, 0xfe);
	buf &= 0xf0;
	buf |= (tmp1 >> 2) & 0x0f;
	ds3103_writereg(state, 0xfe, buf);

	buf = (u8)((tmp1 & 0x03) << 6);
	buf |= tmp2;
	ds3103_writereg(state, 0xea, buf);

	return 0;
}

u32 ds3103_get_mclk(struct ds3103_state *state)
{
	u32 p_mclk_khz = 96000;
	u8 tmp1, tmp2;

	tmp1 = ds3103_readreg(state, 0x22);
	tmp2 = ds3103_readreg(state, 0x24);

	tmp1 >>= 6;
	tmp1 &= 0x03;
	tmp2 >>= 6;
	tmp2 &= 0x03;

	if((tmp1 == 0x00) && (tmp2 == 0x01))
		p_mclk_khz = 144000;
	else if ((tmp1 == 0x00) && (tmp2 == 0x03))
		p_mclk_khz = 72000;
	else if ((tmp1 == 0x01) && (tmp2 == 0x01))
		p_mclk_khz = 115200;
	else if ((tmp1 == 0x02) && (tmp2 == 0x01))
		p_mclk_khz = 96000;
	else if ((tmp1 == 0x03) && (tmp2 == 0x00))
		p_mclk_khz = 192000;

	return p_mclk_khz;
}

static int ds3103_set_clock_ratio(struct ds3103_state *state, u32 target_mclk)
{
	struct dvb_frontend *fe = &state->frontend;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u8	mod_fac, tmp1, tmp2;
	u32	input_datarate;
	u32 mclk_khz = target_mclk;
	u16 divid_ratio = 0;
	u32 sym_rate_kss = p->symbol_rate / 1000;

	if (p->delivery_system == SYS_DVBS2) {
		switch(p->modulation) {
		case PSK_8:
			mod_fac = 3;
			break;
		case APSK_16:
			mod_fac = 4;
			break;
		case APSK_32:
			mod_fac = 5;
			break;
		case QPSK:
		default:
			mod_fac = 2;
			break;
		}

		switch(p->fec_inner) {
/*		case FEC_1_4:
 *			input_datarate = sym_rate_kss * mod_fac / 8 / 4;
 *			break;
 *		case FEC_1_3:
 *			input_datarate = sym_rate_kss * mod_fac / 8 / 3;
 *			break;
 *		case FEC_2_5:
 *			input_datarate = sym_rate_kss * mod_fac * 2 / 8 / 5;
 *			break;
 */
		case FEC_1_2:
			input_datarate = sym_rate_kss * mod_fac / 8 / 2;
			break;
		case FEC_3_5:
			input_datarate = sym_rate_kss * mod_fac * 3 / 8 / 5;
			break;
		case FEC_2_3:
			input_datarate = sym_rate_kss * mod_fac * 2 / 8 / 3;
			break;
		case FEC_3_4:
			input_datarate = sym_rate_kss * mod_fac * 3 / 8 / 4;
			break;
		case FEC_4_5:
			input_datarate = sym_rate_kss * mod_fac * 4 / 8 / 5;
			break;
		case FEC_5_6:
			input_datarate = sym_rate_kss * mod_fac * 5 / 8 / 6;
			break;
		case FEC_8_9:
			input_datarate = sym_rate_kss * mod_fac * 8 / 8 / 9;
			break;
		case FEC_9_10:
			input_datarate = sym_rate_kss * mod_fac * 9 / 8 / 10;
			break;
		default:
			input_datarate = sym_rate_kss * mod_fac * 2 / 8 / 3;
			break;
		}

		/* parallel or CI mode for now */
		if (input_datarate != 0)
			divid_ratio = mclk_khz / input_datarate;
		else
			divid_ratio = 0xff;

		if (divid_ratio > 128)
			divid_ratio = 128;
		
		if (divid_ratio < 2)
			divid_ratio = 2;
		
		tmp1 = divid_ratio / 2;
		tmp2 = divid_ratio / 2;
		
		if ((divid_ratio % 2) != 0)
			tmp2 += 1;

	} else { /* for dvb-s */
		mod_fac = 2;

		switch (p->fec_inner) {
		case FEC_1_2:
			input_datarate = sym_rate_kss * mod_fac / 2 / 8;
			break;
		case FEC_2_3:
			input_datarate = sym_rate_kss * mod_fac * 2 / 3 / 8;
			break;
		case FEC_3_4:
			input_datarate = sym_rate_kss * mod_fac * 3 / 4 / 8;
			break;
		case FEC_5_6:
			input_datarate = sym_rate_kss * mod_fac * 5 / 6 / 8;
			break;
		case FEC_7_8:
			input_datarate = sym_rate_kss * mod_fac * 7 / 8 / 8;
			break;
		default:
			input_datarate = sym_rate_kss * mod_fac * 3 / 4 / 8;
			break;
		}

		if (input_datarate != 0)
			divid_ratio = mclk_khz / input_datarate;
		else
			divid_ratio = 0xff;

		if (divid_ratio > 128)
			divid_ratio = 128;
		
		if (divid_ratio < 2)
			divid_ratio = 2;

		tmp1 = divid_ratio / 2;
		tmp2 = divid_ratio / 2;

		if ((divid_ratio % 2) != 0)
			tmp2 += 1;

	}

	ds3103_set_ts_div(state, tmp1, tmp2);

	return 0;
}

static int ds3103_set_frontend(struct dvb_frontend *fe,
	struct dvb_frontend_parameters *params)
{
	struct ds3103_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	int i;
	fe_status_t status;
	s32 offset_khz;
	u32 tuner_freq;
	u16 value;
	u32 tmp;
	u8 tmp1, tmp2;
	u32 target_mclk = 0;

	dprintk("%s() frec=%d symb=%d", __func__, c->frequency, c->symbol_rate);

	if (state->config->set_ts_params)
		state->config->set_ts_params(fe, 0);

	if (fe->ops.tuner_ops.set_params)
		fe->ops.tuner_ops.set_params(fe, params);

	ds3103_writereg(state, 0xb2, 0x01);
	ds3103_writereg(state, 0x00, 0x01);

	if (fe->ops.tuner_ops.get_frequency)
		fe->ops.tuner_ops.get_frequency(fe, &tuner_freq);

	offset_khz = tuner_freq - c->frequency;

	value = ds3103_readreg(state, 0x08);

	switch (c->delivery_system) {
	case SYS_DVBS2:
		value |= 0x04;
		ds3103_writereg(state, 0x08, value);
		for (i = 0; i < sizeof(ds310x_dvbs2_init_tab); i += 2)
			ds3103_writereg(state,
				ds310x_dvbs2_init_tab[i],
				ds310x_dvbs2_init_tab[i + 1]);

		value = ds3103_readreg(state, 0x4d);
		value &= ~0x02;
		ds3103_writereg(state, 0x4d, value);
		value = ds3103_readreg(state, 0x30);
		value &= ~0x10;
		ds3103_writereg(state, 0x30, value);
		if (c->symbol_rate > 28000000) {
			target_mclk = 192000;
		} else if (c->symbol_rate > 18000000) {
			target_mclk = 144000;
		} else
			target_mclk = 96000;

		if (c->symbol_rate <= 5000000) {
			ds3103_writereg(state, 0xc0, 0x04);
			ds3103_writereg(state, 0x8a, 0x09);
			ds3103_writereg(state, 0x8b, 0x22);
			ds3103_writereg(state, 0x8c, 0x88);
		}

		break;
	case SYS_DVBS:
	default:
		value &= ~0x04;
		ds3103_writereg(state, 0x08, value);
		for (i = 0; i < sizeof(ds310x_dvbs_init_tab); i += 2)
			ds3103_writereg(state,
					ds310x_dvbs_init_tab[i],
					ds310x_dvbs_init_tab[i + 1]);
		
		target_mclk = 96000;
		
		value = ds3103_readreg(state, 0x4d);
		value &= ~0x02;
		ds3103_writereg(state, 0x4d, value);
		value = ds3103_readreg(state, 0x30);
		value &= ~0x10;
		ds3103_writereg(state, 0x30, value);
		
		break;
	}

	ds3103_set_clock_ratio(state, target_mclk);

	tmp1 = ds3103_readreg(state, 0x22);
	tmp2 = ds3103_readreg(state, 0x24);

	switch (target_mclk) {
	case 192000:
		tmp1 |= 0xc0;
		tmp2 &= 0x3f;
		break;

	case 144000:
		tmp1 &= 0x3f;
		tmp2 &= 0x7f;
		tmp2 |= 0x40;
		break;

	case 96000:
	default:
		tmp1 &= 0xbf;
		tmp1 |= 0x80;

		tmp2 &= 0x7f;
		tmp2 |= 0x40;
		break;
	}

	ds3103_writereg(state, 0x22, tmp1);
	ds3103_writereg(state, 0x24, tmp2);

	ds3103_writereg(state, 0x33, 0x99);

	/* enable 27MHz clock output */
	value = ds3103_readreg(state, 0x29);
	value &= 0x80;
	value &= ~0x10;
	ds3103_writereg(state, 0x29, value);

	/* enable ac coupling */
	value = ds3103_readreg(state, 0x25);
	value |= 0x08;
	ds3103_writereg(state, 0x25, value);


	/* enhance symbol rate performance */
	if ((c->symbol_rate / 1000) <= 3000) {
		ds3103_writereg(state, 0xc3, 0x08);
		ds3103_writereg(state, 0xc8, 0x20);
		ds3103_writereg(state, 0xc4, 0x08);
		ds3103_writereg(state, 0xc7, 0x00);
	} else if((c->symbol_rate / 1000) <= 10000) {
		ds3103_writereg(state, 0xc3, 0x08);
		ds3103_writereg(state, 0xc8, 0x10);
		ds3103_writereg(state, 0xc4, 0x08);
		ds3103_writereg(state, 0xc7, 0x00);
	} else {
		ds3103_writereg(state, 0xc3, 0x08);
		ds3103_writereg(state, 0xc8, 0x06);
		ds3103_writereg(state, 0xc4, 0x08);
		ds3103_writereg(state, 0xc7, 0x00);
	}

	/* normalized symbol rate rounded to the closest integer */
	tmp = (((c->symbol_rate / 1000) << 15) + 24000) / 48000;

	ds3103_writereg(state, 0x61, tmp & 0x00ff);
	ds3103_writereg(state, 0x62, (tmp & 0xff00) >> 8);

	/* co-channel interference cancellation disabled */
	value = ds3103_readreg(state, 0x56);
	value &= ~0x01;
	ds3103_writereg(state, 0x56, value);
	/* equalizer disabled */
	value = ds3103_readreg(state, 0x76);
	value &= ~0x80;
	ds3103_writereg(state, 0x76, value);
	/* offset */
	if ((c->symbol_rate / 1000) < 5000)
		offset_khz += 3000;
	ds3103_set_carrier_offset(fe, offset_khz, target_mclk);

	/* ds3000 out of software reset */
	ds3103_writereg(state, 0x00, 0x00);
	/* start ds3000 build-in uC */
	ds3103_writereg(state, 0xb2, 0x00);


	for (i = 0; i < 30 ; i++) {
		ds3103_read_status(fe, &status);
		if (status && FE_HAS_LOCK)
			break;

		msleep(10);
	}

	return 0;
}

static int ds3103_tune(struct dvb_frontend *fe,
			struct dvb_frontend_parameters *p,
			unsigned int mode_flags,
			unsigned int *delay,
			fe_status_t *status)
{
	if (p) {
		int ret = ds3103_set_frontend(fe,p);
		if (ret)
			return ret;
	}

	*delay = HZ / 5;

	return ds3103_read_status(fe, status);
}

static enum dvbfe_algo ds3103_get_algo(struct dvb_frontend *fe)
{
	dprintk("%s()\n", __func__);
	return DVBFE_ALGO_HW;
}

/*
 * Initialize or wake up device
 *
 * Power config will reset and load initial firmware if required
 */
static int ds3103_initfe(struct dvb_frontend *fe)
{
	struct ds3103_state *state = fe->demodulator_priv;
	int ret;
	u8 buf;
	u8 val_08;

	dprintk("%s()\n", __func__);
	/* hard reset */
	buf = ds3103_readreg(state, 0xb2);
	if (buf == 0x01) {
		ds3103_writereg(state, 0x00, 0x00);
		ds3103_writereg(state, 0xb2, 0x00);
	}


	/* global reset */
	ds3103_writereg(state, 0x07, 0xe0);
	ds3103_writereg(state, 0x07, 0x00);
	ds3103_writereg(state, 0x08, 0x01 | ds3103_readreg(state, 0x08));
	msleep(1);

	/* Load the firmware if required */
	ret = ds3103_firmware_ondemand(fe);
	if (ret != 0) {
		printk(KERN_ERR "%s: Unable initialize firmware\n", __func__);
		return ret;
	}
	/* ts out mode */
	val_08 = ds3103_readreg(state, 0x08);
	buf = ds3103_readreg(state, 0x27);
	buf &= ~0x01;
	ds3103_writereg(state, 0x27, buf);
	/* for dvb-s */
	buf = val_08 & (~0x04) ;
	ds3103_writereg(state, 0x08, buf);
	ds3103_set_ts_div(state, 6, 6);

	/* for dvb-s2 */
	buf = val_08 | 0x04 ;
	ds3103_writereg(state, 0x08, buf);
	ds3103_set_ts_div(state, 8, 9);
	buf = ds3103_readreg(state, 0xfd);
	buf |= 0x01;
	buf &= ~0x04;

	buf &= ~0xba;
	if (state->config->ci_mode)
		buf |= 0x40;
	else
		buf &= ~0x40;

	ds3103_writereg(state, 0xfd, buf);
	ds3103_writereg(state, 0x08, val_08);
	buf = ds3103_readreg(state, 0x27);
	buf |= 0x11;
	ds3103_writereg(state, 0x27, buf);
	buf = ds3103_readreg(state, 0x4d);
	buf &= ~0x02;
	ds3103_writereg(state, 0x4d, buf);
	buf = ds3103_readreg(state, 0x30);
	buf &= ~0x10;
	ds3103_writereg(state, 0x30, buf);

	return 0;
}

/* Put device to sleep */
static int ds3103_sleep(struct dvb_frontend *fe)
{
	struct ds3103_state *state = fe->demodulator_priv;

	if (state->config->set_lock_led)
		state->config->set_lock_led(fe, 0);

	dprintk("%s()\n", __func__);
	return 0;
}

static struct dvb_frontend_ops ds3103_ops = {
	.info = {
		.type = FE_QPSK,
		.name = "Montage Technology DS3103/TS2022",
		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_stepsize = 1011, /* kHz for QPSK frontends */
		.frequency_tolerance = 5000,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 | FE_CAN_FEC_5_6 | FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_2G_MODULATION |
			FE_CAN_QPSK | FE_CAN_RECOVER
	},

	.release = ds3103_release,

	.init = ds3103_initfe,
	.sleep = ds3103_sleep,
	.read_status = ds3103_read_status,
	.read_ber = ds3103_read_ber,
	.i2c_gate_ctrl = ds3103_i2c_gate_ctrl,
	.read_snr = ds3103_read_snr,
	.read_ucblocks = ds3103_read_ucblocks,
	.set_voltage = ds3103_set_voltage,
	.set_tone = ds3103_set_tone,
	.diseqc_send_master_cmd = ds3103_send_diseqc_msg,
	.diseqc_send_burst = ds3103_diseqc_send_burst,
	.get_frontend_algo = ds3103_get_algo,

	.set_frontend = ds3103_set_frontend,
	.tune = ds3103_tune,
};

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates frontend debugging (default:0)");

MODULE_DESCRIPTION("DVB Frontend module for Montage Technology "
			"DS3103 hardware");
MODULE_AUTHOR("Tomazzo Muzumici");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(DS3103_DEFAULT_FIRMWARE);
