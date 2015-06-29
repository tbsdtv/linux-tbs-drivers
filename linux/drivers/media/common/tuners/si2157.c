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

#include "si2157_priv.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debugging (default:off).");

#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "si2157: " args); \
	} while (0)

#define dprintk_err(args...) \
	do { \
		printk(KERN_ERR args); \
	} while (0)

#define dprintk_info(args...) \
	do { \
		printk(KERN_INFO args); \
	} while (0)

#define dprintk_notice(args...) \
	do { \
		printk(KERN_NOTICE args); \
	} while (0)

static const struct dvb_tuner_ops si2157_ops;

static int si2157_i2c_master_send(struct si2157_state *state,
					   const char *buf, int count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = state->i2c_addr,
		.flags = 0,
		.len = count,
		.buf = (char *)buf,
	};

	ret = i2c_transfer(state->i2c, &msg, 1);
	return (ret == 1) ? count : ret;
}

static int si2157_i2c_master_recv(struct si2157_state *state,
					   char *buf, int count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = state->i2c_addr,
		.flags = I2C_M_RD,
		.len = count,
		.buf = buf,
	};

	ret = i2c_transfer(state->i2c, &msg, 1);
	return (ret == 1) ? count : ret;
}

/* execute firmware command */
static int si2157_cmd_execute(struct si2157_state *state, struct si2157_cmd *cmd)
{
	int ret;
	unsigned long timeout;

	if (state->fe->ops.i2c_gate_ctrl)
		state->fe->ops.i2c_gate_ctrl(state->fe, 1); /* open I2C-gate */

	if (cmd->wlen) {
		/* write cmd and args for firmware */
		ret = si2157_i2c_master_send(state, cmd->args, cmd->wlen);
		if (ret < 0) {
			goto err;
		} else if (ret != cmd->wlen) {
			ret = -EREMOTEIO;
			goto err;
		}
	}

	if (cmd->rlen) {
		/* wait cmd execution terminate */
		#define TIMEOUT 80
		timeout = jiffies + msecs_to_jiffies(TIMEOUT);
		while (!time_after(jiffies, timeout)) {
			ret = si2157_i2c_master_recv(state, cmd->args, cmd->rlen);
			if (ret < 0) {
				goto err;
			} else if (ret != cmd->rlen) {
				ret = -EREMOTEIO;
				goto err;
			}

			/* firmware ready? */
			if ((cmd->args[0] >> 7) & 0x01)
				break;
		}

		dprintk("%s: cmd execution took %d ms\n", __func__,
				jiffies_to_msecs(jiffies) -
				(jiffies_to_msecs(timeout) - TIMEOUT));

		if (!((cmd->args[0] >> 7) & 0x01)) {
			ret = -ETIMEDOUT;
			goto err;
		}
	}

	if (state->fe->ops.i2c_gate_ctrl)
		state->fe->ops.i2c_gate_ctrl(state->fe, 0); /* close I2C-gate */
	return 0;

err:
	if (state->fe->ops.i2c_gate_ctrl)
		state->fe->ops.i2c_gate_ctrl(state->fe, 0); /* close I2C-gate */

	dprintk_err("%s: failed=%d\n", __func__, ret);
	return ret;
}

static int si2157_init(struct dvb_frontend *fe)
{
	struct si2157_state *state = fe->tuner_priv;
	int ret, len, remaining;
	struct si2157_cmd cmd;
	const struct firmware *fw;
	const char *fw_name;
	unsigned int chip_id;

	if (state->fw_loaded)
		goto warm;

	/* power up */
	if (state->chiptype == SI2157_CHIPTYPE_SI2146) {
		memcpy(cmd.args, "\xc0\x05\x01\x00\x00\x0b\x00\x00\x01", 9);
		cmd.wlen = 9;
	} else {
		memcpy(cmd.args, "\xc0\x00\x0c\x00\x00\x01\x01\x01\x01\x01\x01\x02\x00\x00\x01", 15);
		cmd.wlen = 15;
	}
	cmd.rlen = 1;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	/* query chip revision */
	memcpy(cmd.args, "\x02", 1);
	cmd.wlen = 1;
	cmd.rlen = 13;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	chip_id = cmd.args[1] << 24 | cmd.args[2] << 16 | cmd.args[3] << 8 |
			cmd.args[4] << 0;

	#define SI2158_A20 ('A' << 24 | 58 << 16 | '2' << 8 | '0' << 0)
	#define SI2148_A20 ('A' << 24 | 48 << 16 | '2' << 8 | '0' << 0)
	#define SI2157_A30 ('A' << 24 | 57 << 16 | '3' << 8 | '0' << 0)
	#define SI2147_A30 ('A' << 24 | 47 << 16 | '3' << 8 | '0' << 0)
	#define SI2146_A10 ('A' << 24 | 46 << 16 | '1' << 8 | '0' << 0)

	switch (chip_id) {
	case SI2158_A20:
	case SI2148_A20:
		fw_name = SI2158_A20_FIRMWARE;
		break;
	case SI2157_A30:
	case SI2147_A30:
	case SI2146_A10:
		fw_name = NULL;
		break;
	default:
		dprintk_err("unknown chip version Si21%d-%c%c%c\n",
				cmd.args[2], cmd.args[1],
				cmd.args[3], cmd.args[4]);
		ret = -EINVAL;
		goto err;
	}

	dprintk_info("found a 'Silicon Labs Si21%d-%c%c%c'\n",
			cmd.args[2], cmd.args[1], cmd.args[3], cmd.args[4]);

	if (fw_name == NULL)
		goto skip_fw_download;

	/* request the firmware, this will block and timeout */
	ret = request_firmware(&fw, fw_name, state->i2c->dev.parent);
	if (ret) {
		dprintk_err("firmware file '%s' not found\n",
				fw_name);
		goto err;
	}

	/* firmware should be n chunks of 17 bytes */
	if (fw->size % 17 != 0) {
		dprintk_err("firmware file '%s' is invalid\n",
				fw_name);
		ret = -EINVAL;
		goto err_release_firmware;
	}

	dprintk_info("downloading firmware from file '%s'\n",
			fw_name);

	for (remaining = fw->size; remaining > 0; remaining -= 17) {
		len = fw->data[fw->size - remaining];
		memcpy(cmd.args, &fw->data[(fw->size - remaining) + 1], len);
		cmd.wlen = len;
		cmd.rlen = 1;
		ret = si2157_cmd_execute(state, &cmd);
		if (ret) {
			dprintk_err("firmware download failed %d\n",
					ret);
			goto err_release_firmware;
		}
	}

	release_firmware(fw);

skip_fw_download:
	/* reboot the tuner with new firmware? */
	memcpy(cmd.args, "\x01\x01", 2);
	cmd.wlen = 2;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	/* query firmware version */
	memcpy(cmd.args, "\x11", 1);
	cmd.wlen = 1;
	cmd.rlen = 10;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	dprintk_info("firmware version: %c.%c.%d\n",
			cmd.args[6], cmd.args[7], cmd.args[8]);

	state->fw_loaded = true;

warm:

	state->active = true;
	return 0;
err_release_firmware:
	release_firmware(fw);
err:
	dprintk_err("%s: failed=%d\n", __func__, ret);
	return ret;
}

static int si2157_sleep(struct dvb_frontend *fe)
{
	struct si2157_state *state = fe->tuner_priv;
	int ret;
	struct si2157_cmd cmd;

	state->active = false;

	/* standby */
	memcpy(cmd.args, "\x16\x00", 2);
	cmd.wlen = 2;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	return 0;
err:
	dprintk_err("%s: failed=%d\n", __func__, ret);
	return ret;
}

static int si2157_set_params(struct dvb_frontend *fe,
	struct dvb_frontend_parameters *p)
{
	struct si2157_state *state = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	struct si2157_cmd cmd;
	u8 bandwidth, delivery_system;
	u32 if_frequency = 5000000;

	dprintk("delivery_system=%d frequency=%u bandwidth_hz=%u\n",
			c->delivery_system, c->frequency, c->bandwidth_hz);

	if (!state->active) {
		ret = -EAGAIN;
		goto err;
	}

	if (c->bandwidth_hz <= 6000000)
		bandwidth = 0x06;
	else if (c->bandwidth_hz <= 7000000)
		bandwidth = 0x07;
	else if (c->bandwidth_hz <= 8000000)
		bandwidth = 0x08;
	else
		bandwidth = 0x0f;

	switch (c->delivery_system) {
	case SYS_ATSC:
			delivery_system = 0x00;
			if_frequency = 3250000;
			break;
	case SYS_DVBC_ANNEX_B:
			delivery_system = 0x10;
			if_frequency = 4000000;
			break;
	case SYS_DVBT:
	case SYS_DVBT2: /* it seems DVB-T and DVB-T2 both are 0x20 here */
			delivery_system = 0x20;
			break;
	case SYS_DVBC_ANNEX_A:
			delivery_system = 0x30;
			break;
	default:
			ret = -EINVAL;
			goto err;
	}

	memcpy(cmd.args, "\x14\x00\x03\x07\x00\x00", 6);
	cmd.args[4] = delivery_system | bandwidth;
	if (state->inversion)
		cmd.args[5] = 0x01;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	if (state->chiptype == SI2157_CHIPTYPE_SI2146)
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x01", 6);
	else
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x00", 6);
	cmd.args[4] = state->if_port;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	/* set if frequency if needed */
	if (if_frequency != state->if_frequency) {
		memcpy(cmd.args, "\x14\x00\x06\x07", 4);
		cmd.args[4] = (if_frequency / 1000) & 0xff;
		cmd.args[5] = ((if_frequency / 1000) >> 8) & 0xff;
		cmd.wlen = 6;
		cmd.rlen = 4;
		ret = si2157_cmd_execute(state, &cmd);
		if (ret)
			goto err;

		state->if_frequency = if_frequency;
	}

	/* set frequency */
	memcpy(cmd.args, "\x41\x00\x00\x00\x00\x00\x00\x00", 8);
	cmd.args[4] = (c->frequency >>  0) & 0xff;
	cmd.args[5] = (c->frequency >>  8) & 0xff;
	cmd.args[6] = (c->frequency >> 16) & 0xff;
	cmd.args[7] = (c->frequency >> 24) & 0xff;
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	return 0;
err:
	dprintk_err("%s: failed=%d\n", __func__, ret);
	return ret;
}

static int si2157_get_rf_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct si2157_state *state = fe->tuner_priv;
	struct si2157_cmd cmd;
	int ret;

	memcpy(cmd.args, "\x42\x00", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err;

	*strength = (256 - cmd.args[3]) * (0xffff/100);
	return 0;
err:
	dprintk_err("%s: failed=%d\n", __func__, ret);
	return ret;
}

static int si2157_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static const struct dvb_tuner_ops si2157_ops = {
	.info = {
		.name           = "Silicon Labs Si2146/2147/2148/2157/2158",
		.frequency_min  = 55000000,
		.frequency_max  = 862000000,
	},

	.init = si2157_init,
	.release = si2157_release,
	.sleep = si2157_sleep,
	.set_params = si2157_set_params,
	.get_rf_strength = si2157_get_rf_strength
};

struct dvb_frontend *si2157_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c, struct si2157_config *cfg)
{
	struct si2157_state *state = NULL;
	int ret;
	struct si2157_cmd cmd;

	state = kzalloc(sizeof(struct si2157_state), GFP_KERNEL);
	if (state == NULL)
		return NULL;

	state->fe = fe;
	state->i2c = i2c;
	state->i2c_addr = cfg->i2c_addr;
	state->inversion = cfg->inversion;
	state->if_port = cfg->if_port;
	state->fw_loaded = false;
	state->chiptype = (u8)cfg->chiptype;
	state->if_frequency = 5000000; /* default value of property 0x0706 */

	/* check if the tuner is there */
	cmd.wlen = 0;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(state, &cmd);
	if (ret)
		goto err_kfree;

	memcpy(&fe->ops.tuner_ops, &si2157_ops, sizeof(struct dvb_tuner_ops));
	fe->tuner_priv = state;

	dprintk_info("Silicon Labs %s successfully attached\n",
			state->chiptype == SI2157_CHIPTYPE_SI2146 ?
			"Si2146" : "Si2147/2148/2157/2158");
	return fe;
err_kfree:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(si2157_attach);

MODULE_DESCRIPTION("Silicon Labs Si2146/2147/2148/2157/2158 silicon tuner driver");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(SI2158_A20_FIRMWARE);
