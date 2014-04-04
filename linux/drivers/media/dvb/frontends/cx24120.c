/*	
    Conexant cx24120/cx24118 - DVBS/S2 Satellite demod/tuner driver
	Version 0.0.3	06.09.2009 13:16:39
	
	Copyright (C) 2009 Sergey Tyurin <forum.free-x.de>

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
#include "cx24120.h"
#include "cx24120_const.h"

//==========================
#define dbginfo(args...) do { if(cx24120_debug) { printk(KERN_DEBUG "CX24120: %s: >>> ", __func__); \
			printk(args); }  } while (0)
#define info(args...) do { printk(KERN_INFO "CX24120: %s: -> ", __func__); \
			printk(args); } while (0)
#define err(args...) do {  printk(KERN_ERR "CX24120: %s: ### ERROR: ", __func__); \
			printk(args); } while (0)
//==========================

static int cx24120_debug=0;
static int reg_debug=0;
module_param(cx24120_debug, int, 0644);
MODULE_PARM_DESC(cx24120_debug, "Activates frontend debugging (default:0)");

// ##############################
struct cx24120_state {
	struct i2c_adapter *i2c;
	const struct cx24120_config *config;
	struct dvb_frontend frontend;
	u8 need_set_mpeg_out;
	u8 attached;
	u8 dvb_s2_mode;
	u8 cold_init;	
}; 
// #####################################
// #### Command message to firmware ####
struct cx24120_cmd {			// total size = 36
	u8 id;						// [00] - message id
	u8 arg[30];					// [04] - message first byte
	u8 len;						// [34] - message lengh or first registers to read
	u8 reg;						// [35] - number of registers to read
};

//===================================================================
static int cx24120_readreg(struct cx24120_state *state, u8 reg) 
{
	int ret;
	u8 buf = 0;
	struct i2c_msg msg[] = {
		{ 	.addr = state->config->i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg	},
			
		{ 	.addr = state->config->i2c_addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &buf	}
	};
	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret != 2) {
		err("Read error: reg=0x%02x,  ret=0x%02x)\n", reg, ret);
		return ret;
	}
	if (reg_debug) dbginfo("reg=0x%02x; data=0x%02x\n", reg, buf);
	return buf;
} // end cx24123_readreg
//===================================================================
static int cx24120_writereg(struct cx24120_state *state, u8 reg, u8 data) 
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr = state->config->i2c_addr, 
		.flags = 0, 
		.buf = buf, 
		.len = 2 };
	int ret;
	ret = i2c_transfer(state->i2c, &msg, 1);
	if (ret != 1) {
		err("Write error: i2c_write error(err == %i, 0x%02x: 0x%02x)\n", ret, reg, data);
		return ret;
	}
	if (reg_debug) dbginfo("reg=0x%02x; data=0x%02x\n", reg, data);
	return 0;
} // end cx24123_writereg
//===================================================================
static int cx24120_writeregN(struct cx24120_state *state, u8 reg, u8 *values, u16 len, u8 incr)
{
	u8 buf[5]; /* maximum 4 data bytes at once - flexcop limitation (very limited i2c-interface this one) */
	struct i2c_msg msg = {
		.addr = state->config->i2c_addr, 
		.flags = 0, 
		.buf = buf, 
		.len = len };
	int ret;

	do {
		buf[0] = reg;
		msg.len = len > 4 ? 4 : len;
		memcpy(&buf[1], values, msg.len);
		len  -= msg.len;					// data length revers counter
		values += msg.len;					// incr data pointer
		if (incr) reg += msg.len; 			
		msg.len++; 							/* don't forget the addr byte */
		ret = i2c_transfer(state->i2c, &msg, 1);
		if (ret != 1) {
			err("i2c_write error(err == %i, 0x%02x)\n", ret, reg);
			return ret;
		}
		if (reg_debug) {
			if( !(reg == 0xFA) && !(reg == 0x20) && !(reg == 0x21)) {		// Exclude firmware upload & diseqc messages
				dbginfo("reg=0x%02x; data=0x%02x,0x%02x,0x%02x,0x%02x\n",	// from debug
						reg, 	buf[1], buf[2], buf[3], buf[4]);
			}
		}
	} while (len);
	return 0;
} // end cx24123_writeregN
//===================================================================
static struct dvb_frontend_ops cx24120_ops;
//===================================================================
struct dvb_frontend *cx24120_attach(const struct cx24120_config *config, struct i2c_adapter *i2c)
{
	struct cx24120_state *state = NULL;
	int demod_rev;

	info("Conexant cx24120/cx24118 - DVBS/S2 Satellite demod/tuner\n");
	info("Driver version: 'SVT - 0.0.3 - 06.09.2009 13:16:39'\n");
	state = kzalloc(sizeof(struct cx24120_state),
						GFP_KERNEL);
	if (state == NULL) {
		err("### Unable to allocate memory for cx24120_state structure. :(\n");
		goto error;
	}
	/* setup the state */
	state->config = config;
	state->i2c = i2c;
	/* check if the demod is present and has proper type */
	demod_rev = cx24120_readreg(state, CX24120_REG_REVISION);
	switch (demod_rev) {
	case 0x07:
		info("Demod CX24120 rev. 0x07 detected.\n");
		break;
	case 0x05:
		info("Demod CX24120 rev. 0x05 detected.\n");
		break;
	default:
		err("### Unsupported demod revision: 0x%x detected. Exit.\n", demod_rev);
		goto error;
	}
	/* create dvb_frontend */
	state->attached = 0x10;			// set attached flag
	state->cold_init=0;
	memcpy(&state->frontend.ops, &cx24120_ops, sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;
	info("Conexant cx24120/cx24118 - DVBS/S2 Satellite demod/tuner ATTACHED.\n");
	return &state->frontend;

error:
	kfree(state);
	return NULL;
} 
EXPORT_SYMBOL(cx24120_attach); // end cx24120_attach
//===================================================================
static int cx24120_test_rom(struct cx24120_state *state)
{
	int err, ret;
	err = cx24120_readreg(state, 0xFD);
	if (err & 4 )
		{
			ret = cx24120_readreg(state, 0xDF) & 0xFE;
			err = cx24120_writereg(state, 0xDF, ret);
		}
	return err;
} // end  cx24120_test_rom
//===================================================================
static int cx24120_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct cx24120_state *state = fe->demodulator_priv;
	
	*snr = (cx24120_readreg(state, CX24120_REG_QUALITY_H)<<8) |
				 (cx24120_readreg(state, CX24120_REG_QUALITY_L));
	dbginfo("read SNR index = %d\n", *snr);
	
	return 0;
}
EXPORT_SYMBOL(cx24120_read_snr); // end cx24120_read_snr
//===================================================================
static int cx24120_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct cx24120_state *state = fe->demodulator_priv;

	*ber =  (cx24120_readreg(state, CX24120_REG_BER_HH) << 24)	|	// BER high byte of high word
		(cx24120_readreg(state, CX24120_REG_BER_HL) << 16)		|	// BER low byte of high word
		(cx24120_readreg(state, CX24120_REG_BER_LH)  << 8)		|	// BER high byte of low word
		 cx24120_readreg(state, CX24120_REG_BER_LL);				// BER low byte of low word
		dbginfo("read BER index = %d\n", *ber);

	return 0;
}
EXPORT_SYMBOL(cx24120_read_ber); // end cx24120_read_ber
//===================================================================
static int cx24120_message_send(struct cx24120_state *state, struct cx24120_cmd *cmd);
//===================================================================
static int cx24120_msg_mpeg_output_global_config(struct cx24120_state *state, u8 flag)
{
	u8 tristate;
	struct cx24120_cmd cmd;

	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	
	cmd.id = 0x13;						// (19) message Enable/Disable mpeg output ???
	cmd.arg[0] = 1;
	cmd.arg[1] = 0;
	tristate = flag ? 0 : (u8)(-1);
	cmd.arg[2] = tristate;
	cmd.arg[3] = 1;						
	cmd.len = 4;

	if(flag) dbginfo("MPEG output DISABLED\n");
	else dbginfo("MPEG output ENABLED\n");
  
	return cx24120_message_send(state, &cmd);
}		// end	cx24120_msg_mpeg_output_global_config
//===================================================================
static int cx24120_message_send(struct cx24120_state *state, struct cx24120_cmd *cmd)
{
	u8 xxzz;
	u32 msg_cmd_mask;
	int ret, ficus;

	if(state->dvb_s2_mode & 0x02) {		// is MPEG enabled?
										// if yes:
		xxzz = cmd->id - 0x11;			// look for specific message id
		if ( xxzz <= 0x13 ) {
			msg_cmd_mask = 1 << xxzz;
			//0x0F8021 // if cmd_id 17 or 22 or 33-36, 42, 47, 57-61 etc. disable mpeg output
			if ( msg_cmd_mask & 0x0F8021 ) {		// 000011111000000000100001b
				cx24120_msg_mpeg_output_global_config(state, 0);
				msleep(100);
				state->dvb_s2_mode &=  0xFD;		// reset mpeg out enable flag
			}
		}
	}
	ret = cx24120_writereg(state, 0x00 /* reg id*/, cmd->id /* value */);			// message start & target
	ret = cx24120_writeregN(state, 0x01 /* reg msg*/, &cmd->arg[0], cmd->len /* len*/, 1 /* incr */);		// message data
	ret = cx24120_writereg(state, 0x1F /* reg msg_end */, 0x01 /* value */);		// message end

	ficus = 1000;
	while ( cx24120_readreg(state, 0x1F)) { 	// is command done???
		msleep(1);
		if( !(--ficus)) {
			err("Too long waiting 'done' state from reg(0x1F). :(\n");
			return -EREMOTEIO;
		}
	}
	dbginfo("Successfully send message 0x%02x\n", cmd->id);

	if ( cmd->reg > 30 ) {
		err("Too much registers to read. cmd->reg = %d", cmd->reg);
		return -EREMOTEIO;
	}
	ficus = 0;
	if ( cmd->reg ) {					// â cmd->reg - qty consecutive regs to read 
		while ( ficus < cmd->reg ){		// starts from reg No cmd->len
										// number of registers to read is cmd->reg
										// and write results starts from cmd->arg[0].
			cmd->arg[ficus] = cx24120_readreg(state, (cmd->len+ficus+1)); 		
			++ficus;
		}
	}		
	return 0;
} // end cx24120_message_send
//===================================================================
static int cx24120_set_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	u32 srate, freq;
	fe_code_rate_t fec;
	fe_spectral_inversion_t inversion;
	u8 smbr1, smbr2;
	int ret;
	
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	
	cmd.id = CMD_TUNEREQUEST;		// 0x11 set tuner parametrs
	cmd.len = 15;		
	
	freq = p->frequency;
	srate = p->u.qpsk.symbol_rate;
	fec = p->u.qpsk.fec_inner;
	inversion = p->inversion;
	
	// check symbol rate
	if ( srate  > 31000000 ) {      	// if symbol rate > 31 000
		smbr1 = (-(srate < 31000001) & 3) + 2;		// ebp
		smbr2 = (-(srate < 31000001) & 6) + 4;		// edi
	} else {
		smbr1 = 3;
		smbr2 = 6;
	}

	ret = cx24120_writereg(state, 0xE6, smbr1);
	ret = cx24120_readreg(state, 0xF0);
	ret &= 0xFFFFFFF0;
	ret |= smbr2;
	ret = cx24120_writereg(state, 0xF0, ret);
	
	cmd.arg[0] = 0;		// CMD_TUNER_REQUEST
	
	// Frequency
	cmd.arg[1] = (freq & 0xFF0000) >> 16;		/* intermediate frequency in kHz */
	cmd.arg[2] = (freq & 0x00FF00) >> 8;
	cmd.arg[3] = (freq & 0x0000FF);
	
	// Symbol Rate
	cmd.arg[4] = ((srate/1000) & 0xFF00) >> 8;
	cmd.arg[5] = ((srate/1000) & 0x00FF);
	
	// Inversion
	if ( inversion ) {
    	if ( inversion == 1 ) cmd.arg[6] = 4;
		else cmd.arg[6] = 0x0C;
	} else {
		cmd.arg[6] = 0;
	}

	// FEC
	switch ( fec )			// fec = p->u.qpsk.fec_inner
	{
		case 1:							// FEC_1_2
			cmd.arg[7] = 0x2E; break;	// è [11] = 0 by memset	
		case 2:							// FEC_2_3
			cmd.arg[7] = 0x2F; break;
		case 3:							// FEC_3_4
			cmd.arg[7] = 0x30; break;
		case 5:							// FEC_5_6
			cmd.arg[7] = 0x31; break;
		case 7:							// FEC_7_8
			cmd.arg[7] = 0x32; break;
		default:								// FEC_NONE, FEC_4_5, FEC_6_7, FEC_8_9, 
    											// FEC_AUTO, FEC_3_5, FEC_9_10
			if ( state->dvb_s2_mode & 1 ) {	// if DVB-S2 mode	
				cmd.arg[7] = 0;				
				cmd.arg[11] = 0;
			} else {
				cmd.arg[7] = 0x2E;
				cmd.arg[11] = 0xAC;
			} 
			break;
	}
	cmd.arg[8] = 0x13;
	cmd.arg[9] = 0x88;
	cmd.arg[10] = 0;
	cmd.arg[12] = smbr2;
	cmd.arg[13] = smbr1;
	cmd.arg[14] = 0;

	state->need_set_mpeg_out |= 0x01;		// after tune we need restart mpeg out ?????

	return cx24120_message_send(state, &cmd);
	
} 
EXPORT_SYMBOL(cx24120_set_frontend);		// end cx24120_set_frontend
//===================================================================
void cx24120_message_fill(struct cx24120_cmd *cmd,
		u8 msg_id, 
		u8 *msg_addr,
		u8 msg_len,
		u8 num_regs)
{
	cmd->id = msg_id;
	memcpy(&cmd->arg[0], msg_addr, msg_len);
	cmd->len = msg_len;
	cmd->reg = num_regs;
} // end cx24120_message_fill
//===================================================================
static int cx24120_read_signal_strength(struct dvb_frontend *fe, u16 *signal_strength)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	int result, sigstr_h, sigstr_l;

	cx24120_message_fill(&cmd, 0x1A/*msg_id*/, &cx24120_msg_read_sigstr[0], 1/*msg_len*/, 0/*num_regs*/);

	if( !(cx24120_message_send(state, &cmd)) ) {
		sigstr_h = (cx24120_readreg(state, CX24120_REG_SIGSTR_H) >> 6) << 8;
		sigstr_l = cx24120_readreg(state, CX24120_REG_SIGSTR_L );
		dbginfo("Signal strength from firmware= 0x%x\n", (sigstr_h | sigstr_l));
		*signal_strength = ((sigstr_h | sigstr_l)  << 5) & 0x0000FFFF;
		dbginfo("Signal strength= 0x%x\n", *signal_strength);
		result = 0;
	} else {
		err("error reading signal strength\n");
		result = -EREMOTEIO;
	}
	return result;
}
EXPORT_SYMBOL(cx24120_read_signal_strength);		// end cx24120_read_signal_strength
//===================================================================
static int cx24120_msg_mpeg_output_config(struct cx24120_state *state, u8 num, 
			struct cx24120_skystar2_mpeg_config *config_msg)
{
	struct cx24120_cmd cmd;
	
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
  
	cmd.id = CMD_MPEG_INIT;          // cmd->id=20 - message id
	cmd.len = 7;            
	cmd.arg[0] = num;     // sequental number - can be 0,1,2
	cmd.arg[1] =  	((config_msg->x1 & 0x01) << 1) |
					((config_msg->x1 >> 1) & 0x01);
	cmd.arg[2] = 0x05;
	cmd.arg[3] = 0x02;
	cmd.arg[4] = ((config_msg->x2 >> 1) & 0x01);
	cmd.arg[5] = (config_msg->x2 & 0xF0) | (config_msg->x3 & 0x0F);
	cmd.arg[6] = state->attached; 	/* 0x10 if succesfully attached */
  
  return cx24120_message_send(state, &cmd);
}	// end cx24120_msg_mpeg_output_config
//===================================================================
static int cx24120_diseqc_send_burst(struct dvb_frontend *fe, fe_sec_mini_cmd_t burst)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	
	cmd.id = CMD_DISEQC_BURST;
	cmd.arg[0] = 0x00;
	if (burst)
		cmd.arg[1] = 0x01;
	dbginfo("burst sent.\n");

  return cx24120_message_send(state, &cmd);
}
EXPORT_SYMBOL(cx24120_diseqc_send_burst);		// end cx24120_diseqc_send_burst
//===================================================================
static int cx24120_set_tone(struct dvb_frontend *fe, fe_sec_tone_mode_t tone)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;

	dbginfo("cmd(0x23,4) - tone = %d\n", tone);
	if ((tone != SEC_TONE_ON) && (tone != SEC_TONE_OFF)) {
		err("Invalid tone=%d\n", tone);
		return -EINVAL;
	}
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	cmd.id = CMD_SETTONE;	// 0x23
	cmd.len = 4;
	if (!tone)
		cmd.arg[3] = 0x01;
  return cx24120_message_send(state, &cmd);
}
EXPORT_SYMBOL(cx24120_set_tone);		// end cx24120_set_tone
//===================================================================
static int cx24120_set_voltage(struct dvb_frontend *fe,	fe_sec_voltage_t voltage)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;

	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	cmd.id = CMD_SETVOLTAGE;		//
	cmd.len = 2;
	if (!(voltage - 1))
		cmd.arg[1] = 0x01;
	return cx24120_message_send(state, &cmd);
}
EXPORT_SYMBOL(cx24120_set_voltage);		// end cx24120_set_voltage
//===================================================================
static int cx24120_send_diseqc_msg(struct dvb_frontend *fe,	struct dvb_diseqc_master_cmd *d)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	int back_count;
	
//	dbginfo("Start sending diseqc sequence===============\n");
	
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	
	cmd.id = CMD_DISEQC_MSG1;		// 0x20
	cmd.len = 11;
	cmd.arg[0] = 0x00;
	cmd.arg[1] = 0x00;
	cmd.arg[2] = 0x03;
	cmd.arg[3] = 0x16;
	cmd.arg[4] = 0x28;
	cmd.arg[5] = 0x01;
	cmd.arg[6] = 0x01;
	cmd.arg[7] = 0x14;
	cmd.arg[8] = 0x19;
	cmd.arg[9] = 0x14;
	cmd.arg[10] = 0x1E;
	if ( cx24120_message_send(state, &cmd) ) {
		err("send 1st message(0x%x) filed==========\n", cmd.id);
		return -EREMOTEIO;
	}
	cmd.id = CMD_DISEQC_MSG2;		// 0x21
	cmd.len = d->msg_len + 6;
	cmd.arg[0] = 0x00;
	cmd.arg[1] = 0x01;
	cmd.arg[2] = 0x02;
	cmd.arg[3] = 0x00;
	cmd.arg[4] = 0x00;
	cmd.arg[5] = d->msg_len;
	
	memcpy(&cmd.arg[6], &d->msg, d->msg_len);
	
	if ( cx24120_message_send(state, &cmd) ) {
		err("send 2d message(0x%x) filed========\n", cmd.id);
		return -EREMOTEIO;
	}
	back_count = 100;
	do {
		if ( !(cx24120_readreg(state, 0x93) & 0x01) ) {	
//			dbginfo("diseqc sequence sent success==========.\n");
			return 0;
		}
		msleep(5);
		--back_count;
	} while ( back_count );
	err("Too long waiting for diseqc.=============\n");
	return -ETIMEDOUT;
}
EXPORT_SYMBOL(cx24120_send_diseqc_msg);		// end cx24120_send_diseqc_msg
//===================================================================
static int cx24120_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	int ret, clock_seq_num;
	u8 mode_code, mode_8PSK_flag, attached_flag, clock_id;
	
	ret = cx24120_readreg(state, CX24120_REG_STATUS);		//0x3A
	dbginfo("status = 0x%x\n", ret);
	*status = 0;
	if ( ret & CX24120_HAS_SIGNAL ) *status = FE_HAS_SIGNAL;
	if ( ret & CX24120_HAS_CARRIER) *status |= FE_HAS_CARRIER;
	if ( ret & CX24120_HAS_VITERBI) *status |= (FE_HAS_VITERBI + FE_HAS_SYNC);
	
	if ( ret & CX24120_HAS_LOCK ) {		// 0x08
		*status |= FE_HAS_LOCK;
		if ( state->need_set_mpeg_out & 1 ) {			// just tuned???
			memset(&cmd, 0, sizeof(struct cx24120_cmd));
			cmd.id = CMD_CLOCK_READ;
			cmd.arg[0] = 0x00;
			cmd.len = 1;			// cmd.reg != 0, so it is first register to read 
			cmd.reg = 6;			// number of registers to read (0x01-0x06)
			if ( !cx24120_message_send(state, &cmd) ) {			// in cmd[0]-[5] - result
																//      0x02-0x07
				ret = cx24120_readreg(state, CX24120_REG_FECMODE) & 0x3F;		// ntv - 0x8E & 3F = 14
				dbginfo("Get FEC: %d\n", ret);
				if ( state->dvb_s2_mode & 0x01 ) {							// is DVB-S2?
					switch (ret-4) {
						case 0:
							mode_code = 0x01; goto mode_QPSK;	// FEC_1_2 - qpsk only
						case 1:
						case 8:
							mode_code = 0x64; goto mode_8PSK;	// FEC_3_5 (10)- 8PSK only
						case 2:
						case 9:
							mode_code = 0x02; goto mode_8PSK;	// FEC_2_3
						case 3:
						case 10:
							mode_code = 0x03; goto mode_8PSK;	// FEC_3_4	// 14-4=10 - ntv+
						case 4:
							mode_code = 0x04; goto mode_QPSK;	// FEC_4_5 - qpsk only
						case 5:
						case 11:
							mode_code = 0x05; goto mode_8PSK;	// FEC_5_6
						case 6:
						case 12:
							mode_code = 0x08; goto mode_8PSK;	// FEC_8_9
						case 7:
						case 13:
							mode_code = 0x65; goto mode_8PSK;	// FEC_9_10 (11)- 8PSK only
						default:
							info("Unknown DVB-S2 modefec (not QPSK or 8PSK): %d\n", ret-4);
							mode_code = 0x01; 						// set like for mode 0
					mode_8PSK:
						if ( ret > 11 ) {		// 14
							mode_8PSK_flag = 0x63;			// DVB-S2-8PSK flag
							dbginfo("DVB-S2: 8PSK mode: %d, mode_code= 0x%x\n", ret-4, mode_code);
						} else {
					mode_QPSK:
							mode_8PSK_flag = 0x00;
   		     	      		dbginfo("DVB-S2: QPSK mode: %d\n", ret-4);
						}
						break;
					} // end switch
				} // end if dvb_s2_mode // dvb-s2
				else {								// state->dvb_s2_mode & 1 = 0 -> #### DVB-S
					switch ( ret - 2 ) {
						case 0:
							mode_code = 2; break;	// FEC_2_3
						case 1:
							mode_code = 3; break;	// FEC_3_4
						case 2:
							mode_code = 4; break;	// FEC_4_5
						case 3:
							mode_code = 5; break;	// FEC_5_6
						case 4:
							mode_code = 6; break;	// FEC_6_7
						case 5:
							mode_code = 7; break;	// FEC_7_8
						default: 
							mode_code = 1;break;	// FEC_1_2
					}
					mode_8PSK_flag = 0;
				} // end of switch for dvb-s
				
				attached_flag = 0x10;
				if (state->attached == 0x10)  // must be 0x10 if successfully attached in flexcop_fe_tuner
					attached_flag = 0;
				ret = 0;
				if ( state->dvb_s2_mode & 0x01 )		// if dvb-s2
					ret = (cx24120_readreg(state, CX24120_REG_FECMODE) >> 7) & 0x01;  // QPSK or 8PSK ???
						// bit 4        bit 5			bit 0               bit 3
				clock_id = (ret << 3) | attached_flag | (state->dvb_s2_mode & 1) | 4;		// possible id: 4, 5, 13. 12-impossible, 
				// ntv S2 = 0x8E -> 8 | 1 | 4 = 13											// because 7th bit of ret - is S2 flag
				
				dbginfo("Check clock table for: clock_id=0x%x, 8PSK_mask=0x%x, mode_code=0x%x\n", 
				 	clock_id, mode_8PSK_flag, mode_code);
				
				clock_seq_num = 0;
				while ( (clock_ratios_table[clock_seq_num].ratio_id != clock_id) ||
						(clock_ratios_table[clock_seq_num].mode_xPSK != mode_8PSK_flag) ||
						(clock_ratios_table[clock_seq_num].fec_mode != mode_code) )
				{
				/*	dbginfo("Check table string(%d): clock_id=%d, 8PSK_flag=%d, mode_code=%d\n",	clock_seq_num, 
				 *		clock_ratios_table[clock_seq_num].ratio_id,
				 *		clock_ratios_table[clock_seq_num].mode_xPSK,
				 *		clock_ratios_table[clock_seq_num].fec_mode);
				 */
					++clock_seq_num;
					if ( clock_seq_num == ARRAY_SIZE(clock_ratios_table) ) {
						info("Check in clock table filed: unsupported modulation tuned - data reception in danger. :(\n");
						goto settings_end;
					}
				}
				dbginfo("Check succesful: post lock; m: %d, n: %d, clock_seq_idx: %d m: %d, n: %d, rate: %d\n",
					cmd.arg[2] | (cmd.arg[1] << 8) | (cmd.arg[0] << 16), 		// registers was readed early
					cmd.arg[5] | (cmd.arg[4] << 8) | (cmd.arg[3] << 16),		// in message with id = 0x16
					clock_seq_num,
					clock_ratios_table[clock_seq_num].m_rat,
					clock_ratios_table[clock_seq_num].n_rat,
					clock_ratios_table[clock_seq_num].rate);
 
				cmd.id = CMD_CLOCK_SET;
				cmd.len = 10;
				cmd.reg = 0;
				cmd.arg[0] = 0;
				cmd.arg[1] = state->attached;		// must be 0x10 if successfully attached in flexcop_fe_tuner
				
				cmd.arg[2] = (clock_ratios_table[clock_seq_num].m_rat >> 16) & 0xFF;
				cmd.arg[3] = (clock_ratios_table[clock_seq_num].m_rat >>  8) & 0xFF;
				cmd.arg[4] = (clock_ratios_table[clock_seq_num].m_rat >>  0) & 0xFF;

				cmd.arg[5] = (clock_ratios_table[clock_seq_num].n_rat >> 16) & 0xFF;
				cmd.arg[6] = (clock_ratios_table[clock_seq_num].n_rat >>  8) & 0xFF;
				cmd.arg[7] = (clock_ratios_table[clock_seq_num].n_rat >>  0) & 0xFF;

				cmd.arg[8] = (clock_ratios_table[clock_seq_num].rate >> 8) & 0xFF;
				cmd.arg[9] = (clock_ratios_table[clock_seq_num].rate >> 0) & 0xFF;
				
				cx24120_message_send(state, &cmd);
				
			settings_end:
				msleep(200);
				cx24120_msg_mpeg_output_global_config(state, 1);	
				state->dvb_s2_mode |= 0x02;							// set mpeg flag
				state->need_set_mpeg_out &= 0xFE;					// clocks set done -> clear flag
			}
		}
	}
	return 0;
}
EXPORT_SYMBOL(cx24120_read_status);		// end cx24120_read_status
//===================================================================
int cx24120_init(struct dvb_frontend *fe)
{
	const struct firmware *fw;
	struct cx24120_state *state = fe->demodulator_priv;
	struct cx24120_cmd cmd;
	u8 ret, ret_EA, reg1, fL, fH;
	u32 vco, xtal_khz;
	u64 inv_vco, res, xxyyzz;
	int reset_result;

	if( state->cold_init ) return 0;

	ret = cx24120_writereg(state, 0xEA, 0x00);
	ret = cx24120_test_rom(state);
	ret = cx24120_readreg(state, 0xFB) & 0xFE;
	ret = cx24120_writereg(state, 0xFB, ret);
	ret = cx24120_readreg(state, 0xFC) & 0xFE;
	ret = cx24120_writereg(state, 0xFC, ret);
	ret = cx24120_writereg(state, 0xC3, 0x04);
	ret = cx24120_writereg(state, 0xC4, 0x04);
	ret = cx24120_writereg(state, 0xCE, 0x00);
	ret = cx24120_writereg(state, 0xCF, 0x00);
	ret_EA = cx24120_readreg(state, 0xEA) & 0xFE;
	ret = cx24120_writereg(state, 0xEA, ret_EA);
	ret = cx24120_writereg(state, 0xEB, 0x0C);
	ret = cx24120_writereg(state, 0xEC, 0x06);
	ret = cx24120_writereg(state, 0xED, 0x05);
	ret = cx24120_writereg(state, 0xEE, 0x03);
	ret = cx24120_writereg(state, 0xEF, 0x05);
	ret = cx24120_writereg(state, 0xF3, 0x03);
	ret = cx24120_writereg(state, 0xF4, 0x44);
	
	reg1 = 0xF0;
	do {
		cx24120_writereg(state, reg1, 0x04);
		cx24120_writereg(state, reg1 - 10, 0x02);
		++reg1;
	} while ( reg1 != 0xF3 );

	ret = cx24120_writereg(state, 0xEA, (ret_EA | 0x01));
		reg1 = 0xC5;
	do {
		ret = cx24120_writereg(state, reg1, 0x00);
		ret = cx24120_writereg(state, reg1 + 1, 0x00);
		reg1 += 2;
    } while ( reg1 != 0xCB );
    
    ret = cx24120_writereg(state, 0xE4, 0x03);
    ret = cx24120_writereg(state, 0xEB, 0x0A);
    
    dbginfo("Requesting firmware (%s) to download...\n", CX24120_FIRMWARE);
    ret = state->config->request_firmware(fe, &fw, CX24120_FIRMWARE);
	if (ret) {
		err("Could not load firmware (%s): %d\n", CX24120_FIRMWARE, ret);
		return ret;
	}
	dbginfo("Firmware found and it size is %d bytes (%02x %02x .. %02x %02x)\n",
        fw->size,				// firmware_size in bytes u32*
        fw->data[0],			// fw 1st byte
        fw->data[1],			// fw 2d byte
        fw->data[fw->size - 2],	// fw before last byte
        fw->data[fw->size - 1]);	// fw last byte
        
    ret = cx24120_test_rom(state);
	ret = cx24120_readreg(state, 0xFB) & 0xFE;
	ret = cx24120_writereg(state, 0xFB, ret);
	ret = cx24120_writereg(state, 0xE0, 0x76);
	ret = cx24120_writereg(state, 0xF7, 0x81);
	ret = cx24120_writereg(state, 0xF8, 0x00);
	ret = cx24120_writereg(state, 0xF9, 0x00);
	ret = cx24120_writeregN(state, 0xFA, (u8*) fw->data, (fw->size - 1), 0x00);
	ret = cx24120_writereg(state, 0xF7, 0xC0);
	ret = cx24120_writereg(state, 0xE0, 0x00);
	ret = (fw->size - 2) & 0x00FF;
	ret = cx24120_writereg(state, 0xF8, ret);	// ret now is 0x7a
	ret = ((fw->size - 2) >> 8) & 0x00FF;
	ret = cx24120_writereg(state, 0xF9, ret);	// ret now is 0xaf
	ret = cx24120_writereg(state, 0xF7, 0x00);
	ret = cx24120_writereg(state, 0xDC, 0x00);
	ret = cx24120_writereg(state, 0xDC, 0x07);
	msleep(500);
	
	ret = cx24120_readreg(state, 0xE1);		// now is 0xd5 - last byte of the firmware
    if ( ret == fw->data[fw->size - 1] ) {
		dbginfo("Firmware uploaded successfully\n");
		reset_result = 0;
	} else {
		err("Firmware upload failed. Last byte returned=0x%x\n", ret );
		reset_result = -EREMOTEIO;
	}
	ret = cx24120_writereg(state, 0xDC, 0x00);		
	release_firmware(fw);
	if (reset_result) 
		return reset_result;
		
	//================== Start tuner
	cx24120_message_fill(&cmd, CMD_START_TUNER, &cx24120_msg_tuner_init[0], 3, 0);	// 0x1B
	if(cx24120_message_send(state, &cmd)) {
		err("Error tuner start! :(\n");
		return -EREMOTEIO;
	}
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	
	cmd.id = CMD_VCO_SET;		// 0x10
	cmd.len = 12;
	
	// ######################
	// Calc VCO
	xtal_khz = 10111;
	xxyyzz = 0x400000000ULL;	// 17179869184
	vco = xtal_khz * 10 * 4;	// 404440
	inv_vco = xxyyzz / vco;		// 42478 = 0x00A5EE
	res = xxyyzz % vco;			// 66864 = 0x010530	
	
	if( inv_vco > xtal_khz * 10 * 2) ++inv_vco;
		
	fH = (inv_vco >> 8) & 0xFF;
	fL = (inv_vco) & 0xFF;
    dbginfo("vco= %d, inv_vco= %lld, res= %lld, fL= 0x%x, fH= 0x%x\n", vco, inv_vco, res, fL, fH);
    // ######################
    
	cmd.arg[0] = 0x06;
	cmd.arg[1] = 0x2B;
	cmd.arg[2] = 0xD8;
	cmd.arg[3] = fH;		// 0xA5
	cmd.arg[4] = fL;		// 0xEE
	cmd.arg[5] = 0x03;
	cmd.arg[6] = 0x9D;
	cmd.arg[7] = 0xFC;
	cmd.arg[8] = 0x06;
	cmd.arg[9] = 0x03;
	cmd.arg[10] = 0x27;
	cmd.arg[11] = 0x7F;
	
	if(cx24120_message_send(state, &cmd)) {
		err("Error set VCO! :(\n");
		return -EREMOTEIO;
	}
	memset(&cmd, 0, sizeof(struct cx24120_cmd));
	// set bandwidth
	cmd.id = CMD_BANDWIDTH;		// 0x15
	cmd.len = 12;
	cmd.arg[0] = 0x00;
	cmd.arg[1] = 0x00;
	cmd.arg[2] = 0x00;
	cmd.arg[3] = 0x00;
	cmd.arg[4] = 0x05;
	cmd.arg[5] = 0x02;
	cmd.arg[6] = 0x02;
	cmd.arg[7] = 0x00;
	cmd.arg[8] = 0x05;
	cmd.arg[9] = 0x02;
	cmd.arg[10] = 0x02;
	cmd.arg[11] = 0x00;
	
	if ( cx24120_message_send(state, &cmd) ) {
		err("Error set bandwidth! :(\n");
		return -EREMOTEIO;
	}
	if ( cx24120_readreg(state, 0xBA) > 3) {
		err("Error intitilizing tuner! :(\n");
		return -EREMOTEIO;
	}
	dbginfo("Tuner initialized correctly.\n");

	ret = cx24120_writereg(state, 0xEB, 0x0A);
	if (cx24120_msg_mpeg_output_global_config(state, 0) ||
		cx24120_msg_mpeg_output_config(state, 0, &initial_mpeg_config) ||
		cx24120_msg_mpeg_output_config(state, 1, &initial_mpeg_config) ||
		cx24120_msg_mpeg_output_config(state, 2, &initial_mpeg_config) )
	{
		err("Error initilizing mpeg output. :(\n");
        return -EREMOTEIO;
	} else {
		cmd.id = 0x3C;	// 60
		cmd.len = 0x03;
		cmd.arg[0] = 0x00;
		cmd.arg[1] = 0x10;
		cmd.arg[2] = 0x10;
		if(cx24120_message_send(state, &cmd)) {
			err("Error sending final init message. :(\n");
			return -EREMOTEIO;
		}	
    }
	state->cold_init=1;
	return 0;
} 
EXPORT_SYMBOL(cx24120_init);		// end cx24120_reset
//===================================================================
static int cx24120_tune(struct dvb_frontend *fe, struct dvb_frontend_parameters *p,
	unsigned int mode_flags, unsigned int *delay, fe_status_t *status)
{
	struct cx24120_state *state = fe->demodulator_priv;
	int delay_cnt, sd_idx = 0;
	
	if ( p != NULL ) {
		
//		dbginfo("Compare symrate with table: symrate= %d, in table= %d\n", 
//				p->u.qpsk.symbol_rate, symrates_pairs[sd_idx].symrate);

		while ( p->u.qpsk.symbol_rate > symrates_pairs[sd_idx].symrate ) {
			++sd_idx;
		}
		dbginfo("Found symrate delay = %d\n", symrates_pairs[sd_idx].delay);
		state->dvb_s2_mode &= 0xFE;			// clear bit -> try not DVB-S2
		dbginfo("trying DVB-S now\n");
		cx24120_set_frontend(fe, p);
		
		delay_cnt = symrates_pairs[sd_idx].delay;
		dbginfo("Wait for LOCK=================\n");
		while (delay_cnt >= 0) {
			cx24120_read_status(fe, status);
			if (*status & FE_HAS_LOCK) break;
			msleep(100);
			delay_cnt -=100;
		}
		dbginfo("Waiting finished=================\n");
		
		cx24120_read_status(fe, status);
		if ( !(*status & FE_HAS_LOCK) ) {		// if no lock on S
			dbginfo("trying DVB-S2 now\n");
			state->dvb_s2_mode |= 0x01;			// may be it locked on S2 ?
			p->u.qpsk.fec_inner = FEC_AUTO;
			cx24120_set_frontend(fe, p);
		}
	} else 
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(cx24120_tune);	// end of cx24120_tune
//===================================================================
static int cx24120_get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}
EXPORT_SYMBOL(cx24120_get_algo);
//===================================================================
static int cx24120_sleep(struct dvb_frontend *fe)
{
  return 0;
}
EXPORT_SYMBOL(cx24120_sleep);
//===================================================================
/*static int cx24120_wakeup(struct dvb_frontend *fe)
 * {
 *   return 0;
 * }
 * EXPORT_SYMBOL(cx24120_wakeup);
 */
//===================================================================
static int cx24120_get_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters* params)
{
	return 0;
}
EXPORT_SYMBOL(cx24120_get_frontend);
//===================================================================
static void cx24120_release(struct dvb_frontend *fe)
{
	struct cx24120_state *state = fe->demodulator_priv;
	dbginfo("Clear state structure\n");
	kfree(state);
}
EXPORT_SYMBOL(cx24120_release);
//===================================================================
static int cx24120_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)	// UNCORRECTED_BLOCKS
{
	struct cx24120_state *state = fe->demodulator_priv;

	*ucblocks = (cx24120_readreg(state, CX24120_REG_UCB_H) << 8) |
		cx24120_readreg(state, CX24120_REG_UCB_L);
	dbginfo("Blocks = %d\n", *ucblocks);
	return 0;
}
EXPORT_SYMBOL(cx24120_read_ucblocks);
// ########################################################################################
static struct dvb_frontend_ops cx24120_ops = {

	.info = {
		.name = "Conexant CX24120/CX24118",
		.type = FE_QPSK,
		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_stepsize = 1011, /* kHz for QPSK frontends */
		.frequency_tolerance = 5000,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = 						// 0x500006ff
			FE_CAN_INVERSION_AUTO |		//0x00 000 001
			FE_CAN_FEC_1_2 | 			//0x00 000 002
			FE_CAN_FEC_2_3 | 			//0x00 000 004
			FE_CAN_FEC_3_4 |			//0x00 000 008
			FE_CAN_FEC_4_5 | 			//0x00 000 010
			FE_CAN_FEC_5_6 | 			//0x00 000 020
			FE_CAN_FEC_6_7 |			//0x00 000 040
			FE_CAN_FEC_7_8 | 			//0x00 000 080
			FE_CAN_FEC_AUTO |			//0x00 000 200
			FE_CAN_QPSK | 				//0x00 000 400
//???		FE_HAS_EXTENDED_CAPS |		//0x00 800 000   	/* We need more bitspace for newer APIs, indicate this. */
			FE_CAN_2G_MODULATION |		//0x10 000 000   	/* frontend supports "2nd generation modulation" (DVB-S2) */
			FE_CAN_RECOVER				//0x40 000 000		/* frontend can recover from a cable unplug automatically */
	},									//sum=50 000 6FF
	.release = 					cx24120_release,
	
	.init = 					cx24120_init, 
	.sleep = 					cx24120_sleep,
	
	.tune = 					cx24120_tune,
	.get_frontend_algo = 		cx24120_get_algo,
	.set_frontend = 			cx24120_set_frontend,

	.get_frontend = 			cx24120_get_frontend,
	.read_status = 				cx24120_read_status,
	.read_ber = 				cx24120_read_ber,
	.read_signal_strength = 	cx24120_read_signal_strength,
	.read_snr = 				cx24120_read_snr,
	.read_ucblocks = 			cx24120_read_ucblocks,
	
	.diseqc_send_master_cmd = 	cx24120_send_diseqc_msg,
	
	.diseqc_send_burst = 		cx24120_diseqc_send_burst,
	.set_tone = 				cx24120_set_tone,
	.set_voltage = 				cx24120_set_voltage,
};
//===================================================================
MODULE_PARM_DESC(cx24120_debug, "prints some verbose debugging information (default:0)");
MODULE_AUTHOR("Custler");
MODULE_LICENSE("GPL");
