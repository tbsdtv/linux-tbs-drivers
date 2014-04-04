/*
 * Conexant CX24120/CX24118 - DVB-S/S2 demod/tuner driver
 * DVBS/S2 Satellite demod/tuner driver static definitins
 *
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

#define CX24120_FIRMWARE "dvb-fe-cx24120-1.20.58.2.fw"

// ##############################
// ### cx24120 i2c registers ###
#define CX24120_REG_CMD_START (0x00)	// write cmd_id, and then start write args to next register:
#define CX24120_REG_CMD_ARGS (0x01)		// write command arguments, max 4 at once, then next 4, etc.
#define CX24120_REG_CMD_END (0x1F)		// write 0x01 for end, and read it for command result

#define CX24120_REG_FECMODE (0x39)		// FEC status
#define CX24120_REG_STATUS (0x3A)		// Tuner status - signal, carrier, sync, lock ...
#define CX24120_REG_QUALITY_H (0x40)	// SNR high byte
#define CX24120_REG_QUALITY_L (0x41)	// SNR low byte

#define CX24120_REG_BER_HH (0x47)		// BER high byte of high word
#define CX24120_REG_BER_HL (0x48)		// BER low byte of high word
#define CX24120_REG_BER_LH (0x49)		// BER high byte of low word
#define CX24120_REG_BER_LL (0x4A)		// BER low byte of low word

#define CX24120_REG_SIGSTR_H (0x3A)		// Signal strength high byte & ??? status register ???
#define CX24120_REG_SIGSTR_L (0x3B)		// Signal strength low byte

#define CX24120_REG_UCB_H (0x50)		// UCB high byte
#define CX24120_REG_UCB_L (0x51)		// UCB low byte

#define CX24120_REG_REVISION (0xFF)		// Chip revision (ro). Must be 0x7 or 0x5

// ##############################
/* Command messages */
enum command_message_id {
	CMD_VCO_SET			= 0x10,		// cmdlen = 12;
	CMD_TUNEREQUEST		= 0x11,		// cmd.len = 15;
	
	CMD_MPEG_ONOFF		= 0x13,		// cmd.len = 4;
	CMD_MPEG_INIT		= 0x14,		// cmd.len = 7;
	CMD_BANDWIDTH		= 0x15,		// cmd.len = 12;
	CMD_CLOCK_READ		= 0x16,		// read clock from registers 0x01-0x06
	CMD_CLOCK_SET		= 0x17,		// cmd.len = 10;
	
	CMD_DISEQC_MSG1		= 0x20,		// cmd.len = 11;
	CMD_DISEQC_MSG2		= 0x21,		// cmd.len = d->msg_len + 6;
	CMD_SETVOLTAGE		= 0x22,		// cmd.len = 2;
	CMD_SETTONE			= 0x23,		// cmd.len = 4;
	CMD_DISEQC_BURST	= 0x24,		// cmd.len not used !!!
	
	CMD_READ_SNR		= 0x1A,		// Read signal strength
	CMD_START_TUNER		= 0x1B,		// ???
	
	CMD_TUNER_INIT		= 0x3C,		// cmd.len = 0x03;
};
// ##############################
/* signal status */
#define CX24120_HAS_SIGNAL  (0x01)
#define CX24120_HAS_CARRIER (0x02)
#define CX24120_HAS_VITERBI	(0x04)
#define CX24120_HAS_LOCK 	(0x08)
#define CX24120_HAS_UNK1 	(0x10)
#define CX24120_HAS_UNK2 	(0x20)
#define CX24120_STATUS_MASK (0x0f)
#define CX24120_SIGNAL_MASK (0xc0)

static u8 cx24120_msg_tuner_init[] = { 0,0,0,0,0,0 };
static u8 cx24120_msg_read_sigstr[] = {0,0};

static struct cx24120_skystar2_mpeg_config {
	u8 x1;
	u8 x2;
	u8 x3;
} initial_mpeg_config = {	
	0xA1, 	// 10100001
	0x76, 	// 01110110
	0x07,	// 00000111
};

static struct cx24120_symrate_delay {
	u32 symrate;
	u32 delay;
} symrates_pairs[] = {
	{    3000000, 15000 },
	{    6000000, 10000 },
	{    8000000,  5000 },
	{   10000000,  2000 },
	{0x0FFFFFFFF,   400 },
};

static struct cx24120_clock_ratios_table {
	u32 ratio_id;
	u32 mode_xPSK;
	u32 fec_mode;
	u32	m_rat;
	u32 n_rat;
	u32 rate;
} clock_ratios_table[] = {
{	21	,	0	,	1	,	770068	,	763515	,	258	},
{	21	,	0	,	100	,	97409	,	80370	,	310	},
{	21	,	0	,	2	,	137293	,	101802	,	345	},
{	21	,	0	,	3	,	4633447	,	3054060	,	388	},
{	21	,	0	,	4	,	2472041	,	1527030	,	414	},
{	21	,	0	,	5	,	85904	,	50901	,	432	},
{	21	,	0	,	8	,	2751229	,	1527030	,	461	},
{	21	,	0	,	101	,	1392872	,	763515	,	467	},
{	21	,	99	,	100	,	1850771	,	1019430	,	464	},
{	21	,	99	,	2	,	137293	,	67962	,	517	},
{	21	,	99	,	3	,	4633447	,	2038860	,	581	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	21	,	99	,	5	,	85904	,	33981	,	647	},
{	21	,	99	,	8	,	2751229	,	1019430	,	690	},
{	21	,	99	,	101	,	1392872	,	509715	,	699	},
{	29	,	0	,	1	,	770068	,	782127	,	252	},
{	29	,	0	,	100	,	1850771	,	1564254	,	302	},
{	29	,	0	,	2	,	686465	,	521418	,	337	},
{	29	,	0	,	3	,	4633447	,	3128508	,	379	},
{	29	,	0	,	4	,	2472041	,	1564254	,	404	},
{	29	,	0	,	5	,	429520	,	260709	,	421	},
{	29	,	0	,	8	,	2751229	,	1564254	,	450	},
{	29	,	0	,	101	,	1392872	,	782127	,	455	},
{	29	,	99	,	100	,	1850771	,	1043118	,	454	},
{	29	,	99	,	2	,	686465	,	347706	,	505	},
{	29	,	99	,	3	,	4633447	,	2086236	,	568	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	29	,	99	,	5	,	429520	,	173853	,	632	},
{	29	,	99	,	8	,	2751229	,	1043118	,	675	},
{	29	,	99	,	101	,	1392872	,	521559	,	683	},
{	17	,	0	,	1	,	766052	,	763515	,	256	},
{	17	,	0	,	100	,	96901	,	80370	,	308	},
{	17	,	0	,	2	,	136577	,	101802	,	343	},
{	17	,	0	,	3	,	4609283	,	3054060	,	386	},
{	17	,	0	,	4	,	2459149	,	1527030	,	412	},
{	17	,	0	,	5	,	85456	,	50901	,	429	},
{	17	,	0	,	8	,	2736881	,	1527030	,	458	},
{	17	,	0	,	101	,	1385608	,	763515	,	464	},
{	17	,	99	,	100	,	1841119	,	1019430	,	462	},
{	17	,	99	,	2	,	136577	,	67962	,	514	},
{	17	,	99	,	3	,	4609283	,	2038860	,	578	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	17	,	99	,	5	,	85456	,	33981	,	643	},
{	17	,	99	,	8	,	2736881	,	1019430	,	687	},
{	17	,	99	,	101	,	1385608	,	509715	,	695	},
{	25	,	0	,	1	,	766052	,	782127	,	250	},
{	25	,	0	,	100	,	1841119	,	1564254	,	301	},
{	25	,	0	,	2	,	682885	,	521418	,	335	},
{	25	,	0	,	3	,	4609283	,	3128508	,	377	},
{	25	,	0	,	4	,	2459149	,	1564254	,	402	},
{	25	,	0	,	5	,	427280	,	260709	,	419	},
{	25	,	0	,	8	,	2736881	,	1564254	,	447	},
{	25	,	0	,	101	,	1385608	,	782127	,	453	},
{	25	,	99	,	100	,	1841119	,	1043118	,	451	},
{	25	,	99	,	2	,	682885	,	347706	,	502	},
{	25	,	99	,	3	,	4609283	,	2086236	,	565	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	25	,	99	,	5	,	427280	,	173853	,	629	},
{	25	,	99	,	8	,	2736881	,	1043118	,	671	},
{	25	,	99	,	101	,	1385608	,	521559	,	680	},
{	5	,	0	,	1	,	273088	,	254505	,	274	},
{	5	,	0	,	100	,	17272	,	13395	,	330	},
{	5	,	0	,	2	,	24344	,	16967	,	367	},
{	5	,	0	,	3	,	410788	,	254505	,	413	},
{	5	,	0	,	4	,	438328	,	254505	,	440	},
{	5	,	0	,	5	,	30464	,	16967	,	459	},
{	5	,	0	,	8	,	487832	,	254505	,	490	},
{	5	,	0	,	101	,	493952	,	254505	,	496	},
{	5	,	99	,	100	,	328168	,	169905	,	494	},
{	5	,	99	,	2	,	24344	,	11327	,	550	},
{	5	,	99	,	3	,	410788	,	169905	,	618	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	5	,	99	,	5	,	30464	,	11327	,	688	},
{	5	,	99	,	8	,	487832	,	169905	,	735	},
{	5	,	99	,	101	,	493952	,	169905	,	744	},
{	13	,	0	,	1	,	273088	,	260709	,	268	},
{	13	,	0	,	100	,	328168	,	260709	,	322	},
{	13	,	0	,	2	,	121720	,	86903	,	358	},
{	13	,	0	,	3	,	410788	,	260709	,	403	},
{	13	,	0	,	4	,	438328	,	260709	,	430	},
{	13	,	0	,	5	,	152320	,	86903	,	448	},
{	13	,	0	,	8	,	487832	,	260709	,	479	},
{	13	,	0	,	101	,	493952	,	260709	,	485	},
{	13	,	99	,	100	,	328168	,	173853	,	483	},
{	13	,	99	,	2	,	121720	,	57951	,	537	},
{	13	,	99	,	3	,	410788	,	173853	,	604	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	13	,	99	,	5	,	152320	,	57951	,	672	},
{	13	,	99	,	8	,	487832	,	173853	,	718	},
{	13	,	99	,	101	,	493952	,	173853	,	727	},
{	1	,	0	,	1	,	815248	,	763515	,	273	},
{	1	,	0	,	100	,	51562	,	40185	,	328	},
{	1	,	0	,	2	,	72674	,	50901	,	365	},
{	1	,	0	,	3	,	1226323	,	763515	,	411	},
{	1	,	0	,	4	,	1308538	,	763515	,	438	},
{	1	,	0	,	5	,	90944	,	50901	,	457	},
{	1	,	0	,	8	,	1456322	,	763515	,	488	},
{	1	,	0	,	101	,	1474592	,	763515	,	494	},
{	1	,	99	,	100	,	979678	,	509715	,	492	},
{	1	,	99	,	2	,	72674	,	33981	,	547	},
{	1	,	99	,	3	,	1226323	,	509715	,	615	},	// was 4 - ERRORR!? FEC_4_5 not in DVB-S2
{	1	,	99	,	5	,	90944	,	33981	,	685	},
{	1	,	99	,	8	,	1456322	,	509715	,	731	},
{	1	,	99	,	101	,	1474592	,	509715	,	740	},
{	9	,	0	,	1	,	815248	,	782127	,	266	},
{	9	,	0	,	100	,	979678	,	782127	,	320	},
{	9	,	0	,	2	,	363370	,	260709	,	356	},
{	9	,	0	,	3	,	1226323	,	782127	,	401	},
{	9	,	0	,	4	,	1308538	,	782127	,	428	},
{	9	,	0	,	5	,	454720	,	260709	,	446	},
{	9	,	0	,	8	,	1456322	,	782127	,	476	},
{	9	,	0	,	101	,	1474592	,	782127	,	482	},
{	9	,	99	,	100	,	979678	,	521559	,	480	},
{	9	,	99	,	2	,	363370	,	173853	,	535	},
{	9	,	99	,	3	,	1226323	,	521559	,	601	},	// was 4 - ERRORR! FEC_4_5 not in DVB-S2
{	9	,	99	,	5	,	454720	,	173853	,	669	},
{	9	,	99	,	8	,	1456322	,	521559	,	714	},
{	9	,	99	,	101	,	1474592	,	521559	,	723	},
{	18	,	0	,	1	,	535	,	588	,	233	},
{	18	,	0	,	2	,	1070	,	882	,	311	},
{	18	,	0	,	6	,	3210	,	2058	,	399	},
{	16	,	0	,	1	,	763	,	816	,	239	},
{	16	,	0	,	2	,	1526	,	1224	,	319	},
{	16	,	0	,	3	,	2289	,	1632	,	359	},
{	16	,	0	,	5	,	3815	,	2448	,	399	},
{	16	,	0	,	7	,	5341	,	3264	,	419	},
{	22	,	0	,	1	,	535	,	588	,	233	},
{	22	,	0	,	2	,	1070	,	882	,	311	},
{	22	,	0	,	6	,	3210	,	2058	,	399	},
{	20	,	0	,	1	,	143429	,	152592	,	241	},
{	20	,	0	,	2	,	286858	,	228888	,	321	},
{	20	,	0	,	3	,	430287	,	305184	,	361	},
{	20	,	0	,	5	,	717145	,	457776	,	401	},
{	20	,	0	,	7	,	1004003	,	610368	,	421	},
{	2	,	0	,	1	,	584	,	588	,	254	},
{	2	,	0	,	2	,	1169	,	882	,	339	},
{	2	,	0	,	6	,	3507	,	2058	,	436	},
{	0	,	0	,	1	,	812	,	816	,	255	},
{	0	,	0	,	2	,	1624	,	1224	,	340	},
{	0	,	0	,	3	,	2436	,	1632	,	382	},
{	0	,	0	,	5	,	4060	,	2448	,	425	},
{	0	,	0	,	7	,	5684	,	3264	,	446	},
{	6	,	0	,	1	,	584	,	588	,	254	},
{	6	,	0	,	2	,	1168	,	882	,	339	},
{	6	,	0	,	6	,	3504	,	2058	,	436	},
{	4	,	0	,	1	,	152592	,	152592	,	256	},
{	4	,	0	,	2	,	305184	,	228888	,	341	},
{	4	,	0	,	3	,	457776	,	305184	,	384	},
{	4	,	0	,	5	,	762960	,	457776	,	427	},
{	4	,	0	,	7	,	1068144	,	610368	,	448	},
};
