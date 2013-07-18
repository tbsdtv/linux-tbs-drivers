/*
 * dvb_demux.c - DVB kernel demux API
 *
 * Copyright (C) 2000-2001 Ralph  Metzler <ralph@convergence.de>
 *		       & Marcus Metzler <marcus@convergence.de>
 *			 for convergence integrated media GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include <linux/version.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/crc32.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#include <linux/dvb/frontend.h>

#include "dvb_demux.h"

#define NOBUFS
/*
** #define DVB_DEMUX_SECTION_LOSS_LOG to monitor payload loss in the syslog
*/
// #define DVB_DEMUX_SECTION_LOSS_LOG

/**
 * @brief Use table-driven CRC-8 calculation for Base-band demultiplexer
 */
#define BB_CRC8_TABLE_DRIVEN	1

/**
 * Define this to enable verbose debugging information from the Base-band demux
 */
//#define DVB_DEMUX_DEBUG_BB

static int dvb_demux_tscheck;
module_param(dvb_demux_tscheck, int, 0644);
MODULE_PARM_DESC(dvb_demux_tscheck,
		"enable transport stream continuity and TEI check");

static int dvb_demux_speedcheck;
module_param(dvb_demux_speedcheck, int, 0644);
MODULE_PARM_DESC(dvb_demux_speedcheck,
		"enable transport stream speed check");

#define dprintk_tscheck(x...) do {                              \
		if (dvb_demux_tscheck && printk_ratelimit())    \
			printk(x);                              \
	} while (0)

#ifdef BB_CRC8_TABLE_DRIVEN

/* CRC-8 table for generator polynom 0xd5
 * Created with PYCRC (http://www.tty1.net/pycrc/) using the following commandline:
 * python pycrc.py --generate table -o /tmp/bb-crc8-table.c --width=8 --poly=0xd5 
 *	--reflect-in=false --reflect-out=false --xor-in=0xff --xor-out=0x00
 */
static const u8 bb_crc8_table[256] = {
	0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
	0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
	0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
	0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
	0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
	0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
	0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
	0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
	0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
	0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
	0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
	0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
	0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
	0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
	0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
	0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

#endif 


/******************************************************************************
 * static inlined helper functions
 ******************************************************************************/

static inline u16 section_length(const u8 *buf)
{
	return 3 + ((buf[1] & 0x0f) << 8) + buf[2];
}

static inline u16 ts_pid(const u8 *buf)
{
	return ((buf[1] & 0x1f) << 8) + buf[2];
}

static inline u8 payload(const u8 *tsp)
{
	if (!(tsp[3] & 0x10))	// no payload?
		return 0;

	if (tsp[3] & 0x20) {	// adaptation field?
		if (tsp[4] > 183)	// corrupted data?
			return 0;
		else
			return 184 - 1 - tsp[4];
	}

	return 184;
}

static u32 dvb_dmx_crc32(struct dvb_demux_feed *f, const u8 *src, size_t len)
{
	return (f->feed.sec.crc_val = crc32_be(f->feed.sec.crc_val, src, len));
}

static void dvb_dmx_memcopy(struct dvb_demux_feed *f, u8 *d, const u8 *s,
			    size_t len)
{
	memcpy(d, s, len);
}

#ifndef BB_CRC8_TABLE_DRIVEN

// CRC8 bit-by-bit

#define BB_CRC8_POLY    0xd5
static u8 bb_crc8_single(u8 crc, u8 data)
{
        int i;
	for (i = 0x80; i > 0; i >>= 1) {
		bit = crc & 0x80;
		if (data & i) {
			bit = !bit;
		}
		crc <<= 1;
		if (bit) {
			crc ^= BB_CRC8_POLY;
		}
	}
        return crc;
}

#endif

/**
 * @brief CRC-8 for DVB-S2 Base-band demultiplexer
 */
static u8 bb_crc8(u8 crc, const u8 *src, size_t len)
{
	const unsigned char *end = src + len;
	while(src < end)
#ifdef BB_CRC8_TABLE_DRIVEN
		crc = bb_crc8_table[crc ^ *src++];
#else
		crc = bb_crc8_single(crc, *src++);
#endif
        return crc;
}

/******************************************************************************
 * Software filter functions
 ******************************************************************************/

static inline int dvb_dmx_swfilter_payload(struct dvb_demux_feed *feed,
					   const u8 *buf)
{
	int count = payload(buf);
	int p;
	//int ccok;
	//u8 cc;

	if (count == 0)
		return -1;

	p = 188 - count;

	/*
	cc = buf[3] & 0x0f;
	ccok = ((feed->cc + 1) & 0x0f) == cc;
	feed->cc = cc;
	if (!ccok)
		printk("missed packet!\n");
	*/

	if (buf[1] & 0x40)	// PUSI ?
		feed->peslen = 0xfffa;

	feed->peslen += count;

	return feed->cb.ts(&buf[p], count, NULL, 0, &feed->feed.ts, DMX_OK);
}

static int dvb_dmx_swfilter_sectionfilter(struct dvb_demux_feed *feed,
					  struct dvb_demux_filter *f)
{
	u8 neq = 0;
	int i;

	for (i = 0; i < DVB_DEMUX_MASK_MAX; i++) {
		u8 xor = f->filter.filter_value[i] ^ feed->feed.sec.secbuf[i];

		if (f->maskandmode[i] & xor)
			return 0;

		neq |= f->maskandnotmode[i] & xor;
	}

	if (f->doneq && !neq)
		return 0;

	return feed->cb.sec(feed->feed.sec.secbuf, feed->feed.sec.seclen,
			    NULL, 0, &f->filter, DMX_OK);
}

static inline int dvb_dmx_swfilter_section_feed(struct dvb_demux_feed *feed)
{
	struct dvb_demux *demux = feed->demux;
	struct dvb_demux_filter *f = feed->filter;
	struct dmx_section_feed *sec = &feed->feed.sec;
	int section_syntax_indicator;

	if (!sec->is_filtering)
		return 0;

	if (!f)
		return 0;

	if (sec->check_crc) {
		section_syntax_indicator = ((sec->secbuf[1] & 0x80) != 0);
		if (section_syntax_indicator &&
		    demux->check_crc32(feed, sec->secbuf, sec->seclen))
			return -1;
	}

	do {
		if (dvb_dmx_swfilter_sectionfilter(feed, f) < 0)
			return -1;
	} while ((f = f->next) && sec->is_filtering);

	sec->seclen = 0;

	return 0;
}

static void dvb_dmx_swfilter_section_new(struct dvb_demux_feed *feed)
{
	struct dmx_section_feed *sec = &feed->feed.sec;

#ifdef DVB_DEMUX_SECTION_LOSS_LOG
	if (sec->secbufp < sec->tsfeedp) {
		int i, n = sec->tsfeedp - sec->secbufp;

		/*
		 * Section padding is done with 0xff bytes entirely.
		 * Due to speed reasons, we won't check all of them
		 * but just first and last.
		 */
		if (sec->secbuf[0] != 0xff || sec->secbuf[n - 1] != 0xff) {
			printk("dvb_demux.c section ts padding loss: %d/%d\n",
			       n, sec->tsfeedp);
			printk("dvb_demux.c pad data:");
			for (i = 0; i < n; i++)
				printk(" %02x", sec->secbuf[i]);
			printk("\n");
		}
	}
#endif

	sec->tsfeedp = sec->secbufp = sec->seclen = 0;
	sec->secbuf = sec->secbuf_base;
}

/*
 * Losless Section Demux 1.4.1 by Emard
 * Valsecchi Patrick:
 *  - middle of section A  (no PUSI)
 *  - end of section A and start of section B
 *    (with PUSI pointing to the start of the second section)
 *
 *  In this case, without feed->pusi_seen you'll receive a garbage section
 *  consisting of the end of section A. Basically because tsfeedp
 *  is incemented and the use=0 condition is not raised
 *  when the second packet arrives.
 *
 * Fix:
 * when demux is started, let feed->pusi_seen = 0 to
 * prevent initial feeding of garbage from the end of
 * previous section. When you for the first time see PUSI=1
 * then set feed->pusi_seen = 1
 */
static int dvb_dmx_swfilter_section_copy_dump(struct dvb_demux_feed *feed,
					      const u8 *buf, u8 len)
{
	struct dvb_demux *demux = feed->demux;
	struct dmx_section_feed *sec = &feed->feed.sec;
	u16 limit, seclen, n;

	if (sec->tsfeedp >= DMX_MAX_SECFEED_SIZE)
		return 0;

	if (sec->tsfeedp + len > DMX_MAX_SECFEED_SIZE) {
#ifdef DVB_DEMUX_SECTION_LOSS_LOG
		printk("dvb_demux.c section buffer full loss: %d/%d\n",
		       sec->tsfeedp + len - DMX_MAX_SECFEED_SIZE,
		       DMX_MAX_SECFEED_SIZE);
#endif
		len = DMX_MAX_SECFEED_SIZE - sec->tsfeedp;
	}

	if (len <= 0)
		return 0;

	demux->memcopy(feed, sec->secbuf_base + sec->tsfeedp, buf, len);
	sec->tsfeedp += len;

	/*
	 * Dump all the sections we can find in the data (Emard)
	 */
	limit = sec->tsfeedp;
	if (limit > DMX_MAX_SECFEED_SIZE)
		return -1;	/* internal error should never happen */

	/* to be sure always set secbuf */
	sec->secbuf = sec->secbuf_base + sec->secbufp;

	for (n = 0; sec->secbufp + 2 < limit; n++) {
		seclen = section_length(sec->secbuf);
		if (seclen <= 0 || seclen > DMX_MAX_SECTION_SIZE
		    || seclen + sec->secbufp > limit)
			return 0;
		sec->seclen = seclen;
		sec->crc_val = ~0;
		/* dump [secbuf .. secbuf+seclen) */
		if (feed->pusi_seen)
			dvb_dmx_swfilter_section_feed(feed);
#ifdef DVB_DEMUX_SECTION_LOSS_LOG
		else
			printk("dvb_demux.c pusi not seen, discarding section data\n");
#endif
		sec->secbufp += seclen;	/* secbufp and secbuf moving together is */
		sec->secbuf += seclen;	/* redundant but saves pointer arithmetic */
	}

	return 0;
}

static int dvb_dmx_swfilter_section_packet(struct dvb_demux_feed *feed,
					   const u8 *buf)
{
	u8 p, count;
	int ccok, dc_i = 0;
	u8 cc;

	count = payload(buf);

	if (count == 0)		/* count == 0 if no payload or out of range */
		return -1;

	p = 188 - count;	/* payload start */

	cc = buf[3] & 0x0f;
	ccok = ((feed->cc + 1) & 0x0f) == cc;
	feed->cc = cc;

	if (buf[3] & 0x20) {
		/* adaption field present, check for discontinuity_indicator */
		if ((buf[4] > 0) && (buf[5] & 0x80))
			dc_i = 1;
	}

	if (!ccok || dc_i) {
#ifdef DVB_DEMUX_SECTION_LOSS_LOG
		printk("dvb_demux.c discontinuity detected %d bytes lost\n",
		       count);
		/*
		 * those bytes under sume circumstances will again be reported
		 * in the following dvb_dmx_swfilter_section_new
		 */
#endif
		/*
		 * Discontinuity detected. Reset pusi_seen = 0 to
		 * stop feeding of suspicious data until next PUSI=1 arrives
		 */
		feed->pusi_seen = 0;
		dvb_dmx_swfilter_section_new(feed);
	}

	if (buf[1] & 0x40) {
		/* PUSI=1 (is set), section boundary is here */
		if (count > 1 && buf[p] < count) {
			const u8 *before = &buf[p + 1];
			u8 before_len = buf[p];
			const u8 *after = &before[before_len];
			u8 after_len = count - 1 - before_len;

			dvb_dmx_swfilter_section_copy_dump(feed, before,
							   before_len);
			/* before start of new section, set pusi_seen = 1 */
			feed->pusi_seen = 1;
			dvb_dmx_swfilter_section_new(feed);
			dvb_dmx_swfilter_section_copy_dump(feed, after,
							   after_len);
		}
#ifdef DVB_DEMUX_SECTION_LOSS_LOG
		else if (count > 0)
			printk("dvb_demux.c PUSI=1 but %d bytes lost\n", count);
#endif
	} else {
		/* PUSI=0 (is not set), no section boundary */
		dvb_dmx_swfilter_section_copy_dump(feed, &buf[p], count);
	}

	return 0;
}

static inline void dvb_dmx_swfilter_packet_type(struct dvb_demux_feed *feed,
						const u8 *buf)
{
	switch (feed->type) {
	case DMX_TYPE_TS:
		if (!feed->feed.ts.is_filtering)
			break;
		if (feed->ts_type & TS_PACKET) {
			if (feed->ts_type & TS_PAYLOAD_ONLY)
				dvb_dmx_swfilter_payload(feed, buf);
			else
				feed->cb.ts(buf, 188, NULL, 0, &feed->feed.ts,
					    DMX_OK);
		}
		if (feed->ts_type & TS_DECODER)
			if (feed->demux->write_to_decoder)
				feed->demux->write_to_decoder(feed, buf, 188);
		break;

	case DMX_TYPE_SEC:
		if (!feed->feed.sec.is_filtering)
			break;
		if (dvb_dmx_swfilter_section_packet(feed, buf) < 0)
			feed->feed.sec.seclen = feed->feed.sec.secbufp = 0;
		break;

	default:
		break;
	}
}

#define DVR_FEED(f)							\
	(((f)->type == DMX_TYPE_TS) &&					\
	((f)->feed.ts.is_filtering) &&					\
	(((f)->ts_type & (TS_PACKET | TS_DEMUX)) == TS_PACKET))

static void dvb_dmx_swfilter_packet(struct dvb_demux *demux, const u8 *buf)
{
	struct dvb_demux_feed *feed;
	u16 pid = ts_pid(buf);
	int dvr_done = 0;

	if (dvb_demux_speedcheck) {
		struct timespec cur_time, delta_time;
		u64 speed_bytes, speed_timedelta;

		demux->speed_pkts_cnt++;

		/* show speed every SPEED_PKTS_INTERVAL packets */
		if (!(demux->speed_pkts_cnt % SPEED_PKTS_INTERVAL)) {
			cur_time = current_kernel_time();

			if (demux->speed_last_time.tv_sec != 0 &&
					demux->speed_last_time.tv_nsec != 0) {
				delta_time = timespec_sub(cur_time,
						demux->speed_last_time);
				speed_bytes = (u64)demux->speed_pkts_cnt
					* 188 * 8;
				/* convert to 1024 basis */
				speed_bytes = 1000 * div64_u64(speed_bytes,
						1024);
				speed_timedelta =
					(u64)timespec_to_ns(&delta_time);
				speed_timedelta = div64_u64(speed_timedelta,
						1000000); /* nsec -> usec */
				printk(KERN_INFO "TS speed %llu Kbits/sec \n",
						div64_u64(speed_bytes,
							speed_timedelta));
			};

			demux->speed_last_time = cur_time;
			demux->speed_pkts_cnt = 0;
		};
	};

	if (demux->cnt_storage && dvb_demux_tscheck) {
		/* check pkt counter */
		if (pid < MAX_PID) {
			if (buf[1] & 0x80)
				dprintk_tscheck("TEI detected. "
						"PID=0x%x data1=0x%x\n",
						pid, buf[1]);

			if ((buf[3] & 0xf) != demux->cnt_storage[pid])
				dprintk_tscheck("TS packet counter mismatch. "
						"PID=0x%x expected 0x%x "
						"got 0x%x\n",
						pid, demux->cnt_storage[pid],
						buf[3] & 0xf);

			demux->cnt_storage[pid] = ((buf[3] & 0xf) + 1)&0xf;
		};
		/* end check */
	};

	list_for_each_entry(feed, &demux->feed_list, list_head) {
		if (feed->type != DMX_TYPE_TS || (
			(feed->pid != pid) && (feed->pid != 0x2000)))
			continue;

		/* copy each packet only once to the dvr device, even
		 * if a PID is in multiple filters (e.g. video + PCR) */
		if ((DVR_FEED(feed)) && (dvr_done++))
			continue;

		if (feed->pid == pid) {
			dvb_dmx_swfilter_packet_type(feed, buf);
			if (DVR_FEED(feed))
				continue;
		}
		else if (feed->pid == 0x2000)
			feed->cb.ts(buf, 188, NULL, 0, &feed->feed.ts, DMX_OK);
	}
}

void dvb_dmx_swfilter_packets(struct dvb_demux *demux, const u8 *buf,
			      size_t count)
{
	spin_lock(&demux->lock);

	while (count--) {
		if (buf[0] == 0x47)
			dvb_dmx_swfilter_packet(demux, buf);
		buf += 188;
	}

	spin_unlock(&demux->lock);
}

EXPORT_SYMBOL(dvb_dmx_swfilter_packets);

static inline int find_next_packet(const u8 *buf, int pos, size_t count,
				   const int pktsize)
{
	int start = pos, lost;

	while (pos < count) {
		if (buf[pos] == 0x47 ||
		    (pktsize == 204 && buf[pos] == 0xB8))
			break;
		pos++;
	}

	lost = pos - start;
	if (lost) {
		/* This garbage is part of a valid packet? */
		int backtrack = pos - pktsize;
		if (backtrack >= 0 && (buf[backtrack] == 0x47 ||
		    (pktsize == 204 && buf[backtrack] == 0xB8)))
			return backtrack;
	}

	return pos;
}

/* Filter all pktsize= 188 or 204 sized packets and skip garbage. */
static inline void _dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf,
		size_t count, const int pktsize)
{
	int p = 0, i, j;
	const u8 *q;

	spin_lock(&demux->lock);

	if (demux->tsbufp) { /* tsbuf[0] is now 0x47. */
		i = demux->tsbufp;
		j = pktsize - i;
		if (count < j) {
			memcpy(&demux->tsbuf[i], buf, count);
			demux->tsbufp += count;
			goto bailout;
		}
		memcpy(&demux->tsbuf[i], buf, j);
		if (demux->tsbuf[0] == 0x47) /* double check */
			dvb_dmx_swfilter_packet(demux, demux->tsbuf);
		demux->tsbufp = 0;
		p += j;
	}

	while (1) {
		p = find_next_packet(buf, p, count, pktsize);
		if (p >= count)
			break;
		if (count - p < pktsize)
			break;

		q = &buf[p];

		if (pktsize == 204 && (*q == 0xB8)) {
			memcpy(demux->tsbuf, q, 188);
			demux->tsbuf[0] = 0x47;
			q = demux->tsbuf;
		}
		dvb_dmx_swfilter_packet(demux, q);
		p += pktsize;
	}

	i = count - p;
	if (i) {
		memcpy(demux->tsbuf, &buf[p], i);
		demux->tsbufp = i;
		if (pktsize == 204 && demux->tsbuf[0] == 0xB8)
			demux->tsbuf[0] = 0x47;
	}

bailout:
	spin_unlock(&demux->lock);
}

void dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	_dvb_dmx_swfilter(demux, buf, count, 188);
}
EXPORT_SYMBOL(dvb_dmx_swfilter);

void dvb_dmx_swfilter_204(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	_dvb_dmx_swfilter(demux, buf, count, 204);
}
EXPORT_SYMBOL(dvb_dmx_swfilter_204);

/***********************************
 * DVB-S2 Base-band demultiplexer  *
 ***********************************/

#define dprintk(x...) do { printk(x); } while (0)
/**
 * @brief Process a stream of (possibly fragmented) User Packets (UPs) from a
 * Generic Packetized Stream or Transport Sream for a specific demux feed.
 * @param feed The feed on which the UPs should be delivered
 * @param df Pointer to the data field (df) of a BBFrame containing (fragments of) UPs
 * @param dfl The length of the data field
 * @param upl User Packet Length as stated in the BBHEADER
 * @param upl Sync Distance as stated in the BBHEADER
 * @param upl Sync Byte as stated in the BBHEADER
 * @return
 *	@retval 0 in case of success
 *	@retval <0 in case of errors
 * @todo Add support for feed callback timeouts
 */
int dvb_dmx_swfilter_bbups(struct dvb_demux_feed *feed, const u8 *df, u16 dfl, 
	u16 upl, u16 syncd, u8 sync)
{
	u8 crc8;
	u16 rupl = !sync ? upl-1 : upl; /*real upl */
	size_t len, maxlen;
#ifdef DVB_DEMUX_DEBUG_BB	
	dprintk("%s: feed=%p, df=%p, dfl=%hu, upl=%hu, syncd=%hu, sync=%02x\n",
				__FUNCTION__, feed, df, dfl, upl, syncd, 
				(unsigned int) sync);
#endif
	if (!upl || syncd >= dfl)
		return -EINVAL;
	else if (!feed->buffer || upl > feed->buffer_size)
		return -ENOBUFS;
	
	/* NOTE: if the original input stream does not contain a sync byte,
	 * the DVB-S2 mode adaptation layer inserts a dummy sync byte (value 0) 
	 * for holding the CRC-8 and also increments the upl.
	 * We respect that by using the "real upl" (rupl) variable for packets
	 * in the feed buffer.
	 */
		
	// Limit size of usable buffer part to always store an integral number of packets
	maxlen = (feed->buffer_size/upl) * upl;
		
	if (syncd == (u16) -1) {
		// No packet starts here, so just copy the df to our buffer
		len = min((size_t) dfl, maxlen-feed->bb.bufpos);
		if (len > 0) {
			memcpy(&feed->buffer[feed->bb.bufpos], df, len);
			feed->bb.bufpos += len;
		} else 
			feed->bb.bufpos = 0;
	} else {
		const u8 *pkt = df;
		const u8 *const dfend = df+dfl;
		size_t partlen = feed->bb.bufpos % rupl;
		/* If at least one packet starts in this data field,
		 * check, if the data until the start of this packet
		 * completes a partial packet in our buffer
		 */
		 if (syncd > 0) {
			if ((partlen+syncd) == rupl && syncd <= (maxlen-feed->bb.bufpos)) {
				memcpy(&feed->buffer[feed->bb.bufpos], pkt, syncd);
				feed->bb.bufpos += syncd;
			} else {
				/* Assume that we lost a fragment of the partial 
				 * packet contained in the buffer and thus
				 * drop the partial packet.
				 */
				BUG_ON(feed->bb.bufpos < partlen); 
				feed->bb.bufpos -= partlen;
			}
			pkt += syncd;
		} else if (partlen > 0) {
			/* Assume that we lost a fragment of the partial 
			 * packet contained in the buffer and thus
			 * drop the partial packet.
			 */
			feed->bb.bufpos -= partlen;
		}
		
		
		/* NOTE: pkt is synced to a packet start, i.e. *pkt contains the
		 * CRC-8 of the last packet on this feed.
		 */
		while (pkt < dfend) {
#ifdef DVB_DEMUX_DEBUG_BB			
			dprintk("%s: pkt=%p, dfend=%p, bufpos=%u (of %u), upl=%hu, rupl=%hu\n",
				__FUNCTION__, pkt, dfend, feed->bb.bufpos, maxlen, upl, rupl);
#endif			
			if ((feed->bb.bufpos-feed->bb.crcpos) >= rupl) {
				/* verify the last packet in the buffer (not including the sync byte) */
				crc8 = bb_crc8(0x00, 
					&feed->buffer[feed->bb.crcpos+1], upl-1);
				if (crc8 != *pkt) {
					// discard packet
					feed->bb.bufpos -= upl;
				} else if (feed->bb.bufpos >= maxlen) {
					// Flush feed buffer 
			
					/* NOTE: at this point, all packets in the feed 
					 * buffer have been verified, so we're allowed to deliver
					 * them to the feed callback.
					 */
				
					// Invoke feed callback
					feed->cb.bb(feed->buffer, feed->bb.bufpos, 
						rupl, &feed->feed.bb, DMX_OK);
					
					feed->bb.crcpos = feed->bb.bufpos = 0;
				}
				else
					feed->bb.crcpos = feed->bb.bufpos;
			}
			
			BUG_ON(feed->bb.bufpos+rupl > maxlen);
			
			if (sync) // Sync byte present?
				feed->buffer[feed->bb.bufpos++] = sync;
			pkt++;
			
			len = min((u16) (upl-1), (u16) (dfend-pkt));
			/* NOTE: we do not check for len == 0, as it hopefully does not
			 * happen too often.
			 */
			memcpy(&feed->buffer[feed->bb.bufpos], pkt, len);
			pkt += len;
			feed->bb.bufpos += len;
		}
	}
	return 0;
}

/* Structure of a Base-Band frame:
 
 *  < ------------------------------ BBFrame ----------------------------------- >
 *    2 bytes 2 bytes  2 bytes  1 B  2 bytes  1 B            DFL bits
 *  |--------|--------|--------|----|--------|----|------------------------------|
 *  +--------+--------+--------+----+--------+----+-------------//---------------+
 *  | MATYPE |   UPL  |   DFL  |SYNC| SYNCD  |CRC8|          Data Field          |
 *  +--------+--------+--------+----+--------+----+-------------//---------------+
 *
 * Sub-structure of the MATYPE field:
 *  +--------+----+----+----+----+--------+------------------------------------+
 *  | TS/GS  |SIS/|CCM/|IS- |NPD |   RO   |           ISI or unused            |
 *  |        |MIS |ACM |SYI |    |        |                                    |
 *  +--------+----+----+----+----+--------+------------------------------------+
 *
 */


/**
 * @todo Invoke feed callbacks in case of an error 
 * @todo If mode == BB_TS and SIS-mode is active then we could by default forward
 * the TS to the TS demux. This would avoid the need to explicitely setup 
 * TS compatibility mode using dvb_dmx_set_ts_compat().
 * @note assumes that the caller has locked demux->lock.
 */
ssize_t _dvb_dmx_swfilter_bbframe(struct dvb_demux *demux, const u8 *frame, size_t len)
{
	ssize_t rv = -1;
	u16 framelen;
	int isi, mode;
	struct dvb_demux_feed *feed;
	u8 crc8;
	
	if (len < 10 || len > 7274) {
		// No complete BBHEADER present
		rv = -EINVAL;
		goto done;
	}
	
	// Verify CRC-8 of BBHEADER (frame[9])
	crc8 = bb_crc8(0x00, frame, 9);
	if (crc8 != frame[9]) {
		printk(KERN_NOTICE "%s: CRC check failed for BBHEADER "
			"(calculated %02x, received %02x)\n", __FUNCTION__, 
			(int) crc8, (int) frame[9]);
		goto done;
	}
	
	if (frame[0] & 0x20) { // Single Input Stream (SIS)
		isi = -1;
	} else { // Multiple Input Stream (MIS)
		isi = frame[1];
	}
	mode = (frame[0] & 0xc0);
	
	//  length values are measured in bits -> convert to bytes
	framelen = (frame[4] << 8) | frame[5];
	framelen >>= 3;
	
	if (!framelen || ((len-10) < framelen)) {
		printk(KERN_NOTICE "%s: invalid data field length (length %hu, left %u)\n",
			__FUNCTION__, framelen, len);
		rv = -1;
		goto done;
	}
#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: received frame for input stream %d (df length %u)\n",
		__FUNCTION__, isi, (unsigned int) framelen);
#endif	
	// First, give the raw frame to all BB_FRAME feeds 
	list_for_each_entry(feed, &demux->feed_list, list_head) {
		if (feed->type != DMX_TYPE_BB || feed->bb.type != BB_FRAME)
			continue;
		else if (feed->bb.isi != isi && feed->bb.isi != BB_ISI_ALL)
			continue;
		else 
			feed->cb.bb(frame, framelen+10, 0, &feed->feed.bb, DMX_OK);
	}
	
	if (mode == BB_TS || mode == BB_PACK_GS) {
		// Packetized stream
		u16 upl = ((frame[2] << 8) | frame[3]) >> 3;
		u16 syncd = ((frame[7] << 8) | frame[8]) >> 3;
		if (!upl) {
			if (mode == BB_TS)
				upl = 188;
			else {
				printk(KERN_NOTICE "%s: invalid user packet length of zero\n",
					__FUNCTION__);
				goto done;
			}
		}
		
		list_for_each_entry(feed, &demux->feed_list, list_head) {
			if (feed->type != DMX_TYPE_BB || feed->bb.type != mode)
				continue;
			else if (feed->bb.isi != isi && feed->bb.isi != BB_ISI_ALL)
				continue;
			else
				dvb_dmx_swfilter_bbups(feed, frame+10, framelen, 
					upl, syncd, frame[6]);
		}
	} else if (mode == BB_CONT_GS) {
		// Continuos stream
		list_for_each_entry(feed, &demux->feed_list, list_head) {
			if (feed->type != DMX_TYPE_BB || feed->bb.type != BB_CONT_GS)
				continue;
			else if (feed->bb.isi != isi && feed->bb.isi != BB_ISI_ALL)
				continue;
			else
				feed->cb.bb(frame+10, framelen, 0, &feed->feed.bb, DMX_OK);
		}
	} else {
		printk(KERN_NOTICE "%s: invalid use of reserved TS/GS value 1\n",
			__FUNCTION__);
		goto done;
	}

	rv = framelen+10;
	
done:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
	///@todo Make this configurable via sysfs or module param
	if (rv < 0) {
		print_hex_dump(KERN_DEBUG, "[BBFrame] ", DUMP_PREFIX_OFFSET, 16, 
			1, (void*) frame,  len, 0);
	}
#endif
	return rv;
}

ssize_t dvb_dmx_swfilter_bbframe(struct dvb_demux *demux, const u8 *frame, size_t len)
{
	ssize_t rv;
	spin_lock(&demux->lock);
	rv = _dvb_dmx_swfilter_bbframe(demux, frame, len);
	spin_unlock(&demux->lock);
	return rv;
}
EXPORT_SYMBOL(dvb_dmx_swfilter_bbframe);

#define BB_FRAME_LEN(ptr) (10 + ((((ptr)[4] << 8) | (ptr)[5]) >> 3))
void dvb_dmx_swfilter_data(struct dvb_demux *demux, fe_data_format_t dfmt, 
	const u8 *data, size_t len)
{
#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: demux=%p, dfmt=%d, data=%p, len=%u\n",
				__FUNCTION__, demux, dfmt, data,
				(unsigned int) len);
#endif
	spin_lock(&demux->lock);

	if(dfmt == FE_DFMT_BB_FRAME) {
		// A sequence of base-band frames as a stream of bytes
		u16 bblen;
		ssize_t res;
		size_t clen;
		while(demux->bb.wr > 0 && len > 0) {
#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: wr=%u, data=%p, len=%u\n",
				__FUNCTION__, demux->bb.wr, data, (unsigned int) len);
#endif
			if(demux->bb.wr < 10) {
				clen = min(10-demux->bb.wr, len);
				clen = min(clen, DMX_BB_BUFSZ-demux->bb.wr);
				memcpy(&demux->bb.buf[demux->bb.wr], data, clen);
				demux->bb.wr += clen;
				data += clen;
				len -= clen;
			} else {
				if(bb_crc8(0x00, &demux->bb.buf[0], 10) != 0) {
					demux->bb.wr -= 1;
					memmove(&demux->bb.buf[0], &demux->bb.buf[1], demux->bb.wr);
					continue;
				}
				bblen = BB_FRAME_LEN(demux->bb.buf);
				if(demux->bb.wr < bblen) {
					clen = min(bblen-demux->bb.wr, len);
					clen = min(clen, DMX_BB_BUFSZ-demux->bb.wr);
					memcpy(&demux->bb.buf[demux->bb.wr], data, clen);
					demux->bb.wr += clen;
					data += clen;
					len -= clen;
				}
				if(demux->bb.wr >= bblen) {
					res = _dvb_dmx_swfilter_bbframe(demux, &demux->bb.buf[0], bblen);
					demux->bb.wr -= bblen;
					if(demux->bb.wr > 0)
						memmove(&demux->bb.buf[0], &demux->bb.buf[bblen], demux->bb.wr);
				}
			}
			if(demux->bb.wr == DMX_BB_BUFSZ)
				demux->bb.wr = 0;
		}
		if(demux->bb.wr == 0) {
			while(len > 9) {
				if(bb_crc8(0x00, data, 10) != 0) {
					bblen = 1;
				} else {
					bblen = BB_FRAME_LEN(data);
					if(len < bblen) {
						break;
					}
					res = _dvb_dmx_swfilter_bbframe(demux, data, bblen);
				}
				data += bblen;
				len -= bblen;
			}
		}
		// copy remaining part to buffer
		BUG_ON((demux->bb.wr + len) > DMX_BB_BUFSZ);
		memcpy(&demux->bb.buf[demux->bb.wr], data, len);
		demux->bb.wr += len;
	} else {
		///@todo implement TS path
	}

	spin_unlock(&demux->lock);
}

EXPORT_SYMBOL(dvb_dmx_swfilter_data);

static struct dvb_demux_filter *dvb_dmx_filter_alloc(struct dvb_demux *demux)
{
	int i;

	for (i = 0; i < demux->filternum; i++)
		if (demux->filter[i].state == DMX_STATE_FREE)
			break;

	if (i == demux->filternum)
		return NULL;

	demux->filter[i].state = DMX_STATE_ALLOCATED;

	return &demux->filter[i];
}

static struct dvb_demux_feed *dvb_dmx_feed_alloc(struct dvb_demux *demux)
{
	int i;

	for (i = 0; i < demux->feednum; i++)
		if (demux->feed[i].state == DMX_STATE_FREE)
			break;

	if (i == demux->feednum)
		return NULL;

	demux->feed[i].state = DMX_STATE_ALLOCATED;

	return &demux->feed[i];
}

static int dvb_demux_feed_find(struct dvb_demux_feed *feed)
{
	struct dvb_demux_feed *entry;

	list_for_each_entry(entry, &feed->demux->feed_list, list_head)
		if (entry == feed)
			return 1;

	return 0;
}

static void dvb_demux_feed_add(struct dvb_demux_feed *feed)
{
	spin_lock_irq(&feed->demux->lock);
	if (dvb_demux_feed_find(feed)) {
		printk(KERN_ERR "%s: feed already in list (type=%x state=%x pid=%x)\n",
		       __func__, feed->type, feed->state, feed->pid);
		goto out;
	}

	list_add(&feed->list_head, &feed->demux->feed_list);
out:
	spin_unlock_irq(&feed->demux->lock);
}

static void dvb_demux_feed_del(struct dvb_demux_feed *feed)
{
	spin_lock_irq(&feed->demux->lock);
	if (!(dvb_demux_feed_find(feed))) {
		printk(KERN_ERR "%s: feed not in list (type=%x state=%x pid=%x)\n",
		       __func__, feed->type, feed->state, feed->pid);
		goto out;
	}

	list_del(&feed->list_head);
out:
	spin_unlock_irq(&feed->demux->lock);
}

static int dmx_ts_feed_set(struct dmx_ts_feed *ts_feed, u16 pid, int ts_type,
			   enum dmx_ts_pes pes_type,
			   size_t circular_buffer_size, struct timespec timeout)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)ts_feed;
	struct dvb_demux *demux = feed->demux;

	if (pid > DMX_MAX_PID)
		return -EINVAL;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (ts_type & TS_DECODER) {
		if (pes_type >= DMX_TS_PES_OTHER) {
			mutex_unlock(&demux->mutex);
			return -EINVAL;
		}

		if (demux->pesfilter[pes_type] &&
		    demux->pesfilter[pes_type] != feed) {
			mutex_unlock(&demux->mutex);
			return -EINVAL;
		}

		demux->pesfilter[pes_type] = feed;
		demux->pids[pes_type] = pid;
	}

	dvb_demux_feed_add(feed);

	feed->pid = pid;
	feed->buffer_size = circular_buffer_size;
	feed->timeout = timeout;
	feed->ts_type = ts_type;
	feed->pes_type = pes_type;

	if (feed->buffer_size) {
#ifdef NOBUFS
		feed->buffer = NULL;
#else
		feed->buffer = vmalloc(feed->buffer_size);
		if (!feed->buffer) {
			mutex_unlock(&demux->mutex);
			return -ENOMEM;
		}
#endif
	}

	feed->state = DMX_STATE_READY;
	mutex_unlock(&demux->mutex);

	return 0;
}

static int dmx_ts_feed_start_filtering(struct dmx_ts_feed *ts_feed)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)ts_feed;
	struct dvb_demux *demux = feed->demux;
	int ret;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (feed->state != DMX_STATE_READY || feed->type != DMX_TYPE_TS) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->start_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}

	if ((ret = demux->start_feed(feed)) < 0) {
		mutex_unlock(&demux->mutex);
		return ret;
	}

	spin_lock_irq(&demux->lock);
	ts_feed->is_filtering = 1;
	feed->state = DMX_STATE_GO;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return 0;
}

static int dmx_ts_feed_stop_filtering(struct dmx_ts_feed *ts_feed)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)ts_feed;
	struct dvb_demux *demux = feed->demux;
	int ret;

	mutex_lock(&demux->mutex);

	if (feed->state < DMX_STATE_GO) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->stop_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}

	ret = demux->stop_feed(feed);

	spin_lock_irq(&demux->lock);
	ts_feed->is_filtering = 0;
	feed->state = DMX_STATE_ALLOCATED;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return ret;
}

static int dvbdmx_allocate_ts_feed(struct dmx_demux *dmx,
				   struct dmx_ts_feed **ts_feed,
				   dmx_ts_cb callback)
{
	struct dvb_demux *demux = (struct dvb_demux *)dmx;
	struct dvb_demux_feed *feed;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(demux))) {
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}

	feed->type = DMX_TYPE_TS;
	feed->cb.ts = callback;
	feed->demux = demux;
	feed->pid = 0xffff;
	feed->peslen = 0xfffa;
	feed->buffer = NULL;

	(*ts_feed) = &feed->feed.ts;
	(*ts_feed)->parent = dmx;
	(*ts_feed)->priv = NULL;
	(*ts_feed)->is_filtering = 0;
	(*ts_feed)->start_filtering = dmx_ts_feed_start_filtering;
	(*ts_feed)->stop_filtering = dmx_ts_feed_stop_filtering;
	(*ts_feed)->set = dmx_ts_feed_set;

	if (!(feed->filter = dvb_dmx_filter_alloc(demux))) {
		feed->state = DMX_STATE_FREE;
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}

	feed->filter->type = DMX_TYPE_TS;
	feed->filter->feed = feed;
	feed->filter->state = DMX_STATE_READY;

	mutex_unlock(&demux->mutex);

	return 0;
}

static int dvbdmx_release_ts_feed(struct dmx_demux *dmx,
				  struct dmx_ts_feed *ts_feed)
{
	struct dvb_demux *demux = (struct dvb_demux *)dmx;
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)ts_feed;

	mutex_lock(&demux->mutex);

	if (feed->state == DMX_STATE_FREE) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}
#ifndef NOBUFS
	vfree(feed->buffer);
	feed->buffer = NULL;
#endif

	feed->state = DMX_STATE_FREE;
	feed->filter->state = DMX_STATE_FREE;

	dvb_demux_feed_del(feed);

	feed->pid = 0xffff;

	if (feed->ts_type & TS_DECODER && feed->pes_type < DMX_TS_PES_OTHER)
		demux->pesfilter[feed->pes_type] = NULL;

	mutex_unlock(&demux->mutex);
	return 0;
}

/******************************************************************************
 * dmx_section_feed API calls
 ******************************************************************************/

static int dmx_section_feed_allocate_filter(struct dmx_section_feed *feed,
					    struct dmx_section_filter **filter)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdemux = dvbdmxfeed->demux;
	struct dvb_demux_filter *dvbdmxfilter;

	if (mutex_lock_interruptible(&dvbdemux->mutex))
		return -ERESTARTSYS;

	dvbdmxfilter = dvb_dmx_filter_alloc(dvbdemux);
	if (!dvbdmxfilter) {
		mutex_unlock(&dvbdemux->mutex);
		return -EBUSY;
	}

	spin_lock_irq(&dvbdemux->lock);
	*filter = &dvbdmxfilter->filter;
	(*filter)->parent = feed;
	(*filter)->priv = NULL;
	dvbdmxfilter->feed = dvbdmxfeed;
	dvbdmxfilter->type = DMX_TYPE_SEC;
	dvbdmxfilter->state = DMX_STATE_READY;
	dvbdmxfilter->next = dvbdmxfeed->filter;
	dvbdmxfeed->filter = dvbdmxfilter;
	spin_unlock_irq(&dvbdemux->lock);

	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static int dmx_section_feed_set(struct dmx_section_feed *feed,
				u16 pid, size_t circular_buffer_size,
				int check_crc)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;

	if (pid > 0x1fff)
		return -EINVAL;

	if (mutex_lock_interruptible(&dvbdmx->mutex))
		return -ERESTARTSYS;

	dvb_demux_feed_add(dvbdmxfeed);

	dvbdmxfeed->pid = pid;
	dvbdmxfeed->buffer_size = circular_buffer_size;
	dvbdmxfeed->feed.sec.check_crc = check_crc;

#ifdef NOBUFS
	dvbdmxfeed->buffer = NULL;
#else
	dvbdmxfeed->buffer = vmalloc(dvbdmxfeed->buffer_size);
	if (!dvbdmxfeed->buffer) {
		mutex_unlock(&dvbdmx->mutex);
		return -ENOMEM;
	}
#endif

	dvbdmxfeed->state = DMX_STATE_READY;
	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static void prepare_secfilters(struct dvb_demux_feed *dvbdmxfeed)
{
	int i;
	struct dvb_demux_filter *f;
	struct dmx_section_filter *sf;
	u8 mask, mode, doneq;

	if (!(f = dvbdmxfeed->filter))
		return;
	do {
		sf = &f->filter;
		doneq = 0;
		for (i = 0; i < DVB_DEMUX_MASK_MAX; i++) {
			mode = sf->filter_mode[i];
			mask = sf->filter_mask[i];
			f->maskandmode[i] = mask & mode;
			doneq |= f->maskandnotmode[i] = mask & ~mode;
		}
		f->doneq = doneq ? 1 : 0;
	} while ((f = f->next));
}

static int dmx_section_feed_start_filtering(struct dmx_section_feed *feed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	int ret;

	if (mutex_lock_interruptible(&dvbdmx->mutex))
		return -ERESTARTSYS;

	if (feed->is_filtering) {
		mutex_unlock(&dvbdmx->mutex);
		return -EBUSY;
	}

	if (!dvbdmxfeed->filter) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}

	dvbdmxfeed->feed.sec.tsfeedp = 0;
	dvbdmxfeed->feed.sec.secbuf = dvbdmxfeed->feed.sec.secbuf_base;
	dvbdmxfeed->feed.sec.secbufp = 0;
	dvbdmxfeed->feed.sec.seclen = 0;

	if (!dvbdmx->start_feed) {
		mutex_unlock(&dvbdmx->mutex);
		return -ENODEV;
	}

	prepare_secfilters(dvbdmxfeed);

	if ((ret = dvbdmx->start_feed(dvbdmxfeed)) < 0) {
		mutex_unlock(&dvbdmx->mutex);
		return ret;
	}

	spin_lock_irq(&dvbdmx->lock);
	feed->is_filtering = 1;
	dvbdmxfeed->state = DMX_STATE_GO;
	spin_unlock_irq(&dvbdmx->lock);

	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static int dmx_section_feed_stop_filtering(struct dmx_section_feed *feed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	int ret;

	mutex_lock(&dvbdmx->mutex);

	if (!dvbdmx->stop_feed) {
		mutex_unlock(&dvbdmx->mutex);
		return -ENODEV;
	}

	ret = dvbdmx->stop_feed(dvbdmxfeed);

	spin_lock_irq(&dvbdmx->lock);
	dvbdmxfeed->state = DMX_STATE_READY;
	feed->is_filtering = 0;
	spin_unlock_irq(&dvbdmx->lock);

	mutex_unlock(&dvbdmx->mutex);
	return ret;
}

static int dmx_section_feed_release_filter(struct dmx_section_feed *feed,
					   struct dmx_section_filter *filter)
{
	struct dvb_demux_filter *dvbdmxfilter = (struct dvb_demux_filter *)filter, *f;
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;

	mutex_lock(&dvbdmx->mutex);

	if (dvbdmxfilter->feed != dvbdmxfeed) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}

	if (feed->is_filtering)
		feed->stop_filtering(feed);

	spin_lock_irq(&dvbdmx->lock);
	f = dvbdmxfeed->filter;

	if (f == dvbdmxfilter) {
		dvbdmxfeed->filter = dvbdmxfilter->next;
	} else {
		while (f->next != dvbdmxfilter)
			f = f->next;
		f->next = f->next->next;
	}

	dvbdmxfilter->state = DMX_STATE_FREE;
	spin_unlock_irq(&dvbdmx->lock);
	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static int dvbdmx_allocate_section_feed(struct dmx_demux *demux,
					struct dmx_section_feed **feed,
					dmx_section_cb callback)
{
	struct dvb_demux *dvbdmx = (struct dvb_demux *)demux;
	struct dvb_demux_feed *dvbdmxfeed;

	if (mutex_lock_interruptible(&dvbdmx->mutex))
		return -ERESTARTSYS;

	if (!(dvbdmxfeed = dvb_dmx_feed_alloc(dvbdmx))) {
		mutex_unlock(&dvbdmx->mutex);
		return -EBUSY;
	}

	dvbdmxfeed->type = DMX_TYPE_SEC;
	dvbdmxfeed->cb.sec = callback;
	dvbdmxfeed->demux = dvbdmx;
	dvbdmxfeed->pid = 0xffff;
	dvbdmxfeed->feed.sec.secbuf = dvbdmxfeed->feed.sec.secbuf_base;
	dvbdmxfeed->feed.sec.secbufp = dvbdmxfeed->feed.sec.seclen = 0;
	dvbdmxfeed->feed.sec.tsfeedp = 0;
	dvbdmxfeed->filter = NULL;
	dvbdmxfeed->buffer = NULL;

	(*feed) = &dvbdmxfeed->feed.sec;
	(*feed)->is_filtering = 0;
	(*feed)->parent = demux;
	(*feed)->priv = NULL;

	(*feed)->set = dmx_section_feed_set;
	(*feed)->allocate_filter = dmx_section_feed_allocate_filter;
	(*feed)->start_filtering = dmx_section_feed_start_filtering;
	(*feed)->stop_filtering = dmx_section_feed_stop_filtering;
	(*feed)->release_filter = dmx_section_feed_release_filter;

	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static int dvbdmx_release_section_feed(struct dmx_demux *demux,
				       struct dmx_section_feed *feed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = (struct dvb_demux *)demux;

	mutex_lock(&dvbdmx->mutex);

	if (dvbdmxfeed->state == DMX_STATE_FREE) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}
#ifndef NOBUFS
	vfree(dvbdmxfeed->buffer);
	dvbdmxfeed->buffer = NULL;
#endif
	dvbdmxfeed->state = DMX_STATE_FREE;

	dvb_demux_feed_del(dvbdmxfeed);

	dvbdmxfeed->pid = 0xffff;

	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

/******************************************************************************
 * dmx_bb_feed calls
 ******************************************************************************/

static int dmx_bb_feed_set(struct dmx_bb_feed *bb_feed, int isi, int mode,
	size_t circular_buffer_size, struct timespec timeout)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed*) bb_feed;
	struct dvb_demux *demux = feed->demux;

#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: bb_feed=%p, isi=%d, mode=%d, bufsz=%u\n",
		__FUNCTION__, bb_feed, isi, mode, circular_buffer_size);
#endif
	///@todo Max ISI may be dependent on delivery system
	if (isi > 255 || (isi < 0 && isi != BB_ISI_ALL && isi != BB_ISI_SIS))
		return -EINVAL;
	else if (mode != BB_FRAME && mode != BB_PACK_GS && mode != BB_CONT_GS && 
		mode != BB_TS)
		return -EINVAL;
	else if (isi == BB_ISI_ALL && mode != BB_FRAME)
		return -EINVAL;

	feed->buffer_size = circular_buffer_size; 
	if (feed->buffer_size) {
		feed->buffer = vmalloc(feed->buffer_size);
		if (!feed->buffer) {
			return -ENOMEM;
		}
	} else if (mode == BB_PACK_GS || mode == BB_TS) {
		// we need a buffer for packetized streams!
		return -EINVAL;
	}
	
	feed->bb.isi = isi;
	feed->bb.type = mode;
	feed->state = DMX_STATE_READY;
	
	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;
	
	dvb_demux_feed_add(feed);
	
	mutex_unlock(&demux->mutex);
	return 0;
}

static int dmx_bb_feed_start_filtering(struct dmx_bb_feed *bb_feed)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)bb_feed;
	struct dvb_demux *demux = feed->demux;
	int ret;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (feed->state != DMX_STATE_READY || feed->type != DMX_TYPE_BB) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->start_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}

	if ((ret = demux->start_feed(feed)) < 0) {
		mutex_unlock(&demux->mutex);
		return ret;
	}

	spin_lock_irq(&demux->lock);
	bb_feed->is_filtering = 1;
	feed->state = DMX_STATE_GO;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return 0;
}

static int dmx_bb_feed_stop_filtering(struct dmx_bb_feed *bb_feed)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)bb_feed;
	struct dvb_demux *demux = feed->demux;
	int ret;

	mutex_lock(&demux->mutex);

	if (feed->state < DMX_STATE_GO) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->stop_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}

	ret = demux->stop_feed(feed);

	spin_lock_irq(&demux->lock);
	bb_feed->is_filtering = 0;
	feed->state = DMX_STATE_READY;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return ret;
}

static int dvbdmx_allocate_bb_feed(struct dmx_demux *dmx, struct dmx_bb_feed **bb_feed,
	dmx_bb_cb callback)
{
	struct dvb_demux *demux = (struct dvb_demux *)dmx;
	struct dvb_demux_feed *feed;

#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: dmx=%p, bb_feed=%p, callback=%p\n",
		__FUNCTION__, dmx, bb_feed, callback);
#endif
	
	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(demux))) {
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}

	feed->type = DMX_TYPE_BB;
	feed->cb.bb = callback;
	feed->demux = demux;
	feed->bb.isi = BB_ISI_SIS;
	feed->bb.type = BB_FRAME;
	feed->buffer = NULL;
	feed->bb.bufpos = 0;
	feed->bb.crcpos = 0;
	
	*bb_feed = &feed->feed.bb;
	(*bb_feed)->parent = dmx;
	(*bb_feed)->priv = NULL;
	(*bb_feed)->is_filtering = 0;
	(*bb_feed)->start_filtering = dmx_bb_feed_start_filtering;
	(*bb_feed)->stop_filtering = dmx_bb_feed_stop_filtering;
	(*bb_feed)->set = dmx_bb_feed_set;
	
	mutex_unlock(&demux->mutex);
	return 0;
}

static int dvbdmx_release_bb_feed(struct dmx_demux *dmx, struct dmx_bb_feed *bb_feed)
{
	struct dvb_demux *demux = (struct dvb_demux *)dmx;
	struct dvb_demux_feed *feed = (struct dvb_demux_feed*) bb_feed;

#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: dmx=%p, bb_feed=%p\n",
		__FUNCTION__, dmx, bb_feed);
#endif

	mutex_lock(&demux->mutex);

	if (feed->state == DMX_STATE_FREE) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	feed->state = DMX_STATE_FREE;

	dvb_demux_feed_del(feed);

	mutex_unlock(&demux->mutex);
	return 0;
}

/******************************************************************************
 * dvb_demux kernel data API calls
 ******************************************************************************/

static int dvbdmx_open(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (dvbdemux->users >= MAX_DVB_DEMUX_USERS)
		return -EUSERS;

	dvbdemux->users++;
	return 0;
}

static int dvbdmx_close(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (dvbdemux->users == 0)
		return -ENODEV;

	dvbdemux->users--;
	//FIXME: release any unneeded resources if users==0
	return 0;
}

static int dvbdmx_write(struct dmx_demux *demux, const char __user *buf, size_t count)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	void *p;

	if ((!demux->frontend) || (demux->frontend->source != DMX_MEMORY_FE))
		return -EINVAL;

	p = memdup_user(buf, count);
	if (IS_ERR(p))
		return PTR_ERR(p);
	if (mutex_lock_interruptible(&dvbdemux->mutex)) {
		kfree(p);
		return -ERESTARTSYS;
	}
	dvb_dmx_swfilter(dvbdemux, p, count);
	kfree(p);
	mutex_unlock(&dvbdemux->mutex);

	if (signal_pending(current))
		return -EINTR;
	return count;
}

static int dvbdmx_add_frontend(struct dmx_demux *demux,
			       struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	struct list_head *head = &dvbdemux->frontend_list;

	list_add(&(frontend->connectivity_list), head);

	return 0;
}

static int dvbdmx_remove_frontend(struct dmx_demux *demux,
				  struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	struct list_head *pos, *n, *head = &dvbdemux->frontend_list;

	list_for_each_safe(pos, n, head) {
		if (DMX_FE_ENTRY(pos) == frontend) {
			list_del(pos);
			return 0;
		}
	}

	return -ENODEV;
}

static struct list_head *dvbdmx_get_frontends(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (list_empty(&dvbdemux->frontend_list))
		return NULL;

	return &dvbdemux->frontend_list;
}

static int dvbdmx_connect_frontend(struct dmx_demux *demux,
				   struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (demux->frontend)
		return -EINVAL;

	mutex_lock(&dvbdemux->mutex);

	demux->frontend = frontend;
	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static int dvbdmx_disconnect_frontend(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	mutex_lock(&dvbdemux->mutex);

	demux->frontend = NULL;
	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static int dvbdmx_get_pes_pids(struct dmx_demux *demux, u16 * pids)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	memcpy(pids, dvbdemux->pids, 5 * sizeof(u16));
	return 0;
}

static void dvb_dmx_ts_compat_bb_callback(const u8 *buf, size_t len, size_t upl, 
	struct dmx_bb_feed* source, enum dmx_success success)
{
	unsigned int count;
#ifdef DVB_DEMUX_DEBUG_BB
	dprintk("%s: buf=%p, len=%u, upl=%u, source=%p, success=%d\n",
		__FUNCTION__, buf, len, upl, source, success);
#endif
	if (success != DMX_OK || upl != 188)
		return;
	for(count=len/188; count > 0; count--, buf+=188)
		dvb_dmx_swfilter_packet((struct dvb_demux*) source->parent, buf); 
	
}

/**
 * @brief Enable or disable TS compatibility mode on the specified input stream (ISI).
 */
static int dvb_dmx_set_ts_compat(struct dvb_demux *dvbdemux, int isi, int enable)
{
	int rv = -1, found = 0;
	struct dmx_demux *demux = &dvbdemux->dmx;
	struct dmx_bb_feed *feed;
	struct dvb_demux_feed *dvbfeed;
	struct timespec timeout = {0, 1*1000*1000}; /* 1 ms */
	
	if (!demux->allocate_bb_feed) {
		printk(KERN_ERR "%s: demux does not support bb feeds\n", 
			__FUNCTION__);
		goto done;
	}
	
	spin_lock(&dvbdemux->lock);
	list_for_each_entry(dvbfeed, &dvbdemux->feed_list, list_head) {
		if ((dvbfeed->type != DMX_TYPE_BB) || (dvbfeed->bb.type != BB_TS) || 
			(dvbfeed->bb.isi != isi) || 
			(dvbfeed->cb.bb != dvb_dmx_ts_compat_bb_callback))
			continue;
		found = 1;
		break;
	}
	spin_unlock(&dvbdemux->lock);
	
	if (enable && !found) {
		dprintk("%s: alloc bb feed\n", __FUNCTION__);
		
		rv = demux->allocate_bb_feed(demux, &feed, dvb_dmx_ts_compat_bb_callback);
		if (rv < 0) {
			printk(KERN_ERR "%s: could not allocate bb feed\n", 
				__FUNCTION__);
			goto done;
		}
		
		feed->priv = NULL;
		rv = feed->set(feed,
				isi,		/* isi */
				BB_TS,		/* type */
				7274,   	/* circular buffer size */
				timeout		/* timeout */
				);
		if (rv < 0) {
			printk(KERN_ERR "%s: could not set bb feed\n", __FUNCTION__);
			demux->release_bb_feed(demux, feed);
			goto done;
		}
	
		dprintk("%s: start filtering\n", __FUNCTION__);
		
		if (isi == BB_ISI_ALL)
			printk("%s: enabled TS-Compatibility mode on all ISIs\n", 
				__FUNCTION__);
		else
			printk("%s: enabled TS-Compatibility mode on ISI %d\n", 
				__FUNCTION__, isi);
		
		feed->start_filtering(feed);
	} else if (!enable && found) {
		feed = &dvbfeed->feed.bb;
		if (feed->is_filtering) {
			dprintk("%s: stop filtering\n", __FUNCTION__);
			feed->stop_filtering(feed);
		}
		dprintk("%s: release bb feed\n", __FUNCTION__);
		demux->release_bb_feed(demux, feed);
		
		if (isi == BB_ISI_ALL)
			printk("%s: disabled TS-Compatibility mode on all ISIs\n", 
				__FUNCTION__);
		else
			printk("%s: disabled TS-Compatibility mode on ISI %d\n", 
				__FUNCTION__, isi);
	}
	
	rv = 0;
done:
	return rv;
}

int dvb_dmx_init(struct dvb_demux *dvbdemux)
{
	int i;
	struct dmx_demux *dmx = &dvbdemux->dmx;

	dvbdemux->cnt_storage = NULL;
	dvbdemux->users = 0;
	dvbdemux->filter = vmalloc(dvbdemux->filternum * sizeof(struct dvb_demux_filter));

	if (!dvbdemux->filter)
		return -ENOMEM;

	dvbdemux->feed = vmalloc(dvbdemux->feednum * sizeof(struct dvb_demux_feed));
	if (!dvbdemux->feed) {
		vfree(dvbdemux->filter);
		dvbdemux->filter = NULL;
		return -ENOMEM;
	}
	for (i = 0; i < dvbdemux->filternum; i++) {
		dvbdemux->filter[i].state = DMX_STATE_FREE;
		dvbdemux->filter[i].index = i;
	}
	for (i = 0; i < dvbdemux->feednum; i++) {
		dvbdemux->feed[i].state = DMX_STATE_FREE;
		dvbdemux->feed[i].index = i;
	}

	dvbdemux->cnt_storage = vmalloc(MAX_PID + 1);
	if (!dvbdemux->cnt_storage)
		printk(KERN_WARNING "Couldn't allocate memory for TS/TEI check. Disabling it\n");

	INIT_LIST_HEAD(&dvbdemux->frontend_list);

	for (i = 0; i < DMX_TS_PES_OTHER; i++) {
		dvbdemux->pesfilter[i] = NULL;
		dvbdemux->pids[i] = 0xffff;
	}

	INIT_LIST_HEAD(&dvbdemux->feed_list);

	dvbdemux->playing = 0;
	dvbdemux->recording = 0;
	dvbdemux->tsbufp = 0;

	if (!dvbdemux->check_crc32)
		dvbdemux->check_crc32 = dvb_dmx_crc32;

	if (!dvbdemux->memcopy)
		dvbdemux->memcopy = dvb_dmx_memcopy;

	dmx->frontend = NULL;
	dmx->priv = dvbdemux;
	dmx->open = dvbdmx_open;
	dmx->close = dvbdmx_close;
	dmx->write = dvbdmx_write;
	dmx->allocate_ts_feed = dvbdmx_allocate_ts_feed;
	dmx->release_ts_feed = dvbdmx_release_ts_feed;
	dmx->allocate_section_feed = dvbdmx_allocate_section_feed;
	dmx->release_section_feed = dvbdmx_release_section_feed;
	dmx->allocate_bb_feed = dvbdmx_allocate_bb_feed;
	dmx->release_bb_feed = dvbdmx_release_bb_feed;

	dmx->add_frontend = dvbdmx_add_frontend;
	dmx->remove_frontend = dvbdmx_remove_frontend;
	dmx->get_frontends = dvbdmx_get_frontends;
	dmx->connect_frontend = dvbdmx_connect_frontend;
	dmx->disconnect_frontend = dvbdmx_disconnect_frontend;
	dmx->get_pes_pids = dvbdmx_get_pes_pids;

	mutex_init(&dvbdemux->mutex);
	spin_lock_init(&dvbdemux->lock);

	dvbdemux->bb.buf = vmalloc(DMX_BB_BUFSZ);
	if (!dvbdemux->bb.buf)
		return -ENOMEM;
	dvbdemux->bb.wr = 0;


	/* Activate TS-Compat mode for single-input stream channels */
	//dvb_dmx_set_ts_compat(dvbdemux, BB_ISI_SIS, 1);
	return 0;
}

EXPORT_SYMBOL(dvb_dmx_init);

void dvb_dmx_release(struct dvb_demux *dvbdemux)
{
	dvb_dmx_set_ts_compat(dvbdemux, BB_ISI_SIS, 0);
	vfree(dvbdemux->cnt_storage);
	vfree(dvbdemux->filter);
	vfree(dvbdemux->feed);
	vfree(dvbdemux->bb.buf);
}

EXPORT_SYMBOL(dvb_dmx_release);
