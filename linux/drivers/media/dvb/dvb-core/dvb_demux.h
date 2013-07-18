/*
 * dvb_demux.h: DVB kernel demux API
 *
 * Copyright (C) 2000-2001 Marcus Metzler & Ralph Metzler
 *                         for convergence integrated media GmbH
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

#ifndef _DVB_DEMUX_H_
#define _DVB_DEMUX_H_

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include <linux/dvb/frontend.h>

#include "demux.h"

#define DMX_TYPE_TS  0
#define DMX_TYPE_SEC 1
#define DMX_TYPE_PES 2
#define DMX_TYPE_BB  3

#define DMX_STATE_FREE      0
#define DMX_STATE_ALLOCATED 1
#define DMX_STATE_SET       2
#define DMX_STATE_READY     3
#define DMX_STATE_GO        4

#define DVB_DEMUX_MASK_MAX 18

#define MAX_PID 0x1fff

#define SPEED_PKTS_INTERVAL 50000

struct dvb_demux_filter {
	struct dmx_section_filter filter;
	u8 maskandmode[DMX_MAX_FILTER_SIZE];
	u8 maskandnotmode[DMX_MAX_FILTER_SIZE];
	int doneq;

	struct dvb_demux_filter *next;
	struct dvb_demux_feed *feed;
	int index;
	int state;
	int type;

	u16 hw_handle;
	struct timer_list timer;
};

#define DMX_FEED_ENTRY(pos) list_entry(pos, struct dvb_demux_feed, list_head)

struct dvb_demux_feed {
	union {
		struct dmx_ts_feed ts;
		struct dmx_section_feed sec;
		struct dmx_bb_feed bb;
	} feed;

	union {
		dmx_ts_cb ts;
		dmx_section_cb sec;
		dmx_bb_cb bb;
	} cb;

	struct dvb_demux *demux;
	void *priv;
	int type;
	int state;
	u16 pid;
	u8 *buffer;
	int buffer_size;

	struct timespec timeout;
	struct dvb_demux_filter *filter;

	int ts_type;
	enum dmx_ts_pes pes_type;

	int cc;
	int pusi_seen;		/* prevents feeding of garbage from previous section */

	u16 peslen;

	/* Data for the Base-band demux */
	struct {
		/// Input Stream Identifier. A value of -1 indicates Single Input Stream operation
		int isi;
		/// Feed type (BB_FRAME, BB_PACK_GS, BB_CONT_GS or BB_TS)
		u8 type;
		/// Position in buffer (feed->buffer)
		unsigned int bufpos;
		/**
		 * This is the offset of the last packet in the buffer which needs CRC validation.
		 * @note All packets in the buffer with offsets < crcpos have already been validated.
		 */
		unsigned int crcpos;
	} bb;

	struct list_head list_head;
	unsigned int index;	/* a unique index for each feed (can be used as hardware pid filter index) */
};

struct dvb_demux {
	struct dmx_demux dmx;
	void *priv;
	int filternum;
	int feednum;
	int (*start_feed)(struct dvb_demux_feed *feed);
	int (*stop_feed)(struct dvb_demux_feed *feed);
	int (*write_to_decoder)(struct dvb_demux_feed *feed,
				 const u8 *buf, size_t len);
	u32 (*check_crc32)(struct dvb_demux_feed *feed,
			    const u8 *buf, size_t len);
	void (*memcopy)(struct dvb_demux_feed *feed, u8 *dst,
			 const u8 *src, size_t len);

	int users;
#define MAX_DVB_DEMUX_USERS 10
	struct dvb_demux_filter *filter;
	struct dvb_demux_feed *feed;

	struct list_head frontend_list;

	struct dvb_demux_feed *pesfilter[DMX_TS_PES_OTHER];
	u16 pids[DMX_TS_PES_OTHER];
	int playing;
	int recording;

#define DMX_MAX_PID 0x2000
	struct list_head feed_list;
	u8 tsbuf[204];
	int tsbufp;


	/**
	 * Members which are specific to the BB-Demux 
	 */
	struct {
#define DMX_BB_BUFSZ	8192
		u8 *buf;
		unsigned int wr;
	} bb;

	struct mutex mutex;
	spinlock_t lock;

	uint8_t *cnt_storage; /* for TS continuity check */

	struct timespec speed_last_time; /* for TS speed check */
	uint32_t speed_pkts_cnt; /* for TS speed check */
};

int dvb_dmx_init(struct dvb_demux *dvbdemux);
void dvb_dmx_release(struct dvb_demux *dvbdemux);
void dvb_dmx_swfilter_packets(struct dvb_demux *dvbdmx, const u8 *buf,
			      size_t count);
void dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf, size_t count);
void dvb_dmx_swfilter_204(struct dvb_demux *demux, const u8 *buf,
			  size_t count);
/**
 * Pass a single Base-band frame (BBFrame) to the demultiplexer
 * @param demux Pointer to DVB demux context
 * @param frame Buffer containing the full BBFrame
 * @param len Length of the BBFrame in \c frame
 * @return The length of the BBFrame or a value < 0 in case of an error
 * @note This function MUST NOT be call from inside a feed callback
 */
ssize_t dvb_dmx_swfilter_bbframe(struct dvb_demux *demux, const u8 *frame, size_t len);

void dvb_dmx_swfilter_data(struct dvb_demux *demux, fe_data_format_t dfmt, 
	const u8 *data, size_t len);

#endif /* _DVB_DEMUX_H_ */
