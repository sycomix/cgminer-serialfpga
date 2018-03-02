/**
 *   ztex.c - cgminer worker for Ztex 1.15x fpga board
 *
 *   Copyright (c) 2012 nelisky.btc@gmail.com
 *
 *   This work is based upon the Java SDK provided by ztex which is
 *   Copyright (C) 2009-2011 ZTEX GmbH.
 *   http://www.ztex.de
 *
 *   This work is based upon the icarus.c worker which is
 *   Copyright 2012 Luke Dashjr
 *   Copyright 2012 Xiangfu <xiangfu@openmobilefree.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, see http://www.gnu.org/licenses/.
**/
#include "miner.h"
#include <unistd.h>
#include <sha2.h>
#include "libztex.h"
#include "util.h"

struct device_drv ztex_drv;

static int option_offset = -1;

static void ztex_disable(struct thr_info* thr);
static bool ztex_prepare(struct thr_info *thr);
extern uint32_t ztex_checkNonce(struct work *work, uint32_t nonce);
extern void calc_midstate(struct work *work);

void set_starttime(char *f, struct timeval *tv)
{
	struct tm *tm;

	const time_t tmp_time = tv->tv_sec;
	tm = localtime(&tmp_time);
	sprintf(f, "[%d-%02d-%02d %02d:%02d:%02d]",
		tm->tm_year + 1900,
		tm->tm_mon + 1,
		tm->tm_mday,
		tm->tm_hour,
		tm->tm_min,
		tm->tm_sec);
}

static void ztex_selectFpga(struct libztex_device* ztex)
{
	if (ztex->root->numberOfFpgas > 1) {
		if (ztex->root->selectedFpga != ztex->fpgaNum)
			mutex_lock(&ztex->root->mutex);
		libztex_selectFpga(ztex);
	}
}

static void ztex_releaseFpga(struct libztex_device* ztex)
{
	if (ztex->root->numberOfFpgas > 1) {
		ztex->root->selectedFpga = -1;
		mutex_unlock(&ztex->root->mutex);
	}
}

static void ztex_detect(bool __maybe_unused hotplug)
{
	int cnt;
	int i,j;
	int fpgacount;
	struct libztex_dev_list **ztex_devices;
	struct libztex_device *ztex_slave;
	struct cgpu_info *ztex;

	cnt = libztex_scanDevices(&ztex_devices);
	if (cnt > 0)
		applog(LOG_WARNING, "Found %d ztex board%s", cnt, cnt > 1 ? "s" : "");

	for (i = 0; i < cnt; i++) {
		ztex = calloc(1, sizeof(struct cgpu_info));
		ztex->drv = &ztex_drv;
		ztex->device_ztex = ztex_devices[i]->dev;
		ztex->threads = 1;
		ztex->device_ztex->fpgaNum = 0;
		ztex->device_ztex->root = ztex->device_ztex;
		add_cgpu(ztex);

		fpgacount = libztex_numberOfFpgas(ztex->device_ztex);

		if (fpgacount > 1)
			pthread_mutex_init(&ztex->device_ztex->mutex, NULL);

		for (j = 1; j < fpgacount; j++) {
			ztex = calloc(1, sizeof(struct cgpu_info));
			ztex->drv = &ztex_drv;
			ztex_slave = calloc(1, sizeof(struct libztex_device));
			memcpy(ztex_slave, ztex_devices[i]->dev, sizeof(struct libztex_device));
			ztex->device_ztex = ztex_slave;
			ztex->threads = 1;
			ztex_slave->fpgaNum = j;
			ztex_slave->root = ztex_devices[i]->dev;
			ztex_slave->repr[strlen(ztex_slave->repr) - 1] = ('1' + j);
			add_cgpu(ztex);
		}

		applog(LOG_WARNING,"%s: Found Ztex (fpga count = %d) , mark as %d", ztex->device_ztex->repr, fpgacount, ztex->device_id);
	}

	if (cnt > 0)
		libztex_freeDevList(ztex_devices);
}

static bool ztex_updateFreq(struct libztex_device* ztex)
{
	int i, maxM, bestM;
	double bestR, r;

	for (i = 0; i < ztex->freqMaxM; i++)
		if (ztex->maxErrorRate[i + 1] * i < ztex->maxErrorRate[i] * (i + 20))
			ztex->maxErrorRate[i + 1] = ztex->maxErrorRate[i] * (1.0 + 20.0 / i);

	maxM = 0;
	while (maxM < ztex->freqMDefault && ztex->maxErrorRate[maxM + 1] < LIBZTEX_MAXMAXERRORRATE)
		maxM++;
	while (maxM < ztex->freqMaxM && ztex->errorWeight[maxM] > 150 && ztex->maxErrorRate[maxM + 1] < LIBZTEX_MAXMAXERRORRATE)
		maxM++;

	bestM = 0;
	bestR = 0;
	for (i = 0; i <= maxM; i++) {
		r = (i + 1 + (i == ztex->freqM? LIBZTEX_ERRORHYSTERESIS: 0)) * (1 - ztex->maxErrorRate[i]);
		if (r > bestR) {
			bestM = i;
			bestR = r;
		}
	}

	if (bestM != ztex->freqM) {
		ztex_selectFpga(ztex);
		libztex_setFreq(ztex, bestM);
		ztex_releaseFpga(ztex);
	}

	maxM = ztex->freqMDefault;
	while (maxM < ztex->freqMaxM && ztex->errorWeight[maxM + 1] > 100)
		maxM++;
	if ((bestM < (1.0 - LIBZTEX_OVERHEATTHRESHOLD) * maxM) && bestM < maxM - 1) {
		ztex_selectFpga(ztex);
		libztex_resetFpga(ztex);
		ztex_releaseFpga(ztex);
		applog(LOG_ERR, "%s: frequency drop of %.1f%% detect. This may be caused by overheating. FPGA is shut down to prevent damage.",
		       ztex->repr, (1.0 - 1.0 * bestM / maxM) * 100);
		return false;
	}
	return true;
}

static int64_t ztex_scanwork(struct thr_info *thr)
{
	struct libztex_device *ztex;
	struct libztex_hash_data hdata;
	struct timeval tv_start, tv_end, diff;
	unsigned char sendbuf[60];
	int count, validNonces, errorCount;
	int i, rc;
	uint32_t nonce, hash_count;
	uint32_t golden_nonce1, golden_nonce2;
	uint32_t last_nonce, last_golden1, last_golden2;
	bool overflow;
	uint32_t * sb;
	sb = (uint32_t *)sendbuf;
	
	struct work *work;
	work = get_work(thr, thr->id);
	
	if (thr->cgpu->deven == DEV_DISABLED)
		return -1;

	ztex = thr->cgpu->device_ztex;

	// Copy And Swap The Remaining Block Data
	memcpy(sendbuf,      work->data + 152, 4);
	memcpy(sendbuf +  4, work->data + 148, 4);
	memcpy(sendbuf +  8, work->data + 144, 4);
	memcpy(sendbuf + 12, work->data + 140, 4);
	memcpy(sendbuf + 16, work->data + 136, 4);
	memcpy(sendbuf + 20, work->data + 132, 4);
	memcpy(sendbuf + 24, work->data + 128, 4);

	sb[0] = swab32(sb[0]);
	sb[1] = swab32(sb[1]);
	sb[2] = swab32(sb[2]);
	sb[3] = swab32(sb[3]);
	sb[4] = swab32(sb[4]);
	sb[5] = swab32(sb[5]);
	sb[6] = swab32(sb[6]);
	
	// Copy The Midstate
	calc_midstate(work);
	swap256(sendbuf + 28, work->midstate);

	// Send Work To FPGA
	ztex_selectFpga(ztex);
	rc = libztex_sendHashData(ztex, sendbuf);
	if (rc < 0) {
		applog(LOG_ERR, "%s: Failed to send hash data with err %d, retrying", ztex->repr, rc);
		cgsleep_ms(500);
		rc = libztex_sendHashData(ztex, sendbuf);
		if (rc < 0) {
			ztex_disable(thr);
			ztex_releaseFpga(ztex);
			return -1;
		}
	}
	ztex_releaseFpga(ztex);

	applog(LOG_DEBUG, "%s: sent hashdata", ztex->repr);
	
	overflow = false;
	last_golden1 = 0;
	last_golden2 = 0;
	last_nonce = 0;
	count = 0;
	validNonces = 0;
	hash_count = 0;
	errorCount = 0;
	cgtime(&tv_start);

	applog(LOG_DEBUG, "%s: entering poll loop", ztex->repr);
	while (!(overflow || thr->work_restart)) {
		count++;

		int sleepcount = 0;
		while (thr->work_restart == 0 && sleepcount < 25) {
			cgsleep_ms(10);
			sleepcount += 1;
		}

		// Read Results From FPGA
		ztex_selectFpga(ztex);
		rc = libztex_readHashData(ztex, &hdata);
		if (rc < 0) {
			applog(LOG_ERR, "%s: Failed to read hash data with err %d, retrying", ztex->repr, rc);
			cgsleep_ms(500);
			rc = libztex_readHashData(ztex, &hdata);
			if (rc < 0) {
				ztex_disable(thr);
				ztex_releaseFpga(ztex);
				return -1;
			}
		}
		ztex_releaseFpga(ztex);

		// Check If New Work Is Available
		if (thr->work_restart) {
			applog(LOG_DEBUG, "%s: New work detected", ztex->repr);
			break;
		}

		ztex->errorCount[ztex->freqM] *= 0.995;
		ztex->errorWeight[ztex->freqM] = ztex->errorWeight[ztex->freqM] * 0.995 + 1.0;

		nonce = hdata.nonce;

		// Get Rid of FPGA Noise
		if ((nonce == 0x00000000) || (nonce == hdata.hash7)) {
			continue;
		}

		// Check For Hardware Errors On The FPGA
		if (ztex_checkNonce(work, nonce) != (hdata.hash7)) {
			if (count > 2) {	// Only Count Errors After The First 500ms Of Work Being Sent To FPGA
				thr->cgpu->hw_errors++;
				errorCount += 1;
				applog(LOG_WARNING, "%s: Check Nonce Failed - %08X", ztex->repr, nonce);
			}
			continue;
		}

		// Check If FPGA Has Processed All Nonces For The Work
		if ( nonce < last_nonce ) {
			applog(LOG_DEBUG, "%s: Overflow - Nonce=%08x, Last=%08x", ztex->repr, nonce, last_nonce);
			overflow = true;
			continue;
		}
		else
			last_nonce = nonce;

		hash_count = nonce;
		validNonces++;

		//
		// Golden Nonce 1 Check
		//
		golden_nonce1 = hdata.goldenNonce[0];
		
		if ((golden_nonce1 != 0) && (golden_nonce1 != last_golden1) && (golden_nonce1 != last_golden2)) {

//			applog(LOG_DEBUG, "%s Check Golden Nonce1 - %08X, Prior1 - %08X, Prior2 - %08X", ztex->repr, golden_nonce1, last_golden1, last_golden2);

//			if ( ztex_checkNonce(work, golden_nonce1) != 0) {
//				applog(LOG_WARNING, "%s HW Error GN1: %08X HSH: %08X - N: %08X, H: %08X, E: %08X", ztex->repr, golden_nonce1, ztex_checkNonce(work, golden_nonce1), nonce, hdata.hash7, ztex_checkNonce(work, nonce));
//				continue;
//			}		
		
			last_golden2 = last_golden1;
			last_golden1 = golden_nonce1;
			
			applog(LOG_DEBUG, "%s: Submitted Nonce %08x", ztex->repr, golden_nonce1);
			submit_nonce(thr, work, golden_nonce1);
		}

		//
		// Golden Nonce 2 Check
		//
		golden_nonce2 = hdata.goldenNonce[1];

		if ((golden_nonce2 != 0) && (golden_nonce2 != last_golden1) && (golden_nonce2 != last_golden2)) {

//			applog(LOG_DEBUG, "%s Check Golden Nonce2 - %08X, Prior1 - %08X, Prior2 - %08X", ztex->repr, golden_nonce2, last_golden1, last_golden1);

//			if ( ztex_checkNonce(work, golden_nonce2) != 0) {
//				applog(LOG_WARNING, "%s HW Error GN2: %08X HSH: %08X - N: %08X, H: %08X, E: %08X", ztex->repr, golden_nonce2, ztex_checkNonce(work, golden_nonce2), nonce, hdata.hash7, ztex_checkNonce(work, nonce));
//				continue;
//			}		

			last_golden2 = last_golden1;
			last_golden1 = golden_nonce2;
			
			applog(LOG_DEBUG, "%s: Submitted Nonce %08x", ztex->repr, golden_nonce2);
			submit_nonce(thr, work, golden_nonce2);
		}
		
		cgtime(&tv_end);
		timersub(&tv_end, &tv_start, &diff);
		if (diff.tv_sec > opt_scantime) {
			applog(LOG_DEBUG, "%s: time = %d sec, scan-time = %d sec", ztex->repr, diff.tv_sec, opt_scantime);
			break;
		}
	}

	ztex->nonceCheckValid = validNonces;
	ztex->errorCount[ztex->freqM] += errorCount;
	ztex->errorRate[ztex->freqM] = ztex->errorCount[ztex->freqM] / ztex->errorWeight[ztex->freqM] * (ztex->errorWeight[ztex->freqM] < 100 ? ztex->errorWeight[ztex->freqM] * 0.01 : 1.0);

	if (ztex->errorRate[ztex->freqM] > ztex->maxErrorRate[ztex->freqM])
		ztex->maxErrorRate[ztex->freqM] = ztex->errorRate[ztex->freqM];

	if (!ztex_updateFreq(ztex)) {
		return -1;
	}

	applog(LOG_DEBUG, "%s: Exit %1.8X", ztex->repr, hash_count);

	free_work(work);
	return hash_count;
}

static void ztex_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	if (cgpu->deven == DEV_ENABLED) {
		tailsprintf(buf, bufsiz, "%s-%d | ", cgpu->device_ztex->snString, cgpu->device_ztex->fpgaNum+1);
		tailsprintf(buf, bufsiz, "%5.1fMhz", (float)cgpu->device_ztex->freqM1 * (cgpu->device_ztex->freqM + 1));
	}
	else
		tailsprintf(buf, bufsiz, "       ");

	tailsprintf(buf, bufsiz, " | ");
}

static bool ztex_prepare(struct thr_info *thr)
{
	struct timeval now;
	struct cgpu_info *cgpu = thr->cgpu;
	struct libztex_device *ztex = cgpu->device_ztex;

	cgtime(&now);
	set_starttime(cgpu->init, &now);

	ztex_selectFpga(ztex);
	if (libztex_configureFpga(ztex) != 0) {
		libztex_resetFpga(ztex);
		ztex_releaseFpga(ztex);
		applog(LOG_ERR, "%s: Disabling!", thr->cgpu->device_ztex->repr);
		thr->cgpu->deven = DEV_DISABLED;
		return true;
	}
	
	// KRAMBLE Handle options, based on get_options in driver-icarus.c
	// Use as --ztex-clock freqM:freqMaxM
	// Multiple comma separated vaues are allowed eg 160:180,180:184

	{	// Bare block to isolate variables

		char err_buf[BUFSIZ+1];
		char buf[BUFSIZ+1];
		char *ptr, *comma, *colon, *colon2;
		size_t max;
		int i, tmp;

		int this_option_offset = ++option_offset;

		if (opt_ztex_clock == NULL)
				buf[0] = '\0';
		else {
			ptr = opt_ztex_clock;
			for (i = 0; i < this_option_offset; i++) {
				comma = strchr(ptr, ',');
				if (comma == NULL)
					break;
				ptr = comma + 1;
			}

			comma = strchr(ptr, ',');
			if (comma == NULL)
				max = strlen(ptr);
			else
				max = comma - ptr;

			if (max > BUFSIZ)
				max = BUFSIZ;
			strncpy(buf, ptr, max);
			buf[max] = '\0';
		}


		if (*buf) {
			colon = strchr(buf, ':');
			if (colon)
				*(colon++) = '\0';

			if (*buf) {
				tmp = atoi(buf);
				if (tmp >= 50 && tmp <= 250)
					ztex->freqM = ztex->freqMDefault = tmp/4 - 1;	// NB 4Mhz units
				else {
					sprintf(err_buf, "Invalid ztex-clock must be between 50 and 250", buf);
					quit(1, err_buf);
				}
			}

			if (colon && *colon) {
				tmp = atoi(colon);
				if (tmp >= 50 && tmp <= 250) {
					if (tmp/4 - 1 >= ztex->freqM)
						ztex->freqMaxM = tmp/4 - 1;	// NB 4Mhz units
					else
					{
						sprintf(err_buf, "Invalid ztex-clock max must be less than min", buf);
						quit(1, err_buf);
					}
				}
				else {
					sprintf(err_buf, "Invalid ztex-clock must be between 50 and 250", buf);
					quit(1, err_buf);
				}
			}
		}
	
	}	// End bare block
	
	
	ztex->freqM = ztex->freqMaxM+1;		// KRAMBLE is in original
	// ztex_updateFreq(ztex);			// KRAMBLE Was already commented out in original

	libztex_setFreq(ztex, ztex->freqMDefault);
	ztex_releaseFpga(ztex);
	applog(LOG_DEBUG, "%s: prepare", ztex->repr);
	return true;
}

static void ztex_shutdown(struct thr_info *thr)
{
	if (thr->cgpu->device_ztex != NULL) {
		if (thr->cgpu->device_ztex->fpgaNum == 0)
			pthread_mutex_destroy(&thr->cgpu->device_ztex->mutex);  
		applog(LOG_DEBUG, "%s: shutdown", thr->cgpu->device_ztex->repr);
		libztex_destroy_device(thr->cgpu->device_ztex);
		thr->cgpu->device_ztex = NULL;
	}
}

static void ztex_disable(struct thr_info *thr)
{
	struct cgpu_info *cgpu;

	applog(LOG_ERR, "%s: Disabling!", thr->cgpu->device_ztex->repr);
	cgpu = get_devices(thr->cgpu->device_id);
	cgpu->deven = DEV_DISABLED;
	ztex_shutdown(thr);
}

static void ztex_identify(struct cgpu_info *cgpu)
{
	return;
}

static char *ztex_set(struct cgpu_info *cgpu, char *option, char *setting, char *replybuf)
{
	return NULL;
}

struct device_drv ztex_drv = {
	.drv_id = DRIVER_ztex,
	.dname = "Ztex",
	.name = "ZTX",
	.drv_detect = ztex_detect,
	.hash_work = &hash_driver_work,
//	.get_api_stats = ztex_api_stats,
	.get_statline_before = ztex_statline_before,
	.set_device = ztex_set,
	.identify_device = ztex_identify,
	.thread_prepare = ztex_prepare,
	.scanwork = ztex_scanwork,
	.thread_shutdown = ztex_shutdown,
};

