/*
 * Copyright 2012-2013 Andrew Smith
 * Copyright 2012 Xiangfu <xiangfu@openmobilefree.com>
 * Copyright 2013 Con Kolivas <kernel@kolivas.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include "config.h"

#ifdef WIN32
#include <windows.h>
#endif

#include "compat.h"
#include "miner.h"
#include "fpgautils.h"

// The serial I/O speed - Linux uses a define 'B115200' in bits/termios.h
#define SERIAL_IO_SPEED 115200

// The size of a successful nonce read
#define SERIAL_READ_SIZE 4

// 10ths of a second to wait for each serial byte read
#define SERIAL_READ_TIMEOUT 1

// Default # Seconds To Hash Work (Override using --scan-time option)
#define SERIAL_FPGA_TIMEOUT 10

// Inverse Of Default H/s
#define DEFAULT_HASH_PER_SEC 0.000001	// 1MH/s

// Function Prototypes
static void serial_fpga_close(struct thr_info *thr);
static bool serial_fpga_detect_one(const char *devpath);
static void serial_fpga_detect(bool __maybe_unused hotplug);
static bool serial_fpga_prepare(__maybe_unused struct thr_info *thr);
static int64_t serial_fpga_scanwork(struct thr_info *thr);
static void serial_fpga_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu);
static void serial_fpga_shutdown(__maybe_unused struct thr_info *thr);
static void serial_fpga_identify(struct cgpu_info *cgpu);
static char *serial_fpga_set(struct cgpu_info *cgpu, char *option, char *setting, char *replybuf);


struct FPGA_INFO {
	int device_fd;
	int timeout;
	double Hs;		// Seconds Per Hash
};


static void serial_fpga_close(struct thr_info *thr)
{
	struct cgpu_info *serial_fpga = thr->cgpu;
	struct FPGA_INFO *info = serial_fpga->device_data;

	applog(LOG_DEBUG, "serial_fpga_close...");
	
	close(info->device_fd);
	info->device_fd = -1;
	
}

static bool serial_fpga_detect_one(const char *devpath)
{
	struct FPGA_INFO *info;
	struct cgpu_info *serial_fpga;
	int fd;

	applog(LOG_DEBUG, "serial_fpga_detect_one...");
	
	fd = serial_open(devpath, SERIAL_IO_SPEED, SERIAL_READ_TIMEOUT, true);
	if (fd == -1) {
		applog(LOG_ERR, "Serial FPGA Detect: Failed to open %s", devpath);
		return false;
	}

//
	applog(LOG_DEBUG, "Serial FPGA Detect: Test skipped for: %s", devpath);
	close(fd);
//

	serial_fpga = calloc(1, sizeof(struct cgpu_info));
	if (unlikely(!serial_fpga))
		quit(1, "Failed to calloc cgpu for %s in usb_alloc_cgpu", devpath);
	serial_fpga->drv = &serial_fpga_drv;
	serial_fpga->device_path = strdup(devpath);
	serial_fpga->threads = 1;
	add_cgpu(serial_fpga);
	
	info = (struct FPGA_INFO *)calloc(1, sizeof(struct FPGA_INFO));
	if (unlikely(!info))
		quit(1, "Failed to malloc FPGA_INFO");
	serial_fpga->device_data = (void *)info;
	
	applog(LOG_INFO, "Found Serial FPGA at %s, mark as %d",	devpath, serial_fpga->device_id);

	// Initialise everything to zero for a new device
	memset(info, 0, sizeof(struct FPGA_INFO));

	info->device_fd = -1;
	info->Hs = DEFAULT_HASH_PER_SEC;
	
	if (opt_scantime > 0)
		info->timeout = opt_scantime;
	else
		info->timeout = SERIAL_FPGA_TIMEOUT;
	
	return true;
}

static void serial_fpga_detect(bool __maybe_unused hotplug)
{
	serial_detect(&serial_fpga_drv, serial_fpga_detect_one);
}

static bool serial_fpga_prepare(__maybe_unused struct thr_info *thr)
{
	struct cgpu_info *serial_fpga = thr->cgpu;
	struct FPGA_INFO *info = serial_fpga->device_data;

	applog(LOG_DEBUG, "serial_fpga_prepare...");

	if (info->device_fd == -1) {
		
		applog(LOG_INFO, "Open Serial FPGA on %s", serial_fpga->device_path);
		info->device_fd = serial_open(serial_fpga->device_path, SERIAL_IO_SPEED, SERIAL_READ_TIMEOUT, false);
		if (unlikely(info->device_fd == -1)) {
			applog(LOG_ERR, "Failed to open Serial FPGA on %s",
				   serial_fpga->device_path);
			return false;
		}
	}
	
	return true;
}

static int64_t serial_fpga_scanwork(struct thr_info *thr)
{
	struct cgpu_info *serial_fpga;
	int fd;
	int ret;

	struct FPGA_INFO *info;

	unsigned char ob_bin[44], nonce_buf[SERIAL_READ_SIZE];
	char *ob_hex;
	uint32_t nonce;
	int64_t hash_count;
	struct timeval tv_start, tv_finish, elapsed, tv_end, diff;
	int curr_hw_errors, i, j;
	uint32_t * ob;
	ob = (uint32_t *)ob_bin;

	int count;
	double Hs, W, fullnonce;
	int read_count;
	int64_t estimate_hashes;
	uint32_t values;
	int64_t hash_count_range;

	struct work *work;

	applog(LOG_DEBUG, "serial_fpga_scanwork...");
	
	if (thr->cgpu->deven == DEV_DISABLED)
		return -1;
	

	serial_fpga = thr->cgpu;
	info = serial_fpga->device_data;
	work = get_work(thr, thr->id);
	
	if (info->device_fd == -1) {
		
		applog(LOG_INFO, "Attemping to Reopen Serial FPGA on %s", serial_fpga->device_path);
		fd = serial_open(serial_fpga->device_path, SERIAL_IO_SPEED, SERIAL_READ_TIMEOUT, false);
		if (unlikely(-1 == fd)) {
			applog(LOG_ERR, "Failed to open Serial FPGA on %s",
				   serial_fpga->device_path);
			return -1;
		}
		else
			info->device_fd = fd;
	}

	fd = info->device_fd;
	
	memset(ob_bin, 0, sizeof(ob_bin));

//  Currently, extra nonces are not supported
//
	memset((unsigned char*)work->data + 144, 0, 12);
//
//
	
	calc_midstate(work);

	memcpy(ob_bin, work->midstate, 32);			// Midstate
	memcpy(ob_bin + 32, work->data + 128, 12);	// Remaining Bytes From Block Header

	// Send Bytes To FPGA In Reverse Order
	unsigned char swap[44];
	uint32_t * sw;
	sw = (uint32_t *)swap;
	for (j=0; j<8; j++) {
		sw[j] = swab32(ob[j]);
	}
	
	memcpy(swap + 32, ob_bin + 32, 12);

	for (j=0; j<44; j++) {
		ob_bin[j] = swap[j];
	}
	
//unsigned char* b = (unsigned char*)(ob_bin);
//applog(LOG_WARNING, "swap: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[28],b[29],b[30],b[31],b[32],b[33],b[34],b[35],b[36],b[37],b[38],b[39],b[40],b[41],b[42],b[43]);
//applog(LOG_WARNING, "swap: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14],b[15],b[16],b[17],b[18],b[19],b[20],b[21],b[22],b[23],b[24],b[25],b[26],b[27],b[28],b[29],b[30],b[31],b[32],b[33],b[34],b[35],b[36],b[37],b[38],b[39],b[40],b[41],b[42],b[43]);

	
//#ifndef WIN32
//	tcflush(fd, TCOFLUSH);
//#endif

	// Send Data To FPGA
	ret = write(fd, ob_bin, sizeof(ob_bin));

	if (ret != sizeof(ob_bin)) {
			applog(LOG_ERR, "%s%i: Serial Send Error (ret=%d)", serial_fpga->drv->name, serial_fpga->device_id, ret);
		serial_fpga_close(thr);
		dev_error(serial_fpga, REASON_DEV_COMMS_ERROR);
		return 0;
	}

	if (opt_debug) {
		ob_hex = bin2hex(ob_bin, sizeof(ob_bin));
		applog(LOG_DEBUG, "Serial FPGA %d sent: %s",
			serial_fpga->device_id, ob_hex);
		free(ob_hex);
	}

	elapsed.tv_sec = 0;
	elapsed.tv_usec = 0;
	cgtime(&tv_start);

	applog(LOG_DEBUG, "%s%i: Begin Scan For Nonces", serial_fpga->drv->name, serial_fpga->device_id);
	while (thr && !thr->work_restart) {

		memset(nonce_buf,0,4);
	
		// Check Serial Port For 1/10 Sec For Nonce  
		ret = read(fd, nonce_buf, SERIAL_READ_SIZE);

		// Calculate Elapsed Time
		cgtime(&tv_end);
		timersub(&tv_end, &tv_start, &elapsed);


		if (ret == 0) {		// No Nonce Found
			if (elapsed.tv_sec > info->timeout) {
				applog(LOG_DEBUG, "%s%i: End Scan For Nonces - Time = %d sec", serial_fpga->drv->name, serial_fpga->device_id, elapsed.tv_sec);
				break;
			}
			continue;
		}
		else if (ret < SERIAL_READ_SIZE) {
			applog(LOG_ERR, "%s%i: Serial Read Error (ret=%d)", serial_fpga->drv->name, serial_fpga->device_id, ret);
			serial_fpga_close(thr);
			dev_error(serial_fpga, REASON_DEV_COMMS_ERROR);
			break;
		}

		memcpy((char *)&nonce, nonce_buf, SERIAL_READ_SIZE);
		
#if !defined (__BIG_ENDIAN__) && !defined(MIPSEB)
		nonce = swab32(nonce);
#endif

		curr_hw_errors = serial_fpga->hw_errors;

		applog(LOG_INFO, "%s%i: Nonce Found - %08X (%5.1fMhz)", serial_fpga->drv->name, serial_fpga->device_id, nonce, (double)(1/(info->Hs * 1000000)));
		submit_nonce(thr, work, nonce);

		// Update Hashrate
		if (serial_fpga->hw_errors == curr_hw_errors)
			info->Hs = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000)) / (double)nonce;

	}

	// Estimate Number Of Hashes
	hash_count = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000)) / info->Hs;
	
	free_work(work);
	return hash_count;
}

static void serial_fpga_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	if (cgpu->deven == DEV_ENABLED) {
		tailsprintf(buf, bufsiz, "%s-%d | ", cgpu->drv->dname, cgpu->device_id);
		tailsprintf(buf, bufsiz, "        ");
	}
	else
		tailsprintf(buf, bufsiz, "       ");

	tailsprintf(buf, bufsiz, " | ");
}

static void serial_fpga_shutdown(__maybe_unused struct thr_info *thr)
{
	applog(LOG_DEBUG, "serial_fpga_shutdown...");

	serial_fpga_close(thr);
}

static void serial_fpga_identify(struct cgpu_info *cgpu)
{
	applog(LOG_DEBUG, "serial_fpga_identify...");
}

static char *serial_fpga_set(struct cgpu_info *cgpu, char *option, char *setting, char *replybuf)
{
	applog(LOG_DEBUG, "serial_fpga_set...");
	
	sprintf(replybuf, "Unknown option: %s", option);
	return replybuf;
}

struct device_drv serial_fpga_drv = {
	.drv_id = DRIVER_serial_fpga,
	.dname = "SerialFPGA",
	.name = "SRL",
	.drv_detect = serial_fpga_detect,
	.hash_work = &hash_driver_work,
	.get_statline_before = serial_fpga_statline_before,
	.set_device = serial_fpga_set,
	.identify_device = serial_fpga_identify,
	.thread_prepare = serial_fpga_prepare,
	.scanwork = serial_fpga_scanwork,
	.thread_shutdown = serial_fpga_shutdown,
};
