/* STLink download interface for Linux. */
/*
  This program interacts with the STMicro STLink USB interface for STM8
  and STM32 devices.

  Portions of this code were informed by the stlink-access-test.c code.
  It carried the following self-contradictory notice:
  Copyright (c) 2010 "Capt'ns Missing Link" Authors. All rights reserved.
  Use of this source code is governed by a BSD-style
  license that can be found in the LICENSE file.
  ..provided by Oliver Spencer (OpenOCD) for use in a GPL compatible license.
  Get a clue guys, learn why "(c)" and "all rights reserved" are meaningless.
  And if you use GPL stuff, the result is GPL.  Along with cribbing big
  chunks of text from standards documents.

  References
   ST Micro Applications Notes
   AN3154 Specification of the CAN bootloader protocol
   AN3155 Specification of the USART bootloader protocol
   AN3156 Specification of the USB DFU (Direct Firmware Upload) protocol
   http://www.usb.org/developers/devclass_docs/DFU_1.1.pdf

 Related documents

 Notes:
 gcc -O0 -g3 -Wall -c -std=gnu99 -o stlink-download.o stlink-download.c
 gcc  -o stlink-download stlink-download.o -lsgutils2

 Code format ~ TAB = 8, K&R, linux kernel source, golang oriented
 Tested compatibility: linux, gcc >= 4.3.3

 The communication is based on standard USB mass storage device
 BOT (Bulk Only Transfer)
 - Endpoint 1: BULK_IN, 64 bytes max
 - Endpoint 2: BULK_OUT, 64 bytes max

 All CBW transfers are ordered with the LSB (byte 0) first (little endian).
 Any command must be answered before sending the next command.
 Each USB transfer must complete in less than 1s.

 SB Device Class Definition for Mass Storage Devices:
 www.usb.org/developers/devclass_docs/usbmassbulk_10.pdf

 dt		- Data Transfer (IN/OUT)
 CBW 		- Command Block Wrapper
 CSW		- Command Status Wrapper
 RFU		- Reserved for Future Use
 scsi_pt	- SCSI pass-through
 sg		- SCSI generic

 * usb-storage.quirks
 http://git.kernel.org/?p=linux/kernel/git/torvalds/linux-2.6.git;a=blob_plain;f=Documentation/kernel-parameters.txt
 Each entry has the form VID:PID:Flags where VID and PID are Vendor and Product
 ID values (4-digit hex numbers) and Flags is a set of characters, each corresponding
 to a common usb-storage quirk flag as follows:

 a = SANE_SENSE (collect more than 18 bytes of sense data);
 b = BAD_SENSE (don't collect more than 18 bytes of sense data);
 c = FIX_CAPACITY (decrease the reported device capacity by one sector);
 h = CAPACITY_HEURISTICS (decrease the reported device capacity by one sector if the number is odd);
 i = IGNORE_DEVICE (don't bind to this device);
 l = NOT_LOCKABLE (don't try to lock and unlock ejectable media);
 m = MAX_SECTORS_64 (don't transfer more than 64 sectors = 32 KB at a time);
 o = CAPACITY_OK (accept the capacity reported by the device);
 r = IGNORE_RESIDUE (the device reports bogus residue values);
 s = SINGLE_LUN (the device has only one Logical Unit);
 w = NO_WP_DETECT (don't test whether the medium is write-protected).

 Example: quirks=0419:aaf5:rl,0421:0433:rc
 http://permalink.gmane.org/gmane.linux.usb.general/35053

 modprobe -r usb-storage && modprobe usb-storage quirks=483:3744:l

 Equivalently, you can add a line saying

 options usb-storage quirks=483:3744:l

 to your /etc/modprobe.conf or /etc/modprobe.d/local.conf (or add the "quirks=..."
 part to an existing options line for usb-storage).
 */

#define __USE_GNU
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

/* We use the SCSI-Generic library sgutils2
 * To install do 'sudo apt-get install libsgutils2-dev'
 */
#include <scsi/sg_lib.h>
#include <scsi/sg_pt.h>

static const char version_msg[] =
"STLink firmware download $Id$  Copyright Donald Becker";

static const char usage_msg[] =
	"\nUsage: %s /dev/sg0 ...\n\n"
	"Note: The stlink firmware violates the USB standard.\n"
	" If you plug-in the discovery's stlink, wait a several\n"
	" minutes to let the kernel driver swallow the broken device.\n"
	" Watch:\ntail -f /var/log/messages\n"
	" This command sequence can shorten the waiting time and fix some issues.\n"
	" Unplug the stlink and execute once as root:\n"
	"modprobe -r usb-storage && modprobe usb-storage quirks=483:3744:lrwsro\n"
	"\n";

static char short_opts[] = "BC:D:U:huvV";
static struct option long_options[] = {
    {"blink",	0, NULL, 	'B'},
    {"check",	1, NULL, 	'C'},
    {"verify",	1, NULL, 	'C'},
    {"download", 1, NULL, 	'D'},
    {"upload",	1, NULL, 	'U'},
    {"help",	0, NULL,	'h'},	/* Print a long usage message. */
    {"usage",	0, NULL,	'u'},
    {"verbose", 0, NULL,	'v'},	/* Report each action taken.  */
    {"version", 0, NULL,	'V'},	/* Emit version information.  */
    {NULL,		0, NULL,	0},
};

int verbose = 0;



/* There are a bunch of IDs inside the chip.
 * STMicro wants the debugger writers to 'lock' their code to only work with
 * known chips.  Hmmm, good so that you have to update proprietary tools, but
 * not the approach for us.
 * The MCU DEVICE ID at 0xE0042000.
 */
#define DBGMCU_IDCODE 0xE0042000.
struct stm_device_id {
	uint32_t core_id;
	uint32_t flash_base, flash_size;
	uint32_t sram_base, sram_size;
} stm_devids[] = {
	{0x1ba01477, },				/* STM32F100 on Discovery card. */
	{0, 0, 0,}
};

/* We check that we are talking to a STLink device by the verifying the
 * the Vendor and Product IDentification numbers.
 * These are common across bus types e.g. PCI, USB, SCSI.
 */
#define USB_ST_VID		0x0483
#define USB_STLINK_PID	0x3744

/* Device access is through a SCSI Command Descriptor Block (CDB).
 *  http://en.wikipedia.org/wiki/SCSI_CDB
 * The STLink uses a 10 byte block.
 */
#define RDWR		0
#define RO		1
#define SG_TIMEOUT_SEC	1 // actually 1 is about 2 sec
// Each CDB can be a total of 6, 10, 12, or 16 bytes, later version
// of the SCSI standard also allow for variable-length CDBs (min. CDB is 6).
// the stlink needs max. 10 bytes.
#define CDB_6		6
#define CDB_10		10
#define CDB_12		12
#define CDB_16		16

#define CDB_SL		10

// Query data flow direction.
#define Q_DATA_OUT	0
#define Q_DATA_IN	1

// The SCSI Request Sense command is used to obtain sense data
// (error information) from a target device.
// http://en.wikipedia.org/wiki/SCSI_Request_Sense_Command
#define SENSE_BUF_LEN		32

// Max data transfer size.
// 6kB = max mem32_read block, 8kB sram
//#define Q_BUF_LEN	96
#define Q_BUF_LEN	1024 * 100


// STLINK_DEBUG_RESETSYS, etc:
#define STLINK_OK			0x80
#define STLINK_FALSE			0x81
#define STLINK_CORE_RUNNING		0x80
#define STLINK_CORE_HALTED		0x81
#define STLINK_CORE_STAT_UNKNOWN	-1

enum STLink_Cmds {
	STLinkGetVersion=0xF1,
	STLinkDebugCommand=0xF2,
	STLinkDFUCommand=0xF3,		/* Device/Direct Firmware Update */
	STLinkGetCurrentMode=0xF5,
};

enum STLink_Device_Modes {
	STLinkDevMode_Unknown=-1,
	STLinkDevMode_DFU=0,
	STLinkDevMode_Mass=1,
	STLinkDevMode_Debug=2,
};
	
#define STLINK_GET_VERSION		0xf1
#define STLINK_GET_CURRENT_MODE	0xf5

#define STLINK_DEBUG_COMMAND		0xF2
#define STLINK_DFU_COMMAND		0xF3
#define STLINK_DFU_EXIT		0x07

// STLINK_GET_CURRENT_MODE  0xF5
#define STLINK_DEV_DFU_MODE		0x00
#define STLINK_DEV_MASS_MODE		0x01
#define STLINK_DEV_DEBUG_MODE		0x02
#define STLINK_DEV_UNKNOWN_MODE	-1

// JTAG mode cmds
#define STLINK_DEBUG_ENTER		0x20
#define STLINK_DEBUG_EXIT		0x21
#define STLINK_DEBUG_READCOREID	0x22
#define STLINK_DEBUG_GETSTATUS		0x01
#define STLINK_DEBUG_FORCEDEBUG	0x02
#define STLINK_DEBUG_RESETSYS		0x03
#define STLINK_DEBUG_READALLREGS	0x04
/* Read and write the ARM core register set. */
#define STLINK_DEBUG_READREG		0x05
#define STLINK_DEBUG_WRITEREG		0x06
#define STLINK_DEBUG_READMEM_32BIT	0x07
#define STLINK_DEBUG_WRITEMEM_32BIT	0x08
#define STLINK_DEBUG_RUNCORE		0x09
#define STLINK_DEBUG_STEPCORE		0x0a
#define STLINK_DEBUG_SETFP			0x0b
#define STLINK_DEBUG_WRITEMEM_8BIT	0x0d
#define STLINK_DEBUG_CLEARFP		0x0e
#define STLINK_DEBUG_WRITEDEBUGREG	0x0f
#define STLINK_DEBUG_ENTER_SWD		0xa3
#define STLINK_DEBUG_ENTER_JTAG	0x00

/* The ARM processor core registers, in their transfer order.
 * Index  Register
 * 0..15  r0..r15
 * 16	XPSR
 * 17/18	Main_SP/Process_SP
 * 19/20	RW,RW2
*/
struct ARMcoreRegs {
	uint32_t r[16];
	uint32_t xpsr;
	uint32_t main_sp;
	uint32_t process_sp;
	uint32_t rw;
	uint32_t rw2;
} __attribute__((packed));

typedef uint32_t stm32_addr_t;

struct stlink {
	int sg_fd;
	int do_scsi_pt_err;
	// sg layer verboseness: 0 for no debug info, 10 for lots
	int verbose;

	unsigned char cdb_cmd_blk[CDB_SL];

	// Data transferred from or to device
	unsigned char q_buf[Q_BUF_LEN];
	int q_len;
	int q_data_dir; // Q_DATA_IN, Q_DATA_OUT
	// the start of the query data in the device memory space
	uint32_t q_addr;

	// Sense (error information) data
	unsigned char sense_buf[SENSE_BUF_LEN];

	uint32_t st_vid;
	uint32_t stlink_pid;
	uint32_t stlink_v;
	uint32_t jtag_v;
	uint32_t swim_v;
	uint32_t core_id;			/* Always 0x1BA01477, must be read to unlock. */

	struct ARMcoreRegs reg;
	int core_stat;

	/* medium density stm32 flash settings */
#define STM32_FLASH_BASE 0x08000000
#define STM32_FLASH_SIZE (128 * 1024)
#define STM32_FLASH_PGSZ 1024
	stm32_addr_t flash_base;
	size_t flash_size;
	size_t flash_pgsz;

	/* in flash system memory */
#define STM32_SYSTEM_BASE 0x1ffff000
#define STM32_SYSTEM_SIZE (2 * 1024)
	stm32_addr_t sys_base;
	size_t sys_size;

	/* sram settings */
#define STM32_SRAM_BASE 0x20000000
#define STM32_SRAM_SIZE (8 * 1024)
	stm32_addr_t sram_base;
	size_t sram_size;
};

static void stlink_q(struct stlink* sl);


/* Show text only when in debug mode. */
static void D(struct stlink *sl, char *txt)
{
	if (sl->verbose > 1)
		fputs(txt, stderr);
}

static void delay(int ms)
{
	usleep(1000 * ms);
}

// Endianness
// http://www.ibm.com/developerworks/aix/library/au-endianc/index.html
// const int i = 1;
// #define is_bigendian() ( (*(char*)&i) == 0 )
static inline unsigned int is_bigendian(void) {
	static volatile const unsigned int i = 1;
	return *(volatile const char*) &i == 0;
}

static void write_uint32(unsigned char* buf, uint32_t ui) {
	if (!is_bigendian()) { // le -> le (don't swap)
		buf[0] = ((unsigned char*) &ui)[0];
		buf[1] = ((unsigned char*) &ui)[1];
		buf[2] = ((unsigned char*) &ui)[2];
		buf[3] = ((unsigned char*) &ui)[3];
	} else {
		buf[0] = ((unsigned char*) &ui)[3];
		buf[1] = ((unsigned char*) &ui)[2];
		buf[2] = ((unsigned char*) &ui)[1];
		buf[3] = ((unsigned char*) &ui)[0];
	}
}

static void write_uint16(unsigned char* buf, uint16_t ui)
{
	if (!is_bigendian()) { // le -> le (don't swap)
		buf[0] = ((unsigned char*) &ui)[0];
		buf[1] = ((unsigned char*) &ui)[1];
	} else {
		buf[0] = ((unsigned char*) &ui)[1];
		buf[1] = ((unsigned char*) &ui)[0];
	}
}

static uint32_t read_uint32(const unsigned char *c, const int pt)
{
	uint32_t ui;
	char *p = (char *) &ui;

	if (!is_bigendian()) { // le -> le (don't swap)
		p[0] = c[pt];
		p[1] = c[pt + 1];
		p[2] = c[pt + 2];
		p[3] = c[pt + 3];
	} else {
		p[0] = c[pt + 3];
		p[1] = c[pt + 2];
		p[2] = c[pt + 1];
		p[3] = c[pt];
	}
	return ui;
}

/* Clear the command descriptor block, leaving a default command.
 * This is mostly pointless since the individual commands initialize
 * the required fields.
 */
static void clear_cdb(struct stlink *sl)
{
	memset(sl->cdb_cmd_blk, 0, sizeof(sl->cdb_cmd_blk));
	// set default
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->q_data_dir = Q_DATA_IN;
	return;
}

static void clear_q_buf(struct stlink *sl)
{
	memset(sl->q_buf, 0, sizeof(sl->q_buf));
	return;
}

static struct stlink* stlink_open(const char *dev_name, const int verbose)
{
	int sg_fd;

	if (verbose > 2)
		fprintf(stderr, " stlink_open [%s] ***\n", dev_name);
	sg_fd = scsi_pt_open_device(dev_name, RDWR, verbose);
	if (sg_fd < 0) {
		fprintf(stderr, "Error opening the SCSI device '%s': %s\n",
				dev_name, safe_strerror(-sg_fd));
		return NULL;
	}

	struct stlink *sl = malloc(sizeof(struct stlink));
	if (sl == NULL) {
		fprintf(stderr, "struct stlink: out of memory\n");
		return NULL;
	}

	sl->sg_fd = sg_fd;
	sl->verbose = verbose;
	sl->core_stat = STLINK_CORE_STAT_UNKNOWN;
	sl->core_id = 0;
	sl->q_addr = 0;
	clear_q_buf(sl);

	/* flash memory settings */
	sl->flash_base = STM32_FLASH_BASE;
	sl->flash_size = STM32_FLASH_SIZE;
	sl->flash_pgsz = STM32_FLASH_PGSZ;

	/* system memory */
	sl->sys_base = STM32_SYSTEM_BASE;
	sl->sys_size = STM32_SYSTEM_SIZE;

	/* sram memory settings */
	sl->sram_base = STM32_SRAM_BASE;
	sl->sram_size = STM32_SRAM_SIZE;

	return sl;
}

/* Close the device and free the allocated memory. */
void stlink_close(struct stlink *sl)
{
	D(sl, "\n*** stlink_close ***\n");
	if (sl) {
		scsi_pt_close_device(sl->sg_fd);
		free(sl);
	}
	return;
}

/* Execute a general-form command, with arbitrary parameters.
 * The command has already been written into sl->cdb_cmd_blk[]. */
void st_ecmd(struct stlink *sl, int q_len)
{
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->q_data_dir = Q_DATA_IN;
	sl->q_len = q_len;
	sl->q_addr = 0;
	stlink_q(sl);
	return;
}

/* Execute a regular-form STLink command.
 * This is the generic operation for most simple commands.
 */
int stlink_cmd(struct stlink *sl, uint8_t st_cmd1, uint8_t st_cmd2, int q_len)
{
#if 0
	memset(sl->cdb_cmd_blk, 0x55, sizeof(sl->cdb_cmd_blk));
#endif
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = st_cmd1;
	sl->cdb_cmd_blk[2] = st_cmd2;
	sl->q_data_dir = Q_DATA_IN;
	sl->q_len = q_len;
	sl->q_addr = 0;
	memset(sl->q_buf, 0x55555555, q_len+12);
	stlink_q(sl);
	if (q_len == 2)
		return *(uint16_t *)sl->q_buf;
	else if (q_len == 4)
		return read_uint32(sl->q_buf, 0);
	return 0;
}

#define stl_enter_debug(sl) stlink_cmd(sl, STLINK_DEBUG_FORCEDEBUG, 0, 2)
#define stl_reset(sl) stlink_cmd(sl, STLINK_DEBUG_RESETSYS, 0, 2)
#define stl_get_allregs(sl) stlink_cmd(sl, STLINK_DEBUG_READALLREGS, 0, 84)
#define stl_state_run(sl) stlink_cmd(sl, STLINK_DEBUG_RUNCORE, 0, 2)
#define stl_step(sl) stlink_cmd(sl, STLINK_DEBUG_STEPCORE, 0, 2)
#define stl_clear_bp(sl, fp_nr) stlink_cmd(sl, STLINK_DEBUG_CLEARFP, fp_nr, 2)
#define stl_exit_debug_mode(sl) stlink_cmd(sl, STLINK_DEBUG_EXIT, 0, 0)
#define stl_enter_SWD_mode(sl) \
	stlink_cmd(sl, STLINK_DEBUG_ENTER, STLINK_DEBUG_ENTER_SWD, 0)
#define stl_enter_JTAG_mode(sl) \
	stlink_cmd(sl, STLINK_DEBUG_ENTER, STLINK_DEBUG_ENTER_JTAG, 0)

/* These need extra data. */
#define stl_put_1reg(sl, reg_idx, reg_val)	{	\
		write_uint32(sl->cdb_cmd_blk + 3, (reg_val));					\
		stlink_cmd(sl, STLINK_DEBUG_WRITEREG, (reg_idx), 2); \
	}
#define stl_set_bp(sl, fp_nr) stlink_ecmd(sl, STLINK_DEBUG_SETFP, fp_nr, 2)
/* And these returns data. */
#define stl_get_status(sl) stlink_cmd(sl, STLINK_DEBUG_GETSTATUS, 0, 2)
#define stl_get_status2(sl) \
	(stlink_cmd(sl, STLINK_DEBUG_GETSTATUS, 0, 2), sl->q_buf[0]);
#define stl_get_core_id(sl) (stlink_cmd(sl, STLINK_DEBUG_READCOREID, 0, 4), \
							 read_uint32(sl->q_buf, 0))
/* Read a single ARM register.  See 'struct ARMcoreRegs' for the index. */
#define stl_get_1reg(sl, reg_idx) \
	(stlink_cmd(sl, STLINK_DEBUG_READREG, (reg_idx), 4), read_uint32(sl->q_buf, 0))



//TODO rewrite/cleanup, save the error in sl
static void stlink_confirm_inq(struct stlink *sl, struct sg_pt_base *ptvp)
{
	const int e = sl->do_scsi_pt_err;
	int duration;

	if (e < 0) {
		fprintf(stderr, "scsi_pt error: pass through os error: %s\n",
			safe_strerror(-e));
		return;
	} else if (e == SCSI_PT_DO_BAD_PARAMS) {
		fprintf(stderr, "scsi_pt error: bad pass through setup\n");
		return;
	} else if (e == SCSI_PT_DO_TIMEOUT) {
		fprintf(stderr, "  pass through timeout\n");
		return;
	}

	duration = get_scsi_pt_duration_ms(ptvp);
	if ((sl->verbose > 1) && (duration >= 0))
		fprintf(stderr, "      duration=%d ms\n", duration);

	// XXX stlink fw sends broken residue, so ignore it and use the known q_len
	// "usb-storage quirks=483:3744:r"
	// forces residue to be ignored and calculated, but this causes abort if
	// data_len = 0 and by some other data_len values.

	const int resid = get_scsi_pt_resid(ptvp);
	const int dsize = sl->q_len - resid;

	const int cat = get_scsi_pt_result_category(ptvp);
	char buf[512];
	unsigned int slen;

	switch (cat) {
	case SCSI_PT_RESULT_GOOD:
		if (sl->verbose && (resid > 0))
			fprintf(stderr, "      notice: requested %d bytes but "
				"got %d bytes, ignore [broken] residue = %d\n",
				sl->q_len, dsize, resid);
		break;
	case SCSI_PT_RESULT_STATUS:
		if (sl->verbose) {
			sg_get_scsi_status_str(
				get_scsi_pt_status_response(ptvp), sizeof(buf),
				buf);
			fprintf(stderr, "  scsi status: %s\n", buf);
		}
		return;
	case SCSI_PT_RESULT_SENSE:
		slen = get_scsi_pt_sense_len(ptvp);
		if (sl->verbose) {
			sg_get_sense_str("", sl->sense_buf, slen, (sl->verbose
				> 1), sizeof(buf), buf);
			fprintf(stderr, "%s", buf);
		}
		if (sl->verbose && (resid > 0)) {
			if ((sl->verbose) || (sl->q_len > 0))
				fprintf(stderr, "    requested %d bytes but "
					"got %d bytes\n", sl->q_len, dsize);
		}
		return;
	case SCSI_PT_RESULT_TRANSPORT_ERR:
		if (sl->verbose) {
			get_scsi_pt_transport_err_str(ptvp, sizeof(buf), buf);
			// http://tldp.org/HOWTO/SCSI-Generic-HOWTO/x291.html
			// These codes potentially come from the firmware on a host adapter
			// or from one of several hosts that an adapter driver controls.
			// The 'host_status' field has the following values:
			//	[0x07] Internal error detected in the host adapter.
			// This may not be fatal (and the command may have succeeded).
			fprintf(stderr, "  transport: %s", buf);
		}
		return;
	case SCSI_PT_RESULT_OS_ERR:
		if (sl->verbose) {
			get_scsi_pt_os_err_str(ptvp, sizeof(buf), buf);
			fprintf(stderr, "  os: %s", buf);
		}
		return;
	default:
		fprintf(stderr, "  unknown pass through result category (%d)\n", cat);
	}
}

/* Queue a SCSI command to the STLink.
 * Most of the work is done in the SCSI Pass Through library.
 */
static void stlink_q(struct stlink* sl)
{
	struct sg_pt_base *ptvp;
	int i;

	ptvp = construct_scsi_pt_obj();
	if (NULL == ptvp) {
		fprintf(stderr, "construct_scsi_pt_obj: out of memory\n");
		return;
	}
	if (sl->verbose > 2) {
		fprintf(stderr, "CDB[");
		for (i = 0; i < sizeof(sl->cdb_cmd_blk); i++)
			fprintf(stderr, " 0x%2.2x", sl->cdb_cmd_blk[i]);
		fprintf(stderr, "]\n");
	}

	set_scsi_pt_cdb(ptvp, sl->cdb_cmd_blk, sizeof(sl->cdb_cmd_blk));

	/* The SCSI Request Sense (error information) command is used for
	 * responses.  Provide a buffer for it.
	 * http://en.wikipedia.org/wiki/SCSI_Request_Sense_Command
	 */
	set_scsi_pt_sense(ptvp, sl->sense_buf, sizeof(sl->sense_buf));

	/* Set a buffer to be used for data transferred from device */
	if (sl->q_data_dir == Q_DATA_IN) {
		//clear_q_buf(sl);
		set_scsi_pt_data_in(ptvp, sl->q_buf, sl->q_len);
	} else {
		set_scsi_pt_data_out(ptvp, sl->q_buf, sl->q_len);
	}
	// Executes SCSI command (or at least forwards it to lower layers).
	sl->do_scsi_pt_err = do_scsi_pt(ptvp, sl->sg_fd, SG_TIMEOUT_SEC,
		sl->verbose);

	// check for scsi errors
	stlink_confirm_inq(sl, ptvp);
	// TODO recycle: clear_scsi_pt_obj(struct sg_pt_base * objp);
	destruct_scsi_pt_obj(ptvp);
}

static void stlink_print_data(struct stlink *sl)
{
	if (sl->q_len <= 0 || sl->verbose < 2)
		return;
	if (sl->verbose > 2)
		fprintf(stdout, "data_len = %d 0x%x\n", sl->q_len, sl->q_len);

	for (uint32_t i = 0; i < sl->q_len; i++) {
		if (i % 16 == 0) {
			if (sl->q_data_dir == Q_DATA_OUT)
				fprintf(stdout, "\n<- 0x%08x ", sl->q_addr + i);
			else
				fprintf(stdout, "\n-> 0x%08x ", sl->q_addr + i);
		}
		fprintf(stdout, " %02x", (unsigned int) sl->q_buf[i]);
	}
	fprintf(stdout, "\n\n");
}

// TODO thinking, cleanup
static void stlink_parse_version(struct stlink *sl)
{
	sl->st_vid = 0;
	sl->stlink_pid = 0;
	if (sl->q_len <= 0) {
		fprintf(stderr, "Error: could not parse the stlink version");
		return;
	}
	stlink_print_data(sl);
	uint32_t b0 = sl->q_buf[0]; //lsb
	uint32_t b1 = sl->q_buf[1];
	uint32_t b2 = sl->q_buf[2];
	uint32_t b3 = sl->q_buf[3];
	uint32_t b4 = sl->q_buf[4];
	uint32_t b5 = sl->q_buf[5]; //msb

	// b0 b1                       || b2 b3  | b4 b5
	// 4b        | 6b     | 6b     || 2B     | 2B
	// stlink_v  | jtag_v | swim_v || st_vid | stlink_pid

	sl->stlink_v = (b0 & 0xf0) >> 4;
	sl->jtag_v = ((b0 & 0x0f) << 2) | ((b1 & 0xc0) >> 6);
	sl->swim_v = b1 & 0x3f;
	sl->st_vid = (b3 << 8) | b2;
	sl->stlink_pid = (b5 << 8) | b4;

	if (sl->verbose < 2)
		return;

	fprintf(stderr, "st vid         = 0x%04x (expect 0x%04x)\n",
		sl->st_vid, USB_ST_VID);
	fprintf(stderr, "stlink pid     = 0x%04x (expect 0x%04x)\n",
		sl->stlink_pid, USB_STLINK_PID);
	fprintf(stderr, "stlink version = 0x%x\n", sl->stlink_v);
	fprintf(stderr, "jtag version   = 0x%x\n", sl->jtag_v);
	fprintf(stderr, "swim version   = 0x%x\n", sl->swim_v);
	if (sl->jtag_v == 0)
		fprintf(stderr,
			"    The firmware does not support a JTAG/SWD interface.\n");
	if (sl->swim_v == 0)
		fprintf(stderr,
			"    The firmware does not support a SWIM interface.\n");

}

static int stlink_mode(struct stlink *sl)
{
	int mode;
	char *mode_name;

	if (sl->q_len <= 0)
		return STLINK_DEV_UNKNOWN_MODE;

	stlink_print_data(sl);

	mode = sl->q_buf[0];
	switch (mode) {
	case STLINK_DEV_DFU_MODE:
		mode_name = "DFU (direct firmware update)"; break;
	case STLINK_DEV_DEBUG_MODE:
		mode_name = "Debug (JTAG/SWD)"; break;
	case STLINK_DEV_MASS_MODE:
		mode_name = "Mass storage"; break;
	default:
		mode = STLINK_DEV_UNKNOWN_MODE;
		mode_name = "Unknown"; break;
	}

	if (sl->verbose)
		fprintf(stderr, "stlink mode: %s\n", mode_name);
	return mode;
}

static void stlink_stat(struct stlink *sl, char *txt)
{
	if (sl->q_len <= 0 || sl->verbose == 0)
		return;

	stlink_print_data(sl);

	switch (sl->q_buf[0]) {
	case STLINK_OK:
		fprintf(stderr, "  %s: ok\n", txt);
		return;
	case STLINK_FALSE:
		fprintf(stderr, "  %s: false\n", txt);
		return;
	default:
		fprintf(stderr, "  %s: unknown\n", txt);
	}
}

static void stlink_core_stat(struct stlink *sl)
{
	char *status_name = "unknown";
	if (sl->q_len <= 0)
		return;

	stlink_print_data(sl);

	sl->core_stat = sl->q_buf[0];
	switch (sl->core_stat) {
	case STLINK_CORE_RUNNING: status_name = "running"; break;
	case STLINK_CORE_HALTED:  status_name = "halted";  break;
	default:
		sl->core_stat = STLINK_CORE_STAT_UNKNOWN;
		status_name = "unknown";  break;
	}
	if (sl->verbose)
		fprintf(stderr, "  core status: %s\n", status_name);
	return;
}

void stlink_version(struct stlink *sl)
{
	D(sl, "\n*** stlink_version ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_GET_VERSION;
	sl->q_len = 6;
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_parse_version(sl);
	return;
}

// Get stlink mode:
// STLINK_DEV_DFU_MODE || STLINK_DEV_MASS_MODE || STLINK_DEV_DEBUG_MODE
// usb dfu             || usb mass             || jtag or swd
int stlink_current_mode(struct stlink *sl)
{
	D(sl, "\n*** stlink_current_mode ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[0] = STLINK_GET_CURRENT_MODE;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	return stlink_mode(sl);
}

/* Change from mass storage mode to SWD debug mode. */
void stlink_enter_swd_mode(struct stlink *sl)
{
	D(sl, "\n*** stlink_enter_swd_mode ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_ENTER;
	sl->cdb_cmd_blk[2] = STLINK_DEBUG_ENTER_SWD;
	sl->q_len = 0; // >0 -> aboard
	stlink_q(sl);
}

// Exit the mass mode and enter the jtag debug mode.
// (jtag is disabled in the discovery's stlink firmware)
void stlink_enter_jtag_mode(struct stlink *sl)
{
	D(sl, "\n*** stlink_enter_jtag_mode ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_ENTER;
	sl->cdb_cmd_blk[2] = STLINK_DEBUG_ENTER_JTAG;
	sl->q_len = 0;
	stlink_q(sl);
}

// Exit the jtag or swd mode and enter the mass mode.
void stlink_exit_debug_mode(struct stlink *sl)
{
	D(sl, "\n*** stlink_exit_debug_mode ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_EXIT;
	sl->q_len = 0; // >0 -> aboard
	stlink_q(sl);
}

// XXX kernel driver performs reset, the device temporally disappears
static void stlink_exit_dfu_mode(struct stlink *sl)
{
	D(sl, "\n*** stlink_exit_dfu_mode ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[0] = STLINK_DFU_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DFU_EXIT;
	sl->q_len = 0; // ??
	stlink_q(sl);
	/*
	 [135121.844564] sd 19:0:0:0: [sdb] Unhandled error code
	 [135121.844569] sd 19:0:0:0: [sdb] Result: hostbyte=DID_ERROR driverbyte=DRIVER_OK
	 [135121.844574] sd 19:0:0:0: [sdb] CDB: Read(10): 28 00 00 00 10 00 00 00 08 00
	 [135121.844584] end_request: I/O error, dev sdb, sector 4096
	 [135121.844590] Buffer I/O error on device sdb, logical block 512
	 [135130.122567] usb 6-1: reset full speed USB device using uhci_hcd and address 7
	 [135130.274551] usb 6-1: device firmware changed
	 [135130.274618] usb 6-1: USB disconnect, address 7
	 [135130.275186] VFS: busy inodes on changed media or resized disk sdb
	 [135130.275424] VFS: busy inodes on changed media or resized disk sdb
	 [135130.286758] VFS: busy inodes on changed media or resized disk sdb
	 [135130.292796] VFS: busy inodes on changed media or resized disk sdb
	 [135130.301481] VFS: busy inodes on changed media or resized disk sdb
	 [135130.304316] VFS: busy inodes on changed media or resized disk sdb
	 [135130.431113] usb 6-1: new full speed USB device using uhci_hcd and address 8
	 [135130.629444] usb-storage 6-1:1.0: Quirks match for vid 0483 pid 3744: 102a1
	 [135130.629492] scsi20 : usb-storage 6-1:1.0
	 [135131.625600] scsi 20:0:0:0: Direct-Access     STM32                          PQ: 0 ANSI: 0
	 [135131.627010] sd 20:0:0:0: Attached scsi generic sg2 type 0
	 [135131.633603] sd 20:0:0:0: [sdb] 64000 512-byte logical blocks: (32.7 MB/31.2 MiB)
	 [135131.633613] sd 20:0:0:0: [sdb] Assuming Write Enabled
	 [135131.633620] sd 20:0:0:0: [sdb] Assuming drive cache: write through
	 [135131.640584] sd 20:0:0:0: [sdb] Assuming Write Enabled
	 [135131.640592] sd 20:0:0:0: [sdb] Assuming drive cache: write through
	 [135131.640609]  sdb:
	 [135131.652634] sd 20:0:0:0: [sdb] Assuming Write Enabled
	 [135131.652639] sd 20:0:0:0: [sdb] Assuming drive cache: write through
	 [135131.652645] sd 20:0:0:0: [sdb] Attached SCSI removable disk
	 [135131.671536] sd 20:0:0:0: [sdb] Result: hostbyte=DID_OK driverbyte=DRIVER_SENSE
	 [135131.671548] sd 20:0:0:0: [sdb] Sense Key : Illegal Request [current]
	 [135131.671553] sd 20:0:0:0: [sdb] Add. Sense: Logical block address out of range
	 [135131.671560] sd 20:0:0:0: [sdb] CDB: Read(10): 28 00 00 00 f9 80 00 00 08 00
	 [135131.671570] end_request: I/O error, dev sdb, sector 63872
	 [135131.671575] Buffer I/O error on device sdb, logical block 7984
	 [135131.678527] sd 20:0:0:0: [sdb] Result: hostbyte=DID_OK driverbyte=DRIVER_SENSE
	 [135131.678532] sd 20:0:0:0: [sdb] Sense Key : Illegal Request [current]
	 [135131.678537] sd 20:0:0:0: [sdb] Add. Sense: Logical block address out of range
	 [135131.678542] sd 20:0:0:0: [sdb] CDB: Read(10): 28 00 00 00 f9 80 00 00 08 00
	 [135131.678551] end_request: I/O error, dev sdb, sector 63872
	 ...
	 [135131.853565] end_request: I/O error, dev sdb, sector 4096
	 */
}

/*
 * The first transaction in SW-DP mode must be to read the internal ID code.
 * This ID code is the default ARM one, 0x1BA01477, which identifies the
 * Cortex-M3 r1p1.
 */
void stlink_core_id(struct stlink *sl)
{
	D(sl, "\n*** stlink_core_id ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_READCOREID;
	sl->q_len = 4;
	sl->q_addr = 0;
	stlink_q(sl);
	sl->core_id = read_uint32(sl->q_buf, 0);
	if (sl->verbose > 2) {
		stlink_print_data(sl);
		fprintf(stderr, "core_id = 0x%08x\n", sl->core_id);
	}
	return;
}

/* Reset the ARM core, leaving it in the halted state. */
void stlink_reset(struct stlink *sl)
{
	D(sl, "\n*** stlink_reset ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_RESETSYS;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	if (sl->verbose)
		stlink_stat(sl, "core reset");
}

/* Get the ARM core status: halted or running (or fault/unknown). */
void stlink_status(struct stlink *sl)
{
	D(sl, "\n*** stlink_status ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_GETSTATUS;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_core_stat(sl);
}

/* Force the ARM core into the debug mode -> halted state. */
void stlink_force_debug(struct stlink *sl)
{
	D(sl, "\n*** stlink_force_debug ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_FORCEDEBUG;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_stat(sl, "force debug");
}

void stlink_print_arm_regs(struct ARMcoreRegs *regs)
{
	int i;
	for (i = 0; i < 16; i++)
		fprintf(stderr, "r%02d=0x%08x%c", i, regs->r[i], i%4 == 3 ? '\n' : ' ');
	fprintf(stderr,
			"xpsr       = 0x%08x\n"
			"main_sp    = 0x%08x  process_sp = 0x%08x\n"
			"rw         = 0x%08x  rw2        = 0x%08x\n",
			regs->xpsr, regs->main_sp, regs->process_sp, regs->rw, regs->rw2);
	return;
}

/* Read the ARM core registers, one or all. */
void stlink_read_all_regs(struct stlink *sl)
{
	D(sl, "\n*** stlink_read_all_regs ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_READALLREGS;
	sl->q_len = 84;				/* sizeof(struct ARMcoreRegs) */
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_print_data(sl);

	/* See struct ARMcoreRegs for the ordering. */
	for (int i = 0; i < 16; i++) {
		sl->reg.r[i] = read_uint32(sl->q_buf, 4 * i);
	}
	sl->reg.xpsr = read_uint32(sl->q_buf, 64);
	sl->reg.main_sp = read_uint32(sl->q_buf, 68);
	sl->reg.process_sp = read_uint32(sl->q_buf, 72);
	sl->reg.rw = read_uint32(sl->q_buf, 76);
	sl->reg.rw2 = read_uint32(sl->q_buf, 80);
	if (sl->verbose)
		stlink_print_arm_regs(&sl->reg);
	return;
}

/* Read a single ARM register.  See 'struct ARMcoreRegs' for the index. */
uint32_t stlink_read_1reg(struct stlink *sl, uint8_t reg_idx)
{
	if (reg_idx > 20)
		return 0xFFFFFFFF;
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_READREG;
	sl->cdb_cmd_blk[2] = reg_idx;
	sl->q_len = 4;
	sl->q_addr = 0;
	stlink_q(sl);
	return read_uint32(sl->q_buf, 0);
}

uint32_t stlink_read_reg(struct stlink *sl, int r_idx)
{
	D(sl, "\n*** stlink_read_reg");

	if (r_idx > 20 || r_idx < 0) {
		fprintf(stderr, "Error: register index must be in [0..20]\n");
		return 0xFFFFFFFF;
	}
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_READREG;
	sl->cdb_cmd_blk[2] = r_idx;
	sl->q_len = 4;
	sl->q_addr = 0;
	stlink_q(sl);
	//  0  |  1  | ... |  15   |  16   |   17    |   18       |  19   |  20
	// 0-3 | 4-7 | ... | 60-63 | 64-67 | 68-71   | 72-75      | 76-79 | 80-83
	// r0  | r1  | ... | r15   | xpsr  | main_sp | process_sp | rw    | rw2
	stlink_print_data(sl);

	uint32_t r = read_uint32(sl->q_buf, 0);
	fprintf(stderr, "r_idx (%2d) = 0x%08x\n", r_idx, r);

	switch (r_idx) {
	case 16:
		sl->reg.xpsr = r;
		break;
	case 17:
		sl->reg.main_sp = r;
		break;
	case 18:
		sl->reg.process_sp = r;
		break;
	case 19:
		sl->reg.rw = r; //XXX ?(primask, basemask etc.)
		break;
	case 20:
		sl->reg.rw2 = r; //XXX ?(primask, basemask etc.)
		break;
	default:
		sl->reg.r[r_idx] = r;
	}
	return r;
}

/*
 * Write a single ARM core register.
 */
// Write an arm-core register. Index:
//  0  |  1  | ... |  15   |  16   |   17    |   18       |  19   |  20
// r0  | r1  | ... | r15   | xpsr  | main_sp | process_sp | rw    | rw2
void stlink_write_reg(struct stlink *sl, uint32_t reg, int idx)
{
	D(sl, "\n*** stlink_write_reg ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_WRITEREG;
	//   2: reg index
	// 3-6: reg content
	sl->cdb_cmd_blk[2] = idx;
	write_uint32(sl->cdb_cmd_blk + 3, reg);
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_stat(sl, "write reg");
}

/*
 * Write an ARM core debug module register.
 *  Cmd 2-5: address of reg of the debug module
 *  Cmd 6-9: reg content
 */
// Write a register of the debug module of the core.
// XXX ?(atomic writes)
// TODO test
void stlink_write_dreg(struct stlink *sl, uint32_t reg, uint32_t addr)
{
	D(sl, "\n*** stlink_write_dreg ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_WRITEDEBUGREG;
	// 2-5: address of reg of the debug module
	// 6-9: reg content
	write_uint32(sl->cdb_cmd_blk + 2, addr);
	write_uint32(sl->cdb_cmd_blk + 6, reg);
	sl->q_len = 2;
	sl->q_addr = addr;
	stlink_q(sl);
	stlink_stat(sl, "write debug reg");
}

/*
 * Command the ARM core to exit debug mode and run.
 */
void stlink_run(struct stlink *sl)
{
	D(sl, "\n*** stlink_run ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_RUNCORE;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	if (sl->verbose)
		stlink_stat(sl, "run core");
}

/*
 * Start running at a specific location: set PC and run / jump to a location.
 */
static unsigned int is_core_halted(struct stlink*);
void stlink_run_at(struct stlink *sl, stm32_addr_t addr)
{
	stlink_write_reg(sl, addr, 15); /* pc register */
	stlink_run(sl);
	/* Hmmm, is this really useful? */
	while (is_core_halted(sl) == 0)
		usleep(3000000);
}

// Step the arm-core.
void stlink_step(struct stlink *sl)
{
	D(sl, "\n*** stlink_step ***\n");
	clear_cdb(sl);
	sl->q_data_dir = Q_DATA_IN;
	sl->cdb_cmd_blk[0] = STLINK_DEBUG_COMMAND;
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_STEPCORE;
	sl->q_len = 2;
	sl->q_addr = 0;
	stlink_q(sl);
	stlink_stat(sl, "step core");
}

/*
 * Set and clear hardware breakpoints.
 */
// TODO test
// see Cortex-M3 Technical Reference Manual
void stlink_set_hw_bp(struct stlink *sl, int fp_nr, uint32_t addr, int fp)
{
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_SETFP;
	// 2:The number of the flash patch used to set the breakpoint
	// 3-6: Address of the breakpoint (LSB)
	// 7: FP_ALL (0x02) / FP_UPPER (0x01) / FP_LOWER (0x00)
	sl->q_buf[2] = fp_nr;
	write_uint32(sl->q_buf, addr);
	sl->q_buf[7] = fp;

	sl->q_len = 2;
	stlink_q(sl);
	stlink_stat(sl, "set flash breakpoint");
}

// TODO test
void stlink_clr_hw_bp(struct stlink *sl, int fp_nr)
{
	D(sl, "\n*** stlink_clr_hw_bp ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_CLEARFP;
	sl->cdb_cmd_blk[2] = fp_nr;

	sl->q_len = 2;
	stlink_q(sl);
	stlink_stat(sl, "clear flash breakpoint");
}

/* Read from the ARM memory starting at ADDR for LEN bytes.
 * The transfer size must be a multiple of 4 bytes up to 6144 bytes.
 */
void stlink_read_mem32(struct stlink *sl, uint32_t addr, uint16_t len)
{
	if (sl->verbose > 1)
		fprintf(stderr, "\n*** stlink_read_mem32(0x%8.8x, %d) ***\n",
				addr, len);
	D(sl, "\n*** stlink_read_mem32 ***\n");
	if (len % 4 != 0) { // !!! never ever: fw gives just wrong values
		fprintf(stderr, "Error: stlink_read_mem32() does not "
				"have a 32 bit data alignment: +%d byte.\n",
				len % 4);
		return;
	}
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_READMEM_32BIT;
	// 2-5: addr
	// 6-7: len
	write_uint32(sl->cdb_cmd_blk + 2, addr);
	write_uint16(sl->cdb_cmd_blk + 6, (len+3) & ~3);

	// data_in 0-0x40-len
	// !!! len _and_ q_len must be max 6k,
	//     i.e. >1024 * 6 = 6144 -> aboard)
	// !!! if len < q_len: 64*k, 1024*n, n=1..5  -> aboard
	//     (broken residue issue)
	sl->q_len = len;
	sl->q_addr = addr;
	stlink_q(sl);
	stlink_print_data(sl);
}

/* Write to the ARM memory starting at ADDR for LEN bytes.
 * The *_mem8 variant has a maximum LEN of 64 bytes.
 * The *_mem32 variant must have LEN be a multiple of 4.
 */
void stlink_write_mem8(struct stlink *sl, uint32_t addr, uint16_t len)
{
	D(sl, "\n*** stlink_write_mem8 ***\n");
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_WRITEMEM_8BIT;
	// 2-5: addr
	// 6-7: len (>0x40 (64) -> abort)
	write_uint32(sl->cdb_cmd_blk + 2, addr);
	write_uint16(sl->cdb_cmd_blk + 6, len);
	sl->q_len = len;
	sl->q_addr = addr;
	sl->q_data_dir = Q_DATA_OUT;
	stlink_q(sl);
	stlink_print_data(sl);
}

void stlink_write_mem32(struct stlink *sl, uint32_t addr, uint16_t len);
void stlink_write_mem16(struct stlink *sl, uint32_t addr, uint16_t len)
{
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_WRITEMEM_8BIT;
	write_uint32(sl->cdb_cmd_blk + 2, addr);
	write_uint16(sl->cdb_cmd_blk + 6, len);
	sl->q_len = len;
	sl->q_data_dir = Q_DATA_OUT;
	stlink_q(sl);
}

// Write a "len" bytes from the sl->q_buf to the memory, max Q_BUF_LEN bytes.
void stlink_write_mem32(struct stlink *sl, uint32_t addr, uint16_t len)
{
	D(sl, "\n*** stlink_write_mem32 ***\n");
	if (len % 4 != 0) {
		fprintf(stderr, "Error: Data length does not have a 32 bit "
				"alignment: +%d byte.\n",
				len % 4);
		return;
	}
	clear_cdb(sl);
	sl->cdb_cmd_blk[1] = STLINK_DEBUG_WRITEMEM_32BIT;
	// 2-5: addr
	// 6-7: len "unlimited"
	write_uint32(sl->cdb_cmd_blk + 2, addr);
	write_uint16(sl->cdb_cmd_blk + 6, len);

	// data_out 0-0x40-...-len
	sl->q_len = len;
	sl->q_addr = addr;
	sl->q_data_dir = Q_DATA_OUT;
	stlink_q(sl);
	stlink_print_data(sl);
}

/* FPEC flash controller interface, pm0063 manual */
#define FLASH_REGS_ADDR 0x40022000
#define FLASH_REGS_SIZE 0x28

#define FLASH_ACR (FLASH_REGS_ADDR + 0x00)
#define FLASH_KEYR (FLASH_REGS_ADDR + 0x04)
#define FLASH_SR (FLASH_REGS_ADDR + 0x0c)
#define FLASH_CR (FLASH_REGS_ADDR + 0x10)
#define FLASH_AR (FLASH_REGS_ADDR + 0x14)
#define FLASH_OBR (FLASH_REGS_ADDR + 0x1c)
#define FLASH_WRPR (FLASH_REGS_ADDR + 0x20)

/* Flash unlock key values from PM0075 2.3.1 */
#define FLASH_RDPTR_KEY 0x00a5
#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xcdef89ab

#define FLASH_SR_BSY 0
#define FLASH_SR_EOP 5

#define FLASH_CR_PG 0
#define FLASH_CR_PER 1
#define FLASH_CR_MER 2
#define FLASH_CR_STRT 6
#define FLASH_CR_LOCK 7

static uint32_t __attribute__((unused)) read_flash_rdp(struct stlink* sl)
{
  stlink_read_mem32(sl, FLASH_WRPR, sizeof(uint32_t));
  return (*(uint32_t*)sl->q_buf) & 0xff;
}

static inline uint32_t read_flash_wrpr(struct stlink* sl)
{
  stlink_read_mem32(sl, FLASH_WRPR, sizeof(uint32_t));
  return *(uint32_t*)sl->q_buf;
}

static inline uint32_t read_flash_obr(struct stlink* sl)
{
  stlink_read_mem32(sl, FLASH_OBR, sizeof(uint32_t));
  return *(uint32_t*)sl->q_buf;
}

static inline uint32_t read_flash_cr(struct stlink* sl)
{
  stlink_read_mem32(sl, FLASH_CR, sizeof(uint32_t));
  return *(uint32_t*)sl->q_buf;
}

static inline unsigned int is_flash_locked(struct stlink* sl)
{
	/* return non zero for true */
	return read_flash_cr(sl) & (1 << FLASH_CR_LOCK);
}

/* Unlock the flash.  This takes two write cycles with two key values.
 * The two key values are sequentially written to the FLASH_KEYR register.
 */
static void unlock_flash(struct stlink* sl)
{
  write_uint32(sl->q_buf, FLASH_KEY1);
  stlink_write_mem32(sl, FLASH_KEYR, sizeof(uint32_t));

  write_uint32(sl->q_buf, FLASH_KEY2);
  stlink_write_mem32(sl, FLASH_KEYR, sizeof(uint32_t));
}

/* Unlock flash if already locked. */
static int unlock_flash_if(struct stlink* sl)
{
	if (is_flash_locked(sl)) {
		unlock_flash(sl);
		if (is_flash_locked(sl))
			return -1;
	}
	return 0;
}

/* Lock flash by setting a bit in the control register.
 * The unlock sequence must be used to reverse this.
 */
static void lock_flash(struct stlink* sl)
{
	const uint32_t n = read_flash_cr(sl) | (1 << FLASH_CR_LOCK);

	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static inline void sl_wr32(struct stlink* sl, uint32_t addr, uint32_t val)
{
	write_uint32(sl->q_buf, val);
	stlink_write_mem32(sl, addr, sizeof(uint32_t));
}
static inline uint32_t sl_rd32(struct stlink *sl, uint32_t addr)
{
	stlink_read_mem32(sl, addr, sizeof(uint32_t));
	return *(uint32_t*)sl->q_buf;
}
#define read_flash_ar(sl) sl_rd32(sl, FLASH_AR)
#define read_flash_obr(sl) sl_rd32(sl, FLASH_OBR)

static void set_flash_cr_pg(struct stlink* sl)
{
	const uint32_t n = 1 << FLASH_CR_PG;
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void __attribute__((unused)) clear_flash_cr_pg(struct stlink* sl)
{
	const uint32_t n = read_flash_cr(sl) & ~(1 << FLASH_CR_PG);
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void set_flash_cr_per(struct stlink* sl)
{
	const uint32_t n = 1 << FLASH_CR_PER;
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void __attribute__((unused)) clear_flash_cr_per(struct stlink* sl)
{
	const uint32_t n = read_flash_cr(sl) & ~(1 << FLASH_CR_PER);
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void set_flash_cr_mer(struct stlink* sl)
{
	const uint32_t n = 1 << FLASH_CR_MER;
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void __attribute__((unused)) clear_flash_cr_mer(struct stlink* sl)
{
	const uint32_t n = read_flash_cr(sl) & ~(1 << FLASH_CR_MER);
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static void set_flash_cr_strt(struct stlink* sl)
{
	/* assume come on the flash_cr_per path */
	const uint32_t n = (1 << FLASH_CR_PER) | (1 << FLASH_CR_STRT);
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_CR, sizeof(uint32_t));
}

static inline uint32_t read_flash_acr(struct stlink* sl)
{
	stlink_read_mem32(sl, FLASH_ACR, sizeof(uint32_t));
	return *(uint32_t*)sl->q_buf;
}

static inline uint32_t read_flash_sr(struct stlink* sl)
{
	stlink_read_mem32(sl, FLASH_SR, sizeof(uint32_t));
	return *(uint32_t*)sl->q_buf;
}

static inline unsigned int is_flash_busy(struct stlink* sl)
{
	return read_flash_sr(sl) & (1 << FLASH_SR_BSY);
}

static void wait_flash_busy(struct stlink* sl)
{
	/* todo: add some delays here */
	while (is_flash_busy(sl))
		;
}

static inline unsigned int is_flash_eop(struct stlink* sl)
{
	return read_flash_sr(sl) & (1 << FLASH_SR_EOP);  
}

static void __attribute__((unused)) clear_flash_sr_eop(struct stlink* sl)
{
	const uint32_t n = read_flash_sr(sl) & ~(1 << FLASH_SR_EOP);
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_SR, sizeof(uint32_t));
}

static void __attribute__((unused)) wait_flash_eop(struct stlink* sl)
{
	/* todo: add some delays here */
	while (is_flash_eop(sl) == 0)
		;
}

static inline void write_flash_ar(struct stlink* sl, uint32_t n)
{
	write_uint32(sl->q_buf, n);
	stlink_write_mem32(sl, FLASH_AR, sizeof(uint32_t));
}

#if 0 /* todo */
static void disable_flash_read_protection(struct stlink* sl)
{
  /* erase the option byte area */
  /* rdp = 0x00a5; */
  /* reset */
}
#endif /* todo */

static int write_flash_mem16(struct stlink* sl, uint32_t addr, uint16_t val)
{
	/* Only allow half word writes */
	if (addr % 2)
		return -1;

	printf("Flash write %8.8x %4.4x -> %4.4x.\n", addr, sl_rd32(sl, addr), val);
	printf("Flash status %2.2x, control %4.4x.\n", read_flash_sr(sl),
		   read_flash_cr(sl));
	/* unlock if locked */
	unlock_flash_if(sl);
	printf("Flash status %2.2x, control %4.4x OBR %8.8x.\n",
		   read_flash_sr(sl), read_flash_cr(sl), sl_rd32(sl, FLASH_OBR));

	/* set flash programming chosen bit */
	set_flash_cr_pg(sl);
	printf("Flash status %2.2x, control %4.4x %8.8x.\n",
		   read_flash_sr(sl), read_flash_cr(sl), read_flash_ar(sl));

	write_uint16(sl->q_buf, val);
	stlink_write_mem16(sl, addr, 2);

	printf("Flash write %8.8x %4.4x -> %4.4x.\n", addr, sl_rd32(sl, addr), val);
	printf("Flash status %2.2x, control %4.4x %8.8x.\n",
		   sl_rd32(sl, FLASH_CR), read_flash_cr(sl), read_flash_ar(sl));

	/* The flash should be only briefly busy. */
	wait_flash_busy(sl);

	printf("Flash status %2.2x, control %4.4x.\n", read_flash_sr(sl),
		   read_flash_cr(sl));
	sl_wr32(sl, FLASH_CR, 0x81);
		/* lock_flash(sl);*/
	printf("Flash status after lock %2.2x, control %4.4x.\n",
		   read_flash_sr(sl), read_flash_cr(sl));

	/* Check by reading the programmed value back. */
	stlink_read_mem32(sl, addr & ~3, 4);
	if (*(const uint16_t*)sl->q_buf != val) {
		/* values differ at i * sizeof(uint16_t) */
		fprintf(stderr, "Flash write failed.\n");
		return -1;
	}

	/* success */
	return 0;
}

static int erase_flash_page(struct stlink* sl, stm32_addr_t page)
{
  /* page an addr in the page to erase */

  /* wait for ongoing op to finish */
  wait_flash_busy(sl);

  /* unlock if locked */
  unlock_flash_if(sl);

  /* set the page erase bit */
  set_flash_cr_per(sl);

  /* select the page to erase */
  write_flash_ar(sl, page);

  /* start erase operation, reset by hw with bsy bit */
  set_flash_cr_strt(sl);

  /* wait for completion */
  wait_flash_busy(sl);

  /* relock the flash */
  lock_flash(sl);

  /* todo: verify the erased page */

  return 0;
}

static int __attribute__((unused)) erase_flash_mass(struct stlink* sl)
{
  /* wait for ongoing op to finish */
  wait_flash_busy(sl);

  /* unlock if locked */
  unlock_flash_if(sl);

  /* set the mass erase bit */
  set_flash_cr_mer(sl);

  /* start erase operation, reset by hw with bsy bit */
  set_flash_cr_strt(sl);

  /* wait for completion */
  wait_flash_busy(sl);

  /* relock the flash */
  lock_flash(sl);

  /* todo: verify the erased memory */

  return 0;
}

static unsigned int is_core_halted(struct stlink* sl)
{
	/* return non zero if core is halted */
	stlink_status(sl);
  return sl->q_buf[0] == STLINK_CORE_HALTED;
}

static int write_loader_to_sram(struct stlink* sl, stm32_addr_t* addr,
								size_t* size)
{
	/* From openocd, contrib/loaders/flash/stm32.s
	 * i = STM32_FLASH_BASE;      0x08+r3  0x0800,0000
	 * 
	 */
	static const uint8_t loader_code[] = {
		0x08, 0x4c,			/* ldr	r4, STM32_FLASH_BASE */
		0x1c, 0x44,			/* add	r4, r3 */
		/* write_half_word: */
		0x01, 0x23,			/* movs	r3, #0x01 */
		0x23, 0x61,			/* str	r3, [r4, #STM32_FLASH_CR_OFFSET] */
		0x30, 0xf8, 0x02, 0x3b,	/* ldrh	r3, [r0], #0x02 */
		0x21, 0xf8, 0x02, 0x3b,	/* strh	r3, [r1], #0x02 */
		/* busy: */
		0xe3, 0x68,			/* ldr	r3, [r4, #STM32_FLASH_SR_OFFSET] */
		0x13, 0xf0, 0x01, 0x0f,	/* tst	r3, #0x01 */
		0xfb, 0xd0,			/* beq	busy */
		0x13, 0xf0, 0x14, 0x0f,	/* tst	r3, #0x14 */
		0x01, 0xd1,			/* bne	exit */
		0x01, 0x3a,			/* subs	r2, r2, #0x01 */
		0xf0, 0xd1,			/* bne	write_half_word */
		/* exit: */
		0x00, 0xbe,			/* bkpt	#0x00 */
		0x00, 0x20, 0x02, 0x40,	/* STM32_FLASH_BASE: .word 0x40022000 */
	};

  memcpy(sl->q_buf, loader_code, sizeof(loader_code));
  stlink_write_mem32(sl, sl->sram_base, sizeof(loader_code));

  *addr = sl->sram_base;
  *size = sizeof(loader_code);

  /* success */
  return 0;
}

typedef struct flash_loader
{
  stm32_addr_t loader_addr; /* loader sram adddr */
  stm32_addr_t buf_addr; /* buffer sram address */
} flash_loader_t;

static int write_buffer_to_sram(struct stlink* sl, flash_loader_t* fl,
								const uint8_t* buf, size_t size)
{
	/* Write the buffer right after the loader */
	memcpy(sl->q_buf, buf, size);
	stlink_write_mem8(sl, fl->buf_addr, size);
	return 0;
}

static int init_flash_loader(struct stlink* sl, flash_loader_t* fl)
{
	size_t size;

	/* allocate the loader in sram */
	if (write_loader_to_sram(sl, &fl->loader_addr, &size) == -1) {
		fprintf(stderr, "write_loader_to_sram() == -1\n");
		return -1;
	}

	/* allocate a one page buffer in sram right after loader */
	fl->buf_addr = fl->loader_addr + size;

	return 0;
}

static int
run_flash_loader(struct stlink* sl, flash_loader_t* fl, stm32_addr_t target,
				 const uint8_t* buf, size_t size)
{
	const size_t count = size / sizeof(uint16_t);

	if (write_buffer_to_sram(sl, fl, buf, size) == -1) {
		fprintf(stderr, "write_buffer_to_sram() == -1\n");
		return -1;
	}

	/* setup core */
	stlink_write_reg(sl, fl->buf_addr, 0); /* source */
	stlink_write_reg(sl, target, 1); /* target */
	stlink_write_reg(sl, count, 2); /* count (16 bits half words) */
	stlink_write_reg(sl, 0, 3); /* flash bank 0 (input) */
	stlink_write_reg(sl, fl->loader_addr, 15); /* pc register */

	/* unlock and set programming mode */
	unlock_flash_if(sl);
	set_flash_cr_pg(sl);

	/* run loader */
	stlink_run(sl);
  
	while (is_core_halted(sl) == 0)
		;

	lock_flash(sl);

	/* not all bytes have been written */
	stlink_read_reg(sl, 2);
	if (sl->reg.r[2] != 0) {
		fprintf(stderr, "write error, count == %u\n", sl->reg.r[2]);
		return -1;
	}

	return 0;
}

/* Map a file using mmap(). */
typedef struct mapped_file
{
  uint8_t* base;
  size_t len;
} mapped_file_t;

#define MAPPED_FILE_INITIALIZER { NULL, 0 }

static int map_file(mapped_file_t* mf, const char* path)
{
	int error = -1;
	struct stat st;

	const int fd = open(path, O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
		return -1;
	}

	if (fstat(fd, &st) == -1) {
		fprintf(stderr, "fstat(%s): %s\n", path, strerror(errno));
		goto on_error;
	}

	mf->base = (uint8_t*)mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (mf->base == MAP_FAILED) {
		fprintf(stderr, "mmap() == MAP_FAILED\n");
		goto on_error;
	}

	mf->len = st.st_size;
	/* success */
	error = 0;

 on_error:
	close(fd);

	return error;
}

static void unmap_file(mapped_file_t* mf)
{
	munmap((void*)mf->base, mf->len);
	mf->base = (unsigned char*)MAP_FAILED;
	mf->len = 0;
	return;
}

static int check_file(struct stlink* sl, mapped_file_t* mf, stm32_addr_t addr)
{
	size_t off;

	for (off = 0; off < mf->len; off += sl->flash_pgsz) {
		size_t aligned_size;

		/* adjust last page size */
		size_t cmp_size = sl->flash_pgsz;
		if ((off + sl->flash_pgsz) > mf->len)
			cmp_size = mf->len - off;

		aligned_size = cmp_size;
		if (aligned_size & (4 - 1))
			aligned_size = (cmp_size + 4) & ~(4 - 1);

		stlink_read_mem32(sl, addr + off, aligned_size);

		if (memcmp(sl->q_buf, mf->base + off, cmp_size))
			return -1;
	}
	return 0;
}

/* Verify that the contents of file PATH match the ARM memory
 * starting at ADDR. */
int stlink_fcheck_flash(struct stlink* sl, const char* path,
							   stm32_addr_t addr)
{
	int res;
	mapped_file_t mf = MAPPED_FILE_INITIALIZER;

	if (map_file(&mf, path) == -1)
		return -1;
	res = check_file(sl, &mf, addr);
	unmap_file(&mf);
	return res;
}

/* Write the contents of file PATH into flash starting at ADDR. */
int stlink_fwrite_flash(struct stlink* sl, const char* path,
							   stm32_addr_t addr)
{
	int error = -1;
	size_t off;
	mapped_file_t mf = MAPPED_FILE_INITIALIZER;
	flash_loader_t fl;

	if (map_file(&mf, path) == -1) {
		fprintf(stderr, "map_file(%s) failed\n", path);
		return -1;
	}

	/* Check that address and extent would write only into flash. */
	if (addr < sl->flash_base) {
		fprintf(stderr, "Base address %8.8x is below the flash base of %8.8x\n",
				addr, sl->flash_base);
		goto on_error;
	} else if ((addr + mf.len) < addr) {
		/* Cannot happen. */
		fprintf(stderr, "addr overruns\n");
		goto on_error;
	} else if ((addr + mf.len) > (sl->flash_base + sl->flash_size)) {
		fprintf(stderr, "Write of %d bytes of 0x%8.8X would extend beyond end "
				"of flash at 0x%8.8X.\n",
				mf.len, addr, sl->flash_base + sl->flash_size);
		goto on_error;
	} else if ((addr & 1) || (mf.len & 1)) {
		/* todo */
		fprintf(stderr, "Cannot write data at an unaligned flash address "
				"0x%8.8x\n", addr);
		goto on_error;
	}

	/* Erase each page individually.  A mass erase would be much faster, but
	 * this way we can leave unwritten pages untouched.
	 */
	for (off = 0; off < mf.len; off += sl->flash_pgsz) {
		/* Addr may be any address inside the page. */
		if (erase_flash_page(sl, addr + off) == -1) {
			fprintf(stderr, "erase_flash_page(0x%x) == -1\n", addr + off);
			goto on_error;
		}
	}

	/* flash loader initialization */
	if (init_flash_loader(sl, &fl) == -1) {
		fprintf(stderr, "init_flash_loader() == -1\n");
		goto on_error;
	}

	/* write each page. above WRITE_BLOCK_SIZE fails? */
#define WRITE_BLOCK_SIZE 0x40
	for (off = 0; off < mf.len; off += WRITE_BLOCK_SIZE) {
		/* adjust last write size */
		size_t size = WRITE_BLOCK_SIZE;
		if ((off + WRITE_BLOCK_SIZE) > mf.len)
			size = mf.len - off;

		if (run_flash_loader(sl, &fl, addr + off, mf.base + off, size) == -1) {
			fprintf(stderr, "run_flash_loader(0x%x) == -1\n", addr + off);
			goto on_error;
		}
	}

	/* Check that the file has been written */
	if (check_file(sl, &mf, addr) == -1) {
		fprintf(stderr, "check_file() == -1\n");
		goto on_error;
	}

	/* success */
	error = 0;

 on_error:
	unmap_file(&mf);
	return error;
}

/* Initialize the contents of RAM from file PATH. */
int stlink_fwrite_sram(struct stlink* sl, const char* path,
							  stm32_addr_t addr)
{
	int error = -1;
	size_t off;
	mapped_file_t mf = MAPPED_FILE_INITIALIZER;

	if (map_file(&mf, path) == -1) {
		fprintf(stderr, "map_file() == -1\n");
		return -1;
	}

	/* check addr range is inside the sram */
	if (addr < sl->sram_base) {
		fprintf(stderr, "addr too low\n");
		goto on_error;
	} else if ((addr + mf.len) < addr) {
		fprintf(stderr, "addr overruns\n");
		goto on_error;
	} else if ((addr + mf.len) > (sl->sram_base + sl->sram_size)) {
		fprintf(stderr, "addr too high\n");
		goto on_error;
	} else if ((addr & 3) || (mf.len & 3)) {
		/* todo */
		fprintf(stderr, "unaligned addr or size\n");
		goto on_error;
	}

	/* do the copy by 1k blocks */
	for (off = 0; off < mf.len; off += 1024) {
		size_t size = 1024;
		if ((off + size) > mf.len)
			size = mf.len - off;

		memcpy(sl->q_buf, mf.base + off, size);

		/* Round size if needed  DJB: this looks broken. */
		if (size & 3)
			size += 2;

		stlink_write_mem32(sl, addr + off, size);
	}

  /* Check that the file has been written */
  if (check_file(sl, &mf, addr) == -1) {
	  fprintf(stderr, "check_file() == -1\n");
	  goto on_error;
  }

  /* success */
  error = 0;

 on_error:
  unmap_file(&mf);
  return error;
}


/* Read from the ARM, SIZE bytes at offet ADDR to file */
static int stlink_fread(struct stlink* sl, const char* path,
						stm32_addr_t addr, size_t size)
{
	size_t off;

	const int fd = open(path, O_RDWR | O_TRUNC | O_CREAT, 00664);
	if (fd == -1) {
		fprintf(stderr, " Failed to open '%s': %s\n", path, strerror(errno));
		return -1;
	}

	/* do the copy by 1k blocks */
	for (off = 0; off < size; off += 1024) {
		size_t read_size = 1024;
		if ((off + read_size) > size)
			read_size = off + read_size;

		/* round size if needed */
		if (read_size & 3)
			read_size = (read_size + 4) & ~(3);

		stlink_read_mem32(sl, addr + off, read_size);

		if (write(fd, sl->q_buf, read_size) != (ssize_t)read_size) {
			fprintf(stderr, "write() != read_size\n");
			close(fd);
			return -1;
		}
	}

	return 0;
}

// 1) open a sg device, switch the stlink from dfu to mass mode
// 2) wait 5s until the kernel driver stops reseting the broken device
// 3) reopen the device
// 4) the device driver is now ready for a switch to jtag/swd mode
// TODO thinking, better error handling, wait until the kernel driver stops reseting the plugged-in device
struct stlink* stlink_force_open(const char *dev_name, const int verbose)
{
	struct stlink *sl = stlink_open(dev_name, verbose);
	if (sl == NULL) {
		fputs("Error: could not open stlink device\n", stderr);
		return NULL;
	}

	stlink_version(sl);

	if (sl->st_vid != USB_ST_VID || sl->stlink_pid != USB_STLINK_PID) {
		fprintf(stderr, "Error: the device %s is not a stlink\n"
				"       VID: got %04x expect %04x \n"
				"       PID: got %04x expect %04x \n",
				dev_name, sl->st_vid, USB_ST_VID,
				sl->stlink_pid, USB_STLINK_PID);
		return NULL;
	}

	D(sl, "\n*** stlink_force_open ***\n");
	switch (stlink_current_mode(sl)) {
	case STLINK_DEV_MASS_MODE:
		return sl;
	case STLINK_DEV_DEBUG_MODE:
		// TODO go to mass?
		return sl;
	}
	fprintf(stderr, "\n*** switch the stlink to mass mode ***\n");
	stlink_exit_dfu_mode(sl);
	// exit the dfu mode -> the device is gone
	fprintf(stderr, "\n*** reopen the stlink device ***\n");
	delay(1000);
	stlink_close(sl);
	delay(5000);

	sl = stlink_open(dev_name, verbose);
	if (sl == NULL) {
		fprintf(stderr, "Error: failed to open the STLink device.\n");
		return NULL;
	}
	// re-query device info
	stlink_version(sl);
	return sl;
}

static void stm_info(struct stlink* sl)
{
	/* Read the device parameters: flash size and serial number. */
	stlink_read_mem32(sl, 0x1FFFf7e0, 16);
	printf("Flash size %dK (register %4.4x).\n",
		   *(uint16_t*)sl->q_buf, *(uint16_t*)(sl->q_buf+2));
	stlink_read_mem32(sl, 0x1FFFf800, 16);
	printf("Information block %8.8x %8.8x %8.8x %8.8x.\n",
		   *(uint32_t*)sl->q_buf, *(uint32_t*)(sl->q_buf+4),
		   *(uint32_t*)(sl->q_buf+8), *(uint32_t*)(sl->q_buf+12));
	stlink_read_mem32(sl, 0xE0042000, 4);
	printf("DBGMC_IDCODE %3.3x (Rev ID %4.4x).\n",
		   0x0FFF & *(uint16_t*)sl->q_buf, *(uint16_t*)(sl->q_buf+2));
	return;
}

/* Blink the LEDs on a STM32VLDiscovery board.
 * The LEDs are on PortC pins PC8 and PC9
 * RM0041 Reference manual - STM32F100xx advanced ARM-based 32-bit MCUs
 */
#define GPIOC		0x40011000 /* PortC register base */
#define GPIOC_CRH	(GPIOC + 0x04) /* Port configuration register high */
#define GPIOC_ODR	(GPIOC + 0x0c) /* Port output data register */
#define LED_BLUE	(1<<8)
#define LED_GREEN	(1<<9)

static void stm_discovery_blink(struct stlink* sl)
{
	uint32_t PortCh_iocfg;
	int i;

	PortCh_iocfg = sl_rd32(sl, GPIOC_CRH);
	if (sl->verbose)
		fprintf(stderr, "GPIOC_CRH = 0x%08x", PortCh_iocfg);

	/* Set PC8 and PC9 to outputs. */
	sl_wr32(sl, GPIOC_CRH, (PortCh_iocfg & ~0xff) | 0x11);
	for (i = 0; i < 10; i++) {
		sl_wr32(sl, GPIOC_ODR, LED_GREEN);
		usleep(100* 1000);
		sl_wr32(sl, GPIOC_ODR, LED_BLUE);
		usleep(100* 1000);
	}
	sl_wr32(sl, GPIOC_CRH, PortCh_iocfg);  /* Restore original pin settings. */
	return;
}

void stlink_discovery_blink(struct stlink* sl)
{
	uint32_t io_conf;
	int i;

	stlink_read_mem32(sl, GPIOC_CRH, 4);
	io_conf = read_uint32(sl->q_buf, 0);
	if (sl->verbose)
		fprintf(stderr, "GPIOC_CRH = 0x%08x", io_conf);

	/* Set general purpose output push-pull, output mode, max speed 10 MHz. */
	write_uint32(sl->q_buf, 0x44444411);
	stlink_write_mem32(sl, GPIOC_CRH, 4);

	clear_q_buf(sl);
	for (i = 0; i < 100; i++) {
		write_uint32(sl->q_buf, LED_BLUE | LED_GREEN);
		stlink_write_mem32(sl, GPIOC_ODR, 4);
		/* stlink_read_mem32(sl, 0x4001100c, 4); */
		/* fprintf(stderr, "GPIOC_ODR = 0x%08x", read_uint32(sl->q_buf, 0)); */
		delay(100);

		clear_q_buf(sl);
		stlink_write_mem32(sl, GPIOC_ODR, 4); // PC lo
		delay(100);
	}
	write_uint32(sl->q_buf, io_conf); // set old state
	return;
}

static void __attribute__((unused)) mark_buf(struct stlink *sl)
{
	clear_q_buf(sl);
	sl->q_buf[0] = 0x12;
	sl->q_buf[1] = 0x34;
	sl->q_buf[2] = 0x56;
	sl->q_buf[3] = 0x78;
	sl->q_buf[4] = 0x90;
	sl->q_buf[15] = 0x42;
	sl->q_buf[16] = 0x43;
	sl->q_buf[63] = 0x42;
	sl->q_buf[64] = 0x43;
	sl->q_buf[1024 * 6 - 1] = 0x42; //6kB
	sl->q_buf[1024 * 8 - 1] = 0x42; //8kB
}

int main(int argc, char *argv[])
{
    char *program;		/* Program name without path. */
    int c, errflag = 0;
	char *dev_name;				/* Path of SCSI device e.g. "/dev/sg1" */
	char *upload_path = 0, *download_path = 0, *verify_path = 0;
	int i, do_blink = 0;

    program = rindex(argv[0], '/') ? rindex(argv[0], '/') + 1 : argv[0];

	while ((c = getopt_long(argc, argv, short_opts, long_options, 0)) != -1) {
		switch (c) {
		case 'B': do_blink++; break;
		case 'C': verify_path = optarg; break;
		case 'D': download_path = optarg; break;
		case 'U': upload_path = optarg; break;
		case 'h':
		case 'u': printf(usage_msg, program); return 0;
		case 'v': verbose++; break;
		case 'V': printf("%s\n", version_msg); return 0;
		default:
		case '?': errflag++; break;
		}
    }

    if (errflag || argv[optind] == NULL) {
		fprintf(stderr, usage_msg, program);
		return errflag ? 1 : 2;
    }

	if (verbose)
		fprintf(stderr, "Using sg_lib %s and scsi_pt %s\n",
				sg_lib_version(), scsi_pt_version());

	dev_name = argv[optind];
	struct stlink *sl = stlink_force_open(dev_name, verbose);
	if (sl == NULL)
		return EXIT_FAILURE;

	/* When we open the device it is likely in mass storage mode.
	 * Switch to Single Wire Debug (SWD) mode and issue the required
	 * first command, reading the core ID.
	 */
	stl_enter_SWD_mode(sl);
	stlink_current_mode(sl);
	{
		uint32_t core_id = stl_get_core_id(sl);
		if (core_id != 0x1BA01477)
			fprintf(stderr, "Warning: SWD core ID %8.8x did not match the "
					"expected value of %8.8x.\n", core_id, 0x1BA01477);
	}

	while (argv[++optind]) {
		char *cmd = argv[optind];
		if (verbose) printf("Executing command %s.\n", argv[optind]);

		if (strcmp("regs", cmd) == 0) {
			printf("Register 0 is %8.8x.\n", stl_get_1reg(sl, 0));

			stlink_read_all_regs(sl);
			stlink_print_arm_regs(&sl->reg);
			stl_get_allregs(sl);
			for (i = 0; i < 21; i++)
				sl->reg.r[i] = ((uint32_t *)sl->q_buf)[i];
			stlink_print_arm_regs(&sl->reg);
		} else if (strncmp("flash:r:", cmd, 8) == 0) {
			char *path = cmd + 8;
			/* Read the program area. */
			fprintf(stderr, " Reading ARM memory 0x%8.8x..0x%8.8x into %s.\n",
					sl->flash_base, sl->flash_base+sl->flash_size, path);
			stlink_fread(sl, path, sl->flash_base, sl->flash_size);
		} else if (strncmp("flash:w:", cmd, 8) == 0) {
		} else if (strncmp("flash:v:", cmd, 8) == 0) {
			char *path = cmd + 8;
			const int res = stlink_fcheck_flash(sl, path, sl->flash_base);
			printf("  Check flash: file %s %s flash contents\n", path,
				   res == 0 ? "matched" : "did not match");
		} else if (strcmp("run", cmd) == 0) {
			stl_state_run(sl);
		} else if (strcmp("status", cmd) == 0) {
			unsigned status = stl_get_status(sl);
			printf("ARM status is 0x%4.4x: %s.\n", status,
				   status==STLINK_CORE_RUNNING ? "running" :
				   (status==STLINK_CORE_HALTED ? "halted" : "unknown"));
		} else if (strcmp("blink", cmd) == 0) {
			stm_discovery_blink(sl);
		} else if (strcmp("info", cmd) == 0) {
			stm_info(sl);
		} else if (strcmp("write", cmd) == 0) {
			write_flash_mem16(sl, 0x08000ba0, 0xDBEC);
			write_flash_mem16(sl, 0x20000040, 0xDBEC);
		}
	}

	stlink_status(sl);
	//stlink_force_debug(sl);
	stlink_reset(sl);
	stlink_status(sl);
#if 0
	// core system control block
	stlink_read_mem32(sl, 0xe000ed00, 4);
	fprintf(stderr, "cpu id base register: SCB_CPUID = got 0x%08x expect 0x411fc231", read_uint32(sl->q_buf, 0));
	// no MPU
	stlink_read_mem32(sl, 0xe000ed90, 4);
	fprintf(stderr, "mpu type register: MPU_TYPER = got 0x%08x expect 0x0", read_uint32(sl->q_buf, 0));

	stlink_read_mem32(sl, 0xe000edf0, 4);
	fprintf(stderr, "DHCSR = 0x%08x", read_uint32(sl->q_buf, 0));

	stlink_read_mem32(sl, 0x4001100c, 4);
	fprintf(stderr, "GPIOC_ODR = 0x%08x", read_uint32(sl->q_buf, 0));
#endif

#if 0
	// TODO rtfm: stlink doesn't have flash write routines
	// writing to the flash area confuses the fw for the next read access

	//stlink_read_mem32(sl, 0, 1024*6);
	// flash 0x08000000 128kB
	fputs("++++++++++ read a flash at 0x0800 0000\n", stderr);
	stlink_read_mem32(sl, 0x08000000, 1024 * 6); //max 6kB
	clear_q_buf(sl);
	stlink_read_mem32(sl, 0x08000c00, 5);
	stlink_read_mem32(sl, 0x08000c00, 4);
	mark_buf(sl);
	stlink_write_mem32(sl, 0x08000c00, 4);
	stlink_read_mem32(sl, 0x08000c00, 256);
	stlink_read_mem32(sl, 0x08000c00, 256);
#endif
#if 0
	// sram 0x20000000 8kB
	fputs("\n++++++++++ read/write 8bit, sram at 0x2000 0000 ++++++++++++++++\n\n", stderr);
	clear_q_buf(sl);
	stlink_write_mem8(sl, 0x20000000, 16);

	mark_buf(sl);
	stlink_write_mem8(sl, 0x20000000, 1);
	stlink_write_mem8(sl, 0x20000001, 1);
	stlink_write_mem8(sl, 0x2000000b, 3);
	stlink_read_mem32(sl, 0x20000000, 16);
#endif
#if 0
	// a not aligned mem32 access doesn't work indeed
	fputs("\n++++++++++ read/write 32bit, sram at 0x2000 0000 ++++++++++++++++\n\n", stderr);
	clear_q_buf(sl);
	stlink_write_mem8(sl, 0x20000000, 32);

	mark_buf(sl);
	stlink_write_mem32(sl, 0x20000000, 1);
	stlink_read_mem32(sl, 0x20000000, 16);
	mark_buf(sl);
	stlink_write_mem32(sl, 0x20000001, 1);
	stlink_read_mem32(sl, 0x20000000, 16);
	mark_buf(sl);
	stlink_write_mem32(sl, 0x2000000b, 3);
	stlink_read_mem32(sl, 0x20000000, 16);

	mark_buf(sl);
	stlink_write_mem32(sl, 0x20000000, 17);
	stlink_read_mem32(sl, 0x20000000, 32);
#endif
#if 0
	// sram 0x20000000 8kB
	fputs("++++++++++ read/write 32bit, sram at 0x2000 0000 ++++++++++++\n", stderr);
	mark_buf(sl);
	stlink_write_mem8(sl, 0x20000000, 64);
	stlink_read_mem32(sl, 0x20000000, 64);

	mark_buf(sl);
	stlink_write_mem32(sl, 0x20000000, 1024 * 8); //8kB
	stlink_read_mem32(sl, 0x20000000, 1024 * 6);
	stlink_read_mem32(sl, 0x20000000 + 1024 * 6, 1024 * 2);
#endif
#if 0
	stlink_read_all_regs(sl);
	stlink_step(sl);
	fputs("++++++++++ write r0 = 0x12345678\n", stderr);
	stlink_write_reg(sl, 0x12345678, 0);
	stlink_read_reg(sl, 0);
	stlink_read_all_regs(sl);
#endif
#if 0
	stlink_run(sl);
	stlink_status(sl);

	stlink_force_debug(sl);
	stlink_status(sl);
#endif
	/* read the system bootloader */
	if (upload_path) {
		fprintf(stderr, " Reading ARM memory 0x%8.8x..0x%8.8x bytes into %s.\n",
				sl->sys_base, sl->sys_base+sl->sys_size, upload_path);
		stlink_fread(sl, upload_path, sl->sys_base, sl->sys_size);
	}
#if 0 /* read the flash memory */
	fputs("\n+++++++ read flash memory\n\n", stderr);
	/* mark_buf(sl); */
	stlink_read_mem32(sl, 0x08000000, 4);
#endif
#if 0 /* flash programming */
	fputs("\n+++++++ program flash memory\n\n", stderr);
	stlink_fwrite_flash(sl, "/tmp/foobar", 0x08000000);
#endif
#if 0 /* check file contents */
	fputs("\n+++++++ check flash memory\n\n", stderr);
	{
	  const int res = stlink_fcheck_flash(sl, "/tmp/foobar", 0x08000000);
	  printf("_____ stlink_fcheck_flash() == %d\n", res);
	}
#endif
#if 0
	fputs("\n+++++++ sram write and execute\n\n", stderr);
	stlink_fwrite_sram(sl, "/tmp/foobar", sl->sram_base);
	stlink_run_at(sl, sl->sram_base);
#endif

	stlink_run(sl);
	stlink_status(sl);

	/* Switch back to mass storage mode before closing. */
	stlink_exit_debug_mode(sl);
	stlink_current_mode(sl);
	stlink_close(sl);

	return EXIT_SUCCESS;
}

/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
