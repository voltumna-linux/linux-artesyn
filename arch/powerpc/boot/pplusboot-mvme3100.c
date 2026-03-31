/*
 * Motorola ECC MVME3100 bootwrapper code.
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * 2007 (c) Motorola, Inc.  
 *
 * This file is licensed under * the terms of the GNU 
 * General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, 
 * whether express or implied.
 */

#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "elf.h"
#include "page.h"
#include "string.h"
#include "stdio.h"
#include "io.h"
#include "ops.h"
#include "gunzip_util.h"
#include "mvme3100.h"
#include "mot_vpd.h"
#include "i2c_mpc.h"
#include "ppcboot.h"

extern char _end[];
extern char _vmlinux_start[], _vmlinux_end[];
extern char _dtb_start[], _dtb_end[];


extern unsigned char *vpdGetPacket(vpdHeader_t *pVpdHeader,
                        unsigned int vpdSromSize,
                        unsigned int vpdPid,
                        unsigned char *pVpdPacket);

extern int i2c_mpc_open(void);
extern int i2c_mpc_read(u8 dev_addr, u32 offset, u32 offset_size, u8 *buf, u32 len);
extern int i2c_mpc_write(u8 dev_addr, u32 offset, u32 offset_size, u8 *buf, u32 len);
extern void i2c_mpc_close(void);


static unsigned char vpd_buffer[VPD_BLOCK_SIZE];
static unsigned char enet0_addr[6];
static unsigned char enet1_addr[6];
static unsigned char enet2_addr[6];
static bd_t bdinfo;

#define KB	1024U
#define MB	(KB*KB)

#define EEPROM_ADDR	0xA8

BSS_STACK(16 * KB);

/* Get VPD from i2c eeprom */
static void mvme3100_get_board_info(void)
{
	int rc;

	if (i2c_mpc_open())
		fatal("Error: Can't open i2c device\n\r");

	/* Get VPD from i2c eeprom */
	rc = i2c_mpc_read(EEPROM_ADDR, 0x0, 2, vpd_buffer, sizeof(vpd_buffer));
	if (rc < 0)
		fatal("Error: Couldn't read eeprom\n\r");
	i2c_mpc_close();
}

static void mvme3100_reset(void)
{
	out_8((u8 *) MVME3100_SYSTEM_CONTROL_REG, MVME3100_BOARD_RESET);

	for (;;) 
		;
}

static void mvme3100_fixups(void)
{
	int i;
	void *devp;
	vpdEthernet_t *vpd_eth_packet;
	u32 v[2];

	mvme3100_get_board_info();

	/* Now fixup device tree properties */
	/* Set /model appropriately */
	devp = finddevice("/");
	if (devp == NULL)
		fatal("Error: Missing '/' device tree node\n\r");

	/* Set /cpus/PowerPC,8540/clock-frequency */
	devp = finddevice("/cpus/PowerPC,8540");
	if (devp == NULL)
		fatal("Error: Missing proper /cpus device tree node\n\r");

	v[0] = bdinfo.bi_intfreq;
	v[0] *= 1000000;
	setprop(devp, "clock-frequency", &v[0], sizeof(v[0]));

	v[0] = bdinfo.bi_busfreq;
	v[0] *= 1000000;
	setprop(devp, "bus-frequency", &v[0], sizeof(v[0]));

        v[0] /= 8;
        setprop(devp, "timebase-frequency", &v[0], sizeof(v[0]));

	/* Set memory size */
	devp = finddevice("/memory");
	if (devp == NULL)
		fatal("Error: Missing /memory device tree node\n\r");
	v[0] = 0;
	v[1] = bdinfo.bi_memsize;
	setprop(devp, "reg", v, sizeof(v));

        devp = finddevice("/soc8540@e1000000");
        if (devp == NULL)
                fatal("Error: Missing soc8540 device tree node\n\r");
        v[0] = bdinfo.bi_busfreq;
        v[0] *= 1000000;
        setprop(devp, "bus-frequency", &v[0], sizeof(v[0]));

        devp = finddevice("/soc8540/serial@4500");
        if (devp == NULL)
                fatal("Error: Missing soc8540 serial device tree node\n\r");
        v[0] = bdinfo.bi_busfreq;
        v[0] *= 1000000;
        setprop(devp, "clock-frequency", &v[0], sizeof(v[0]));

	vpd_eth_packet = (vpdEthernet_t *) 0;
	for (i = 0; i <= 2; i++) {
		vpd_eth_packet = (vpdEthernet_t *) vpdGetPacket(
					(vpdHeader_t *)&vpd_buffer[0],
					VPD_BLOCK_SIZE,
					VPD_PID_EA,
					(unsigned char *)vpd_eth_packet);
		if (vpd_eth_packet == NULL)
			fatal("Error: Could not determine MAC address from VPD\n\r");
		switch (vpd_eth_packet->instanceNum) {
		case 0:
			memcpy(enet0_addr, vpd_eth_packet->address, 6);
			break;
		case 1:
			memcpy(enet1_addr, vpd_eth_packet->address, 6);
			break;
		case 2:
			memcpy(enet2_addr, vpd_eth_packet->address, 6);
			break;
		default:
			break;
		}
	}

	devp = finddevice("/soc8540/ethernet@24000");
	if (devp == NULL)
		fatal("Error: Missing ethernet device tree node\n\r");
	setprop(devp, "local-mac-address", &enet0_addr[0], sizeof(enet0_addr));

	devp = finddevice("/soc8540/ethernet@25000");
	if (devp == NULL)
		fatal("Error: Missing ethernet device tree node\n\r");
	setprop(devp, "local-mac-address", &enet1_addr[0], sizeof(enet0_addr));

	devp = finddevice("/soc8540/ethernet@26000");
	if (devp == NULL)
		fatal("Error: Missing ethernet device tree node\n\r");
	setprop(devp, "local-mac-address", &enet2_addr[0], sizeof(enet0_addr));
}

#define HEAP_SIZE	(16*MB)
static struct gunzip_state gzstate;

void platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
		   unsigned long r6, unsigned long r7)
{
	struct elf_info ei;
	char *heap_start, *dtb;
	int dt_size = _dtb_end - _dtb_start;
	void *vmlinuz_addr = _vmlinux_start;
	unsigned long vmlinuz_size = _vmlinux_end - _vmlinux_start;
	char elfheader[256];
	bd_t *bp = (bd_t *) r3;

	memcpy(&bdinfo, bp, sizeof(bd_t));

	if (dt_size <= 0) {	/* No fdt */
		exit();
	}

	/*
	 * Start heap after end of the kernel (after decompressed to
	 * address 0) or the end of the zImage, whichever is higher.
	 * That's so things allocated by simple_alloc won't overwrite
	 * any part of the zImage and the kernel won't overwrite the dtb
	 * when decompressed & relocated.
	 */
	gunzip_start(&gzstate, vmlinuz_addr, vmlinuz_size);
	gunzip_exactly(&gzstate, elfheader, sizeof(elfheader));

	if (!parse_elf32(elfheader, &ei))
		exit();

	heap_start = (char *)(ei.memsize + ei.elfoffset);/* end of kernel */
	heap_start = max(heap_start, (char *)_end);	/* end of zImage */

	if ((unsigned)simple_alloc_init(heap_start, HEAP_SIZE, 2 * KB, 16)
	    > (128 * MB))
		exit();

	/* Relocate dtb to safe area past end of zImage & kernel */
	dtb = malloc(dt_size);
	if (!dtb)
		exit();
	memmove(dtb, _dtb_start, dt_size);
	if (ft_init(dtb, dt_size, 16))
		exit();

	platform_ops.fixups = mvme3100_fixups;
	platform_ops.exit = mvme3100_reset;

	if (serial_console_init() < 0)
		exit();
}

asm("	.globl _zimage_start\n\
	_zimage_start:\n\
		mfmsr   10\n\
		rlwinm  10,10,0,~(1<<15)        /* Clear MSR_EE */\n\
		sync\n\
		mtmsr   10\n\
		isync\n\
		b _zimage_start_lib\n\
");
