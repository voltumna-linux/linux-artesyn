/*
 * Module name: mot_vpd.c
 * Description:
 *
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
#include "mot_vpd.h"		/* vital product data */

/*
 * data-buffer types
 */

#define VPD_TYPE_ASCII	0	/* ascii string data */
#define VPD_TYPE_ARRAY	1	/* array of data (some number of bytes) */
#define VPD_TYPE_INT32	2	/* 32-bit integer */

unsigned char *vpdGetPacket(vpdHeader_t *pVpdHeader,
			unsigned int vpdSromSize,
			unsigned int vpdPid, 
			unsigned char *pVpdPacket)
{
	unsigned int sromOffset;
	unsigned int vpdPidLocal;
	unsigned char *pVpdBuffer;

	/*
	 * assign buffer pointer to the first packet of scan
	 */
	if (pVpdPacket == (unsigned char *)0) {
		/*
		 * assign pointer to the first packet (skip over header)
		 */
		pVpdBuffer = (unsigned char *)pVpdHeader + sizeof(vpdHeader_t);
	} else {
		/*
		 * verify the packet pointer is real (in bounds to the buffer)
		 */
		sromOffset =
		    (unsigned int)pVpdPacket - (unsigned int)pVpdHeader;
		if (sromOffset >= vpdSromSize) {
			return ((unsigned char *)0);
		}

		/*
		 * move pointer past the previously processed packet (i.e., to the
		 * next packet)
		 */
		pVpdBuffer =
		    pVpdPacket + *(pVpdPacket + VPD_PHDR_OFFSET_SIZE) +
		    VPD_PHDR_SIZE;

		/*
		 * verify the size of the SROM was not exceeded
		 */
		sromOffset =
		    (unsigned int)pVpdBuffer - (unsigned int)pVpdHeader;
		if (sromOffset >= vpdSromSize) {
			return ((unsigned char *)0);
		}
	}

	/*
	 * scan VPD buffer for the desired packet
	 */
	for (;;) {
		/*
		 * retrieve packet identifier
		 */
		vpdPidLocal = (unsigned int)*(pVpdBuffer + VPD_PHDR_OFFSET_ID);

		/*
		 * check for termination packet
		 */
		if (vpdPidLocal == VPD_PID_TERM) {
			break;
		}

		/*
		 * does the packet identifier match?, if so, return pointer
		 * to packet and exit
		 */
		if (vpdPid == vpdPidLocal) {
			return (pVpdBuffer);
		}

		/*
		 * move pointer to the next packet
		 */
		pVpdBuffer =
		    pVpdBuffer + *(pVpdBuffer + VPD_PHDR_OFFSET_SIZE) +
		    VPD_PHDR_SIZE;

		/*
		 * verify the size of the SROM was not exceeded
		 */
		sromOffset =
		    (unsigned int)pVpdBuffer - (unsigned int)pVpdHeader;
		if (sromOffset >= vpdSromSize) {
			break;
		}
	}

	return ((unsigned char *)0);
}
