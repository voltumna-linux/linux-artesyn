/*
 * Description:
 * 	Vital Product Data (VPD) Header-Module for ECC boards
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */


#ifndef __mot_vpd_h	
#define __mot_vpd_h

#define VPD_BLOCK_SIZE  512     /* size of VPD data block */
unsigned char palVpdBBBuffer[VPD_BLOCK_SIZE];

#define GET_VPD_BLOCK	0	/* IOCTL */

/*
 * packet identifiers
 *
 * 0x00 - 0x19	these packet id's are in use or reserved
 * 0x1a - 0xfd	reserved for future packet definition
 * 0xFE	 re-defined packet, set a packet ID to this to have packet
 *		ignored.
 * 0xff  termination packet
 */

#define VPD_PID_GI	0x00	/* guaranteed illegal */
#define VPD_PID_PID	0x01	/* product identifier (ASCII) */
#define VPD_PID_FAN	0x02	/* factory assembly-number (ASCII) */
#define VPD_PID_SN	0x03	/* serial-number (ASCII) */
#define VPD_PID_PCO	0x04	/* product configuration options (binary) */
#define VPD_PID_ICS	0x05	/* internal clock speed in HZ (integer) */
#define VPD_PID_ECS	0x06	/* external clock speed in HZ (integer) */
#define VPD_PID_RCS	0x07	/* reference clock speed in HZ (integer) */
#define VPD_PID_EA	0x08	/* ethernet address (binary) */
#define VPD_PID_MT	0x09	/* microprocessor type (ASCII) */
#define VPD_PID_CRC	0x0A	/* EEPROM CRC (integer) */
#define VPD_PID_FMC	0x0B	/* FLASH memory configuration (binary) */
#define VPD_PID_VLSI	0x0C	/* VLSI revisions/versions (binary) */
#define VPD_PID_PCS	0x0D	/* host PCI-bus clock speed (integer) */
#define VPD_PID_L2C	0x0E	/* l2 cache configuration (binary) */
#define VPD_PID_VPDR	0x0F	/* VPD revision (binary) */
#define VPD_PID_BCC	0x10	/* board component configuration (binary) */
#define VPD_PID_L3C     0x19    /* external L3 cache */ 
#define VPD_PID_TERM	0xFF	/* termination */

/*
 * VPD structure (format)
 */

#define VPD_EYE_SIZE	8	/* eyecatcher size */

typedef struct vpdHeader {
   unsigned char eyeCatcher[VPD_EYE_SIZE];	/* eyecatcher - "MOTOROLA" */
   unsigned char size[2];			/* size of EEPROM */
} vpdHeader_t;

/*
 * packet tuple structure (format)
 */

typedef struct vpdPacket {
   unsigned char identifier;	/* identifier (PIDs above) */
   unsigned char size;		/* size of the following data area */
   unsigned char data[1];	/* data (size is dependent upon PID) */
} vpdPacket_t;

/*
 * packet header definitions
 */

#define VPD_PHDR_OFFSET_ID	0	/* offset to identifier */
#define VPD_PHDR_OFFSET_SIZE	1	/* offset to size */

#define VPD_PHDR_SIZE		2	/* size of header */

/*
 * overall structure
 */

typedef struct vpd {
   vpdHeader_t header;		/* header */
   vpdPacket_t packets[1];	/* data */
} vpd_t;

/*
 * clock packet struct
 */

typedef struct vpdClk{
   unsigned char packetID;
   unsigned char dataSize;
   unsigned char clk[4];      /* Compiler alignment doesn't allow UINT here */
   unsigned char processor;   /* This stores a binary encoding 
   			       * indicating which processors this
			       * packet is used for:
			       * 0x1 :1st processor
			       * 0x2 :2nd processor
			       * 0x4 :3rd processor
			       * 0x8 :4th processor
			       * etc.
			       */
} vpdClk_t;

#define sizeof_VPD_CLK 4   /* Used to get the correct size of a structure. */

/*
 * CRC packet struct
 */

typedef struct vpdCrc {
   unsigned char packetId;   
   unsigned char dataSize; 
   unsigned char crc[4];   /* Compiler alignment doesn't allow UINT here */
} vpdCrc_t;

#define sizeof_VPD_CRC 4   /* Used to get the correct size of a sturcture. */

/*
 * Flash packet struct
 */

typedef struct vpdFlash {
   unsigned char packetId;
   unsigned char dataSize;
   unsigned char vendorID[2];
   unsigned char deviceID[2];
   unsigned char devWidth;
   unsigned char numDevices;
   unsigned char numColumns;
   unsigned char columnWidth;
   unsigned char writeWidth;
   unsigned char bankNum;
   unsigned char accessSpeed;
   unsigned char bankSize;
} vpdFlash_t;
   
/*
 * L2 cache packet struct
 */

typedef struct vpdL2Cache {
   unsigned char packetId;
   unsigned char dataSize;
   unsigned char vendorID[2];
   unsigned char deviceID[2];
   unsigned char devWidth;
   unsigned char numDevices;
   unsigned char numColumns;
   unsigned char columnWidth;
   unsigned char cacheType;
   unsigned char processor;		/* This stores a binary encoding 
   					 * indicating which processors this
					 * cache packet is used for:
					 * 0x1 :1st processor
					 * 0x2 :2nd processor
					 * 0x4 :3rd processor
					 * 0x8 :4th processor
					 * etc.
					 */
   unsigned char operationMode;
   unsigned char errorType;		/* data/tag error detection type 
   					 * 0:none 1:parity 2:ecc
					 */
   unsigned char cacheSize;
   unsigned char cacheSettings;
   unsigned char coreRatio;
} vpdL2cache_t;

/*
 * revision packet struct
 */

typedef struct vpdRevision {
   unsigned char packetId;
   unsigned char dataSize;
   unsigned char vpdType;
   unsigned char archRev;
   unsigned char buildRev;
   unsigned char reasonCode;
} vpdRevision_t;
   
/*
 * ethernet packet struct
 */

typedef struct vpdEthernet {
   unsigned char packetId;
   unsigned char dataSize;
   unsigned char address[6];
   unsigned char instanceNum;
} vpdEthernet_t;

/*
 * L3 cache packet struct
 * This structure contains elements which could change when the
 * L3-specific board artwork or the SRAM devices are changed.  
 */

typedef struct vpdL3Cache {
   unsigned char packetId;
   unsigned char dataSize;
   unsigned char processor;		/* This stores a binary encoding 
   					 * indicating which processors this
					 * cache packet is used for:
					 * 0x1 :1st processor
					 * 0x2 :2nd processor
					 * 0x4 :3rd processor
					 * 0x8 :4th processor
					 * etc.
					 */
   unsigned char cacheSize; 		/* 0x0 = 1Mb, 0x1 = 2Mb */
   unsigned char clockDividerRatio;	/* core to cache clock ratio 
   					 * 0:/6 (value 0 == divide by 6) 
					 * 1:reserved 2:/2 3:/2.5 
					 * 4:/3 5:/3.5 6:/4 7:/5 
					 */
   unsigned char cksp;			/* cache clock sample point 
   					 * 0: 2 clocks, 1: 3 clocks 
   					 * 2: 4 clocks, 3: 5 clocks 
					 */
   unsigned char psp;			/* processor clock sample point 
   					 * values 0 thru 5 indicate the clock
					 * the sample should occur in.  values
					 * 6 and 7 are reserved. 
					 */
   unsigned char spo;			/* sample point override 
   					 * 0:disabled, 1:enabled
					 */
   unsigned char nirca;			/* SRAM clock control 
   					 * 0:special timing disabled
					 * 1:special timing enabled
					 */
   unsigned char sramType;		/* 0:DDR, 1:late-write, 
   					 * 2:reserved, 3:PB2 SRAM 
					 */
   unsigned char dataErrCk;		/* data bus error detection type 
   					 * 0:none 1:parity 
					 */
   unsigned char addrErrCk;		/* address bus error detection type 
   					 * 0:none 1:parity 
					 */
} vpdL3Cache_t;

#endif				/* __mot_vpd_h */
