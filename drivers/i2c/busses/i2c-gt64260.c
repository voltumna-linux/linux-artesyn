/*
 * I2C driver for GT64260, mostly for Vital Product access.
 *

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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include "mot_vpd.h"

#define VERSION "1.0.1"

static int __init gt64260_i2c_init(void);

/* I2C Srom defintions */
static int deviceType = 1;
static unsigned int deviceAddress = 0xA8;
static unsigned int baudRate = 0x2C;

/*
 * Discovery I2C controller registers
 */

#define DISCOVERY_I2C_SLAVE_ADDR 	0xC000
#define DISCOVERY_I2C_DATA_REG 		0xC004
#define DISCOVERY_I2C_CONTROL_REG 	0xC008
#define DISCOVERY_I2C_STATUS_REG 	0xC00C
#define DISCOVERY_I2C_EXTSLAVE_ADDR 	0xC010
#define DISCOVERY_I2C_RESET_REG 	0xC01C

/*
 * Discovery I2C controller register structure template
 */

typedef struct i2cDiscovery {
	unsigned int slaveAddr;	/* 0xC000 - slave address */
	unsigned int data;	/* 0xC004 - data */
	unsigned int control;	/* 0xC008 - control */
	unsigned int status;	/* 0xC00C - status */
	unsigned int extSlaveAddr;	/* 0xC010 - extended slave address */
	unsigned int reserved[2];	/* 0xC014 - 0xC018 */
	unsigned int reset;	/* 0xC01C - reset */
} i2cDiscovery_t;

/*
 * DISCOVERY_I2C_CONTROL_REG bits
 */

#define DISCOVERY_I2C_CONTROL_REG_ACK		(1<<2)
#define DISCOVERY_I2C_CONTROL_REG_INTFLG	(1<<3)
#define DISCOVERY_I2C_CONTROL_REG_STOP		(1<<4)
#define DISCOVERY_I2C_CONTROL_REG_START		(1<<5)
#define DISCOVERY_I2C_CONTROL_REG_ENABLE    	(1<<6)
#define DISCOVERY_I2C_CONTROL_REG_INTENABLE	(1<<7)

/*
 * DISCOVERY_I2C_STATUS_REG bits
 */
#define DISCOVERY_I2C_STATUS_BUSERR 			0x00
#define DISCOVERY_I2C_STATUS_STARTTX			0x08
#define DISCOVERY_I2C_STATUS_REPEATSTARTX	 	0x10
#define DISCOVERY_I2C_STATUS_ADDRWRITEACK 		0x18
#define DISCOVERY_I2C_STATUS_ADDRWRITENACK 		0x20
#define DISCOVERY_I2C_STATUS_MSTDATAWRITEACK		0x28
#define DISCOVERY_I2C_STATUS_MSTDATAWRITENACK		0x30
#define DISCOVERY_I2C_STATUS_MSTLOSTARB			0x38
#define DISCOVERY_I2C_STATUS_ADDRREADACK		0x40
#define DISCOVERY_I2C_STATUS_ADDRREADNACK		0x48
#define DISCOVERY_I2C_STATUS_MSTDATARECVACK		0x50
#define DISCOVERY_I2C_STATUS_MSTDATARECVNACK		0x58
#define DISCOVERY_I2C_STATUS_SLVADDRRECACK		0x60
#define DISCOVERY_I2C_STATUS_MSTLOSTARBADDR		0x68
#define DISCOVERY_I2C_STATUS_GENCALLRECVACK		0x70
#define DISCOVERY_I2C_STATUS_MSTLOSTARBADDRGC		0x78
#define DISCOVERY_I2C_STATUS_SLVRECWDATAACK		0x80
#define DISCOVERY_I2C_STATUS_SLVRECWDATANACK		0x88
#define DISCOVERY_I2C_STATUS_SLVRECWDATAACKGC		0x90
#define DISCOVERY_I2C_STATUS_SLVRECWDATANACKGC		0x98
#define DISCOVERY_I2C_STATUS_SLVRECSTOP			0xA0
#define DISCOVERY_I2C_STATUS_SLVRECADDRACK		0xA8
#define DISCOVERY_I2C_STATUS_MSTLOSTARBADDRtx		0xB0
#define DISCOVERY_I2C_STATUS_SLVTXRDATAACK		0xB8
#define DISCOVERY_I2C_STATUS_SLVTXRDATANACK		0xC0
#define DISCOVERY_I2C_STATUS_SLVTXLASTDATAACK		0xC8
#define DISCOVERY_I2C_STATUS_SECADDRWTXACK		0xD0
#define DISCOVERY_I2C_STATUS_SECADDRWTXNACK		0xD8
#define DISCOVERY_I2C_STATUS_SECADDRRTXACK		0xE0
#define DISCOVERY_I2C_STATUS_SECADDRRTXNACK		0xE8
#define DISCOVERY_I2C_STATUS_NOSTATUS			0xF8

/*
 * command identifiers
 */
#define I2C_READOP	0	/* read operation */
#define I2C_WRITOP	1	/* write operation */
#define I2C_FRMTOP	2	/* format operation */
#define I2C_CHCKOP	3	/* check operation */

/*
 * routine call macros
 */
#define I2C_CYCLE_ACKIN		 i2cCycleDiscoveryAckIn(pI2cDiscovery)
#define I2C_CYCLE_ACKOUT	 i2cCycleDiscoveryAckOut(pI2cDiscovery)
#define I2C_CYCLE_READ(ad)	 i2cCycleDiscoveryRead(pI2cDiscovery,ad)
#define I2C_CYCLE_START		 i2cCycleDiscoveryStart(pI2cDiscovery)
#define I2C_CYCLE_STOP		 i2cCycleDiscoveryStop(pI2cDiscovery)
#define I2C_CYCLE_WRITE(wd)	 i2cCycleDiscoveryWrite(pI2cDiscovery,wd)
#define I2C_KNOWN_STATE		 i2cCycleDiscoveryKnownState(pI2cDiscovery)

#define I2C_BYTE_ADDRESS(address,offset) (((address)>>(offset*8))&0xff)

#define I2C_DEVICE_TYPE_EEPROM          0	/* 1-byte address/index */
#define I2C_DEVICE_TYPE_EEPROM_AB2      1	/* 2-byte address/index */
#define I2C_DEVICE_TYPE_EEPROM_AB3      2	/* 3-byte address/index */

/*
 * error codes
 */

#define I2C_ERROR_CYCLE_START	1	/* start cycle */
#define I2C_ERROR_CYCLE_STOP	2	/* stop cycle */
#define I2C_ERROR_CYCLE_READ	3	/* read cycle */
#define I2C_ERROR_CYCLE_WRITE	4	/* write cycle */
#define I2C_ERROR_CYCLE_ACKIN	5	/* acknowledge in cycle */
#define I2C_ERROR_CYCLE_ACKOUT	6	/* acknowledge out cycle */
#define I2C_ERROR_KNOWN_STATE	7	/* known state */

int gt64260_i2cMacGet(char *ptr, int index);

/*
 * global data
 */

static int I2cInitFlag;

static unsigned int i2cAddressMunge(unsigned int deviceAddress,
				    unsigned int byteOffset);
static unsigned int i2cAddressMunge(unsigned int deviceAddress,
				    unsigned int byteOffset);
static int i2cCycleDiscoveryStart(i2cDiscovery_t * pI2cDiscovery);
static int i2cCycleDiscoveryStop(i2cDiscovery_t * pI2cDiscovery);
static int i2cCycleDiscoveryRead(i2cDiscovery_t * pI2cDiscovery,
				 unsigned char *DataBuf);
static int i2cCycleDiscoveryWrite(i2cDiscovery_t * pI2cDiscovery,
				  unsigned int writeData);
static int i2cCycleDiscoveryAckIn(i2cDiscovery_t * pI2cDiscovery);
static int i2cCycleDiscoveryKnownState(i2cDiscovery_t * pI2cDiscovery);
static int i2cIntFlagRead(i2cDiscovery_t * pI2cDiscovery);
static int i2cDoOpLowLevel(int cmd, char *palVpdBBBuffer);
static unsigned char *vpdGetPacket(vpdHeader_t * pVpdHeader,
				   unsigned int vpdSromSize,
				   unsigned int vpdPid,
				   unsigned char *pVpdPacket);

static int i2cReadFlag;
static i2cDiscovery_t *pI2cDiscovery = (i2cDiscovery_t *) (0xf1000000 + 0xC000);

/*
 * i2cCycleDiscoveryStart - perform I2C "start" cycle
 * description:
 *	This function's purpose is to perform an I2C start cycle.
 * call:
 *	argument #1 = pointer to controller device
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */

static int i2cCycleDiscoveryStart(i2cDiscovery_t * pI2cDiscovery)
{
	unsigned int timeOutCount;
	unsigned int statusReg;
	unsigned int data = 0;

	/*
	 * init the I2C interface, if needed.
	 */
	i2cCycleDiscoveryKnownState(pI2cDiscovery);

	/*
	 * START OF NORMAL DRIVER CODE
	 */
	data |= (DISCOVERY_I2C_CONTROL_REG_START |
		 DISCOVERY_I2C_CONTROL_REG_ENABLE);
	writel(data, &pI2cDiscovery->control);
	udelay(1000);

	/*
	 * verify the INT FLAG bit in the CONTROL register is set
	 * before checking the STATUS register.
	 */
	if (i2cIntFlagRead(pI2cDiscovery)) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (1);	/* indicates an ERROR occurred */
	}

	/*
	 * check for completion of START sequence
	 */
	for (timeOutCount = 10; timeOutCount; timeOutCount--) {
		statusReg = readl(&pI2cDiscovery->status);
#ifdef DISCO_I2C_DRV_DEBUG
		printf("START: status reg = 0x%04X\r\n", statusReg);
#endif
		if ((statusReg == DISCOVERY_I2C_STATUS_STARTTX) ||
		    (statusReg == DISCOVERY_I2C_STATUS_REPEATSTARTX)) {
			break;
		}
		udelay(1000);
	}

	/*
	 * if we timedout without notification of a completed START cycle,
	 * bailout and return with an ERROR status.
	 */
	if (!timeOutCount) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (-1);
	}

	return (0);
}

/*
 * i2cCycleDiscoveryStop - perform I2C "stop" cycle
 * description:
 *	This function's purpose is to perform an I2C stop cycle.
 * call:
 *	argument #1 = pointer to controller device
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */
static int i2cCycleDiscoveryStop(i2cDiscovery_t * pI2cDiscovery)
{
	unsigned int controlReg;

	/*
	 * set the STOP bit.  This does not cause a STOP
	 * bus transaction until the INT FLAG has been
	 * cleared.  The INT FLAG can be cleared after the
	 * interrupt flag bit has been set in the control register.
	 */
	controlReg = readl(&pI2cDiscovery->control);
	controlReg |= (DISCOVERY_I2C_CONTROL_REG_STOP);
	writel(controlReg, &pI2cDiscovery->control);
	udelay(1000);

	/*
	 * verify the INT FLAG bit in the CONTROL register is set
	 * before trying to clear it.
	 */
	if (i2cIntFlagRead(pI2cDiscovery)) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (1);	/* indicates an ERROR occurred */
	}

	/*
	 * clear the INT FLAG, but do NOT turn off the interface
	 * by clearing the control register.  A STOP will be transmitted
	 * on the bus 'after' the INT FLAG has been cleared...
	 */
	controlReg = readl(&pI2cDiscovery->control);
	controlReg &= (~DISCOVERY_I2C_CONTROL_REG_INTFLG);
	writel(controlReg, &pI2cDiscovery->control);
	udelay(1000);

	return (0);
}

/*
 * i2cCycleDiscoveryRead - perform I2C "read" cycle
 * description:
 *	This function's purpose is to perform an I2C read cycle.
 * call:
 *	argument #1 = pointer to controller device
 *	argument #2 = pointer to read data buffer
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */

static int
i2cCycleDiscoveryRead(i2cDiscovery_t * pI2cDiscovery, unsigned char *DataBuf)
{
	unsigned int statusReg;
	unsigned int timeOutCount;

	/*
	 * clear the INT FLAG by writing the ACK and ENABLE bits.
	 */
	writel((DISCOVERY_I2C_CONTROL_REG_ACK |
		DISCOVERY_I2C_CONTROL_REG_ENABLE), &pI2cDiscovery->control);
	udelay(1000);

	/*
	 * verify the INT FLAG bit in the CONTROL register is set
	 * before checking the STATUS register.
	 */
	if (i2cIntFlagRead(pI2cDiscovery)) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (1);	/* indicates an ERROR occurred */
	}

	/*
	 * check for READ status of ACK.
	 */
	for (timeOutCount = 10; timeOutCount; timeOutCount--) {
		statusReg = readl(&pI2cDiscovery->status);

/*
      printk("READ: status reg = 0x%04X\r\n", statusReg);
*/
		if ((statusReg == DISCOVERY_I2C_STATUS_MSTDATARECVACK) ||
		    (statusReg == DISCOVERY_I2C_STATUS_ADDRWRITENACK)) {
			/*
			 * get the data requested by the user
			 */
			*DataBuf = readl(&pI2cDiscovery->data);
			udelay(1000);
			writel(DISCOVERY_I2C_CONTROL_REG_ENABLE,
			       &pI2cDiscovery->control);
			udelay(1000);
			break;
		}
		udelay(1000);
	}

	/*
	 * if we timedout without notification of a completed READ cycle,
	 * bailout and return with an ERROR status.
	 */
	if (!timeOutCount) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (-1);
	}

	/*
	 * if we get here, then we have good data
	 */
	return (0);
}

/*
 * i2cCycleDiscoveryWrite - perform I2C "write" cycle
 * description:
 *	This function's purpose is to perform an I2C write cycle.
 * call:
 *	argument #1 = pointer to controller device
 *	argument #2 = write data
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */

static int
i2cCycleDiscoveryWrite(i2cDiscovery_t * pI2cDiscovery, unsigned int writeData)
{
	unsigned int statusReg;
	unsigned int timeOutCount;

	/*
	 * write the data into the I2C data register, 
	 * then clear the INT FLAG to drive the data onto the bus.
	 */
	writel(writeData, &pI2cDiscovery->data);
	writel(0, &pI2cDiscovery->control);
	udelay(1000);

	/*
	 * verify the INT FLAG bit in the CONTROL register is set
	 * before checking the STATUS register.
	 */
	if (i2cIntFlagRead(pI2cDiscovery)) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (1);	/* indicates an ERROR occurred */
	}

	for (timeOutCount = 10; timeOutCount; timeOutCount--) {
		udelay(1000);
		statusReg = readl(&pI2cDiscovery->status);

		if (statusReg == DISCOVERY_I2C_STATUS_ADDRWRITENACK) {
			/*
			 * no device at the specified address is responding,
			 * so generate a STOP and return with a status ERROR.
			 */
			i2cCycleDiscoveryStop(pI2cDiscovery);
			return (1);
		}

		if ((statusReg == DISCOVERY_I2C_STATUS_ADDRWRITEACK) ||
		    (statusReg == DISCOVERY_I2C_STATUS_MSTDATAWRITEACK) ||
		    (statusReg == DISCOVERY_I2C_STATUS_ADDRREADACK)) {
			break;
		}

		/*
		 * since we haven't received the correct status, keep trying
		 * until the loop count has expired.
		 */
		udelay(1000);
	}

	/*
	 * if we timedout without notification of a completed WRITE cycle,
	 * bailout and return with an ERROR status.
	 */
	if (!timeOutCount) {
		i2cCycleDiscoveryStop(pI2cDiscovery);
		return (-1);
	}

	return (0);
}

/*
 * i2cCycleDiscoveryAckIn - perform I2C "acknowledge-in" cycle
 * description:
 *	This function's purpose is to perform an I2C acknowledge-in
 *	cycle.
 * call:
 *	argument #1 = pointer to controller device
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */

static int i2cCycleDiscoveryAckIn(i2cDiscovery_t * pI2cDiscovery)
{
	return (0);
}

/*
 * i2cCycleDiscoveryKnownState - initialize the I2C bus to a known state
 * description:
 *	This function's purpose is to initialize the I2C bus to a
 *	known state.  This can be called after an error has been
 *	determined, or to recover from a unstable state.
 * call:
 *	argument #1 = pointer to controller device
 * return:
 *	zero		= operation successful
 *	non-zero	= operation failed
 */

static int i2cCycleDiscoveryKnownState(i2cDiscovery_t * pI2cDiscovery)
{

	/*
	 * DEVICE INIT CODE
	 * The following code runs only once.  it's purpose is to
	 * initialize the I2C interface based on the external clock
	 * of the processor.
	 */
	if (I2cInitFlag == 0) {
		writel(0, &pI2cDiscovery->reset);
		udelay(1000);

		writel(baudRate, &pI2cDiscovery->status);
		udelay(1000);

		writel((DISCOVERY_I2C_CONTROL_REG_ACK |
			DISCOVERY_I2C_CONTROL_REG_ENABLE),
		       &pI2cDiscovery->control);
		udelay(4000);
		I2cInitFlag = 1;
	}

	return (0);
}

/*
 * i2cIntFlagRead - checks the control register for an interrupt.
 * description:
 *      This function's purpose is to read the CONTROL register
 *      of the Discovery device checking for the INT FLAG to
 *      be set.
 * call:
 *      argument #1 = pointer to controller device
 * return:
 *      zero            = INT FLAG bit is set
 *      non-zero        = INT FLAG bit is NOT set
 */

static int i2cIntFlagRead(i2cDiscovery_t * pI2cDiscovery)
{
	unsigned int status;
	unsigned int timeOutCount;

	for (timeOutCount = 10; timeOutCount; timeOutCount--) {
		status = readl(&pI2cDiscovery->control);
		if (status & DISCOVERY_I2C_CONTROL_REG_INTFLG) {
			return (0);
		} else {
			continue;
		}
		udelay(1000);
	}

	/*
	 * if we reach this point, then the INT FLAG did not get set.
	 * return with a value not equal to 0, as an indication of ERROR.
	 */
	return (1);
}

/*
 * i2cDoOpLowLevel - i2c do operation (low level)
 * description:
 *	This function's purpose is to execute the operation as specified
 *	by the passed command packet.  Currently, the only device types
 *	that are recognized are EEPROM types.
 * call:
 *	argument #1 = pointer to command packet
 *	argument #2 = controller type
 *	argument #3 = controller identifier
 *	argument #4 = device address
 *	argument #5 = device type
 *	argument #6 = pointer to device access routines
 * return:
 *	zero		= okay
 *	non-zero	= failure
 */

static int i2cDoOpLowLevel(int cmd, char *palVpdBBBuffer)
{
	int byteCount;		/* byte counter */
	int statusVariable = 0;	/* local status variable */
	unsigned long flags;	/* interrupt lock key */

	unsigned char *pWriteData;	/* pointer to write data buffer */

	/*
	 * read operation (EEPROM type devices), for each byte
	 * perform the random read operation
	 */
	if (cmd == I2C_READOP) {

		if (i2cReadFlag)
			return (0);
		/*
		 * read the specified number of bytes from the EEPROM
		 */
		statusVariable = 0;
		for (byteCount = 0; byteCount < VPD_BLOCK_SIZE; byteCount++) {
			if (byteCount == 0) {
				if (I2C_KNOWN_STATE) {
					statusVariable = I2C_ERROR_KNOWN_STATE;
					break;
				}
			}

			/*
			 * lock (disable) interrupts from taking place
			 */
			local_irq_save(flags);

			if (deviceType == I2C_DEVICE_TYPE_EEPROM) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (i2cAddressMunge(deviceAddress, byteCount)))
				{
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE(byteCount)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (i2cAddressMunge
				     (deviceAddress | 0x01, byteCount))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (deviceType == I2C_DEVICE_TYPE_EEPROM_AB2) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 1))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 0))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress | 0x01)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (deviceType == I2C_DEVICE_TYPE_EEPROM_AB3) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 2))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 1))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 0))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress | 0x01)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (I2C_CYCLE_READ(palVpdBBBuffer + byteCount)) {
				statusVariable = I2C_ERROR_CYCLE_READ;
				break;
			}

			if (I2C_CYCLE_STOP) {
				statusVariable = I2C_ERROR_CYCLE_STOP;
				break;
			}

			/*
			 * unlock (enable) interrupts
			 */
			local_irq_restore(flags);
		}

		/*
		 * leave the I2C bus in a known state
		 */
		I2C_KNOWN_STATE;

		/*
		 * update the caller's command packet with status of
		 * the operation
		 */
		i2cReadFlag = 1;

		return (0);
	}

	/*
	 * write operation (EEPROM type devices), for each byte
	 * perform the byte write operation, a delay must be
	 * exercised following each byte write
	 */
	if (cmd == I2C_WRITOP) {
		/*
		 * initialize pointer to caller's write data buffer
		 */
		pWriteData = (unsigned char *)palVpdBBBuffer;

		/*
		 * write the specified number of bytes from the EEPROM
		 */
		statusVariable = 0;
		for (byteCount = 0; byteCount < VPD_BLOCK_SIZE; byteCount++) {
			if (byteCount == 0) {
				if (I2C_KNOWN_STATE) {
					statusVariable = I2C_ERROR_KNOWN_STATE;
					break;
				}
			}

			/*
			 * lock (disable) interrupts from taking place
			 */
			local_irq_save(flags);

			if (deviceType == I2C_DEVICE_TYPE_EEPROM) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (i2cAddressMunge(deviceAddress, byteCount)))
				{
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE(byteCount)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (deviceType == I2C_DEVICE_TYPE_EEPROM_AB2) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 1))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 0))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (deviceType == I2C_DEVICE_TYPE_EEPROM_AB3) {
				if (I2C_CYCLE_START) {
					statusVariable = I2C_ERROR_CYCLE_START;
					break;
				}

				if (I2C_CYCLE_WRITE(deviceAddress)) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 2))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 1))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}

				if (I2C_CYCLE_WRITE
				    (I2C_BYTE_ADDRESS(byteCount, 0))) {
					statusVariable = I2C_ERROR_CYCLE_WRITE;
					break;
				}

				if (I2C_CYCLE_ACKIN) {
					statusVariable = I2C_ERROR_CYCLE_ACKIN;
					break;
				}
			}

			if (I2C_CYCLE_WRITE(*(pWriteData + byteCount))) {
				statusVariable = I2C_ERROR_CYCLE_WRITE;
				break;
			}

			if (I2C_CYCLE_ACKIN) {
				statusVariable = I2C_ERROR_CYCLE_ACKIN;
				break;
			}

			if (I2C_CYCLE_STOP) {
				statusVariable = I2C_ERROR_CYCLE_STOP;
				break;
			}

			/*
			 * unlock (enable) interrupts
			 */
			local_irq_restore(flags);

			/*
			 * delay for at least 10ms to allow EEPROM to complete
			 * the write cycle (internal operation)
			 */
			udelay(10 * 1000);
		}

		/*
		 * leave the I2C bus in a known state
		 */
		I2C_KNOWN_STATE;

		/*
		 * update the caller's command packet with status of
		 * the operation
		 */
		return (statusVariable);
	}

	return (statusVariable);
}

static unsigned int
i2cAddressMunge(unsigned int deviceAddress, unsigned int byteOffset)
{
	return (deviceAddress | ((byteOffset & 0x700) >> 7));
}

int gt64260_i2cMacGet(char *ptr, int index)
{
	long indexVariable;
	vpdEthernet_t *pVpdPacket;

	i2cDoOpLowLevel(I2C_READOP, palVpdBBBuffer);

	pVpdPacket = (vpdEthernet_t *) 0;

	for (indexVariable = 0; indexVariable <= index; indexVariable++) {
		pVpdPacket = (vpdEthernet_t *) vpdGetPacket((vpdHeader_t
							     *) ((unsigned char
								  *)
								 &palVpdBBBuffer
								 [0]),
							    VPD_BLOCK_SIZE,
							    VPD_PID_EA,
							    (char *)pVpdPacket);

		if (pVpdPacket == 0)
			return (-1);
		if (pVpdPacket->instanceNum == index)
			break;
	}
	memcpy(ptr, pVpdPacket->address, 6);
	return (0);
}

/*
 * vpdGetPacket - vpd get packet
 * description:
 *	This function's purpose is to retrieve a pointer (into the
 *	passed buffer) to the specified VPD packet.  The optional
 *	previous VPD packet pointer argument allows for the retrieval
 *	of packets which have the same identifier.
 * call:
 *	argument #1 = pointer to VPD header
 *	argument #2 = VPD SROM size in bytes
 *	argument #3 = VPD packet identifier
 *	argument #4 = pointer to previous VPD packet, or null pointer
 * return:
 *	zero		= error: termination, out-of-bounds, or not-found
 *	non-zero	= pointer to the specified packet
 */

static unsigned char *vpdGetPacket(vpdHeader_t * pVpdHeader,
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

static int __init gt64260_i2c_init(void)
{
	return (0);
}

static void __exit gt64260_i2c_exit(void)
{
	return;
}

module_init(gt64260_i2c_init);
module_exit(gt64260_i2c_exit);

MODULE_LICENSE("GPL");
