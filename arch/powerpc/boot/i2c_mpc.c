/*
 * i2c_mpc.c (Bootwrapper version)
 *
 * Author: Ajit Prem
 *
 * Derived from i2c-mpc.c 
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <stdarg.h>
#include <stddef.h>
#include "../../../include/linux/autoconf.h"
#include "types.h"
#include "elf.h"
#include "page.h"
#include "string.h"
#include "stdio.h"
#include "io.h"
#include "ops.h"
#include "i2c_mpc.h"
#ifdef CONFIG_MVME3100
#include "mvme3100.h"
#endif

#undef DEBUG_I2C_MPC
                                                                                
#ifdef DEBUG_I2C_MPC
#define DP(fmt,args...) printk(fmt ,##args)
#else
#define DP(fmt,args...)
#endif

extern void udelay(long);

/* Register offsets */
#define MPC_I2C_ADDR	0x00
#define MPC_I2C_FDR     0x04
#define MPC_I2C_CR      0x08
#define MPC_I2C_SR      0x0c
#define MPC_I2C_DR      0x10
#define MPC_I2C_DFSRR	0x14

#define MPC_I2C_REGION	0x20

/* Control Register Bits */
#define CCR_MEN		0x80	/* Enable */
#define CCR_MIEN	0x40	/* Interrupt Enable */
#define CCR_MSTA	0x20	/* Start Bit */
#define CCR_MTX		0x10	/* Transmit bit */
#define CCR_TXAK	0x08	/* TX Acknowledge */
#define CCR_RSTA	0x04	/* Restart bit */
#define CCR_BCST	0x01	/* Broadcast */

/* Status Register bits */
#define CSR_MCF		0x80	/* Transfer Pending */
#define CSR_MAAS	0x40	/* Master/Slave */
#define CSR_MBB		0x20	/* Bus Busy */
#define CSR_MAL		0x10	/* Arbitration lost */
#define CSR_BCSTM	0x08	/* Broadcast */
#define CSR_SRW		0x04	/* Slave read/write */
#define CSR_MIF		0x02	/* Interrupt */
#define CSR_RXAK	0x01	/* RX Acknowledged */

/* Data Register Mask */
#define MPC_I2CDR_DATA		0xFF

/* Default DFSRR value */
#define MPC_I2C_DEF_DFSRR 	0x10

/* DFSRR Mask */
#define MPC_I2C_DFSRR_MASK	0x3F

/* I2C address when addressed as a slave */
#define I2C_MPC_ADDR        0xfe

/* Masks of which bits get set when writing
 * to the register (ADR or FDR) */
#define MPC_I2CADR_MASK     0xFE
#define MPC_I2CFDR_MASK     0x3F

/* Default Clock Speed */
#define MPC_I2C_CLKSPD 0x3f

/* Timeout for waiting for the bus */
#define MPC_TIMEOUT 50

/* Mask for writing the entire register in I2C */
#define MPC_I2C_FULLREG 0xff

#define I2C_READ	1
#define I2C_WRITE	0

/* Init value for the CR */
#define MPC_I2CCR_INIT  (CCR_MEN | \
                CCR_MSTA| \
                CCR_MTX | \
                CCR_RSTA)

static u8 *mpc_i2c_base;

static int wait_for_txcomplete(void);
static int wait_for_bus(void);
static void init_i2c_registers(void);

/* Write val to addr, changing only the bits specified in mask */
static void mpc_i2c_write8(volatile u8 * addr, u8 val, u8 mask)
{
	u8 tmp;

	tmp = in_8(addr);

	out_8(addr, (tmp & ~mask) | (val & mask));
}

static u8 mpc_i2c_read8(volatile u8 * addr)
{
	u8 val = in_8(addr);
	return val;
}

/* Issues an i2c START command */
static inline void mpc_i2c_start(void)
{
//	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MSTA | CCR_MTX , 
//				CCR_MSTA | CCR_MTX);
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MSTA, CCR_MSTA);
}

/* Issues an i2c STOP command */
static inline void mpc_i2c_stop(void)
{
//	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MSTA | CCR_MTX);
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MSTA);
}

static void init_i2c_registers(void)
{
	/* Disable interrupts */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MIEN);

	/* Set the slave address */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_ADDR, I2C_MPC_ADDR, MPC_I2CADR_MASK);

	/* Set clock */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_FDR, 
			MPC_I2C_CLKSPD, MPC_I2CFDR_MASK);

	/* Set the default digital filter sampling rate. */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DFSRR, MPC_I2C_DEF_DFSRR,
		       MPC_I2C_DFSRR_MASK);

	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MSTA);
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MTX, CCR_MTX);

	/* Clear interrupt bit */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_SR, 0, CSR_MIF);
}

static void mpc_i2c_reset(void)
{
	 /* Put the controller into reset */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MEN);
	/* Make it the master */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MSTA, CCR_MSTA);
	/* Reenable it */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MEN, CCR_MEN);
	/* Dummy read */
	mpc_i2c_read8(mpc_i2c_base + MPC_I2C_DR);
	/* Put us back to a wait mode (STOP) */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MSTA);
	init_i2c_registers();
}

static int wait_for_txcomplete(void)
{
	int ret = 0;
	unsigned long timeout = MPC_TIMEOUT;

	/* Keep going until the interrupt field is set */
	while ((!(mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_MIF)) && 
		(timeout > 0)) {
		udelay(100);
		timeout--;
	}
	if (timeout <= 0) {
		ret = -1;
	}
	/* If the transmit completed, clear the interrupt bit */
	if ((mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_MIF)) {
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_SR, 0, CSR_MIF);
//		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_SR, 0, MPC_I2C_FULLREG);
		ret = 0;
	}
	if (ret < 0) {
		mpc_i2c_stop();
	}

	return ret;

}

static int wait_for_bus(void)
{
	int timeout = MPC_TIMEOUT;

	udelay(100);
	while ((mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_MBB) &&
	       timeout--) {
		/* If we're here, that means something is wrong.
		 * We only support being the master here, and all
		 * transactions should be done.  Reset, and hope
		 * that fixes it */
		mpc_i2c_reset();
		udelay(2000);
	}

	return (timeout <= 0);
}

static int mpc_do_address(u8 addr, int rw)
{
	u8 i2ccrval = CCR_MTX;
	int err = 0;

	/* Modify the MTX bit and the R/~W bit based on whether this is
	 * a read or a write */
	if (rw == I2C_READ)
		addr |= 1;


	/* Send the address and R/~W indicator */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, i2ccrval, CCR_MTX);

	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, addr, MPC_I2C_FULLREG);

	/* Wait for the address to finish being sent */
	err = wait_for_txcomplete();

	if (err)
		return err;

	/* Return if we didn't get the ack for the address byte */
	if ((mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_RXAK)) {
		mpc_i2c_stop();

		return -1;
	}

	return err;
}

int i2c_mpc_read(u8 dev_addr,	/* i2c address of target device */
		u32 offset, 	/* offset within device */
		u32 offset_size,
		u8 *buf,	/* pointer to data byte */
		u32 len)	/* bytes to read */
{
	int i;
	int err = 0;
	int mien_status = 0;

	if (mpc_i2c_read8(mpc_i2c_base + MPC_I2C_CR) & CCR_MIEN) {
		mien_status = 1;
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MIEN);
	} 

	err = wait_for_bus();

	/* Exit if the BUS is busy */
	if (err) {
		DP("mpc-i2c-read: bus was busy 1\n");
		goto read_err_ret;
	}

	/* Clear interrupts */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_SR, 0, CSR_MIF);

	mpc_i2c_start();

	err = mpc_do_address(dev_addr, I2C_WRITE);
	if (err < 0) {
		DP("early-mpc-i2c: Error on do_address write\n");
		goto read_err_ret;
	}

	if (offset_size > 1) {
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, offset >> 8, 
			MPC_I2C_FULLREG);
		err = wait_for_txcomplete();
		if (err < 0) {
			DP("early-mpc-i2c: Error on write of offset(h)\n");
			goto read_err_ret;
		}
	}
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, offset, MPC_I2C_FULLREG);
	err = wait_for_txcomplete();
	if (err < 0) {
		DP("early-mpc-i2c: Error on write of offset(l)\n");
		goto read_err_ret;
	}

	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_RSTA, CCR_RSTA);
	udelay(20);

	err = mpc_do_address(dev_addr, I2C_READ);
	if (err < 0) {
		DP("early-mpc-i2c: Error on do_address read\n");
		goto read_err_ret;
	}

	/* Change to read mode */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MTX);

	/* If there is only one byte, deal with TXAK now */
	if (len == 1)
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_TXAK, CCR_TXAK);
	else
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_TXAK);

	udelay(5);

	/* Do a dummy read, to initiate the first read */
	mpc_i2c_read8(mpc_i2c_base + MPC_I2C_DR);

	for (i = 0; i < len; ++i) {
		/* Wait for the previous read to finish */
		err = wait_for_txcomplete();

		if (err)
			goto read_err_ret;

		/* If this is the 2nd to last byte, deal with the TXAK signal */
		if (i == len - 2) {
			mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_TXAK, CCR_TXAK);
		}

		/* If this is the last byte, send STOP */
		if (i == len - 1)
			mpc_i2c_stop();

		buf[i] = mpc_i2c_read8(mpc_i2c_base + MPC_I2C_DR);
	}

	udelay(20);
read_err_ret:
	mpc_i2c_stop();
	if (mien_status == 1)
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MIEN, CCR_MIEN);
	return err;
}

int i2c_mpc_write(u8 dev_addr,	/* i2c address of target device */
		u32 offset, 	/* offset within device */	
		u32 offset_size,
		u8 *buf,	/* pointer to data byte */
		u32 len		/* number of bytes */
    )
{
	int i;
	int err = 0;
	int mien_status = 0;

	if (mpc_i2c_read8(mpc_i2c_base + MPC_I2C_CR) & CCR_MIEN) {
		mien_status = 1;
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, 0, CCR_MIEN);
	} 
	err = wait_for_bus();

	/* Exit if the BUS is busy */
	if (err) {
		DP("mpc-i2c-write: bus was busy\n");
		goto write_err_ret;
	}

	/* Clear interrupts */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_SR, 0, CSR_MIF);

	mpc_i2c_start();

	err = mpc_do_address(dev_addr, I2C_WRITE);

	if (offset_size > 1) {
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, offset >> 8, 
			MPC_I2C_FULLREG);
		err = wait_for_txcomplete();
		if (err < 0) {
			DP("early-mpc-i2c: Error on write of offset(h)\n");
			goto write_err_ret;
		}
	}
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, offset, MPC_I2C_FULLREG);
	err = wait_for_txcomplete();
	if (err < 0) {
		DP("early-mpc-i2c: Error on write of offset(l)\n");
		goto write_err_ret;
	}

	/* Return if we didn't get the ack for the register offset byte */
	if ((mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_RXAK)) {
		DP("early-mpc-i2c: No RXAK on write of register offset\n");
		goto write_err_ret;
	}

	for (i = 0; i < len; ++i) {
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_DR, buf[i], 
					MPC_I2C_FULLREG);
		/* Wait for the transaction to finish */
		err = wait_for_txcomplete();
		if (err)
			goto write_err_ret;
		/* Return error if we didn't get the ack */
		if ((mpc_i2c_read8(mpc_i2c_base + MPC_I2C_SR) & CSR_RXAK)) {
			DP("NO ACK!\n");
			err = -1;
		}

	}

write_err_ret:
	mpc_i2c_stop();
	if (mien_status == 1)
		mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MIEN, CCR_MIEN);
	return err;
}

int i2c_mpc_open(void)
{
	u32 v;
	void *devp = NULL;

#ifdef CONFIG_MVME3100
	devp = finddevice("/soc8540/i2c");
#endif
#ifdef CONFIG_MVME7100
	devp = finddevice("/soc8641/i2c@3000");
#endif
	if (devp == NULL)
		return -1;
	if (getprop(devp, "virtual-reg", &v, sizeof(v)) != sizeof(v))
		return -1;

	mpc_i2c_base = (u8 *)v;
#if 0
	mpc_i2c_base = ioremap(BOARD_CCSRBAR + 0x3000, MPC_I2C_REGION);
	if (!mpc_i2c_base) {
		printk(KERN_ERR "mpc-i2c: failed to map controller\n");
		return 0;
	}
#endif

	init_i2c_registers();

	/* Enable the I2C Interface */
	mpc_i2c_write8(mpc_i2c_base + MPC_I2C_CR, CCR_MEN, CCR_MEN);

	return 0;
}

void i2c_mpc_close(void)
{
	mpc_i2c_base = NULL;
}
//EXPORT_SYMBOL_GPL(i2c_mpc_read);
//EXPORT_SYMBOL_GPL(i2c_mpc_write);
