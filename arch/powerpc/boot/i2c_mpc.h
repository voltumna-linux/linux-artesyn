/*
 * i2c_mpc.h
 *
 * Author: Ajit Prem
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __BOOT_I2C_MPC_H
#define __BOOT_I2C_MPC_H

extern int i2c_mpc_open(void);
extern void i2c_mpc_close(void);

extern int i2c_mpc_read(u8 dev_addr,	/* i2c addr of target device */
		u32 offset, 		/* offset within device */
		u32 offset_size,	/* 1=>8bit, 2=>16bit */
		u8 * pbuf,		/* pointer to data byte */
		u32 len);		/* number of bytes to read */

extern int i2c_mpc_write(u8 dev_addr,	/* i2c addr of target device */
		u32 offset,		/* offset within device */ 
		u32 offset_size,	/* 1=>8bit, 2=>16bit */
		u8 * pbuf,		/* pointer to data byte */
		u32 len);		/* number of bytes to write */

#endif				/* __BOOT_I2C_MPC_H */
