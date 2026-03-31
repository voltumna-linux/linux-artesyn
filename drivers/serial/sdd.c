/*
 * sdd.c 
 *    Serial driver for the Zilog Z85230 with a Z8536
 *
 *    This driver can be used on MVME5100, MVME5500, and 
 *    MVME6100 boards that use an IPMC761 or IPMC712.
 *    The driver can also be used on MCP750 and MCP820 boards.
 *
 *    The sdd driver has been derived from the macserial driver
 *    for the boards mentioned above by:
 *    Ajit Prem (Ajit.Prem@motorola.com)
 *
 * Derived from macserial.c: Serial port driver for Power Macintosh
 * Z8530 serial driver.
 *
 * Derived from drivers/sbus/char/sunserial.c by Paul Mackerras.
 *
 * Copyright (C) 1996 Paul Mackerras (Paul.Mackerras@cs.anu.edu.au)
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pci.h>

#include <asm/sections.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#include "sdd.h"

#define MESQSERIAL_VERSION	"2.0"

#define  CDTRCTS

/*
 * It would be nice to dynamically allocate everything that
 * depends on NUM_SERIAL, so we could support any number of
 * Z8530s, but for now...
 */
#define NUM_SERIAL	1	/* Max number of ZS chips supported  in Mesq */
#define NUM_CHANNELS	(NUM_SERIAL * 2)	/* 2 channels per chip */

/* On PowerMacs, the hardware takes care of the SCC recovery time,
   but we need the eieio to make sure that the accesses occur
   in the order we want. */
#define RECOVERY_DELAY	eieio()

static struct mesq_zschannel zs_channels[NUM_CHANNELS];

static struct mesq_serial zs_soft[NUM_CHANNELS];
static int zs_channels_found;
static struct mesq_serial *zs_chain;	/* list of all channels */

static spinlock_t z8536_lock;

#define ZS_CLOCK         10000000	/* Z8530 RTxC input clock rate */

/*
 * Get the value from the machine specific isa_io_base, see xxx_setup.c
 */
#define Z85C230BA       (_IO_BASE+0x840)	/* Z85C230 (SCC) base address */

#if defined(CONFIG_MVME6100) || defined(CONFIG_MVME5500)
#define Z85230_IRQ	105
#else
#define Z85230_IRQ	9
#endif

static struct tty_driver *serial_driver;

/* serial subtype definitions */
#define SERIAL_TYPE_NORMAL	1

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

/*
 * Debugging.
 */
#undef SERIAL_DEBUG_INIT
#undef SERIAL_DEBUG_INTR
#undef SERIAL_DEBUG_OPEN
#undef SERIAL_DEBUG_FLOW
#undef SERIAL_DEBUG_WRITE
#undef SERIAL_DEBUG_POWER
#undef SERIAL_DEBUG_THROTTLE
#undef SERIAL_DEBUG_STOP
#undef SERIAL_DEBUG_BAUDS

#define RS_STROBE_TIME 10
#define RS_ISR_PASS_LIMIT 256

#define _INLINE_ inline

#ifdef SERIAL_DEBUG_OPEN
#define OPNDBG(fmt, arg...)     printk(KERN_INFO fmt , ## arg)
#else
#define OPNDBG(fmt, arg...)     do { } while (0)
#endif

#ifdef SERIAL_DEBUG_POWER
#define PWRDBG(fmt, arg...)     printk(KERN_INFO fmt , ## arg)
#else
#define PWRDBG(fmt, arg...)     do { } while (0)
#endif

#ifdef SERIAL_DEBUG_BAUDS
#define BAUDBG(fmt, arg...)     printk(fmt , ## arg)
#else
#define BAUDBG(fmt, arg...)     do { } while (0)
#endif

static int probe_sccs(void);
static void change_speed(struct mesq_serial *info, struct ktermios *old);
static void rs_wait_until_sent(struct tty_struct *tty, int timeout);
static int setup_scc(struct mesq_serial *info);

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

/*
 * tmp_buf is used as a temporary buffer by serial_write.  We need to
 * lock it in case the copy_from_user blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *tmp_buf;
static DECLARE_MUTEX(tmp_buf_sem);

static inline int 
serial_paranoia_check(struct mesq_serial *info,
                      char *name, const char *routine)
{
#ifdef SERIAL_PARANOIA_CHECK
        static const char badmagic[] = KERN_WARNING
                "Warning: bad magic number for serial struct %s in %s\n";
        static const char badinfo[] = KERN_WARNING
                "Warning: null mac_serial for %s in %s\n";
                                                                                
        if (!info) {
                printk(badinfo, name, routine);
                return 1;
        }
        if (info->magic != SERIAL_MAGIC) {
                printk(badmagic, name, routine);
                return 1;
        }
#endif
        return 0;
}

/* 
 * Reading and writing Z8530 registers.
 */
static inline unsigned char read_zsreg(struct mesq_zschannel *channel,
				       unsigned char reg)
{
	unsigned char retval;
	unsigned long flags;

	/*
	 * We have to make this atomic.
	 */
	local_irq_save(flags);
	if (reg != 0) {
		*channel->control = reg;
		RECOVERY_DELAY;
	}
	retval = *channel->control;
	RECOVERY_DELAY;
	local_irq_restore(flags);
	return retval;
}

static inline void write_zsreg(struct mesq_zschannel *channel,
			       unsigned char reg, unsigned char value)
{
	unsigned long flags;

	local_irq_save(flags);
	if (reg != 0) {
		*channel->control = reg;
		RECOVERY_DELAY;
	}
	*channel->control = value;
	RECOVERY_DELAY;
	local_irq_restore(flags);
	return;
}

static inline unsigned char read_zsdata(struct mesq_zschannel *channel)
{
	unsigned char retval;

	retval = *channel->data;
	RECOVERY_DELAY;
	return retval;
}

static inline void write_zsdata(struct mesq_zschannel *channel,
				unsigned char value)
{
	*channel->data = value;
	RECOVERY_DELAY;
	return;
}

/* 
 * Reading and writing Z8536 registers.
 */
static inline unsigned char read_z8536reg(volatile unsigned char *control_reg,
					  unsigned char reg_offset)
{
	unsigned char retval;
	unsigned long flags;

	/*
	 * We have to make this atomic.
	 */
	local_irq_save(flags);
	if (reg_offset != 0) {
		*control_reg = reg_offset;
		RECOVERY_DELAY;
	}
	retval = *control_reg;
	RECOVERY_DELAY;
	local_irq_restore(flags);
	return retval;
}

static inline void write_z8536reg(volatile unsigned char *control_reg,
				  unsigned char reg_offset, unsigned char value)
{
	unsigned long flags;

	spin_lock_irqsave(&z8536_lock, flags);
	if (reg_offset != 0) {
		*control_reg = reg_offset;
		RECOVERY_DELAY;
	}
	*control_reg = value;
	RECOVERY_DELAY;
	spin_unlock_irqrestore(&z8536_lock, flags);
	return;
}

static inline void z8536_dtr_assert(struct mesq_serial *ss)
{
	unsigned long system_base = Z85C230BA;
	volatile unsigned char *control_reg;
	int data_port;
	unsigned char reg;

	if (ss->zs_chan_a == ss->zs_channel)
		data_port = Z8536_RO_PAD;
	else
		data_port = Z8536_RO_PBD;
	control_reg = (volatile unsigned char *)system_base + CIOOFFS;
	reg = read_z8536reg(control_reg, data_port);
	write_z8536reg(control_reg, data_port, reg | Z8536_IOP_DTR);
}

static inline void dump_zsregs(struct mesq_zschannel *channel,
			       unsigned char *regs)
{
	printk("rr0 0x%x\n", read_zsreg(channel, 0));
	printk("rr1 0x%x\n", read_zsreg(channel, 1));
	printk("rr2 0x%x\n", read_zsreg(channel, 2));
	printk("rr3 0x%x\n", read_zsreg(channel, 3));
	printk("rr4 0x%x\n", read_zsreg(channel, 4));
	printk("rr5 0x%x\n", read_zsreg(channel, 5));
	printk("rr6 0x%x\n", read_zsreg(channel, 6));
	printk("rr7 0x%x\n", read_zsreg(channel, 7));
	printk("rr8 0x%x\n", read_zsreg(channel, 8));
	printk("rr9 0x%x\n", read_zsreg(channel, 9));
	printk("rr10 0x%x\n", read_zsreg(channel, 10));
	printk("rr11 0x%x\n", read_zsreg(channel, 11));
	printk("rr12 0x%x\n", read_zsreg(channel, 12));
	printk("rr13 0x%x\n", read_zsreg(channel, 13));
	printk("rr14 0x%x\n", read_zsreg(channel, 14));
	printk("rr15 0x%x\n", read_zsreg(channel, 15));
	write_zsreg(channel, R15, regs[R15] | 1);
	write_zsreg(channel, R7, 0x40);
	printk("wr3 0x%x\n", read_zsreg(channel, 9));
	printk("wr4 0x%x\n", read_zsreg(channel, 4));
	printk("wr5 0x%x\n", read_zsreg(channel, 5));
	printk("wr7p 0x%x\n", read_zsreg(channel, 14));
	printk("wr10 0x%x\n", read_zsreg(channel, 11));
	write_zsreg(channel, R7, 0x00);
	write_zsreg(channel, R15, regs[R15]);
}

static inline void load_zsregs(struct mesq_zschannel *channel,
			       unsigned char *regs)
{
	ZS_CLEARERR(channel);
	ZS_CLEARFIFO(channel);
	/* Load 'em up */
	write_zsreg(channel, R4, regs[R4]);
	write_zsreg(channel, R10, regs[R10]);
	write_zsreg(channel, R3, regs[R3] & ~RxENABLE);
	write_zsreg(channel, R5, regs[R5] & ~TxENAB);
	write_zsreg(channel, R1, regs[R1]);
	write_zsreg(channel, R9, regs[R9]);
	write_zsreg(channel, R11, regs[R11]);
	write_zsreg(channel, R12, regs[R12]);
	write_zsreg(channel, R13, regs[R13]);
	write_zsreg(channel, R14, regs[R14]);
	write_zsreg(channel, R15, regs[R15]);
	write_zsreg(channel, R3, regs[R3]);
	write_zsreg(channel, R5, regs[R5]);
	return;
}

/* Sets or clears DTR/RTS on the requested line */
static void zs_rtsdtr(struct mesq_serial *ss, int set)
{
	z8536_dtr_assert(ss);

	if (set) {
		ss->curregs[5] |= RTS;
	} else {
		ss->curregs[5] &= ~RTS;
	}

	write_zsreg(ss->zs_channel, 5, ss->curregs[5]);
	return;
}

/* Utility routines for the Zilog */
static inline int get_zsbaud(struct mesq_serial *ss)
{
	struct mesq_zschannel *channel = ss->zs_channel;
	int brg;

	if ((ss->curregs[R11] & TCBR) == 0) {
		/* higher rates don't use the baud rate generator */
		return (ss->curregs[R4] & X32CLK) ? ZS_CLOCK / 32 : ZS_CLOCK /
		    16;
	}
	/* The baud rate is split up between two 8-bit registers in
	 * what is termed 'BRG time constant' format in my docs for
	 * the chip, it is a function of the clk rate the chip is
	 * receiving which happens to be constant.
	 */
	brg = (read_zsreg(channel, 13) << 8);
	brg |= read_zsreg(channel, 12);
	return BRG_TO_BPS(brg, (ZS_CLOCK / (ss->clk_divisor)));
}

/* On receive, this clears errors and the receiver interrupts */
static inline void rs_recv_clear(struct mesq_zschannel *zsc)
{
	write_zsreg(zsc, 0, ERR_RES);
	write_zsreg(zsc, 0, RES_H_IUS);	/* XXX this is unnecessary */
}

/*
 * ----------------------------------------------------------------------
 *
 * Here starts the interrupt handling routines.  All of the following
 * subroutines are declared as inline and are folded into
 * rs_interrupt().  They were separated out for readability's sake.
 *
 * 				- Ted Ts'o (tytso@mit.edu), 7-Mar-93
 * -----------------------------------------------------------------------
 */

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */
/*  we will "borrow" the MAC bottom half intr position, for now */
static _INLINE_ void rs_sched_event(struct mesq_serial *info, int event)
{
	info->event |= 1 << event;
	schedule_work(&info->tqueue);
}

/* Work out the flag value for a z8530 status value. */
static _INLINE_ int stat_to_flag(int stat)
{
	int flag;

	if (stat & Rx_OVR) {
		flag = TTY_OVERRUN;
	} else if (stat & FRM_ERR) {
		flag = TTY_FRAME;
	} else if (stat & PAR_ERR) {
		flag = TTY_PARITY;
	} else
		flag = TTY_NORMAL;
	return flag;
}

static _INLINE_ void receive_chars(struct mesq_serial *info)
{
	struct tty_struct *tty = info->tty;
	unsigned char ch, stat, flag;

	while ((read_zsreg(info->zs_channel, 0) & Rx_CH_AV) != 0) {
		stat = read_zsreg(info->zs_channel, R1);
		ch = read_zsdata(info->zs_channel);

		if (!tty)
			continue;

		flag = stat_to_flag(stat);
		/* reset the error indication */
		if (flag)
			write_zsreg(info->zs_channel, 0, ERR_RES);
		tty_insert_flip_char(tty, ch, flag);

	}
	if (tty)
		tty_flip_buffer_push(tty);
	return;
}

static void transmit_chars(struct mesq_serial *info)
{
	if ((read_zsreg(info->zs_channel, 0) & Tx_BUF_EMP) == 0)
		return;

	info->tx_active = 0;

	if (info->x_char) {
		/* Send next char */
		write_zsdata(info->zs_channel, info->x_char);
		info->x_char = 0;
		info->tx_active = 1;
		return;
	}

	if ((info->xmit_cnt <= 0) || info->tty->stopped || info->tx_stopped) {
		write_zsreg(info->zs_channel, 0, RES_Tx_P);
		return;
	}

	/* Send char */
#ifdef SERIAL_DEBUG_WRITE
	printk("transmit_char: %c\n", info->xmit_buf[info->xmit_tail]);
#endif
	write_zsdata(info->zs_channel, info->xmit_buf[info->xmit_tail++]);
	info->xmit_tail = info->xmit_tail & (SERIAL_XMIT_SIZE - 1);
	info->xmit_cnt--;
	info->tx_active = 1;

	if (info->xmit_cnt < WAKEUP_CHARS)
		rs_sched_event(info, RS_EVENT_WRITE_WAKEUP);
	return;
}

static _INLINE_ void status_handle(struct mesq_serial *info)
{
	unsigned char status;

	/* Get status from Read Register 0 */
	status = read_zsreg(info->zs_channel, 0);

	/* Check for DCD transitions */
	if (((status ^ info->read_reg_zero) & DCD) != 0
	    && info->tty && !C_CLOCAL(info->tty)) {
		if (status & DCD) {
			wake_up_interruptible(&info->open_wait);
		} else if (!(info->flags & ZILOG_CALLOUT_ACTIVE)) {
			if (info->tty)
				tty_hangup(info->tty);
		}
	}

	/* Check for CTS transitions */
	if (info->tty && C_CRTSCTS(info->tty)) {
		/*
		 * For some reason, on the Power Macintosh,
		 * it seems that the CTS bit is 1 when CTS is
		 * *negated* and 0 when it is asserted.
		 * The DCD bit doesn't seem to be inverted
		 * like this.
		 */
		if ((status & CTS) == 0) {
			if (info->tx_stopped) {
#ifdef SERIAL_DEBUG_FLOW
				printk("CTS up\n");
#endif
				info->tx_stopped = 0;
				if (!info->tx_active)
					transmit_chars(info);
			}
		} else {
#ifdef SERIAL_DEBUG_FLOW
			printk("CTS down\n");
#endif
			info->tx_stopped = 1;
		}
	}

	/* Clear status condition... */
	write_zsreg(info->zs_channel, 0, RES_EXT_INT);
	info->read_reg_zero = status;
}

/*
 * This is the serial driver's generic interrupt routine
 */

#define CHAN_IRQMASK (CHBRxIP | CHBTxIP | CHBEXT)

static irqreturn_t rs_interrupt(int irq, void *dev_id)
{
	struct mesq_serial *info = (struct mesq_serial *)dev_id;
	unsigned char zs_intreg;
	int shift;
	unsigned long flags;
	int handled = 0;

	/* NOTE: The read register 3, which holds the irq status,
	 *       does so for both channels on each chip.  Although
	 *       the status value itself must be read from the A
	 *       channel and is only valid when read from channel A.
	 *       Yes... broken hardware...
	 */

	if (info->zs_chan_a == info->zs_channel)
		shift = 3;	/* Channel A */
	else
		shift = 0;	/* Channel B */

	if (!(info->flags & ZILOG_INITIALIZED)) {
		/* There may be a race condition between clearing the
		 * ZILOG_INITIALIZED flag and getting an interrupt.
		 * We must clear a status change.
		 */
		zs_intreg = read_zsreg(info->zs_chan_a, 3) >> shift;

		if (zs_intreg & CHBEXT) 
			write_zsreg(info->zs_channel, 0, RES_EXT_INT);

		if (zs_intreg & CHBRxIP)
			receive_chars(info);

#ifdef SERIAL_DEBUG_INTR
		printk(KERN_INFO
		    "sdd: rs_interrupt: irq %d, port not initialized, zs_intreg 0x%x\n",
		     irq, (int)zs_intreg);
#endif
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&info->lock, flags);
	for (;;) {
		zs_intreg = read_zsreg(info->zs_chan_a, 3) >> shift;
#ifdef SERIAL_DEBUG_INTR
		printk(KERN_INFO "rs_interrupt: irq %d, zs_intreg 0x%x\n",
		       irq, (int)zs_intreg);
#endif

		if ((zs_intreg & CHAN_IRQMASK) == 0)
			break;
		handled = 1;

		if (zs_intreg & CHBRxIP) 
			receive_chars(info);
		if (zs_intreg & CHBTxIP)
			transmit_chars(info);
		if (zs_intreg & CHBEXT)
			status_handle(info);
	}
	spin_unlock_irqrestore(&info->lock, flags);
	return IRQ_RETVAL(handled);
}

/*
 * ------------------------------------------------------------
 * rs_stop() and rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * ------------------------------------------------------------
 */
static void rs_stop(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;

#ifdef SERIAL_DEBUG_STOP
	printk("rs_stop 0x%x....\n", tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->name, "rs_stop"))
		return;

#if 0
	spin_lock_irqsave(&info->lock, flags);
	if (info->curregs[5] & TxENAB) {
		info->curregs[5] &= ~TxENAB;
		info->pendregs[5] &= ~TxENAB;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
	}
	spin_unlock_irqrestore(&info->lock, flags);
#endif
}

static void rs_start(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

#ifdef SERIAL_DEBUG_STOP
	printk("rs_start 0x%x....\n", tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->name, "rs_start"))
		return;

	spin_lock_irqsave(&info->lock, flags);
#if 0
	if (info->xmit_cnt && info->xmit_buf && !(info->curregs[5] & TxENAB)) {
		info->curregs[5] |= TxENAB;
		info->pendregs[5] = info->curregs[5];
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
	}
#else
	if (info->xmit_cnt && info->xmit_buf && !info->tx_active) {
		transmit_chars(info);
	}
#endif
	spin_unlock_irqrestore(&info->lock, flags);
}

/*
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * rs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using rs_sched_event(), and they get done here.
 */

//static void do_softint(void *private_)
static void do_softint(struct work_struct *work)
{
//	struct mesq_serial *info = (struct mesq_serial *)private_;
	struct mesq_serial *info = container_of(work, struct mesq_serial, tqueue);
	struct tty_struct *tty;

	tty = info->tty;
	if (!tty)
		return;

	if (test_and_clear_bit(RS_EVENT_WRITE_WAKEUP, &info->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup) (tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

static int startup(struct mesq_serial *info, int can_sleep)
{
	int delay = 0;

	OPNDBG("startup() (ttyS%d, irq %d)\n", info->line, info->irq);

	if (info->flags & ZILOG_INITIALIZED) {
		OPNDBG(" -> already inited\n");
		return 0;
	}

	if (!info->xmit_buf) {
		info->xmit_buf = (unsigned char *)get_zeroed_page(GFP_KERNEL);
		if (!info->xmit_buf)
			return -ENOMEM;
	}

	OPNDBG("starting up ttyS%d (irq %d)...\n", info->line, info->irq);

	setup_scc(info);

	if (delay) {
		if (can_sleep) {
			/* we need to wait a bit before using the port */
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(delay * HZ / 1000);
		} else
			mdelay(delay);
	}

	OPNDBG("enabling IRQ on ttyS%d (irq %d)...\n", info->line, info->irq);

	info->flags |= ZILOG_INITIALIZED;

	return 0;
}

static int setup_scc(struct mesq_serial *info)
{
	unsigned long flags;

	OPNDBG("setting up ttyS%d SCC...\n", info->line);

	spin_lock_irqsave(&info->lock, flags);

	/*
	 * Reset the chip.
	 */
	write_zsreg(info->zs_channel, 9,
		    (info->zs_channel == info->zs_chan_a ? CHRA : CHRB));
	udelay(10);
	write_zsreg(info->zs_channel, 9, 0);

	/*
	 * Clear the receive FIFO.
	 */
	ZS_CLEARFIFO(info->zs_channel);
	info->xmit_fifo_size = 1;

	/*
	 * Clear the interrupt registers.
	 */
	write_zsreg(info->zs_channel, 0, ERR_RES);
	write_zsreg(info->zs_channel, 0, RES_H_IUS);

	/*
	 * Turn on RTS and DTR.
	 */
	zs_rtsdtr(info, 1);

	/*
	 * Finally, enable sequencing and interrupts
	 */
	/* Interrupt on ext/status changes, all received chars, 
	   transmit ready */
	info->curregs[1] =
		(info->curregs[1] & ~0x18) | (EXT_INT_ENAB | INT_ALL_Rx | TxINT_ENAB);
	info->pendregs[1] = info->curregs[1];
	info->curregs[3] |= (RxENABLE | Rx8);
	info->pendregs[3] = info->curregs[3];
	info->curregs[5] |= (TxENAB | Tx8);
	info->pendregs[5] = info->curregs[5];
	info->curregs[9] |= (NV | MIE);
	info->pendregs[9] = info->curregs[9];
	write_zsreg(info->zs_channel, 3, info->curregs[3]);
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	write_zsreg(info->zs_channel, 9, info->curregs[9]);

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	spin_unlock_irqrestore(&info->lock, flags);

	/*
	 * Set the speed of the serial port
	 */
	change_speed(info, 0);

	/* Save the current value of RR0 */
	info->read_reg_zero = read_zsreg(info->zs_channel, 0);

	return 0;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void shutdown(struct mesq_serial *info)
{
	unsigned long flags;

	OPNDBG("Shutting down serial port %d (irq %d)....\n", info->line,
	       info->irq);

	if (!(info->flags & ZILOG_INITIALIZED)) {
		OPNDBG("(already shutdown)\n");
		return;
	}

	local_irq_save(flags);
	info->pendregs[1] = info->curregs[1] = 0;
	write_zsreg(info->zs_channel, 1, 0);	/* no interrupts */

	info->curregs[3] &= ~RxENABLE;
	info->pendregs[3] = info->curregs[3];
	write_zsreg(info->zs_channel, 3, info->curregs[3]);

	info->curregs[5] &= ~TxENAB;
	if (!info->tty || C_HUPCL(info->tty))
		info->curregs[5] &= ~DTR;
	info->pendregs[5] = info->curregs[5];
	write_zsreg(info->zs_channel, 5, info->curregs[5]);

	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);

	if (info->xmit_buf) {
		free_page((unsigned long)info->xmit_buf);
		info->xmit_buf = 0;
	}

	memset(info->curregs, 0, sizeof(info->curregs));
	memset(info->curregs, 0, sizeof(info->pendregs));

	info->flags &= ~ZILOG_INITIALIZED;
	local_irq_restore(flags);
}

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
static void change_speed(struct mesq_serial *info, struct ktermios *old_termios)
{
	unsigned cflag;
	int bits;
	int brg, baud;
	unsigned long flags;

	if (!info->tty || !info->tty->termios)
		return;

	cflag = info->tty->termios->c_cflag;
	baud = tty_get_baud_rate(info->tty);
	if (baud == 0) {
		if (old_termios) {
			info->tty->termios->c_cflag &= ~CBAUD;
			info->tty->termios->c_cflag |=
			    (old_termios->c_cflag & CBAUD);
			cflag = info->tty->termios->c_cflag;
			baud = tty_get_baud_rate(info->tty);
		} else
			baud = info->zs_baud;
	}
	if (baud > 460800)
		baud = 460800;
	else if (baud == 0)
		baud = 38400;

	spin_lock_irqsave(&info->lock, flags);
	info->zs_baud = baud;
	info->clk_divisor = 16;

	BAUDBG("set speed to %d bds, ", baud);

	switch (baud) {
	case 230400:
		info->curregs[4] = X1CLK;
		info->curregs[11] = TRxCBR | TCBR | RCBR;
		brg = BPS_TO_BRG(baud, ZS_CLOCK);
		info->curregs[12] = (brg & 255);
		info->curregs[13] = ((brg >> 8) & 255);
		info->curregs[14] = BRENABL | BRSRC | SSBR;
		break;
	case 460800:
		info->curregs[4] = X1CLK;
		info->curregs[11] = TRxCBR | TCBR | RCBR;
		brg = BPS_TO_BRG(baud, ZS_CLOCK);
		info->curregs[12] = (brg & 255);
		info->curregs[13] = ((brg >> 8) & 255);
		info->curregs[14] = BRENABL | BRSRC | SSBR;
		break;
	default:
		info->curregs[4] = X16CLK;
		info->curregs[11] = TRxCBR | TCBR | RCBR;
		brg = BPS_TO_BRG(baud, ZS_CLOCK / info->clk_divisor);
		info->curregs[12] = (brg & 255);
		info->curregs[13] = ((brg >> 8) & 255);
		info->curregs[14] = BRENABL | BRSRC | SSBR;
	}

	/* byte size and parity */
	info->curregs[3] &= ~RxNBITS_MASK;
	info->curregs[5] &= ~TxNBITS_MASK;
	switch (cflag & CSIZE) {
	case CS5:
		info->curregs[3] |= Rx5;
		info->curregs[5] |= Tx5;
		BAUDBG("5 bits, ");
		bits = 7;
		break;
	case CS6:
		info->curregs[3] |= Rx6;
		info->curregs[5] |= Tx6;
		BAUDBG("6 bits, ");
		bits = 8;
		break;
	case CS7:
		info->curregs[3] |= Rx7;
		info->curregs[5] |= Tx7;
		BAUDBG("7 bits, ");
		bits = 9;
		break;
	case CS8:
	default:		/* defaults to 8 bits */
		info->curregs[3] |= Rx8;
		info->curregs[5] |= Tx8;
		BAUDBG("8 bits, ");
		bits = 10;
		break;
	}
	info->pendregs[3] = info->curregs[3];
	info->pendregs[5] = info->curregs[5];

	info->curregs[4] &= ~(SB_MASK | PAR_ENA | PAR_EVEN);
	if (cflag & CSTOPB) {
		info->curregs[4] |= SB2;
		bits++;
		BAUDBG("2 stop, ");
	} else {
		info->curregs[4] |= SB1;
		BAUDBG("1 stop, ");
	}
	if (cflag & PARENB) {
		bits++;
		info->curregs[4] |= PAR_ENA;
		BAUDBG("parity, ");
	}
	if (!(cflag & PARODD)) {
		info->curregs[4] |= PAR_EVEN;
	}
	info->pendregs[4] = info->curregs[4];

	if (!(cflag & CLOCAL)) {
		if (!(info->curregs[15] & DCDIE))
			info->read_reg_zero = read_zsreg(info->zs_channel, 0);
		info->curregs[15] |= DCDIE;
	} else
		info->curregs[15] &= ~DCDIE;
	if (cflag & CRTSCTS) {
		info->curregs[15] |= CTSIE;
		if ((read_zsreg(info->zs_channel, 0) & CTS) != 0)
			info->tx_stopped = 1;
	} else {
		info->curregs[15] &= ~CTSIE;
		info->tx_stopped = 0;
	}
	info->pendregs[15] = info->curregs[15];

	/* Calc timeout value. This is pretty broken with high baud rates with HZ=100.
	   This code would love a larger HZ and a >1 fifo size, but this is not
	   a priority. The resulting value must be >HZ/2
	 */
	info->timeout = ((info->xmit_fifo_size * HZ * bits) / baud);
	info->timeout += HZ / 50 + 1;	/* Add .02 seconds of slop */

	BAUDBG("timeout=%d/%ds, base:%d\n", (int)info->timeout, (int)HZ,
	       (int)info->baud_base);

	/* Load up the new values */
	load_zsregs(info->zs_channel, info->curregs);

/*
	dump_zsregs(info->zs_channel, info->curregs);
*/
	spin_unlock_irqrestore(&info->lock, flags);
}


static void rs_flush_chars(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

	if (serial_paranoia_check(info, tty->name, "rs_flush_chars"))
		return;

	spin_lock_irqsave(&info->lock, flags);
	if (!(info->xmit_cnt <= 0 || tty->stopped || info->tx_stopped ||
	    !info->xmit_buf))
		/* Enable transmitter */
		transmit_chars(info);
	spin_unlock_irqrestore(&info->lock, flags);
}

static int rs_write(struct tty_struct *tty, 
		    const unsigned char *buf, int count)
{
	int c, ret = 0;
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

#ifdef SERIAL_DEBUG_WRITE
	printk("rs_write: count %d\n", count);
#endif

	if (serial_paranoia_check(info, tty->name, "rs_write"))
		return 0;

	if (!tty || !info->xmit_buf || !tmp_buf)
		return 0;

	while (1) {
		spin_lock_irqsave(&info->lock, flags);
		c = MIN(count,
			MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
			    SERIAL_XMIT_SIZE - info->xmit_head));
		if (c <= 0) {
			spin_unlock_irqrestore(&info->lock, flags);
			break;
		}
		memcpy(info->xmit_buf + info->xmit_head, buf, c);
		info->xmit_head = ((info->xmit_head + c) &
				   (SERIAL_XMIT_SIZE - 1));
		info->xmit_cnt += c;
		spin_unlock_irqrestore(&info->lock, flags);
		buf += c;
		count -= c;
		ret += c;
	}
	spin_lock_irqsave(&info->lock, flags);
	if (info->xmit_cnt && !tty->stopped && !info->tx_stopped
	    && !info->tx_active)
		transmit_chars(info);
	spin_unlock_irqrestore(&info->lock, flags);

	return ret;
}

static int rs_write_room(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	int ret;

	if (serial_paranoia_check(info, tty->name, "rs_write_room"))
		return 0;
	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int rs_chars_in_buffer(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;

	if (serial_paranoia_check(info, tty->name, "rs_chars_in_buffer"))
		return 0;
	return info->xmit_cnt;
}

static void rs_flush_buffer(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

	if (serial_paranoia_check(info, tty->name, "rs_flush_buffer"))
		return;
	spin_lock_irqsave(&info->lock, flags);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	spin_unlock_irqrestore(&info->lock, flags);
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup) (tty);
}

/*
 * ------------------------------------------------------------
 * rs_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void rs_throttle(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

#ifdef SERIAL_DEBUG_THROTTLE
	printk("throttle 0x%x....\n", tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->name, "rs_throttle"))
		return;

	if (I_IXOFF(tty)) {
		spin_lock_irqsave(&info->lock, flags);
		info->x_char = STOP_CHAR(tty);
		if (!info->tx_active)
			transmit_chars(info);
		spin_unlock_irqrestore(&info->lock, flags);
	}

	if (C_CRTSCTS(tty)) {
		/*
		 * Here we want to turn off the RTS line.  On Macintoshes,
		 * we only get the DTR line, which goes to both DTR and
		 * RTS on the modem.  RTS doesn't go out to the serial
		 * port socket.  So you should make sure your modem is
		 * set to ignore DTR if you're using CRTSCTS.
		 */
		spin_lock_irqsave(&info->lock, flags);
		info->curregs[5] &= ~(DTR | RTS);
		info->pendregs[5] &= ~(DTR | RTS);
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		spin_unlock_irqrestore(&info->lock, flags);
	}
#ifdef CDTRCTS
//	if (tty->termios->c_cflag & CDTRCTS) {
		spin_lock_irqsave(&info->lock, flags);
		info->curregs[5] &= ~DTR;
		info->pendregs[5] &= ~DTR;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		spin_unlock_irqrestore(&info->lock, flags);
//	}
#endif		/* CDTRCTS */
}

static void rs_unthrottle(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

#ifdef SERIAL_DEBUG_THROTTLE
	printk("unthrottle %d: ...\n", tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->name, "rs_unthrottle"))
		return;

	if (I_IXOFF(tty)) {
		spin_lock_irqsave(&info->lock, flags);
		if (info->x_char)
			info->x_char = 0;
		else {
			info->x_char = START_CHAR(tty);
			if (!info->tx_active)
				transmit_chars(info);
		}
		spin_unlock_irqrestore(&info->lock, flags);
	}

	if (C_CRTSCTS(tty)) {
		/* Assert RTS and DTR lines */
		spin_lock_irqsave(&info->lock, flags);
		info->curregs[5] |= DTR | RTS;
		info->pendregs[5] |= DTR | RTS;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		spin_unlock_irqrestore(&info->lock, flags);
	}
#ifdef CDTRCTS
//	if (tty->termios->c_cflag & CDTRCTS) {
		/* Assert DTR line */
		spin_lock_irqsave(&info->lock, flags);
		info->curregs[5] |= DTR;
		info->pendregs[5] |= DTR;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		spin_unlock_irqrestore(&info->lock, flags);
//	}
#endif
}

/*
 * ------------------------------------------------------------
 * rs_ioctl() and friends
 * ------------------------------------------------------------
 */

static int get_serial_info(struct mesq_serial *info,
                           struct serial_struct __user * retinfo)
{
	struct serial_struct tmp;

	if (!retinfo)
		return -EFAULT;
	memset(&tmp, 0, sizeof(tmp));
	tmp.type = info->type;
	tmp.line = info->line;
	tmp.port = info->port;
	tmp.irq = info->irq;
	tmp.flags = info->flags;
	tmp.baud_base = info->baud_base;
	tmp.close_delay = info->close_delay;
	tmp.closing_wait = info->closing_wait;
	tmp.custom_divisor = info->custom_divisor;
	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

static int set_serial_info(struct mesq_serial *info,
                           struct serial_struct __user *new_info)
{
	struct serial_struct new_serial;
	struct mesq_serial old_info;
	int retval = 0;

	if (!new_info)
		return -EFAULT;

	if (copy_from_user(&new_serial, new_info, sizeof(new_serial)))
		return -EFAULT;
	old_info = *info;

	if (!capable(CAP_SYS_ADMIN)) {
		if ((new_serial.baud_base != info->baud_base) ||
		    (new_serial.type != info->type) ||
		    (new_serial.close_delay != info->close_delay) ||
		    ((new_serial.flags & ~ZILOG_USR_MASK) !=
		     (info->flags & ~ZILOG_USR_MASK)))
			return -EPERM;
		info->flags = ((info->flags & ~ZILOG_USR_MASK) |
			       (new_serial.flags & ZILOG_USR_MASK));
		info->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	if (info->count > 1)
		return -EBUSY;

	/*
	 * OK, past this point, all the error checking has been done.
	 * At this point, we start making changes.....
	 */

	info->baud_base = new_serial.baud_base;
	info->flags = ((info->flags & ~ZILOG_FLAGS) |
		       (new_serial.flags & ZILOG_FLAGS));
	info->type = new_serial.type;
	info->close_delay = new_serial.close_delay;
	info->closing_wait = new_serial.closing_wait;

check_and_exit:
	if (info->flags & ZILOG_INITIALIZED)
		retval = setup_scc(info);
	return retval;
}

/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space. 
 */
static int get_lsr_info(struct mesq_serial *info, unsigned int *value)
{
	unsigned char status;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	status = read_zsreg(info->zs_channel, 0);
	spin_unlock_irqrestore(&info->lock, flags);
	status = (status & Tx_BUF_EMP) ? TIOCSER_TEMT : 0;
	return put_user(status, value);
}

static int rs_tiocmget(struct tty_struct *tty, struct file *file)
{
        struct mesq_serial * info = (struct mesq_serial *)tty->driver_data;
        unsigned char control, status;
        unsigned long flags;
                                                                                
        if (serial_paranoia_check(info, tty->name, __FUNCTION__))
                return -ENODEV;
                                                                                
        if (tty->flags & (1 << TTY_IO_ERROR))
                return -EIO;
                                                                                
	spin_lock_irqsave(&info->lock, flags);
        control = info->curregs[5];
        status = read_zsreg(info->zs_channel, 0);
	spin_unlock_irqrestore(&info->lock, flags);
        return    ((control & RTS) ? TIOCM_RTS: 0)
                | ((control & DTR) ? TIOCM_DTR: 0)
                | ((status  & DCD) ? TIOCM_CAR: 0)
		| ((status  & CTS) ? 0: TIOCM_CTS);
}
                                                                                
static int rs_tiocmset(struct tty_struct *tty, struct file *file,
                       unsigned int set, unsigned int clear)
{
        struct mesq_serial * info = (struct mesq_serial *)tty->driver_data;
        unsigned long flags;
                                                                                
        if (serial_paranoia_check(info, tty->name, __FUNCTION__))
                return -ENODEV;
                                                                                
        if (tty->flags & (1 << TTY_IO_ERROR))
                return -EIO;
                                                                                
	spin_lock_irqsave(&info->lock, flags);
        if (set & TIOCM_RTS)
                info->curregs[5] |= RTS;
                                                                                
        if (set & TIOCM_DTR)
                info->curregs[5] |= DTR;
        if (clear & TIOCM_RTS)
                info->curregs[5] &= ~RTS;
        if (clear & TIOCM_DTR)
                info->curregs[5] &= ~DTR;
                                                                                
        info->pendregs[5] = info->curregs[5];
        write_zsreg(info->zs_channel, 5, info->curregs[5]);
	spin_unlock_irqrestore(&info->lock, flags);
        return 0;
}

/*
 * rs_break - turn transmit break condition on/off
 */
static void rs_break(struct tty_struct *tty, int break_state)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

	if (serial_paranoia_check(info, tty->name, "rs_break"))
		return;

	spin_lock_irqsave(&info->lock, flags);
	if (break_state == -1)
		info->curregs[5] |= SND_BRK;
	else
		info->curregs[5] &= ~SND_BRK;
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	spin_unlock_irqrestore(&info->lock, flags);
}

static int rs_ioctl(struct tty_struct *tty, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;

	if (serial_paranoia_check(info, tty->name, "rs_ioctl"))
		return -ENODEV;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
			return -EIO;
	}

	switch (cmd) {
	case TIOCGSERIAL:
		return get_serial_info(info, (struct serial_struct __user *)arg);
	case TIOCSSERIAL:
		return set_serial_info(info, (struct serial_struct __user *)arg);
	case TIOCSERGETLSR:	/* Get line status register */
		return get_lsr_info(info, (unsigned int *)arg);

	case TIOCSERGSTRUCT:
		if (copy_to_user((struct mesq_serial __user *)arg,
				 info, sizeof(struct mesq_serial)))
			return -EFAULT;
		return 0;

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static void rs_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	int was_stopped;

	if (tty->termios->c_cflag == old_termios->c_cflag)
		return;
	was_stopped = info->tx_stopped;

	change_speed(info, old_termios);

	if (was_stopped && !info->tx_stopped) {
		tty->hw_stopped = 0;
		rs_start(tty);
	}
}

/*
 * ------------------------------------------------------------
 * rs_close()
 * 
 * This routine is called when the serial port gets closed.
 * Wait for the last remaining data to be sent.
 * ------------------------------------------------------------
 */
static void rs_close(struct tty_struct *tty, struct file *filp)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long flags;

	if (!info || serial_paranoia_check(info, tty->name, "rs_close"))
		return;

	spin_lock_irqsave(&info->lock, flags);

	if (tty_hung_up_p(filp)) {
		spin_unlock_irqrestore(&info->lock, flags);
		return;
	}

	OPNDBG("rs_close ttyS%d, count = %d\n", info->line, info->count);
	if ((tty->count == 1) && (info->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk(KERN_ERR
		       "rs_close: bad serial port count; tty->count is 1, "
		       "info->count is %d\n", info->count);
		info->count = 1;
	}
	if (--info->count < 0) {
		printk(KERN_ERR
		       "rs_close: bad serial port count for ttyS%d: %d\n",
		       info->line, info->count);
		info->count = 0;
	}
	if (info->count) {
		spin_unlock_irqrestore(&info->lock, flags);
		return;
	}
	info->flags |= ZILOG_CLOSING;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify 
	 * the line discipline to only process XON/XOFF characters.
	 */
	OPNDBG("waiting end of Tx... (timeout:%d)\n", info->closing_wait);
	tty->closing = 1;
	if (info->closing_wait != ZILOG_CLOSING_WAIT_NONE) {
		spin_unlock_irqrestore(&info->lock, flags);
		tty_wait_until_sent(tty, info->closing_wait);
		spin_lock_irqsave(&info->lock, flags);
	}
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receiver and receive interrupts.
	 */
	info->curregs[3] &= ~RxENABLE;
	info->pendregs[3] = info->curregs[3];
	write_zsreg(info->zs_channel, 3, info->curregs[3]);
	info->curregs[1] &= ~(0x18);	/* disable any rx ints */
	info->pendregs[1] = info->curregs[1];
	write_zsreg(info->zs_channel, 1, info->curregs[1]);
	ZS_CLEARFIFO(info->zs_channel);
	if (info->flags & ZILOG_INITIALIZED) {
		/*
		 * Before we drop DTR, make sure the SCC transmitter
		 * has completely drained.
		 */
		OPNDBG("waiting end of Rx...\n");
		spin_unlock_irqrestore(&info->lock, flags);
		rs_wait_until_sent(tty, info->timeout);
		spin_lock_irqsave(&info->lock, flags);
	}

	shutdown(info);
	/* restore flags now since shutdown() will have disabled this port's
	   specific irqs */
	spin_unlock_irqrestore(&info->lock, flags);

        if (tty->driver->flush_buffer)
                tty->driver->flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;

	if (info->blocked_open) {
		if (info->close_delay) {
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(info->close_delay);
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ZILOG_NORMAL_ACTIVE | ZILOG_CALLOUT_ACTIVE |
			 ZILOG_CLOSING);
	wake_up_interruptible(&info->close_wait);
}

/*
 * rs_wait_until_sent() --- wait until the transmitter is empty
 */
static void rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;
	unsigned long orig_jiffies, char_time;

	if (serial_paranoia_check(info, tty->name, "rs_wait_until_sent"))
		return;

/*	printk("rs_wait_until_sent, timeout:%d, tty_stopped:%d, tx_stopped:%d\n",
			timeout, tty->stopped, info->tx_stopped);
*/
	orig_jiffies = jiffies;
	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send also be less than the timeout.
	 */
	if (info->timeout <= HZ / 50) {
		printk("sdd: invalid info->timeout=%d\n", info->timeout);
		info->timeout = HZ / 50 + 1;
	}

	char_time = (info->timeout - HZ / 50) / info->xmit_fifo_size;
	char_time = char_time / 5;
	if (char_time > HZ) {
		printk("sdd: char_time %ld >HZ !!!\n", char_time);
		char_time = 1;
	} else if (char_time == 0)
		char_time = 1;
	if (timeout)
		char_time = MIN(char_time, timeout);
	while ((read_zsreg(info->zs_channel, 1) & ALL_SNT) == 0) {
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(char_time);
		if (signal_pending(current))
			break;
		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
	current->state = TASK_RUNNING;
}

/*
 * rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void rs_hangup(struct tty_struct *tty)
{
	struct mesq_serial *info = (struct mesq_serial *)tty->driver_data;

	if (serial_paranoia_check(info, tty->name, "rs_hangup"))
		return;

	rs_flush_buffer(tty);
	shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ZILOG_NORMAL_ACTIVE | ZILOG_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * rs_open() and friends
 * ------------------------------------------------------------
 */
static int block_til_ready(struct tty_struct *tty, struct file *filp,
			   struct mesq_serial *info)
{
	DECLARE_WAITQUEUE(wait, current);
	int retval;
	int do_clocal = 0;

	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if (info->flags & ZILOG_CLOSING) {
		interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		return ((info->flags & ZILOG_HUP_NOTIFY) ?
			-EAGAIN : -ERESTARTSYS);
#else
		return -EAGAIN;
#endif
	}

	/*
	 * If this is a callout device, then just make sure the normal
	 * device isn't being used.
	 */

	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ((filp->f_flags & O_NONBLOCK) || (tty->flags & (1 << TTY_IO_ERROR))) {
		info->flags |= ZILOG_NORMAL_ACTIVE;
		return 0;
	}

	if (tty->termios->c_cflag & CLOCAL)
		do_clocal = 1;

	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * rs_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
	OPNDBG("block_til_ready before block: ttyS%d, count = %d\n",
	       info->line, info->count);
	spin_lock_irq(&info->lock);
	if (!tty_hung_up_p(filp))
		info->count--;
	spin_unlock_irq(&info->lock);
	info->blocked_open++;
	while (1) {
		spin_lock_irq(&info->lock);
                if ((tty->termios->c_cflag & CBAUD) && !info->is_irda) {
			zs_rtsdtr(info, 1);
		}
		spin_unlock_irq(&info->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		if (tty_hung_up_p(filp) || !(info->flags & ZILOG_INITIALIZED)) {
#ifdef SERIAL_DO_RESTART
			if (info->flags & ZILOG_HUP_NOTIFY)
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;
#else
			retval = -EAGAIN;
#endif
			break;
		}
               if (!(info->flags & ZILOG_CLOSING) &&
		    (do_clocal || (read_zsreg(info->zs_channel, 0) & DCD)))
			break;
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		OPNDBG("block_til_ready blocking: ttyS%d, count = %d\n",
		       info->line, info->count);
		schedule();
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&info->open_wait, &wait);
	if (!tty_hung_up_p(filp))
		info->count++;
	info->blocked_open--;
	OPNDBG("block_til_ready after blocking: ttyS%d, count = %d\n",
	       info->line, info->count);
	if (retval)
		return retval;
	info->flags |= ZILOG_NORMAL_ACTIVE;
	return 0;
}

static void show_serial_version(void)
{
        printk(KERN_INFO "Z85230/Z8536 serial driver version " MESQSERIAL_VERSION "\n");
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its ZILOG structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int rs_open(struct tty_struct *tty, struct file *filp)
{
	struct mesq_serial *info;
	int retval, line;
	unsigned long page;

	line = tty->index;
	if ((line < 0) || (line >= zs_channels_found))
		return -ENODEV;
	info = zs_soft + line;

	if (serial_paranoia_check(info, tty->name, "rs_open"))
		return -ENODEV;
	OPNDBG("rs_open %s, count = %d tty=%p\n",
	       tty->name, info->count, tty);

	info->count++;
	tty->driver_data = info;
	info->tty = tty;

	if (!tmp_buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;
		if (tmp_buf)
			free_page(page);
		else
			tmp_buf = (unsigned char *)page;
	}

	/*
	 * If the port is the middle of closing, bail out now
	 */
	if (tty_hung_up_p(filp) || (info->flags & ZILOG_CLOSING)) {
		if (info->flags & ZILOG_CLOSING)
			interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		return ((info->flags & ZILOG_HUP_NOTIFY) ?
			-EAGAIN : -ERESTARTSYS);
#else
		return -EAGAIN;
#endif
	}

	/*
	 * Start up serial port
	 */

	retval = startup(info, 1);
	if (retval)
		return retval;


	retval = block_til_ready(tty, filp, info);
	if (retval) {
		OPNDBG("rs_open returning after block_til_ready with %d\n",
		       retval);
		return retval;
	}

	OPNDBG("rs_open ttyS%d successful...\n", info->line);
	return 0;
}

static inline int line_info(char *buf, struct mesq_serial *info)
{
	int ret = 0;

	ret += sprintf(buf, "%d: port:0x%X irq:%d",
		       info->line, info->port, info->irq);
	ret += sprintf(buf + ret, "\n");
	return ret;
}

int mesqserial_read_proc(char *page, char **start, off_t off, int count,
			 int *eof, void *data)
{
	int l, len = 0;
	off_t begin = 0;
	struct mesq_serial *info;

	len += sprintf(page, "serinfo:1.0 driver:" MESQSERIAL_VERSION "\n");
	for (info = zs_chain; info && len < 4000; info = info->zs_next) {
		l = line_info(page + len, info);
		len += l;
		if (len + begin > off + count)
			goto done;
		if (len + begin < off) {
			begin += len;
			len = 0;
		}
	}
	*eof = 1;
      done:
	if (off >= len + begin)
		return 0;
	*start = page + (off - begin);
	return ((count < begin + len - off) ? count : begin + len - off);
}

static int probe_sccs()
{
	struct mesq_serial **pp;
	int n_chips = 0, n_channels, chip, channel;
	unsigned long system_base = Z85C230BA;
	struct pci_dev *dev;

#ifdef SERIAL_DEBUG_INIT
	printk("SDD probe \n");
#endif

#ifdef CONFIG_PPC_PREP
        dev = pci_find_device(PCI_VENDOR_ID_VIA,
                                   PCI_DEVICE_ID_VIA_82C586_0, NULL);

#else
        dev = pci_find_device(PCI_VENDOR_ID_WINBOND,
                                   PCI_DEVICE_ID_WINBOND_83C553, NULL);
#endif

	if (dev == NULL) 
		return -1;

	n_chips = NUM_SERIAL;

	pp = &zs_chain;

	n_channels = 0;

	for (chip = 0; chip < n_chips; chip++) {
		for (channel = 0; channel <= 1; channel++) {

			zs_channels[n_channels].control =
			    (volatile unsigned char *)
			    system_base + (0 == channel ? 2 : 0);
			zs_channels[n_channels].data =
			    zs_channels[n_channels].control + 1;
			spin_lock_init(&zs_channels[n_channels].lock);
			zs_soft[n_channels].zs_channel =
			    &zs_channels[n_channels];

			zs_soft[n_channels].irq = Z85230_IRQ;
			zs_soft[n_channels].zs_chan_a = &zs_channels[0];

			*pp = &zs_soft[n_channels];
			pp = &zs_soft[n_channels].zs_next;
			n_channels++;
		}
	}

	*pp = 0;
	zs_channels_found = n_channels;
#ifdef SERIAL_DEBUG_INIT
	printk("chan_a 0 0x%p ", zs_soft[0].zs_chan_a->control);
	printk("chan_a 1 0x%p\n", zs_soft[1].zs_chan_a->control);
	printk("SDD probe done\n");
#endif
	return 0;
}

static void z8536_init(void)
{
	unsigned long system_base = Z85C230BA;
	volatile unsigned char *control_reg;

	control_reg = (volatile unsigned char *)system_base + CIOOFFS;

	spin_lock_init(&z8536_lock);
	/*
	 * Disable master interrupt enable
	 * Disable all counters
	 * Disable counters interrupt enables
	 */

	write_z8536reg(control_reg, Z8536_RO_MIC, 0x00);
	write_z8536reg(control_reg, Z8536_RO_MCC, 0x00);
	write_z8536reg(control_reg, Z8536_RO_CT1MS, 0x00);
	write_z8536reg(control_reg, Z8536_RO_CT2MS, 0x00);
	write_z8536reg(control_reg, Z8536_RO_CT3MS, 0x00);

	/* Setup port specifics */

	write_z8536reg(control_reg, Z8536_RO_PAD, 0x40);
	write_z8536reg(control_reg, Z8536_RO_PBD, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PCD, 0x02);

	write_z8536reg(control_reg, Z8536_RO_PAMS, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PBMS, 0x00);

	write_z8536reg(control_reg, Z8536_RO_PAHS, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PBHS, 0x00);

	write_z8536reg(control_reg, Z8536_RO_PACS, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PBCS, 0x00);

	write_z8536reg(control_reg, Z8536_RO_PADPP, 0xBF);
	write_z8536reg(control_reg, Z8536_RO_PBDPP, 0x3F);
	write_z8536reg(control_reg, Z8536_RO_PCDPP, 0x00);

	write_z8536reg(control_reg, Z8536_RO_PAIOC, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PBIOC, 0x00);
	write_z8536reg(control_reg, Z8536_RO_PCIOC, 0x00);

	write_z8536reg(control_reg, Z8536_RO_PADD, 0x07);
	write_z8536reg(control_reg, Z8536_RO_PBDD, 0xC7);
	write_z8536reg(control_reg, Z8536_RO_PCDD, 0x0D);

	/* Enable ports A, B, and C */
	write_z8536reg(control_reg, Z8536_RO_MCC, 0x94);
}

static struct tty_operations serial_ops = {
        .open = rs_open,
        .close = rs_close,
        .write = rs_write,
        .flush_chars = rs_flush_chars,
        .write_room = rs_write_room,
        .chars_in_buffer = rs_chars_in_buffer,
        .flush_buffer = rs_flush_buffer,
        .ioctl = rs_ioctl,
        .throttle = rs_throttle,
        .unthrottle = rs_unthrottle,
        .set_termios = rs_set_termios,
        .stop = rs_stop,
        .start = rs_start,
        .hangup = rs_hangup,
        .read_proc = mesqserial_read_proc,
        .break_ctl = rs_break,
        .wait_until_sent = rs_wait_until_sent,
        .tiocmget = rs_tiocmget,
        .tiocmset = rs_tiocmset,
};

/* rs_init inits the driver */
static int __init mesqserial_init(void)
{
	int channel, i;
	struct mesq_serial *info;
	unsigned long flags;
	int ret;

#ifdef SERIAL_DEBUG_INIT
	printk("SDD init\n");
#endif

	/* Find out how many Z8530 SCCs we have */
	if (zs_chain == 0)
		if (probe_sccs() != 0)
			return -ENODEV;

	z8536_init();

        serial_driver = alloc_tty_driver(zs_channels_found);
        if (!serial_driver)
                return -ENOMEM;

	show_serial_version();

	/* Initialize the tty_driver structure */
	/* Not all of this is exactly right for us. */

	serial_driver->owner = THIS_MODULE;
        serial_driver->driver_name = "mesqserial";
#if 0
        serial_driver->name = "ttyS";
	serial_driver->major = TTY_MAJOR;
#if defined(CONFIG_MVME6100) || defined(CONFIG_MVME5500) || defined(CONFIG_MVME5100)
	serial_driver->minor_start = 68;
#else
	serial_driver->minor_start = 66;
#endif
#endif
        serial_driver->name = "ttyPZ";
        serial_driver->major = 204;
	serial_driver->minor_start = 192;
        serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
        serial_driver->subtype = SERIAL_TYPE_NORMAL;
        serial_driver->init_termios = tty_std_termios;
        serial_driver->init_termios.c_cflag =
                B38400 | CS8 | CREAD | HUPCL | CLOCAL;
        serial_driver->init_termios.c_ispeed = 38400;
        serial_driver->init_termios.c_ospeed = 38400;
        serial_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(serial_driver, &serial_ops);

	if ((ret = tty_register_driver(serial_driver))) {
		printk(KERN_ERR "sdd: Couldn't register serial driver. ret %d\n", ret);
		put_tty_driver(serial_driver);
		return (-EBUSY);
	}

	for (channel = 0; channel < zs_channels_found; ++channel) {
		zs_soft[channel].clk_divisor = 16;
// 		zs_soft[channel].zs_baud = get_zsbaud(&zs_soft[channel]);
		zs_soft[channel].zs_baud = 38400;

		zs_soft[channel].is_cons = 0;	/* no  console. */
		zs_soft[channel].is_internal_modem = 0;	
		zs_soft[channel].is_irda = 0;	

		/* If console serial line, then enable interrupts. */
		if (zs_soft[channel].is_cons) {
			printk("sdd: console line, enabling interrupt %d\n",
			       zs_soft[channel].irq);
			panic("sdd: console not supported yet !");
			write_zsreg(zs_soft[channel].zs_channel, R1,
				    (EXT_INT_ENAB | INT_ALL_Rx | TxINT_ENAB));
			write_zsreg(zs_soft[channel].zs_channel, R9,
				    (NV | MIE));
		}
	}

	for (info = zs_chain, i = 0; info; info = info->zs_next, i++) {
		info->magic = SERIAL_MAGIC;
		info->port = (int)info->zs_channel->control;
		info->line = i;
		info->tty = 0;
		info->custom_divisor = 16;
		info->timeout = 0;
		info->close_delay = 50;
		info->closing_wait = 3000;
		info->x_char = 0;
		info->event = 0;
		info->count = 0;
		info->blocked_open = 0;
		INIT_WORK(&info->tqueue, do_softint);
		spin_lock_init(&info->lock);
		init_waitqueue_head(&info->open_wait);
		init_waitqueue_head(&info->close_wait);
		info->timeout = HZ;
//		printk(KERN_INFO "ttyS%01d at 0x%08x (irq = %d) is a Z85230 ESCC\n", 
		printk(KERN_INFO "ttyPZ%01d at 0x%08x (irq = %d) is a Z85230 ESCC\n", 
//			(serial_driver->minor_start - 64) + info->line, 
			info->line, 
			info->port, 
			info->irq);

#ifdef CONFIG_XMON
		if (!info->is_internal_modem)
			continue;
#endif
	}
	local_irq_save(flags);
	/* Register the interrupt handler for each chip */
	for (i = 0; i < zs_channels_found; ++i) {
		if ((ret = request_irq(zs_soft[i].irq, rs_interrupt, IRQF_SHARED,
				       "SDD", &zs_soft[i])))
			printk(KERN_ERR "sdd: can't get irq %d, ret=%d\n",
			       zs_soft[i].irq, ret);
	}
	local_irq_restore(flags);

	tmp_buf = 0;
#ifdef SERIAL_DEBUG_INIT
	printk("SDD init done\n");
#endif

	return 0;
}


static void __exit mesqserial_exit(void)
{
	int i;
	int e1;

	for (i = 0; i < zs_channels_found; ++i)
		free_irq(zs_soft[i].irq, &zs_soft[i]);

	if ((e1 = tty_unregister_driver(serial_driver)))
		printk("SDD: failed to unregister serial driver (%d)\n", e1);

	put_tty_driver(serial_driver);
	if (tmp_buf) {
		free_page((unsigned long)tmp_buf);
		tmp_buf = 0;
	}
}

module_init(mesqserial_init);
module_exit(mesqserial_exit);

MODULE_LICENSE("GPL");

