/*
 *  open_pic3.h -- OpenPIC Interrupt Handling
 *
 *  Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive
 *  for more details.
 *
 */

#ifndef _OPEN_PIC3_H
#define _OPEN_PIC3_H


#define OPENPIC_SIZE	0x40000

/*
 *  Non-offset'ed vector numbers
 */

#define OPENPIC_VEC_TIMER	64	/* and up */
#define OPENPIC_VEC_IPI		72	/* and up */
#define OPENPIC_VEC_SPURIOUS	127

/* Priorities */
#define OPENPIC_PRIORITY_IPI_BASE       10
#define OPENPIC_PRIORITY_DEFAULT        4
#define OPENPIC_PRIORITY_NMI            9

/* OpenPIC IRQ controller structure */
extern struct hw_interrupt_type open_pic;

/* OpenPIC IPI controller structure */
#ifdef CONFIG_SMP
extern struct hw_interrupt_type open_pic_ipi;
#endif /* CONFIG_SMP */

extern u_int OpenPIC_NumInitSenses;
extern u_char *OpenPIC_InitSenses;
extern void __iomem * OpenPIC_Addr;
extern int epic_serial_mode;

extern u_int OpenPIC2_NumInitSenses;
extern u_char *OpenPIC2_InitSenses;
extern void __iomem * OpenPIC2_Addr;

extern inline int openpic_to_irq(int irq)
{
	/* IRQ 0 usually means 'disabled'.. don't mess with it
	 * exceptions to this (sandpoint maybe?)
	 * shouldn't use openpic_to_irq
	 */
	if (irq != 0){
		return irq += NUM_8259_INTERRUPTS;
	} else {
		return 0;
	}
}
/*extern int open_pic_irq_offset;*/

extern void openpic_set_sources(int first_irq, int num_irqs, void __iomem *isr);
extern void openpic2_set_sources(int first_irq, int num_irqs, void __iomem *isr);

typedef enum irq_polarity_en {
	OP_IRQ_POS = 1,
	OP_IRQ_NEG = 0
} irq_polarity_type;

typedef enum irq_sense_en {
	OP_IRQ_LEVEL = 1,
	OP_IRQ_EDGE  = 0
} irq_sense_type;

typedef struct openpic_irq_def_str {
	int			PICIrq;		/* Must be int (not u_int) to mark end of table */
	u_int			Vector;
	u_char			Priority;
	irq_sense_type		IrqSense;
	irq_polarity_type	IrqPolarity;
	/* This can be used for cascade, but should work well for the chrp specials too */
	int			(*CascadeAckHandler)(void);
} openpic_irq_def;

typedef struct openpic_def_str {
	volatile struct OpenPIC	__iomem *OpenPIC_Addr;
	int				slave_pic;
	openpic_irq_def	*IRQdef;	/* The lowest vector first in the table !!! */
} openpic_def;

#define ALLOC_OPENPIC_DEF(name) static openpic_def name __initdata = {NULL, 0, NULL}

/* Exported functions */
extern void openpic_init(openpic_def *);
extern void openpic2_init(openpic_def *);
extern u_int openpic_irq(void);
extern void openpic_eoi(void);
extern void openpic_enable_irq(u_int irq);
extern void openpic_request_IPIs(void);
extern void do_openpic_setup_cpu(void);
extern int openpic_get_irq(void);
extern void openpic_reset_processor_phys(u_int cpumask);
extern void openpic_setup_ISU(int isu_num, unsigned long addr);
extern void openpic_cause_IPI(u_int ipi, u_int cpumask);
extern void smp_openpic_message_pass(int target, int msg);

#endif /* __OPEN_PIC3_H */
