#include <asm/processor.h>
#include <asm/cache.h>
#include <asm/ppc_asm.h>   

	.text


	.globl e500_enable_l1_data
e500_enable_l1_data:
        addis   r3,r0,0x0000    /* add CE */
        ori     r3,r3,0x0001    /* add CE */
        sync                    /* synchronize */
        isync                   /* synchronize */
        mtspr   1010,r3         /* store L1CSR0 register */
        isync                   /* synchronize */
        sync                    /* synchronize */
        blr             	/* return to caller */ 


	.globl e500_disable_l1_data
e500_disable_l1_data:
        /*
         * read 512K to force castouts
         */
        addis   r3,r0,0x0000
        ori     r3,r3,0x0000
        addis   r4,r0,0x0008
        ori     r4,r4,0x0000
e500_disable_l1:
        lwz     r5,0(r3)
        addi    r3,r3,0x20
        cmp     0,0,r3,r4
        bne     e500_disable_l1
 
        sync
        addis   r3,r0,0x0000    /* add CE */
        ori     r3,r3,0x0001    /* add CE */
        isync                   /* synchronize */
        sync                    /* synchronize */
        mfspr   r4,1010         /* load spr1010 register */
        sync                    /* synchronize */
        andc    r4,r4,r3        /* clear bit (disable cache) */
        ori     r4,r4,2         /* set bit (invalidate cache) */
        sync                    /* synchronize */
        mtspr   1010,r4         /* store spr1010 register */
        sync                    /* synchronize */
        blr            		/* return to caller */  


	.globl e500_invalidate_l1_data
e500_invalidate_l1_data:
        addi    r3,r0,0x0002    /* add CFI */
        isync                   /* synchronize */
        mfspr   r4,1010         /* load L1CSR0 register */
        isync                   /* synchronize */
        or      r4,r4,r3        /* set bit (invalidate cache) */
        sync                    /* synchronize */
        isync                   /* synchronize */
        mtspr   1010,r4         /* store L1CSR0 register */
        isync                   /* synchronize */
        sync                    /* synchronize */
        blr            		/* return to caller */  

	.globl e500_flush_l1_data
e500_flush_l1_data:
        /*
         * No flush assist from HW, so just read 512K to force castouts
         */
        addis   r3,r0,0x0000
        ori     r3,r3,0x0000
        addis   r4,r0,0x0008
        ori     r4,r4,0x0000
e500_flush_l1:
        lwz     r5,0(r3)
        addi    r3,r3,0x20
        cmp     0,0,r3,r4
        bne    e500_flush_l1 
        sync
        b       e500_invalidate_l1_data /* return through invalidate routine */ 

	.globl e500_enable_l1_inst
e500_enable_l1_inst:
        addis   r3,r0,0x0000    /* add CE */
        ori     r3,r3,0x0001    /* add CE */
        sync                    /* synchronize */
        isync                   /* synchronize */
        mtspr   1011,r3         /* store L1CSR1 register */
        isync                   /* synchronize */
        sync                    /* synchronize */
        blr            		/* return to caller */ 


	.globl e500_disable_l1_inst
e500_disable_l1_inst:
        addis   r3,r0,0x0000    /* add ICE */
        ori     r3,r3,0x0001    /* add ICE */
        isync                   /* synchronize */
        sync                    /* synchronize */
        mfspr   r4,1011         /* load spr1011 register */
        sync                    /* synchronize */
        andc    r4,r4,r3        /* clear bit (disable cache) */
        ori     r4,r4,2         /* set bit (invalidate cache) */
        sync                    /* synchronize */
        mtspr   1011,r4         /* store spr1011 register */
        sync                    /* synchronize */
        blr            		/* return to caller */ 

	.globl e500_invalidate_l1_inst
e500_invalidate_l1_inst:
        addi    r3,r0,0x0002    /* add ICFI */
        isync                   /* synchronize */
        mfspr   r4,1011         /* load L1CSR1 register */
        isync                   /* synchronize */
        or      r4,r4,r3        /* set bit (invalidate cache) */
        sync                    /* synchronize */
        isync                   /* synchronize */
        mtspr   1011,r4         /* store L1CSR1 register */
        isync                   /* synchronize */
        sync                    /* synchronize */
        blr            		/* return to caller */ 

