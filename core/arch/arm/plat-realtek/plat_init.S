/*
 * Entry points for the A7 init.
 */

#include <arm.h>
#include <arm32.h>
#include <arm32_macros.S>
#include <asm.S>
#include <platform_config.h>


.section .text
.balign 4
.code 32


FUNC plat_cpu_reset_early , :
	mov r10, lr
	
	/* Check if primary core */
	bl	__get_core_pos
	cmp	r0, #0
	bne 1f

	/* Check if it is a warm boot */
	mov_imm	r9, 0x42008264
	ldr	r8, [r9]
	mov_imm	r9, 0x10000
	tst	r8, r9
	beq	1f

	/* Switch to monitor mode */
	cps	#CPSR_MODE_MON

	/* Set cntvoff to 0 */
	read_scr r0
	orr	r0, r0, #SCR_NS /* Set NS bit in SCR */
	write_scr r0
	isb

	read_id_pfr1 r2
	mov	r3, r2
	ands    r3, r3, #IDPFR1_GENTIMER_MASK
	beq	.no_gentimer
	ands    r2, r2, #IDPFR1_VIRT_MASK
	beq	.no_gentimer
	mov	r2, #0
	write_cntvoff r2, r2

.no_gentimer:
	bic	r0, r0, #SCR_NS /* Clr NS bit in SCR */
	write_scr r0
	isb

	/* Start resume in secure world */
	b sm_pm_cpu_resume
1:
	mov lr, r10
	bx	lr
END_FUNC plat_cpu_reset_early

