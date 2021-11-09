// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2021, Realtek Semiconductor Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <console.h>
#include <drivers/serial8250_uart.h>
#include <platform_config.h>
#include <stdint.h>
#include <string.h>
#include <drivers/gic.h>
#include <arm.h>
#include <initcall.h>
#include <keep.h>
#include <kernel/boot.h>
#include <kernel/interrupt.h>
#include <kernel/misc.h>
#include <kernel/tee_time.h>
#include <kernel/panic.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <sm/optee_smc.h>
#include <sm/psci.h>
#include <stdint.h>
#include <string.h>
#include <trace.h>

static struct serial8250_uart_data console_data;

static struct gic_data gic_data;

register_phys_mem_pgdir(MEM_AREA_IO_SEC,
		  CONSOLE_UART_BASE, SERIAL8250_UART_REG_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC, GICD_BASE, GIC_DIST_REG_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC, GICC_BASE, GIC_DIST_REG_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC,
	PLAT_SHEIPA_TRUSTED_MAILBOX_BASE, PLAT_SHEIPA_TRUSTED_MAILBOX_SIZE);

void main_init_gic(void)
{
	vaddr_t gicc_base, gicd_base;

	gicc_base = (vaddr_t)phys_to_virt(GICC_BASE,
					  MEM_AREA_IO_SEC);
	gicd_base = (vaddr_t)phys_to_virt(GICD_BASE,
					  MEM_AREA_IO_SEC);

	if (!gicc_base || !gicd_base)
		panic();

#if defined(CFG_WITH_ARM_TRUSTED_FW)
	/* On ARMv8, GIC configuration is initialized in ARM-TF */
	gic_init_base_addr(&gic_data, gicc_base, gicd_base);
#else
	/* Initialize GIC */
	gic_init(&gic_data, gicc_base, gicd_base);
#endif
	itr_init(&gic_data.chip);
}

#if !defined(CFG_WITH_ARM_TRUSTED_FW)
void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
}
#endif

void itr_core_handler(void)
{
	gic_it_handle(&gic_data);
}

void console_init(void)
{
	serial8250_uart_init(&console_data, CONSOLE_UART_BASE,
			     CONSOLE_UART_CLK_IN_HZ, CONSOLE_BAUDRATE);
	register_serial_console(&console_data.chip);
}

static void release_secondary_early_hpen(size_t pos)
{
	struct mailbox {
		uint64_t ep;
		uint64_t hpen[];
	} *mailbox;

	if (cpu_mmu_enabled())
		mailbox = phys_to_virt(PLAT_SHEIPA_TRUSTED_MAILBOX_BASE, MEM_AREA_IO_SEC);
	else
		mailbox = (void *)PLAT_SHEIPA_TRUSTED_MAILBOX_BASE;

	if (!mailbox)
		panic();

	mailbox->ep = TEE_LOAD_ADDR;
	dsb_ishst();
	mailbox->hpen[pos] = 1;
	dsb_ishst();
	sev();
}

int psci_cpu_on(uint32_t core_id, uint32_t entry, uint32_t context_id)
{
	size_t pos = get_core_pos_mpidr(core_id);
	static bool core_is_released[CFG_TEE_CORE_NB_CORE];

	if (!pos || pos >= CFG_TEE_CORE_NB_CORE)
		return PSCI_RET_INVALID_PARAMETERS;

	DMSG("core pos: %zu: ns_entry %#" PRIx32, pos, entry);

	if (core_is_released[pos]) {
		EMSG("core %zu already released", pos);
		return PSCI_RET_DENIED;
	}
	core_is_released[pos] = true;

	boot_set_core_ns_entry(pos, entry, context_id);
	release_secondary_early_hpen(pos);

	return PSCI_RET_SUCCESS;
}
