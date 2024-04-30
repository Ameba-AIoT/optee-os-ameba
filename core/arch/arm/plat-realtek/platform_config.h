/* SPDX-License-Identifier: BSD-2-Clause */
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

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

/* Make stacks aligned to data cache line length */
#define STACK_ALIGNMENT		64

#ifdef CFG_WITH_PAGER
#error "Pager not supported for amebasmart"
#endif

#if defined(PLATFORM_FLAVOR_amebasmart_armv8a) || defined(PLATFORM_FLAVOR_amebasmart_armv7a)
#ifdef CFG_ARM_GICV3
#define GIC_BASE		0x02000000
#define GICD_OFFSET		0x00100000
#else
#define GIC_BASE		0xA0100000
#define GICD_OFFSET		0x00001000
#endif

#define GICC_OFFSET		0x00002000	/* gicv3 cannot use it */

#define GICD_BASE		(GIC_BASE + GICD_OFFSET)
#define GICC_BASE		(GIC_BASE + GICC_OFFSET)

#define UART_BASE		0x4200C000
#define UART_CLK_IN_HZ		250000000	//not used
#define UART_BAUDRATE		115200		//not used

#define CONSOLE_UART_BASE	UART_BASE
#define CONSOLE_UART_CLK_IN_HZ	UART_CLK_IN_HZ
#define CONSOLE_BAUDRATE	UART_BAUDRATE

#define DRAM0_BASE		0x60000000
#define DRAM0_SIZE		0x20000000

/* Location of trusted dram */
#define TZDRAM_BASE		CFG_TZDRAM_START
#define TZDRAM_SIZE		CFG_TZDRAM_SIZE

/* please modify share mem address if you want to use it */
#define TEE_SHMEM_START		CFG_SHMEM_START
#define TEE_SHMEM_SIZE		CFG_SHMEM_SIZE

#define PLAT_AMEBASMART_TRUSTED_MAILBOX_BASE	0x701B0000
#define PLAT_AMEBASMART_TRUSTED_MAILBOX_SIZE	(0x100)
#else
#error "Unknown platform flavor"
#endif
/*
 * Assumes that either TZSRAM isn't large enough or TZSRAM doesn't exist,
 * everything is in TZDRAM.
 * +------------------+
 * |        | TEE_RAM |
 * + TZDRAM +---------+
 * |        | TA_RAM  |
 * +--------+---------+
 */
#define TEE_RAM_START		TZDRAM_BASE
#define TEE_RAM_PH_SIZE		TEE_RAM_VA_SIZE
#define TEE_RAM_VA_SIZE		CFG_TEE_RAM_VA_SIZE
#define TEE_LOAD_ADDR		(TZDRAM_BASE)
#define TA_RAM_START		(TZDRAM_BASE + TEE_RAM_VA_SIZE)
#define TA_RAM_SIZE		(TZDRAM_SIZE - TEE_RAM_VA_SIZE)

#define DEVICE0_PA_BASE		ROUNDDOWN(CONSOLE_UART_BASE, \
					  CORE_MMU_PGDIR_SIZE)
#define DEVICE0_VA_BASE		DEVICE0_PA_BASE
#define DEVICE0_SIZE		CORE_MMU_PGDIR_SIZE
#define DEVICE0_TYPE		MEM_AREA_IO_SEC

#define DEVICE1_PA_BASE		ROUNDDOWN(GIC_BASE + GICD_OFFSET, \
					CORE_MMU_PGDIR_SIZE)
#define DEVICE1_VA_BASE		DEVICE1_PA_BASE
#define DEVICE1_SIZE		CORE_MMU_PGDIR_SIZE
#define DEVICE1_TYPE		MEM_AREA_IO_SEC

#define DEVICE2_PA_BASE		ROUNDDOWN(GIC_BASE + GICC_OFFSET, \
					  CORE_MMU_PGDIR_SIZE)
#define DEVICE2_VA_BASE		DEVICE2_PA_BASE
#define DEVICE2_SIZE		CORE_MMU_PGDIR_SIZE
#define DEVICE2_TYPE		MEM_AREA_IO_SEC

#endif /*PLATFORM_CONFIG_H*/
