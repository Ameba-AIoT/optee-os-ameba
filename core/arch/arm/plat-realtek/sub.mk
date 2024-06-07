global-incdirs-y += .
srcs-y += main.c
srcs-y += plat_init.S
ifeq (${CONFIG_SOC_CPU_ARMv8_2},y)
srcs-$(CFG_ARM32_core) += amebasmart_core_pos_v8_2.S
else
srcs-$(CFG_ARM32_core) += amebasmart_core_pos_v8.S
endif
