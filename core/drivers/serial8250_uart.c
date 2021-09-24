// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2015, Linaro Limited
 */

#include <compiler.h>
#include <console.h>
#include <drivers/serial8250_uart.h>
#include <io.h>
#include <keep.h>
#include <util.h>
#include <kernel/dt.h>

/* uart register defines */
#define LOGUART_UART_LSR	0x14
#define LOGUART_UART_RBR	0x24
#define LOGUART_UART_THR4	0x68

#define LOGUART_BIT_DRDY			(1<<0) /* DATA Ready */
#define LOGUART_BIT_TP4F_EMPTY		(1<<19)


static vaddr_t chip_to_base(struct serial_chip *chip)
{
	struct serial8250_uart_data *pd =
		container_of(chip, struct serial8250_uart_data, chip);

	return io_pa_or_va(&pd->base);
}

static void serial8250_uart_flush(struct serial_chip *chip)
{
	vaddr_t base = chip_to_base(chip);

	while (1) {
		uint32_t state = io_read32(base + LOGUART_UART_LSR);

		/* Wait until transmit FIFO is empty */
		if ((state & LOGUART_BIT_TP4F_EMPTY) == LOGUART_BIT_TP4F_EMPTY)
			break;
	}
}

static bool serial8250_uart_have_rx_data(struct serial_chip *chip)
{
	vaddr_t base = chip_to_base(chip);

	return (io_read32(base + LOGUART_UART_LSR) & LOGUART_BIT_DRDY);
}

static int serial8250_uart_getchar(struct serial_chip *chip)
{
	vaddr_t base = chip_to_base(chip);

	while (!serial8250_uart_have_rx_data(chip)) {
		/* Transmit FIFO is empty, waiting again */
		;
	}
	return io_read32(base + LOGUART_UART_RBR) & 0xff;
}

static void serial8250_uart_putc(struct serial_chip *chip, int ch)
{
	vaddr_t base = chip_to_base(chip);

	serial8250_uart_flush(chip);

	/* Write out character to transmit FIFO */
	io_write32(base + LOGUART_UART_THR4, ch);
}

static const struct serial_ops serial8250_uart_ops = {
	.flush = serial8250_uart_flush,
	.getchar = serial8250_uart_getchar,
	.have_rx_data = serial8250_uart_have_rx_data,
	.putc = serial8250_uart_putc,
};
DECLARE_KEEP_PAGER(serial8250_uart_ops);

void serial8250_uart_init(struct serial8250_uart_data *pd, paddr_t base,
			  uint32_t __unused uart_clk,
			  uint32_t __unused baud_rate)

{
	pd->base.pa = base;
	pd->chip.ops = &serial8250_uart_ops;

	/*
	 * do nothing, debug uart(uart0) share with normal world,
	 * everything for uart0 is ready now.
	 */
}

#ifdef CFG_DT

static struct serial_chip *serial8250_uart_dev_alloc(void)
{
	struct serial8250_uart_data *pd = calloc(1, sizeof(*pd));

	if (!pd)
		return NULL;
	return &pd->chip;
}

static int serial8250_uart_dev_init(struct serial_chip *chip,
			       const void *fdt,
			       int offs,
			       const char *parms)
{
	struct serial8250_uart_data *pd =
		container_of(chip, struct serial8250_uart_data, chip);
	vaddr_t vbase;
	paddr_t pbase;
	size_t size;

	if (parms && parms[0])
		IMSG("serial8250_uart: device parameters ignored (%s)", parms);

	if (dt_map_dev(fdt, offs, &vbase, &size) < 0)
		return -1;

	if (size < SERIAL8250_UART_REG_SIZE) {
		EMSG("serial8250_uart: register size too small: %zx", size);
		return -1;
	}

	pbase = virt_to_phys((void *)vbase);
	serial8250_uart_init(pd, pbase, 0, 0);

	return 0;
}

static void serial8250_uart_dev_free(struct serial_chip *chip)
{
	struct serial8250_uart_data *pd =
	  container_of(chip,  struct serial8250_uart_data, chip);

	free(pd);
}

static const struct serial_driver serial8250_driver = {
	.dev_alloc = serial8250_uart_dev_alloc,
	.dev_init = serial8250_uart_dev_init,
	.dev_free = serial8250_uart_dev_free,
};

static const struct dt_device_match serial8250_match_table[] = {
	{ .compatible = "snps,dw-apb-uart" },  //TODO: "realsil, ameba_serial"
	{ 0 }
};

const struct dt_driver serial8250_dt_driver __dt_driver = {
	.name = "serial8250_uart",
	.match_table = serial8250_match_table,
	.driver = &serial8250_driver,
};

#endif /* CFG_DT */
