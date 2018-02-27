/*
 * Driver for Fluke Norwich fast uart, for use with Vanquish.
 * Copyright 2018 Fluke Corporation
 * Author: Frank Mori Hess fmh6jj@gmail.com
 * 
 * Based on Synopsys DesignWare 8250 driver.
 *
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <asm/byteorder.h>

#include "8250.h"
#include "ioctl_8250_fast_uart.h"

#define FN_FAST_UART_RX_FIFO_SIZE 0x800
#define FN_FAST_UART_TX_FIFO_SIZE 0x200
#define FN_FAST_BURST_LENGTH 16

/* FAST UART Registers locations */
#define UART_FIFO_LSB          	0x20
#define UART_FIFO_MSB          	0x24
#define UART_WATERMARK_LSB     	0x28
#define UART_WATERMARK_MSB     	0x2C
#define UART_TIMEOUT_LSB		0x30
#define UART_TIMEOUT_MSB		0x34

struct fn_fast_uart_data {
	int			line;
	struct uart_8250_dma	dma;
	struct uart_ops ops;
};

/* Hardware has fixed divisor and doesn't support divisor latch access bit.
 * Since it can't enable writes to uart divisor latch registers, make it a no-op */
static void fn_fast_uart_dl_write(struct uart_8250_port *up, int value)
{
}
static int fn_fast_uart_dl_read(struct uart_8250_port *up)
{
	return 1;
}

static unsigned int read_watermark(struct uart_port *port)
{
	unsigned int watermark_level = 0;
	watermark_level = readl(port->membase + UART_WATERMARK_MSB);
	watermark_level = (watermark_level & 0x1F) << 8;
	watermark_level |= readl(port->membase + UART_WATERMARK_LSB);
	return watermark_level;
}

static unsigned int read_timeout(struct uart_port *port)
{
	unsigned int timeout = 0;
	timeout = readl(port->membase + UART_TIMEOUT_MSB);
	timeout = (timeout & 0xFF) << 8;
	timeout |= readl(port->membase + UART_TIMEOUT_LSB);
	return timeout;
}

#if 0
static int read_fifo_level(struct uart_port *port)
{
	int fifo_level = 0;
	fifo_level = readl(port->membase + UART_FIFO_MSB);
	fifo_level = (fifo_level & 0x1F) << 8;
	fifo_level |= readl(port->membase + UART_FIFO_LSB);
	return fifo_level;
}
#endif

static int write_timeout(struct uart_port *port, int timeout)
{
	if(timeout <= 0)
		return -1;
	writel((timeout & 0xFF00) >> 8, port->membase + UART_TIMEOUT_MSB);
	writel(timeout & 0xFF, port->membase + UART_TIMEOUT_LSB);
	return 0;
}

/* watermark should be at least 1 greater than max burst length, to
prevent a burst transfer from completely emptying the fifo (and thus
disabling rx timeouts). */
static int write_watermark(struct uart_port *port, unsigned long watermark_level)
{
	if(watermark_level > FN_FAST_UART_RX_FIFO_SIZE ||
		watermark_level <= FN_FAST_BURST_LENGTH)
		return -EINVAL;
	writel((watermark_level & 0xFF00) >> 8, port->membase + UART_WATERMARK_MSB);
	writel(watermark_level & 0xFF, port->membase + UART_WATERMARK_LSB);
	return 0;
}

static int fn_fast_uart_ioctl(struct uart_port *port, unsigned int reg, unsigned long value)
{
	switch(reg) {
	case READ_UART_WATERMARK:
		return read_watermark(port);
	case READ_UART_TIMEOUT:
		return read_timeout(port);
	case WRITE_UART_TIMEOUT:
		return write_timeout(port, value);
	default:
		return -ENOIOCTLCMD;
	}
}

static void fn_fast_uart_serial_out(struct uart_port *p, int offset, int value)
{
	writeb(value, p->membase + (offset << p->regshift));
}

static unsigned int fn_fast_uart_serial_in(struct uart_port *p, int offset)
{
	return readb(p->membase + (offset << p->regshift));
}

static bool fn_fast_uart_dma_filter(struct dma_chan *chan, void *param)
{
	return false;
}

static void fn_fast_uart_setup_port(struct uart_8250_port *up)
{
	struct uart_port	*p = &up->port;

	p->type = PORT_16550A;
	p->flags |= UPF_FIXED_TYPE | UPF_SKIP_TEST;
	p->fifosize = FN_FAST_UART_RX_FIFO_SIZE;
	up->tx_loadsz = FN_FAST_UART_TX_FIFO_SIZE;
	up->capabilities = UART_CAP_FIFO;

	up->dl_write = &fn_fast_uart_dl_write;
	up->dl_read = &fn_fast_uart_dl_read;
}

static int fn_fast_uart_probe_of(struct uart_port *p,
			   struct fn_fast_uart_data *data)
{
	struct device_node	*np = p->dev->of_node;
	struct uart_8250_port *up = up_to_u8250p(p);
	u32			val;
	int id;

	fn_fast_uart_setup_port(up);

	/* if we have a valid fifosize, try hooking up DMA here */
	if (p->fifosize) {
		up->dma = &data->dma;
	}

	if (!of_property_read_u32(np, "reg-shift", &val))
		p->regshift = val;

	/* get index of serial line, if found in DT aliases */
	id = of_alias_get_id(np, "serial");
	if (id >= 0)
		p->line = id;

	return 0;
}

static int fn_fast_uart_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irq = platform_get_irq(pdev, 0);
	struct fn_fast_uart_data *data;
	int err;

	if (!regs) {
		dev_err(&pdev->dev, "no registers defined\n");
		return -EINVAL;
	}

	if (irq < 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(&pdev->dev, "cannot get irq\n");
		return irq;
	}

	spin_lock_init(&uart.port.lock);
	uart.port.mapbase = regs->start;
	uart.port.irq = irq;
	uart.port.type = PORT_8250;
	uart.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF | UPF_FIXED_PORT;
	uart.port.dev = &pdev->dev;

	uart.port.membase = devm_ioremap(&pdev->dev, regs->start,
					 resource_size(regs));
	if (!uart.port.membase)
		return -ENOMEM;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Always ask for fixed clock rate from a property. */
	device_property_read_u32(&pdev->dev, "clock-frequency",
				 &uart.port.uartclk);

	/* If no clock rate is defined, fail. */
	if (!uart.port.uartclk) {
		dev_err(&pdev->dev, "clock rate not defined\n");
		return -EINVAL;
	}

	data->dma.rx_param = data;
	data->dma.tx_param = data;
	data->dma.fn = &fn_fast_uart_dma_filter;
	data->dma.rxconf.src_maxburst = FN_FAST_BURST_LENGTH;
	data->dma.rxconf.dst_maxburst = FN_FAST_BURST_LENGTH;
	data->dma.txconf.src_maxburst = 1;
	data->dma.txconf.dst_maxburst = 1;

	
	uart.port.iotype = UPIO_MEM;
	uart.port.serial_in = fn_fast_uart_serial_in;
	uart.port.serial_out = fn_fast_uart_serial_out;
	uart.port.private_data = data;

	if (pdev->dev.of_node) {
		err = fn_fast_uart_probe_of(&uart.port, data);
		if (err)
			goto err_out;
	} else {
		err = -ENODEV;
		goto err_out;
	}

	/* Init watermark.  We set it to the max burst size plus one to avoid
	 completely emptying the fifo with a burst.  This matters because if
	 the fifo is completely empty it suppresses rx fifo timeout interrupts. */
	write_watermark(&uart.port, FN_FAST_BURST_LENGTH + 1);
	/* Init rx timeout to a reasonable 10usec value */
	write_timeout(&uart.port, 600);
	
	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0) {
		err = data->line;
		goto err_out;
	}

	/* Abuse serial8250_get_port in order to set custom ioctl handling,
	  which the serial core currently provides no way to do. */
	data->ops = *serial8250_get_port(data->line)->port.ops;
	data->ops.ioctl = &fn_fast_uart_ioctl;
	serial8250_get_port(data->line)->port.ops = &data->ops;

	platform_set_drvdata(pdev, data);

	return 0;

err_out:

	return err;
}

static int fn_fast_uart_remove(struct platform_device *pdev)
{
	struct fn_fast_uart_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);

	return 0;
}

static const struct of_device_id fn_fast_uart_of_match[] = {
	{ .compatible = "flk,fn-fast-uart" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, fn_fast_uart_of_match);

static struct platform_driver fn_fast_uart_platform_driver = {
	.driver = {
		.name		= "fluke-fn-fast-uart",
		.of_match_table	= fn_fast_uart_of_match,
	},
	.probe			= fn_fast_uart_probe,
	.remove			= fn_fast_uart_remove,
};

module_platform_driver(fn_fast_uart_platform_driver);

MODULE_AUTHOR("Frank Mori Hess");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fluke Norwich fast uart driver");
MODULE_ALIAS("platform:fluke-fn-fast-uart");
