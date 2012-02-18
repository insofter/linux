/*
 * linux/arch/arm/mach-at91/board-sam9260ek.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>

#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
	//at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
	//		   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
	//		   | ATMEL_UART_RI);

	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	//at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

/*
 * SPI devices.
 */
//static struct spi_board_info ek_spi_devices[] = {
//#if !defined(CONFIG_MMC_AT91)
//	{	/* DataFlash chip */
//		.modalias	= "mtd_dataflash",
//		.chip_select	= 1,
//		.max_speed_hz	= 15 * 1000 * 1000,
//		.bus_num	= 0,
//	},
//#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
//	{	/* DataFlash card */
//		.modalias	= "mtd_dataflash",
//		.chip_select	= 0,
//		.max_speed_hz	= 15 * 1000 * 1000,
//		.bus_num	= 0,
//	},
//#endif
//#endif
//};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA7,
	.is_rmii	= 0,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "bootstrap",
		.offset	= 0,
		.size	= SZ_256K,
	},
	{
		.name	= "u-boot",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_256K,
	},
	{
		.name	= "u-boot environment",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_1M + SZ_512K,
	},
	{
		.name	= "kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_8M - SZ_2M,
	},
	{
		.name	= "filesystems",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 1,
	.wire4		= 1,
//	.det_pin	= ... not connected
//	.wp_pin		= ... not connected
//	.vcc_pin	= ... not connected
};


/*
 * LEDs
 */
static struct gpio_led ek_leds[] = {
	{	/* "power" led, yellow */
		.name			= "usr",
		.gpio			= AT91_PIN_PC15,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	}
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
        {
                .gpio           = AT91_PIN_PB10,
                .code           = BTN_0,
                .desc           = "Button0",
                .active_low     = 1,
                .wakeup         = 1,
        },
        {
                .gpio           = AT91_PIN_PB11,
                .code           = BTN_1,
                .desc           = "Button1",
                .active_low     = 1,
                .wakeup         = 1,
        },
        {
                .gpio           = AT91_PIN_PB12,
                .code           = BTN_2,
                .desc           = "Button2",
                .active_low     = 1,
                .wakeup         = 1,
        },
        {
                .gpio           = AT91_PIN_PB13,
                .code           = BTN_3,
                .desc           = "Button3",
                .active_low     = 1,
                .wakeup         = 1,
        },
        {
                .gpio           = AT91_PIN_PA30,
                .code           = BTN_4,
                .desc           = "Button4",
                .active_low     = 1,
                .wakeup         = 1,
        },
        {
                .gpio           = AT91_PIN_PA31,
                .code           = BTN_5,
                .desc           = "Button5",
                .active_low     = 1,
                .wakeup         = 1,
        }
};

static struct gpio_keys_platform_data ek_button_data = {
        .buttons        = ek_buttons,
        .nbuttons       = ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
        .name           = "gpio-keys",
        .id             = -1,
        .num_resources  = 0,
        .dev            = {
                .platform_data  = &ek_button_data,
        }
};

static void __init ek_add_device_buttons(void)
{
        at91_set_gpio_input(AT91_PIN_PB10, 1);  /* btn0 */
        at91_set_deglitch(AT91_PIN_PB10, 1);
        at91_set_gpio_input(AT91_PIN_PB11, 1);  /* btn1 */
        at91_set_deglitch(AT91_PIN_PB11, 1);
        at91_set_gpio_input(AT91_PIN_PB12, 1);  /* btn2 */
        at91_set_deglitch(AT91_PIN_PB12, 1);
        at91_set_gpio_input(AT91_PIN_PB13, 1);  /* btn3 */
        at91_set_deglitch(AT91_PIN_PB13, 1);
        at91_set_gpio_input(AT91_PIN_PA30, 1);  /* btn4 */
        at91_set_deglitch(AT91_PIN_PA30, 1);
        at91_set_gpio_input(AT91_PIN_PA31, 1);  /* btn5 */
        at91_set_deglitch(AT91_PIN_PA31, 1);

        platform_device_register(&ek_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif


static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
//	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* NAND */
	at91_add_device_nand(&ek_nand_data);
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);
	/* I2C */
	at91_add_device_i2c(NULL, 0);
	/* LEDs */
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	/* Push Buttons */
	ek_add_device_buttons();
}

MACHINE_START(AT91SAM9260EK, "Insofter icdtcp3")
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
