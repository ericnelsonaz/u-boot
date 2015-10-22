/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* SPL */
#define CONFIG_SPL_WATCHDOG_SUPPORT

#include "imx6_spl.h"                  /* common IMX6 SPL configuration */

#define CONFIG_SYS_DCACHE_OFF

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART2_BASE
#define CONFIG_BAUDRATE			115200

#define CONFIG_SYS_CBSIZE	       1024
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + 16)
#define CONFIG_SYS_MAXARGS	       48

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR
#define CONFIG_RESET_CAUSE_ADDR	       (PHYS_SDRAM + 0x80)

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_IS_NOWHERE

#define CONFIG_LOADADDR		       0x12000000
#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR
#define CONFIG_SYS_TEXT_BASE	       0x17800000

#undef CONFIG_CMD_IMLS

#endif	       /* __CONFIG_H */
