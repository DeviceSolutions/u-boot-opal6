/*
 * Copyright (C) 2015 Device Solutions Ltd
 * Author: Martin Welford <martin@device.solutions>
 *
 * Configuration settings for the Device Solutions Opal-6 board
 *
 * based on mx6*sabre*.h which are :
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __OPAL6_CONFIG_H
#define __OPAL6_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#include "mx6_common.h"

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_SYS_GENERIC_BOARD

#define CONFIG_MXC_UART_BASE		UART4_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc1"
#define CONFIG_MMCROOT			"/dev/mmcblk1p1"
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)


#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_EDID

/* RTC */
#define CONFIG_CMD_DATE
#define CONFIG_RTC_M41T62
#define CONFIG_SYS_I2C_RTC_ADDR  0x68


/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0

#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9031

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               1

#define CONFIG_LOADADDR                0x10008000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#ifdef CONFIG_SUPPORT_EMMC_BOOT
#define EMMC_ENV \
	"emmcdev=2\0" \
	"update_emmc_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${update_sd_firmware_filename}; then " \
			"if mmc dev ${emmcdev}; then "	\
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0"
#else
#define EMMC_ENV ""
#endif

#ifdef CONFIG_CMD_SF
#define SF_ENV \
	"update_spi_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${update_spi_firmware_filename}; then " \
			"if sf probe; then "	\
				"sf erase 0 0xc0000; " \
				"sf write ${loadaddr} 0x400 ${filesize}; " \
			"fi; "	\
		"fi\0"
#else
#define SF_ENV ""
#endif

#ifdef CONFIG_ENV_IS_NOWHERE  /* manufacturing only! */
#define CONFIG_EXTRA_ENV_SETTINGS \
    "video=mxcfb0:dev=ldb,LDB-XGA,if=RGB666 video=mxcfb1:dev=hdmi,1920x1080M@60,if=RGB24 video=mxcfb2:off video=mxcfb3:off ldb=sep0\0" \
    "mmcargs=setenv bootargs enable_wait_mode=off ${video} console=ttymxc3,115200 consoleblank=0 vmalloc=400M fbmem=28M rootfstype=ramfs rdinit=/sbin/init\0"

#define CONFIG_BOOTCOMMAND \
    "run mmcargs; " \
    "mmc dev 0; " \
    "fatload mmc 0 0x12000000 imx6dl-opaldk.dtb; " \
    "fatload mmc 0 0x10008000 uImage-factory; " \
    "bootm 0x10008000 - 0x12000000"
#else  /* release settings */
#define CONFIG_EXTRA_ENV_SETTINGS \
    "lvds0=setenv video mxcfb0:dev=ldb,LDB-XGA,if=RGB666 video=mxcfb1:dev=hdmi,1920x1080M@60,if=RGB24 video=mxcfb2:off video=mxcfb3:off ldb=sep0\0" \
    "lvds1=setenv video mxcfb0:dev=ldb,LDB-XGA,if=RGB666 video=mxcfb1:dev=hdmi,1920x1080M@60,if=RGB24 video=mxcfb2:off video=mxcfb3:off ldb=sep1\0" \
    "hdmi=setenv video video=mxcfb0:dev=hdmi,1920x1080M@60,if=RGB24  video=mxcfb1:off video=mxcfb2:off video=mxcfb3:off ldb=sep0\0" \
    "video=mxcfb0:dev=ldb,LDB-XGA,if=RGB666 video=mxcfb1:dev=hdmi,1920x1080M@60,if=RGB24 video=mxcfb2:off video=mxcfb3:off ldb=sep0\0" \
    "mmcargs=setenv bootargs enable_wait_mode=off ${video} console=ttymxc3,115200 consoleblank=0 vmalloc=400M fbmem=28M root=/dev/mmcblk2p2\0" \
    "update_uboot_from_emmc=" \
        "if fatload mmc 1 ${loadaddr} u-boot.imx; then " \
            "setexpr fw_sz ${filesize} / 0x200; " \
            "setexpr fw_sz ${fw_sz} + 1; "	\
            "mmc dev 1:0; " \
            "mmc enable-boot-write 1; " \
            "mmc write ${loadaddr} 0x2 ${fw_sz}; " \
        "fi\0" \
    "update_uboot_from_sd=" \
        "if fatload mmc 0 ${loadaddr} u-boot.imx; then " \
            "setexpr fw_sz ${filesize} / 0x200; " \
            "setexpr fw_sz ${fw_sz} + 1; "	\
            "mmc dev 1:0; " \
            "mmc enable-boot-write 1; " \
            "mmc write ${loadaddr} 0x2 ${fw_sz}; " \
        "fi\0"

#define CONFIG_BOOTCOMMAND \
    "run mmcargs; " \
    "mmc dev 1; " \
    "fatload mmc 1 0x12000000 imx6dl-opaldk.dtb; " \
    "fatload mmc 1 0x10008000 uImage; " \
    "bootm 0x10008000 - 0x12000000"
#endif


#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "Opal-6 > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             48
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(64 * 1024)

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_DEFAULT_FDT_FILE	"imx6dl-opal-6.dtb"

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC4 */
#define CONFIG_ENV_OFFSET		(1 * 1024 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/* USB Mass Storage Gadget */
#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2

/* Netchip IDs */
#define CONFIG_G_DNL_VENDOR_NUM 0x0525
#define CONFIG_G_DNL_PRODUCT_NUM 0xa4a5
#define CONFIG_G_DNL_MANUFACTURER "Device Solutions"

/* PCI express */
/*
#define CONFIG_CMD_PCI
#define CONFIG_CMD_PCI_ENUM
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(4, 11)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(6, 10)
#endif
*/

/* Framebuffer */
/*
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
*/
/*#define CONFIG_IMX_HDMI
#define CONFIG_CMD_HDMIDETECT*/

/*
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_CMD_BMP
*/
#endif                         /* __OPAL6_CONFIG_H */
