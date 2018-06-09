/*
 * Copyright (C) 2018 Device Solutions Ltd
 *
 * based on sabresd.c which is :
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * and on hummingboard.c which is :
 * Copyright (C) 2013 SolidRun ltd.
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <splash.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <netdev.h>
#include <usb/ehci-ci.h>
#include <power/da9063.h>


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CLK_CTRL (PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST |			\
	PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_MDIO_PAD_CTRL  (PAD_CTL_PUS_22K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_34ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define STRONG_PULLUP	(PAD_CTL_PUS_22K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)


#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

static int board_type = -1;
#define BOARD_IS_OPAL6S		0
#define BOARD_IS_OPAL6DL	1
#define BOARD_IS_OPAL6D		2
#define BOARD_IS_OPAL6Q		3


struct splash_location splash_locations[] = {
	{
		.name = "mmc_fs",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "1:1",
	},
};

int splash_screen_prepare(void)
{
	return splash_source_load(splash_locations,
				  ARRAY_SIZE(splash_locations));
}

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_MDIO_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_MDIO_PAD_CTRL),
	/* GPIO16 -> AR8035 25MHz */
	/* MX6_PAD_GPIO_16__ENET_REF_CLK | MUX_PAD_CTRL(NO_PAD_CTRL), *** not sure what this does */
	MX6_PAD_RGMII_TXC__RGMII_TXC | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8035 CLK_25M --> ENET_REF_CLK (V22) */
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL_CLK),
	MX6_PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
	MX6_PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
	MX6_PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
	/* Micrel PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
	/* Micrel PHY Interrupt */
	MX6_PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset Micrel PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}

int mx6_rgmii_rework(struct phy_device *phydev)
{
	/* from linux/arch/arm/mach-imx/mach-imx6q.c :
	 * Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 *
	 * Not required for Micrel??
	 */
/*
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x805d);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4003);
*/
	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WP */
	MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK | MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* eMMC RST */
	MX6_PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)


int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SDCard slot
	 * mmc1                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[1].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}
#endif

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_EB2__I2C2_SCL
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_EB2__GPIO2_IO30
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(2, 30)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D16__I2C2_SDA
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D16__GPIO3_IO16
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 16)
	}
};

struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18
				| MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 18)
	}
};

iomux_v3_cfg_t const lvds_pads[] = {
	/* LCD_PWR_EN */
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* TOUCH_INT */
	MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* LED_PWR_EN -- not used on Opal6 */
	/*MX6_PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),*/
	/* BL LEVEL */
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* add pads for ATDM board */
	/* GPIO1_01 and GPIO7_13 */
	MX6_PAD_GPIO_1__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


#if defined(CONFIG_VIDEO_IPUV3)

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
	/* set backlight level to ON */
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 1);
}

static void enable_lvds_24(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	/* set backlight level to ON */
	gpio_direction_output(IMX_GPIO_NR(7, 13) , 1);
}

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* set backlight level to OFF */
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 0);

	clrbits_le32(&iomux->gpr[2], IOMUXC_GPR2_LVDS_CH0_MODE_MASK);
}

#ifdef CONFIG_IMX_HDMI
static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}
#endif

static int detect_i2c(struct display_info_t const *dev)
{
	return (0 == i2c_set_bus_num(dev->bus)) &&
		(0 == i2c_probe(dev->addr));
}

struct display_info_t const displays[] = {	{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,  
	.detect	= NULL,
	.enable	= enable_lvds_24,
	.mode	= {
		.name           = "HSD100PXN1",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
	}},
	{
	.bus	= 1,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }
#ifdef CONFIG_IMX_HDMI
, {
	.bus	= 3,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }
#endif
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_HIGH
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
		   | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	       | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */


static iomux_v3_cfg_t const init_pads[] = {
	/* SGTL5000 sys_mclk */
	NEW_PAD_CTRL(MX6_PAD_GPIO_0__CCM_CLKO1, OUTPUT_40OHM),

	/* Camera MCLK */
	NEW_PAD_CTRL(MX6_PAD_GPIO_3__CCM_CLKO2, OUTPUT_40OHM),

	/* MIPI CSI RESET */
	NEW_PAD_CTRL(MX6_PAD_EIM_EB0__GPIO2_IO28, OUTPUT_40OHM),

	/* MIPI CSI PWRON */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA2__GPIO3_IO02, OUTPUT_40OHM),

	/* LEDs */
	NEW_PAD_CTRL(MX6_PAD_EIM_DA5__GPIO3_IO05, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_EIM_DA1__GPIO3_IO01, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_EIM_DA4__GPIO3_IO04, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_EIM_DA14__GPIO3_IO14, OUTPUT_40OHM),

	/* Outputs */
	NEW_PAD_CTRL(MX6_PAD_NANDF_D0__GPIO2_IO00, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_D1__GPIO2_IO01, OUTPUT_40OHM),

	/* PCIe */
	NEW_PAD_CTRL(MX6_PAD_KEY_ROW2__GPIO4_IO11, OUTPUT_40OHM),
	NEW_PAD_CTRL(MX6_PAD_NANDF_RB0__GPIO6_IO10, OUTPUT_40OHM),

	/* Spare GPIOs on expansion */
	NEW_PAD_CTRL(MX6_PAD_GPIO_6__GPIO1_IO06, OUTPUT_40OHM),

	/* Inputs */
	NEW_PAD_CTRL(MX6_PAD_NANDF_D2__GPIO2_IO02, STRONG_PULLUP),
	NEW_PAD_CTRL(MX6_PAD_NANDF_D3__GPIO2_IO03, STRONG_PULLUP),

	/* Buttons */
	NEW_PAD_CTRL(MX6_PAD_EIM_D31__GPIO3_IO31, STRONG_PULLUP),
	NEW_PAD_CTRL(MX6_PAD_EIM_D30__GPIO3_IO30, STRONG_PULLUP),
	NEW_PAD_CTRL(MX6_PAD_KEY_ROW3__GPIO4_IO13, STRONG_PULLUP),


};


static unsigned gpios_out_low[] = {
	IMX_GPIO_NR(2, 28),	/* MIPI CSI RESET  */
	IMX_GPIO_NR(3, 5),	/* LED G1 */
	IMX_GPIO_NR(3, 1),	/* LED G2 */
	IMX_GPIO_NR(3, 4),	/* LED R1 */
	IMX_GPIO_NR(3, 14),	/* LED R2 */
	IMX_GPIO_NR(4, 11),	/* PCIE_RST_B */
	IMX_GPIO_NR(6, 10),	/* PCIE_PWR_EN */
	IMX_GPIO_NR(2, 0),	/* OUTPUT 1 */
	IMX_GPIO_NR(2, 1),	/* OUTPUT 2 */
};

static unsigned gpios_out_high[] = {
	IMX_GPIO_NR(3, 2),	/* MIPI CSI PWRON */
	IMX_GPIO_NR(1, 6),	/* MIPI CSI PWRON */
};

static void set_gpios(unsigned *p, int cnt, int val)
{
	int i;

	for (i = 0; i < cnt; i++)
		gpio_direction_output(*p++, val);
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}



int board_early_init_f(void)
{
	u32 cputype = cpu_type(get_cpu_rev());

	switch (cputype) {
	case MXC_CPU_MX6SOLO:
		board_type = BOARD_IS_OPAL6S;
		break;
	case MXC_CPU_MX6DL:
		board_type = BOARD_IS_OPAL6DL;
		break;
	case MXC_CPU_MX6D:
		board_type = BOARD_IS_OPAL6D;
		break;
	case MXC_CPU_MX6Q:
		board_type = BOARD_IS_OPAL6Q;
		break;
	}

	setup_iomux_uart();
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	imx_iomux_v3_setup_multiple_pads(lvds_pads, ARRAY_SIZE(lvds_pads));
#if defined(CONFIG_VIDEO_IPUV3)
	/* power ON LCD */
	gpio_direction_output(IMX_GPIO_NR(2, 29) , 1);
	/* touch interrupt is an input */
	gpio_direction_input(IMX_GPIO_NR(3, 21));
	/* set backlight level to off */
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 0);
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);
	gpio_direction_output(IMX_GPIO_NR(7, 13) , 1);
	setup_display();
#endif

	return 0;
}





int setup_pmic(void)
{
	int result = 0;

	for(;;)
	{
		/* LDO2 - 1.2V */
		if( da9063_set_register( DA9063_VLDO2_A_ADDR, 0x1E, 0) )
		{
			puts("Failed to configure LDO2 voltage A to 1.2V.\n");
			break;
		}

		if( da9063_set_register( DA9063_VLDO2_B_ADDR, 0x1E, 0) )
		{
			puts("Failed to configure LDO2 voltage B to 1.2V.\n");
			break;
		}

		if( da9063_set_register( DA9063_VLDO2_CONT_ADDR, 0x01, 0x01) )
		{
			puts("Failed to enable LDO2.\n");
			break;
		}
		puts("LDO2 voltage set to 1.2V.\n");

		/* LDO3 - 1.2V */
		if( da9063_set_register( DA9063_VLDO3_A_ADDR, 0x1E, 0) )
		{
			puts("Failed to configure LDO3 voltage A to 1.2V.\n");
			break;
		}

		if( da9063_set_register( DA9063_VLDO3_B_ADDR, 0x1E, 0) )
		{
			puts("Failed to configure LDO3 voltage B to 1.2V.\n");
			break;
		}

		if( da9063_set_register( DA9063_VLDO3_CONT_ADDR, 0x01, 0x01) )
		{
			puts("Failed to enable LDO3.\n");
			break;
		}
		puts("LDO3 voltage set to 1.2V.\n");

		result = 1;
		break;
	}

	return result;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	/* i2c1 : PMIC, Audio codec on DevKit, RTC, Expansion on DevKit */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	/* i2c2 : MIPI CSI, LVDS1 TOUCH, LVDS0 EDID */
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	/* i2c3 : CSI0, LVDS1 EDID, LVDO0 TOUCH */
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
	/* i2c4 : HDMI EDID, Expansion on DevKit */
/*	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info4); */
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

	/* Check for DA9063 and power on Ethernet - LDO2 and 3 to 1.2V */
	if( da9063_detect() == 1 )
	{
		puts("DA9063 detected - setting up 1.2V rails for Ethernet\n");
		setup_pmic();
	}
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode opal6_boot_modes[] = {
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(opal6_boot_modes);
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: ");
	if (board_type == BOARD_IS_OPAL6Q)
		puts("Opal6Q\n");
	else if (board_type == BOARD_IS_OPAL6DL)
		puts("Opal6DL\n");
	else if (board_type == BOARD_IS_OPAL6S)
		puts("Opal6S\n");
	else
		printf("unknown - cputype : %02x\n", cpu_type(get_cpu_rev()));

	return 0;
}
