/*
 * Allwinner sunxi AHCI SATA platform driver
 * Copyright 2013 Olliver Schinagl <oliver@schinagl.nl>
 *
 * Based on the AHCI SATA platform driver by Freescale and Allwinner
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Daniel Wang <danielwang@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <linux/ahci_platform.h>
#include "ahci.h"

#define DRV_NAME "sunxi-sata"

#define AHCI_BISTAFR 0x00a0
#define AHCI_BISTCR 0x00a4
#define AHCI_BISTFCTR 0x00a8
#define AHCI_BISTSR 0x00ac
#define AHCI_BISTDECR 0x00b0
#define AHCI_DIAGNR0 0x00b4
#define AHCI_DIAGNR1 0x00b8
#define AHCI_OOBR 0x00bc
#define AHCI_PHYCS0R 0x00c0
#define AHCI_PHYCS1R 0x00c4
#define AHCI_PHYCS2R 0x00c8
#define AHCI_TIMER1MS 0x00e0
#define AHCI_GPARAM1R 0x00e8
#define AHCI_GPARAM2R 0x00ec
#define AHCI_PPARAMR 0x00f0
#define AHCI_TESTR 0x00f4
#define AHCI_VERSIONR 0x00f8
#define AHCI_IDR 0x00fc
#define AHCI_RWCR 0x00fc
#define AHCI_P0DMACR 0x0170
#define AHCI_P0PHYCR 0x0178
#define AHCI_P0PHYSR 0x017c

struct sunxi_ahci_data {
	struct platform_device *ahci_pdev;
	struct regulator *regulator;
	struct clk *sata_clk;
	struct clk *ahb_clk;
};

static void sunxi_clrbits(void __iomem *reg, u32 clr_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	writel(reg_val, reg);
}

static void sunxi_setbits(void __iomem *reg, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static void sunxi_clrsetbits(void __iomem *reg, u32 clr_val, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static u32 sunxi_getbits(void __iomem *reg, u8 mask, u8 shift)
{
	return (readl(reg) >> shift) & mask;
}

static int sunxi_ahci_phy_init(struct device *dev, void __iomem *reg_base)
{
	u32 reg_val;
	int timeout;

	/* This magic is from the original code */
	writel(0, reg_base + AHCI_RWCR);
	mdelay(5);

	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 24),
			 (0x5 << 24) | BIT(23) | BIT(18));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS1R,
			 (0x3 << 16) | (0x1f << 8) | (0x3 << 6),
			 (0x2 << 16) | (0x6 << 8) | (0x2 << 6));
	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(28) | BIT(15));
	sunxi_clrbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 20), (0x3 << 20));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS2R,
			 (0x1f << 5), (0x19 << 5));
	mdelay(5);

	sunxi_setbits(reg_base + AHCI_PHYCS0R, (0x1 << 19));

	timeout = 0x100000;
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS0R, 0x7, 28);
	} while (--timeout && (reg_val != 0x2));
	if (!timeout)
		dev_err(dev, "PHY power up failed.\n");

	sunxi_setbits(reg_base + AHCI_PHYCS2R, (0x1 << 24));

	timeout = 0x100000;
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS2R, 0x1, 24);
	} while (--timeout && reg_val);
	if (!timeout)
		dev_err(dev, "PHY calibration failed.\n");
	mdelay(15);

	writel(0x7, reg_base + AHCI_RWCR);

	return 0;
}

static int sunxi_ahci_init(struct device *dev, void __iomem *reg_base)
{
	struct sunxi_ahci_data *ahci_data;
	int ret;

	ahci_data = dev_get_drvdata(dev->parent);

	ret = clk_prepare_enable(ahci_data->sata_clk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(ahci_data->ahb_clk);
	if (ret < 0)
		return ret;

	ret = regulator_enable(ahci_data->regulator);
	if (ret)
		return ret;

	return sunxi_ahci_phy_init(dev, reg_base);
}

static void sunxi_ahci_exit(struct device *dev)
{
	struct sunxi_ahci_data *ahci_data;

	ahci_data = dev_get_drvdata(dev->parent);

	regulator_disable(ahci_data->regulator);

	clk_disable_unprepare(ahci_data->ahb_clk);
	clk_disable_unprepare(ahci_data->sata_clk);
}

static struct ahci_platform_data sunxi_ahci_pdata = {
	.init = sunxi_ahci_init,
	.exit = sunxi_ahci_exit,
};

static int sunxi_ahci_remove(struct platform_device *pdev)
{
	struct sunxi_ahci_data *ahci_data;

	ahci_data = platform_get_drvdata(pdev);
	platform_device_unregister(ahci_data->ahci_pdev);

	dev_dbg(&pdev->dev, "driver unloaded\n");

	return 0;
}

static const struct of_device_id sunxi_ahci_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-ahci", .data = &sunxi_ahci_pdata},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, sunxi_ahci_of_match);

static int sunxi_ahci_probe(struct platform_device *pdev)
{
	const struct ahci_platform_data *pdata;
	const struct of_device_id *of_dev_id;
	struct resource *mem, *irq, res[2];
	struct platform_device *ahci_pdev;
	struct sunxi_ahci_data *ahci_data;
	struct regulator *regulator;
	int ret;

	regulator = devm_regulator_get(&pdev->dev, "pwr");
	if (IS_ERR(regulator)) {
		ret = PTR_ERR(regulator);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "no regulator found (%d)\n", ret);
		return ret;
	}

	ahci_data = devm_kzalloc(&pdev->dev, sizeof(*ahci_data), GFP_KERNEL);
	if (!ahci_data)
		return -ENOMEM;

	ahci_pdev = platform_device_alloc("sunxi-ahci", -1);
	if (!ahci_pdev)
		return -ENODEV;

	ahci_pdev->dev.parent = &pdev->dev;

	ahci_data->regulator = regulator;
	ahci_data->ahb_clk = devm_clk_get(&pdev->dev, "ahb_sata");
	if (IS_ERR(ahci_data->ahb_clk)) {
		ret = PTR_ERR(ahci_data->ahb_clk);
		goto err_out;
	}

	ahci_data->sata_clk = devm_clk_get(&pdev->dev, "pll6_sata");
	if (IS_ERR(ahci_data->sata_clk)) {
		ret = PTR_ERR(ahci_data->sata_clk);
		goto err_out;
	}

	ahci_data->ahci_pdev = ahci_pdev;
	platform_set_drvdata(pdev, ahci_data);

	ahci_pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	ahci_pdev->dev.dma_mask = &ahci_pdev->dev.coherent_dma_mask;
	ahci_pdev->dev.of_node = pdev->dev.of_node;

	of_dev_id = of_match_device(sunxi_ahci_of_match, &pdev->dev);
	if (of_dev_id) {
		pdata = of_dev_id->data;
	} else {
		ret = -EINVAL;
		goto err_out;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!mem || !irq) {
		ret = -ENOMEM;
		goto err_out;
	}
	res[0] = *mem;
	res[1] = *irq;
	ret = platform_device_add_resources(ahci_pdev, res, 2);
	if (ret)
		goto err_out;

	ret = platform_device_add_data(ahci_pdev, pdata, sizeof(*pdata));
	if (ret)
		goto err_out;

	ret = platform_device_add(ahci_pdev);
	if (ret)
		goto err_out;

	return 0;

err_out:
	platform_device_put(ahci_pdev);
	return ret;
}

static struct platform_driver sunxi_ahci_driver = {
	.probe = sunxi_ahci_probe,
	.remove = sunxi_ahci_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sunxi_ahci_of_match,
	},
};
module_platform_driver(sunxi_ahci_driver);

MODULE_DESCRIPTION("Allwinner sunxi AHCI SATA platform driver");
MODULE_AUTHOR("Olliver Schinagl <oliver@schinagl.nl>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ahci:sunxi");
