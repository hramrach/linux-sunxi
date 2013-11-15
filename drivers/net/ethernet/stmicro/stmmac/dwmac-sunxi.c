/**
 * dwmac-sunxi.c - Allwinner sunxi DWMAC specific glue layer
 *
 * Copyright (C) 2013 Chen-Yu Tsai
 *
 * Chen-Yu Tsai  <wens@csie.org>
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
 */

#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/stmmac.h>

#define GMAC_IF_TYPE_RGMII	0x4

#define GMAC_TX_CLK_MASK	0x3
#define GMAC_TX_CLK_MII		0x0
#define GMAC_TX_CLK_RGMII_INT	0x2

static int sun7i_gmac_init(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	void __iomem *addr = NULL;
	struct plat_stmmacenet_data *plat_dat = NULL;
	u32 priv_clk_reg;

	plat_dat = dev_get_platdata(&pdev->dev);
	if (!plat_dat)
		return -EINVAL;

	/* Get GMAC clock register in CCU */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	priv_clk_reg = readl(addr);

	/* Set GMAC interface port mode */
	if (plat_dat->interface == PHY_INTERFACE_MODE_RGMII)
		priv_clk_reg |= GMAC_IF_TYPE_RGMII;
	else
		priv_clk_reg &= ~GMAC_IF_TYPE_RGMII;

	/* Set GMAC transmit clock source. */
	priv_clk_reg &= ~GMAC_TX_CLK_MASK;
	if (plat_dat->interface == PHY_INTERFACE_MODE_RGMII
			|| plat_dat->interface == PHY_INTERFACE_MODE_GMII)
		priv_clk_reg |= GMAC_TX_CLK_RGMII_INT;
	else
		priv_clk_reg |= GMAC_TX_CLK_MII;

	writel(priv_clk_reg, addr);

	/* mask out phy addr 0x0 */
	plat_dat->mdio_bus_data->phy_mask = 0x1;

	return 0;
}

const struct plat_stmmacenet_data sun7i_gmac_data = {
	.has_gmac = 1,
	.tx_coe = 1,
	.init = sun7i_gmac_init,
};

