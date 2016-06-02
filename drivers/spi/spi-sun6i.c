/*
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <linux/spi/spi.h>

/*
 * The manual says FIFO depth is 128 bytes but transfer of more than 68 bytes
 * never finishes.
 */
#define SUNXI_FIFO_DEPTH		68

#define SUNXI_GBL_CTL_REG		0x04
#define SUNXI_CTL_ENABLE		BIT(0)
#define SUNXI_CTL_MASTER		BIT(1)
#define SUNXI_CTL_TP			BIT(7)

#define SUNXI_TFR_CTL_REG		0x08
#define SUNXI_TFR_CTL_CPHA		BIT(0)
#define SUNXI_TFR_CTL_CPOL		BIT(1)
#define SUNXI_TFR_CTL_CS_ACTIVE_LOW	BIT(2)
#define SUNXI_TFR_CTL_CS_MASK		0x30
#define SUNXI_TFR_CTL_CS(cs)		(((cs) << 4) & SUNXI_TFR_CTL_CS_MASK)
#define SUNXI_TFR_CTL_CS_MANUAL		BIT(6)
#define SUNXI_TFR_CTL_CS_LEVEL		BIT(7)
#define SUNXI_TFR_CTL_DHB		BIT(8)
#define SUNXI_TFR_CTL_FBS		BIT(12)
#define SUNXI_TFR_CTL_XCH		BIT(31)

#define SUNXI_INT_CTL_REG		0x10
#define SUNXI_INT_CTL_TC		BIT(12)

#define SUNXI_INT_STA_REG		0x14

#define SUNXI_FIFO_CTL_REG		0x18
#define SUNXI_CTL_RF_RST		BIT(15)
#define SUNXI_CTL_TF_RST		BIT(31)

#define SUNXI_FIFO_STA_REG		0x1c
#define SUNXI_FIFO_STA_RF_CNT_MASK	0x7f
#define SUNXI_FIFO_STA_RF_CNT_BITS	0
#define SUNXI_FIFO_STA_TF_CNT_MASK	0x7f
#define SUNXI_FIFO_STA_TF_CNT_BITS	16

#define SUNXI_CLK_CTL_REG		0x24
#define SUNXI_CLK_CTL_CDR2_MASK		0xff
#define SUNXI_CLK_CTL_CDR2(div)		(((div) & SUNXI_CLK_CTL_CDR2_MASK) << 0)
#define SUNXI_CLK_CTL_CDR1_MASK		0xf
#define SUNXI_CLK_CTL_CDR1(div)		(((div) & SUNXI_CLK_CTL_CDR1_MASK) << 8)
#define SUNXI_CLK_CTL_DRS		BIT(12)

#define SUNXI_BURST_CNT_REG		0x30
#define SUNXI_BURST_CNT(cnt)		((cnt) & 0xffffff)

#define SUNXI_XMIT_CNT_REG		0x34
#define SUNXI_XMIT_CNT(cnt)		((cnt) & 0xffffff)

#define SUNXI_BURST_CTL_CNT_REG		0x38
#define SUNXI_BURST_CTL_CNT_STC(cnt)	((cnt) & 0xffffff)

#define SUNXI_TXDATA_REG		0x200
#define SUNXI_RXDATA_REG		0x300

struct sunxi_spi {
	struct spi_master	*master;
	void __iomem		*base_addr;
	struct clk		*hclk;
	struct clk		*mclk;
	struct reset_control	*rstc;

	struct completion	done;

	const u8		*tx_buf;
	u8			*rx_buf;
	int			len;
};

static inline u32 sunxi_spi_read(struct sunxi_spi *sspi, u32 reg)
{
	return readl(sspi->base_addr + reg);
}

static inline void sunxi_spi_write(struct sunxi_spi *sspi, u32 reg, u32 value)
{
	writel(value, sspi->base_addr + reg);
}

static inline void sunxi_spi_drain_fifo(struct sunxi_spi *sspi, int len)
{
	u32 reg, cnt;
	u8 byte;

	/* See how much data is available */
	reg = sunxi_spi_read(sspi, SUNXI_FIFO_STA_REG);
	reg &= SUNXI_FIFO_STA_RF_CNT_MASK;
	cnt = reg >> SUNXI_FIFO_STA_RF_CNT_BITS;

	if (len > cnt)
		len = cnt;

	while (len--) {
		byte = readb(sspi->base_addr + SUNXI_RXDATA_REG);
		if (sspi->rx_buf)
			*sspi->rx_buf++ = byte;
	}
}

static inline void sunxi_spi_fill_fifo(struct sunxi_spi *sspi, int len)
{
	u8 byte;

	if (len > sspi->len)
		len = sspi->len;

	while (len--) {
		byte = sspi->tx_buf ? *sspi->tx_buf++ : 0;
		writeb(byte, sspi->base_addr + SUNXI_TXDATA_REG);
		sspi->len--;
	}
}

static void sunxi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct sunxi_spi *sspi = spi_master_get_devdata(spi->master);
	u32 reg;

	reg = sunxi_spi_read(sspi, SUNXI_TFR_CTL_REG);
	reg &= ~SUNXI_TFR_CTL_CS_MASK;
	reg |= SUNXI_TFR_CTL_CS(spi->chip_select);

	/* We want to control the chip select manually */
	reg |= SUNXI_TFR_CTL_CS_MANUAL;

	if (enable)
		reg |= SUNXI_TFR_CTL_CS_LEVEL;
	else
		reg &= ~SUNXI_TFR_CTL_CS_LEVEL;

	/*
	 * Even though this looks irrelevant since we are supposed to
	 * be controlling the chip select manually, this bit also
	 * controls the levels of the chip select for inactive
	 * devices.
	 *
	 * If we don't set it, the chip select level will go low by
	 * default when the device is idle, which is not really
	 * expected in the common case where the chip select is active
	 * low.
	 */
	if (spi->mode & SPI_CS_HIGH)
		reg &= ~SUNXI_TFR_CTL_CS_ACTIVE_LOW;
	else
		reg |= SUNXI_TFR_CTL_CS_ACTIVE_LOW;

	sunxi_spi_write(sspi, SUNXI_TFR_CTL_REG, reg);
}

static size_t sunxi_spi_max_transfer_size(struct spi_device *spi)
{
	return SUNXI_FIFO_DEPTH - 1;
}

static int sunxi_spi_transfer_one(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *tfr)
{
	struct sunxi_spi *sspi = spi_master_get_devdata(master);
	unsigned int mclk_rate, div, timeout;
	unsigned int start, end, tx_time;
	unsigned int tx_len = 0;
	int ret = 0;
	u32 reg;

	/* A 0 length transfer never finishes if programmed in the hardware */
	if (!tfr->len)
		return 0;

	/* We don't support transfer larger than the FIFO */
	if (tfr->len > SUNXI_FIFO_DEPTH)
		return -EINVAL;

	reinit_completion(&sspi->done);
	sspi->tx_buf = tfr->tx_buf;
	sspi->rx_buf = tfr->rx_buf;
	sspi->len = tfr->len;

	/* Clear pending interrupts */
	sunxi_spi_write(sspi, SUNXI_INT_STA_REG, ~0);

	reg = sunxi_spi_read(sspi, SUNXI_TFR_CTL_REG);

	/* Reset FIFOs */
	sunxi_spi_write(sspi, SUNXI_FIFO_CTL_REG,
			SUNXI_CTL_RF_RST | SUNXI_CTL_TF_RST);

	/*
	 * Setup the transfer control register: Chip Select,
	 * polarities, etc.
	 */
	if (spi->mode & SPI_CPOL)
		reg |= SUNXI_TFR_CTL_CPOL;
	else
		reg &= ~SUNXI_TFR_CTL_CPOL;

	if (spi->mode & SPI_CPHA)
		reg |= SUNXI_TFR_CTL_CPHA;
	else
		reg &= ~SUNXI_TFR_CTL_CPHA;

	if (spi->mode & SPI_LSB_FIRST)
		reg |= SUNXI_TFR_CTL_FBS;
	else
		reg &= ~SUNXI_TFR_CTL_FBS;

	/*
	 * If it's a TX only transfer, we don't want to fill the RX
	 * FIFO with bogus data
	 */
	if (sspi->rx_buf)
		reg &= ~SUNXI_TFR_CTL_DHB;
	else
		reg |= SUNXI_TFR_CTL_DHB;

	sunxi_spi_write(sspi, SUNXI_TFR_CTL_REG, reg);

	/* Ensure that we have a parent clock fast enough */
	mclk_rate = clk_get_rate(sspi->mclk);
	if (mclk_rate < (2 * tfr->speed_hz)) {
		clk_set_rate(sspi->mclk, 2 * tfr->speed_hz);
		mclk_rate = clk_get_rate(sspi->mclk);
	}

	/*
	 * Setup clock divider.
	 *
	 * We have two choices there. Either we can use the clock
	 * divide rate 1, which is calculated thanks to this formula:
	 * SPI_CLK = MOD_CLK / (2 ^ cdr)
	 * Or we can use CDR2, which is calculated with the formula:
	 * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
	 * Wether we use the former or the latter is set through the
	 * DRS bit.
	 *
	 * First try CDR2, and if we can't reach the expected
	 * frequency, fall back to CDR1.
	 */
	div = mclk_rate / (2 * tfr->speed_hz);
	if (div <= (SUNXI_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg = SUNXI_CLK_CTL_CDR2(div) | SUNXI_CLK_CTL_DRS;
	} else {
		div = ilog2(mclk_rate) - ilog2(tfr->speed_hz);
		reg = SUNXI_CLK_CTL_CDR1(div);
	}

	sunxi_spi_write(sspi, SUNXI_CLK_CTL_REG, reg);

	/* Setup the transfer now... */
	if (sspi->tx_buf)
		tx_len = tfr->len;

	/* Setup the counters */
	sunxi_spi_write(sspi, SUNXI_BURST_CNT_REG, SUNXI_BURST_CNT(tfr->len));
	sunxi_spi_write(sspi, SUNXI_XMIT_CNT_REG, SUNXI_XMIT_CNT(tx_len));
	sunxi_spi_write(sspi, SUNXI_BURST_CTL_CNT_REG,
			SUNXI_BURST_CTL_CNT_STC(tx_len));

	/* Fill the TX FIFO */
	sunxi_spi_fill_fifo(sspi, SUNXI_FIFO_DEPTH);

	/* Enable the interrupts */
	sunxi_spi_write(sspi, SUNXI_INT_CTL_REG, SUNXI_INT_CTL_TC);

	/* Start the transfer */
	reg = sunxi_spi_read(sspi, SUNXI_TFR_CTL_REG);
	sunxi_spi_write(sspi, SUNXI_TFR_CTL_REG, reg | SUNXI_TFR_CTL_XCH);

	tx_time = max(tfr->len * 8 * 2 / (tfr->speed_hz / 1000), 100U);
	start = jiffies;
	timeout = wait_for_completion_timeout(&sspi->done,
					      msecs_to_jiffies(tx_time));
	end = jiffies;
	if (!timeout) {
		dev_warn(&master->dev,
			 "%s: timeout transferring %u bytes@%iHz for %i(%i)ms",
			 dev_name(&spi->dev), tfr->len, tfr->speed_hz,
			 jiffies_to_msecs(end - start), tx_time);
		ret = -ETIMEDOUT;
		goto out;
	}

	sunxi_spi_drain_fifo(sspi, SUNXI_FIFO_DEPTH);

out:
	sunxi_spi_write(sspi, SUNXI_INT_CTL_REG, 0);

	return ret;
}

static irqreturn_t sunxi_spi_handler(int irq, void *dev_id)
{
	struct sunxi_spi *sspi = dev_id;
	u32 status = sunxi_spi_read(sspi, SUNXI_INT_STA_REG);

	/* Transfer complete */
	if (status & SUNXI_INT_CTL_TC) {
		sunxi_spi_write(sspi, SUNXI_INT_STA_REG, SUNXI_INT_CTL_TC);
		complete(&sspi->done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int sunxi_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sunxi_spi *sspi = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(sspi->hclk);
	if (ret) {
		dev_err(dev, "Couldn't enable AHB clock\n");
		goto out;
	}

	ret = clk_prepare_enable(sspi->mclk);
	if (ret) {
		dev_err(dev, "Couldn't enable module clock\n");
		goto err;
	}

	ret = reset_control_deassert(sspi->rstc);
	if (ret) {
		dev_err(dev, "Couldn't deassert the device from reset\n");
		goto err2;
	}

	sunxi_spi_write(sspi, SUNXI_GBL_CTL_REG,
			SUNXI_CTL_ENABLE | SUNXI_CTL_MASTER | SUNXI_CTL_TP);

	return 0;

err2:
	clk_disable_unprepare(sspi->mclk);
err:
	clk_disable_unprepare(sspi->hclk);
out:
	return ret;
}

static int sunxi_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sunxi_spi *sspi = spi_master_get_devdata(master);

	reset_control_assert(sspi->rstc);
	clk_disable_unprepare(sspi->mclk);
	clk_disable_unprepare(sspi->hclk);

	return 0;
}

static int sunxi_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct sunxi_spi *sspi;
	struct resource	*res;
	int ret = 0, irq;

	master = spi_alloc_master(&pdev->dev, sizeof(struct sunxi_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);
	sspi = spi_master_get_devdata(master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sspi->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sspi->base_addr)) {
		ret = PTR_ERR(sspi->base_addr);
		goto err_free_master;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No spi IRQ specified\n");
		ret = -ENXIO;
		goto err_free_master;
	}

	ret = devm_request_irq(&pdev->dev, irq, sunxi_spi_handler,
			       0, "sunxi-spi", sspi);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ\n");
		goto err_free_master;
	}

	sspi->master = master;
	master->max_speed_hz = 100 * 1000 * 1000;
	master->min_speed_hz = 3 * 1000;
	master->set_cs = sunxi_spi_set_cs;
	master->transfer_one = sunxi_spi_transfer_one;
	master->num_chipselect = 4;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->dev.of_node = pdev->dev.of_node;
	master->auto_runtime_pm = true;
	master->max_transfer_size = sunxi_spi_max_transfer_size;

	sspi->hclk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(sspi->hclk)) {
		dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
		ret = PTR_ERR(sspi->hclk);
		goto err_free_master;
	}

	sspi->mclk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(sspi->mclk)) {
		dev_err(&pdev->dev, "Unable to acquire module clock\n");
		ret = PTR_ERR(sspi->mclk);
		goto err_free_master;
	}

	init_completion(&sspi->done);

	sspi->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(sspi->rstc)) {
		dev_err(&pdev->dev, "Couldn't get reset controller\n");
		ret = PTR_ERR(sspi->rstc);
		goto err_free_master;
	}

	/*
	 * This wake-up/shutdown pattern is to be able to have the
	 * device woken up, even if runtime_pm is disabled
	 */
	ret = sunxi_spi_runtime_resume(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't resume the device\n");
		goto err_free_master;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		goto err_pm_disable;
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	sunxi_spi_runtime_suspend(&pdev->dev);
err_free_master:
	spi_master_put(master);
	return ret;
}

static int sunxi_spi_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id sunxi_spi_match[] = {
	{ .compatible = "allwinner,sun6i-a31-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_spi_match);

static const struct dev_pm_ops sunxi_spi_pm_ops = {
	.runtime_resume		= sunxi_spi_runtime_resume,
	.runtime_suspend	= sunxi_spi_runtime_suspend,
};

static struct platform_driver sunxi_spi_driver = {
	.probe	= sunxi_spi_probe,
	.remove	= sunxi_spi_remove,
	.driver	= {
		.name		= "sunxi-spi",
		.of_match_table	= sunxi_spi_match,
		.pm		= &sunxi_spi_pm_ops,
	},
};
module_platform_driver(sunxi_spi_driver);

MODULE_AUTHOR("Pan Nan <pannan@allwinnertech.com>");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A31 SPI controller driver");
MODULE_LICENSE("GPL");
