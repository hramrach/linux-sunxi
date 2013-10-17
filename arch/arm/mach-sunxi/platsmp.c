/*
 *  linux/arch/arm/mach-sun7i/platsmp.c
 *
 *  Copyright (C) 2013 Fan Rong <cinifr@gmail.com>
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/smp.h>

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

/*
 * CPU Configure module support
 * 1: Software reset for smp cpus
 * 2: Configure for smp cpus including boot.
 * 3: Three 64-bit idle counters and two 64-bit common counters
 * it is needed for smp cpus
 */
void __iomem *sun7i_cc_base; /*CPU Configure Base*/
extern void sun7i_secondary_startup(void);

/*
 * CPUCFG
 */
#define SUN7I_CPUCFG_BOOTADDR	0x01a4

#define SUN7I_CPUCFG_GENCTL	0x0184
#define SUN7I_CPUCFG_DBGCTL0	0x01e0
#define SUN7I_CPUCFG_DBGCTL1	0x01e4

#define SUN7I_CPU1_PWR_CLAMP	0x01b0
#define SUN7I_CPU1_PWROFF_REG	0x01b4
#define SUN7I_CPUX_RESET_CTL(x)	(0x40 + (x)*0x40)

static struct of_device_id sun7i_cc_ids[] = {
	{ .compatible = "allwinner,sun7i-a20-cpuconfig"},
	{ /*sentinel*/ }
};

static int sun7i_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	long paddr;
	uint32_t pwr_reg;
	uint32_t j = 0xff << 1;
	if (!sun7i_cc_base) {
		pr_debug("error map cpu configure\n");
		return -ENOSYS;
	}
	/* Set boot addr */
	paddr = virt_to_phys(sun7i_secondary_startup);
	writel(paddr, sun7i_cc_base + SUN7I_CPUCFG_BOOTADDR);

	/* Assert cpu core reset */
	writel(0, sun7i_cc_base + SUN7I_CPUX_RESET_CTL(cpu));

	/* Ensure CPU reset also invalidates L1 caches */
	pwr_reg = readl(sun7i_cc_base + SUN7I_CPUCFG_GENCTL);
	pwr_reg &= ~BIT(cpu);
	writel(pwr_reg, sun7i_cc_base + SUN7I_CPUCFG_GENCTL);

	/* DBGPWRDUP hold low */
	pwr_reg = readl(sun7i_cc_base + SUN7I_CPUCFG_DBGCTL1);
	pwr_reg &= ~BIT(cpu);
	writel(pwr_reg, sun7i_cc_base + SUN7I_CPUCFG_DBGCTL1);

	/* Ramp up power to CPU1 */
	do {
		writel(j, sun7i_cc_base + SUN7I_CPU1_PWR_CLAMP);
		j = j >> 1;
	} while (j != 0);

	mdelay(10);

	pwr_reg = readl(sun7i_cc_base + SUN7I_CPU1_PWROFF_REG);
	pwr_reg &= ~1;
	writel(pwr_reg, sun7i_cc_base + SUN7I_CPU1_PWROFF_REG);
	mdelay(1);

	/* Release CPU reset */
	writel(3, sun7i_cc_base + SUN7I_CPUX_RESET_CTL(cpu));

	/* Unlock CPU */
	pwr_reg = readl(sun7i_cc_base + SUN7I_CPUCFG_DBGCTL1);
	pwr_reg |= BIT(cpu);
	writel(pwr_reg, sun7i_cc_base + SUN7I_CPUCFG_DBGCTL1);

	return 0;
}

static void __init sun7i_init_cpuconfig_map(unsigned int max_cpus)
{
	struct device_node *np;
	np = of_find_matching_node(NULL, sun7i_cc_ids);
	if (WARN(!np, "unable to setup cup configure"))
		return;
	sun7i_cc_base = of_iomap(np, 0);
	if (WARN(!sun7i_cc_base, "failed to map cup configure base address"))
		return;
}

struct smp_operations sun7i_smp_ops __initdata = {
	.smp_boot_secondary = sun7i_boot_secondary,
	.smp_prepare_cpus = sun7i_init_cpuconfig_map,
};
