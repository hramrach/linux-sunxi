/*
 * Copyright 2012 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * OF helpers for mtd.
 *
 * This file is released under the GPLv2
 *
 */
#include <linux/kernel.h>
#include <linux/of_mtd.h>
#include <linux/mtd/nand.h>
#include <linux/export.h>

/**
 * It maps 'enum nand_ecc_modes_t' found in include/linux/mtd/nand.h
 * into the device tree binding of 'nand-ecc', so that MTD
 * device driver can get nand ecc from device tree.
 */
static const char *nand_ecc_modes[] = {
	[NAND_ECC_NONE]		= "none",
	[NAND_ECC_SOFT]		= "soft",
	[NAND_ECC_HW]		= "hw",
	[NAND_ECC_HW_SYNDROME]	= "hw_syndrome",
	[NAND_ECC_HW_OOB_FIRST]	= "hw_oob_first",
	[NAND_ECC_SOFT_BCH]	= "soft_bch",
};

/**
 * of_get_nand_ecc_mode - Get nand ecc mode for given device_node
 * @np:	Pointer to the given device_node
 *
 * The function gets ecc mode string from property 'nand-ecc-mode',
 * and return its index in nand_ecc_modes table, or errno in error case.
 */
int of_get_nand_ecc_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "nand-ecc-mode", &pm);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(nand_ecc_modes); i++)
		if (!strcasecmp(pm, nand_ecc_modes[i]))
			return i;

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(of_get_nand_ecc_mode);

/**
 * of_get_nand_bus_width - Get nand bus witdh for given device_node
 * @np:	Pointer to the given device_node
 *
 * return bus width option, or errno in error case.
 */
int of_get_nand_bus_width(struct device_node *np)
{
	u32 val;

	if (of_property_read_u32(np, "nand-bus-width", &val))
		return 8;

	switch(val) {
	case 8:
	case 16:
		return val;
	default:
		return -EIO;
	}
}
EXPORT_SYMBOL_GPL(of_get_nand_bus_width);

/**
 * of_get_nand_on_flash_bbt - Get nand on flash bbt for given device_node
 * @np:	Pointer to the given device_node
 *
 * return true if present false other wise
 */
bool of_get_nand_on_flash_bbt(struct device_node *np)
{
	return of_property_read_bool(np, "nand-on-flash-bbt");
}
EXPORT_SYMBOL_GPL(of_get_nand_on_flash_bbt);

/**
 * of_get_nand_timings - Get nand timings for the given device_node
 * @np:	Pointer to the given device_node
 *
 * return 0 on success errno other wise
 */
int of_get_nand_timings(struct device_node *np, struct nand_timings *timings)
{
	memset(timings, 0, sizeof(*timings));

	of_property_read_u32(np, "tCLS-min", &timings->tCLS_min);
	of_property_read_u32(np, "tCLH-min", &timings->tCLH_min);
	of_property_read_u32(np, "tCS-min", &timings->tCS_min);
	of_property_read_u32(np, "tCH-min", &timings->tCH_min);
	of_property_read_u32(np, "tWP-min", &timings->tWP_min);
	of_property_read_u32(np, "tALS-min", &timings->tALS_min);
	of_property_read_u32(np, "tALH-min", &timings->tALH_min);
	of_property_read_u32(np, "tDS-min", &timings->tDS_min);
	of_property_read_u32(np, "tDH-min", &timings->tDH_min);
	of_property_read_u32(np, "tWC-min", &timings->tWC_min);
	of_property_read_u32(np, "tWH-min", &timings->tWH_min);
	of_property_read_u32(np, "tR-max", &timings->tR_max);
	of_property_read_u32(np, "tAR-min", &timings->tAR_min);
	of_property_read_u32(np, "tCLR-min", &timings->tCLR_min);
	of_property_read_u32(np, "tRR-min", &timings->tRR_min);
	of_property_read_u32(np, "tRP-min", &timings->tRP_min);
	of_property_read_u32(np, "tWB-max", &timings->tWB_max);
	of_property_read_u32(np, "tRC-min", &timings->tRC_min);
	of_property_read_u32(np, "tREA-max", &timings->tREA_max);
	of_property_read_u32(np, "tRHZ-max", &timings->tRHZ_max);
	of_property_read_u32(np, "tCHZ-max", &timings->tCHZ_max);
	of_property_read_u32(np, "tRHOH-min", &timings->tRHOH_min);
	of_property_read_u32(np, "tRLOH-min", &timings->tRLOH_min);
	of_property_read_u32(np, "tCOH-min", &timings->tCOH_min);
	of_property_read_u32(np, "tREH-min", &timings->tREH_min);
	of_property_read_u32(np, "tWHR-min", &timings->tWHR_min);
	of_property_read_u32(np, "tRHW-min", &timings->tRHW_min);
	of_property_read_u32(np, "tIR-min", &timings->tIR_min);
	of_property_read_u32(np, "tCR-min", &timings->tCR_min);
	of_property_read_u32(np, "tADL-min", &timings->tADL_min);
	of_property_read_u32(np, "tRST-max", &timings->tRST_max);
	of_property_read_u32(np, "tWW-min", &timings->tWW_min);

	return 0;
}
EXPORT_SYMBOL_GPL(of_get_nand_timings);
