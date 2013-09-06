/*
 * drivers/mmc/host/sunxi-mci.c
 * (C) Copyright 2007-2011
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * Aaron.Maoye <leafy.myeh@reuuimllatech.com>
 * (C) Copyright 20013-2017
 * O2S GmbH <www.o2s.ch>
 * David Lanzendörfer <david.lanzendoerfer@o2s.ch>
 *
 * This is the driver for the SD/MMC host controller within the AllWinner A1X SoCs
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk-private.h>

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#include "sunxi-mci.h"

// Where to find what in the device tree
#define OF_AHB_CLK_POSITION 0
#define OF_MOD_CLK_POSITION 1

// Our debuglevel
#define CONFIG_MMC_DEBUG_LEVEL 3

static s32 sunxi_mmc_init_host(struct mmc_host* mmc)
{
	u32 rval;
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);

	SMC_DBG(smc_host, "MMC Driver init host %d\n", smc_host->pdev->id);

	/* reset controller */
	rval = mci_readl(smc_host, REG_GCTRL) | SDXC_HWReset;
	mci_writel(smc_host, REG_GCTRL, rval);

	mci_writel(smc_host, REG_FTRGL, 0x70008);
	mci_writel(smc_host, REG_TMOUT, 0xffffffff);
	mci_writel(smc_host, REG_IMASK, 0);
	mci_writel(smc_host, REG_RINTR, 0xffffffff);
	mci_writel(smc_host, REG_DBGC, 0xdeb);
	mci_writel(smc_host, REG_FUNS, 0xceaa0000);
	rval = mci_readl(smc_host, REG_GCTRL)|SDXC_INTEnb;
	rval &= ~SDXC_AccessDoneDirect;
	mci_writel(smc_host, REG_GCTRL, rval);

	smc_host->voltage = SDC_WOLTAGE_OFF;

	return 0;
}

s32 sunxi_mmc_exit_host(struct sunxi_mmc_host* smc_host)
{
	u32 rval;

	SMC_DBG(smc_host, "MMC Driver exit host %d\n", smc_host->pdev->id);
	smc_host->ferror = 0;
	smc_host->voltage = SDC_WOLTAGE_OFF;

	rval = mci_readl(smc_host, REG_GCTRL) | SDXC_HWReset;
	mci_writel(smc_host, REG_GCTRL, SDXC_HWReset);
	return 0;
}

s32 sunxi_mmc_set_vddio(struct sunxi_mmc_host* smc_host, u32 vdd)
{
	int ret = 0;
	switch (vdd) {
		case SDC_WOLTAGE_3V3:
			regulator_set_voltage(smc_host->regulator, 3300000, 3300000);
			ret=regulator_enable(smc_host->regulator);
			break;
		case SDC_WOLTAGE_1V8:
			regulator_set_voltage(smc_host->regulator, 1800000, 1800000);
			ret=regulator_enable(smc_host->regulator);
			break;
		case SDC_WOLTAGE_1V2:
			regulator_set_voltage(smc_host->regulator, 1200000, 1200000);
			ret=regulator_enable(smc_host->regulator);
			break;
		case SDC_WOLTAGE_OFF:
			ret=regulator_force_disable(smc_host->regulator);
			break;
	}
	return ret;
}

/* /\* UHS-I Operation Modes */
/*  * DS		25MHz	12.5MB/s	3.3V */
/*  * HS		50MHz	25MB/s		3.3V */
/*  * SDR12	25MHz	12.5MB/s	1.8V */
/*  * SDR25	50MHz	25MB/s		1.8V */
/*  * SDR50	100MHz	50MB/s		1.8V */
/*  * SDR104	208MHz	104MB/s		1.8V */
/*  * DDR50	50MHz	50MB/s		1.8V */
/*  * MMC Operation Modes */
/*  * DS		26MHz	26MB/s		3/1.8/1.2V */
/*  * HS		52MHz	52MB/s		3/1.8/1.2V */
/*  * HSDDR	52MHz	104MB/s		3/1.8/1.2V */
/*  * HS200	200MHz	200MB/s		1.8/1.2V */
/*  * */
/*  * Spec. Timing */
/*  * SD3.0 */
/*  * Fcclk    Tcclk   Fsclk   Tsclk   Tis     Tih     odly  RTis     RTih */
/*  * 400K     2.5us   24M     41ns    5ns     5ns     1     2209ns   41ns */
/*  * 25M      40ns    600M    1.67ns  5ns     5ns     3     14.99ns  5.01ns */
/*  * 50M      20ns    600M    1.67ns  6ns     2ns     3     14.99ns  5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  6ns     0.8ns   2     6.67ns   3.33ns */
/*  * 104M     9.6ns   600M    1.67ns  3ns     0.8ns   1     7.93ns   1.67ns */
/*  * 208M     4.8ns   600M    1.67ns  1.4ns   0.8ns   1     3.33ns   1.67ns */

/*  * 25M      40ns    300M    3.33ns  5ns     5ns     2     13.34ns   6.66ns */
/*  * 50M      20ns    300M    3.33ns  6ns     2ns     2     13.34ns   6.66ns */
/*  * 50MDDR   20ns    300M    3.33ns  6ns     0.8ns   1     6.67ns    3.33ns */
/*  * 104M     9.6ns   300M    3.33ns  3ns     0.8ns   0     7.93ns    1.67ns */
/*  * 208M     4.8ns   300M    3.33ns  1.4ns   0.8ns   0     3.13ns    1.67ns */

/*  * eMMC4.5 */
/*  * 400K     2.5us   24M     41ns    3ns     3ns     1     2209ns    41ns */
/*  * 25M      40ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50M      20ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  2.5ns   2.5ns   2     6.67ns    3.33ns */
/*  * 200M     5ns     600M    1.67ns  1.4ns   0.8ns   1     3.33ns    1.67ns */
/*  *\/ */

// Clock functions:
//------------------------------------------------------------------------------------------

static void sunxi_mmc_send_cmd(struct sunxi_mmc_host* smc_host, struct mmc_command* cmd)
{
	u32 imask = SDXC_IntErrBit;
	u32 cmd_val = SDXC_Start|(cmd->opcode&0x3f);
	unsigned long iflags;
	u32 wait = SDC_WAIT_NONE;

	wait = SDC_WAIT_CMD_DONE;
	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		cmd_val |= SDXC_SendInitSeq;
		imask |= SDXC_CmdDone;
	}

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		cmd_val |= SDXC_VolSwitch;
		imask |= SDXC_VolChgDone;
		smc_host->voltage_switching = 1;
		wait = SDC_WAIT_SWITCH1V8;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmd_val |= SDXC_RspExp;
		if (cmd->flags & MMC_RSP_136)
			cmd_val |= SDXC_LongRsp;
		if (cmd->flags & MMC_RSP_CRC)
			cmd_val |= SDXC_CheckRspCRC;

		if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
			cmd_val |= SDXC_DataExp | SDXC_WaitPreOver;
			wait = SDC_WAIT_DATA_OVER;
			if (cmd->data->flags & MMC_DATA_STREAM) {
				imask |= SDXC_AutoCMDDone;
				cmd_val |= SDXC_Seqmod | SDXC_SendAutoStop;
				wait = SDC_WAIT_AUTOCMD_DONE;
			}
			if (cmd->data->stop) {
				imask |= SDXC_AutoCMDDone;
				cmd_val |= SDXC_SendAutoStop;
				wait = SDC_WAIT_AUTOCMD_DONE;
			} else
				imask |= SDXC_DataOver;

			if (cmd->data->flags & MMC_DATA_WRITE)
				cmd_val |= SDXC_Write;
			else
				wait |= SDC_WAIT_DMA_DONE;
		} else
			imask |= SDXC_CmdDone;

	} else
		imask |= SDXC_CmdDone;
	SMC_DBG(smc_host, "smc %d cmd %d(%08x) arg %x ie 0x%08x wt %x len %d\n",
		smc_host->pdev->id, cmd_val&0x3f, cmd->arg, cmd_val, imask, wait,
		smc_host->mrq->data ? smc_host->mrq->data->blksz * smc_host->mrq->data->blocks : 0);
	spin_lock_irqsave(&smc_host->lock, iflags);
	smc_host->wait = wait;
	smc_host->state = SDC_STATE_SENDCMD;
	mci_writew(smc_host, REG_IMASK, imask);
	mci_writel(smc_host, REG_CARG, cmd->arg);
	mci_writel(smc_host, REG_CMDR, cmd_val);
	smp_wmb();
	spin_unlock_irqrestore(&smc_host->lock, iflags);
	SMC_DBG(smc_host, "%s: done\n",__FUNCTION__);
}

static void sunxi_mmc_init_idma_des(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	struct sunxi_mmc_idma_des* pdes = (struct sunxi_mmc_idma_des*)smc_host->sg_cpu;
	struct sunxi_mmc_idma_des* pdes_pa = (struct sunxi_mmc_idma_des*)smc_host->sg_dma;
	u32 des_idx = 0;
	u32 buff_frag_num = 0;
	u32 remain;
	u32 i, j;
	u32 config;

	for (i=0; i<data->sg_len; i++) {
		buff_frag_num = data->sg[i].length >> SDXC_DES_NUM_SHIFT;
		remain = data->sg[i].length & (SDXC_DES_BUFFER_MAX_LEN-1);
		if (remain)
			buff_frag_num ++;
		else
			remain = SDXC_DES_BUFFER_MAX_LEN;

		for (j=0; j < buff_frag_num; j++, des_idx++) {
			memset((void*)&pdes[des_idx], 0, sizeof(struct sunxi_mmc_idma_des));
			config = SDXC_IDMAC_DES0_CH|SDXC_IDMAC_DES0_OWN|SDXC_IDMAC_DES0_DIC;

		    	if (buff_frag_num > 1 && j != buff_frag_num-1)
				pdes[des_idx].data_buf1_sz = SDXC_DES_BUFFER_MAX_LEN;
		    	else
				pdes[des_idx].data_buf1_sz = remain;

			pdes[des_idx].buf_addr_ptr1 = sg_dma_address(&data->sg[i])
							+ j * SDXC_DES_BUFFER_MAX_LEN;
			if (i==0 && j==0)
				config |= SDXC_IDMAC_DES0_FD;

			if ((i == data->sg_len-1) && (j == buff_frag_num-1)) {
				config &= ~SDXC_IDMAC_DES0_DIC;
				config |= SDXC_IDMAC_DES0_LD|SDXC_IDMAC_DES0_ER;
				pdes[des_idx].buf_addr_ptr2 = 0;
				#ifdef CONFIG_CACULATE_TRANS_TIME
				last_pdes = &pdes[des_idx];
				#endif
			} else {
				pdes[des_idx].buf_addr_ptr2 = (u32)&pdes_pa[des_idx+1];
			}
			pdes[des_idx].config = config;
			SMC_INFO(smc_host, "sg %d, frag %d, remain %d, des[%d](%08x): "
		    		"[0] = %08x, [1] = %08x, [2] = %08x, [3] = %08x\n", i, j, remain,
				des_idx, (u32)&pdes[des_idx],
				(u32)((u32*)&pdes[des_idx])[0], (u32)((u32*)&pdes[des_idx])[1],
				(u32)((u32*)&pdes[des_idx])[2], (u32)((u32*)&pdes[des_idx])[3]);
		}
	}
	smp_wmb();
	return;
}

static int sunxi_mmc_prepare_dma(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	u32 dma_len;
	u32 i;
	u32 temp;
	struct scatterlist *sg;

	SMC_DBG(smc_host, "%s\n",__FUNCTION__);

	if (smc_host->sg_cpu == NULL)
		return -ENOMEM;

	dma_len = dma_map_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
			(data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (dma_len == 0) {
		SMC_ERR(smc_host, "no dma map memory\n");
		return -ENOMEM;
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3) {
			SMC_ERR(smc_host, "unaligned scatterlist: os %x length %d\n",
				sg->offset, sg->length);
			return -EINVAL;
		}
	}

	sunxi_mmc_init_idma_des(smc_host, data);

	temp = mci_readl(smc_host, REG_GCTRL);
	temp |= SDXC_DMAEnb;
	mci_writel(smc_host, REG_GCTRL, temp);
	temp |= SDXC_DMAReset;
	mci_writel(smc_host, REG_GCTRL, temp);
	mci_writel(smc_host, REG_DMAC, SDXC_IDMACSoftRST);
	temp = SDXC_IDMACFixBurst|SDXC_IDMACIDMAOn;
	mci_writel(smc_host, REG_DMAC, temp);
	temp = mci_readl(smc_host, REG_IDIE);
	temp &= ~(SDXC_IDMACReceiveInt|SDXC_IDMACTransmitInt);
	if (data->flags & MMC_DATA_WRITE)
		temp |= SDXC_IDMACTransmitInt;
	else
		temp |= SDXC_IDMACReceiveInt;
	mci_writel(smc_host, REG_IDIE, temp);

	//write descriptor address to register
	mci_writel(smc_host, REG_DLBA, smc_host->sg_dma);

	return 0;
}

int sunxi_mmc_send_manual_stop(struct sunxi_mmc_host* smc_host, struct mmc_request* req)
{
	struct mmc_data* data = req->data;
	u32 cmd_val = SDXC_Start | SDXC_RspExp | SDXC_StopAbortCMD
			| SDXC_CheckRspCRC | MMC_STOP_TRANSMISSION;
	u32 iflags = 0;
	u32 imask = 0;
	int ret = 0;
	u32 expire = jiffies + msecs_to_jiffies(1000);

	if (!data) {
		SMC_ERR(smc_host, "no data request\n");
		return -1;
	}
	/* disable interrupt */
	imask = mci_readw(smc_host, REG_IMASK);
	mci_writew(smc_host, REG_IMASK, 0);

	mci_writel(smc_host, REG_CARG, 0);
	mci_writel(smc_host, REG_CMDR, cmd_val);
	do {
		iflags = mci_readw(smc_host, REG_RINTR);
	} while(!(iflags & (SDXC_CmdDone | SDXC_IntErrBit)) && jiffies < expire);

	if (iflags & SDXC_IntErrBit) {
		SMC_ERR(smc_host, "sdc %d send stop command failed\n", smc_host->pdev->id);
		ret = -1;
	}

	if (req->stop)
		req->stop->resp[0] = mci_readl(smc_host, REG_RESP0);

	mci_writew(smc_host, REG_RINTR, iflags);

	/* enable interrupt */
	mci_writew(smc_host, REG_IMASK, imask);

	return ret;
}

void sunxi_mmc_dump_errinfo(struct sunxi_mmc_host* smc_host)
{
	SMC_ERR(smc_host, "smc %d err, cmd %d, %s%s%s%s%s%s%s%s%s%s !!\n",
		smc_host->pdev->id, smc_host->mrq->cmd ? smc_host->mrq->cmd->opcode : -1,
		smc_host->int_sum & SDXC_RespErr     ? " RE"     : "",
		smc_host->int_sum & SDXC_RespCRCErr  ? " RCE"    : "",
		smc_host->int_sum & SDXC_DataCRCErr  ? " DCE"    : "",
		smc_host->int_sum & SDXC_RespTimeout ? " RTO"    : "",
		smc_host->int_sum & SDXC_DataTimeout ? " DTO"    : "",
		smc_host->int_sum & SDXC_DataStarve  ? " DS"     : "",
		smc_host->int_sum & SDXC_FIFORunErr  ? " FE"     : "",
		smc_host->int_sum & SDXC_HardWLocked ? " HL"     : "",
		smc_host->int_sum & SDXC_StartBitErr ? " SBE"    : "",
		smc_host->int_sum & SDXC_EndBitErr   ? " EBE"    : ""
		);
}

s32 sunxi_mmc_wait_access_done(struct sunxi_mmc_host* smc_host)
{
	s32 own_set = 0;
	unsigned long expire = jiffies + msecs_to_jiffies(5);
	while (!(mci_readl(smc_host, REG_GCTRL) & SDXC_MemAccessDone) && jiffies < expire);
	if (!(mci_readl(smc_host, REG_GCTRL) & SDXC_MemAccessDone)) {
		SMC_MSG(smc_host, "wait memory access done timeout !!\n");
	}
	return own_set;
}

s32 sunxi_mmc_request_done(struct sunxi_mmc_host* smc_host)
{
	struct mmc_request* req = smc_host->mrq;
	u32 temp;
	s32 ret = 0;

	if (smc_host->int_sum & SDXC_IntErrBit) {
		/* if we got response timeout error information, we should check
		   if the command done status has been set. if there is no command
		   done information, we should wait this bit to be set */
		if ((smc_host->int_sum & SDXC_RespTimeout) && !(smc_host->int_sum & SDXC_CmdDone)) {
			u32 rint;
			u32 expire = jiffies + 1;
			do {
				rint = mci_readl(smc_host, REG_RINTR);
			} while (jiffies < expire && !(rint & SDXC_CmdDone));
		}

		sunxi_mmc_dump_errinfo(smc_host);
		if (req->data)
			SMC_ERR(smc_host, "In data %s operation\n",
				req->data->flags & MMC_DATA_WRITE ? "write" : "read");
		ret = -1;
		goto out;
	}

	if (req->cmd) {
		if (req->cmd->flags & MMC_RSP_136) {
			req->cmd->resp[0] = mci_readl(smc_host, REG_RESP3);
			req->cmd->resp[1] = mci_readl(smc_host, REG_RESP2);
			req->cmd->resp[2] = mci_readl(smc_host, REG_RESP1);
			req->cmd->resp[3] = mci_readl(smc_host, REG_RESP0);
		} else {
			req->cmd->resp[0] = mci_readl(smc_host, REG_RESP0);
		}
	}

out:
	if (req->data) {
		struct mmc_data* data = req->data;

		sunxi_mmc_wait_access_done(smc_host);
		mci_writel(smc_host, REG_IDST, 0x337);
		mci_writel(smc_host, REG_IDIE, 0);
		mci_writel(smc_host, REG_DMAC, 0);
		temp = mci_readl(smc_host, REG_GCTRL);
		mci_writel(smc_host, REG_GCTRL, temp|SDXC_DMAReset);
		temp &= ~SDXC_DMAEnb;
		mci_writel(smc_host, REG_GCTRL, temp);
		temp |= SDXC_FIFOReset;
		mci_writel(smc_host, REG_GCTRL, temp);
		dma_unmap_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
                                data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

	mci_writew(smc_host, REG_IMASK, 0);
	if (smc_host->int_sum & (SDXC_RespErr | SDXC_HardWLocked | SDXC_RespTimeout)) {
		SMC_DBG(smc_host, "sdc %d abnormal status: %s\n", smc_host->pdev->id,
			smc_host->int_sum & SDXC_HardWLocked ? "HardWLocked" : "RespErr");
	}

	mci_writew(smc_host, REG_RINTR, 0xffff);

	SMC_DBG(smc_host, "smc %d done, resp %08x %08x %08x %08x\n", smc_host->pdev->id,
		req->cmd->resp[0], req->cmd->resp[1], req->cmd->resp[2], req->cmd->resp[3]);

	if (req->data  && (smc_host->int_sum & SDXC_IntErrBit)) {
		SMC_MSG(smc_host, "found data error, need to send stop command !!\n");
		sunxi_mmc_send_manual_stop(smc_host, req);
	}

	return ret;
}

static int sunxi_mmc_resource_request(struct sunxi_mmc_host *host)
{
	struct platform_device *pdev = host->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *regs;
	int ret = 0;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;
	host->reg_base = ioremap(regs->start, resource_size(regs));

	host->clk_ahb = of_clk_get(np, OF_AHB_CLK_POSITION); // AHB gate
	if (IS_ERR(host->clk_ahb)) {
		dev_err(&pdev->dev, "Couldn't get AHB gate\n");
		ret = PTR_ERR(host->clk_ahb);
		goto resource_request_out;
	} else {
		SMC_DBG(host,"%s: got ahb_gate, name is %s\n", __FUNCTION__, host->clk_ahb->name);
	}

	host->clk_mod = of_clk_get(np, OF_MOD_CLK_POSITION); // Module 0
	if (IS_ERR(host->clk_mod)) {
		dev_err(&pdev->dev, "Couldn't get module clock\n");
		ret = PTR_ERR(host->clk_mod);
 		goto free_ahb_clk;
	} else {
		SMC_DBG(host,"%s: got clk_mod, name is %s\n", __FUNCTION__, host->clk_mod->name);
	}

	host->sg_cpu = dma_alloc_writecombine(NULL, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptor\n");
		goto free_mod_clk;
	}

	host->regulator = devm_regulator_get(&pdev->dev, "mmc");
	if (IS_ERR(host->regulator)) {
			if (PTR_ERR(host->regulator) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			else
				dev_info(&pdev->dev, "no regulator found\n");
	}

	host->wp_pin=of_get_named_gpio(np, "wp-gpios", 0);
	if (gpio_is_valid(host->wp_pin)) {
		gpio_request(host->wp_pin,"sdc0_wp");
		gpio_direction_input(host->wp_pin);
	}
	host->cd_pin=of_get_named_gpio(np, "cd-gpios", 0);
	if (gpio_is_valid(host->cd_pin)) {
		gpio_request(host->cd_pin,"sdc0_cd");
		gpio_direction_input(host->cd_pin);
	}

	SMC_DBG(host,"%s: WP-GPIO=%i,CD-GPIO=%i",__FUNCTION__,host->wp_pin,host->cd_pin);

	goto resource_request_out;

free_mod_clk:
	clk_put(host->clk_mod);
free_ahb_clk:
	clk_put(host->clk_ahb);
resource_request_out:
	return ret;
}

static int sunxi_mmc_resource_release(struct sunxi_mmc_host *host)
{
	if (host->sg_cpu) {
		dma_free_coherent(NULL, PAGE_SIZE,
				  host->sg_cpu, host->sg_dma);
	}
	clk_put(host->clk_ahb);
	clk_put(host->clk_mod);
	return 0;
}

static void sunxi_mmc_finalize_request(struct sunxi_mmc_host *smc_host)
{
	struct mmc_request* mrq = smc_host->mrq;
	unsigned long iflags;

	spin_lock_irqsave(&smc_host->lock, iflags);
	if (smc_host->wait != SDC_WAIT_FINALIZE) {
		spin_unlock_irqrestore(&smc_host->lock, iflags);
		SMC_MSG(smc_host, "nothing finalize, wt %x, st %d\n",
				smc_host->wait, smc_host->state);
		return;
	}
	smc_host->wait = SDC_WAIT_NONE;
	smc_host->state = SDC_STATE_IDLE;
	smc_host->trans_done = 0;
	smc_host->dma_done = 0;
	spin_unlock_irqrestore(&smc_host->lock, iflags);

	sunxi_mmc_request_done(smc_host);
	if (smc_host->error) {
		mrq->cmd->error = -ETIMEDOUT;
		if (mrq->data)
			mrq->data->error = -ETIMEDOUT;
		if (mrq->stop)
			mrq->stop->error = -ETIMEDOUT;
	} else {
		if (mrq->data)
			mrq->data->bytes_xfered = (mrq->data->blocks * mrq->data->blksz);
	}

	smc_host->mrq = NULL;
	smc_host->error = 0;
	smc_host->int_sum = 0;
	smp_wmb();
	mmc_request_done(smc_host->mmc, mrq);
	return;
}

static s32 sunxi_mmc_get_ro(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	
	int read_only = 0;

	if (gpio_is_valid(host->wp_pin)) {
		pinctrl_request_gpio(host->wp_pin);
		read_only = gpio_get_value(host->wp_pin);
	} else {
		SMC_DBG(host,"WP pin not found. Assuming RW\n");
	}

	return read_only;
}

static irqreturn_t sunxi_mmc_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *smc_host = dev_id;
	u32 sdio_int = 0;
	u32 raw_int;
	u32 msk_int;
	u32 idma_inte;
	u32 idma_int;

	spin_lock(&smc_host->lock);

	idma_int  = mci_readl(smc_host, REG_IDST);
	idma_inte = mci_readl(smc_host, REG_IDIE);
	raw_int   = mci_readl(smc_host, REG_RINTR);
	msk_int   = mci_readl(smc_host, REG_MISTA);
	if (!msk_int && !idma_int) {
		SMC_MSG(smc_host, "sdc%d nop irq: ri %08x mi %08x ie %08x idi %08x\n",
			smc_host->pdev->id, raw_int, msk_int, idma_inte, idma_int);
		spin_unlock(&smc_host->lock);
		return IRQ_HANDLED;
	}

	smc_host->int_sum |= raw_int;
	SMC_INFO(smc_host, "smc %d irq, ri %08x(%08x) mi %08x ie %08x idi %08x\n",
		smc_host->pdev->id, raw_int, smc_host->int_sum,
		msk_int, idma_inte, idma_int);

	if (msk_int & SDXC_SDIOInt) {
		sdio_int = 1;
		mci_writel(smc_host, REG_RINTR, SDXC_SDIOInt);
		goto sdio_out;
	}

	if (smc_host->wait == SDC_WAIT_NONE && !sdio_int) {
		SMC_ERR(smc_host, "smc %x, nothing to complete, ri %08x, "
			"mi %08x\n", smc_host->pdev->id, raw_int, msk_int);
		goto irq_out;
	}

	if ((raw_int & SDXC_IntErrBit) || (idma_int & SDXC_IDMA_ERR)) {
		smc_host->error = raw_int & SDXC_IntErrBit;
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
		goto irq_out;
	}
	if (idma_int & (SDXC_IDMACTransmitInt|SDXC_IDMACReceiveInt))
		smc_host->dma_done = 1;
	if (msk_int & (SDXC_AutoCMDDone|SDXC_DataOver|SDXC_CmdDone|SDXC_VolChgDone))
		smc_host->trans_done = 1;
	if ((smc_host->trans_done && (smc_host->wait == SDC_WAIT_AUTOCMD_DONE
					|| smc_host->wait == SDC_WAIT_DATA_OVER
					|| smc_host->wait == SDC_WAIT_CMD_DONE
					|| smc_host->wait == SDC_WAIT_SWITCH1V8))
		|| (smc_host->trans_done && smc_host->dma_done && (smc_host->wait & SDC_WAIT_DMA_DONE))) {
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
	}

irq_out:
	mci_writel(smc_host, REG_RINTR, msk_int&(~SDXC_SDIOInt));
	mci_writel(smc_host, REG_IDST, idma_int);

	if (smc_host->wait == SDC_WAIT_FINALIZE) {
		smp_wmb();
		mci_writew(smc_host, REG_IMASK, 0);
		tasklet_schedule(&smc_host->tasklet);
	}

sdio_out:
	spin_unlock(&smc_host->lock);

	if (sdio_int)
		mmc_signal_sdio_irq(smc_host->mmc);

	return IRQ_HANDLED;
}

static void sunxi_mmc_tasklet(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *) data;
	sunxi_mmc_finalize_request(smc_host);
}

s32 sunxi_mmc_update_clk(struct sunxi_mmc_host* smc_host)
{
	u32 rval;
	s32 expire = jiffies + msecs_to_jiffies(1000);  //1000ms timeout
	s32 ret = 0;

	SMC_DBG(smc_host,"%s\n",__FUNCTION__);
	rval = SDXC_Start|SDXC_UPCLKOnly|SDXC_WaitPreOver;
	if (smc_host->voltage_switching)
		rval |= SDXC_VolSwitch;
	mci_writel(smc_host, REG_CMDR, rval);

	do {
		rval = mci_readl(smc_host, REG_CMDR);
	} while (jiffies < expire && (rval & SDXC_Start));

	if (rval & SDXC_Start) {
		smc_host->ferror = 1;
		SMC_ERR(smc_host, "update clock timeout, fatal error!!!\n");
		ret = -1;
	}

	return ret;
}

static void sunxi_mmc_clk_set_rate(struct sunxi_mmc_host *smc_host, unsigned int rate)
{
	unsigned long newrate;
// 	u32 idiv = 0;
	u16 temp;

	SMC_DBG(smc_host,"%s: requested rate %i\n", __FUNCTION__, rate);

// 	newrate = clk_round_rate(smc_host->clk_mod, rate);
// 	SMC_DBG(smc_host,"%s: setting mod0 clock to %i, rounded %i\n",__FUNCTION__,rate,newrate);

	/* setting clock rate */
// 	clk_set_rate(smc_host->clk_mod,newrate);
	clk_set_rate(smc_host->clk_mod,rate);
	smc_host->clk_mod_rate = newrate = clk_get_rate(smc_host->clk_mod);
	SMC_DBG(smc_host,"%s: mod0 clock is now: %i\n",__FUNCTION__,newrate);

	/* set internal divider */
	/*idiv = (smc_host->clk_mod_rate) / rate / 2;
	temp = mci_readl(smc_host, REG_CLKCR);
	temp &= ~0xff;
	temp |= idiv | SDXC_CardClkOn;
	mci_writel(smc_host, REG_CLKCR, temp);*/
	temp = mci_readl(smc_host, REG_CLKCR);
	temp &= ~0xff;
	temp |= BIT(0) | SDXC_CardClkOn;
	mci_writel(smc_host, REG_CLKCR, temp);

	sunxi_mmc_update_clk(smc_host);
}

static void sunxi_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	u32 temp;
	s32 err;

	/* Set the power state */
	switch (ios->power_mode) {
	case MMC_POWER_ON:
		break;

	case MMC_POWER_UP:
		if (!smc_host->power_on) {
			err =  clk_prepare_enable(smc_host->clk_ahb);
			if (err) {
				dev_err(&smc_host->pdev->dev, "Failed to enable AHB gate\n");
				return;
			} else {
				SMC_DBG(smc_host,"%s: enabled ahb clock\n",__FUNCTION__);
			}
			mdelay(1);

			err =  clk_prepare_enable(smc_host->clk_mod);
			if (err) {
				dev_err(&smc_host->pdev->dev, "Failed to enable module clock\n");
				return;
			} else {
				SMC_DBG(smc_host,"%s: enabled module clock\n",__FUNCTION__);
			}
			mdelay(1);

			sunxi_mmc_init_host(mmc);
			enable_irq(smc_host->irq);
			SMC_DBG(smc_host, "sdc%d power on!\n", smc_host->pdev->id);
			smc_host->power_on = 1;
			smc_host->ferror = 0;
		}
		break;

	case MMC_POWER_OFF:
		if (smc_host->power_on) {
			SMC_DBG(smc_host, "sdc%d power off !!\n", smc_host->pdev->id);
			disable_irq(smc_host->irq);
			sunxi_mmc_exit_host(smc_host);
			clk_disable_unprepare(smc_host->clk_ahb);
			clk_disable_unprepare(smc_host->clk_mod);
			smc_host->power_on = 0;
			smc_host->ferror = 0;
		}
		break;
	}

	/* set bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH1);
		smc_host->bus_width = 1;
		break;
	case MMC_BUS_WIDTH_4:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH4);
		smc_host->bus_width = 4;
		break;
	case MMC_BUS_WIDTH_8:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH8);
		smc_host->bus_width = 8;
		break;
	}

	/* disable ddr mode */
	temp = mci_readl(smc_host, REG_GCTRL);
	temp &= ~SDXC_DDR_MODE;
	smc_host->ddr = 0;
	mci_writel(smc_host, REG_GCTRL, temp);

	/* set up clock */
	if (ios->clock && smc_host->power_on) {
		SMC_DBG(smc_host, "%s, ios->clock: %i\n", __FUNCTION__, ios->clock);
		sunxi_mmc_clk_set_rate(smc_host,ios->clock);
		usleep_range(50000, 55000);
	}
}

static void sunxi_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	unsigned long flags;
	u32 imask;

	spin_lock_irqsave(&smc_host->lock, flags);
	imask = mci_readl(smc_host, REG_IMASK);
	if (enable)
		imask |= SDXC_SDIOInt;
	else
		imask &= ~SDXC_SDIOInt;
	mci_writel(smc_host, REG_IMASK, imask);
	spin_unlock_irqrestore(&smc_host->lock, flags);
}

void sunxi_mmc_hw_reset(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	mci_writel(smc_host, REG_HWRST, 0);
	udelay(10);
	mci_writel(smc_host, REG_HWRST, 1);
	udelay(300);
}

int sunxi_mmc_card_present(struct mmc_host* mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);

	int present = 0;

	if (gpio_is_valid(smc_host->cd_pin)) {
		present = !gpio_get_value(smc_host->cd_pin); // Signal inverted "SDn-DET#"!
	} else {
		SMC_ERR(smc_host,"Could not allocate CD pin. Check your device tree for cd_pin entry!\n");
	}

	SMC_DBG(smc_host, "%s: %i\n",__FUNCTION__,present);

	return present;
}

static void sunxi_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	struct mmc_command* cmd = mrq->cmd;
	struct mmc_data* data = mrq->data;
	u32 byte_cnt = 0;
	int ret;

	SMC_DBG(smc_host, "%s: mrq: %i\n",__FUNCTION__,mrq->cmd->opcode);

	if (sunxi_mmc_card_present(mmc) == 0 || smc_host->ferror || !smc_host->power_on) {
		SMC_DBG(smc_host, "no medium present, ferr %d, pwd %d\n",
			    smc_host->ferror, smc_host->power_on);
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	smc_host->mrq = mrq;
	if (data) {
		byte_cnt = data->blksz * data->blocks;
		mci_writel(smc_host, REG_BLKSZ, data->blksz);
		mci_writel(smc_host, REG_BCNTR, byte_cnt);
		ret = sunxi_mmc_prepare_dma(smc_host, data);
		if (ret < 0) {
			SMC_ERR(smc_host, "smc %d prepare DMA failed\n", smc_host->pdev->id);
			cmd->error = ret;
			cmd->data->error = ret;
			smp_wmb();
			mmc_request_done(smc_host->mmc, mrq);
			return;
		}
	}
	sunxi_mmc_send_cmd(smc_host, cmd);
}

static const struct platform_device_id sunxi_mmc_devtype[] = {
	{
		.name = "sunxi-mci",
		.driver_data = 0,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, sunxi_mmc_devtype);

static const struct of_device_id sunxi_mmc_of_match[] = {
	{
		.compatible = "allwinner,sunxi-mmc",
		.data = &sunxi_mmc_devtype[0],
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_mmc_of_match);

static struct mmc_host_ops sunxi_mmc_ops = {
	.request	= sunxi_mmc_request,
	.set_ios	= sunxi_mmc_set_ios,
	.get_ro		= sunxi_mmc_get_ro,
	.get_cd		= sunxi_mmc_card_present,
	.enable_sdio_irq= sunxi_mmc_enable_sdio_irq,
	.hw_reset	= sunxi_mmc_hw_reset,
};

static int __init sunxi_mmc_probe(struct platform_device *pdev)
{
	struct sunxi_mmc_host *smc_host = NULL;
	struct mmc_host *mmc = NULL;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct sunxi_mmc_host), &pdev->dev);
	if (!mmc) {
		SMC_ERR(smc_host, "mmc alloc host failed\n");
		ret = -ENOMEM;
		goto sunxi_mmc_probe_out;
	}

	smc_host = mmc_priv(mmc);
	memset((void*)smc_host, 0, sizeof(smc_host));
#ifdef CONFIG_MMC_DEBUG
	smc_host->debuglevel=CONFIG_MMC_DEBUG_LEVEL;
#endif
	smc_host->mmc	= mmc;
	smc_host->pdev	= pdev;
	smc_host->pdata	= dev_get_platdata(&pdev->dev);
	spin_lock_init(&smc_host->lock);
	tasklet_init(&smc_host->tasklet, sunxi_mmc_tasklet, (unsigned long) smc_host);

	if (sunxi_mmc_resource_request(smc_host)) {
		SMC_ERR(smc_host, "%s: Failed to get resouce.\n", dev_name(&pdev->dev));
		goto probe_free_host;
	}

	smc_host->irq=platform_get_irq(pdev, 0);
	if (request_irq(smc_host->irq, sunxi_mmc_irq, 0, DRIVER_NAME, smc_host)) {
		SMC_DBG(smc_host, "%s: Failed to request smc card interrupt %i\n",__FUNCTION__,smc_host->irq);
		ret = -ENOENT;
		goto probe_free_resource;
	}
	disable_irq(smc_host->irq);

	if (smc_host->cd_mode == CARD_ALWAYS_PRESENT) {
		smc_host->present = 1;
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ) {
		SMC_ERR(smc_host, "Failed to get gpio irq for card detection\n");
		request_irq(gpio_to_irq(smc_host->cd_pin), sunxi_mmc_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mmc-detect", smc_host);
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL) {
		init_timer(&smc_host->cd_timer);
		smc_host->cd_timer.expires = jiffies + 1*HZ;
		smc_host->cd_timer.data = (unsigned long)smc_host;
		add_timer(&smc_host->cd_timer);
		smc_host->present = 0;
	}

	mmc->ops			= &sunxi_mmc_ops;
	mmc->max_blk_count	= 8192;
	mmc->max_blk_size	= 4096;
	mmc->max_req_size	= mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size	= mmc->max_req_size;
	mmc->max_segs	    = 128;
	//400kHz ~ 100MHz
	mmc->f_min			= 400000;
	mmc->f_max			= 10000000;

	ret = mmc_add_host(mmc);
	if (ret) {
		SMC_ERR(smc_host, "Failed to add mmc host.\n");
		goto probe_free_irq;
	}
	platform_set_drvdata(pdev, mmc);

	SMC_MSG(smc_host, "sdc%d Probe: base:0x%p irq:%u sg_cpu:%p(%x) ret %d.\n",
		pdev->id, smc_host->reg_base, smc_host->irq,
		smc_host->sg_cpu, smc_host->sg_dma, ret);
	goto sunxi_mmc_probe_out;

probe_free_irq:
	if (smc_host->irq)
		free_irq(smc_host->irq, smc_host);
probe_free_resource:
	sunxi_mmc_resource_release(smc_host);
probe_free_host:
	mmc_free_host(mmc);
sunxi_mmc_probe_out:
	SMC_MSG(smc_host, "Init done\n");
	return ret;
}

static int __exit sunxi_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host    	*mmc  = platform_get_drvdata(pdev);
	struct sunxi_mmc_host	*smc_host = mmc_priv(mmc);

	SMC_MSG(smc_host, "%s: Remove.\n", dev_name(&pdev->dev));

	sunxi_mmc_exit_host(smc_host);

	mmc_remove_host(mmc);

	tasklet_disable(&smc_host->tasklet);
	free_irq(smc_host->irq, smc_host);
	if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL)
		del_timer(&smc_host->cd_timer);

	sunxi_mmc_resource_release(smc_host);

	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sunxi_mmc_driver = {
	.driver = {
		.name	= "sunxi-mci",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sunxi_mmc_of_match),
	},
	.probe		= sunxi_mmc_probe,
	.remove		= __exit_p(sunxi_mmc_remove),
};
module_platform_driver(sunxi_mmc_driver);

MODULE_DESCRIPTION("Allwinner's SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Lanzendörfer <david.lanzendoerfer@o2s.ch>");
MODULE_ALIAS("platform:sunxi-mmc");
