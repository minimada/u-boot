/*
 *  Copyright (c) 2017 Nuvoton Technology Corp.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <dm.h>
#include <spi.h>
#include <fdtdec.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/gcr.h>
#include <asm/arch/fiu.h>
#include <asm/arch/spi_flash.h>

DECLARE_GLOBAL_DATA_PTR;

enum npcm750_sf_state {
	SF_CMD		    = 0,
	SF_ID		    = 1,
	SF_ADDR		    = 2,
	SF_READ		    = 3,
	SF_WRITE	    = 4,
	SF_ERASE	    = 5,
	SF_READ_STATUS	    = 6,
	SF_READ_STATUS1	    = 7,
	SF_WRITE_STATUS	    = 8,
	SF_READ_BAR,
	SF_WRITE_BAR,
};

/* Bits for the status register */
#define STAT_WIP        (1 << 0)
#define STAT_WEL        (1 << 1)

static const char *npcm750_sf_state_name(enum npcm750_sf_state state)
{
	static const char * const states[] = {
		"CMD", "ID", "ADDR", "READ", "WRITE", "ERASE", "READ_STATUS",
		"READ_STATUS1", "WRITE_STATUS", "READ_BAR", "WRITE_BAR"
	};
	return states[state];
}

struct npcm750_fiu_spi_platdata {
	void* regs;
	s32 frequency;
	u32 dev_num;
};

struct npcm750_fiu_spi_priv {
	struct npcm750_fiu_regs *regs;
	u32 cs;
	enum fiu_moudle_tag dev_num;
	enum npcm750_sf_state state;
	u16 status; /* The current flash status (see STAT_XXX defines above) */
	u32 cmd;
};

static void gcr_muxfiu(enum fiu_moudle_tag dev_num,
		bool cs0_en, bool cs1_en, bool cs2_en,
		bool cs3_en, bool quad_mode)
{
	struct npcm750_gcr *gcr = (struct npcm750_gcr *)npcm750_get_base_gcr();

	switch (dev_num) {
	case FIU_MODULE_0:
		/* config CS */
		if (cs0_en)
			/* nothing to do */;
		/* config CS1 */
		if (cs1_en)
			writel(readl(&gcr->mfsel1) | (1 << MFSEL1_S0CS1SEL),
				&gcr->mfsel1);
		/* config CS2 */
		if (cs2_en)
			writel(readl(&gcr->mfsel1) | (1 << MFSEL1_S0CS2SEL),
				&gcr->mfsel1);
		/* config CS3 */
		if (cs3_en)
			writel(readl(&gcr->mfsel1) | (1 << MFSEL1_S0CS3SEL),
				&gcr->mfsel1);
		/* select io bus width (1/2/4  <=> single/dual/quad ) */
		if (quad_mode) {
			/*  0: GPIO33/SPI0D2 ,1: nSPI0CS2 */
			writel(readl(&gcr->mfsel1) & ~(1 << MFSEL1_S0CS2SEL),
				&gcr->mfsel1);
			writel(readl(&gcr->mfsel1) & ~(1 << MFSEL1_S0CS3SEL),
				&gcr->mfsel1);
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_SP0QSEL),
				&gcr->mfsel4);
		}
		break;
	case FIU_MODULE_3:
		/* Select SPI3 */
		writel(readl(&gcr->mfsel4) | (1 << MFSEL4_SP3SEL),
			&gcr->mfsel4);
		/* config CS */
		if (cs0_en)
			/* nothing to do */;
		/* config CS1 */
		if (cs1_en)
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_S3CS1SEL),
				&gcr->mfsel4);
		/* Config CS2 */
		if (cs2_en)
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_S3CS2SEL),
				&gcr->mfsel4);
		/* Config CS3 */
		if (cs3_en)
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_S3CS3SEL),
				&gcr->mfsel4);
		/* select io bus width (1/2/4  <=> single/dual/quad ) */
		if (quad_mode) {
			writel(readl(&gcr->mfsel4) & ~(1 << MFSEL4_S3CS2SEL),
				&gcr->mfsel4);
			writel(readl(&gcr->mfsel4) & ~(1 << MFSEL4_S3CS3SEL),
				&gcr->mfsel4);
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_SP3QSEL),
				&gcr->mfsel4);
		}
		break;
	case FIU_MODULE_X:
		/* config CS */
		if (cs0_en)
			/* nothing to do */;
		/* config CS1 */
		if (cs1_en)
			writel(readl(&gcr->mfsel4) | (1 << MFSEL4_SXCS1SEL),
				&gcr->mfsel4);
		writel(readl(&gcr->mfsel4) | (1 << MFSEL4_SPXSEL),
			&gcr->mfsel4);
		break;
	default:
		break;
	};
}

static void fiu_spi_cs_activate(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);

	priv->state = SF_CMD;
	priv->cmd = SF_CMD;
}

static void fiu_spi_cs_deactivate(struct udevice *dev)
{
	return;
}

static int npcm750_fiu_spi_set_speed(struct udevice *bus, uint speed)
{
	return 0;
}

static int npcm750_fiu_spi_set_mode(struct udevice *bus, uint mode)
{
	return 0;
}

static int npcm750_fiu_spi_claim_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_platdata *slave_plat =
			dev_get_parent_platdata(dev);
	bool cs0_en, cs1_en, cs2_en, cs3_en, quad_mode;

	cs0_en = cs1_en = cs2_en = cs3_en = quad_mode = false;

	switch (slave_plat->cs) {
	case 0:
	default:
		cs0_en = true;
		quad_mode = true;
		break;
	case 1:
		cs1_en = true;
		quad_mode = true;
	case 2:
	case 3:
	case 4:
		break;
	};

	/* SPI flash init */
	gcr_muxfiu(priv->dev_num, cs0_en, cs1_en, cs2_en, cs3_en, quad_mode);
	return 0;
}

static int npcm750_fiu_spi_release_bus(struct udevice *dev)
{
	return 0;
}

static int fiu_uma_read(struct udevice *bus, u32 address, uint8_t *tx,
			bool is_address_size, u32 data_size)
{
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);
	struct npcm750_fiu_regs *regs = priv->regs;
	u32 data_reg[4];
	u32 uma_cfg = 0;
	u32 address_size = 0;

	uma_cfg = uma_cfg | (1 << FIU_UMA_CFG_CMDSIZ);

	if (is_address_size)
		address_size = 3;

	uma_cfg = (uma_cfg & ~(0x07 << FIU_UMA_CFG_ADDSIZ))
			| (address_size << FIU_UMA_CFG_ADDSIZ);

	/* Set the UMA address registers */
	writel(address, &regs->uma_addr);

	/* Set data size and direction */
	uma_cfg = (uma_cfg & ~(0x1f << FIU_UMA_CFG_RDATSIZ)) |
		(data_size << FIU_UMA_CFG_RDATSIZ);
	uma_cfg = uma_cfg & ~(0x1f << FIU_UMA_CFG_WDATSIZ);

	/* Set UMA CFG */
	writel(uma_cfg, &regs->uma_cfg);

	/* Initiate the read */
	writel(readl(&regs->uma_cts) | (1 << FIU_UMA_CTS_EXEC_DONE),
		&regs->uma_cts);

	/* wait for indication that transaction has terminated */
	while ((readl(&regs->uma_cts) & (1 << FIU_UMA_CTS_EXEC_DONE))
		== FIU_TRANS_STATUS_IN_PROG);

	/* copy read data from FIU_UMA_DB0-3 regs to data buffer */
	/* Set the UMA data registers - FIU_UMA_DB0-3 */
	if (data_size >= FIU_UMA_DATA_SIZE_1)
		data_reg[0] = readl(&regs->uma_dr0);
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		data_reg[1] = readl(&regs->uma_dr1);
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		data_reg[2] = readl(&regs->uma_dr2);
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		data_reg[3] = readl(&regs->uma_dr3);

	memcpy(tx, data_reg, data_size);

	return 0;
}

static int fiu_uma_write(struct udevice *bus, u32 address, const uint8_t *rx,
			bool is_address_size, u32 data_size)
{
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);
	struct npcm750_fiu_regs *regs = priv->regs;
	u32 data_reg[4];
	u32 uma_cfg = 0;
	u32 address_size = 0;

	uma_cfg = uma_cfg | (1 << FIU_UMA_CFG_CMDSIZ);

	if (is_address_size)
		address_size = 3;

	uma_cfg = (uma_cfg & ~(0x07 << FIU_UMA_CFG_ADDSIZ))
			| (address_size << FIU_UMA_CFG_ADDSIZ);
	/* Set the UMA address registers */
	writel(address, &regs->uma_addr);

	/* Set the UMA data registers - FIU_UMA_DB0-3 */
	memcpy(data_reg, rx, data_size);

	if (data_size >= FIU_UMA_DATA_SIZE_1)
		writel(data_reg[0], &regs->uma_dw0);
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		writel(data_reg[1], &regs->uma_dw1);
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		writel(data_reg[2], &regs->uma_dw2);
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		writel(data_reg[3], &regs->uma_dw3);

	/* Set data size and direction */
	uma_cfg = (uma_cfg & ~(0x1f << FIU_UMA_CFG_WDATSIZ)) |
		(data_size << FIU_UMA_CFG_WDATSIZ);
	uma_cfg = uma_cfg & ~(0x1f << FIU_UMA_CFG_RDATSIZ);

	/* Set UMA status */
	writel(uma_cfg, &regs->uma_cfg);

	/* Initiate the read */
	writel(readl(&regs->uma_cts) | (1 << FIU_UMA_CTS_EXEC_DONE),
		&regs->uma_cts);

	/* wait for indication that transaction has terminated */
	while ((readl(&regs->uma_cts) & (1 << FIU_UMA_CTS_EXEC_DONE))
		== FIU_TRANS_STATUS_IN_PROG);

	return 0;
}

static int npcm750_sf_process_cmd(struct udevice *dev, const u8 *rx, u8 *tx)
{
	struct udevice *bus = dev->parent;
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);
	struct npcm750_fiu_regs *regs = priv->regs;
	enum npcm750_sf_state oldstate = priv->state;
	struct dm_spi_slave_platdata *slave_plat =
			dev_get_parent_platdata(dev);

	priv->cmd = rx[0];
	/* set device number - DEV_NUM in FIU_UMA_CTS.
	   Select the chip select to be used in the following
	   UMA transaction */
	writel((readl(&regs->uma_cts) & ~(0x03 << FIU_UMA_CTS_DEV_NUM))
			| ((slave_plat->cs & 0x3) << FIU_UMA_CTS_DEV_NUM),
			&regs->uma_cts);

	/* set transaction code in FIU_UMA_CODE */
	writel((readl(&regs->uma_cmd) & ~(0xff << FIU_UMA_CMD_CMD))
			| (priv->cmd << FIU_UMA_CMD_CMD), &regs->uma_cmd);

	switch (priv->cmd) {
		case SPI_READ_JEDEC_ID_CMD:
			priv->state = SF_ID;
			priv->cmd = SF_ID;
			break;
		case SPI_READ_STATUS_REG_CMD:
			priv->state = SF_READ_STATUS;
			break;
		case SPI_WRITE_STATUS_REG_CMD:
			priv->state = SF_WRITE_STATUS;
			break;
		case SPI_EXTNADDR_RDEAR_CMD:
			priv->state = SF_READ_BAR;
			break;
		case SPI_EXTNADDR_WREAR_CMD:
			priv->state = SF_WRITE_BAR;
			break;
		case SPI_WRITE_ENABLE_CMD:
			fiu_uma_write(bus, 0x0, NULL, false, 0);
			priv->status |= STAT_WEL;
			break;
		case SPI_WRITE_DISABLE_CMD:
			priv->status &= ~STAT_WEL;
			break;
		case SPI_4K_SECTOR_ERASE_CMD:
		case SPI_64K_BLOCK_ERASE_CMD:
			priv->state = SF_ADDR;
			break;
		case SPI_PAGE_PRGM_CMD:
			writel(readl(&regs->uma_cts)
				& ~(1 << FIU_UMA_CTS_SW_CS), &regs->uma_cts);
			priv->state = SF_ADDR;
			break;
		default:
			printf("%s: unknown cmd:0x%x\n", __func__, priv->cmd);
			return -EIO;
	}

	if (oldstate != priv->state)
		debug(" %s: transition to %s state\n", dev->name,
				npcm750_sf_state_name(priv->state));

	return 0;
}

static int npcm750_fiu_spi_xfer(struct udevice *dev, unsigned int bitlen,
			const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);
	struct npcm750_fiu_regs *regs = priv->regs;
	struct dm_spi_slave_platdata *slave_plat =
			dev_get_parent_platdata(dev);
	const uint8_t *rx = dout;
	uint8_t *tx = din;
	int bytes = bitlen / 8;
	u32 address, pos = 0;
	u32 remain_data = 0; /* For SF_WRITE */
	int ret;
	//debug("%s: flags 0x%lx\n", __func__, flags);

	if (flags & SPI_XFER_BEGIN)
		fiu_spi_cs_activate(dev);

	if ((flags & SPI_XFER_MMAP) || (flags & SPI_XFER_MMAP_END))
		goto done;

	if (priv->state == SF_CMD) {
		ret = npcm750_sf_process_cmd(dev, rx, tx);
		if (ret)
			return ret;
		++pos;
	}

	if (priv->state == SF_WRITE)
		remain_data = bytes % CHUNK_SIZE;

	while (pos < bytes) {
		switch (priv->state) {
		case SF_ID:
			fiu_uma_read(bus, 0x0, tx, false, bytes);
			pos += bytes;
			break;
		case SF_READ_STATUS:
			fiu_uma_read(bus, 0x0, tx, false, 1);
			pos ++;
			break;
		case SF_WRITE_STATUS:
			fiu_uma_write(bus, 0x0, rx, false, 1);
			pos ++;
			break;
		case SF_READ_BAR:
			fiu_uma_read(bus, 0x0, tx, false, 1);
			pos ++;
			break;
		case SF_WRITE_BAR:
			fiu_uma_write(bus, 0x0, rx, false, 1);
			pos ++;
			break;
		case SF_ADDR:
			address = (rx[1] << 16) | (rx[2] << 8) | (rx[3] << 0);
			pos += 3;
			switch (priv->cmd) {
			case SPI_4K_SECTOR_ERASE_CMD:
			case SPI_64K_BLOCK_ERASE_CMD:
				if (!(priv->status & STAT_WEL)) {
					printf("%s: write enable not set"
						" before erase!\n",
						__func__);
				}
				fiu_uma_write(bus, address, NULL, true, 0);
				priv->status &= ~STAT_WEL;
				break;
			case SPI_PAGE_PRGM_CMD:
				priv->state = SF_WRITE;
				fiu_uma_write(bus, address, NULL, true, 0);
				break;
			}
			break;
		case SF_WRITE:
			if (!(priv->status & STAT_WEL)) {
				printf("%s: write enable not set"
					" before write!\n",
					__func__);
				goto done;
			}

			writel((readl(&regs->uma_cts) &
				~(0x03 << FIU_UMA_CTS_DEV_NUM))
				| ((slave_plat->cs & 0x3) << FIU_UMA_CTS_DEV_NUM),
				&regs->uma_cts);

			/* set transaction code in FIU_UMA_CODE */
			writel((readl(&regs->uma_cmd) &
				~(0xff << FIU_UMA_CMD_CMD))
				| (rx[0] << FIU_UMA_CMD_CMD),
				&regs->uma_cmd);

			if ((pos + remain_data) < bytes) {
				fiu_uma_write(bus, 0x0, &rx[1], false,
						CHUNK_SIZE - 1);
				pos = pos + CHUNK_SIZE;
				rx = rx + CHUNK_SIZE;
			} else if (remain_data) {
				fiu_uma_write(bus, 0x0, &rx[1], false,
						remain_data - 1);
				pos = pos + remain_data;
			}

			if (pos >= bytes) {
				writel(readl(&regs->uma_cts) |
					(1 << FIU_UMA_CTS_SW_CS),
					&regs->uma_cts);
				priv->status &= ~STAT_WEL;
			}
			break;
		default:
			printf("%s: no idea what to do.\n", __func__);
			goto done;
		}
	}

done:
	if (flags & SPI_XFER_END)
		fiu_spi_cs_deactivate(dev);

	return pos == bytes ? 0 : -EIO;
}

static int npcm750_fiu_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct npcm750_fiu_spi_platdata *plat = dev_get_platdata(bus);
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(bus);

	plat->regs = dev_read_addr_ptr(bus);

	/* Use 500KHz as a suitable default */
	plat->frequency = fdtdec_get_int(blob, node, "spi-max-frequency",
			500000);

	plat->dev_num = fdtdec_get_int(blob, node, "index", 0);

	return 0;
}

static int npcm750_fiu_spi_probe(struct udevice *bus)
{
	struct npcm750_fiu_spi_platdata *plat = dev_get_platdata(bus);
	struct npcm750_fiu_spi_priv *priv = dev_get_priv(bus);

	debug("%s, dev:%d\n", __func__, plat->dev_num);
	priv->regs = (struct npcm750_fiu_regs *)plat->regs;
	priv->dev_num = (enum fiu_moudle_tag)plat->dev_num;
	data = readl(&priv->regs->drd_cfg);
	debug("FIU drd: %08X\n", data);
	if (priv->dev_num == FIU_MODULE_X){
		// BUV w25q256 must use 4byte mode in direct read
		writel(0x030011bb, &priv->regs->drd_cfg);
		printf("force update FIU_DRD_CFG for FIUX\n");
	}

	return 0;
}

static const struct dm_spi_ops npcm750_fiu_spi_ops = {
	.claim_bus      = npcm750_fiu_spi_claim_bus,
	.release_bus    = npcm750_fiu_spi_release_bus,
	.xfer           = npcm750_fiu_spi_xfer,
	.set_speed      = npcm750_fiu_spi_set_speed,
	.set_mode       = npcm750_fiu_spi_set_mode,
	/*
	 * cs_info is not needed, since we require all chip selects to be
	 * in the device tree explicitly
	 */
};

static const struct udevice_id npcm750_fiu_spi_ids[] = {
	{ .compatible = "nuvoton,npcm750-fiu" },
	{ }
};

U_BOOT_DRIVER(npcm750_fiu_spi) = {
	.name   = "npcm750_fiu_spi",
	.id     = UCLASS_SPI,
	.of_match = npcm750_fiu_spi_ids,
	.ops    = &npcm750_fiu_spi_ops,
	.ofdata_to_platdata = npcm750_fiu_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct npcm750_fiu_spi_platdata),
	.priv_auto_alloc_size = sizeof(struct npcm750_fiu_spi_priv),
	.probe  = npcm750_fiu_spi_probe,
};
