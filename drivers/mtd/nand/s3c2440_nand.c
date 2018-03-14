/*
 * (C) Copyright 2006 DENX Software Engineering
 *
 * Implementation for U-Boot 1.1.6 by Samsung
 *
 * (C) Copyright 2008
 * Guennadi Liakhovetki, DENX Software Engineering, <lg@denx.de>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* Nand Flash driver for s3c2440 based on 6400 
 * by bhahn
 */


#include <common.h>

#include <linux/mtd/mtd.h>
#include <nand.h>
#include <asm/arch/s3c24x0_cpu.h>

#include <asm/io.h>
#include <asm/errno.h>

#if 0
#define MAX_CHIPS	2
static int nand_cs[MAX_CHIPS] = {0, 1};
#endif

#define S3C2440_NFCONF_TACLS(x)    ((x)<<12)
#define S3C2440_NFCONF_TWRPH0(x)   ((x)<<8)
#define S3C2440_NFCONF_TWRPH1(x)   ((x)<<4)
#define S3C2440_NFCONT_MODE     		(1<<0)  // NAND Flash Cont Enable
#define S3C2440_NFCONT_REG_nCE     		(1<<1)  
#define S3C2440_NFCONT_INIT_ECC     	(1<<4)
#define S3C2440_NFCONT_MAIN_ECC_LOCK  	(1<<5)
#define S3C2440_NFCONT_SPARE_ECC_LOCK  	(1<<6)
#define S3C2440_NFSTAT_RnB  			(1<<0)



#ifdef CONFIG_NAND_SPL
#define printf(arg...) do {} while (0)
#endif

/* Nand flash definition values by jsgood */
#ifdef S3C_NAND_DEBUG
/*
 * Function to print out oob buffer for debugging
 * Written by jsgood
 */
static void print_oob(const char *header, struct mtd_info *mtd)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	printf("%s:\t", header);

	for (i = 0; i < 64; i++)
		printf("%02x ", chip->oob_poi[i]);

	printf("\n");
}
#endif /* S3C_NAND_DEBUG */

#ifdef CONFIG_NAND_SPL
static u_char nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	return readb(this->IO_ADDR_R);
}

static void nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	for (i = 0; i < len; i++)
		writeb(buf[i], this->IO_ADDR_W);
}

static void nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	for (i = 0; i < len; i++)
		buf[i] = readb(this->IO_ADDR_R);
}
#endif


static void s3c_nand_select_chip(struct mtd_info *mtd, int chip)
{

	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	int ctrl = readl(&nand->nfcont);

	switch (chip) {
	case -1:
		ctrl |= S3C2440_NFCONT_REG_nCE;
		break;
	case 0:
		ctrl &= ~S3C2440_NFCONT_REG_nCE;
		break;
#if 0
	case 1:
		ctrl &= ~4;
		break;
#endif
	default:
		return;
	}

	writel(ctrl, &nand->nfcont);
}


/*
 * Hardware specific access to control-lines function
 * Written by jsgood
 * Modified by bhahn for s3c2440
 */
static void s3c_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;

	struct s3c2440_nand *nand = s3c2440_get_base_nand();


	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_CLE)
			this->IO_ADDR_W = (void __iomem *)(&(nand->nfcmd));
		else if (ctrl & NAND_ALE)
			this->IO_ADDR_W = (void __iomem *)(&(nand->nfaddr));
		else
			this->IO_ADDR_W = (void __iomem *)(&(nand->nfdata));

		if (ctrl & NAND_NCE)
            writel(readl(&nand->nfcont) & ~S3C2440_NFCONT_REG_nCE,
                   &nand->nfcont);
        else
            writel(readl(&nand->nfcont) | S3C2440_NFCONT_REG_nCE,
                   &nand->nfcont);


	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, this->IO_ADDR_W);

	
//	MTDDEBUG(MTD_DEBUG_LEVEL1,"nand hwcont cmd:0x%X, ctrl:0x%X, addr:0x%X\n",cmd,ctrl,this->IO_ADDR_W);

}

/*
 * Function for checking device ready pin
 * Written by jsgood
 */
static int s3c_nand_device_ready(struct mtd_info *mtdinfo)
{

	struct s3c2440_nand *nand = s3c2440_get_base_nand();
	return !!(readl(&nand->nfstat) & S3C2440_NFSTAT_RnB);
}

#ifdef CONFIG_S3C2440_NAND_HWECC
/*
 * This function is called before encoding ecc codes to ready ecc engine.
 * Written by jsgood
 */

#if 0
static void s3c_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u_long nfcont, nfconf;

	/*
	 * The original driver used 4-bit ECC for "new" MLC chips, i.e., for
	 * those with non-zero ID[3][3:2], which anyway only holds for ST
	 * (Numonyx) chips
	 */
	nfconf = readl(NFCONF) & ~NFCONF_ECC_4BIT;

	writel(nfconf, NFCONF);

	/* Initialize & unlock */
	nfcont = readl(NFCONT);
	nfcont |= NFCONT_INITECC;
	nfcont &= ~NFCONT_MECCLOCK;

	if (mode == NAND_ECC_WRITE)
		nfcont |= NFCONT_ECC_ENC;
	else if (mode == NAND_ECC_READ)
		nfcont &= ~NFCONT_ECC_ENC;

	writel(nfcont, NFCONT);
}

#else
static void s3c_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u_long nfcont;

	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	/* Initialize & unlock */
	nfcont = readl(&nand->nfcont);
	nfcont |= S3C2440_NFCONT_INIT_ECC ;
	nfcont &= ~S3C2440_NFCONT_MAIN_ECC_LOCK;

	writel(nfcont, &nand->nfcont);
}

#endif



/*
 * This function is called immediately after encoding ecc codes.
 * This function returns encoded ecc codes.
 * Written by jsgood
 */
static int s3c_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{
	u_long nfcont, nfmecc0;

	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	/* Lock */
	nfcont = readl(&nand->nfcont);
	nfcont |= S3C2440_NFCONT_MAIN_ECC_LOCK;
	writel(nfcont, &nand->nfcont);

	nfmecc0 = readl(&nand->nfmecc0);

	ecc_code[0] = nfmecc0 & 0xff;
	ecc_code[1] = (nfmecc0 >> 8) & 0xff;
	ecc_code[2] = (nfmecc0 >> 16) & 0xff;
	ecc_code[3] = (nfmecc0 >> 24) & 0xff;

	return 0;
}

/*
 * This function determines whether read data is good or not.
 * If SLC, must write ecc codes to controller before reading status bit.
 * If MLC, status bit is already set, so only reading is needed.
 * If status bit is good, return 0.
 * If correctable errors occured, do that.
 * If uncorrectable errors occured, return -1.
 * Written by jsgood
 */

#if 0
static int s3c_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	int ret = -1;
	u_long nfestat0, nfmeccdata0, nfmeccdata1, err_byte_addr;
	u_char err_type, repaired;



	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	/* SLC: Write ecc to compare */
	nfmeccdata0 = (calc_ecc[1] << 16) | calc_ecc[0];
	nfmeccdata1 = (calc_ecc[3] << 16) | calc_ecc[2];
	writel(nfmeccdata0, NFMECCDATA0);
	writel(nfmeccdata1, NFMECCDATA1);

	/* Read ecc status */
	nfestat0 = readl(NFESTAT0);
	err_type = nfestat0 & 0x3;

	switch (err_type) {
	case 0: /* No error */
		ret = 0;
		break;

	case 1:
		/*
		 * 1 bit error (Correctable)
		 * (nfestat0 >> 7) & 0x7ff	:error byte number
		 * (nfestat0 >> 4) & 0x7	:error bit number
		 */
		err_byte_addr = (nfestat0 >> 7) & 0x7ff;
		repaired = dat[err_byte_addr] ^ (1 << ((nfestat0 >> 4) & 0x7));

		printf("S3C NAND: 1 bit error detected at byte %ld. "
		       "Correcting from 0x%02x to 0x%02x...OK\n",
		       err_byte_addr, dat[err_byte_addr], repaired);

		dat[err_byte_addr] = repaired;

		ret = 1;
		break;

	case 2: /* Multiple error */
	case 3: /* ECC area error */
		printf("S3C NAND: ECC uncorrectable error detected. "
		       "Not correctable.\n");
		ret = -1;
		break;
	}

	return ret;
}
#else

static int s3c_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	int ret = -1;
	u_long nfestat0, nfmeccdata0, nfmeccdata1, err_byte_addr;
	u_char err_type, repaired;


	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	/* Write ecc to compare */
	nfmeccdata0 = (calc_ecc[1] << 16) | calc_ecc[0];
	nfmeccdata1 = (calc_ecc[3] << 16) | calc_ecc[2];
	writel(nfmeccdata0, &nand->nfmeccd0);
	writel(nfmeccdata1, &nand->nfmeccd1);

	/* Read ecc status */
	nfestat0 = readl(&nand->nfestat0);
	err_type = nfestat0 & 0x3;

	switch (err_type) {
	case 0: /* No error */
		ret = 0;
		break;

	case 1:
		/*
		 * 1 bit error (Correctable)
		 * (nfestat0 >> 7) & 0x7ff	:error byte number
		 * (nfestat0 >> 4) & 0x7	:error bit number
		 */
		err_byte_addr = (nfestat0 >> 7) & 0x7ff;
		repaired = dat[err_byte_addr] ^ (1 << ((nfestat0 >> 4) & 0x7));

		printf("S3C NAND: 1 bit error detected at byte %ld. "
		       "Correcting from 0x%02x to 0x%02x...OK\n",
		       err_byte_addr, dat[err_byte_addr], repaired);

		dat[err_byte_addr] = repaired;

		ret = 1;
		break;

	case 2: /* Multiple error */
	case 3: /* ECC area error */
		printf("S3C NAND: ECC uncorrectable error detected. "
		       "Not correctable.\n");
		ret = -1;
		break;
	}

	return ret;
}



#endif


#endif /* CONFIG_S3C2440_NAND_HWECC */

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int board_nand_init(struct nand_chip *nand)
{

#if 0
	static int chip_n;

	if (chip_n >= MAX_CHIPS)
		return -ENODEV;


	NFCONT_REG = (NFCONT_REG & ~NFCONT_WP) | NFCONT_ENABLE | 0x6;

#endif

	u_int32_t cfg,cont;
    u_int8_t tacls, twrph0, twrph1;
    struct s3c24x0_clock_power *clk_power = s3c24x0_get_base_clock_power();
    struct s3c2440_nand *nand_reg = s3c2440_get_base_nand();

    debug("2440 board_nand_init()\n");

    /* Control HCLK into NAND Flash Controller block */
    writel(readl(&clk_power->clkcon) | (1 << 4), &clk_power->clkcon);


#if defined(CONFIG_S3C24XX_CUSTOM_NAND_TIMING)
    tacls  = CONFIG_S3C24XX_TACLS;
    twrph0 = CONFIG_S3C24XX_TWRPH0;
    twrph1 =  CONFIG_S3C24XX_TWRPH1;
#else	
	/* Might be the default value */
    tacls = 3;
    twrph0 = 7;
    twrph1 = 7;
#endif


	/* NAND flash controller operating mode - enable */
	cont = nand_reg->nfcont;
	cont |= S3C2440_NFCONT_MODE; // Nand controller enable
	 
    writel(cont, &nand_reg->nfcont);

	/* s3c2440 nand flash timing set */
	cfg = nand_reg->nfconf;
    cfg |= S3C2440_NFCONF_TACLS(tacls);
    cfg |= S3C2440_NFCONF_TWRPH0(twrph0);
    cfg |= S3C2440_NFCONF_TWRPH1(twrph1);
    writel(cfg, &nand_reg->nfconf);


	nand->IO_ADDR_R		= (void __iomem *)&nand_reg->nfdata;
	nand->IO_ADDR_W		= (void __iomem *)&nand_reg->nfdata;
	nand->cmd_ctrl		= s3c_nand_hwcontrol;
	nand->dev_ready		= s3c_nand_device_ready;
	nand->select_chip	= s3c_nand_select_chip;
	nand->options		= 0;

#ifdef CONFIG_NAND_SPL
	nand->read_byte		= nand_read_byte;
	nand->write_buf		= nand_write_buf;
	nand->read_buf		= nand_read_buf;
#endif

#ifdef CONFIG_S3C2440_NAND_HWECC
	nand->ecc.hwctl		= s3c_nand_enable_hwecc;
	nand->ecc.calculate	= s3c_nand_calculate_ecc;
	nand->ecc.correct	= s3c_nand_correct_data;

	nand->ecc.mode		= NAND_ECC_HW;
	nand->ecc.size		= CONFIG_SYS_NAND_ECCSIZE;
	nand->ecc.bytes		= CONFIG_SYS_NAND_ECCBYTES;
#else
	nand->ecc.mode		= NAND_ECC_SOFT;
#endif /* ! CONFIG_S3C2440_NAND_HWECC */

//	nand->priv		= nand_cs + chip_n++;

	return 0;
}
