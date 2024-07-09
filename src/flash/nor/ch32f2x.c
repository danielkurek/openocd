// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023 by MengFanYu                                       *
 *   SecondHandCoder@gmail.com                                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>


#define FLASH_ERASE_TIMEOUT			160
#define FLASH_WRITE_TIMEOUT			20
#define FLASH_COMMAND_TIMEOUT			5

#define CH32F2X_FLASH_SECTOR_KB			4
#define CH32F2X_FLASH_PAGE_SIZE			256
#define CH32F2X_WRITE_ALGORITHM_STACK		32

#define CH32F2X_FLASH_BANK_BASE			0x08000000
#define CH32F2X_OBR_BANK_BASE			0x1FFFF800

#define CH32F2X_FLASH_INFO			0x1FFFF7E0
#define CH32F2X_IDCODE_BASE			0xE0042000

/* FLASH register addr */
#define CH32F2X_FLASH_BASE			0x40022000

#define CH32F2X_FLASH_ACR_OFFSET		0x00
#define CH32F2X_FLASH_KEYR_OFFSET		0x04
#define CH32F2X_FLASH_OBKEYR_OFFSET 		0x08
#define CH32F2X_FLASH_STATR_OFFSET		0x0C
#define CH32F2X_FLASH_CTRL_OFFSET		0x10
#define CH32F2X_FLASH_ADDR_OFFSET		0x14
#define CH32F2X_FLASH_OBR_OFFSET		0x1C
#define CH32F2X_FLASH_WPR_OFFSET		0x20
#define CH32F2X_FLASH_MODEKRY_OFFSET		0x24

/* FLASH_STATR register bits */
#define CH32F2X_FLASH_STATR_BSY			0x00000001
#define CH32F2X_FLASH_STATR_WRBSY		0x00000002
#define CH32F2X_FLASH_STATR_WRPRTERR		0x00000010
#define CH32F2X_FLASH_STATR_EOP			0x00000020
#define CH32F2X_FLASH_STATR_EHMODS		0x00000080

/* FLASH_CTRL register bits */
#define CH32F2X_FLASH_CTRL_PG			0x00000001
#define CH32F2X_FLASH_CTRL_PER			0x00000002
#define CH32F2X_FLASH_CTRL_MER			0x00000004
#define CH32F2X_FLASH_CTRL_OBPG			0x00000010
#define CH32F2X_FLASH_CTRL_OBER			0x00000020
#define CH32F2X_FLASH_CTRL_STRT			0x00000040
#define CH32F2X_FLASH_CTRL_LOCK			0x00000080
#define CH32F2X_FLASH_CTRL_OBWRE		0x00000200
#define CH32F2X_FLASH_CTRL_ERRIR		0x00000400
#define CH32F2X_FLASH_CTRL_EOPIE		0x00001000
#define CH32F2X_FLASH_CTRL_FLOCK		0x00008000
#define CH32F2X_FLASH_CTRL_FTPG			0x00010000
#define CH32F2X_FLASH_CTRL_FTER			0x00020000
#define CH32F2X_FLASH_CTRL_BER32		0x00040000
#define CH32F2X_FLASH_CTRL_BER64		0x00080000
#define CH32F2X_FLASH_CTRL_PGSTRT		0x00200000
#define CH32F2X_FLASH_CTRL_PSENACT		0x00400000
#define CH32F2X_FLASH_CTRL_EHMOD		0x01000000
#define CH32F2X_FLASH_CTRL_SCKMOD		0x02000000

/* FLASH_OBR register bits */
#define CH32F2X_FLASH_OBR_OBERR			0x00000001
#define CH32F2X_FLASH_OBR_RDRRT			0x00000002
#define CH32F2X_FLASH_OBR_IWDGSW		0x00000004
#define CH32F2X_FLASH_OBR_STOPRST		0x00000008
#define CH32F2X_FLASH_OBR_STANDYRST		0x00000010
#define CH32F2X_FLASH_OBR_RAM_CODE_MODE		0x00000300

/* register unlock keys */
#define CH32F2X_OBR_KEY				0xA5
#define CH32F2X_KEY1				0x45670123
#define CH32F2X_KEY2				0xCDEF89AB


struct ch32f2x_options {
	uint8_t rdp;
	uint8_t user;
	uint16_t data;
	uint32_t protection;
};

struct ch32f2x_flash_bank {
	struct ch32f2x_options option_bytes;
	int ppage_size;
	bool probed;

	uint32_t register_base;
	uint8_t default_rdp;
	uint32_t user_bank_size;
};

/* flash bank ch32f2x <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(ch32f2x_flash_bank_command)
{
	struct ch32f2x_flash_bank *ch32f2x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch32f2x_info = malloc(sizeof(struct ch32f2x_flash_bank));

	bank->driver_priv = ch32f2x_info;
	ch32f2x_info->probed = false;
	ch32f2x_info->register_base = CH32F2X_FLASH_BASE;
	ch32f2x_info->user_bank_size = bank->size;

	/* The flash write must be aligned to a halfword boundary */
	bank->write_start_alignment = bank->write_end_alignment = 2;

	return ERROR_OK;
}

static inline int ch32f2x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	return ch32f2x_info->register_base + reg;
}

static inline int ch32f2x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_STATR_OFFSET), status);
}

static int ch32f2x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	for (;;) {
		retval = ch32f2x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32, status);
		if ((status & CH32F2X_FLASH_STATR_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & CH32F2X_FLASH_STATR_WRPRTERR) {
		LOG_ERROR("ch32f2x device protected");
		/* Clear WRPRTERR bit */
		target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_STATR_OFFSET),
			status | CH32F2X_FLASH_STATR_WRPRTERR);
		retval = ERROR_FAIL;
	}

	return retval;
}

static int ch32f2x_unlock_reg(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & CH32F2X_FLASH_CTRL_LOCK) == 0)
		return ERROR_OK;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_KEYR_OFFSET), CH32F2X_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_KEYR_OFFSET), CH32F2X_KEY2);
	if (retval != ERROR_OK)
		return retval;

	int timeout = FLASH_COMMAND_TIMEOUT;
	for(;;) {
		retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
		if ((ctrl & CH32F2X_FLASH_CTRL_LOCK) == 0)
			return ERROR_OK;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash unlock, maybe flash is locked-up, please reset");
			return ERROR_TARGET_FAILURE;
		}
		alive_sleep(1);	
	}

	return ERROR_OK;
}

static int ch32f2x_unlock_options_reg(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (((ctrl & CH32F2X_FLASH_CTRL_LOCK) == 0)
		&&(ctrl & CH32F2X_FLASH_CTRL_OBWRE))
		return ERROR_OK;

	/* unlock flash registers */
	if (ctrl & CH32F2X_FLASH_CTRL_LOCK) {
		retval = ch32f2x_unlock_reg(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* unlock options registers */
	if ((ctrl & CH32F2X_FLASH_CTRL_OBWRE) == 0) {
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_OBKEYR_OFFSET), CH32F2X_KEY1);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_OBKEYR_OFFSET), CH32F2X_KEY2);
		if (retval != ERROR_OK)
			return retval;

		int timeout = FLASH_COMMAND_TIMEOUT;
		for(;;) {
			retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
			if (ctrl & CH32F2X_FLASH_CTRL_OBWRE)
				return ERROR_OK;
			if (timeout-- <= 0) {
				LOG_ERROR("timed out waiting for flash options unlock, maybe flash options is locked-up, please reset");
				return ERROR_TARGET_FAILURE;
			}
			alive_sleep(1);	
		}
	}

	return ERROR_OK;
}

static int ch32f2x_read_options(struct flash_bank *bank)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t option_bytes;
	int retval = ERROR_OK;

	/* read user and read protection option bytes */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_OBR_OFFSET), &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32f2x_info->option_bytes.rdp = (option_bytes & CH32F2X_FLASH_OBR_RDRRT) ? 0 : ch32f2x_info->default_rdp;
	ch32f2x_info->option_bytes.user = (option_bytes >> 2) & 0xFF;

	/* read user data option bytes */
	retval = target_read_u32(target, CH32F2X_OBR_BANK_BASE + 4, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	ch32f2x_info->option_bytes.data = ((option_bytes & 0xFF) | (((option_bytes >> 16) & 0xFF) << 8));
	
	/* read write protection option bytes */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_WPR_OFFSET), &ch32f2x_info->option_bytes.protection);
	if (retval != ERROR_OK)
		return retval;

	/* notice read protect status */
	if (ch32f2x_info->option_bytes.rdp != ch32f2x_info->default_rdp)
		LOG_INFO("Device Read Protect Bit Set");

	return ERROR_OK;
}

static int ch32f2x_erase_options(struct flash_bank *bank)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	/* read current options */
	retval = ch32f2x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = ch32f2x_unlock_options_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* check busy */
	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set ober strt bit */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_OBER);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_OBER | CH32F2X_FLASH_CTRL_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;
	
	/* clear read protection option byte
	 * this will also force a device unlock if set */
	ch32f2x_info->option_bytes.rdp = ch32f2x_info->default_rdp;

	/* clr ober bit */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_OBER);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;	

	return ERROR_OK;
}

static int ch32f2x_write_options(struct flash_bank *bank)
{	
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint16_t options_buff;
	uint32_t ctrl;
	int retval = ERROR_OK;

	/* unlock option flash registers */
	retval = ch32f2x_unlock_options_reg(bank);
	if (retval != ERROR_OK)
		return retval;
	
	/* check busy */
	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set obpg strt bit */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_OBPG);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_OBPG | CH32F2X_FLASH_CTRL_STRT);
	if (retval != ERROR_OK)
		return retval;

	/* set read protect */
	options_buff = ch32f2x_info->option_bytes.rdp; 
	retval = target_write_u16(target, CH32F2X_OBR_BANK_BASE, options_buff);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set user data */
	options_buff = ch32f2x_info->option_bytes.user;
	retval = target_write_u16(target, CH32F2X_OBR_BANK_BASE + 2, options_buff);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set user private data */
	options_buff = ch32f2x_info->option_bytes.data & 0xFF;
	retval = target_write_u16(target, CH32F2X_OBR_BANK_BASE + 4, options_buff);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	options_buff = (ch32f2x_info->option_bytes.data >> 8) & 0xFF;
	retval = target_write_u16(target, CH32F2X_OBR_BANK_BASE + 6, options_buff);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set write protect */
	for (uint8_t i = 0; i < 4; i++) {
		options_buff = (ch32f2x_info->option_bytes.protection >> (i * 8)) & 0xFF;
		retval = target_write_u16(target, CH32F2X_OBR_BANK_BASE + 8 + i * 2, options_buff);
		if (retval != ERROR_OK)
			return retval;

		retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	/* clr obpg bit */		
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_OBPG);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ch32f2x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t write_protection, read_protection;
	int retval = ERROR_OK;

	/* medium density - each bit refers to a 8 sector protection block
	 * bit 31 refers to all remaining sectors in a bank */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_WPR_OFFSET), &write_protection);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_OBR_OFFSET), &read_protection);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (write_protection & (1 << i)) ? 0 : 1;

	/* if read protection set first block(0 - 15 sector 4k) is write protected automatic */
	if (read_protection & CH32F2X_FLASH_OBR_RDRRT)
		bank->prot_blocks[0].is_protected = 1;

	return ERROR_OK;
}

static int ch32f2x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock flash registers */
	retval = ch32f2x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* check busy */
	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* set mer strt */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_MER);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_MER | CH32F2X_FLASH_CTRL_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* clr mer */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_MER);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int ch32f2x_page_erase(struct flash_bank *bank, unsigned int *start_page,
				unsigned int end_page, uint32_t size)
{
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	/* run fast erase except 4k erase */
	if (size != 4 * 1024) {
		retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
		if (retval != ERROR_OK)
			return retval;

		if (ctrl & CH32F2X_FLASH_CTRL_FLOCK) {
			retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_MODEKRY_OFFSET), CH32F2X_KEY1);
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_MODEKRY_OFFSET), CH32F2X_KEY2);
			if (retval != ERROR_OK)
				return retval;

			retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
			if (retval != ERROR_OK)
				return retval;
		}
	}	

	/* set erase size */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (size == 64 * 1024) {
		ctrl |= CH32F2X_FLASH_CTRL_BER64;
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == 32 * 1024) {
		ctrl |= CH32F2X_FLASH_CTRL_BER32;
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == 4 * 1024) {
		ctrl |= CH32F2X_FLASH_CTRL_PER;
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == CH32F2X_FLASH_PAGE_SIZE) {
		ctrl |= CH32F2X_FLASH_CTRL_FTER;
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl);
		if (retval != ERROR_OK)
			return retval;
	}

	/* set erase address and start erase loop until remain sector less than current erase size */
	unsigned int count = end_page - *start_page + 1; 
	while (count >= size / CH32F2X_FLASH_PAGE_SIZE) {
		/* set address */
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_ADDR_OFFSET), bank->base + bank->sectors[*start_page].offset);
		if (retval != ERROR_OK)
			return retval;

		/* set strt */
		ctrl |= CH32F2X_FLASH_CTRL_STRT;
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl);
		if (retval != ERROR_OK)
			return retval;

		/* wait erase */
		retval = ch32f2x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		count -= size / CH32F2X_FLASH_PAGE_SIZE;
		*start_page += size / CH32F2X_FLASH_PAGE_SIZE;
	}

	/* clr erase size register */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (size == 64 * 1024) {
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_BER64);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == 32 * 1024) {
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_BER32);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == 4 * 1024) {
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_PER);
		if (retval != ERROR_OK)
			return retval;
	}
	else if (size == CH32F2X_FLASH_PAGE_SIZE) {
		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_FTER);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int ch32f2x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock flash registers */
	retval = ch32f2x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* check busy */
	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	if (first > last)
		return ERROR_TARGET_INVALID;

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return ch32f2x_mass_erase(bank);

	if (last - first + 1 >= 64 * 1024 / CH32F2X_FLASH_PAGE_SIZE) {
		retval = ch32f2x_page_erase(bank, &first, last, 64 * 1024);
		if (retval != ERROR_OK)
			return retval;
	}

	if (last - first + 1 >= 32 * 1024 / CH32F2X_FLASH_PAGE_SIZE) {
		retval = ch32f2x_page_erase(bank, &first, last, 32 * 1024);
		if (retval != ERROR_OK)
			return retval;
	}

	if (last - first + 1 >= 4 * 1024 / CH32F2X_FLASH_PAGE_SIZE) {
		retval = ch32f2x_page_erase(bank, &first, last, 4 * 1024);
		if (retval != ERROR_OK)
			return retval;
	}

	if (last - first + 1 >= 1) {
		retval = ch32f2x_page_erase(bank, &first, last, CH32F2X_FLASH_PAGE_SIZE);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int ch32f2x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32f2x_erase_options(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("ch32f2x failed to erase options");
		return retval;
	}

	for (unsigned int i = first; (i <= last) && (i < 32); i++) {
		if (set)
			ch32f2x_info->option_bytes.protection &= ~(1 << i);
		else
			ch32f2x_info->option_bytes.protection |= (1 << i);
	}

	return ch32f2x_write_options(bank);
}

static int ch32f2x_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
					uint32_t address, uint32_t hwords_count)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct working_area *write_algorithm_stack;
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t ch32f2x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/ch32/ch32f2x.inc"
	};

	/* flash write code */
	retval = target_alloc_working_area(target, sizeof(ch32f2x_flash_write_code), &write_algorithm);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ch32f2x_flash_write_code), ch32f2x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer reserve stack area at the end of ram */
	buffer_size = target_get_working_area_avail(target);
	if (buffer_size > CH32F2X_WRITE_ALGORITHM_STACK) {
		buffer_size -= CH32F2X_WRITE_ALGORITHM_STACK;
	}	
	else {
		LOG_WARNING("no working area available for stack area, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;	
	}

	buffer_size = MIN(hwords_count * 2 + 8, MAX(buffer_size, 256));
	/* Normally we allocate all available working area except stack area.
	 * MIN shrinks buffer_size if the size of the written block is smaller.
	 * MAX prevents using async algo if the available working area is smaller
	 * than 256, the following allocation fails with
	 * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
	 */

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always 32-bit word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* stack area */
	retval = target_alloc_working_area(target, CH32F2X_WRITE_ALGORITHM_STACK, &write_algorithm_stack);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		target_free_working_area(target, write_algorithm_stack);
		LOG_DEBUG("no working area for target algorithm stack");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Transfer target algorithm function parameters.
	 * Sse r0 - r3 transfer the first four parameters, use stack transfer the more than four parameters.
	 * when use stack, the last parameter is pushed onto the stack first, and then the other parameters
	 * are sequentially pushed onto the stack from back to front.
	 * When executing the target algorithm function, the fifth parameter first pops up from the stack.
	 * To ensure the correct operation of the target algorithm function, it is best to confirm the storage
	 * location of the parameters in the .lst file of the target algorithm function.
	 * Tips: The target algorithm function need to be set to __attribute__((naked)) to ensure that the compiler
	 * does not generate any function entry and exit codes, so the programmer can control the all operation 
	 * of the stack independently.
	 * Additionally, it is possible to store data at a fixed address on the target RAM and read the data
	 * at that address in the target algorithm function. However, this is not in line with the general 
	 * application of functions
	 */
	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* Flash register base address and return value */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);	/* count (halfword-16bit) */

	struct mem_param mem_params[1];

	uint32_t stack_top_address;
	/* set stack top 8 byte align */
	stack_top_address = (write_algorithm_stack->address + CH32F2X_WRITE_ALGORITHM_STACK) & ~0x07;
	/* push target algorithm function last parameter to stack */
	init_mem_param(&mem_params[0], stack_top_address - 4, 32, PARAM_OUT);
	/* save last parameter value to current stack point position */	
	buf_set_u32(mem_params[0].value, 0, 32, hwords_count);

	buf_set_u32(reg_params[0].value, 0, 32, ch32f2x_info->register_base);
	buf_set_u32(reg_params[1].value, 0, 32, source->address);
	buf_set_u32(reg_params[2].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[3].value, 0, 32, address);
	buf_set_u32(reg_params[4].value, 0, 32, stack_top_address - 4);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, hwords_count, 2,
			ARRAY_SIZE(mem_params), mem_params,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		/* Actually we just need to check for programming errors
		 * ch32f2x_wait_status_busy also reports error and clears status bits.
		 *
		 * Target algo returns flash status in r0 only if properly finished.
		 * It is safer to re-read status register.
		 */
		int retval2 = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
		if (retval2 != ERROR_OK)
			retval = retval2;

		LOG_ERROR("flash write failed just before address 0x%"PRIx32,
				buf_get_u32(reg_params[3].value, 0, 32));
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	for (unsigned int i = 0; i < ARRAY_SIZE(mem_params); i++)
		destroy_mem_param(&mem_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, write_algorithm_stack);

	return retval;
}

/** Writes a block to flash either using target algorithm
 *  or use fallback, host controlled halfword-by-halfword access.
 *  Flash controller must be unlocked before this call.
 */
static int ch32f2x_write_block(struct flash_bank *bank,
				const uint8_t *buffer, uint32_t address, uint32_t hwords_count)
{
	struct target *target = bank->target;
	uint32_t ctrl;
	int retval = ERROR_OK;

	/* The flash write must be aligned to a halfword boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(address % 2 == 0);

	struct arm *arm = target_to_arm(target);
	if (is_arm(arm)) {
		/* try using a block write - on ARM architecture or... */
		retval = ch32f2x_write_block_async(bank, buffer, address, hwords_count);
	} else {
		/* ... RISC-V architecture */
		LOG_ERROR("RISC-V is currently not supported");
		return ERROR_FAIL;
	}

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		/* set pg */
		retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl | CH32F2X_FLASH_CTRL_PG);
		if (retval != ERROR_OK)
			return retval;

		while (hwords_count) {
			retval = target_write_memory(target, address, 2, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = ch32f2x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
			if (retval != ERROR_OK)
				return retval;

			hwords_count--;
			buffer += 2;
			address += 2;
		}

		/* clr pg */
		retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), &ctrl);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_CTRL_OFFSET), ctrl & ~CH32F2X_FLASH_CTRL_PG);
		if (retval != ERROR_OK)
			return retval;

		retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;	
	}
	return retval;
}

static int ch32f2x_write(struct flash_bank *bank, const uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a halfword boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(offset % 2 == 0);
	assert(count % 2 == 0);

	/* unlock flash registers */
	retval = ch32f2x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_wait_status_busy(bank, FLASH_COMMAND_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* write to flash */
	retval = ch32f2x_write_block(bank, buffer, bank->base + offset, count / 2);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;	
}

struct ch32f2x_property_addr {
	uint32_t device_id;
	uint32_t flash_size;
};

static int ch32f2x_get_property_addr(struct target *target, struct ch32f2x_property_addr *addr)
{
	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	// switch (cortex_m_get_partno_safe(target)) {
	switch (cortex_m_get_impl_part(target)) {
		case CORTEX_M3_PARTNO:
			addr->device_id = CH32F2X_IDCODE_BASE;
			addr->flash_size = CH32F2X_FLASH_INFO;
			return ERROR_OK;
		default:
			LOG_ERROR("Cannot identify target as a ch32f2x");
			return ERROR_FAIL;
	}
}

static int ch32f2x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	struct target *target = bank->target;
	struct ch32f2x_property_addr addr;
	int retval = ERROR_OK;

	retval = ch32f2x_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

	/* 0x20500418 */
	return target_read_u32(target, addr.device_id, device_id);
}

static int ch32f2x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	struct ch32f2x_property_addr addr;
	int retval = ERROR_OK;

	retval = ch32f2x_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

	return target_read_u16(target, addr.flash_size, flash_size_in_kb);
}

static int get_ch32f2x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t dbgmcu_idcode;
	int retval = ERROR_OK;

	retval = ch32f2x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xFFF;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {
		case 0x418:
			device_str = "CH32F2x (Medium Density)";
			switch (rev_id) {
				case 0x2050:
					rev_str = "5";
					break;
				default:
					break;	
			}
			break;
		default:
			command_print_sameline(cmd, "Cannot identify target as a CH32Fx\n");
			return ERROR_FAIL;
	}

	if (rev_str)
		command_print_sameline(cmd, "%s - Rev: %s", device_str, rev_str);
	else
		command_print_sameline(cmd, "%s - Rev: unknown (0x%04x)", device_str, rev_id);	

	return ERROR_OK;
}

static int ch32f2x_probe(struct flash_bank *bank)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t dbgmcu_idcode;
	int page_size;
	int retval = ERROR_OK;

	ch32f2x_info->probed = false;
	ch32f2x_info->register_base = CH32F2X_FLASH_BASE;

	/* default factory not read protection */
	ch32f2x_info->default_rdp = CH32F2X_OBR_KEY;

	/* read ch32f2x device id register */
	retval = ch32f2x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", dbgmcu_idcode);
	
	uint16_t device_id = dbgmcu_idcode & 0xFFF;
	uint16_t rev_id = dbgmcu_idcode >> 16;

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id) {
		case 0x41c:
		case 0x418: 
			page_size = CH32F2X_FLASH_PAGE_SIZE;
			ch32f2x_info->ppage_size = CH32F2X_FLASH_SECTOR_KB * 1024 / CH32F2X_FLASH_PAGE_SIZE;
			switch (rev_id) {
				case 0x2050: 
					max_flash_size_in_kb = 128;
					break;
				default:
					break;
			}	
			break;
		default:
			LOG_WARNING("Cannot identify target as a CH32 family.");
			return ERROR_FAIL;
	}		
	
	/* get flash size from target. */
	retval = ch32f2x_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xFFFF || flash_size_in_kb == 0) {
		LOG_WARNING("CH32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}
	
	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (ch32f2x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = ch32f2x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xFFFF);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = CH32F2X_FLASH_BANK_BASE;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / ch32f2x_info->ppage_size;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, ch32f2x_info->ppage_size * page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_pages - (31 * ch32f2x_info->ppage_size)) * page_size;

	ch32f2x_info->probed = true;

	return ERROR_OK;
}

static int ch32f2x_auto_probe(struct flash_bank *bank)
{
	struct ch32f2x_flash_bank *ch32f2x_info = bank->driver_priv;
	if (ch32f2x_info->probed)
		return ERROR_OK;
	return ch32f2x_probe(bank);
}

COMMAND_HANDLER(ch32f2x_handle_lock_command)
{
	struct target *target = NULL;
	struct ch32f2x_flash_bank *ch32f2x_info = NULL;
	int retval = ERROR_OK;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	ch32f2x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (ch32f2x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	ch32f2x_info->option_bytes.rdp = 0;

	if (ch32f2x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "ch32f2x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(ch32f2x_handle_unlock_command)
{
	struct target *target = NULL;
	int retval = ERROR_OK;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (ch32f2x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to erase options");
		return ERROR_OK;
	}

	if (ch32f2x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to unlock device");
		return ERROR_OK;
	}

	command_print(CMD, "ch32f2x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(ch32f2x_handle_mass_erase_command)
{
	int retval = ERROR_OK;
	
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32f2x_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "ch32f2x mass erase complete");
	else
		command_print(CMD, "ch32f2x mass erase failed");

	return retval;
}

COMMAND_HANDLER(ch32f2x_handle_options_read_command)
{
	uint32_t option_bytes, protection, user_data;
	struct target *target = NULL;
	int retval = ERROR_OK;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read user and read protection option bytes */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_OBR_OFFSET), &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	/* read user data option bytes */
	retval = target_read_u32(target, CH32F2X_OBR_BANK_BASE + 4, &user_data);
	if (retval != ERROR_OK)
		return retval;

	/* read write protection option bytes */
	retval = target_read_u32(target, ch32f2x_get_flash_reg(bank, CH32F2X_FLASH_WPR_OFFSET), &protection);
	if (retval != ERROR_OK)
		return retval;

	if (option_bytes & CH32F2X_FLASH_OBR_OBERR)
		command_print(CMD, "option byte complement error");

	/* ch32f205 ram code mode is not use */
	command_print(CMD, "ram code mode = 0x%01" PRIx8 "", (option_bytes >> 8) & 0x03);

	command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

	command_print(CMD, "read protection: %s",
				(option_bytes & CH32F2X_FLASH_OBR_RDRRT) ? "on" : "off");

	/* user option bytes are offset depending on variant */
	command_print(CMD, "watchdog: %sware",
				(option_bytes & (1 << 2)) ? "soft" : "hard");

	command_print(CMD, "stop mode: %sreset generated upon entry",
				(option_bytes & (1 << 3)) ? "no " : "");

	command_print(CMD, "standby mode: %sreset generated upon entry",
				(option_bytes & (1 << 4)) ? "no " : "");

	command_print(CMD, "user data = 0x%04" PRIx16 "", ((user_data & 0xFF) | (((user_data >> 16) & 0xFF) << 8)));

	return ERROR_OK;
}

COMMAND_HANDLER(ch32f2x_handle_options_write_command)
{
	struct target *target = NULL;
	struct ch32f2x_flash_bank *ch32f2x_info = NULL;
	uint8_t optionbyte;
	uint8_t ram_code_mode;
	uint16_t useropt;
	int retval = ERROR_OK;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	ch32f2x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = ch32f2x_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	/* start with current options */
	optionbyte = ch32f2x_info->option_bytes.user;
	useropt = ch32f2x_info->option_bytes.data;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 0);
		else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 0);
		else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 1);
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 1);
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 2);
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 2);
		else if (strcmp("RAM_CODE_MODE", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], ram_code_mode);
			optionbyte &= ~(0x03 << 6);
			optionbyte |= (ram_code_mode << 6);
			CMD_ARGC--;
			CMD_ARGV++;		
		}
		else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
			CMD_ARGC--;
			CMD_ARGV++;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (ch32f2x_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to erase options");
		return ERROR_OK;
	}

	ch32f2x_info->option_bytes.user = optionbyte;
	ch32f2x_info->option_bytes.data = useropt;

	if (ch32f2x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "ch32f2x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "ch32f2x write options complete.\n"
				"INFO: power cycle is required "
				"for the new settings to take effect.");

	return ERROR_OK;
}

static const struct command_registration ch32f2x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = ch32f2x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = ch32f2x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = ch32f2x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = ch32f2x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = ch32f2x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)"
			"('RAM_CODE_MODE' mode)",
		.help = "Replace bits in device option bytes.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch32f2x_command_handlers[] = {
	{
		.name = "ch32f2x",
		.mode = COMMAND_ANY,
		.help = "ch32f2x flash command group",
		.usage = "",
		.chain = ch32f2x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ch32f2x_flash = {
	.name = "ch32f2x",
	.commands = ch32f2x_command_handlers,
	.flash_bank_command = ch32f2x_flash_bank_command,
	.erase = ch32f2x_erase,
	.protect = ch32f2x_protect,
	.write = ch32f2x_write,
	.read = default_flash_read,
	.probe = ch32f2x_probe,
	.auto_probe = ch32f2x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = ch32f2x_protect_check,
	.info = get_ch32f2x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
