/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by MengFanYu                                       *
 *   SecondHandCoder@gmail.com                                             *
 ***************************************************************************/

#include  "stdint.h"

#define CH32F2X_IWDG_BASE 0x40003000
#define CH32F2X_IWDG_UNLOCK 0x5555
#define CH32F2X_IWDG_FEED	0xAAAA

#define CH32F2X_KEY1				0x45670123
#define CH32F2X_KEY2				0xCDEF89AB

/* FLASH_STATR register bits */
#define CH32F2X_FLASH_STATR_BSY			0x00000001
#define CH32F2X_FLASH_STATR_WRBSY		0x00000002
#define CH32F2X_FLASH_STATR_WRPRTERR		0x00000010

/* FLASH_CTRL register bits */
#define CH32F2X_FLASH_CTRL_PG			0x00000001
#define CH32F2X_FLASH_CTRL_LOCK		    0x00000080
#define CH32F2X_FLASH_CTRL_FLOCK		0x00008000
#define CH32F2X_FLASH_CTRL_FTPG			0x00010000
#define CH32F2X_FLASH_CTRL_PGSTRT		0x00200000
#define CH32F2X_FLASH_OBR_IWDG_SW		0x00000004

#define CH32F2X_FLASH_STATR_REG			(*(volatile uint32_t *)(flash_regs_base + 0x0C))
#define CH32F2X_FLASH_CTRL_REG			(*(volatile uint32_t *)(flash_regs_base + 0x10))
#define CH32F2X_FLASH_MODEKEY_REG		(*(volatile uint32_t *)(flash_regs_base + 0x24))
#define CH32F2X_FLASH_OBR_REG			(*(volatile uint32_t *)(flash_regs_base + 0x1C))

#define CH32F2X_IWDG_CTLR_REG		(*(volatile uint16_t *)(CH32F2X_IWDG_BASE + 0x00))
#define CH32F2X_IWDG_PSCR_REG		(*(volatile uint16_t *)(CH32F2X_IWDG_BASE + 0x04))
#define CH32F2X_IWDG_RLDR_REG		(*(volatile uint16_t *)(CH32F2X_IWDG_BASE + 0x08))

/* wait data fifo value valid if wp = 0, write zero to r0 */
#define WAIT_FIFO								\
do {										\
	while (wp == rp) {							\
		wp = *(uint32_t *)buffer_start;					\
		if (wp == 0) {							\
			flash_regs_base = 0;					\
			goto exit;						\
		}								\
		rp = *(uint32_t *)(buffer_start + 4);				\
	}									\
} while (0)

/* wait write busy, if error set rp to zero and write status register value to r0 */
#define WAIT_BUSY								\
do {											\
	while (CH32F2X_FLASH_STATR_REG & CH32F2X_FLASH_STATR_BSY);		\
	if (CH32F2X_FLASH_STATR_REG & CH32F2X_FLASH_STATR_WRPRTERR) {		\
		*(uint32_t *)(buffer_start + 4) = 0;				\
		flash_regs_base = CH32F2X_FLASH_STATR_REG;			\
		goto exit;							\
	}									\
} while(0)


__attribute__((naked)) void flash_write(uint32_t flash_regs_base,
					uint8_t *buffer_start,
					uint8_t *buffer_end,
					uint8_t *target_addr,
					uint32_t hwords_count)
{
	uint32_t wp = 0, rp = 0;

	// Check if IWDG is enabled
	if((CH32F2X_FLASH_OBR_REG & CH32F2X_FLASH_OBR_IWDG_SW) == 0x00){
		// Set IWDG time out to ~32.768 second
		CH32F2X_IWDG_CTLR_REG = CH32F2X_IWDG_UNLOCK;
		CH32F2X_IWDG_PSCR_REG = 0x06;
		CH32F2X_IWDG_RLDR_REG = 0xfff;
	}
	
	while (hwords_count) {
		CH32F2X_IWDG_CTLR_REG = CH32F2X_IWDG_FEED;
		/* target_addr 256byte align and remain bytes greater than 256 fast page write */
		if ((((uint32_t)target_addr & 0xFF) == 0) && (hwords_count >= 0x200)) {
			CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_PG;
			if (CH32F2X_FLASH_CTRL_REG & CH32F2X_FLASH_CTRL_LOCK) {
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY1;
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY2;
			}
			if (CH32F2X_FLASH_CTRL_REG & CH32F2X_FLASH_CTRL_FLOCK) {
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY1;
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY2;
			}

			WAIT_BUSY;
			
			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_FTPG;

			/* write 64*4 byte data */	
			for (uint8_t i = 0; i < 64; i++) {
				WAIT_FIFO;
				*(uint32_t *)target_addr = *(uint32_t *)rp;
				
				/* rp and target_addr add 4 bytes */
				rp += 4;
				target_addr += 4;

				while (CH32F2X_FLASH_STATR_REG & CH32F2X_FLASH_STATR_WRBSY);
				if (rp >= (uint32_t)buffer_end)
					rp = (uint32_t)buffer_start + 8;
				
				*(uint32_t *)(buffer_start + 4) = rp;
				hwords_count -= 2;	
			}

			/* start write */
			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_PGSTRT;
			WAIT_BUSY;	
		}
		/* halfword write */
		else {
			CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_FTPG;	
			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_PG;

			WAIT_FIFO;
			*(uint16_t *)target_addr = *(uint16_t *)rp;

			/* rp and target_addr add 2 bytes */
			rp += 2;
			target_addr += 2;

			WAIT_BUSY;
			if (rp >= (uint32_t)buffer_end)
				rp = (uint32_t)buffer_start + 8;
			
			*(uint32_t *)(buffer_start + 4) = rp;
			hwords_count -= 1;	
		}	
	}
exit:
	CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_PG;
	CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_FTPG;
	__asm("bkpt #0");
}
