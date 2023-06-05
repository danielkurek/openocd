/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by mengfanyu                              		*
 *   SecondHandCoder@gmail.com                                          *
 ***************************************************************************/
#define CH32F2X_KEY1           				0x45670123
#define CH32F2X_KEY2           				0xCDEF89AB

#define CH32F2X_FLASH_BASE    				0x40022000

/* FLASH_STATR register bits */
#define CH32F2X_FLASH_STATR_BSY      		0x00000001
#define CH32F2X_FLASH_STATR_WRPRTERR 		0x00000010

/* FLASH_CTRL register bits */
#define CH32F2X_FLASH_CTRL_PG      	 		0x00000001
#define CH32F2X_FLASH_CTRL_FLOCK   	 		0x00008000
#define CH32F2X_FLASH_CTRL_FTPG   	 		0x00010000
#define CH32F2X_FLASH_CTRL_PGSTRT    		0x00200000

#define CH32F2X_FLASH_STATR_REG				(*(volatile uint32_t *)(CH32F2X_FLASH_BASE + 0x0C))
#define CH32F2X_FLASH_CTRL_REG				(*(volatile uint32_t *)(CH32F2X_FLASH_BASE + 0x10))
#define CH32F2X_FLASH_MODEKEY_REG			(*(volatile uint32_t *)(CH32F2X_FLASH_BASE + 0x24))

#include  "stdint.h"

#define WAIT_FIFO															\
do {																		\
	while (wp == rp) {														\
		wp = *buffer_start;													\
		if (wp == 0) {														\
			hwords_count = 0;												\
			goto exit;														\
		}																	\
		rp = *(buffer_start + 4);											\
	}																		\
} while (0)

#define WAIT_BUSY															\
do {																		\
	while(CH32F2X_FLASH_STATR_REG & CH32F2X_FLASH_STATR_BSY);				\
	if (CH32F2X_FLASH_STATR_REG & CH32F2X_FLASH_STATR_WRPRTERR) {			\
		*(buffer_start + 4) = 0;											\
		hwords_count = CH32F2X_FLASH_STATR_REG;								\
		goto exit;															\
	}																		\
} while(0)


__attribute__((naked)) void flash_write(uint32_t hwords_count,
										uint32_t *buffer_start,
										uint32_t *buffer_end,
										register uint32_t *target_addr)
{
	while (hwords_count > 0) {
		register uint32_t rp = 0, wp = 0;
		if ((((uint32_t)target_addr & 0xFF) == 0) && (hwords_count >= 0x100)) {
			CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_PG;
			if (CH32F2X_FLASH_CTRL_REG & CH32F2X_FLASH_CTRL_FLOCK) {
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY1;
				CH32F2X_FLASH_MODEKEY_REG = CH32F2X_KEY2;
			}
			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_FTPG;

			do {
				WAIT_FIFO;
				*target_addr = *(uint32_t *)rp;

				rp += 4;
				target_addr += 4;

				WAIT_BUSY;
				if (rp >= (uint32_t)buffer_end)
					rp = (uint32_t)buffer_start + 8;
				
				*(buffer_start + 4) = rp;
				hwords_count -= 2;	
			} while((uint32_t)target_addr & 0xFF); 

			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_PGSTRT;
			WAIT_BUSY;	
		}
		else {
			CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_FTPG;	
			CH32F2X_FLASH_CTRL_REG |= CH32F2X_FLASH_CTRL_PG;

			WAIT_FIFO;
			*(uint16_t *)target_addr = *(uint16_t *)rp;

			rp += 2;
			target_addr += 2;

			WAIT_BUSY;
			if (rp >= (uint32_t)buffer_end)
				rp = (uint32_t)buffer_start + 8;
			
			*(buffer_start + 4) = rp;
			hwords_count -= 1;	
		}	
	}
	hwords_count = CH32F2X_FLASH_STATR_REG;			
exit:
	CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_PG;
	CH32F2X_FLASH_CTRL_REG &= ~CH32F2X_FLASH_CTRL_FTPG;
	__asm("bkpt #0");
}
