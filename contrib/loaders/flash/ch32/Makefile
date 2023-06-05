# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump


CFLAGS = -c -mthumb -mcpu=cortex-m3 -nostdlib -nostartfiles -Os -g -fPIC

all: ch32f2x.inc

.PHONY: clean

%.elf: %.c
	$(CC) $(CFLAGS) $< -o $@
	
%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc
