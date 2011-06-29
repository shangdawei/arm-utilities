#!/usr/bin/make
# ARM build environment for Qar code
# Written 2010/2011 by WWC and DB
# Released under the GPL v2

STLINK-DOWNLOAD=~/qmc/stlink/stlink-download

ifeq "$(wildcard /usr/bin/arm-none-eabi-gcc)" ""
ARM=arm-linux-gnueabi
else
ARM=arm-none-eabi
endif

CC=$(ARM)-gcc
LD=$(ARM)-ld
ARMCC=$(ARM)-gcc
ARMLD=$(ARM)-ld
ARMAR=$(ARM)-ar
ARMAS=$(ARM)-as
OBJCOPY=$(ARM)-objcopy

ASFLAGS= -g
LDFLAGS= -T../armduino/stm32.ld -nostartfiles
#LDFLAGS=-Tstm32f100.ld -L../lib
CFLAGS=-fno-common -fconserve-stack -fcaller-saves -mcpu=cortex-m3 -mthumb -I.
CFLAGS+=-g -O2 -Wall -Wstrict-prototypes
CFLAGS+=-I../armduino/ -DSTM32=1

INC = stm.h discovery.h
STMINC = armduino.h
OTHERLIB =
OTHEROBJ =
LIB =

PREFIX=..

%.lst: %.c
	$(ARMCC) $(CFLAGS) -c $< -Wa,-adhlns=$(<:.c=.lst)
%.elf: %.o
	$(ARMLD) $(LDFLAGS) $< -o $@
%.bin: %.elf
	$(OBJCOPY) $< -O binary $@

# Other interesting options:     -ffreestanding -nostdlib -nostdinc

# Disassemble
ARM-OBJDUMP=arm-none-eabi-objdump
ARM-DISASM=$(ARM-OBJDUMP)  -marm -Mthumb -EL -b binary -D

all: crt-stm32.o printf.o
crt-stm32.o: armduino.h ARM-core.h

clean:
	rm -f *.o *.elf *.bin
