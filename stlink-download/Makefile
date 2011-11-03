#!/bin/make
# Makefile rules for the stlink-download utility.
# Written 2010,2011 by Donald Becker and William Carlson

PRGS := stlink-download stlinkv2-util
DEV := /dev/stlink

all: $(PRGS)

CFLAGS = -O -Wall -Wstrict-prototypes

# We use the ARM compiler only to compile the flash-write function.
# The output is hand tuned and used in the stlink-download program.
ARMCC = arm-none-eabi-gcc 
ARMCFLAGS = -Os -fconserve-stack -fcaller-saves -mcpu=cortex-m3 -mthumb \
    -Wa,-adhlns=flash.lst
ARMCFLAGS+= -Wa,-adhlns=$(<:.c=.lst)

stlink-download: stlink-download.c
stlinkv2-util: stlinkv2-util.c
	$(CC) $(CFLAGS) -o $@ $< -lusb-1.0

flash-transfer.lst: flash-transfer.c
	$(ARMCC) $(ARMCFLAGS) -c $< -Wa,-adhlns=$(<:.c=.lst)

# You will likely need root permission to do this e.g. sudo
install: stlink-download
	install -C -o root -g root -m 644 10-stlink.rules /etc/udev/rules.d/
	install -m 755 $(PRGS) /usr/bin/

tar: stlink.tgz
stlink.tgz: Makefile 10-stlink.rules stlink-download.c flash-transfer.c
	tar cfvz $@ $^
clean:
	rm -f *.d *.o *.lst *.s $(PRGS)

run: all
#	cp $(PRG) /tmp/
#	sudo /tmp/$(PRG) $(DEV)
