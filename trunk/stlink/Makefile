PRGS := stlink-download
DEV := /dev/sg2

all: $(PRGS)

LIBS :=	-lsgutils2
CFLAGS = -O -Wall -fmessage-length=0 -std=gnu99 -MMD -MP
LDFLAGS := -lsgutils2

# We use the ARM compiler only to compile the flash-write function.
# The output is hand tuned and used in the stlink-download program.
ARMCC = arm-none-eabi-gcc 
ARMCFLAGS = -Os -fconserve-stack -fcaller-saves -mcpu=cortex-m3 -mthumb \
    -Wa,-adhlns=flash.lst
ARMCFLAGS+= -Wa,-adhlns=$(<:.c=.lst)


#$(PRG): $(OBJS)
#	@echo 'Invoking: GCC C Linker'
#	gcc  -o $(PRG) $(OBJS) $(LIBS)

#%.o: %.c
#	@echo 'Building file: $<'
#	gcc -O0 -g3 -Wall -c -fmessage-length=0 -std=gnu99 -MMD -MP \
#		-MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"\
#		-o"$@" "$<"

all: stlink-download flash-transfer.lst

flash-transfer.lst: flash-transfer.c
	$(ARMCC) $(ARMCFLAGS) -c $< -Wa,-adhlns=$(<:.c=.lst)

clean:
	@rm -vf *.d *.o *.lst *.s $(PRGS)

run: all
#	cp $(PRG) /tmp/
#	sudo /tmp/$(PRG) $(DEV)
