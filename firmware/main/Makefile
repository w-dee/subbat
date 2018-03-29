DEVICE  = atmega328p
F_CPU   = 8000000	# in Hz
# for mega328p
FUSE_X  = 0xf9 # 0b11111100
FUSE_L  = 0xe2 # 0b11100010
FUSE_H  = 0xdf # 0b11011001


HIDSPX  = sudo /workspace/avr/hidspx -d1 -ph
CFLAGS  = -I. -Wl,-u,vfscanf -lscanf_flt -Wl,-u,vfprintf -lprintf_flt -lm -std=gnu99
OBJECTS = main.o TinyI2CMaster.o bme280.o

COMPILE = avr-gcc -g -Wall -Os  -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ....... to build main.hex"
	@echo "make program ... to flash fuses and firmware"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash the firmware (use this on metaboard)"
	@echo "make clean ..... to delete objects and hex file"

hex: main.hex

program: flash fuse

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(HIDSPX) -fx$(FUSE_X)
	$(HIDSPX) -fh$(FUSE_H)
	$(HIDSPX) -fl$(FUSE_L)

# rule for uploading firmware:
flash: main.hex
	$(HIDSPX) main.hex

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf  *.o  *.lst

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

main.elf: $(OBJECTS)
	$(COMPILE) \
		-o main.elf $(OBJECTS)


main.hex: main.elf
	rm -f main.hex main.eep.hex
	avr-objdump -S main.elf > main.elf.lst
	avr-objcopy -j .text -j .data \
		-O ihex main.elf main.hex
	avr-size main.hex

# debugging targets:

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
