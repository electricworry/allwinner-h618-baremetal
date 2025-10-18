TOOLCHAIN=../extern/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf/bin
CC=$(TOOLCHAIN)/aarch64-none-elf-gcc
OBJCOPY=$(TOOLCHAIN)/aarch64-none-elf-objcopy
# CFLAGS=-T linker.ld -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a7 -fpic -ffreestanding -O3 -nostdlib -Wextra
CFLAGS=-T linker.ld -mcpu=cortex-a53 -O0 -Wextra

.PHONY: edid-test

os.bin: os.elf
	$(OBJCOPY) -O binary --remove-section .uncached os.elf os.bin
os.elf: linker.ld boot.o main.o uart.o ports.o system.o display.o interrupts.o edid.o
	$(CC) $(CFLAGS) -o os.elf boot.o main.o uart.o ports.o system.o display.o interrupts.o edid.o -lm

boot.o: boot.s
	$(CC) $(CFLAGS) -c boot.s
main.o: main.c
	$(CC) $(CFLAGS) -c main.c
uart.o: uart.c
	$(CC) $(CFLAGS) -c uart.c
ports.o: ports.c
	$(CC) $(CFLAGS) -c ports.c
system.o: system.c
	$(CC) $(CFLAGS) -c system.c
display.o: display.c
	$(CC) $(CFLAGS) -c display.c
edid.o: edid.c
	$(CC) $(CFLAGS) -c edid.c

clean:
	rm -f *.o os.*

install: os.bin ../u-boot/spl/sunxi-spl.bin
	sunxi-fel spl ../u-boot/spl/sunxi-spl.bin write 0x40000000 os.bin reset64 0x40000000

../u-boot/spl/sunxi-spl.bin:
	cd .. && make u-boot/u-boot-sunxi-with-spl.bin

edid-test:
	gcc -o edid-test edid.c edid-test.c -lm
	./edid-test