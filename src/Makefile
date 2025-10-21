TOOLCHAIN=../external/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf/bin
CC=$(TOOLCHAIN)/aarch64-none-elf-gcc
OBJCOPY=$(TOOLCHAIN)/aarch64-none-elf-objcopy
# CFLAGS=-T linker.ld -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a7 -fpic -ffreestanding -O3 -nostdlib -Wextra
CFLAGS=-T linker.ld -mcpu=cortex-a53 -O0 -Wextra

.PHONY: clean

os.bin: os.elf
	$(OBJCOPY) -O binary --remove-section .uncached $^ $@
os.elf: boot.o main.o uart.o ports.o system.o display.o interrupts.o edid.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

boot.o: boot.s
	$(CC) $(CFLAGS) -o $@ -c $^

clean:
	rm -f *.o os.*

edid-test: edid.c edid-test.c
	gcc -o $@ $^ -lm
