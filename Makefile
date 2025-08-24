CC=aarch64-none-elf-gcc
OBJCOPY=aarch64-none-elf-objcopy
# CFLAGS=-T linker.ld -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a7 -fpic -ffreestanding -O3 -nostdlib -Wextra
CFLAGS=-T linker.ld -mcpu=cortex-a53 -O0 -Wextra

os.bin: os.elf
	$(OBJCOPY) -O binary --remove-section .uncached os.elf os.bin
os.elf: linker.ld boot.o main.o uart.o ports.o mmu.o system.o display.o interrupts.o spritelayers.o usb.o demo.o
	$(CC) $(CFLAGS) -o os.elf boot.o main.o uart.o ports.o mmu.o system.o display.o interrupts.o spritelayers.o usb.o demo.o

boot.o: boot.s
	$(CC) $(CFLAGS) -c boot.s
main.o: main.c
	$(CC) $(CFLAGS) -c main.c
uart.o: uart.c
	$(CC) $(CFLAGS) -c uart.c
ports.o: ports.c
	$(CC) $(CFLAGS) -c ports.c
mmu.o: mmu.c
	$(CC) $(CFLAGS) -c mmu.c
system.o: system.c
	$(CC) $(CFLAGS) -c system.c
display.o: display.c
	$(CC) $(CFLAGS) -c display.c
interrupts.o: interrupts.c
	$(CC) $(CFLAGS) -c interrupts.c
spritelayers.o: spritelayers.c
	$(CC) $(CFLAGS) -c spritelayers.c
usb.o: usb.c
	$(CC) $(CFLAGS) -c usb.c
demo.o: demo.c
	$(CC) $(CFLAGS) -c demo.c

clean:
	rm -f *.o os.*

install: os.bin ../u-boot/spl/sunxi-spl.bin
	sunxi-fel spl ../u-boot/spl/sunxi-spl.bin write 0x40000000 os.bin exe 0x40000000

../u-boot/spl/sunxi-spl.bin:
	cd .. && make u-boot/u-boot-sunxi-with-spl.bin
