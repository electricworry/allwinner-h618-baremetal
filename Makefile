export PATH := $(PWD)/external/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf/bin:$(PATH)
TOOLCHAIN=external/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf/bin
CC=$(TOOLCHAIN)/aarch64-none-elf-gcc
OBJCOPY=$(TOOLCHAIN)/aarch64-none-elf-objcopy

.PHONY: test-edid clean install

install: src/os.bin external/u-boot/spl/sunxi-spl.bin
	sunxi-fel spl external/u-boot/spl/sunxi-spl.bin write 0x40000000 src/os.bin reset64 0x40000000

$(CC) $(OBJCOPY):
	utils/download.sh
	cd external && tar -xf arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf.tar.xz

clean:
	make -C src clean
	cd external/trusted-firmware-a && make clean
	cd external/u-boot && make clean && rm -f bl31.bin

src/os.bin: $(CC) $(OBJCOPY)
	make -C src

external/u-boot/spl/sunxi-spl.bin: external/u-boot/bl31.bin $(CC)
	cd external/u-boot && \
	    make orangepi_zero3_defconfig && \
		CROSS_COMPILE=aarch64-none-elf- make -j`nproc`

external/u-boot/bl31.bin:
	cd external/trusted-firmware-a && \
		make PLAT=sun50i_h616 DEBUG=1 bl31
	cd external/u-boot && \
		ln -s ../trusted-firmware-a/build/sun50i_h616/debug/bl31.bin

test-edid:
	make -C src edid-test
	./src/edid-test
