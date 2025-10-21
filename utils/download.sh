#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DOWNLOAD_DIR=$(realpath $SCRIPT_DIR/../external)
mkdir -p $DOWNLOAD_DIR
echo Downloading files to $DOWNLOAD_DIR...

FILE=$DOWNLOAD_DIR/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf.tar.xz
if [ -f $FILE ]; then
    if [ "$(sha256sum $FILE | awk '{print $1}')" != "ebaf2d47f2e7f7b645864c5c8cf839e526daed83a2e675a3525d03f5ba3d2be9" ]; then
        rm $FILE
    fi
fi
if [ ! -f $FILE ]; then
    wget -O $FILE https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-elf.tar.xz
fi
