#!/bin/bash
# Run from build folder

# build qemu
../configure --target-list=arm-softmmu
make