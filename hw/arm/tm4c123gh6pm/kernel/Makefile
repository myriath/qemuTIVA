INCLUDE=-Iinclude

build: bin/kernel.sys

run: bin/kernel.sys
	../../../../build/qemu-system-arm -nographic -semihosting -machine tm4c123gh6pm_debug -kernel "bin/kernel.sys"

run_cybot: bin/kernel.sys
	../../../../build/qemu-system-arm -nographic -semihosting -machine cybot_debug -kernel bin/kernel.sys

bin/kernel.sys: bin/kernel.elf bin
	arm-none-eabi-objcopy -O binary bin/kernel.elf bin/kernel.sys

bin/kernel.elf: src/tm4c123gh6pm.ld bin/main.o bin/kernel.o bin
	arm-none-eabi-ld -T src/tm4c123gh6pm.ld bin/main.o bin/kernel.o -o bin/kernel.elf

bin/main.o: src/main.c bin
	arm-none-eabi-gcc $(INCLUDE) -c -mcpu=cortex-m4 src/main.c -o bin/main.o

bin/kernel.o: src/tm4c123gh6pm_kernel.c bin
	arm-none-eabi-gcc $(INCLUDE) -c -mcpu=cortex-m4 src/tm4c123gh6pm_kernel.c -o bin/kernel.o

dbg: bin/kernel.sys.g
	../../../../build/qemu-system-arm -nographic -semihosting -machine tm4c123gh6pm_debug -kernel "bin/kernel.sys.g" -d in_asm,int,exec,cpu,guest_errors,unimp -s -S
	
dbg_cybot: bin/kernel.sys.g
	../../../../build/qemu-system-arm -nographic -semihosting -machine cybot_debug -kernel bin/kernel.sys.g -d in_asm,int,exec,cpu,guest_errors,unimp -s -S

bin/kernel.sys.g: bin/kernel.elf.g bin
	arm-none-eabi-objcopy -O binary bin/kernel.elf.g bin/kernel.sys.g

bin/kernel.elf.g: src/tm4c123gh6pm.ld bin/main.o.g bin/kernel.o.g bin
	arm-none-eabi-ld -g -T src/tm4c123gh6pm.ld bin/main.o.g bin/kernel.o.g -o bin/kernel.elf.g

bin/main.o.g: src/main.c bin
	arm-none-eabi-gcc $(INCLUDE) -g -c -mcpu=cortex-m4 src/main.c -o bin/main.o.g

bin/kernel.o.g: src/tm4c123gh6pm_kernel.c bin
	arm-none-eabi-gcc $(INCLUDE) -g -c -mcpu=cortex-m4 src/tm4c123gh6pm_kernel.c -o bin/kernel.o.g
	
bin:
	mkdir bin

clean:
	rm bin/*