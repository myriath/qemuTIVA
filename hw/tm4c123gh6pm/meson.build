tm4_ss = ss.source_set()

tm4_ss.add(files('../arm/boot.c'), fdt)
tm4_ss.add(when: 'CONFIG_ARM_V7M', if_true: files('../arm/armv7m.c'))

tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('board/src/tm4c123gh6pm.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('board/src/i2c.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('board/src/adc.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('board/src/gpio.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('board/src/timer.c'))

hw_arch += {'tm4': tm4_ss}
