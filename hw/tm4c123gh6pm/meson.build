tm4_ss = ss.source_set()

tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('tm4c123gh6pm.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('i2c.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('adc.c'))
tm4_ss.add(when: 'CONFIG_TM4C123GH6PM', if_true: files('timer.c'))

hw_arch += {'arm': tm4_ss}
