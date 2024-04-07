#ifndef TM4_ADC_H_
#define TM4_ADC_H_

#define TM4_ADC_EM_CONTROLLER 0
#define TM4_ADC_EM_COMP       1
#define TM4_ADC_EM_EXTERNAL   4
#define TM4_ADC_EM_TIMER      5
#define TM4_ADC_EM_PWM0       6
#define TM4_ADC_EM_PWM1       7
#define TM4_ADC_EM_PWM2       8

#define TM4_ADC_FIFO_EMPTY    0x0100
#define TM4_ADC_FIFO_FULL     0x1000

#define TYPE_TM4_ADC "tm4-adc"

static void adc_register_types(void);

#endif
