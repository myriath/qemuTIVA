#include "main.h"

int a = 0;

void handle_adc(void)
{
    ADC0_ISC_R |= 1;
    while (~ADC0_SSFSTAT0_R & 0x100) {
        a = ADC0_SSFIFO0_R;
    }
}

void hard_fault(void)
{
    a = -1;
    ExitQEMU();
}

#define TEST_ANALOG_VOLTAGE (*((volatile uint32_t *)0x40002000))
#define TEST_ANALOG_AIN_SELECT (*((volatile uint32_t *)0x40002004))

/**
 * Main function. Write your code here
*/
int main() 
{
    IntMasterEnable();
    IntEnable(INT_ADC1SS0);
    IntRegister(INT_ADC1SS0, handle_adc);
    IntEnable(INT_ADC0SS0);
    IntRegister(INT_ADC0SS0, handle_adc);
    IntRegister(3, hard_fault);
    
    ADC1_SSMUX0_R = 0x00001234;
    ADC1_SSCTL0_R = 0x00006000;
    ADC1_IM_R = 1;

    ADC0_SSMUX0_R = 0x00006789;
    ADC0_SSCTL0_R = 0x00006000;
    ADC0_IM_R = 1;

    TEST_ANALOG_AIN_SELECT = 4;
    TEST_ANALOG_VOLTAGE = 3300;

    TEST_ANALOG_AIN_SELECT = 3;
    TEST_ANALOG_VOLTAGE = 3000;
    
    TEST_ANALOG_AIN_SELECT = 2;
    TEST_ANALOG_VOLTAGE = 2000;

    TEST_ANALOG_AIN_SELECT = 1;
    TEST_ANALOG_VOLTAGE = 1000;

    TEST_ANALOG_AIN_SELECT = 9;
    TEST_ANALOG_VOLTAGE = 100;

    TEST_ANALOG_AIN_SELECT = 8;
    TEST_ANALOG_VOLTAGE = 1234;
    
    TEST_ANALOG_AIN_SELECT = 7;
    TEST_ANALOG_VOLTAGE = 694;

    TEST_ANALOG_AIN_SELECT = 6;
    TEST_ANALOG_VOLTAGE = 2515;

    ADC1_PSSI_R = 1;
    ADC0_PSSI_R = 1;

    int i;
    int j = 0;
    for (i = 0; i < 1000; i++) {
        // busy loop to allow adc interrupt
        j += 2;
    }

    // Force hard fault for testing interrupt
    __asm__("b 0x30000000");
    return 0;
}
