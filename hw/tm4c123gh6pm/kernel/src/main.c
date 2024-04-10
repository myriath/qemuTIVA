#include "main.h"

int a = 0;

void handle_adc(void)
{
    while (~ADC0_SSFSTAT0_R & 0x100) {
        a = ADC0_SSFIFO0_R;
    }
}

void hard_fault(void)
{
    a = -1;
    ExitQEMU();
}

/**
 * Main function. Write your code here
*/
int main() 
{
    IntMasterEnable();
    IntEnable(INT_ADC0SS0);
    IntRegister(INT_ADC0SS0, handle_adc);
    IntRegister(3, hard_fault);
    ADC0_SSMUX0_R = 0x00001234;
    ADC0_SSCTL0_R = 0x00000006;

    ADC0_IM_R = 1;

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
