#include "main.h"

int a = 0;

void handle_adc0(void)
{
    ADC0_ISC_R |= 1;
    while (~ADC0_SSFSTAT0_R & 0x100) {
        GPIO_PORTC_DATA_R = ADC0_SSFIFO0_R >> 8;
        GPIO_PORTC_DATA_R = ADC0_SSFIFO0_R & 0xff;
    }
}

void handle_adc1(void)
{
    ADC1_ISC_R |= 1;
    while (~ADC1_SSFSTAT0_R & 0x100) {
        GPIO_PORTA_DATA_R = ADC1_SSFIFO0_R >> 8;
        GPIO_PORTA_DATA_R = ADC1_SSFIFO0_R & 0xff;
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
    IntRegister(INT_ADC1SS0, handle_adc1);
    IntEnable(INT_ADC0SS0);
    IntRegister(INT_ADC0SS0, handle_adc0);
    IntRegister(3, hard_fault);
    
    GPIO_PORTB_DEN_R = 0;
    GPIO_PORTD_DEN_R = 0;
    GPIO_PORTE_DEN_R = 0;

    GPIO_PORTC_DEN_R = 0xff;
    GPIO_PORTC_DIR_R = 0xff;
    GPIO_PORTA_DEN_R = 0xff;
    GPIO_PORTA_DIR_R = 0xff;
    // GPIO_PORTA_DATA_R = 'h';
    // GPIO_PORTA_DATA_R = 'e';
    // GPIO_PORTA_DATA_R = 'l';
    // GPIO_PORTA_DATA_R = 'l';
    // GPIO_PORTA_DATA_R = 'o';
    // GPIO_PORTA_DATA_R = ' ';
    // GPIO_PORTA_DATA_R = 'w';
    // GPIO_PORTA_DATA_R = 'o';
    // GPIO_PORTA_DATA_R = 'r';
    // GPIO_PORTA_DATA_R = 'l';
    // GPIO_PORTA_DATA_R = 'd';
    // GPIO_PORTA_DATA_R = '!';
    // GPIO_PORTA_DATA_R = '\n';

    GPIO_PORTB_AMSEL_R = 0xff;
    GPIO_PORTD_AMSEL_R = 0xff;
    GPIO_PORTE_AMSEL_R = 0xff;

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
    // ADC0_PSSI_R = 1;

    while (~ADC1_SSFSTAT0_R & 0x100);
    while (~ADC0_SSFSTAT0_R & 0x100);

    // Force hard fault for testing interrupt
    __asm__("b 0x30000000");
    return 0;
}
