#include "main.h"

int a = 0;

void uart_write(int data) {
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = data;
}

void handle_adc0(void)
{
    int adc;
    ADC0_ISC_R |= 1;
    while (~ADC0_SSFSTAT0_R & 0x100) {
        adc = ADC0_SSFIFO0_R;
        uart_write(adc >> 8);
        uart_write(adc & 0xff);
    }
}

void handle_adc1(void)
{
    int adc;
    ADC1_ISC_R |= 1;
    while (~ADC1_SSFSTAT0_R & 0x100) {
        adc = ADC1_SSFIFO0_R;
        uart_write(adc >> 8);
        uart_write(adc & 0xff);
    }
}

void handle_uart1(void)
{
    int uart;
    if (UART1_MIS_R & UART_MIS_RXMIS) {
        while (!(UART1_FR_R & UART_FR_RXFE)) {
            // read uart while fifo not empty
            uart = UART1_DR_R;
        }
        UART1_ICR_R |= UART_ICR_RXIC;
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
    // ADC GPIO
    GPIO_PORTD_DEN_R = 0;
    GPIO_PORTE_DEN_R = 0;

    GPIO_PORTD_AMSEL_R = 0xff;
    GPIO_PORTE_AMSEL_R = 0xff;

    // UART GPIO
    GPIO_PORTA_DEN_R = 0xff;
    GPIO_PORTA_DIR_R = 0xfe; // PA0 is U0Rx (input)
    GPIO_PORTA_PCTL_R = 0x00000011;
    GPIO_PORTA_AFSEL_R = 0x03;

    // Port B used for both UART1 and AIN10-11
    GPIO_PORTB_DEN_R = 0x03;
    GPIO_PORTB_DIR_R = 0xfe;
    GPIO_PORTB_PCTL_R = 0x00000011;
    GPIO_PORTB_AFSEL_R = 0x03;
    GPIO_PORTB_AMSEL_R = 0x30;
    
    // Generic GPIO
    GPIO_PORTC_DEN_R = 0xff;
    GPIO_PORTC_DIR_R = 0xff;

    // Send 'hello world\n' through GPIO C
    GPIO_PORTC_DATA_R = 'h';
    GPIO_PORTC_DATA_R = 'e';
    GPIO_PORTC_DATA_R = 'l';
    GPIO_PORTC_DATA_R = 'l';
    GPIO_PORTC_DATA_R = 'o';
    GPIO_PORTC_DATA_R = ' ';
    GPIO_PORTC_DATA_R = 'w';
    GPIO_PORTC_DATA_R = 'o';
    GPIO_PORTC_DATA_R = 'r';
    GPIO_PORTC_DATA_R = 'l';
    GPIO_PORTC_DATA_R = 'd';
    GPIO_PORTC_DATA_R = '!';
    GPIO_PORTC_DATA_R = '\n';

    // Configure ADC
    ADC1_SSMUX0_R = 0x00001234;
    ADC1_SSCTL0_R = 0x00006000;
    ADC1_IM_R = 1;

    ADC0_SSMUX0_R = 0x00006789;
    ADC0_SSCTL0_R = 0x00006000;
    ADC0_IM_R = 1;

    // Configure UART
    UART0_CTL_R &= ~0x1;
    UART0_CTL_R = UART_CTL_TXE;
    // UART0_IBRD_R = 104;
    // UART0_FBRD_R = 11;
    UART0_IBRD_R = 100000;
    UART0_FBRD_R = 11;
    UART0_CC_R = 0;
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2 | UART_LCRH_EPS | UART_LCRH_PEN | UART_LCRH_FEN;
    UART0_ICR_R |= UART_ICR_RXIC | UART_ICR_TXIC;
    UART0_IM_R |= UART_IM_RXIM | UART_IM_TXIM;
    UART0_CTL_R |= 1;

    UART1_CTL_R &= ~0x1;
    UART1_CTL_R = UART_CTL_RXE;
    // UART1_IBRD_R = 104;
    // UART1_FBRD_R = 11;
    UART1_IBRD_R = 100000;
    UART1_FBRD_R = 11;
    UART1_CC_R = 0;
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2 | UART_LCRH_EPS | UART_LCRH_PEN | UART_LCRH_FEN;
    UART1_ICR_R |= UART_ICR_RXIC | UART_ICR_TXIC;
    UART1_IM_R |= UART_IM_RXIM | UART_IM_TXIM;
    UART1_CTL_R |= 1;
    
    // Setup interrupts
    IntMasterEnable();

    IntEnable(INT_ADC1SS0);
    IntRegister(INT_ADC1SS0, handle_adc1);
    IntEnable(INT_ADC0SS0);
    IntRegister(INT_ADC0SS0, handle_adc0);
    IntEnable(INT_UART1);
    IntRegister(INT_UART1, handle_uart1);

    IntRegister(3, hard_fault);

    // Set AIN voltages
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

    // Trigger ADC conversion
    ADC1_PSSI_R = 1;
    ADC0_PSSI_R = 1;

    // Wait for interrupts to handle ADC
    while (~ADC1_SSFSTAT0_R & 0x100);
    while (~ADC0_SSFSTAT0_R & 0x100);

    while (UART0_FR_R & UART_FR_BUSY);
    while (~UART0_FR_R & UART_FR_TXFE);
    while (~UART1_FR_R & UART_FR_RXFE);

    // Force hard fault for testing interrupt
    __asm__("b 0x30000000");
    return 0;
}
