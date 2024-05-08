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
int tm4() 
{
    // Enable clocks for devices
    SYSCTL_RCGCGPIO_R = 0x3f;
    while (~SYSCTL_PRGPIO_R & 0x3f);
    SYSCTL_RCGCADC_R = 0x3;
    while (~SYSCTL_PRADC_R & 0x3f);
    SYSCTL_RCGCUART_R = 0x7f;
    while (~SYSCTL_PRUART_R & 0x7f);

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

#define PULSE_WIDTH (*((volatile uint32_t *)0x50000000))

int cybot() 
{
    // Set pulse width to 20,000,000 ns (20ms)
    PULSE_WIDTH = 20000000;

    // Enable clock
    SYSCTL_RCGCGPIO_R |= 0x2;
    while (~SYSCTL_PRGPIO_R & 0x2);
    SYSCTL_RCGCTIMER_R |= 0x2;
    while (~SYSCTL_PRTIMER_R & 0x2);

    // Setup timer GPIO port
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R |= 0x700000;

    // Setup timer
    TIMER1_CFG_R = 4;
    TIMER1_TBMR_R = 0xa;
    // load value = 250,000 = 20 ms (12.5 MHz)
    TIMER1_TBILR_R = 0xd090;
    TIMER1_TBPR_R = 0x3;
    // match value = 237,500 = 19ms
    TIMER1_TBMATCHR_R = 0x9fbc;
    TIMER1_TBPMR_R = 0x3;
    // thus, 1ms high = 5% duty cycle
    TIMER1_CTL_R = 0x100;

    while (true);
}

volatile char command_byte = 0;

void UART1_Handler(void) 
{
    if (UART1_MIS_R & UART_MIS_RXMIS) {
        UART1_ICR_R |= UART_ICR_RXIC;
        command_byte = UART1_DR_R & 0xff;
    }
}

void uart_send(char data) {
    while (UART1_FR_R & UART_FR_TXFF || UART1_FR_R & UART_FR_BUSY);
    UART1_DR_R = data;
}

void uart_print(char *data) {
    char *ptr = data;
    while (*ptr != 0) {
        uart_send(*(ptr++));
    }
}

int strcmp(char *p0, char *p1) 
{
    char last = -1;
    while (*p0 == *p1) {
        last = *p0;
        p0++;
        p1++;
    }
    return last != 0;
}

#define MAX_COMMAND_LEN 100

int cybot_wifi()
{
    SYSCTL_RCGCGPIO_R |= 0x2;
    SYSCTL_RCGCUART_R |= 0x2;
    while ((SYSCTL_PRGPIO_R & 0x2) == 0);
    while ((SYSCTL_PRUART_R & 0x2) == 0);

    GPIO_PORTB_AFSEL_R |= 0x3;
    GPIO_PORTB_DEN_R |= 0x3;
    GPIO_PORTB_PCTL_R = 0x11;
    uint16_t iBRD = 500;
    uint16_t fBRD = 0;

    UART1_CTL_R &= ~0xcbbf;
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    UART1_LCRH_R = 0x70;
    UART1_CC_R = 0x0;

    UART1_ICR_R |= 0x10;
    UART1_IM_R |= 0x10;
    NVIC_PRI1_R = (NVIC_PRI1_R * 0xff0fffff) | 0x00200000;
    NVIC_EN0_R |= 0x40;

    IntRegister(INT_UART1, UART1_Handler);
    IntMasterEnable();    

    UART1_CTL_R |= 0x301;
                          
    int commandLen = 0;
    char command[MAX_COMMAND_LEN];

    int commandPtr = 0;

    while (true) {
        char i = command_byte;
        if (i != 0) {
            command_byte = 0;
            uart_send(i);
            if (i == 0x7f) {
                if (commandPtr > 0) {
                    command[--commandPtr] = 0;
                }
            } else if (i == 13 || commandPtr >= MAX_COMMAND_LEN - 1) {
                uart_print("\nReceived: '");
                uart_print(command);
                uart_print("'\n\r");
                if (strcmp(command, "exit") == 0) {
                    return;
                }
                for (int j = 0; j < commandPtr; j++) {
                    command[j] = 0;
                }
                commandPtr = 0;
            } else {
                command[commandPtr++] = i;
            }
        }
    }
}

int main() 
{
    // cybot();
    cybot_wifi();
    // tm4();
}
