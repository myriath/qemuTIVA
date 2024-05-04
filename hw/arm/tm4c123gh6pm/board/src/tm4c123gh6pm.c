#include "hw/arm/tm4c123gh6pm/board/include/tm4c123gh6pm.h"

static const char *ssys_clocks[] = {
    "SYSCLK",
    "GPIOACLK",
    "GPIOBCLK",
    "GPIOCCLK",
    "GPIODCLK",
    "GPIOECLK",
    "GPIOFCLK",
    "UART0CLK",
    "UART1CLK",
    "UART2CLK",
    "UART3CLK",
    "UART4CLK",
    "UART5CLK",
    "UART6CLK",
    "UART7CLK",
    "TIMER0CLK",
    "TIMER1CLK",
    "TIMER2CLK",
    "TIMER3CLK",
    "TIMER4CLK",
    "TIMER5CLK",
    "ADC0CLK",
    "ADC1CLK",
};

static const uint32_t uart_addr[COUNT_UART] = {
    0x4000c000, 0x4000d000, 0x4000e000, 0x4000f000,
    0x40010000, 0x40011000, 0x40012000, 0x40013000
};
static const int uart_irq[COUNT_UART] = {5, 6, 33, 59, 60, 61, 62, 63};

static const uint32_t timer_addr[2][6] = {
    {
        0x40030000, 0x40031000, 0x40032000,
        0x40033000, 0x40034000, 0x40035000
    },
    {
        0x40036000, 0x40037000, 0x4004c000,
        0x4004d000, 0x4004e000, 0x4004f000
    }
};
static const int timer_irq[2][6] = {{19, 21, 23, 35, 70, 92}, {94, 96, 98, 100, 102, 104}};

static const uint32_t gpio_addr[N_GPIOS] = { 
    0x40004000, 0x40005000, 0x40006000, 0x40007000,
    0x40024000, 0x40025000
};
static const int gpio_irq[N_GPIOS] = {0, 1, 2, 3, 4, 30};

static const uint32_t adc_addr[COUNT_ADC] = {0x40038000, 0x40039000};
static const int adc_irq[COUNT_ADC][COUNT_SS] = {
    {14, 15, 16, 17}, {48, 49, 50, 51}
};

uint8_t gpio_ain_ports[COUNT_AIN] = {
    GPIO_E, GPIO_E, GPIO_E, GPIO_E, 
    GPIO_D, GPIO_D, GPIO_D, GPIO_D, 
    GPIO_E, GPIO_E, GPIO_B, GPIO_B
};

uint8_t gpio_ain_pins[COUNT_AIN] = {
    3, 2, 1, 0,
    3, 2, 1, 0,
    5, 4, 5, 4
};

uint8_t gpio_uart_ports[COUNT_UART] = {
    GPIO_A, GPIO_B, GPIO_D, GPIO_C,
    GPIO_C, GPIO_E, GPIO_D, GPIO_E
};

// Only store rx, tx is always +1
uint8_t gpio_uart_rx_pins[COUNT_UART] = {
    0, 0, 6, 6, 4, 4, 4, 0
};

uint8_t timer_ccp_ports[COUNT_TIMERS] = {
    GPIO_B, GPIO_B, GPIO_B,
    GPIO_B, GPIO_C, GPIO_C
};

uint8_t timer_ccp_pins[COUNT_TIMERS] = {
    6, 4, 0, 2, 0, 2
};

static void ssys_update(ssys_state *s)
{
    qemu_set_irq(s->irq, (s->ris & s->imc) != 0);
}

static uint64_t ssys_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    switch (offset) {
    case 0x000: /* DID0 */
        return s->did[0];
    case 0x004: /* DID1 */
        return s->did[1];
    case 0x008: /* DC0 */
        return s->dc[0];
    case 0x010: /* DC1 */
        return s->dc[0];
    case 0x014: /* DC2 */
        return s->dc[0];
    case 0x018: /* DC3 */
        return s->dc[0];
    case 0x01c: /* DC4 */
        return s->dc[0];
    case 0x020: /* DC5 */
        return s->dc[5];
    case 0x024: /* DC6 */
        return s->dc[6];
    case 0x028: /* DC7 */
        return s->dc[7];
    case 0x02c: /* DC8 */
        return s->dc[8];
    case 0x030: /* PBORCTL */
        return s->pborctl;
    case 0x040: /* SRCR0 */
        return 0;
    case 0x044: /* SRCR1 */
        return 0;
    case 0x048: /* SRCR2 */
        return 0;
    case 0x050: /* RIS */
        return s->ris;
    case 0x054: /* IMC */
        return s->imc;
    case 0x058: /* MISC */
        return s->ris & s->imc;
    case 0x05c: /* RESC */
        return s->resc;
    case 0x060: /* RCC */
        return s->rcc;
    case 0x06c:
        return s->gpiohbctl;
    case 0x070: /* RCC2 */
        return s->rcc2;
    case 0x07c:
        return s->moscctl;
    case 0x100: /* RCGC0 */
        return s->rcgc[0];
    case 0x104: /* RCGC1 */
        return s->rcgc[1];
    case 0x108: /* RCGC2 */
        return s->rcgc[2];
    case 0x110: /* SCGC0 */
        return s->scgc[0];
    case 0x114: /* SCGC1 */
        return s->scgc[1];
    case 0x118: /* SCGC2 */
        return s->scgc[2];
    case 0x120: /* DCGC0 */
        return s->dcgc[0];
    case 0x124: /* DCGC1 */
        return s->dcgc[1];
    case 0x128: /* DCGC2 */
        return s->dcgc[2];
    case 0x144:
        return s->dslpclkcfg;
    case 0x14c:
        return s->sysprop;
    case 0x150:
        return s->piosccal;
    case 0x154:
        return s->pioscstat;
    case 0x160:
        return s->pllfreq0;
    case 0x164:
        return s->pllfreq1;
    case 0x168:
        return s->pllstat;
    case 0x188:
        return s->slppwrcfg;
    case 0x18c:
        return s->dslppwrcfg;
    case 0x190: /* DC9 */
        return s->dc[9];
    case 0x1a0:
        return s->nvmstat;
    case 0x1b4:
        return s->ldospctl;
    case 0x1b8:
        return s->ldospcal;
    case 0x1bc:
        return s->ldodpctl;
    case 0x1c0:
        return s->ldodpcal;
    case 0x1cc:
        return s->sdpmst;
    case 0x300:
        return s->ppwd;
    case 0x304:
        return s->pptimer;
    case 0x308:
        return s->ppgpio;
    case 0x30c:
        return s->ppdma;
    case 0x314:
        return s->pphib;
    case 0x318:
        return s->ppuart;
    case 0x31c:
        return s->ppssi;
    case 0x320:
        return s->ppi2c;
    case 0x328:
        return s->ppusb;
    case 0x334:
        return s->ppcan;
    case 0x338:
        return s->ppadc;
    case 0x33c:
        return s->ppacmp;
    case 0x340:
        return s->pppwm;
    case 0x344:
        return s->ppqei;
    case 0x358:
        return s->ppeeprom;
    case 0x35c:
        return s->ppwtimer;
    case 0x500:
        return s->srwd;
    case 0x504:
        return s->srtimer;
    case 0x508:
        return s->srgpio;
    case 0x50c:
        return s->srdma;
    case 0x514:
        return s->srhib;
    case 0x518:
        return s->sruart;
    case 0x51c:
        return s->srssi;
    case 0x520:
        return s->sri2c;
    case 0x528:
        return s->srusb;
    case 0x534:
        return s->srcan;
    case 0x538:
        return s->sradc;
    case 0x53c:
        return s->sracmp;
    case 0x540:
        return s->srpwm;
    case 0x544:
        return s->srqei;
    case 0x558:
        return s->sreeprom;
    case 0x55c:
        return s->srwtimer;
    case 0x600:
        return s->rcgcwd;
    case 0x604:
        return s->rcgctimer;
    case 0x608:
        return s->rcgcgpio;
    case 0x60c:
        return s->rcgcdma;
    case 0x614:
        return s->rcgchib;
    case 0x618:
        return s->rcgcuart;
    case 0x61c:
        return s->rcgcssi;
    case 0x620:
        return s->rcgci2c;
    case 0x628:
        return s->rcgcusb;
    case 0x634:
        return s->rcgccan;
    case 0x638:
        return s->rcgcadc;
    case 0x63c:
        return s->rcgcacmp;
    case 0x640:
        return s->rcgcpwm;
    case 0x644:
        return s->rcgcqei;
    case 0x658:
        return s->rcgceeprom;
    case 0x65c:
        return s->rcgcwtimer;
    case 0x700:
        return s->scgcwd;
    case 0x704:
        return s->scgctimer;
    case 0x708:
        return s->scgcgpio;
    case 0x70c:
        return s->scgcdma;
    case 0x714:
        return s->scgchib;
    case 0x718:
        return s->scgcuart;
    case 0x71c:
        return s->scgcssi;
    case 0x720:
        return s->scgci2c;
    case 0x728:
        return s->scgcusb;
    case 0x734:
        return s->scgccan;
    case 0x738:
        return s->scgcadc;
    case 0x73c:
        return s->scgcacmp;
    case 0x740:
        return s->scgcpwm;
    case 0x744:
        return s->scgcqei;
    case 0x758:
        return s->scgceeprom;
    case 0x75c:
        return s->scgcwtimer;
    case 0x800:
        return s->dcgcwd;
    case 0x804:
        return s->dcgctimer;
    case 0x808:
        return s->dcgcgpio;
    case 0x80c:
        return s->dcgcdma;
    case 0x814:
        return s->dcgchib;
    case 0x818:
        return s->dcgcuart;
    case 0x81c:
        return s->dcgcssi;
    case 0x820:
        return s->dcgci2c;
    case 0x828:
        return s->dcgcusb;
    case 0x834:
        return s->dcgccan;
    case 0x838:
        return s->dcgcadc;
    case 0x83c:
        return s->dcgcacmp;
    case 0x840:
        return s->dcgcpwm;
    case 0x844:
        return s->dcgcqei;
    case 0x858:
        return s->dcgceeprom;
    case 0x85c:
        return s->dcgcwtimer;
    case 0xa00:
        return s->prwd;
    case 0xa04:
        return s->prtimer;
    case 0xa08:
        return s->prgpio;
    case 0xa0c:
        return s->prdma;
    case 0xa14:
        return s->prhib;
    case 0xa18:
        return s->pruart;
    case 0xa1c:
        return s->prssi;
    case 0xa20:
        return s->pri2c;
    case 0xa28:
        return s->prusb;
    case 0xa34:
        return s->prcan;
    case 0xa38:
        return s->pradc;
    case 0xa3c:
        return s->pracmp;
    case 0xa40:
        return s->prpwm;
    case 0xa44:
        return s->prqei;
    case 0xa58:
        return s->preeprom;
    case 0xa5c:
        return s->prwtimer;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SSYS: read at bad offset 0x%x\n", (int)offset);
        return 0;
    }
}

static bool ssys_use_rcc2(ssys_state *s)
{
    // TODO
    return (s->rcc2 >> 31) & 0x1;
}

static void update_clock(uint64_t period, int clock_count, Clock *clocks[], int rcgc_value)
{
    for (int i = 0; i < clock_count; i++) {
        int mask = 1 << i;
        if (rcgc_value & mask) {
            clock_update_ns(clocks[i], period);
        } else {
            clock_update_ns(clocks[i], 0);
        }
    }
}

static void update_all_clocks(ssys_state *s)
{
    update_clock(s->period, N_GPIOS, s->gpio_clks, s->rcgcgpio);
    update_clock(s->period, COUNT_TIMERS, s->timer_clks, s->rcgctimer);
    update_clock(s->period, COUNT_UART, s->uart_clks, s->rcgcuart);
    update_clock(s->period, COUNT_ADC, s->adc_clks, s->rcgcadc);
}

/*
 * Calculate the system clock period. We only want to propagate
 * this change to the rest of the system if we're not being called
 * from migration post-load.
 */
static void ssys_calculate_system_clock(ssys_state *s, bool propagate_clock)
{
    // TODO
    int period_ns;
    /*
     * SYSDIV field specifies divisor: 0 == /1, 1 == /2, etc.  Input
     * clock is 200MHz, which is a period of 5 ns. Dividing the clock
     * frequency by X is the same as multiplying the period by X.
     */
    if (ssys_use_rcc2(s)) {
        period_ns = 5 * (((s->rcc2 >> 23) & 0x3f) + 1);
    } else {
        period_ns = 5 * (((s->rcc >> 23) & 0xf) + 1);
    }
    s->period = period_ns;
    clock_set_ns(s->sysclk, period_ns);
    if (propagate_clock) {
        clock_propagate(s->sysclk);
        update_all_clocks(s);
    }
}

static void ssys_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    const char *reg = "BAD OFFSET";
    uint32_t output = value;

    // TODO
    switch (offset) {
    case 0x030: /* PBORCTL */
        reg = "PBORCTL";
        s->pborctl = value;
        break;
    case 0x054: /* IMC */
        reg = "IMC";
        s->imc = value;
        break;
    case 0x058: /* MISC */
        reg = "MISC";
        s->ris &= ~value;
        break;
    case 0x05c: /* RESC */
        reg = "RESC";
        s->resc = value;
        break;
    case 0x060: /* RCC */
        reg = "RCC";
        if ((s->rcc & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->ris |= (1 << 6);
        }
        s->rcc = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x06c:
        reg = "GPIOHBCTL";
        s->gpiohbctl = value;
        break;
    case 0x070: /* RCC2 */
        reg = "RCC2";
        if ((s->rcc2 & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->ris |= (1 << 6);
        }
        s->rcc2 = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x07c:
        reg = "MOSCCTL";
        s->moscctl = value;
        break;
    case 0x144:
        reg = "DSLPCLKCFG";
        s->dslpclkcfg = value;
        break;
    case 0x150:
        reg = "PIOSCCAL";
        s->piosccal = value;
        break;
    case 0x188:
        reg = "SLPPWRCFG";
        s->slppwrcfg = value;
        break;
    case 0x18c:
        reg = "DSLPPWRCFG";
        s->dslppwrcfg = value;
        break;
    case 0x1b4:
        reg = "LDOSPCTL";
        s->ldospctl = value;
        break;
    case 0x1bc:
        reg = "LDODPCTL";
        s->ldodpctl = value;
        break;
    case 0x500:
        reg = "SRWD";
        s->srwd = value;
        break;
    case 0x504:
        reg = "SRTIMER";
        s->srtimer = value;
        break;
    case 0x508:
        reg = "SRGPIO";
        s->srgpio = value;
        break;
    case 0x50c:
        reg = "SRDMA";
        s->srdma = value;
        break;
    case 0x514:
        reg = "SRHIB";
        s->srhib = value;
        break;
    case 0x518:
        reg = "SRUART";
        s->sruart = value;
        break;
    case 0x51c:
        reg = "SRSSI";
        s->srssi = value;
        break;
    case 0x520:
        reg = "SRI2C";
        s->sri2c = value;
        break;
    case 0x528:
        reg = "SRUSB";
        s->srusb = value;
        break;
    case 0x534:
        reg = "SRCAN";
        s->srcan = value;
        break;
    case 0x538:
        reg = "SRADC";
        s->sradc = value;
        break;
    case 0x53c:
        reg = "SRACMP";
        s->sracmp = value;
        break;
    case 0x540:
        reg = "SRPWM";
        s->srpwm = value;
        break;
    case 0x544:
        reg = "SRQEI";
        s->srqei = value;
        break;
    case 0x558:
        reg = "SREEPROM";
        s->sreeprom = value;
        break;
    case 0x55c:
        reg = "SRWTIMER";
        s->srwtimer = value;
        break;
    case 0x600:
        reg = "RCGCWD";
        s->rcgcwd = value;
        break;
    case 0x604:
        reg = "RCGCTIMER";
        s->rcgctimer = value;
        s->prtimer &= ~value;
        update_clock(s->period, COUNT_TIMERS, s->timer_clks, value);
        s->prtimer |= value;
        break;
    case 0x608:
        reg = "RCGCGPIO";
        s->rcgcgpio = value;
        s->prgpio &= ~value;
        update_clock(s->period, N_GPIOS, s->gpio_clks, value);
        s->prgpio |= value;
        break;
    case 0x60c:
        reg = "RCGCDMA";
        s->rcgcdma = value;
        break;
    case 0x614:
        reg = "RCGCHIB";
        s->rcgchib = value;
        break;
    case 0x618:
        reg = "RCGCUART";
        s->rcgcuart = value;
        s->pruart &= ~value;
        update_clock(s->period, COUNT_UART, s->uart_clks, value);
        s->pruart |= value;
        break;
    case 0x61c:
        reg = "RCGCSSI";
        s->rcgcssi = value;
        break;
    case 0x620:
        reg = "RCGCI2C";
        s->rcgci2c = value;
        break;
    case 0x628:
        reg = "RCGCUSB";
        s->rcgcusb = value;
        break;
    case 0x634:
        reg = "RCGCCAN";
        s->rcgccan = value;
        break;
    case 0x638:
        reg = "RCGCADC";
        s->rcgcadc = value;
        s->pradc &= ~value;
        update_clock(s->period, COUNT_ADC, s->adc_clks, value);
        s->pradc |= value;
        break;
    case 0x63c:
        reg = "RCGCACMP";
        s->rcgcacmp = value;
        break;
    case 0x640:
        reg = "RCGCPWM";
        s->rcgcpwm = value;
        break;
    case 0x644:
        reg = "RCGCQEI";
        s->rcgcqei = value;
        break;
    case 0x658:
        reg = "RCGCEEPROM";
        s->rcgceeprom = value;
        break;
    case 0x65c:
        reg = "RCGCWTIMER";
        s->rcgcwtimer = value;
        break;
    case 0x700:
        reg = "SCGCWD";
        s->scgcwd = value;
        break;
    case 0x704:
        reg = "SCGCTIMER";
        s->scgctimer = value;
        break;
    case 0x708:
        reg = "SCGCGPIO";
        s->scgcgpio = value;
        break;
    case 0x70c:
        reg = "SCGCDMA";
        s->scgcdma = value;
        break;
    case 0x714:
        reg = "SCGCHIB";
        s->scgchib = value;
        break;
    case 0x718:
        reg = "SCGCUART";
        s->scgcuart = value;
        break;
    case 0x71c:
        reg = "SCGCSSI";
        s->scgcssi = value;
        break;
    case 0x720:
        reg = "SCGCI2C";
        s->scgci2c = value;
        break;
    case 0x728:
        reg = "SCGCUSB";
        s->scgcusb = value;
        break;
    case 0x734:
        reg = "SCGCCAN";
        s->scgccan = value;
        break;
    case 0x738:
        reg = "SCGCADC";
        s->scgcadc = value;
        break;
    case 0x73c:
        reg = "SCGCACMP";
        s->scgcacmp = value;
        break;
    case 0x740:
        reg = "SCGCPWM";
        s->scgcpwm = value;
        break;
    case 0x744:
        reg = "SCGCQEI";
        s->scgcqei = value;
        break;
    case 0x758:
        reg = "SCGCEEPROM";
        s->scgceeprom = value;
        break;
    case 0x75c:
        reg = "SCGCWTIMER";
        s->scgcwtimer = value;
        break;
    case 0x800:
        reg = "DCGCWD";
        s->dcgcwd = value;
        break;
    case 0x804:
        reg = "DCGCTIMER";
        s->dcgctimer = value;
        break;
    case 0x808:
        reg = "DCGCGPIO";
        s->dcgcgpio = value;
        break;
    case 0x80c:
        reg = "DCGCDMA";
        s->dcgcdma = value;
        break;
    case 0x814:
        reg = "DCGCHIB";
        s->dcgchib = value;
        break;
    case 0x818:
        reg = "DCGCUART";
        s->dcgcuart = value;
        break;
    case 0x81c:
        reg = "DCGCSSI";
        s->dcgcssi = value;
        break;
    case 0x820:
        reg = "DCGCI2C";
        s->dcgci2c = value;
        break;
    case 0x828:
        reg = "DCGCUSB";
        s->dcgcusb = value;
        break;
    case 0x834:
        reg = "DCGCCAN";
        s->dcgccan = value;
        break;
    case 0x838:
        reg = "DCGCADC";
        s->dcgcadc = value;
        break;
    case 0x83c:
        reg = "DCGCACMP";
        s->dcgcacmp = value;
        break;
    case 0x840:
        reg = "DCGCPWM";
        s->dcgcpwm = value;
        break;
    case 0x844:
        reg = "DCGCQEI";
        s->dcgcqei = value;
        break;
    case 0x858:
        reg = "DCGCEEPROM";
        s->dcgceeprom = value;
        break;
    case 0x85c:
        reg = "DCGCWTIMER";
        s->dcgcwtimer = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SSYS: write at bad offset 0x%x\n", (int)offset);
    }

    if (s->debug) {
        printf("[SYSCTL %s] 0x%08X\n", reg, output);
    }

    ssys_update(s);
}

static const MemoryRegionOps ssys_ops = {
    .read = ssys_read,
    .write = ssys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tm4c123gh6pm_sys_reset_enter(Object *obj, ResetType type)
{
    ssys_state *s = TM4_SYS(obj);

    s->pborctl = 0x7ffd;
    s->rcc = 0x078e3ad1;
    s->rcc2 = 0x07c06810;

    // All peripheral ready registers initialized with 1s since clock gating starts set up
    s->pracmp = 0xffffffff;
    s->pradc = 0xffffffff;
    s->prcan = 0xffffffff;
    s->prdma = 0xffffffff;
    s->preeprom = 0xffffffff;
    s->prgpio = 0xffffffff;
    s->prhib = 0xffffffff;
    s->pri2c = 0xffffffff;
    s->prpwm = 0xffffffff;
    s->prqei = 0xffffffff;
    s->prssi = 0xffffffff;
    s->prtimer = 0xffffffff;
    s->pruart = 0xffffffff;
    s->prusb = 0xffffffff;
    s->prwd = 0xffffffff;
    s->prwtimer = 0xffffffff;

    s->rcgc[0] = 1;
    s->scgc[0] = 1;
    s->dcgc[0] = 1;
}

static void tm4c123gh6pm_sys_reset_hold(Object *obj, ResetType type)
{
    ssys_state *s = TM4_SYS(obj);

    /* OK to propagate clocks from the hold phase */
    ssys_calculate_system_clock(s, true);
}

static void tm4c123gh6pm_sys_reset_exit(Object *obj, ResetType type)
{
}

static int tm4c123gh6pm_sys_post_load(void *opaque, int version_id)
{
    ssys_state *s = opaque;

    ssys_calculate_system_clock(s, false);

    return 0;
}

static const VMStateDescription vmstate_tm4c123gh6pm_sys = {
    .name = "tm4c123gh6pm_sys",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = tm4c123gh6pm_sys_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pborctl, ssys_state),
        VMSTATE_UINT32(ris, ssys_state),
        VMSTATE_UINT32(imc, ssys_state),
        VMSTATE_UINT32(resc, ssys_state),
        VMSTATE_UINT32(rcc, ssys_state),
        VMSTATE_UINT32_V(rcc2, ssys_state, 2),
        VMSTATE_UINT32_ARRAY(rcgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(scgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(dcgc, ssys_state, 3),
        // TODO
        /* No field for sysclk -- handled in post-load instead */
        VMSTATE_END_OF_LIST()
    }
};

static Property tm4c123gh6pm_sys_properties[] = {
    DEFINE_PROP_UINT32("did0", ssys_state, did[0], 0),
    DEFINE_PROP_UINT32("did1", ssys_state, did[1], 0),
    DEFINE_PROP_UINT32("dc0", ssys_state, dc[0], 0),
    DEFINE_PROP_UINT32("dc1", ssys_state, dc[1], 0),
    DEFINE_PROP_UINT32("dc2", ssys_state, dc[2], 0),
    DEFINE_PROP_UINT32("dc3", ssys_state, dc[3], 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc[4], 0),
    DEFINE_PROP_UINT32("dc5", ssys_state, dc[5], 0),
    DEFINE_PROP_UINT32("dc6", ssys_state, dc[6], 0),
    DEFINE_PROP_UINT32("dc7", ssys_state, dc[7], 0),
    DEFINE_PROP_UINT32("dc8", ssys_state, dc[8], 0),
    DEFINE_PROP_UINT32("dc9", ssys_state, dc[9], 0),
    DEFINE_PROP_UINT32("ppwd", ssys_state, ppwd, 0),
    DEFINE_PROP_UINT32("pptimer", ssys_state, pptimer, 0),
    DEFINE_PROP_UINT32("ppgpio", ssys_state, ppgpio, 0),
    DEFINE_PROP_UINT32("ppdma", ssys_state, ppdma, 0),
    DEFINE_PROP_UINT32("pphib", ssys_state, pphib, 0),
    DEFINE_PROP_UINT32("ppuart", ssys_state, ppuart, 0),
    DEFINE_PROP_UINT32("ppssi", ssys_state, ppssi, 0),
    DEFINE_PROP_UINT32("ppi2c", ssys_state, ppi2c, 0),
    DEFINE_PROP_UINT32("ppusb", ssys_state, ppusb, 0),
    DEFINE_PROP_UINT32("ppcan", ssys_state, ppcan, 0),
    DEFINE_PROP_UINT32("ppadc", ssys_state, ppadc, 0),
    DEFINE_PROP_UINT32("ppacmp", ssys_state, ppacmp, 0),
    DEFINE_PROP_UINT32("pppwm", ssys_state, pppwm, 0),
    DEFINE_PROP_UINT32("ppqei", ssys_state, ppqei, 0),
    DEFINE_PROP_UINT32("ppeeprom", ssys_state, ppeeprom, 0),
    DEFINE_PROP_UINT32("ppwtimer", ssys_state, ppwtimer, 0),
    DEFINE_PROP_BOOL("debug", ssys_state, debug, false),
    DEFINE_PROP_END_OF_LIST()
};

static void create_clocks(int clk_index, int count, Clock **clocks, DeviceState *dev)
{
    for (int i = 0; i < count; i++) {
        clocks[i] = qdev_init_clock_out(dev, ssys_clocks[clk_index + i]);
    }
}

static void tm4c123gh6pm_sys_instance_init(Object *obj)
{
    ssys_state *s = TM4_SYS(obj);
    DeviceState *dev = DEVICE(s);
    SysBusDevice *sbd = SYS_BUS_DEVICE(s);

    memory_region_init_io(&s->iomem, obj, &ssys_ops, s, "ssys", 0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    create_clocks(CLK_SYS, 1, &s->sysclk, dev);
    create_clocks(CLK_GPIO, N_GPIOS, s->gpio_clks, dev);
    create_clocks(CLK_TIMER, COUNT_TIMERS, s->timer_clks, dev);
    create_clocks(CLK_UART, COUNT_UART, s->uart_clks, dev);
    create_clocks(CLK_ADC, COUNT_ADC, s->adc_clks, dev);
}

static void board_init(MachineState *ms, struct tiva_devices *devices, bool debug)
{
    /* Memory map of SoC devices, from
     * Stellaris LM3S6965 Microcontroller Data Sheet (rev I)
     * http://www.ti.com/lit/ds/symlink/lm3s6965.pdf
     * 
     * Memory
     * 00000000 flash 0003ffff
     * 20000000 bit-banded sram 20007fff
     * 22000000 bit-banded alias sram of 20000000
     * 
     * Peripherals
     * 40000000 wdtimer0 (unimplemented)
     * 40001000 wdtimer1 (unimplemented)
     * 40004000 GPIO A
     * 40005000 GPIO B
     * 40006000 GPIO C
     * 40007000 GPIO D
     * 40008000 SSI0 (unimplemented)
     * 40009000 SSI1 (unimplemented)
     * 4000a000 SSI2 (unimplemented)
     * 4000b000 SSI3 (unimplemented)
     * 4000c000 UART0
     * 4000d000 UART1
     * 4000e000 UART2
     * 4000f000 UART3
     * 40010000 UART4
     * 40011000 UART5
     * 40012000 UART6
     * 40013000 UART7
     * 40020000 i2c0 (unimplemented)
     * 40021000 i2c1 (unimplemented)
     * 40022000 i2c2 (unimplemented)
     * 40023000 i2c3 (unimplemented)
     * 40024000 GPIO E
     * 40025000 GPIO F
     * 40028000 PWM0 (unimplemented)
     * 40029000 PWM1 (unimplemented)
     * 4002c000 QEI0 (unimplemented)
     * 4002d000 QEI1 (unimplemented)
     * 40030000 16/32 timer 0
     * 40031000 16/32 timer 1
     * 40032000 16/32 timer 2
     * 40033000 16/32 timer 3
     * 40034000 16/32 timer 4
     * 40035000 16/32 timer 5
     * 40036000 32/64 timer 0 (unimplemented)
     * 40037000 32/64 timer 1 (unimplemented)
     * 40038000 ADC0
     * 40039000 ADC1
     * 4003c000 analogue comparator (unimplemented)
     * 40040000 CAN0 (unimplemented)
     * 40041000 CAN1 (unimplemented)
     * 4004c000 32/64 timer 2 (unimplemented)
     * 4004d000 32/64 timer 3 (unimplemented)
     * 4004e000 32/64 timer 4 (unimplemented)
     * 4004f000 32/64 timer 5 (unimplemented)
     * 40050000 USB (unimplemented)
     * 40058000 GPIO A (AHB) (unimplemented)
     * 40059000 GPIO B (AHB) (unimplemented)
     * 4005a000 GPIO C (AHB) (unimplemented)
     * 4005b000 GPIO D (AHB) (unimplemented)
     * 4005c000 GPIO E (AHB) (unimplemented)
     * 4005d000 GPIO F (AHB) (unimplemented)
     * 400af000 EEPROM & key locker (unimplemented)
     * 400f9000 System Exception module (unimplemented)
     * 400fc000 hibernation module (unimplemented)
     * 400fd000 flash memory control (unimplemented)
     * 400fe000 system control
     * 400ff000 uDMA (unimplemented)
     * 44000000 bit-banded alias 40000000 - 400fffff (unimplemented)
     * 
     * Private Peripheral Bus
     * e0000000 ITM
     * e0001000 DWT
     * e0002000 FPB
     * e000e000 Cortex-M4F Peripherals
     * e0040000 TPIU
     * e0041000 ETM
     */

    DeviceState **gpio_dev = devices->gpio;
    DeviceState **nvic = &devices->nvic;
    DeviceState **ssys_dev = &devices->ssys_dev;

    qemu_irq (*gpio_in)[N_GPIO_BITS][N_PCTL_OPTS] = devices->gpio_in;
    qemu_irq (*gpio_out)[N_GPIO_BITS][N_PCTL_OPTS] = devices->gpio_out;

    DeviceState *dev;

    int i;
    int j;
    int k;

    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *system_memory = get_system_memory();

    int flash_size = 256 * 1024;
    int sram_size = 32 * 1024;

    Object *soc_container = object_new("container");
    devices->soc = soc_container;
    object_property_add_child(OBJECT(ms), "soc", soc_container);

    /* Flash programming is done via the SCU, so pretend it is ROM.  */
    memory_region_init_rom(flash, NULL, "tm4.flash", flash_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0, flash);

    memory_region_init_ram(sram, NULL, "tm4.sram", sram_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory, 0x20000000, sram);

    /*
     * Create the system-registers object early, because we will
     * need its sysclk output.
     */
    *ssys_dev = qdev_new(TYPE_TM4_SYS);
    qdev_prop_set_bit(*ssys_dev, "debug", debug);
    object_property_add_child(soc_container, "sys", OBJECT(*ssys_dev));

    // Values fetched from CyBot
    qdev_prop_set_uint32(*ssys_dev, "did0", 0x18050102);
    qdev_prop_set_uint32(*ssys_dev, "did1", 0x10a1606e);
    qdev_prop_set_uint32(*ssys_dev, "dc0",  0x007f007f);
    qdev_prop_set_uint32(*ssys_dev, "dc1",  0x13332fff);
    qdev_prop_set_uint32(*ssys_dev, "dc2",  0x030ff337);
    qdev_prop_set_uint32(*ssys_dev, "dc3",  0xbfff8fff);
    qdev_prop_set_uint32(*ssys_dev, "dc4",  0x0004f03f);
    qdev_prop_set_uint32(*ssys_dev, "dc5",  0x013000ff);
    qdev_prop_set_uint32(*ssys_dev, "dc6",  0x00000013);
    qdev_prop_set_uint32(*ssys_dev, "dc7",  0xffffffff);
    qdev_prop_set_uint32(*ssys_dev, "dc8",  0x0fff0fff);
    qdev_prop_set_uint32(*ssys_dev, "dc9",  0x00ff00ff);
    qdev_prop_set_uint32(*ssys_dev, "ppwd", 0x3);
    qdev_prop_set_uint32(*ssys_dev, "pptimer", 0x3f);
    qdev_prop_set_uint32(*ssys_dev, "ppgpio", 0x3f);
    qdev_prop_set_uint32(*ssys_dev, "ppdma", 0x1);
    qdev_prop_set_uint32(*ssys_dev, "pphib", 0x1);
    qdev_prop_set_uint32(*ssys_dev, "ppuart", 0xff);
    qdev_prop_set_uint32(*ssys_dev, "ppssi", 0xf);
    qdev_prop_set_uint32(*ssys_dev, "ppi2c", 0xf);
    qdev_prop_set_uint32(*ssys_dev, "ppusb", 0x1);
    qdev_prop_set_uint32(*ssys_dev, "ppcan", 0x3);
    qdev_prop_set_uint32(*ssys_dev, "ppadc", 0x3);
    qdev_prop_set_uint32(*ssys_dev, "ppacmp", 0x1);
    qdev_prop_set_uint32(*ssys_dev, "pppwm", 0x3);
    qdev_prop_set_uint32(*ssys_dev, "ppqei", 0x3);
    qdev_prop_set_uint32(*ssys_dev, "ppeeprom", 0x1);
    qdev_prop_set_uint32(*ssys_dev, "ppwtimer", 0x3f);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(*ssys_dev), &error_fatal);

    *nvic = qdev_new(TYPE_ARMV7M);
    object_property_add_child(soc_container, "v7m", OBJECT(*nvic));
    qdev_prop_set_uint32(*nvic, "num-irq", NUM_IRQ_LINES);
    qdev_prop_set_uint8(*nvic, "num-prio-bits", NUM_PRIO_BITS);
    qdev_prop_set_string(*nvic, "cpu-type", ms->cpu_type);
    qdev_prop_set_bit(*nvic, "enable-bitband", true);
    qdev_connect_clock_in(*nvic, "cpuclk",
                          qdev_get_clock_out(*ssys_dev, "SYSCLK"));
    /* This SoC does not connect the systick reference clock */
    object_property_set_link(OBJECT(*nvic), "memory",
                             OBJECT(get_system_memory()), &error_abort);
    /* This will exit with an error if the user passed us a bad cpu_type */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(*nvic), &error_fatal);

    /* Now we can wire up the IRQ and MMIO of the system registers */
    sysbus_mmio_map(SYS_BUS_DEVICE(*ssys_dev), 0, 0x400fe000);
    sysbus_connect_irq(SYS_BUS_DEVICE(*ssys_dev), 0, qdev_get_gpio_in(*nvic, 28)); // Interrupt 28: System Control


    // GPIO
    for (i = 0; i < N_GPIOS; i++) {
        gpio_dev[i] = gpio_create(debug, gpio_addr[i], qdev_get_gpio_in(*nvic, gpio_irq[i]), i, qdev_get_clock_out(*ssys_dev, ssys_clocks[CLK_GPIO + i]));
        
        for (j = 0; j < N_GPIO_BITS; j++) {
            for (k = 0; k < N_PCTL_OPTS; k++) {
                // GPIO OUT are the irqs of data going OUT of the system via GPIO
                gpio_out[i][j][k] = NULL;
                // GPIO IN are the irqs of data coming IN to the system via GPIO
                gpio_in[i][j][k] = qdev_get_gpio_in(gpio_dev[i], j * N_PCTL_OPTS + k);
            }
        }
    }

    DeviceState **adc = devices->adc;

    // ADC
    for (i = 0; i < COUNT_ADC; i++) {
        qemu_irq adc_nvic[COUNT_SS];
        for (j = 0; j < COUNT_SS; j++) {
            adc_nvic[j] = qdev_get_gpio_in(*nvic, adc_irq[i][j]);
        }
        adc[i] = adc_create(debug, adc_addr[i], adc_nvic, i, qdev_get_clock_out(*ssys_dev, ssys_clocks[CLK_ADC + i]));
    }

    // Connect GPIO to ADC
    for (i = 0; i < COUNT_AIN; i++) {
        uint8_t port = gpio_ain_ports[i];
        uint8_t pin = gpio_ain_pins[i];
        // Connect gpio to adcs
        DeviceState *splitter = qdev_new(TYPE_SPLIT_IRQ);
        object_property_add_child(OBJECT(ms), "splitter[*]", OBJECT(splitter));
        qdev_prop_set_uint32(splitter, "num-lines", COUNT_ADC);
        qdev_realize_and_unref(splitter, NULL, &error_fatal);

        for (j = 0; j < COUNT_ADC; j++) {
            qdev_connect_gpio_out(splitter, j, qdev_get_gpio_in(adc[j], i));
        }

        gpio_out[port][pin][F_ANALOG] = qdev_get_gpio_in(splitter, 0);
        // qdev_connect_gpio_out_named(gpio_dev[port], GPIO_NAMED_PINS[pin], F_ANALOG, qdev_get_gpio_in(splitter, 0));
    }

    // Timers
    for (i = 0; i < COUNT_TIMERS; i++) {
        int width = i / 6;
        int index = i % 6;
        dev = timer_16_create(
            debug,
            qdev_get_gpio_in(*nvic, timer_irq[width][index]), 
            timer_addr[width][index], 
            i,
            qdev_get_clock_out(*ssys_dev, ssys_clocks[CLK_TIMER + i])
        );
        devices->timer[i] = dev;

        uint8_t port = timer_ccp_ports[i];
        uint8_t pin = timer_ccp_pins[i];
        qdev_connect_gpio_out(dev, 0, gpio_in[port][pin][F_TIMER]);
        qdev_connect_gpio_out(dev, 1, gpio_in[port][pin + 1][F_TIMER]);
    }

    // UART
    for (i = 0; i < COUNT_UART; i++) {
        // UART1 is weird, do it outside the loop
        uint8_t port = gpio_uart_ports[i];
        uint8_t pin = gpio_uart_rx_pins[i];

        qemu_irq tx_gpio = gpio_in[port][pin + 1][F_UART];
        qemu_irq cts = NULL;
        qemu_irq rts = NULL;

        DeviceState *rx_dev = gpio_dev[port];
        const char *rx_name = GPIO_NAMED_PINS[pin];

        uint8_t rx_irq = F_UART;
        
        if (i == 1) {
            // UART 1 has two GPIO AFSEL table slots
            DeviceState *splitter = qdev_new(TYPE_SPLIT_IRQ);
            object_property_add_child(OBJECT(ms), "splitter[*]", OBJECT(splitter));
            qdev_prop_set_uint32(splitter, "num-lines", 2);
            qdev_realize_and_unref(splitter, NULL, &error_fatal);

            qdev_connect_gpio_out(splitter, 0, gpio_in[port][pin + 1][F_UART]);
            qdev_connect_gpio_out(splitter, 1, gpio_in[GPIO_C][5][F_UART_ALT]);

            // Split TX from UART 1 to both gpio
            tx_gpio = qdev_get_gpio_in(splitter, 0);

            DeviceState *joiner = qdev_new(TYPE_JOIN_IRQ);
            object_property_add_child(OBJECT(ms), "joiner[*]", OBJECT(joiner));
            qdev_prop_set_uint32(joiner, "num-lines", 2);
            qdev_realize_and_unref(joiner, NULL, &error_fatal);

            gpio_out[port][pin][F_UART] = qdev_get_gpio_in(joiner, 0);
            gpio_out[GPIO_C][4][F_UART_ALT] = qdev_get_gpio_in(joiner, 1);

            rx_dev = joiner;
            rx_name = NULL;
            rx_irq = 0;

            // TODO RTS and CTS
        }

        dev = uart_create(debug, uart_addr[i], i, qdev_get_gpio_in(*nvic, uart_irq[i]), 
                          tx_gpio, rts, cts, qdev_get_clock_out(*ssys_dev, ssys_clocks[CLK_UART + i]));
        devices->uart[i] = dev;
        // Connect RX pin
        qdev_connect_gpio_out_named(rx_dev, rx_name, rx_irq, qdev_get_gpio_in(dev, 0));
    }


    /* Add dummy regions for the devices we don't implement yet,
     * so guest accesses don't cause unlogged crashes.
     */
    create_unimplemented_device("watchdog-0", 0x40000000, 0x1000);
    create_unimplemented_device("watchdog-1", 0x40001000, 0x1000);
    create_unimplemented_device("ssi0", 0x40008000, 0x1000);
    create_unimplemented_device("ssi1", 0x40009000, 0x1000);
    create_unimplemented_device("ssi2", 0x4000a000, 0x1000);
    create_unimplemented_device("ssi3", 0x4000b000, 0x1000);
    create_unimplemented_device("i2c-0", 0x40020000, 0x1000);
    create_unimplemented_device("i2c-1", 0x40021000, 0x1000);
    create_unimplemented_device("i2c-2", 0x40022000, 0x1000);
    create_unimplemented_device("i2c-3", 0x40023000, 0x1000);
    create_unimplemented_device("PWM0", 0x40028000, 0x1000);
    create_unimplemented_device("PWM1", 0x40029000, 0x1000);
    create_unimplemented_device("QEI0", 0x4002c000, 0x1000);
    create_unimplemented_device("QEI1", 0x4002d000, 0x1000);
    create_unimplemented_device("analogue-comparator", 0x4003c000, 0x1000);
    create_unimplemented_device("can0", 0x40040000, 0x1000);
    create_unimplemented_device("can1", 0x40041000, 0x1000);
    create_unimplemented_device("usb", 0x40050000, 0x1000);
    create_unimplemented_device("eeprom-keylocker", 0x400af000, 0x1000);
    create_unimplemented_device("system-exception", 0x400f9000, 0x1000);
    create_unimplemented_device("hibernation", 0x400fc000, 0x1000);
    create_unimplemented_device("flash-control", 0x400fd000, 0x1000);
    create_unimplemented_device("udma", 0x400ff000, 0x1000);

    armv7m_load_kernel(ARM_CPU(first_cpu), ms->kernel_filename, 0, flash_size);
}

static void connect_gpios(struct tiva_devices *devices)
{
    DeviceState **gpio_dev = devices->gpio;
    qemu_irq (*gpio_out)[N_GPIO_BITS][N_PCTL_OPTS] = devices->gpio_out;

    int i, j, k;
    // Connect GPIO outs
    for (i = 0; i < N_GPIOS; i++) {
        for (j = 0; j < N_GPIO_BITS; j++) {
            for (k = 0; k < N_PCTL_OPTS; k++) {
                if (!gpio_out[i][j][k]) {
                    continue;
                }
                qdev_connect_gpio_out_named(gpio_dev[i], GPIO_NAMED_PINS[j],
                                            k, gpio_out[i][j][k]);
            }
        }
    }
}

static void tm4c123gh6pm_init(MachineState *ms, bool debug)
{
    int i;

    DeviceState *dev;
    struct tiva_devices devices;

    board_init(ms, &devices, debug);

    // Analog test input device
    dev = sysbus_create_varargs(TYPE_TEST_ANALOG, 0x40002000, NULL);
    for (i = 0; i < COUNT_AIN; i++) {
        uint8_t port = gpio_ain_ports[i];
        uint8_t pin = gpio_ain_pins[i];
        // Connect analog test device to ains
        qdev_connect_gpio_out(dev, i, devices.gpio_in[port][pin][F_ANALOG]);
    }

    // Connect U0 TX to U1 RX for testing
    devices.gpio_out[GPIO_A][1][F_UART] = devices.gpio_in[GPIO_B][0][F_UART];

    connect_gpios(&devices);
}

static void tm4c123gh6pm_init_debug(MachineState *ms)
{
    tm4c123gh6pm_init(ms, true);
}

static void tm4c123gh6pm_init_nodebug(MachineState *ms)
{
    tm4c123gh6pm_init(ms, false);
}

static void cybot_init(MachineState *ms, bool debug)
{
    DeviceState *dev;
    struct tiva_devices devices;

    board_init(ms, &devices, debug);

    dev = servo_create(debug, 0x50000000, qdev_get_clock_out(devices.ssys_dev, ssys_clocks[CLK_SYS]));
    // Connect servo to GPIO B pin 5 (from cybot baseboard ref) [timer 1 b]
    devices.gpio_out[GPIO_B][5][F_TIMER] = qdev_get_gpio_in(dev, 0);

    connect_gpios(&devices);
}

static void cybot_init_debug(MachineState *ms)
{
    cybot_init(ms, true);
}

static void cybot_init_nodebug(MachineState *ms)
{
    cybot_init(ms, false);
}

static void cybot_machine_init_nodebug(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "CyBot (Cortex-M4F)";
    mc->init = cybot_init_nodebug;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");
}

static void cybot_machine_init_debug(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "CyBot (Cortex-M4F) [Debug]";
    mc->init = cybot_init_debug;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");
}

static const TypeInfo cybot_type_nodebug = {
    .name = MACHINE_TYPE_NAME("cybot"),
    .parent = TYPE_MACHINE,
    .class_init = cybot_machine_init_nodebug,
};

static const TypeInfo cybot_type_debug = {
    .name = MACHINE_TYPE_NAME("cybot_debug"),
    .parent = TYPE_MACHINE,
    .class_init = cybot_machine_init_debug,
};

static void cybot_machine_init_register_types(void)
{
    type_register_static(&cybot_type_nodebug);
    type_register_static(&cybot_type_debug);
}
type_init(cybot_machine_init_register_types)

static void tm4c123gh6pm_machine_init_nodebug(ObjectClass *oc, void *data) 
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Tiva TM4C123GH6PM (Cortex-M4F)";
    mc->init = tm4c123gh6pm_init_nodebug;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4"); // TODO: cortex-m4f
}

static void tm4c123gh6pm_machine_init_debug(ObjectClass *oc, void *data) 
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Tiva TM4C123GH6PM (Cortex-M4F) [Debug]";
    mc->init = tm4c123gh6pm_init_debug;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4"); // TODO: cortex-m4f
}

static const TypeInfo tm4c123gh6pm_type_nodebug = {
    .name = MACHINE_TYPE_NAME("tm4c123gh6pm"),
    .parent = TYPE_MACHINE,
    .class_init = tm4c123gh6pm_machine_init_nodebug,
};

static const TypeInfo tm4c123gh6pm_type_debug = {
    .name = MACHINE_TYPE_NAME("tm4c123gh6pm_debug"),
    .parent = TYPE_MACHINE,
    .class_init = tm4c123gh6pm_machine_init_debug,
};

static void tm4c123gh6pm_machine_init_register_types(void)
{
    type_register_static(&tm4c123gh6pm_type_nodebug);
    type_register_static(&tm4c123gh6pm_type_debug);
}
type_init(tm4c123gh6pm_machine_init_register_types)


static void tm4c123gh6pm_sys_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_tm4c123gh6pm_sys;
    rc->phases.enter = tm4c123gh6pm_sys_reset_enter;
    rc->phases.hold = tm4c123gh6pm_sys_reset_hold;
    rc->phases.exit = tm4c123gh6pm_sys_reset_exit;
    device_class_set_props(dc, tm4c123gh6pm_sys_properties);
}

static const TypeInfo tm4c123gh6pm_sys_info = {
    .name = TYPE_TM4_SYS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ssys_state),
    .instance_init = tm4c123gh6pm_sys_instance_init,
    .class_init = tm4c123gh6pm_sys_class_init,
};

static void tm4c123gh6pm_register_types(void)
{
    type_register_static(&tm4c123gh6pm_sys_info);
}
type_init(tm4c123gh6pm_register_types)
