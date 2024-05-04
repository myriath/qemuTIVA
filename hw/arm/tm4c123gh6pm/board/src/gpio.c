#include "hw/arm/tm4c123gh6pm/board/include/gpio.h"

DeviceState *gpio_create(bool debug, hwaddr addr, qemu_irq nvic_irq, uint8_t port, Clock *clk)
{
    // Needed to set a property before realization, this
    // is just the internals of sysbus_create_simple()
    DeviceState *dev = qdev_new(TYPE_TM4_GPIO);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    qdev_connect_clock_in(dev, "clk", clk);

    qdev_prop_set_uint8(dev, "port", port);
    qdev_prop_set_bit(dev, "debug", debug);
    sysbus_realize_and_unref(sbd, &error_fatal);

    sysbus_mmio_map(sbd, 0, addr);
    sysbus_connect_irq(sbd, 0, nvic_irq);
        
    return dev;
}

static const uint8_t pl061_id_luminary[12] =
  { 0x00, 0x00, 0x00, 0x00, 0x61, 0x00, 0x18, 0x01, 0x0d, 0xf0, 0x05, 0xb1 };

const char *GPIO_NAMED_PINS[N_GPIO_BITS] = {
    "p0", "p1", "p2", "p3", "p4", "p5", "p6", "p7"
};

const int8_t GPIO_ALTERNATE_FUNCTIONS[6][N_GPIO_BITS][16] =
{
    // GPIO A
    {
        {F_NONE, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_CAN1Rx_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_CAN1Tx_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI0Clk_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI0Fss_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI0Rx_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI0Tx_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_I2C1SCL_A, F_NONE, F_M1PWM2_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_I2C1SDA_A, F_NONE, F_M1PWM3_A, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE}
    },
    // GPIO B
    {
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_I2C0SCL_B, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_I2C0SDA_B, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_NONE, F_SSI2Clk_B, F_NONE, F_M0PWM2_B, F_NONE, F_NONE, F_TIMER, F_CAN0Rx_B, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_NONE, F_SSI2Fss_B, F_NONE, F_M0PWM3_B, F_NONE, F_NONE, F_TIMER, F_CAN0Tx_B, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI2Rx_B, F_NONE, F_M0PWM0_B, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_SSI2Tx_B, F_NONE, F_M0PWM1_B, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE}
    },
    // GPIO C
    {
        {F_NONE, F_TCK_SWCLK_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_TMS_SWDIO_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_TDI_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_TDO_SWO_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_UART_ALT, F_NONE, F_M0PWM6_C, F_NONE, F_IDX1_C, F_WT0CCP0_C, F_U1RTS_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_UART_ALT, F_NONE, F_M0PWM7_C, F_NONE, F_PhA1_C, F_WT0CCP1_C, F_U1CTS_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_PhB1_C, F_WT1CCP0_C, F_USB0EPEN_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_WT1CCP1_C, F_USB0PFLT_C, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE}
    },
    // GPIO D
    {
        {F_ANALOG, F_SSI3Clk_D, F_SSI1Clk_D, F_I2C3SCL_D, F_M0PWM6_D, F_M1PWM0_D, F_NONE, F_WT2CCP0_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_SSI3Fss_D, F_SSI1Fss_D, F_I2C3SDA_D, F_M0PWM7_D, F_M1PWM1_D, F_NONE, F_WT2CCP1_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_SSI3Rx_D, F_SSI1Rx_D, F_NONE, F_M0FAULT0_D_0, F_NONE, F_NONE, F_WT3CCP0_D, F_USB0EPEN_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_SSI3Tx_D, F_SSI1Tx_D, F_NONE, F_NONE, F_NONE, F_IDX0_D, F_WT3CCP1_D, F_USB0PFLT_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_WT4CCP0_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_WT4CCP1_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_UART, F_NONE, F_NONE, F_M0FAULT0_D_1, F_NONE, F_PhA0_D, F_WT5CCP0_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_PhB0_D, F_WT5CCP1_D, F_NMI_D, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
    },
    // GPIO E
    {
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_I2C2SCL_E, F_M0PWM4_E, F_M1PWM2_E, F_NONE, F_NONE, F_CAN0Rx_E, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_ANALOG, F_UART, F_NONE, F_I2C2SDA_E, F_M0PWM5_E, F_M1PWM3_E, F_NONE, F_NONE, F_CAN0Tx_E, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
    },
    // GPIO F
    {
        {F_NONE, F_U1RTS_F, F_SSI1Rx_F, F_CAN0Rx_F, F_NONE, F_M1PWM4_F, F_PhA0_F, F_TIMER, F_NMI_F, F_C0o_F, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_U1CTS_F, F_SSI1Tx_F, F_NONE, F_NONE, F_M1PWM5_F, F_PhB0_F, F_TIMER, F_NONE, F_C1o_F, F_NONE, F_NONE, F_NONE, F_NONE, F_TRD1_F, F_NONE},
        {F_NONE, F_NONE, F_SSI1Clk_F, F_NONE, F_M0FAULT0_F, F_M1PWM6_F, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TRD0_F, F_NONE},
        {F_NONE, F_NONE, F_SSI1Fss_F, F_CAN0Tx_F, F_NONE, F_M1PWM7_F, F_NONE, F_TIMER, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_TRCLK_F, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_M1FAULT0_F, F_IDX0_F, F_TIMER, F_USB0EPEN_F, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE},
        {F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE, F_NONE}
    }
};

static const VMStateDescription vmstate_gpio = {
    .name = TYPE_TM4_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(data, GPIOState),
        VMSTATE_UINT32(dir, GPIOState),
        VMSTATE_UINT32(is, GPIOState),
        VMSTATE_UINT32(ibe, GPIOState),
        VMSTATE_UINT32(iev, GPIOState),
        VMSTATE_UINT32(im, GPIOState),
        VMSTATE_UINT32(ris, GPIOState),
        VMSTATE_UINT32(afsel, GPIOState),
        VMSTATE_UINT32(dr2r, GPIOState),
        VMSTATE_UINT32(dr4r, GPIOState),
        VMSTATE_UINT32(dr8r, GPIOState),
        VMSTATE_UINT32(odr, GPIOState),
        VMSTATE_UINT32(pur, GPIOState),
        VMSTATE_UINT32(pdr, GPIOState),
        VMSTATE_UINT32(slr, GPIOState),
        VMSTATE_UINT32(den, GPIOState),
        VMSTATE_UINT32(lock, GPIOState),
        VMSTATE_UINT32(cr, GPIOState),
        VMSTATE_UINT32(amsel, GPIOState),
        VMSTATE_UINT32(pctl, GPIOState),
        VMSTATE_UINT32(adcctl, GPIOState),
        VMSTATE_UINT32(dmactl, GPIOState),
        VMSTATE_CLOCK(clk, GPIOState),
        VMSTATE_END_OF_LIST()
    }
};

/**
 * Gets a mask of floating inputs, which are defined
 * as pins configured as an input and neither 1 or 0.
 */
static uint8_t gpio_floating(GPIOState *s)
{
    uint8_t floating = ~(s->pur | s->pdr);
    return floating & ~s->dir;
}

/**
 * Gets a mask of input pins pulled to high (1)
 */
static uint8_t gpio_pullups(GPIOState *s)
{
    return s->pur & ~s->dir;
}

/**
 * Update the IRQs based on new state
 */
static void gpio_update(GPIOState *s)
{
    uint8_t delta, mask, den, out, iev_edges;
    int8_t fun;
    int i;
    uint8_t pullups = gpio_pullups(s);
    uint8_t floating = gpio_floating(s);

    uint8_t data_out = s->data & s->dir;
    uint8_t floating_out = s->old_out_data & floating;

    /*
    Pins configured as outputs are driven from the data register;
    otherwise if pulled up they are 1, and if they're floating, we
    keep the same value as before.
    */
    out = data_out | pullups | floating_out;
    delta = s->old_out_data ^ out;
    if (delta) {
        s->old_out_data = out;
        for (i = 0; i < N_GPIO_BITS; i++) {
            mask = 1 << i;
            if (!(delta & mask)) {
                continue;
            }
            den = s->den & mask;
            if (den) {
                // Digital
                if (s->afsel & mask) {
                    // Alt func
                    uint8_t pctl = (s->pctl >> (i * 4)) & 0x0f;
                    fun = GPIO_ALTERNATE_FUNCTIONS[s->port][i][pctl];
                    if (fun != F_NONE) {
                        qemu_set_irq(s->outputs[i][fun], s->levels[i] == 3300);
                    }
                } else {
                    // Default
                    int level = (out & mask) != 0;
                    qemu_set_irq(s->outputs[i][F_GENERAL], level);
                }
            } else if (s->amsel & mask) {
                // Analog
                fun = GPIO_ALTERNATE_FUNCTIONS[s->port][i][F_ANALOG];
                if (fun != F_NONE) {
                    qemu_set_irq(s->outputs[i][fun], s->levels[i]);
                }
            }
        }
    }

    // IEV selected edges, where both iev and data are the same
    // The check for if the bit has changed comes later
    iev_edges = ~(s->data ^ s->iev);
    /*
    Input delta calculated by xoring the data reg
    with old in status and masking with the input pins.
    */
    delta = (s->old_in_data ^ s->data) & ~s->dir;
    if (delta) {
        s->old_in_data = s->data;
        for (i = 0; i < N_GPIO_BITS; i++) {
            mask = 1 << i;
            // Check bit i changed
            if (!(delta & mask)) {
                continue;
            }
            // Check bit i is sensing edge interrupts
            if (s->is & mask) {
                continue;
            }
            // Edge interrupts
            if (!(s->is & mask)) {
                // Any edge triggers interrupt
                if (s->ibe & mask) {
                    s->ris |= mask;
                }
                // Selected by IEV
                else {
                    s->ris |= iev_edges & mask;
                }
            }
        }
    }

    // Level interrupts 
    s->ris |= iev_edges & s->is;
    
    qemu_set_irq(s->nvic_irq, (s->ris & s->im) != 0);
}

static void gpio_input_handler(void *opaque, int irq, int level)
{
    GPIOState *s = opaque;

    uint8_t pin = irq / 16;
    uint8_t fun = irq % 16;

    uint8_t mask = 1 << pin;

    // TODO: See if this is needed?
    // // Make sure pin is an input
    // if (s->dir & mask) {
    //     return;
    // }

    if (s->den & mask) {
        // Digital
        uint8_t pctl = (s->pctl >> (4 * pin)) & 0xf;
        if (pctl != fun) {
            return;
        }
        // Clear data bit
        s->data &= ~mask;
        // Set data bit according to level
        if (level) {
            s->levels[pin] = 3300;
            s->data |= mask;
        } else {
            s->levels[pin] = 0;
        }
        qemu_set_irq(s->outputs[pin][fun], s->levels[pin] == 3300);
    } else if (s->amsel & mask) {
        // Analog
        // Clear pur, pdr, and data regs for this pin
        s->pur &= ~mask;
        s->pdr &= ~mask;
        s->data &= ~mask;
        s->levels[pin] = level;
        // Set output irq for pin to analog level
        // qemu_set_irq(s->outputs[pin][F_NONE], level);
        qemu_set_irq(s->outputs[pin][fun], level);
    } else {
        return;
    }

    // gpio_update(s);
}

static uint64_t gpio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    GPIOState *s = opaque;
    
    switch (offset) {
    case 0x000 ... 0x3ff:
        return s->data & (offset >> 2);
    case 0x400:
        return s->dir;
    case 0x404:
        return s->is;
    case 0x408:
        return s->ibe;
    case 0x40c:
        return s->iev;
    case 0x410:
        return s->im;
    case 0x414:
        return s->ris;
    case 0x418:
        return s->ris & s->im;
    case 0x420:
        return s->afsel;
    case 0x500:
        return s->dr2r;
    case 0x504:
        return s->dr4r;
    case 0x508:
        return s->dr8r;
    case 0x50c:
        return s->odr;
    case 0x510:
        return s->pur;
    case 0x514:
        return s->pdr;
    case  0x518:
        return s->slr;
    case 0x51c:
        return s->den;
    case 0x520:
        return s->lock;
    case 0x524:
        return s->cr;
    case 0x528:
        return s->amsel;
    case 0x52c:
        return s->pctl;
    case 0x530:
        return s->adcctl;
    case 0x534:
        return s->dmactl;
    case 0xfd0 ... 0xfff:
        return s->id[(offset - 0xfd0) >> 2];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gpio_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void gpio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    GPIOState *s = opaque;

    if (!s->clock_active) {
        return;
    }

    uint8_t mask;
    uint8_t value8 = value & 0xff;

    uint32_t output = value;
    const char *reg = "BAD OFFSET";

    switch (offset) {
    case 0x000 ... 0x3ff:
        reg = "DATA";
        mask = (offset >> 2) & s->dir;
        s->data = (s->data & ~mask) | (value & mask);
        output = value & mask;
        break;
    case 0x400:
        reg = "DIR";
        s->dir = value8;
        break;
    case 0x404:
        reg = "IS";
        s->is = value8;
        break;
    case 0x408:
        reg = "IBE";
        s->ibe = value8;
        break;
    case 0x40c:
        reg = "IEV";
        s->iev = value8;
        break;
    case 0x410:
        reg = "IM";
        s->im = value8;
        break;
    case 0x41c:
        reg = "ICR";
        s->ris &= ~value;
        break;
    case 0x420:
        reg = "AFSEL";
        mask = s->cr;
        s->afsel = (s->afsel & ~mask) | (value & mask);
        output = s->afsel;
        break;
    case 0x500:
        reg = "DR2R";
        s->dr2r = value8;
        break;
    case 0x504:
        reg = "DR4R";
        s->dr4r = value8;
        break;
    case 0x508:
        reg = "DR8R";
        s->dr8r = value8;
        break;
    case 0x50c:
        reg = "ODR";
        s->odr = value8;
        break;
    case 0x510:
        reg = "PUR";
        mask = s->cr;
        s->pur = (s->pur & ~mask) | (value & mask);
        output = s->pur;
        break;
    case 0x514:
        reg = "PDR";
        mask = s->cr;
        s->pdr = (s->pdr & ~mask) | (value & mask);
        output = s->pdr;
        break;
    case  0x518:
        reg = "SLR";
        s->slr = value8;
        break;
    case 0x51c:
        reg = "DEN";
        mask = s->cr;
        s->den = (s->den & ~mask) | (value & mask);
        output = s->den;
        break;
    case 0x520:
        reg = "LOCK";
        // Locked unless you write 0x4c4f434b as per datasheet
        s->lock = (value != 0x4c4f434b);
        output = s->lock;
        break;
    case 0x524:
        reg = "CR";
        output = 0;
        if (s->lock) break;
        s->cr = value8;
        output = value8;
        break;
    case 0x528:
        reg = "AMSEL";
        s->amsel = value8;
        break;
    case 0x52c:
        reg = "PCTL";
        s->pctl = value;
        break;
    case 0x530:
        reg = "ADCCTL";
        s->adcctl = value8;
        break;
    case 0x534:
        reg = "DMACTL";
        s->dmactl = value8;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gpio_write: Bad offset %x\n", (int)offset);
    }

    if (s->debug) {
        printf("[GPIO %c %s] 0x%.2X\n", s->port + 'A', reg, output);
    }
    gpio_update(s);
}

static void gpio_enter_reset(Object *obj, ResetType type)
{
    GPIOState *s = TM4_GPIO(obj);

    s->data = 0;
    s->dir = 0;
    s->is = 0;
    s->ibe = 0;
    s->iev = 0;
    s->im = 0;
    s->ris = 0;
    s->afsel = 0;
    s->dr2r = 0xff;
    s->dr4r = 0;
    s->dr8r = 0;
    s->odr = 0;
    s->pur = 0;
    s->pdr = 0;
    s->slr = 0;
    s->den = 0;
    s->lock = 1;
    s->cr = 0xff;
    s->amsel = 0;
    s->pctl = 0;
    s->adcctl = 0;
    s->dmactl = 0;
}

static void gpio_hold_reset(Object *obj, ResetType type)
{
    GPIOState *s = TM4_GPIO(obj);

    uint8_t mask;
    int i;
    uint8_t floating = gpio_floating(s);
    uint8_t pullups = gpio_pullups(s);

    for (i = 0; i < N_GPIO_BITS; i++) {
        mask = 1 << i;
        if (floating & mask) {
            continue;
        }
        qemu_set_irq(s->outputs[i][0], pullups & mask);
    }
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void check_clock(void *opaque, ClockEvent event)
{
    if (event != ClockUpdate) {
        return;
    }
    
    GPIOState *s = opaque;
    s->clock_active = clock_get(s->clk) != 0;
}

static void gpio_init(Object *obj)
{
    GPIOState *s = TM4_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->id = pl061_id_luminary;

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s, TYPE_TM4_GPIO, 0x1000);
    
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->nvic_irq);

    // Initialize Outputs
    int i;
    for (i = 0; i < N_GPIO_BITS; i++) {
        qdev_init_gpio_out_named(dev, s->outputs[i], GPIO_NAMED_PINS[i], N_PCTL_OPTS);
    }
    // Initialize inputs
    qdev_init_gpio_in(dev, gpio_input_handler, N_GPIO_TABLE);

    s->clk = qdev_init_clock_in(dev, "clk", check_clock, s, ClockUpdate);
}

static void gpio_realize(DeviceState *dev, Error **errp)
{
    GPIOState *s = TM4_GPIO(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "gpio: clk must be connected");
        return;
    }
    if (s->pur > 0xff) {
        error_setg(errp, "pullups property must be between 0 and 0xff");
        return;
    }
    if (s->pdr > 0xff) {
        error_setg(errp, "pulldowns property must be between 0 and 0xff");
        return;
    }
    if (s->pur & s->pdr) {
        error_setg(errp, "no bit may be set both in pullups and pulldowns");
        return;
    }
}

static Property gpio_props[] = 
{
    DEFINE_PROP_UINT32("pur", GPIOState, pur, 0xff),
    DEFINE_PROP_UINT32("pdr", GPIOState, pdr, 0x0),
    DEFINE_PROP_UINT8("port", GPIOState, port, -1),
    DEFINE_PROP_BOOL("debug", GPIOState, debug, false),
    DEFINE_PROP_END_OF_LIST()
};

static void gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_gpio;
    dc->realize = gpio_realize;
    device_class_set_props(dc, gpio_props);

    rc->phases.enter = gpio_enter_reset;
    rc->phases.hold = gpio_hold_reset;
}

static const TypeInfo gpio_info =
{
    .name = TYPE_TM4_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GPIOState),
    .instance_init = gpio_init,
    .class_init = gpio_class_init
};

static void gpio_register_types(void)
{
    type_register_static(&gpio_info);
}
type_init(gpio_register_types)
