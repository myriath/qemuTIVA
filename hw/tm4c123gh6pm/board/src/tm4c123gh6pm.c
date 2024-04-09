#include "hw/tm4c123gh6pm/board/include/tm4c123gh6pm.h"

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
    clock_set_ns(s->sysclk, period_ns);
    if (propagate_clock) {
        clock_propagate(s->sysclk);
    }
}

static void ssys_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    // TODO
    switch (offset) {
    case 0x030: /* PBORCTL */
        s->pborctl = value;
        break;
    case 0x054: /* IMC */
        s->imc = value;
        break;
    case 0x058: /* MISC */
        s->ris &= ~value;
        break;
    case 0x05c: /* RESC */
        s->resc = value;
        break;
    case 0x060: /* RCC */
        if ((s->rcc & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->ris |= (1 << 6);
        }
        s->rcc = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x06c:
        s->gpiohbctl = value;
        break;
    case 0x070: /* RCC2 */
        if ((s->rcc2 & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->ris |= (1 << 6);
        }
        s->rcc2 = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x07c:
        s->moscctl = value;
        break;
    case 0x144:
        s->dslpclkcfg = value;
        break;
    case 0x150:
        s->piosccal = value;
        break;
    case 0x188:
        s->slppwrcfg = value;
        break;
    case 0x18c:
        s->dslppwrcfg = value;
        break;
    case 0x1b4:
        s->ldospctl = value;
        break;
    case 0x1bc:
        s->ldodpctl = value;
        break;
    case 0x500:
        s->srwd = value;
        break;
    case 0x504:
        s->srtimer = value;
        break;
    case 0x508:
        s->srgpio = value;
        break;
    case 0x50c:
        s->srdma = value;
        break;
    case 0x514:
        s->srhib = value;
        break;
    case 0x518:
        s->sruart = value;
        break;
    case 0x51c:
        s->srssi = value;
        break;
    case 0x520:
        s->sri2c = value;
        break;
    case 0x528:
        s->srusb = value;
        break;
    case 0x534:
        s->srcan = value;
        break;
    case 0x538:
        s->sradc = value;
        break;
    case 0x53c:
        s->sracmp = value;
        break;
    case 0x540:
        s->srpwm = value;
        break;
    case 0x544:
        s->srqei = value;
        break;
    case 0x558:
        s->sreeprom = value;
        break;
    case 0x55c:
        s->srwtimer = value;
        break;
    case 0x600:
        s->rcgcwd = value;
        break;
    case 0x604:
        s->rcgctimer = value;
        break;
    case 0x608:
        s->rcgcgpio = value;
        break;
    case 0x60c:
        s->rcgcdma = value;
        break;
    case 0x614:
        s->rcgchib = value;
        break;
    case 0x618:
        s->rcgcuart = value;
        break;
    case 0x61c:
        s->rcgcssi = value;
        break;
    case 0x620:
        s->rcgci2c = value;
        break;
    case 0x628:
        s->rcgcusb = value;
        break;
    case 0x634:
        s->rcgccan = value;
        break;
    case 0x638:
        s->rcgcadc = value;
        break;
    case 0x63c:
        s->rcgcacmp = value;
        break;
    case 0x640:
        s->rcgcpwm = value;
        break;
    case 0x644:
        s->rcgcqei = value;
        break;
    case 0x658:
        s->rcgceeprom = value;
        break;
    case 0x65c:
        s->rcgcwtimer = value;
        break;
    case 0x700:
        s->scgcwd = value;
        break;
    case 0x704:
        s->scgctimer = value;
        break;
    case 0x708:
        s->scgcgpio = value;
        break;
    case 0x70c:
        s->scgcdma = value;
        break;
    case 0x714:
        s->scgchib = value;
        break;
    case 0x718:
        s->scgcuart = value;
        break;
    case 0x71c:
        s->scgcssi = value;
        break;
    case 0x720:
        s->scgci2c = value;
        break;
    case 0x728:
        s->scgcusb = value;
        break;
    case 0x734:
        s->scgccan = value;
        break;
    case 0x738:
        s->scgcadc = value;
        break;
    case 0x73c:
        s->scgcacmp = value;
        break;
    case 0x740:
        s->scgcpwm = value;
        break;
    case 0x744:
        s->scgcqei = value;
        break;
    case 0x758:
        s->scgceeprom = value;
        break;
    case 0x75c:
        s->scgcwtimer = value;
        break;
    case 0x800:
        s->dcgcwd = value;
        break;
    case 0x804:
        s->dcgctimer = value;
        break;
    case 0x808:
        s->dcgcgpio = value;
        break;
    case 0x80c:
        s->dcgcdma = value;
        break;
    case 0x814:
        s->dcgchib = value;
        break;
    case 0x818:
        s->dcgcuart = value;
        break;
    case 0x81c:
        s->dcgcssi = value;
        break;
    case 0x820:
        s->dcgci2c = value;
        break;
    case 0x828:
        s->dcgcusb = value;
        break;
    case 0x834:
        s->dcgccan = value;
        break;
    case 0x838:
        s->dcgcadc = value;
        break;
    case 0x83c:
        s->dcgcacmp = value;
        break;
    case 0x840:
        s->dcgcpwm = value;
        break;
    case 0x844:
        s->dcgcqei = value;
        break;
    case 0x858:
        s->dcgceeprom = value;
        break;
    case 0x85c:
        s->dcgcwtimer = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SSYS: write at bad offset 0x%x\n", (int)offset);
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

    // TODO
    s->pborctl = 0x7ffd;
    s->rcc = 0x078e3ad1;
    s->rcc2 = 0x07c06810;

    s->rcgc[0] = 1;
    s->scgc[0] = 1;
    s->dcgc[0] = 1;
}

static void tm4c123gh6pm_sys_reset_hold(Object *obj)
{
    ssys_state *s = TM4_SYS(obj);

    /* OK to propagate clocks from the hold phase */
    ssys_calculate_system_clock(s, true);
}

static void tm4c123gh6pm_sys_reset_exit(Object *obj)
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
    DEFINE_PROP_END_OF_LIST()
};

static void tm4c123gh6pm_sys_instance_init(Object *obj)
{
    ssys_state *s = TM4_SYS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(s);

    memory_region_init_io(&s->iomem, obj, &ssys_ops, s, "ssys", 0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->sysclk = qdev_init_clock_out(DEVICE(s), "SYSCLK");
}


static void tm4c123gh6pm_init(MachineState *ms)
{
    static const uint32_t uart_addr[8] = {
        0x4000c000, 0x4000d000, 0x4000e000, 0x4000f000,
        0x40010000, 0x40011000, 0x40012000, 0x40013000
    };
    static const int uart_irq[8] = {5, 6, 33, 59, 60, 61, 62, 63};

    // static const uint32_t timer_addr[2][6] = {
    //     {
    //         0x40030000, 0x40031000, 0x40032000,
    //         0x40033000, 0x40034000, 0x40035000
    //     },
    //     {
    //         0x40036000, 0x40037000, 0x4004c000,
    //         0x4004d000, 0x4004e000, 0x4004f000
    //     }
    // };
    // static const int timer_irq[2][6] = {{19, 21, 23, 35, 70, 92}, {94, 96, 98, 100, 102, 104}};

    static const uint32_t gpio_addr[6] = { 
        0x40004000, 0x40005000, 0x40006000, 0x40007000,
        0x40024000, 0x40025000
    };
    static const int gpio_irq[6] = {0, 1, 2, 3, 4, 30};

    static const uint32_t adc_addr[2] = {0x40038000, 0x40039000};
    static const int adc_irq[2][4] = {{14, 15, 16, 17}, {48, 49, 50, 51}};

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
     * 40036000 32/64 timer 0
     * 40037000 32/64 timer 1
     * 40038000 ADC0
     * 40039000 ADC1
     * 4003c000 analogue comparator (unimplemented)
     * 40040000 CAN0 (unimplemented)
     * 40041000 CAN1 (unimplemented)
     * 4004c000 32/64 timer 2
     * 4004d000 32/64 timer 3
     * 4004e000 32/64 timer 4
     * 4004f000 32/64 timer 5
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

    DeviceState *gpio_dev[6], *nvic;
    qemu_irq gpio_in[6][8];
    // qemu_irq gpio_out[6][8];
    DeviceState *dev;
    DeviceState *ssys_dev;
    int i;
    int j;

    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *system_memory = get_system_memory();

    int flash_size = 256 * 1024;
    int sram_size = 32 * 1024;

    Object *soc_container = object_new("container");
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
    ssys_dev = qdev_new(TYPE_TM4_SYS);
    object_property_add_child(soc_container, "sys", OBJECT(ssys_dev));

    // Values fetched from CyBot
    qdev_prop_set_uint32(ssys_dev, "did0", 0x18050102);
    qdev_prop_set_uint32(ssys_dev, "did1", 0x10a1606e);
    qdev_prop_set_uint32(ssys_dev, "dc0",  0x007f007f);
    qdev_prop_set_uint32(ssys_dev, "dc1",  0x13332fff);
    qdev_prop_set_uint32(ssys_dev, "dc2",  0x030ff337);
    qdev_prop_set_uint32(ssys_dev, "dc3",  0xbfff8fff);
    qdev_prop_set_uint32(ssys_dev, "dc4",  0x0004f03f);
    qdev_prop_set_uint32(ssys_dev, "dc5",  0x013000ff);
    qdev_prop_set_uint32(ssys_dev, "dc6",  0x00000013);
    qdev_prop_set_uint32(ssys_dev, "dc7",  0xffffffff);
    qdev_prop_set_uint32(ssys_dev, "dc8",  0x0fff0fff);
    qdev_prop_set_uint32(ssys_dev, "dc9",  0x00ff00ff);
    qdev_prop_set_uint32(ssys_dev, "ppwd", 0x3);
    qdev_prop_set_uint32(ssys_dev, "pptimer", 0x3f);
    qdev_prop_set_uint32(ssys_dev, "ppgpio", 0x3f);
    qdev_prop_set_uint32(ssys_dev, "ppdma", 0x1);
    qdev_prop_set_uint32(ssys_dev, "pphib", 0x1);
    qdev_prop_set_uint32(ssys_dev, "ppuart", 0xff);
    qdev_prop_set_uint32(ssys_dev, "ppssi", 0xf);
    qdev_prop_set_uint32(ssys_dev, "ppi2c", 0xf);
    qdev_prop_set_uint32(ssys_dev, "ppusb", 0x1);
    qdev_prop_set_uint32(ssys_dev, "ppcan", 0x3);
    qdev_prop_set_uint32(ssys_dev, "ppadc", 0x3);
    qdev_prop_set_uint32(ssys_dev, "ppacmp", 0x1);
    qdev_prop_set_uint32(ssys_dev, "pppwm", 0x3);
    qdev_prop_set_uint32(ssys_dev, "ppqei", 0x3);
    qdev_prop_set_uint32(ssys_dev, "ppeeprom", 0x1);
    qdev_prop_set_uint32(ssys_dev, "ppwtimer", 0x3f);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(ssys_dev), &error_fatal);

    nvic = qdev_new(TYPE_ARMV7M);
    object_property_add_child(soc_container, "v7m", OBJECT(nvic));
    qdev_prop_set_uint32(nvic, "num-irq", NUM_IRQ_LINES);
    qdev_prop_set_uint8(nvic, "num-prio-bits", NUM_PRIO_BITS);
    qdev_prop_set_string(nvic, "cpu-type", ms->cpu_type);
    qdev_prop_set_bit(nvic, "enable-bitband", true);
    qdev_connect_clock_in(nvic, "cpuclk",
                          qdev_get_clock_out(ssys_dev, "SYSCLK"));
    /* This SoC does not connect the systick reference clock */
    object_property_set_link(OBJECT(nvic), "memory",
                             OBJECT(get_system_memory()), &error_abort);
    /* This will exit with an error if the user passed us a bad cpu_type */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(nvic), &error_fatal);

    /* Now we can wire up the IRQ and MMIO of the system registers */
    sysbus_mmio_map(SYS_BUS_DEVICE(ssys_dev), 0, 0x400fe000);
    sysbus_connect_irq(SYS_BUS_DEVICE(ssys_dev), 0, qdev_get_gpio_in(nvic, 28)); // Interrupt 28: System Control

    // GPIO
    for (i = 0; i < 6; i++) {
        gpio_dev[i] = sysbus_create_simple("pl061_luminary", gpio_addr[i],
                                            qdev_get_gpio_in(nvic,
                                                            gpio_irq[i]));
        for (j = 0; j < 8; j++) {
            gpio_in[i][j] = qdev_get_gpio_in(gpio_dev[i], j);
            // gpio_out[i][j] = qdev_connect_gpio_out(gpio_dev[i], j);
        }
    }

    // ADC
    dev = sysbus_create_varargs(TYPE_TM4_ADC, adc_addr[0],
                                // ADC Sample Sequence 0-3 Interrupts
                                qdev_get_gpio_in(nvic, adc_irq[0][0]),
                                qdev_get_gpio_in(nvic, adc_irq[0][1]),
                                qdev_get_gpio_in(nvic, adc_irq[0][2]),
                                qdev_get_gpio_in(nvic, adc_irq[0][3]),
                                // AIN inputs
                                gpio_in[GPIO_E][3],
                                gpio_in[GPIO_E][2],
                                gpio_in[GPIO_E][1],
                                gpio_in[GPIO_E][0],
                                gpio_in[GPIO_D][3],
                                gpio_in[GPIO_D][2],
                                gpio_in[GPIO_D][1],
                                gpio_in[GPIO_D][0],
                                gpio_in[GPIO_E][5],
                                gpio_in[GPIO_E][4],
                                gpio_in[GPIO_B][5],
                                gpio_in[GPIO_B][4],
                                NULL);
    // qemu_irq adc0 = qdev_get_gpio_in(dev, 0);
    dev = sysbus_create_varargs(TYPE_TM4_ADC, adc_addr[1],
                                // ADC Sample Sequence 0-3 Interrupts
                                qdev_get_gpio_in(nvic, adc_irq[1][0]),
                                qdev_get_gpio_in(nvic, adc_irq[1][1]),
                                qdev_get_gpio_in(nvic, adc_irq[1][2]),
                                qdev_get_gpio_in(nvic, adc_irq[1][3]),
                                // AIN inputs
                                gpio_in[GPIO_E][3],
                                gpio_in[GPIO_E][2],
                                gpio_in[GPIO_E][1],
                                gpio_in[GPIO_E][0],
                                gpio_in[GPIO_D][3],
                                gpio_in[GPIO_D][2],
                                gpio_in[GPIO_D][1],
                                gpio_in[GPIO_D][0],
                                gpio_in[GPIO_E][5],
                                gpio_in[GPIO_E][4],
                                gpio_in[GPIO_B][5],
                                gpio_in[GPIO_B][4],
                                NULL);
    // qemu_irq adc1 = qdev_get_gpio_in(dev, 0);

    // // Timers
    // for (i = 0; i < 12; i++) {
    //     SysBusDevice *sbd;

    //     dev = qdev_new(TYPE_TM4_TIMER); // TODO
    //     sbd = SYS_BUS_DEVICE(dev);
    //     object_property_add_child(soc_container, "gptm[*]", OBJECT(dev));
    //     qdev_connect_clock_in(dev, "clk",
    //                             qdev_get_clock_out(ssys_dev, "SYSCLK"));
    //     sysbus_realize_and_unref(sbd, &error_fatal);
    //     int width = i / 6;
    //     int index = i % 6;
    //     sysbus_mmio_map(sbd, 0, timer_addr[width][index]);
    //     sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, timer_irq[width][index]));
    //     /* TODO: This is incorrect, but we get away with it because
    //         the ADC output is only ever pulsed.  */
    //     qdev_connect_gpio_out(dev, 0, adc1);
    // }

    // I2C TODO
    // if (board->dc2 & (1 << 12)) {
    //     dev = sysbus_create_simple(TYPE_TM4_I2C, 0x40020000,
    //                                qdev_get_gpio_in(nvic, 8));
    //     i2c = (I2CBus *)qdev_get_child_bus(dev, "i2c");
    //     if (board->peripherals & BP_OLED_I2C) {
    //         i2c_slave_create_simple(i2c, "ssd0303", 0x3d);
    //     }
    // }

    // UART
    for (i = 0; i < 1/*8*/; i++) {
        SysBusDevice *sbd;

        dev = qdev_new("pl011_luminary");
        object_property_add_child(soc_container, "uart[*]", OBJECT(dev));
        sbd = SYS_BUS_DEVICE(dev);
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, uart_addr[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, uart_irq[i]));
    }

    // SSI TODO
    // if (board->dc2 & (1 << 4)) {
    //     dev = sysbus_create_simple("pl022", 0x40008000,
    //                                qdev_get_gpio_in(nvic, 7));
    //     if (board->peripherals & BP_OLED_SSI) {
    //         void *bus;
    //         DeviceState *sddev;
    //         DeviceState *ssddev;
    //         DriveInfo *dinfo;
    //         DeviceState *carddev;
    //         DeviceState *gpio_d_splitter;
    //         BlockBackend *blk;

    //         /*
    //          * Some boards have both an OLED controller and SD card connected to
    //          * the same SSI port, with the SD card chip select connected to a
    //          * GPIO pin.  Technically the OLED chip select is connected to the
    //          * SSI Fss pin.  We do not bother emulating that as both devices
    //          * should never be selected simultaneously, and our OLED controller
    //          * ignores stray 0xff commands that occur when deselecting the SD
    //          * card.
    //          *
    //          * The h/w wiring is:
    //          *  - GPIO pin D0 is wired to the active-low SD card chip select
    //          *  - GPIO pin A3 is wired to the active-low OLED chip select
    //          *  - The SoC wiring of the PL061 "auxiliary function" for A3 is
    //          *    SSI0Fss ("frame signal"), which is an output from the SoC's
    //          *    SSI controller. The SSI controller takes SSI0Fss low when it
    //          *    transmits a frame, so it can work as a chip-select signal.
    //          *  - GPIO A4 is aux-function SSI0Rx, and wired to the SD card Tx
    //          *    (the OLED never sends data to the CPU, so no wiring needed)
    //          *  - GPIO A5 is aux-function SSI0Tx, and wired to the SD card Rx
    //          *    and the OLED display-data-in
    //          *  - GPIO A2 is aux-function SSI0Clk, wired to SD card and OLED
    //          *    serial-clock input
    //          * So a guest that wants to use the OLED can configure the PL061
    //          * to make pins A2, A3, A5 aux-function, so they are connected
    //          * directly to the SSI controller. When the SSI controller sends
    //          * data it asserts SSI0Fss which selects the OLED.
    //          * A guest that wants to use the SD card configures A2, A4 and A5
    //          * as aux-function, but leaves A3 as a software-controlled GPIO
    //          * line. It asserts the SD card chip-select by using the PL061
    //          * to control pin D0, and lets the SSI controller handle Clk, Tx
    //          * and Rx. (The SSI controller asserts Fss during tx cycles as
    //          * usual, but because A3 is not set to aux-function this is not
    //          * forwarded to the OLED, and so the OLED stays unselected.)
    //          *
    //          * The QEMU implementation instead is:
    //          *  - GPIO pin D0 is wired to the active-low SD card chip select,
    //          *    and also to the OLED chip-select which is implemented
    //          *    as *active-high*
    //          *  - SSI controller signals go to the devices regardless of
    //          *    whether the guest programs A2, A4, A5 as aux-function or not
    //          *
    //          * The problem with this implementation is if the guest doesn't
    //          * care about the SD card and only uses the OLED. In that case it
    //          * may choose never to do anything with D0 (leaving it in its
    //          * default floating state, which reliably leaves the card disabled
    //          * because an SD card has a pullup on CS within the card itself),
    //          * and only set up A2, A3, A5. This for us would mean the OLED
    //          * never gets the chip-select assert it needs. We work around
    //          * this with a manual raise of D0 here (despite board creation
    //          * code being the wrong place to raise IRQ lines) to put the OLED
    //          * into an initially selected state.
    //          *
    //          * In theory the right way to model this would be:
    //          *  - Implement aux-function support in the PL061, with an
    //          *    extra set of AFIN and AFOUT GPIO lines (set up so that
    //          *    if a GPIO line is in auxfn mode the main GPIO in and out
    //          *    track the AFIN and AFOUT lines)
    //          *  - Wire the AFOUT for D0 up to either a line from the
    //          *    SSI controller that's pulled low around every transmit,
    //          *    or at least to an always-0 line here on the board
    //          *  - Make the ssd0323 OLED controller chipselect active-low
    //          */
    //         bus = qdev_get_child_bus(dev, "ssi");
    //         sddev = ssi_create_peripheral(bus, "ssi-sd");

    //         dinfo = drive_get(IF_SD, 0, 0);
    //         blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    //         carddev = qdev_new(TYPE_SD_CARD_SPI);
    //         qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
    //         qdev_realize_and_unref(carddev,
    //                                qdev_get_child_bus(sddev, "sd-bus"),
    //                                &error_fatal);

    //         ssddev = qdev_new("ssd0323");
    //         object_property_add_child(OBJECT(ms), "oled", OBJECT(ssddev));
    //         qdev_prop_set_uint8(ssddev, "cs", 1);
    //         qdev_realize_and_unref(ssddev, bus, &error_fatal);

    //         gpio_d_splitter = qdev_new(TYPE_SPLIT_IRQ);
    //         object_property_add_child(OBJECT(ms), "splitter",
    //                                   OBJECT(gpio_d_splitter));
    //         qdev_prop_set_uint32(gpio_d_splitter, "num-lines", 2);
    //         qdev_realize_and_unref(gpio_d_splitter, NULL, &error_fatal);
    //         qdev_connect_gpio_out(
    //                 gpio_d_splitter, 0,
    //                 qdev_get_gpio_in_named(sddev, SSI_GPIO_CS, 0));
    //         qdev_connect_gpio_out(
    //                 gpio_d_splitter, 1,
    //                 qdev_get_gpio_in_named(ssddev, SSI_GPIO_CS, 0));
    //         gpio_out[GPIO_D][0] = qdev_get_gpio_in(gpio_d_splitter, 0);

    //         gpio_out[GPIO_C][7] = qdev_get_gpio_in(ssddev, 0);

    //         /* Make sure the select pin is high.  */
    //         qemu_irq_raise(gpio_out[GPIO_D][0]);
    //     }
    // }

    // // Connect GPIO outs
    // for (i = 0; i < 6; i++) {
    //     for (j = 0; j < 8; j++) {
    //         if (gpio_out[i][j]) {
    //             qdev_connect_gpio_out(gpio_dev[i], j, gpio_out[i][j]);
    //         }
    //     }
    // }

    /* Add dummy regions for the devices we don't implement yet,
     * so guest accesses don't cause unlogged crashes.
     */
    // create_unimplemented_device("watchdog-0", 0x40000000, 0x1000);
    // create_unimplemented_device("watchdog-1", 0x40001000, 0x1000);
    // create_unimplemented_device("ssi0", 0x40008000, 0x1000);
    // create_unimplemented_device("ssi1", 0x40009000, 0x1000);
    // create_unimplemented_device("ssi2", 0x4000a000, 0x1000);
    // create_unimplemented_device("ssi3", 0x4000b000, 0x1000);
    // create_unimplemented_device("i2c-0", 0x40020000, 0x1000);
    // create_unimplemented_device("i2c-1", 0x40021000, 0x1000);
    // create_unimplemented_device("i2c-2", 0x40022000, 0x1000);
    // create_unimplemented_device("i2c-3", 0x40023000, 0x1000);
    // create_unimplemented_device("PWM0", 0x40028000, 0x1000);
    // create_unimplemented_device("PWM1", 0x40029000, 0x1000);
    // create_unimplemented_device("QEI0", 0x4002c000, 0x1000);
    // create_unimplemented_device("QEI1", 0x4002d000, 0x1000);
    // create_unimplemented_device("analogue-comparator", 0x4003c000, 0x1000);
    // create_unimplemented_device("can0", 0x40040000, 0x1000);
    // create_unimplemented_device("can1", 0x40041000, 0x1000);
    // create_unimplemented_device("usb", 0x40050000, 0x1000);
    // create_unimplemented_device("eeprom-keylocker", 0x400af000, 0x1000);
    // create_unimplemented_device("system-exception", 0x400f9000, 0x1000);
    // create_unimplemented_device("hibernation", 0x400fc000, 0x1000);
    // create_unimplemented_device("flash-control", 0x400fd000, 0x1000);
    // create_unimplemented_device("udma", 0x400ff000, 0x1000);

    armv7m_load_kernel(ARM_CPU(first_cpu), ms->kernel_filename, 0, flash_size);
}


static void tm4c123gh6pm_machine_init(MachineClass *mc) 
{
    mc->desc = "Tiva TM4C123GH6PM (Cortex-M4F)";
    mc->init = tm4c123gh6pm_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4"); // TODO: cortex-m4f
}

static void tm4c123gh6pm_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    tm4c123gh6pm_machine_init(mc);
}

static const TypeInfo tm4c123gh6pm_type = {
    .name = MACHINE_TYPE_NAME("tm4c123gh6pm"),
    .parent = TYPE_MACHINE,
    .class_init = tm4c123gh6pm_class_init,
};

static void tm4c123gh6pm_machine_init_register_types(void)
{
    type_register_static(&tm4c123gh6pm_type);
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
