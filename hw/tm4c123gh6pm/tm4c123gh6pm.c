/*
 * Luminary Micro Stellaris peripherals
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */
#include "adc.h"
#include "i2c.h"

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/split-irq.h"
#include "hw/sysbus.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "hw/arm/boot.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "net/net.h"
#include "hw/boards.h"
#include "qemu/log.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/arm/armv7m.h"
#include "hw/char/pl011.h"
#include "hw/input/stellaris_gamepad.h"
#include "hw/irq.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "migration/vmstate.h"
#include "hw/misc/unimp.h"
#include "hw/timer/stellaris-gptm.h"
#include "hw/qdev-clock.h"
#include "qom/object.h"
#include "qapi/qmp/qlist.h"
#include "ui/input.h"

#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5

#define BP_OLED_I2C  0x01
#define BP_OLED_SSI  0x02
#define BP_GAMEPAD   0x04

#define NUM_IRQ_LINES 138
#define NUM_PRIO_BITS 3


/* System controller.  */
#define TYPE_TM4_SYS "tm4-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ssys_state, TM4_SYS)

struct ssys_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t pborctl;
    uint32_t ris_int_status;
    uint32_t imc_int_mask;
    uint32_t resc;
    uint32_t rcc;
    uint32_t gpiohbctl;
    uint32_t rcc2;
    uint32_t moscctl;
    uint32_t dslpclkcfg;
    uint32_t piosccal;
    uint32_t slppwrcfg;
    uint32_t dslppwrcfg;
    uint32_t ldospctl;
    uint32_t ldodpctl;
    /* Software resets */
    uint32_t srwd;
    uint32_t srtimer;
    uint32_t srgpio;
    uint32_t srdma;
    uint32_t srhib;
    uint32_t sruart;
    uint32_t srssi;
    uint32_t sri2c;
    uint32_t srusb;
    uint32_t srcan;
    uint32_t sradc;
    uint32_t sracmp;
    uint32_t srpwm;
    uint32_t srqei;
    uint32_t sreeprom;
    uint32_t srwtimer;
    /* Run mode clock gating controls */
    uint32_t rcgcwd;
    uint32_t rcgctimer;
    uint32_t rcgcgpio;
    uint32_t rcgcdma;
    uint32_t rcgchib;
    uint32_t rcgcuart;
    uint32_t rcgcssi;
    uint32_t rcgci2c;
    uint32_t rcgcusb;
    uint32_t rcgccan;
    uint32_t rcgcadc;
    uint32_t rcgcacmp;
    uint32_t rcgcpwm;
    uint32_t rcgcqei;
    uint32_t rcgceeprom;
    uint32_t rcgcwtimer;
    /* sleep mode clock gating controls */
    uint32_t scgcwd;
    uint32_t scgctimer;
    uint32_t scgcgpio;
    uint32_t scgcdma;
    uint32_t scgchib;
    uint32_t scgcuart;
    uint32_t scgcssi;
    uint32_t scgci2c;
    uint32_t scgcusb;
    uint32_t scgccan;
    uint32_t scgcadc;
    uint32_t scgcacmp;
    uint32_t scgcpwm;
    uint32_t scgcqei;
    uint32_t scgceeprom;
    uint32_t scgcwtimer;
    /* deep-sleep clock gating controls */
    uint32_t dcgcwd;
    uint32_t dcgctimer;
    uint32_t dcgcgpio;
    uint32_t dcgcdma;
    uint32_t dcgchib;
    uint32_t dcgcuart;
    uint32_t dcgcssi;
    uint32_t dcgci2c;
    uint32_t dcgcusb;
    uint32_t dcgccan;
    uint32_t dcgcadc;
    uint32_t dcgcacmp;
    uint32_t dcgcpwm;
    uint32_t dcgcqei;
    uint32_t dcgceeprom;
    uint32_t dcgcwtimer;

    /* Properties (read-only) */
    /* System */
    uint32_t did[2];
    uint32_t sysprop;
    uint32_t pioscstat;
    uint32_t pllfreq0;
    uint32_t pllfreq1;
    uint32_t pllstat;
    uint32_t ldospcal;
    uint32_t ldodpcal;
    uint32_t sdpmst;
    /* Peripherals present */
    uint32_t ppwd;
    uint32_t pptimer;
    uint32_t ppgpio;
    uint32_t ppdma;
    uint32_t pphib;
    uint32_t ppuart;
    uint32_t ppssi;
    uint32_t ppi2c;
    uint32_t ppusb;
    uint32_t ppcan;
    uint32_t ppadc;
    uint32_t ppacmp;
    uint32_t pppwm;
    uint32_t ppqei;
    uint32_t ppeeprom;
    uint32_t ppwtimer;
    /* peripherals ready */
    uint32_t prwd;
    uint32_t prtimer;
    uint32_t prgpio;
    uint32_t prdma;
    uint32_t prhib;
    uint32_t pruart;
    uint32_t prssi;
    uint32_t pri2c;
    uint32_t prusb;
    uint32_t prcan;
    uint32_t pradc;
    uint32_t pracmp;
    uint32_t prpwm;
    uint32_t prqei;
    uint32_t preeprom;
    uint32_t prwtimer;
    /* Legacy control registers */
    uint32_t dc[10];
    uint32_t srcr[3];
    uint32_t rcgc[3];
    uint32_t scgc[3];
    uint32_t dcgc[3];
    uint32_t nvmstat;

    qemu_irq irq;
    Clock *sysclk;
};

static void ssys_update(ssys_state *s)
{
    qemu_set_irq(s->irq, (s->int_status & s->int_mask) != 0);
}

// TODO
static uint32_t pllcfg_sandstorm[16] = {
    0x31c0, /* 1 Mhz */
    0x1ae0, /* 1.8432 Mhz */
    0x18c0, /* 2 Mhz */
    0xd573, /* 2.4576 Mhz */
    0x37a6, /* 3.57954 Mhz */
    0x1ae2, /* 3.6864 Mhz */
    0x0c40, /* 4 Mhz */
    0x98bc, /* 4.906 Mhz */
    0x935b, /* 4.9152 Mhz */
    0x09c0, /* 5 Mhz */
    0x4dee, /* 5.12 Mhz */
    0x0c41, /* 6 Mhz */
    0x75db, /* 6.144 Mhz */
    0x1ae6, /* 7.3728 Mhz */
    0x0600, /* 8 Mhz */
    0x585b /* 8.192 Mhz */
};

static uint32_t pllcfg_fury[16] = {
    0x3200, /* 1 Mhz */
    0x1b20, /* 1.8432 Mhz */
    0x1900, /* 2 Mhz */
    0xf42b, /* 2.4576 Mhz */
    0x37e3, /* 3.57954 Mhz */
    0x1b21, /* 3.6864 Mhz */
    0x0c80, /* 4 Mhz */
    0x98ee, /* 4.906 Mhz */
    0xd5b4, /* 4.9152 Mhz */
    0x0a00, /* 5 Mhz */
    0x4e27, /* 5.12 Mhz */
    0x1902, /* 6 Mhz */
    0xec1c, /* 6.144 Mhz */
    0x1b23, /* 7.3728 Mhz */
    0x0640, /* 8 Mhz */
    0xb11c /* 8.192 Mhz */
};

#define DID0_VER_MASK        0x70000000
#define DID0_VER_0           0x00000000
#define DID0_VER_1           0x10000000

#define DID0_CLASS_MASK      0x00FF0000
#define DID0_CLASS_SANDSTORM 0x00000000
#define DID0_CLASS_FURY      0x00010000

static int ssys_board_class(const ssys_state *s)
{
    // TODO
    uint32_t did0 = s->did0;
    switch (did0 & DID0_VER_MASK) {
    case DID0_VER_0:
        return DID0_CLASS_SANDSTORM;
    case DID0_VER_1:
        switch (did0 & DID0_CLASS_MASK) {
        case DID0_CLASS_SANDSTORM:
        case DID0_CLASS_FURY:
            return did0 & DID0_CLASS_MASK;
        }
        /* for unknown classes, fall through */
    default:
        /* This can only happen if the hardwired constant did0 value
         * in this board's stellaris_board_info struct is wrong.
         */
        g_assert_not_reached();
    }
}

static uint64_t ssys_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    ssys_state *s = (ssys_state *)opaque;

    // TODO
    switch (offset) {
    case 0x000: /* DID0 */
        return s->did0;
    case 0x004: /* DID1 */
        return s->did1;
    case 0x008: /* DC0 */
        return s->dc0;
    case 0x010: /* DC1 */
        return s->dc1;
    case 0x014: /* DC2 */
        return s->dc2;
    case 0x018: /* DC3 */
        return s->dc3;
    case 0x01c: /* DC4 */
        return s->dc4;
    case 0x030: /* PBORCTL */
        return s->pborctl;
    case 0x034: /* LDOPCTL */
        return s->ldopctl;
    case 0x040: /* SRCR0 */
        return 0;
    case 0x044: /* SRCR1 */
        return 0;
    case 0x048: /* SRCR2 */
        return 0;
    case 0x050: /* RIS */
        return s->int_status;
    case 0x054: /* IMC */
        return s->int_mask;
    case 0x058: /* MISC */
        return s->int_status & s->int_mask;
    case 0x05c: /* RESC */
        return s->resc;
    case 0x060: /* RCC */
        return s->rcc;
    case 0x064: /* PLLCFG */
        {
            int xtal;
            xtal = (s->rcc >> 6) & 0xf;
            switch (ssys_board_class(s)) {
            case DID0_CLASS_FURY:
                return pllcfg_fury[xtal];
            case DID0_CLASS_SANDSTORM:
                return pllcfg_sandstorm[xtal];
            default:
                g_assert_not_reached();
            }
        }
    case 0x070: /* RCC2 */
        return s->rcc2;
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
    case 0x150: /* CLKVCLR */
        return s->clkvclr;
    case 0x160: /* LDOARST */
        return s->ldoarst;
    case 0x1e0: /* USER0 */
        return s->user0;
    case 0x1e4: /* USER1 */
        return s->user1;
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
        s->pborctl = value & 0xffff;
        break;
    case 0x034: /* LDOPCTL */
        s->ldopctl = value & 0x1f;
        break;
    case 0x040: /* SRCR0 */
    case 0x044: /* SRCR1 */
    case 0x048: /* SRCR2 */
        qemu_log_mask(LOG_UNIMP, "Peripheral reset not implemented\n");
        break;
    case 0x054: /* IMC */
        s->int_mask = value & 0x7f;
        break;
    case 0x058: /* MISC */
        s->int_status &= ~value;
        break;
    case 0x05c: /* RESC */
        s->resc = value & 0x3f;
        break;
    case 0x060: /* RCC */
        if ((s->rcc & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->int_status |= (1 << 6);
        }
        s->rcc = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x070: /* RCC2 */
        if (ssys_board_class(s) == DID0_CLASS_SANDSTORM) {
            break;
        }

        if ((s->rcc2 & (1 << 13)) != 0 && (value & (1 << 13)) == 0) {
            /* PLL enable.  */
            s->int_status |= (1 << 6);
        }
        s->rcc2 = value;
        ssys_calculate_system_clock(s, true);
        break;
    case 0x100: /* RCGC0 */
        s->rcgc[0] = value;
        break;
    case 0x104: /* RCGC1 */
        s->rcgc[1] = value;
        break;
    case 0x108: /* RCGC2 */
        s->rcgc[2] = value;
        break;
    case 0x110: /* SCGC0 */
        s->scgc[0] = value;
        break;
    case 0x114: /* SCGC1 */
        s->scgc[1] = value;
        break;
    case 0x118: /* SCGC2 */
        s->scgc[2] = value;
        break;
    case 0x120: /* DCGC0 */
        s->dcgc[0] = value;
        break;
    case 0x124: /* DCGC1 */
        s->dcgc[1] = value;
        break;
    case 0x128: /* DCGC2 */
        s->dcgc[2] = value;
        break;
    case 0x150: /* CLKVCLR */
        s->clkvclr = value;
        break;
    case 0x160: /* LDOARST */
        s->ldoarst = value;
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
    s->rcc = 0x078e3ac0;

    if (ssys_board_class(s) == DID0_CLASS_SANDSTORM) {
        s->rcc2 = 0;
    } else {
        s->rcc2 = 0x07802810;
    }
    s->rcgc[0] = 1;
    s->scgc[0] = 1;
    s->dcgc[0] = 1;
}

static void tm4c123gh6pm_sys_reset_hold(Object *obj)
{
    ssys_state *s = STELLARIS_SYS(obj);

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
        VMSTATE_UINT32(ldopctl, ssys_state),
        VMSTATE_UINT32(int_mask, ssys_state),
        VMSTATE_UINT32(int_status, ssys_state),
        VMSTATE_UINT32(resc, ssys_state),
        VMSTATE_UINT32(rcc, ssys_state),
        VMSTATE_UINT32_V(rcc2, ssys_state, 2),
        VMSTATE_UINT32_ARRAY(rcgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(scgc, ssys_state, 3),
        VMSTATE_UINT32_ARRAY(dcgc, ssys_state, 3),
        VMSTATE_UINT32(clkvclr, ssys_state),
        VMSTATE_UINT32(ldoarst, ssys_state),
        // TODO
        /* No field for sysclk -- handled in post-load instead */
        VMSTATE_END_OF_LIST()
    }
};

static Property tm4c123gh6pm_sys_properties[] = {
    DEFINE_PROP_UINT32("did0", ssys_state, did0, 0),
    DEFINE_PROP_UINT32("did1", ssys_state, did1, 0),
    DEFINE_PROP_UINT32("dc0", ssys_state, dc0, 0),
    DEFINE_PROP_UINT32("dc1", ssys_state, dc1, 0),
    DEFINE_PROP_UINT32("dc2", ssys_state, dc2, 0),
    DEFINE_PROP_UINT32("dc3", ssys_state, dc3, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc4, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc5, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc6, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc7, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc8, 0),
    DEFINE_PROP_UINT32("dc4", ssys_state, dc9, 0),
    DEFINE_PROP_UINT32("ppwd", ssys_state, ppwd, 0),
    DEFINE_PROP_UINT32("pptimer", ssys_state, pptimer, 0),
    DEFINE_PROP_UINT32("ppgpio", ssys_state, ppgpio, 0),
    DEFINE_PROP_UINT32("ppdma", ssys_state, ppdma, 0),
    DEFINE_PROP_UINT32("pphib", ssys_state, pphib, 0),
    DEFINE_PROP_UINT32("ppuart", ssys_state, ppuart, 0),
    DEFINE_PROP_UINT32("ppssi", ssys_state, ppssi, 0),
    DEFINE_PROP_UINT32("ppi2c", ssys_state, ppi2c, 0,
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

    static const uint32_t gpio_addr[6] = { 
        0x40004000, 0x40005000, 0x40006000, 0x40007000,
        0x40024000, 0x40025000
    };
    static const int gpio_irq[6] = {0, 1, 2, 3, 4, 30};

    static const uint32_t adc_addr[2] = {0x40038000, 0x40039000};
    static const int adc_irq[2][4] = {{14, 15, 16, 17}, {48, 49, 50, 51}}

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
     * 40058000 GPIO A (AHB)
     * 40059000 GPIO B (AHB)
     * 4005a000 GPIO C (AHB)
     * 4005b000 GPIO D (AHB)
     * 4005c000 GPIO E (AHB)
     * 4005d000 GPIO F (AHB)
     * 400af000 EEPROM & key locker (unimplemented)
     * 400f9000 System Exception module (unimplemented)
     * 400fc000 hibernation module (unimplemented)
     * 400fd000 flash memory control (unimplemented)
     * 400fe000 system control
     * 400ff000 uDMA (unimplemented)
     * 44000000 bit-banded alias 40000000 - 400fffff
     * 
     * Private Peripheral Bus
     * e0000000 ITM
     * e0001000 DWT
     * e0002000 FPB
     * e000e000 Cortex-M4F Peripherals
     * e0040000 TPIU
     * e0041000 ETM
     */

    DeviceState *gpio_dev[7], *nvic;
    qemu_irq gpio_in[7][8];
    qemu_irq gpio_out[7][8];
    I2CBus *i2c;
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

    // ADC
    dev = sysbus_create_varargs(TYPE_TM4_ADC, adc_addr[0],
                                qdev_get_gpio_in(nvic, adc_irq[0][0]),
                                qdev_get_gpio_in(nvic, adc_irq[0][1]),
                                qdev_get_gpio_in(nvic, adc_irq[0][2]),
                                qdev_get_gpio_in(nvic, adc_irq[0][3]),
                                NULL);
    qemu_irq adc0 = qdev_get_gpio_in(dev, 0);
    dev = sysbus_create_varargs(TYPE_TM4_ADC, adc_addr[1],
                                qdev_get_gpio_in(nvic, adc_irq[1][0]),
                                qdev_get_gpio_in(nvic, adc_irq[1][1]),
                                qdev_get_gpio_in(nvic, adc_irq[1][2]),
                                qdev_get_gpio_in(nvic, adc_irq[1][3]),
                                NULL);
    qemu_irq adc1 = qdev_get_gpio_in(dev, 0);

    // Timers
    for (i = 0; i < 12; i++) {
        SysBusDevice *sbd;

        dev = qdev_new(TYPE_TM4_TIMER); // TODO
        sbd = SYS_BUS_DEVICE(dev);
        object_property_add_child(soc_container, "gptm[*]", OBJECT(dev));
        qdev_connect_clock_in(dev, "clk",
                                qdev_get_clock_out(ssys_dev, "SYSCLK"));
        sysbus_realize_and_unref(sbd, &error_fatal);
        int width = i / 6;
        int index = i % 6;
        sysbus_mmio_map(sbd, 0, timer_addr[width][index]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, timer_irq[width][index]));
        /* TODO: This is incorrect, but we get away with it because
            the ADC output is only ever pulsed.  */
        qdev_connect_gpio_out(dev, 0, adc1);
    }

    // Watchdogs TODO
    // if (board->dc1 & (1 << 3)) { /* watchdog present */
    //     dev = qdev_new(TYPE_LUMINARY_WATCHDOG);
    //     object_property_add_child(soc_container, "wdg", OBJECT(dev));
    //     qdev_connect_clock_in(dev, "WDOGCLK",
    //                           qdev_get_clock_out(ssys_dev, "SYSCLK"));

    //     sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    //     sysbus_mmio_map(SYS_BUS_DEVICE(dev),
    //                     0,
    //                     0x40000000u);
    //     sysbus_connect_irq(SYS_BUS_DEVICE(dev),
    //                        0,
    //                        qdev_get_gpio_in(nvic, 18));
    // }

    // GPIO
    for (i = 0; i < 6; i++) {
        gpio_dev[i] = sysbus_create_simple("pl061_luminary", gpio_addr[i],
                                            qdev_get_gpio_in(nvic,
                                                            gpio_irq[i]));
        for (j = 0; j < 8; j++) {
            gpio_in[i][j] = qdev_get_gpio_in(gpio_dev[i], j);
            gpio_out[i][j] = NULL;
        }
    }

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
    for (i = 0; i < 8; i++) {
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

    // Connect GPIO outs
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 8; j++) {
            if (gpio_out[i][j]) {
                qdev_connect_gpio_out(gpio_dev[i], j, gpio_out[i][j]);
            }
        }
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

    dc->vmsd = &vmstate_stellaris_sys;
    rc->phases.enter = tm4c123gh6pm_sys_reset_enter;
    rc->phases.hold = tm4c123gh6pm_sys_reset_hold;
    rc->phases.exit = tm4c123gh6pm_sys_reset_exit;
    device_class_set_props(dc, tm4c123gh6pm_sys_properties);
}

static const TypeInfo stellaris_sys_info = {
    .name = TYPE_TM4_SYS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ssys_state),
    .instance_init = stellaris_sys_instance_init,
    .class_init = stellaris_sys_class_init,
};

static void stellaris_register_types(void)
{
    i2c_register_types();
    adc_register_types();
    type_register_static(&stellaris_sys_info);
}
type_init(stellaris_register_types)
