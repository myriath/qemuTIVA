#ifndef TM4_H_
#define TM4_H_

#include "joiner.h"
#include "gpio.h"
#include "timer_16_32.h"
#include "adc.h"
#include "uart.h"
#include "test_analog.h"
// #include "i2c.h"

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

struct tiva_devices {
    DeviceState *timer[COUNT_TIMERS];
    DeviceState *gpio[N_GPIOS];
    DeviceState *uart[COUNT_UART];
    DeviceState *adc[COUNT_ADC];

    Object *soc;
    DeviceState *nvic;
    DeviceState *ssys_dev;

    qemu_irq gpio_in[N_GPIOS][N_GPIO_BITS][N_PCTL_OPTS];
    qemu_irq gpio_out[N_GPIOS][N_GPIO_BITS][N_PCTL_OPTS];
};

struct ssys_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t pborctl;
    uint32_t ris;
    uint32_t imc;
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

/* System controller.  */
#define TYPE_TM4_SYS "tm4-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ssys_state, TM4_SYS)

#endif
