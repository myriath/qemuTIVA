/*
 * Luminary Micro Stellaris General Purpose Timer Module
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#ifndef TM4_TIMER_H_
#define TM4_TIMER_H_

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "hw/qdev-clock.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/clock.h"

#define GPTM_A              0
#define GPTM_B              1

#define GPTM_GPIO_CCP0      0
#define GPTM_GPIO_CCP1      1
#define GPTM_GPIO_ADC       2

#define GPTM_CFG_LONG       0
#define GPTM_CFG_RTC        1
#define GPTM_CFG_SHORT      4

#define GPTM_WIDTH_16_32    1
#define GPTM_WIDTH_32_64    1

#define GPTM_LEN_16         0
#define GPTM_LEN_32         1
#define GPTM_LEN_64         2

#define GPTM_MODE_ONE_SHOT  1
#define GPTM_MODE_PERIODIC  2
#define GPTM_MODE_CAPTURE   3

#define GPTM_MODE_MR_M      0x0003
#define GPTM_MODE_CMR       0x0004
#define GPTM_MODE_AMS       0x0008
#define GPTM_MODE_CDIR      0x0010
#define GPTM_MODE_MIE       0x0020
#define GPTM_MODE_WOT       0x0040
#define GPTM_MODE_SNAPS     0x0080
#define GPTM_MODE_ILD       0x0100
#define GPTM_MODE_PWMIE     0x0200
#define GPTM_MODE_MRSU      0x0400
#define GPTM_MODE_PLO       0x0800

#define GPTM_INT_TATO_M     0x0001
#define GPTM_INT_CAM_M      0x0002
#define GPTM_INT_CAE_M      0x0004
#define GPTM_INT_RTC_M      0x0008
#define GPTM_INT_TAM_M      0x0010
#define GPTM_INT_TBTO_M     0x0100
#define GPTM_INT_CBM_M      0x0200
#define GPTM_INT_CBE_M      0x0400
#define GPTM_INT_TBM_M      0x0800
#define GPTM_INT_WUE_M      0x10000

#define GPTM_CTL_TAEN       0x0001
#define GPTM_CTL_TASTALL    0x0002
#define GPTM_CTL_TAEVENT    0x000c
#define GPTM_CTL_RTCEN      0x0010
#define GPTM_CTL_TAOTE      0x0020
#define GPTM_CTL_TAPWML     0x0040
#define GPTM_CTL_TBEN       0x0100
#define GPTM_CTL_TBSTALL    0x0200
#define GPTM_CTL_TBEVENT    0x0c00
#define GPTM_CTL_TBOTE      0x2000
#define GPTM_CTL_TBPWML     0x4000

#define GPTM_INVALID        -1
#define GPTM_ONE_SHOT_IND   0
#define GPTM_ONE_SHOT_CON   1
#define GPTM_PERIODIC_IND   2
#define GPTM_PERIODIC_CON   3
#define GPTM_RTC            4
#define GPTM_EDGE_COUNT     5
#define GPTM_EDGE_TIME      6
#define GPTM_PWM_PR         7
#define GPTM_PWM_OS         8


/*
 * QEMU interface:
 *  + sysbus MMIO region 0: register bank
 *  + sysbus IRQ 0: timer interrupt
 *  + unnamed GPIO output 0: trigger output for the ADC
 *  + Clock input "clk": the 32-bit countdown timer runs at this speed
 */
struct GPTMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    // Config
    uint32_t cfg;
    uint32_t mode[2];
    uint32_t ctl;
    uint32_t sync;
    uint32_t imr;
    uint32_t ris;
    uint32_t load[2];
    uint64_t load_64;
    uint32_t match[2];
    uint64_t match_64;
    uint32_t prescale[2];
    uint32_t prescale_match[2];
    uint32_t value[2];

    // Properties
    uint32_t rtcpd;
    uint32_t prescale_snapshot[2];
    uint32_t prescale_value[2];
    uint32_t pp;

    uint8_t width;

    // uint32_t config;
    // uint32_t mode[2];
    // uint32_t control;
    // uint32_t state;
    // uint32_t mask;
    // uint32_t load[2];
    // uint32_t match[2];
    // uint32_t prescale[2];
    // uint32_t match_prescale[2];
    // uint32_t rtc;
    // Used for Counter mode
    int64_t count[2];
    // Used for Edge-time mode
    int64_t capture[2];
    // Timer stuff
    int64_t ns_per_cycle;
    int64_t start[2];
    int64_t tick[2];
    // Stored timer value when timer is disabled;
    int64_t time[2];
    struct GPTMState *opaque[2];
    QEMUTimer *timer[2];
    bool matched[2];
    /* The timers have an alternate output used to trigger the ADC.  */
    qemu_irq trigger_adc;
    qemu_irq ccp_gpio[2];
    qemu_irq nvic_irq;
    Clock *clk;
};

#define TYPE_TM4_TIMER "tm4-timer"
OBJECT_DECLARE_SIMPLE_TYPE(GPTMState, TM4_TIMER)

#endif
