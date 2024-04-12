#ifndef TM4_GPIO_H_
#define TM4_GPIO_H_

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define F_NONE          -1
#define F_U0Rx           0
#define F_CAN1Rx         1
#define F_U0Tx           2
#define F_CAN1Tx         3
#define F_SSI0Clk        4
#define F_SSI0Fss        5
#define F_SSI0Rx         6
#define F_SSI0Tx         7
#define F_I2C1SCL        8
#define F_M1PWM2         9
#define F_I2C1SDA        10
#define F_M1PWM3         11
#define F_USB0ID         12
#define F_U1Rx           13
#define F_T2CCP0         14
#define F_USB0VBUS       15
#define F_U1Tx           16
#define F_T2CCP1         17
#define F_I2C0SCL        18
#define F_T3CCP0         19
#define F_I2C0SDA        20
#define F_T3CCP1         21
#define F_AIN10          22
#define F_SSI2Clk        23
#define F_M0PWM2         24
#define F_T1CCP0         25
#define F_CAN0Rx         26
#define F_AIN11          27
#define F_SSI2Fss        28
#define F_M0PWM3         29
#define F_T1CCP1         30
#define F_CAN0Tx         31
#define F_SSI2Rx         32
#define F_M0PWM0         33
#define F_T0CCP0         34
#define F_SSI2Tx         35
#define F_M0PWM1         36
#define F_T0CCP1         37
#define F_TCK_SWCLK      38
#define F_T4CCP0         39
#define F_TMS_SWDIO      40
#define F_T4CCP1         41
#define F_TDI            42
#define F_T5CCP0         43
#define F_TDO_SWO        44
#define F_T5CCP1         45
#define F_C1_M           46
#define F_U4Rx           47
#define F_M0PWM6         48
#define F_IDX1           49
#define F_WT0CCP0        50
#define F_U1RTS          51
#define F_C1_P           52
#define F_U4Tx           53
#define F_M0PWM7         54
#define F_PhA1           55
#define F_WT0CCP1        56
#define F_U1CTS          57
#define F_C0_P           58
#define F_U3Rx           59
#define F_PhB1           60
#define F_WT1CCP0        61
#define F_USB0EPEN       62
#define F_C0_M           63
#define F_U3Tx           64
#define F_WT1CCP1        65
#define F_USB0PFLT       66
#define F_AIN7           67
#define F_SSI3Clk        68
#define F_SSI1Clk        69
#define F_I2C3SCL        70
#define F_M1PWM0         71
#define F_WT2CCP0        72
#define F_AIN6           73
#define F_SSI3Fss        74
#define F_SSI1Fss        75
#define F_I2C3SDA        76
#define F_M1PWM1         77
#define F_WT2CCP1        78
#define F_AIN5           79
#define F_SSI3Rx         80
#define F_SSI1Rx         81
#define F_M0FAULT0       82
#define F_WT3CCP0        83
#define F_AIN4           84
#define F_SSI3Tx         85
#define F_SSI1Tx         86
#define F_IDX0           87
#define F_WT3CCP1        88
#define F_USB0DM         89
#define F_U6Rx           90
#define F_WT4CCP0        91
#define F_USB0DP         92
#define F_U6Tx           93
#define F_WT4CCP1        94
#define F_U2Rx           95
#define F_PhA0           96
#define F_WT5CCP0        97
#define F_U2Tx           98
#define F_PhB0           99
#define F_WT5CCP1        100
#define F_NMI            101
#define F_AIN3           102
#define F_U7Rx           103
#define F_AIN2           104
#define F_U7Tx           105
#define F_AIN1           106
#define F_AIN0           107
#define F_AIN9           108
#define F_U5Rx           109
#define F_I2C2SCL        110
#define F_M0PWM4         111
#define F_AIN8           112
#define F_U5Tx           113
#define F_I2C2SDA        114
#define F_M0PWM5         115
#define F_M1PWM4         116
#define F_C0o            117
#define F_M1PWM5         118
#define F_C1o            119
#define F_TRD1           120
#define F_SSIClk         121
#define F_M1PWM6         122
#define F_TRD0           123
#define F_M1PWM7         124
#define F_TRCLK          125
#define F_M1FAULT0       126

#define N_ALT_F 127
#define N_BITS 8

extern const int8_t GPIO_ALTERNATE_FUNCTIONS[6][N_BITS][16];

struct GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t data;
    uint32_t dir;
    uint32_t is;
    uint32_t ibe;
    uint32_t iev;
    uint32_t im;
    uint32_t ris;
    uint32_t afsel;
    uint32_t dr2r;
    uint32_t dr4r;
    uint32_t dr8r;
    uint32_t odr;
    uint32_t pur;
    uint32_t pdr;
    uint32_t slr;
    uint32_t den;
    uint32_t lock;
    uint32_t cr;
    uint32_t amsel;
    uint32_t pctl;
    uint32_t adcctl;
    uint32_t dmactl;
    const unsigned char *id;

    uint32_t old_out_data;
    uint32_t old_in_data;

    uint16_t levels[N_BITS];

    qemu_irq nvic_irq;
    qemu_irq out[N_BITS];
    qemu_irq alt_out[N_ALT_F];

    uint8_t port;
};

#define TYPE_TM4_GPIO "tm4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(GPIOState, TM4_GPIO)

#endif
