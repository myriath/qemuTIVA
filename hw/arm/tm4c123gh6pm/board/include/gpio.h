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

#define F_NONE          0
#define F_AIN           1

#define F_U0Rx_A        1
#define F_CAN1Rx_A      2
#define F_U0Tx_A        1
#define F_CAN1Tx_A      2
#define F_SSI0Clk_A     1
#define F_SSI0Fss_A     1
#define F_SSI0Rx_A      1
#define F_SSI0Tx_A      1
#define F_I2C1SCL_A     1
#define F_M1PWM2_A      2
#define F_I2C1SDA_A     1
#define F_M1PWM3_A      2
#define F_USB0ID_B      1
#define F_U1Rx_B        2
#define F_T2CCP0_B      3
#define F_USB0VBUS_B    1
#define F_U1Tx_B        2
#define F_T2CCP1_B      3
#define F_I2C0SCL_B     1
#define F_T3CCP0_B      2
#define F_I2C0SDA_B     1
#define F_T3CCP1_B      2
#define F_AIN10_B       1
#define F_SSI2Clk_B     2
#define F_M0PWM2_B      3
#define F_T1CCP0_B      4
#define F_CAN0Rx_B      5
#define F_AIN11_B       1
#define F_SSI2Fss_B     2
#define F_M0PWM3_B      3
#define F_T1CCP1_B      4
#define F_CAN0Tx_B      5
#define F_SSI2Rx_B      1
#define F_M0PWM0_B      2
#define F_T0CCP0_B      3
#define F_SSI2Tx_B      1
#define F_M0PWM1_B      2
#define F_T0CCP1_B      3
#define F_TCK_SWCLK_C   1
#define F_T4CCP0_C      2
#define F_TMS_SWDIO_C   1
#define F_T4CCP1_C      2
#define F_TDI_C         1
#define F_T5CCP0_C      2
#define F_TDO_SWO_C     1
#define F_T5CCP1_C      2
#define F_C1_M_C        1
#define F_U4Rx_C        2
#define F_U1Rx_C        3
#define F_M0PWM6_C      4
#define F_IDX1_C        5
#define F_WT0CCP0_C     6
#define F_U1RTS_C       7
#define F_C1_P_C        1
#define F_U4Tx_C        2
#define F_U1Tx_C        3
#define F_M0PWM7_C      4
#define F_PhA1_C        5
#define F_WT0CCP1_C     6
#define F_U1CTS_C       7
#define F_C0_P_C        1
#define F_U3Rx_C        2
#define F_PhB1_C        3
#define F_WT1CCP0_C     4
#define F_USB0EPEN_C    5
#define F_C0_M_C        1
#define F_U3Tx_C        2
#define F_WT1CCP1_C     3
#define F_USB0PFLT_C    4
#define F_AIN7_D        1
#define F_SSI3Clk_D     2
#define F_SSI1Clk_D     3
#define F_I2C3SCL_D     4
#define F_M0PWM6_D      5
#define F_M1PWM0_D      6
#define F_WT2CCP0_D     7
#define F_AIN6_D        1
#define F_SSI3Fss_D     2
#define F_SSI1Fss_D     3
#define F_I2C3SDA_D     4
#define F_M0PWM7_D      5
#define F_M1PWM1_D      6
#define F_WT2CCP1_D     7
#define F_AIN5_D        1
#define F_SSI3Rx_D      2
#define F_SSI1Rx_D      3
#define F_M0FAULT0_D_0  4
#define F_WT3CCP0_D     5
#define F_USB0EPEN_D    6
#define F_AIN4_D        1
#define F_SSI3Tx_D      2
#define F_SSI1Tx_D      3
#define F_IDX0_D        4
#define F_WT3CCP1_D     5
#define F_USB0PFLT_D    6
#define F_USB0DM_D      1
#define F_U6Rx_D        2
#define F_WT4CCP0_D     3
#define F_USB0DP_D      1
#define F_U6Tx_D        2
#define F_WT4CCP1_D     3
#define F_U2Rx_D        1
#define F_M0FAULT0_D_1  2
#define F_PhA0_D        3
#define F_WT5CCP0_D     4
#define F_U2Tx_D        1
#define F_PhB0_D        2
#define F_WT5CCP1_D     3
#define F_NMI_D         4
#define F_AIN3_E        1
#define F_U7Rx_E        2
#define F_AIN2_E        1
#define F_U7Tx_E        2
#define F_AIN1_E        1
#define F_AIN0_E        1
#define F_AIN9_E        1
#define F_U5Rx_E        2
#define F_I2C2SCL_E     3
#define F_M0PWM4_E      4
#define F_M1PWM2_E      5
#define F_CAN0Rx_E      6
#define F_AIN8_E        1
#define F_U5Tx_E        2
#define F_I2C2SDA_E     3
#define F_M0PWM5_E      4
#define F_M1PWM3_E      5
#define F_CAN0Tx_E      6
#define F_U1RTS_F       1
#define F_SSI1Rx_F      2
#define F_CAN0Rx_F      3
#define F_M1PWM4_F      4
#define F_PhA0_F        5
#define F_T0CCP0_F      6
#define F_NMI_F         7
#define F_C0o_F         8
#define F_U1CTS_F       1
#define F_SSI1Tx_F      2
#define F_M1PWM5_F      3
#define F_PhB0_F        4
#define F_T0CCP1_F      5
#define F_C1o_F         6
#define F_TRD1_F        7
#define F_SSI1Clk_F     1
#define F_M0FAULT0_F    2
#define F_M1PWM6_F      3
#define F_T1CCP0_F      4
#define F_TRD0_F        5
#define F_SSI1Fss_F     1
#define F_CAN0Tx_F      2
#define F_M1PWM7_F      3
#define F_T1CCP1_F      4
#define F_TRCLK_F       5
#define F_M1FAULT0_F    1
#define F_IDX0_F        2
#define F_T2CCP0_F      3
#define F_USB0EPEN_F    4

#define N_ALTS_PER_LINE 9
#define N_BITS 8

#define GPIO_ALT_F_PINS "gpio-alt-functions"

#define ALT_F_TO_IRQ(ALT_F, PIN) (N_ALTS_PER_LINE * PIN + ALT_F + N_BITS)

extern const int8_t GPIO_ALTERNATE_FUNCTIONS[6][N_BITS][16];
extern const char *GPIO_NAMED_OUTS[N_BITS];

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
    qemu_irq outputs[N_BITS][N_ALTS_PER_LINE];

    uint8_t port;
};

#define TYPE_TM4_GPIO "tm4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(GPIOState, TM4_GPIO)

DeviceState *gpio_create(hwaddr addr, qemu_irq nvic_irq, uint8_t port);

#endif
