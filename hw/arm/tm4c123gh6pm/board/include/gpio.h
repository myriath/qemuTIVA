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
#include "hw/qdev-clock.h"

#define F_NONE          -1
#define F_GENERAL       0
#define F_ANALOG        0
#define F_UART          1
#define F_UART_ALT      2
#define F_TIMER         7

#define F_CAN1Rx_A      8
#define F_CAN1Tx_A      8
#define F_SSI0Clk_A     2
#define F_SSI0Fss_A     2
#define F_SSI0Rx_A      2
#define F_SSI0Tx_A      2
#define F_I2C1SCL_A     3
#define F_M1PWM2_A      5
#define F_I2C1SDA_A     3
#define F_M1PWM3_A      5
#define F_I2C0SCL_B     3
#define F_I2C0SDA_B     3
#define F_SSI2Clk_B     2
#define F_M0PWM2_B      4
#define F_CAN0Rx_B      8
#define F_SSI2Fss_B     2
#define F_M0PWM3_B      4
#define F_CAN0Tx_B      8
#define F_SSI2Rx_B      2
#define F_M0PWM0_B      4
#define F_SSI2Tx_B      2
#define F_M0PWM1_B      4
#define F_TCK_SWCLK_C   1
#define F_TMS_SWDIO_C   1
#define F_TDI_C         1
#define F_TDO_SWO_C     1
#define F_M0PWM6_C      4
#define F_IDX1_C        6
#define F_WT0CCP0_C     7
#define F_U1RTS_C       8
#define F_M0PWM7_C      4
#define F_PhA1_C        6
#define F_WT0CCP1_C     7
#define F_U1CTS_C       8
#define F_PhB1_C        6
#define F_WT1CCP0_C     7
#define F_USB0EPEN_C    8
#define F_WT1CCP1_C     7
#define F_USB0PFLT_C    8
#define F_SSI3Clk_D     1
#define F_SSI1Clk_D     2
#define F_I2C3SCL_D     3
#define F_M0PWM6_D      4
#define F_M1PWM0_D      5
#define F_WT2CCP0_D     7
#define F_SSI3Fss_D     1
#define F_SSI1Fss_D     2
#define F_I2C3SDA_D     3
#define F_M0PWM7_D      4
#define F_M1PWM1_D      5
#define F_WT2CCP1_D     7
#define F_SSI3Rx_D      1
#define F_SSI1Rx_D      2
#define F_M0FAULT0_D_0  4
#define F_WT3CCP0_D     7
#define F_USB0EPEN_D    8
#define F_SSI3Tx_D      1
#define F_SSI1Tx_D      2
#define F_IDX0_D        6
#define F_WT3CCP1_D     7
#define F_USB0PFLT_D    8
#define F_WT4CCP0_D     7
#define F_WT4CCP1_D     7
#define F_M0FAULT0_D_1  4
#define F_PhA0_D        6
#define F_WT5CCP0_D     7
#define F_PhB0_D        6
#define F_WT5CCP1_D     7
#define F_NMI_D         8
#define F_I2C2SCL_E     3
#define F_M0PWM4_E      4
#define F_M1PWM2_E      5
#define F_CAN0Rx_E      8
#define F_I2C2SDA_E     3
#define F_M0PWM5_E      4
#define F_M1PWM3_E      5
#define F_CAN0Tx_E      8
#define F_U1RTS_F       1
#define F_SSI1Rx_F      2
#define F_CAN0Rx_F      3
#define F_M1PWM4_F      5
#define F_PhA0_F        6
#define F_NMI_F         8
#define F_C0o_F         9
#define F_U1CTS_F       1
#define F_SSI1Tx_F      2
#define F_M1PWM5_F      5
#define F_PhB0_F        6
#define F_C1o_F         9
#define F_TRD1_F        14
#define F_SSI1Clk_F     2
#define F_M0FAULT0_F    4
#define F_M1PWM6_F      5
#define F_TRD0_F        14
#define F_SSI1Fss_F     2
#define F_CAN0Tx_F      3
#define F_M1PWM7_F      5
#define F_TRCLK_F       14
#define F_M1FAULT0_F    5
#define F_IDX0_F        6
#define F_USB0EPEN_F    8

#define N_GPIOS         6
#define N_GPIO_BITS     8
#define N_PCTL_OPTS     16
#define N_GPIO_TABLE    (N_GPIO_BITS * N_PCTL_OPTS)

#define GPIO_ALT_F_PINS "gpio-alt-functions"

extern const int8_t GPIO_ALTERNATE_FUNCTIONS[N_GPIOS][N_GPIO_BITS][N_PCTL_OPTS];
extern const char *GPIO_NAMED_PINS[N_GPIO_BITS];

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

    uint16_t levels[N_GPIO_BITS];

    // IRQ for NVIC
    qemu_irq nvic_irq;
    // Outputs go from GPIO controller to TM4 board
    qemu_irq outputs[N_GPIO_BITS][N_PCTL_OPTS];
    // Inputs go from TM4 board to GPIO controller
    qemu_irq inputs[N_GPIO_TABLE];
    uint8_t port;
    bool debug;

    Clock *clk;
    bool clock_active;
};

#define TYPE_TM4_GPIO "tm4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(GPIOState, TM4_GPIO)

DeviceState *gpio_create(bool debug, hwaddr addr, qemu_irq nvic_irq, uint8_t port, Clock *clk);

#endif
