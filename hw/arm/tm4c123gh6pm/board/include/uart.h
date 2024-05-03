#ifndef TM4_UART_H_
#define TM4_UART_H_

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define UART_FLAG_TXFE  0x0080
#define UART_FLAG_RXFF  0x0040
#define UART_FLAG_TXFF  0x0020
#define UART_FLAG_RXFE  0x0010
#define UART_FLAG_BUSY  0x0008
#define UART_FLAG_CTS   0x0001

#define UART_INT_9BIT   0x1000
#define UART_INT_OE     0x0400
#define UART_INT_BE     0x0200
#define UART_INT_PE     0x0100
#define UART_INT_FE     0x0080
#define UART_INT_RT     0x0040
#define UART_INT_TX     0x0020
#define UART_INT_RX     0x0010
#define UART_INT_CTS    0x0002

#define UART_LCRH_BRK   0x0001
#define UART_LCRH_PEN   0x0002
#define UART_LCRH_EPS   0x0004
#define UART_LCRH_STP2  0x0008
#define UART_LCRH_FEN   0x0010
#define UART_LCRH_WLEN  0x0020
#define UART_LCRH_SPS   0x0080

#define UART_DR_OE      0x0800
#define UART_DR_BE      0x0400
#define UART_DR_PE      0x0200
#define UART_DR_FE      0x0100

#define UART_CTL_CTSEN  0x8000
#define UART_CTL_RTSEN  0x4000
#define UART_CTL_RTS    0x0800
#define UART_CTL_RXE    0x0200
#define UART_CTL_TXE    0x0100
#define UART_CTL_LBE    0x0080
#define UART_CTL_HSE    0x0020
#define UART_CTL_EOT    0x0010
#define UART_CTL_SMART  0x0008
#define UART_CTL_SIRLP  0x0004
#define UART_CTL_SIREN  0x0002
#define UART_CTL_UARTEN 0x0001

#define UART_FIFO_DEPTH 16

#define COUNT_UART      8

struct UARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t dr;
    uint32_t rsrecr;
    uint32_t fr;
    uint32_t ilpr;
    uint32_t ibrd;
    uint32_t fbrd;
    uint32_t lcrh;
    uint32_t ctl;
    uint32_t ifls;
    uint32_t im;
    uint32_t ris;
    uint32_t dmactl;
    uint32_t addr;
    uint32_t amask;
    uint32_t pp;
    uint32_t cc;

    bool migrate_clk;

    int frame_size;

    int read_pos;
    int read_count;
    int read_trigger;
    int read_bit;
    uint32_t receive_frame;
    bool rx_state;
    bool reading;
    uint32_t read_fifo[UART_FIFO_DEPTH];

    int write_pos;
    int write_count;
    int write_trigger;
    int write_bit;
    uint32_t write_fifo[UART_FIFO_DEPTH];

    // Output IRQs
    qemu_irq nvic_irq;
    qemu_irq tx_gpio;
    // Input IRQ
    qemu_irq rx_gpio;

    // Stuff for timing uart send / receive
    int64_t write_tick;
    int64_t read_tick;
    QEMUTimer *write_timer;
    QEMUTimer *read_timer;
    Clock *clk;

    const uint8_t *id;
    const uint8_t uart;
    bool debug;
};

#define TYPE_TM4_UART "tm4-uart"
OBJECT_DECLARE_SIMPLE_TYPE(UARTState, TM4_UART)

DeviceState *uart_create(bool debug, hwaddr addr, uint8_t uart, qemu_irq nvic_irq, qemu_irq tx_gpio, qemu_irq rts, qemu_irq cts, Clock *clk);

#endif
