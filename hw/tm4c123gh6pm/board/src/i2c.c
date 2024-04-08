// #include "qemu/osdep.h"
// #include "qapi/error.h"
// #include "hw/core/split-irq.h"
// #include "hw/sysbus.h"
// #include "hw/sd/sd.h"
// #include "hw/ssi/ssi.h"
// #include "hw/arm/boot.h"
// #include "qemu/timer.h"
// #include "hw/i2c/i2c.h"
// #include "net/net.h"
// #include "hw/boards.h"
// #include "qemu/log.h"
// #include "exec/address-spaces.h"
// #include "sysemu/sysemu.h"
// #include "hw/arm/armv7m.h"
// #include "hw/char/pl011.h"
// #include "hw/input/stellaris_gamepad.h"
// #include "hw/irq.h"
// #include "hw/watchdog/cmsdk-apb-watchdog.h"
// #include "migration/vmstate.h"
// #include "hw/misc/unimp.h"
// #include "hw/timer/stellaris-gptm.h"
// #include "hw/qdev-clock.h"
// #include "qom/object.h"
// #include "qapi/qmp/qlist.h"
// #include "ui/input.h"
// #include "hw/tm4c123gh6pm/board/include/i2c.h"

// /*
//  * I2C controller.
//  * ??? For now we only implement the master interface.
//  */

// #define TYPE_TM4_I2C "tm4-i2c"
// OBJECT_DECLARE_SIMPLE_TYPE(stellaris_i2c_state, TM4_I2C)

// struct stellaris_i2c_state {
//     SysBusDevice parent_obj;

//     I2CBus *bus;
//     qemu_irq irq;
//     MemoryRegion iomem;
//     uint32_t msa;
//     uint32_t mcs;
//     uint32_t mdr;
//     uint32_t mtpr;
//     uint32_t mimr;
//     uint32_t mris;
//     uint32_t mcr;
// };

// #define STELLARIS_I2C_MCS_BUSY    0x01
// #define STELLARIS_I2C_MCS_ERROR   0x02
// #define STELLARIS_I2C_MCS_ADRACK  0x04
// #define STELLARIS_I2C_MCS_DATACK  0x08
// #define STELLARIS_I2C_MCS_ARBLST  0x10
// #define STELLARIS_I2C_MCS_IDLE    0x20
// #define STELLARIS_I2C_MCS_BUSBSY  0x40

// static uint64_t stellaris_i2c_read(void *opaque, hwaddr offset,
//                                    unsigned size)
// {
//     stellaris_i2c_state *s = (stellaris_i2c_state *)opaque;

//     switch (offset) {
//     case 0x00: /* MSA */
//         return s->msa;
//     case 0x04: /* MCS */
//         /* We don't emulate timing, so the controller is never busy.  */
//         return s->mcs | STELLARIS_I2C_MCS_IDLE;
//     case 0x08: /* MDR */
//         return s->mdr;
//     case 0x0c: /* MTPR */
//         return s->mtpr;
//     case 0x10: /* MIMR */
//         return s->mimr;
//     case 0x14: /* MRIS */
//         return s->mris;
//     case 0x18: /* MMIS */
//         return s->mris & s->mimr;
//     case 0x20: /* MCR */
//         return s->mcr;
//     default:
//         qemu_log_mask(LOG_GUEST_ERROR,
//                       "stellaris_i2c: read at bad offset 0x%x\n", (int)offset);
//         return 0;
//     }
// }

// static void stellaris_i2c_update(stellaris_i2c_state *s)
// {
//     int level;

//     level = (s->mris & s->mimr) != 0;
//     qemu_set_irq(s->irq, level);
// }

// static void stellaris_i2c_write(void *opaque, hwaddr offset,
//                                 uint64_t value, unsigned size)
// {
//     stellaris_i2c_state *s = (stellaris_i2c_state *)opaque;

//     switch (offset) {
//     case 0x00: /* MSA */
//         s->msa = value & 0xff;
//         break;
//     case 0x04: /* MCS */
//         if ((s->mcr & 0x10) == 0) {
//             /* Disabled.  Do nothing.  */
//             break;
//         }
//         /* Grab the bus if this is starting a transfer.  */
//         if ((value & 2) && (s->mcs & STELLARIS_I2C_MCS_BUSBSY) == 0) {
//             if (i2c_start_transfer(s->bus, s->msa >> 1, s->msa & 1)) {
//                 s->mcs |= STELLARIS_I2C_MCS_ARBLST;
//             } else {
//                 s->mcs &= ~STELLARIS_I2C_MCS_ARBLST;
//                 s->mcs |= STELLARIS_I2C_MCS_BUSBSY;
//             }
//         }
//         /* If we don't have the bus then indicate an error.  */
//         if (!i2c_bus_busy(s->bus)
//                 || (s->mcs & STELLARIS_I2C_MCS_BUSBSY) == 0) {
//             s->mcs |= STELLARIS_I2C_MCS_ERROR;
//             break;
//         }
//         s->mcs &= ~STELLARIS_I2C_MCS_ERROR;
//         if (value & 1) {
//             /* Transfer a byte.  */
//             /* TODO: Handle errors.  */
//             if (s->msa & 1) {
//                 /* Recv */
//                 s->mdr = i2c_recv(s->bus);
//             } else {
//                 /* Send */
//                 i2c_send(s->bus, s->mdr);
//             }
//             /* Raise an interrupt.  */
//             s->mris |= 1;
//         }
//         if (value & 4) {
//             /* Finish transfer.  */
//             i2c_end_transfer(s->bus);
//             s->mcs &= ~STELLARIS_I2C_MCS_BUSBSY;
//         }
//         break;
//     case 0x08: /* MDR */
//         s->mdr = value & 0xff;
//         break;
//     case 0x0c: /* MTPR */
//         s->mtpr = value & 0xff;
//         break;
//     case 0x10: /* MIMR */
//         s->mimr = 1;
//         break;
//     case 0x1c: /* MICR */
//         s->mris &= ~value;
//         break;
//     case 0x20: /* MCR */
//         if (value & 1) {
//             qemu_log_mask(LOG_UNIMP,
//                           "stellaris_i2c: Loopback not implemented\n");
//         }
//         if (value & 0x20) {
//             qemu_log_mask(LOG_UNIMP,
//                           "stellaris_i2c: Slave mode not implemented\n");
//         }
//         s->mcr = value & 0x31;
//         break;
//     default:
//         qemu_log_mask(LOG_GUEST_ERROR,
//                       "stellaris_i2c: write at bad offset 0x%x\n", (int)offset);
//     }
//     stellaris_i2c_update(s);
// }

// static void stellaris_i2c_reset_enter(Object *obj, ResetType type)
// {
//     stellaris_i2c_state *s = TM4_I2C(obj);

//     if (s->mcs & STELLARIS_I2C_MCS_BUSBSY)
//         i2c_end_transfer(s->bus);
// }

// static void stellaris_i2c_reset_hold(Object *obj)
// {
//     stellaris_i2c_state *s = TM4_I2C(obj);

//     s->msa = 0;
//     s->mcs = 0;
//     s->mdr = 0;
//     s->mtpr = 1;
//     s->mimr = 0;
//     s->mris = 0;
//     s->mcr = 0;
// }

// static void stellaris_i2c_reset_exit(Object *obj)
// {
//     stellaris_i2c_state *s = TM4_I2C(obj);

//     stellaris_i2c_update(s);
// }

// static const MemoryRegionOps stellaris_i2c_ops = {
//     .read = stellaris_i2c_read,
//     .write = stellaris_i2c_write,
//     .endianness = DEVICE_NATIVE_ENDIAN,
// };

// static const VMStateDescription vmstate_stellaris_i2c = {
//     .name = "stellaris_i2c",
//     .version_id = 1,
//     .minimum_version_id = 1,
//     .fields = (const VMStateField[]) {
//         VMSTATE_UINT32(msa, stellaris_i2c_state),
//         VMSTATE_UINT32(mcs, stellaris_i2c_state),
//         VMSTATE_UINT32(mdr, stellaris_i2c_state),
//         VMSTATE_UINT32(mtpr, stellaris_i2c_state),
//         VMSTATE_UINT32(mimr, stellaris_i2c_state),
//         VMSTATE_UINT32(mris, stellaris_i2c_state),
//         VMSTATE_UINT32(mcr, stellaris_i2c_state),
//         VMSTATE_END_OF_LIST()
//     }
// };

// static void stellaris_i2c_init(Object *obj)
// {
//     DeviceState *dev = DEVICE(obj);
//     stellaris_i2c_state *s = TM4_I2C(obj);
//     SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
//     I2CBus *bus;

//     sysbus_init_irq(sbd, &s->irq);
//     bus = i2c_init_bus(dev, "i2c");
//     s->bus = bus;

//     memory_region_init_io(&s->iomem, obj, &stellaris_i2c_ops, s,
//                           "i2c", 0x1000);
//     sysbus_init_mmio(sbd, &s->iomem);
// }


// // TODO I2C
// static void tm4c123gh6pm_i2c_class_init(ObjectClass *klass, void *data)
// {
//     DeviceClass *dc = DEVICE_CLASS(klass);
//     ResettableClass *rc = RESETTABLE_CLASS(klass);

//     rc->phases.enter = tm4c123gh6pm_i2c_reset_enter;
//     rc->phases.hold = tm4c123gh6pm_i2c_reset_hold;
//     rc->phases.exit = tm4c123gh6pm_i2c_reset_exit;
//     dc->vmsd = &vmstate_tm4c123gh6pm_i2c;
// }

// static const TypeInfo stellaris_i2c_info = {
//     .name          = TYPE_TM4_I2C,
//     .parent        = TYPE_SYS_BUS_DEVICE,
//     .instance_size = sizeof(tm4c123gh6pm_i2c_state),
//     .instance_init = tm4c123gh6pm_i2c_init,
//     .class_init    = tm4c123gh6pm_i2c_class_init,
// };

// // attribute used to prevent compiler warnings, static function
// // used in tm4c123gh6pm.c, not here
// void i2c_register_types(void)
// {
//     type_register_static(&tm4c123gh6pm_i2c_info);
// }
