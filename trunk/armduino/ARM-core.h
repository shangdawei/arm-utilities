#ifndef _ARM_CORE_H
#define _ARM_CORE_H
/* ARM-core.h: Cortex M3 core definitions. */
/*
  $Revision: 1.0 $ $Date: 2011/1/22 00:00:21 $
 Hardware definition settings.
 This file provides symbolic names and macros for the ARM Cortex M3
 common core registers.

 The typical header file approach of doing this is a hierarchy of macros,
 which must understood as a whole and traced through to get the final
 expansion.

 My viewpoint is that understanding the address decoding logic is of no
 value when you are programming the devices.  You want
 immediately-understood definitions that can be used as a index.  For
 other information you can have the datasheet in front of you.

 For the same reason there is no reason to re-document the details of
 a register or configuration bit.  The name only needs to evoke a memory
 of the function, not necessarily allow a reimplementation of the chip
 just from reading the source.
*/

typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;

#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define _MMIO_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
#define _MMIO_DWORD(mem_addr) (*(volatile uint32_t *)(mem_addr))

/* Bitbanding is a feature of the ARM core.  We implement the ARM-recommend
 * interface with uses the bit number within the word, as well as one that
 * calculates the bit number from a single-bit-set mask.
 */
#define BITBAND_SRAM_REF 0x20000000
#define BITBAND_SRAM_BASE 0x22000000
#define BITBAND_SRAM(addr,bitnum) \
	((BITBAND_SRAM_BASE + ((addr)-BITBAND_SRAM_REF)*32 + ((bitnum)*4)))
#define BITBAND_PERI_REF 0x40000000
#define BITBAND_PERI_BASE 0x42000000
#define BITBAND_PERI(addr,bitnum) \
	((BITBAND_PERI_BASE + ((addr)-BITBAND_PERI_REF)*32 + ((bitnum)*4)))

#define BITBAND_PERIPH(addr, mask) 	BITBAND_PERI(addr, MASK_TO_BITNUM(mask))

/* Convert a mask with a single bit set to the bit index.
 * There are clever ways to do this, but we make it explicit for clarity.
 */
#define MASK16_TO_BITNUM(mask) \
	((mask) == 0x01 ? 0 : (mask) == 0x02 ? 1 : (mask) == 0x04 ? 2 :		\
	 (mask)==0x008 ?  3 : (mask) == 0x10 ? 4 : (mask) == 0x20 ? 5 :		\
	 (mask)==0x040 ?  6 : (mask) == 0x80 ? 7 :							\
	 (mask)==0x100 ?  8 : (mask) == 0x200 ? 9 : (mask)==0x400 ? 10		\
	 (mask)==0x800 ? 11: (mask) == 0x1000 ? 12 : (mask)==0x2000 ? 13	\
	 (mask)==0x4000 ? 14: (mask) == 0x8000 ? 15 :						\
	 "Multiple bits set in bitband mask.")

#define MASK32_TO_BITNUM(mask) \
	((mask) & 0xFFFF0000 == 0 ? MASK16_TO_BITNUM(mask) :	\
	 (mask & 0x0000FFFF == 0) ? MASK16_TO_BITNUM((mask)>>16) :	\
	 "Multiple bits set in bitband mask")

/* The SysTick timer is common across all Cortex implementations.
 * It typically defaults to a 10msec period.
 */
#define SYSTICK_CR		_MMIO_DWORD(0xE000E010)
#define SYSTICK_ARR		_MMIO_DWORD(0xE000E014)
#define SYSTICK_CNT		_MMIO_DWORD(0xE000E018)
#define SYSTICK_STCALIB _MMIO_DWORD(0xE000E01C) /* 10msec period */

#define SysTick_Control		 _MMIO_DWORD(0xE000E010)
#define SysTick_Reload_Value _MMIO_DWORD(0xE000E014)
#define SysTick_Count		 _MMIO_DWORD(0xE000E018)
#define SysTick_Calibration	 _MMIO_DWORD(0xE000E01C)

/* Interrupts are enabled with a pair of 256 bit fields.  One sets the
 * enable, the other clears the enable.
 * Software may set and clear pending interrupts with SET/CLRPEND bit fields.
 */
#define ICTR _MMIO_DWORD(0xE000E004) /* Count of 32 bit words in fields */
#define INTR_SETENA_BASE	((volatile uint32_t *)0xE000E100)
#define INTR_CLRENA_BASE	((volatile uint32_t *)0xE000E180)
#define INTR_SETPEND_BASE	((volatile uint32_t *)0xE000E200)
#define INTR_CLRPEND_BASE	((volatile uint32_t *)0xE000E280)
#define INTR_ACTIVE_BASE	((volatile uint32_t *)0xE000E300) /* Read only */
#define NVIC_IPR_BASE		((volatile uint8_t  *)0xE000E400)
#define NVIC_STIR			_MMIO_DWORD(0xE000EF00)
#define NVIC_PRIORITY NVIC_IPR_BASE

#define INTR_SETENA(intr_num) \
  INTR_SETENA_BASE[(intr_num)>>5] = 1 << ((intr_num) & 0x1F)
#define INTR_CLRENA(intr_num) \
  INTR_CLRENA_BASE[(intr_num)>>5] = 1 << ((intr_num) & 0x1F)
#define INTR_SETPEND(intr_num) \
  INTR_SETPEND_BASE[(intr_num)>>5] = 1 << ((intr_num) & 0x1F)
#define INTR_CLRPEND(intr_num) \
  INTR_CLRPEND_BASE[(intr_num)>>5] = 1 << ((intr_num) & 0x1F)
#define INTR_ACTIVE(intr_num) \
	(INTR_ACTIVE_BASE[(intr_num)>>5] & (1 << ((intr_num) & 0x1F)))

/* The function names suggested by ARM. */
#define __disable_irq()  __asm__("cpsie i	@ __sti" : : : "memory", "cc")
#define __enable_irq() __asm__("cpsid i	@ __cli" : : : "memory", "cc")
#define NVIC_SetPendingIRQ(intr_num) NVIC_STIR = (intr_num)
#define NVIC_EnableIRQ(intr_num) INTR_SETENA(intr_num)
#define NVIC_DisableIRQ(intr_num) INTR_CLRENA(intr_num)
#define NVIC_SetPriority(IRQn, priority) \
	NVIC_IPR_BASE[(IRQn)>>2] = \
		(NVIC_IPR_BASE[(IRQn)>>2] | (0xFF << (((IRQn)&3)*8))) & \
		(priority) << (((IRQn)&3)*8)
#define NVIC_GetPriority(IRQn) \
	(((((volatile uint32_t *)0xE000E400)[(IRQn)>>2]) >> (((IRQn)&3)*8)) & 0xFF


/* The list of ARM core interrupts. */
enum ARMCore_Interrupts	{
	NMI=0, HardFault=1, MemManage=2, BusFault=3, UsageFault=4, SVC=5,
	DebugMon=6, PendSV=7, SysTick=8
};
#define _ARM_HANDLER_ATTRS __attribute__((used,interrupt)) 
void _ARM_HANDLER_ATTRS Reset_Handler(void);
void _ARM_HANDLER_ATTRS BusFault_Handler(void);
void _ARM_HANDLER_ATTRS UsageFault_Handler(void);
void _ARM_HANDLER_ATTRS SVC_Handler(void);
void _ARM_HANDLER_ATTRS DebugMon_Handler(void);
void _ARM_HANDLER_ATTRS PendSV_Handler(void);
void _ARM_HANDLER_ATTRS SysTick_Handler(void);

/* Debug module, Cortex-M3 TRM 7.1.3 */
#define DFSR _MMIO_DWORD(0xE000ED30)
#define DHCSR _MMIO_DWORD(0xE000EDF0)
#define DCRSR _MMIO_DWORD(0xE000EDF4)
#define DCRDR _MMIO_DWORD(0xE000EDF8)
#define DEMCR _MMIO_DWORD(0xE000EDFC)

/* Debug support. */
#define DBGMCU_IDCODE 	_MMIO_DWORD(0xE0042000)
#define DBGMCU_CR 		_MMIO_DWORD(0xE0042004)

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
