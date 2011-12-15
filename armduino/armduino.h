#ifndef _ARMDUINO_H
#define _ARMDUINO_H
/* armduino.h: Arduino-like programming for the STM ARM chips. */
/*
  $Revision: 1.0 $ $Date: 2011/1/22 00:00:21 $
 Hardware definition settings.
 This file provides symbolic names and macros to access the STM 32F10x
 series device registers.

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

 In a similar vein, not every register bit needs a symbolic definition.
 Many are set once, with detailed semantics that requires a datasheet
 reference to change.  It's often difficult to hint at their semantics
 in a name of only a few characters. And for chips like the STM32, most
 configuration fields are left at their default settings

*/

#include <ARM-core.h>

#ifndef __ASSEMBLER__
/* These only work in C programs.  */
#if 0
#include <inttypes.h>

typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;
#endif

#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define _MMIO_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
#define _MMIO_DWORD(mem_addr) (*(volatile uint32_t *)(mem_addr))
#endif

#define BITBAND_SRAM(byte_offset, bit) \
	((BITBAND_SRAM_BASE + (byte_offset - BITBAND_SRAM_REF)*32 + (bit*4)))

/* The AVR has a bunch of ugly types and macros to handle Harvard+flash */
#define PROGMEM const
#define PGM_P const void *
#define PSTR(str) str
#define pgm_read_byte(addr) (*(const char *)(addr))
extern int serprintf(const char *format, ...) __attribute__ ((format(printf, 1, 2)));;
typedef uint8_t prog_uint8_t;
typedef uint16_t prog_uint16_t;
typedef uint32_t prog_uint32_t;
typedef int8_t prog_int8_t;
typedef int16_t prog_int16_t;
typedef int32_t prog_int32_t;


#define TIM1		0x40012C00 	/* Timer1, not in order */
#define TIM2		0x40000000 		/* Timers 2-7 */
#define TIM3		0x40000400
#define TIM4		0x40000800
#define TIM5		0x40000C00 		/* Not on low-density parts */
#define TIM6		0x40001000
#define TIM7		0x40001400
#define TIM12		0x40001800 		/* TIM12-14 not on low-dens */
#define TIM13		0x40001C00
#define TIM14		0x40002000
#define TIM15		0x40014000 	/* Out of address order */
#define TIM16		0x40014400
#define TIM17		0x40014800

#define RTC		0x40002800
#define WWDG		0x40002800 	/* Window Watchdog timer */
#define IWDG		0x40002800 	/* Independent watchdog */
#define SPI1		0x40013000 	/* SPI 1 out of order. */
#define SPI2		0x40003800
#define SPI3		0x40003C00 	/* Hi-dens only */
#if 0
#endif
#define USART1_BASE		0x40013800 	/* Address out of order */
#define USART2_BASE		0x40004400
#define USART3_BASE		0x40004800
#define UART4_BASE		0x40004C00 	/* UART4-5 Hi-dens only */
#define UART5_BASE		0x40005000
#define I2C1		0x40005400
#define I2C2		0x40005800

#define BKP		0x40006C00 	/* Data preserved at Power-down  */
#define PWR		0x40007000 	/* Power control  */
#define DAC		0x40007400 	/* D/A converter  */
#define CEC		0x40007800 	/* Consumer Electronics  */
#define AFIO		0x40010000 	/* Alternate function remap */
#define EXTI		0x40010400 	/* External interrupt config  */

#define ADC1		0x40012400		/* A/D Converter */
/* TIM1, SPI1 UART1 and TIM15-17 are next in address order. */
#define DMA1		0x40020000
#define DMA2		0x40020400 	/* High density only */
#define RCC		0x40021000		/* Reset and Clock Control */
#define FLASH		0x40022000		/* Flash memory interface */
#define CRC		0x40023000		/* CRC Checksum generator */


/* Port A-G, base, config high/low, output data registers */
#define GPIOA_BASE	0x40010800		/* PortA register base */
#define GPIOB_BASE	0x40010C00		/* PortB register base */
#define GPIOC_BASE	0x40011000		/* PortC register base */
#define GPIOD_BASE	0x40011400		/* PortD register base */
#define GPIOE_BASE	0x40011800		/* PortE register base */
#define GPIOF_BASE	0x40011C00		/* PortF register base */
#define GPIOG_BASE	0x40012000		/* PortG register base */



#define PORTA		_MMIO_DWORD(0x40010808) /* The input data register */
#define GPIOA_CRL	_MMIO_DWORD(0x40010800)
#define GPIOA_CRH	_MMIO_DWORD(0x40010804)
#define GPIOA_IDR	_MMIO_DWORD(0x40010808)
#define GPIOA_ODR	_MMIO_DWORD(0x4001080C)
#define GPIOA_BSRR	_MMIO_DWORD(0x40010810)
#define GPIOA_BRR	_MMIO_DWORD(0x40010814)
#define GPIOA_LCKR	_MMIO_DWORD(0x40010818)

#define PORTB		_MMIO_DWORD(0x40010C08)
#define GPIOB_CRL	_MMIO_DWORD(0x40010C00)
#define GPIOB_CRH	_MMIO_DWORD(0x40010C04)
#define GPIOB_IDR	_MMIO_DWORD(0x40010C08)
#define GPIOB_ODR	_MMIO_DWORD(0x40010C0C)
#define GPIOB_BSRR	_MMIO_DWORD(0x40010C10)
#define GPIOB_BRR	_MMIO_DWORD(0x40010C14)
#define GPIOB_LCKR	_MMIO_DWORD(0x40010C18)

#define PORTC		_MMIO_DWORD(0x40011008)
#define GPIOC_CRL	_MMIO_DWORD(0x40011000)
#define GPIOC_CRH	_MMIO_DWORD(0x40011004)
#define GPIOC_IDR	_MMIO_DWORD(0x40011008)
#define GPIOC_ODR	_MMIO_DWORD(0x4001100C)
#define GPIOC_BSRR	_MMIO_DWORD(0x40011010)
#define GPIOC_BRR	_MMIO_DWORD(0x40011014)
#define GPIOC_LCKR	_MMIO_DWORD(0x40011018)

#define PORTD		_MMIO_DWORD(0x40011408)
#define GPIOD_CRL	_MMIO_DWORD(0x40011400)
#define GPIOD_CRH	_MMIO_DWORD(0x40011404)
#define GPIOD_IDR	_MMIO_DWORD(0x40011408)
#define GPIOD_ODR	_MMIO_DWORD(0x4001140C)
#define GPIOD_BSRR	_MMIO_DWORD(0x40011410)
#define GPIOD_BRR	_MMIO_DWORD(0x40011414)
#define GPIOD_LCKR	_MMIO_DWORD(0x40011418)

#define PORTE		_MMIO_DWORD(0x40011808)
#define GPIOE_CRL	_MMIO_DWORD(0x40011800)
#define GPIOE_CRH	_MMIO_DWORD(0x40011804)
#define GPIOE_IDR	_MMIO_DWORD(0x40011808)
#define GPIOE_ODR	_MMIO_DWORD(0x4001180C)
#define GPIOE_BSRR	_MMIO_DWORD(0x40011810)
#define GPIOE_BRR	_MMIO_DWORD(0x40011814)
#define GPIOE_LCKR	_MMIO_DWORD(0x40011818)

#define PORTF		_MMIO_DWORD(0x40011C08)
#define GPIOF_CRL	_MMIO_DWORD(0x40011C00)
#define GPIOF_CRH	_MMIO_DWORD(0x40011C04)
#define GPIOF_IDR	_MMIO_DWORD(0x40011C08)
#define GPIOF_ODR	_MMIO_DWORD(0x40011C0C)
#define GPIOF_BSRR	_MMIO_DWORD(0x40011C10)
#define GPIOF_BRR	_MMIO_DWORD(0x40011C14)
#define GPIOF_LCKR	_MMIO_DWORD(0x40011C18)

#define PORTG		_MMIO_DWORD(0x40012008)
#define GPIOG_CRL	_MMIO_DWORD(0x40012000)
#define GPIOG_CRH	_MMIO_DWORD(0x40012004)
#define GPIOG_IDR	_MMIO_DWORD(0x40012008)
#define GPIOG_ODR	_MMIO_DWORD(0x4001200C)
#define GPIOG_BSRR	_MMIO_DWORD(0x40012010)
#define GPIOG_BRR	_MMIO_DWORD(0x40012014)
#define GPIOG_LCKR	_MMIO_DWORD(0x40012018)

/* Reset and Clock Control at 0x40021000 */
#define RCC_CR		_MMIO_DWORD(0x40021000)
#define RCC_CFGR	_MMIO_DWORD(0x40021004)
#define RCC_CIR		_MMIO_DWORD(0x40021008)
#define APB2RSTR	_MMIO_DWORD(0x4002100C)
#define APB1RSTR	_MMIO_DWORD(0x40021010)
#define AHBENR		_MMIO_DWORD(0x40021014)
#define APB2ENR		_MMIO_DWORD(0x40021018)	/* Enable peripheral clocks */
#define APB1ENR		_MMIO_DWORD(0x4002101C)
#define RCC_BDCR	_MMIO_DWORD(0x40021020)
#define RCC_CSR		_MMIO_DWORD(0x40021024)
#define RCC_CFGR2	_MMIO_DWORD(0x4002102C)

/* Timer registers.  32 bit access used, 16 bit access also OK*/
/* Timer 1 base is at 0x40012C00 */
#define TIM1_CR1	_MMIO_DWORD(0x40012C00)
#define TIM1_CR2	_MMIO_DWORD(0x40012C04)
#define TIM1_SMCR	_MMIO_DWORD(0x40012C08)
#define TIM1_DIER	_MMIO_DWORD(0x40012C0C)
#define TIM1_SR		_MMIO_DWORD(0x40012C10)
#define TIM1_EGR	_MMIO_DWORD(0x40012C14)
#define TIM1_CCMR1	_MMIO_DWORD(0x40012C18) /* chn 1 & 2 */
#define TIM1_CCMR2	_MMIO_DWORD(0x40012C1C) /* chn 3 & 4 */
#define TIM1_CCER	_MMIO_DWORD(0x40012C20)
#define TIM1_CNT	_MMIO_DWORD(0x40012C24) /* Count value */
#define TIM1_PSC	_MMIO_DWORD(0x40012C28) /* Prescale divider */
#define TIM1_ARR	_MMIO_DWORD(0x40012C2C) /* Counter reset value. */
#define TIM1_RCR	_MMIO_DWORD(0x40012C30) /* Repetition counter. */
#define TIM1_CCR1	_MMIO_DWORD(0x40012C34) /* Capture/compare values. */
#define TIM1_CCR2	_MMIO_DWORD(0x40012C38)
#define TIM1_CCR3	_MMIO_DWORD(0x40012C3C)
#define TIM1_CCR4	_MMIO_DWORD(0x40012C40)
#define TIM1_BDTR	_MMIO_DWORD(0x40012C44)
#define TIM1_DCR	_MMIO_DWORD(0x40012C48) /* DMA config and address */
#define TIM1_DMAR	_MMIO_DWORD(0x40012C4C)

/* Timer 2 base is at 0x40000000 */
#define TIM2_CR1	_MMIO_DWORD(0x40000000)
#define TIM2_CR2	_MMIO_DWORD(0x40000004)
#define TIM2_SMCR	_MMIO_DWORD(0x40000008)
#define TIM2_DIER	_MMIO_DWORD(0x4000000C)
#define TIM2_SR		_MMIO_DWORD(0x40000010)
#define TIM2_EGR	_MMIO_DWORD(0x40000014)
#define TIM2_CCMR1	_MMIO_DWORD(0x40000018) /* chn 1 & 2 */
#define TIM2_CCMR2	_MMIO_DWORD(0x4000001C) /* chn 3 & 4 */
#define TIM2_CCER	_MMIO_DWORD(0x40000020)
#define TIM2_CNT	_MMIO_DWORD(0x40000024) /* Count value */
#define TIM2_PSC	_MMIO_DWORD(0x40000028) /* Prescale divider */
#define TIM2_ARR	_MMIO_DWORD(0x4000002C) /* Counter reset value. */
/* No RCR / repetition counter, it is only on TIM1. */
#define TIM2_CCR1	_MMIO_DWORD(0x40000034) /* Capture/compare values. */
#define TIM2_CCR2	_MMIO_DWORD(0x40000038)
#define TIM2_CCR3	_MMIO_DWORD(0x4000003C)
#define TIM2_CCR4	_MMIO_DWORD(0x40000040)
/* No DeadTime / BDTR register, it is only on TIM1. */
#define TIM2_DCR	_MMIO_DWORD(0x40000048) /* DMA config and address */
#define TIM2_DMAR	_MMIO_DWORD(0x4000004C)
#define TIM2_OR 	_MMIO_DWORD(0x40000050) /* Trigger options, only w/TIM2 */

/* Timer 3 registers, base 0x40000400. */
#define TIM3_CR1	_MMIO_DWORD(0x40000400)
#define TIM3_CR2	_MMIO_DWORD(0x40000404)
#define TIM3_SMCR	_MMIO_DWORD(0x40000408)
#define TIM3_DIER	_MMIO_DWORD(0x4000040C)
#define TIM3_SR		_MMIO_DWORD(0x40000410)
#define TIM3_EGR	_MMIO_DWORD(0x40000414)
#define TIM3_CCMR1	_MMIO_DWORD(0x40000418) /* chn 1 & 2 */
#define TIM3_CCMR2	_MMIO_DWORD(0x4000041C) /* chn 3 & 4 */
#define TIM3_CCER	_MMIO_DWORD(0x40000420)
#define TIM3_CNT	_MMIO_DWORD(0x40000424) /* Count value */
#define TIM3_PSC	_MMIO_DWORD(0x40000428) /* Prescale divider */
#define TIM3_ARR	_MMIO_DWORD(0x4000042C) /* Counter reset value. */
#define TIM3_CCR1	_MMIO_DWORD(0x40000434) /* Capture/compare values. */
#define TIM3_CCR2	_MMIO_DWORD(0x40000438)
#define TIM3_CCR3	_MMIO_DWORD(0x4000043C)
#define TIM3_CCR4	_MMIO_DWORD(0x40000440)
#define TIM3_DCR	_MMIO_DWORD(0x40000448) /* DMA config and address */
#define TIM3_DMAR	_MMIO_DWORD(0x4000044C)

/* Timer 4 registers, base 0x40000800. */
#define TIM4_CR1	_MMIO_DWORD(0x40000800)
#define TIM4_CR2	_MMIO_DWORD(0x40000804)
#define TIM4_SMCR	_MMIO_DWORD(0x40000808)
#define TIM4_DIER	_MMIO_DWORD(0x4000080C)
#define TIM4_SR		_MMIO_DWORD(0x40000810)
#define TIM4_EGR	_MMIO_DWORD(0x40000814)
#define TIM4_CCMR1	_MMIO_DWORD(0x40000818) /* chn 1 & 2 */
#define TIM4_CCMR2	_MMIO_DWORD(0x4000081C) /* chn 3 & 4 */
#define TIM4_CCER	_MMIO_DWORD(0x40000820)
#define TIM4_CNT	_MMIO_DWORD(0x40000824) /* Count value */
#define TIM4_PSC	_MMIO_DWORD(0x40000828) /* Prescale divider */
#define TIM4_ARR	_MMIO_DWORD(0x4000082C) /* Counter reset value. */
#define TIM4_CCR1	_MMIO_DWORD(0x40000834) /* Capture/compare values. */
#define TIM4_CCR2	_MMIO_DWORD(0x40000838)
#define TIM4_CCR3	_MMIO_DWORD(0x4000083C)
#define TIM4_CCR4	_MMIO_DWORD(0x40000840)
#define TIM4_DCR	_MMIO_DWORD(0x40000848) /* DMA config and address */
#define TIM4_DMAR	_MMIO_DWORD(0x4000084C)

/* Window Watchdog timer */
#define WWDG_CR		_MMIO_DWORD(0x40002800)
#define  WDGA	0x80
#define WWDG_CFR	_MMIO_DWORD(0x40002804)
#define WWDG_SR		_MMIO_DWORD(0x40002808)
#define  EWIF	0x01

#define SPI1		0x40013000 	/* SPI 1 is not in address order. */
#define SPI1_CR1	_MMIO_WORD(0x40013000)
#define  SPI_LSBFIRST	0x0080
#define  SPI_SPE	0x0040
#define  SPI_BR0	3			/* Shift value */
#define  SPI_BRdiv2 0x0000
#define  SPI_BRdiv4 0x0008
#define  SPI_BRdiv8 0x0010
#define  SPI_MSTR	0x0004
#define  SPI_CPOL	0x0002		/* Clock polarity and phase */
#define  SPI_CPHA	0x0001
#define SPI1_CR2	_MMIO_WORD(0x40013004)
#define  SPI_TXEIE	0x0080
#define  SPI_RXNEIE	0x0040
#define  SPI_ERRIE	0x0020
#define  SPI_SSOE	0x0004
#define  SPI_TXDMAEN	0x0002
#define  SPI_RXDMAEN	0x0001
#define SPI1_SR 	_MMIO_WORD(0x40013008)
#define  SPI_TXE	0x02
#define  SPI_RXNE	0x01
#define SPI1_DR 	_MMIO_WORD(0x4001300C)
#define SPI1_CRC 	_MMIO_WORD(0x40013010)
#define SPI1_RXCRCR	_MMIO_WORD(0x40013014)
#define SPI1_TXCRCR	_MMIO_WORD(0x40013018)

#define USART1_SR	_MMIO_WORD(0x40013800)
#define  USART_CTS	0x200
#define  USART_LBD	0x100
#define  USART_TXE	0x80
#define  USART_TC	0x40
#define  USART_RXNE	0x20
#define  USART_IDLE	0x10
#define  USART_OVE	0x08
#define  USART_NF	0x04
#define  USART_FE	0x02
#define  USART_PE	0x01
#define USART1_DR	_MMIO_WORD(0x40013804)
#define USART1_BRR	_MMIO_WORD(0x40013808)
#define USART1_CR1	_MMIO_WORD(0x4001380C)
#define  USART_OVER8 0x8000		/* 0: 16x baud clock, 1: 8x clock */
#define  USART_UE 	 0x2000		/* USART enable */
#define  USART_M9	 0x1000		/* Set for 9 bits */
#define  USART_PCE	 0x0400		/* Set for parity on */
#define  USART_PS	 0x0200		/* Set for odd parity */
#define  USART_TXEIE 0x0080		/* TXE interrupt enable */
#define  USART_TCIE	 0x0040		/* Tx done interrupt enable */
#define  USART_RXNEIE 0x0020	/* Rx not empty interrupt enable */
#define  USART_TE	 0x0008		/* Tx enable */
#define  USART_RE	 0x0004		/* Rx enable */
#define USART1_CR2	_MMIO_WORD(0x40013810)
#define USART1_CR3	_MMIO_WORD(0x40013814)
#define USART1_GTPR	_MMIO_WORD(0x40013818)

#define USART2_SR	_MMIO_WORD(0x40004400)
#define USART2_DR	_MMIO_WORD(0x40004404)
#define USART2_BRR	_MMIO_WORD(0x40004408)
#define USART2_CR1	_MMIO_WORD(0x4000440C)
#define USART2_CR2	_MMIO_WORD(0x40004410)
#define USART2_CR3	_MMIO_WORD(0x40004414)
#define USART2_GTPR	_MMIO_WORD(0x40004418)

#define USART3_SR	_MMIO_WORD(0x40004800)
#define USART3_DR	_MMIO_WORD(0x40004804)
#define USART3_BRR	_MMIO_WORD(0x40004808)
#define USART3_CR1	_MMIO_WORD(0x4000480C)
#define USART3_CR2	_MMIO_WORD(0x40004810)
#define USART3_CR3	_MMIO_WORD(0x40004814)
#define USART3_GTPR	_MMIO_WORD(0x40004818)

/* Alternate function configuration */
#define AFIO_EVCR	_MMIO_DWORD(0x40010000)
#define AFIO_MAPR	_MMIO_DWORD(0x40010004)
#define AFIO_MAPR1	_MMIO_DWORD(0x40010004)
#define  REMAP1_JTAGRST_OFF	 0x01000000
#define  REMAP1_JTAG_OFF	 0x02000000
#define  REMAP1_JTAG_SWD_OFF 0x04000000
#define  REMAP1_TIM4	0x1000
#define  REMAP1_USART2  0x0008
#define  REMAP1_USART1  0x0004
#define  REMAP1_I2C1	0x0002
#define  REMAP1_SPI1	0x0001

#define AFIO_MAPR2	_MMIO_DWORD(0x4001001C)
#define AFIO_EXTICR1	_MMIO_DWORD(0x40010008)
#define AFIO_EXTICR2	_MMIO_DWORD(0x4001000C)
#define AFIO_EXTICR3	_MMIO_DWORD(0x40010010)
#define AFIO_EXTICR4	_MMIO_DWORD(0x40010014)

#define ADC1_BASE		0x40012400	/* A/D Converter, dword access only */
#define ADC1_SR		_MMIO_DWORD(0x40012400)
#define  ADC_STRT	 0x0010
#define  ADC_JSTRT	 0x0008
#define  ADC_JEOC	 0x0004		/* End of injected conversion signal */
#define  ADC_EOC	 0x0002		/* End of conversion signal */
#define  ADC_AWD	 0x0001
#define ADC1_CR1	_MMIO_DWORD(0x40012404)
#define ADC1_CR2	_MMIO_DWORD(0x40012408)
#define  ADC_TSVREFE  0x800000	/* Enable temp and 1.2V channels. */
#define  ADC_SWSTART  0x400000	/* Start conversion of regular channels */
#define  ADC_JSWSTART 0x200000	/* Start conversion of injected channels */
#define  ADC_EXTTRIG  0x100000	/* Trigger mode for regular channels */
#define  ADC_JEXTTRIG 0x8000	/* Trigger source for injected channels */
#define  ADC_ALIGN	  0x0800	/* Set for left alignment */
#define  ADC_RSTCAL	  0x0008
#define  ADC_CAL	  0x0004
#define  ADC_CONT	  0x0002
#define  ADC_ADON	  0x0001
#define ADC1_SMPR1	_MMIO_DWORD(0x4001240C)
#define ADC1_SMPR2	_MMIO_DWORD(0x40012410)
#define ADC1_JOFR1	_MMIO_DWORD(0x40012414) /* ADC injected sequence list */
#define ADC1_JOFR2	_MMIO_DWORD(0x40012418)
#define ADC1_JOFR3	_MMIO_DWORD(0x4001241C)
#define ADC1_JOFR4	_MMIO_DWORD(0x40012420)
#define ADC1_HTR	_MMIO_DWORD(0x40012424) /* Comparator high threshold  */
#define ADC1_LTR	_MMIO_DWORD(0x40012428) /* Comparator low threshold  */
#define ADC1_SQR1	_MMIO_DWORD(0x4001242C) /* ADC regular scan sequence list */
#define ADC1_SQR2	_MMIO_DWORD(0x40012430)
#define ADC1_SQR3	_MMIO_DWORD(0x40012434)
#define ADC1_JSQR	_MMIO_DWORD(0x40012438) /* Injected sequence list */
#define ADC1_JDR1	_MMIO_DWORD(0x4001243C) /* Injected conversion result */
#define ADC1_JDR2	_MMIO_DWORD(0x40012440) /* Injected conversion result */
#define ADC1_JDR3	_MMIO_DWORD(0x40012444) /* Injected conversion result */
#define ADC1_JDR4	_MMIO_DWORD(0x40012448) /* Injected conversion result */
#define ADC1_DR		_MMIO_DWORD(0x4001244C) /* ADC result */

#define DMA_ISR		_MMIO_DWORD(0x40020000) /* DMA1 intr status */
#define DMA_IFCR	_MMIO_DWORD(0x40020004) /* DMA1 ctrl */
#define DMA_CCR1	_MMIO_DWORD(0x40020008) /* DMA1 ch1 control/config */
#define DMA_CNDTR1	_MMIO_DWORD(0x4002000C) /*  data count */
#define DMA_CPAR1	_MMIO_DWORD(0x40020010) /*  peripheral addr */
#define DMA_CMAR1	_MMIO_DWORD(0x40020014) /*  memory addr*/
#define DMA_CCR2	_MMIO_DWORD(0x4002001C) /* DMA1 ch2 control/config */
#define DMA_CNDTR2	_MMIO_DWORD(0x40020020) /*  data count */
#define DMA_CPAR2	_MMIO_DWORD(0x40020024) /*  peripheral addr */
#define DMA_CMAR2	_MMIO_DWORD(0x40020028) /*  memory addr*/
#define DMA_CCR3	_MMIO_DWORD(0x40020030) /* DMA1 ch3 control/config */
#define DMA_CNDTR3	_MMIO_DWORD(0x40020034) /*  data count */
#define DMA_CPAR3	_MMIO_DWORD(0x40020038) /*  peripheral addr */
#define DMA_CMAR3	_MMIO_DWORD(0x4002003C) /*  memory addr*/
#define DMA_CCR4	_MMIO_DWORD(0x40020044) /* DMA1 ch4 control/config */
#define DMA_CNDTR4	_MMIO_DWORD(0x40020048) /*  data count */
#define DMA_CPAR4	_MMIO_DWORD(0x4002004C) /*  peripheral addr */
#define DMA_CMAR4	_MMIO_DWORD(0x40020050) /*  memory addr*/
#define DMA_CCR5	_MMIO_DWORD(0x40020058) /* DMA1 ch5 control/config */
#define DMA_CNDTR5	_MMIO_DWORD(0x4002005C) /*  data count */
#define DMA_CPAR5	_MMIO_DWORD(0x40020060) /*  peripheral addr */
#define DMA_CMAR5	_MMIO_DWORD(0x40020064) /*  memory addr*/
#define DMA_CCR6	_MMIO_DWORD(0x4002006C) /* DMA1 ch6 control/config */
#define DMA_CNDTR6	_MMIO_DWORD(0x40020070) /*  data count */
#define DMA_CPAR6	_MMIO_DWORD(0x40020074) /*  peripheral addr */
#define DMA_CMAR6	_MMIO_DWORD(0x40020078) /*  memory addr*/
#define DMA_CCR7	_MMIO_DWORD(0x40020080) /* DMA1 ch7 control/config */
#define DMA_CNDTR7	_MMIO_DWORD(0x40020084) /*  data count */
#define DMA_CPAR7	_MMIO_DWORD(0x40020088) /*  peripheral addr */
#define DMA_CMAR7	_MMIO_DWORD(0x4002008C) /*  memory addr*/



#define CAN1_MCR	_MMIO_DWORD(0x40006400) /* CAN master control register. */
#define  CAN_MCR_DBF   0x10000	/* Set to turn off Rx/Tx during debug. */
#define  CAN_MCR_RESET 0x8000	/* Reset and leave in sleep mode. */
#define  CAN_MCR_TTCM 0x80
#define  CAN_MCR_ABOM 0x40		/* A good example of why symbolic names */
#define  CAN_MCR_AWUM 0x20		/* are of marginal use -- you must read */
#define  CAN_MCR_NART 0x10		/* the manual to understand anyway. */
#define  CAN_MCR_RFLM	0x08
#define  CAN_MCR_TXFP	0x04
#define  CAN_MCR_SLEEP	0x02
#define  CAN_MCR_INRQ	0x01		/* Initialization request. */
#define CAN1_MSR	_MMIO_DWORD(0x40006404) /* Status register. */
#define  CAN_MSR_SLAKI	0x10		/* Sleep interrupt+ack. */
#define  CAN_MSR_WAKI	0x08		/* Wake  interrupt+ack. */
#define  CAN_MSR_ERRI	0x04		/* Error interrupt+ack. */
#define  CAN_MSR_SLAK	0x02		/* Sleep ack. */
#define  CAN_MSR_INAK	0x01		/* Initialization ack. */
#define CAN1_TSR	_MMIO_DWORD(0x40006408) /* Tx status register. */
#define  CAN_TSR_TME2  0x10000000
#define  CAN_TSR_TME1  0x08000000
#define  CAN_TSR_TME0  0x04000000
#define  CAN_TSR_TERR0 0x0002
#define  CAN_TSR_TXOK0 0x0002
#define  CAN_TSR_RQCP0 0x0001
#define CAN1_RF0R	_MMIO_DWORD(0x4000640C) /* Rx FIFO 0 status register. */
#define  CAN_RxFIFO_Release 0x20
#define  CAN_RxFIFO_FMP 0x03
#define CAN1_RF1R	_MMIO_DWORD(0x40006410) /* Rx FIFO 1 */
#define CAN1_IER	_MMIO_DWORD(0x40006414) /* Interrupt enable. */
/* Interrupt enables */
#define  CAN_IER_Rx1Msg	0x0010
#define  CAN_IER_FMPIE1	0x0010
#define  CAN_IER_Rx0Msg	0x0002
#define  CAN_IER_FMPIE0	0x0002
#define  CAN_IER_TMEIE	0x0001
#define CAN1_ESR	_MMIO_DWORD(0x40006418) /* Error status. */
#define CAN1_BTR	_MMIO_DWORD(0x4000641C) /* Bit timing. */
#define  CAN_BTR_SILM	0x80000000			/* Silent mode */
#define  CAN_BTR_LBKM	0x40000000			/* Loopback mode */
#define CAN1_TI0R 	_MMIO_DWORD(0x40006580) /* Tx mailbox 0 ID */
#define CAN1_TI1R 	_MMIO_DWORD(0x40006590) /* Tx 1 ID */
#define CAN1_TI2R 	_MMIO_DWORD(0x400065A0) /* Tx 2 ID */
#define CAN1_RI0R 	_MMIO_DWORD(0x400065B0) /* Rx mailbox 0 ID */
#define CAN1_RI1R 	_MMIO_DWORD(0x400065C0) /* Rx 1 ID */

#define CAN_FMR 	_MMIO_DWORD(0x40006600) /* Filter master register */
/* The following are bitmaps for the 14 or 28 filter sets. */
#define CAN_FM1R 	_MMIO_DWORD(0x40006604) /* Filter mode, ID+mask or ID+ID */
#define CAN_FS1R 	_MMIO_DWORD(0x4000660C) /* Filter scale */
#define CAN_FFA1R 	_MMIO_DWORD(0x40006614) /* Filter to RxFIFO 0 or 1 */
#define CAN_FA1R 	_MMIO_DWORD(0x4000661C) /* Filter activation */
#define CAN_FILTERS  (&_MMIO_DWORD(0x40006640))

#define CAN2_MCR	_MMIO_DWORD(0x40006800) /* CAN master control register. */
#define CAN2_MSR	_MMIO_DWORD(0x40006804) /* Status register. */
#define CAN2_TSR	_MMIO_DWORD(0x40006808) /* Tx status register. */
#define CAN2_RF0R	_MMIO_DWORD(0x4000680C) /* Rx FIFO 0 status register. */
#define CAN2_RF1R	_MMIO_DWORD(0x40006810) /* Rx FIFO 1 */
#define CAN2_IER	_MMIO_DWORD(0x40006814) /* Interrupt enable. */
#define CAN2_ESR	_MMIO_DWORD(0x40006818) /* Error status. */
#define CAN2_BTR	_MMIO_DWORD(0x4000681C) /* Bit timing. */
#define CAN2_TI0R 	_MMIO_DWORD(0x40006980)
#define CAN2_TI1R 	_MMIO_DWORD(0x40006990)
#define CAN2_TI2R 	_MMIO_DWORD(0x400069A0)
#define CAN2_RI0R 	_MMIO_DWORD(0x400069B0)
#define CAN2_RI1R 	_MMIO_DWORD(0x400069C0)


/* Reset and Clock enables bits in APB1RSTR and APB1ENR: */
#define APB1ENR_TIM2EN  0x0001
#define APB1ENR_TIM3EN  0x0002
#define APB1ENR_TIM4EN  0x0004
#define APB1ENR_TIM5EN  0x0008
#define APB1ENR_TIM6EN  0x0010
#define APB1ENR_TIM7EN  0x0020
#define APB1ENR_TIM12EN 0x0040
#define APB1ENR_TIM13EN 0x0080
#define APB1ENR_TIM14EN 0x0100
#define APB1ENR_WWDGEN  0x0800	    /* Window Watchdog */
#define APB1ENR_SPI2EN  0x4000
#define APB1ENR_SPI3EN  0x8000
#define APB1ENR_USART2EN 0x00020000
#define APB1ENR_USART3EN 0x00040000
#define APB1ENR_UART4EN	 0x00080000
#define APB1ENR_UART5EN	 0x00100000
#define APB1ENR_I2C1EN	 0x00200000
#define APB1ENR_I2C2EN	 0x00400000
#define APB1ENR_USBEN	 0x00800000  /* USB */
#define APB1ENR_CAN1EN	 0x02000000  /* CAN bus */
#define APB1ENR_CAN2EN	 0x04000000
#define APB1ENR_BKPEN	 0x08000000  /* Backup interface */
#define APB1ENR_PWREN	 0x10000000  /* Power interface */
#define APB1ENR_DACEN	 0x20000000
#define APB1ENR_CECEN	 0x40000000  /* CEC interface */ 

#if defined(STM32F10X_LD_VL) || defined(STM32F10X_MD_VL)
#undef APB1ENR_CAN1EN
#endif

#if defined(STM32F10X_LD_VL) || defined(STM32F10X_LD)
#undef APB1ENR_TIM4EN
#undef APB1ENR_SPI2EN
#undef APB1ENR_USART3EN
#undef APB1ENR_I2C2EN
#endif

#if ! (defined(STM32F10X_HD) || defined(STM32F10X_MD) || defined(STM32F10X_LD))
#undef APB1ENR_USBEN
#endif

#if ! (defined (STM32F10X_HD) || defined  (STM32F10X_CL))
#undef APB1ENR_TIM5EN
#undef APB1ENR_TIM6EN
#undef APB1ENR_TIM7EN
#undef APB1ENR_SPI3EN
#undef APB1ENR_UART4EN
#undef APB1ENR_UART5EN
#undef APB1ENR_DACEN
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL)
#undef APB1ENR_TIM6EN
#undef APB1ENR_TIM7EN
#undef APB1ENR_DACEN
#undef APB1ENR_CECEN
#endif

#if ! defined(STM32F10X_CL)
#undef APB1ENR_CAN2EN
#endif

#if ! defined(STM32F10X_XL)
#undef APB1ENR_TIM12EN
#undef APB1ENR_TIM13EN
#undef APB1ENR_TIM14EN
#endif

#define APB2ENR_AFIOEN  0x00000001 /* Alternate Function I/O clock enable */
#define APB2ENR_IOPAEN  0x00000004 /* I/O port A clock enable */
#define APB2ENR_IOPBEN  0x00000008 /* I/O port B clock enable */
#define APB2ENR_IOPCEN  0x00000010 /* I/O port C clock enable */
#define APB2ENR_IOPDEN  0x00000020 /* I/O port D clock enable */
#define APB2ENR_ADC1EN  0x00000200 /* ADC 1 interface clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL)
#define APB2ENR_ADC2EN  0x00000400 /* ADC 2 interface clock enable */
#endif

#define APB2ENR_TIM1EN  0x00000800 /* TIM1 Timer clock enable */
#define APB2ENR_SPI1EN  0x00001000 /* SPI 1 clock enable */
#define APB2ENR_USART1EN  0x00004000 /* USART1 clock enable */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
#define APB2ENR_TIM15EN  0x00010000 /* TIM15 Timer clock enable */
#define APB2ENR_TIM16EN  0x00020000 /* TIM16 Timer clock enable */
#define APB2ENR_TIM17EN  0x00040000 /* TIM17 Timer clock enable */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define APB2ENR_IOPEEN  0x00000040 /* I/O port E clock enable */
#endif

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define APB2ENR_IOPFEN  0x00000080 /* I/O port F clock enable */
 #define APB2ENR_IOPGEN  0x00000100 /* I/O port G clock enable */
 #define APB2ENR_TIM8EN  0x00002000 /* TIM8 Timer clock enable */
 #define APB2ENR_ADC3EN  0x00008000 /* DMA1 clock enable */
#endif

#ifdef STM32F10X_XL
 #define APB2ENR_TIM9EN  0x00080000 /* TIM9 Timer clock enable  */
 #define APB2ENR_TIM10EN  0x00100000 /* TIM10 Timer clock enable  */
 #define APB2ENR_TIM11EN  0x00200000 /* TIM11 Timer clock enable */
#endif

#define CRC_DR	_MMIO_DWORD(0x40023000)
#define CRC_IDR _MMIO_DWORD(0x40023004) 	/* 8 bit temp reg, pointless */
#define CRC_CR	_MMIO_DWORD(0x40023008) 	/* Reset by writing 0x01 */


#define __INTR_ATTRS used,interrupt
#define __ISR_ATTRS __attribute__((__INTR_ATTRS)) 
#define IRQHandler(vector, ...) ISR(vector, __VA_ARGS__)
#define ISR(vector, ...)            \
    void vector##_IRQHandler(void) __attribute__ ((__INTR_ATTRS)) __VA_ARGS__; \
    void vector##_IRQHandler(void)

/* STM32 interrupt index.  This number is used when enabling the
 * interrupt or setting the priority.
 */
enum STM_Interrupts {
	WWDG_Intr=0, PVD_Intr=1, TAMPER_STAMP_Intr=2,
	RTC_WKUP_Intr=3, FLASH_Intr=4, RCC_Intr=5,
	EXTI0_Intr=6, EXTI1_Intr=7, EXTI2_Intr=8, EXTI3_Intr=9, EXTI4_Intr=10,
	DMA1_Channel1_Intr=11, DMA1_Channel2_Intr=12, DMA1_Channel3_Intr=13,
	DMA1_Channel4_Intr=14, DMA1_Channel5_Intr=15, DMA1_Channel6_Intr=16,
	DMA1_Channel7_Intr=17,
	ADC1_Intr=18,
	CAN1_Tx_Intr=19, CAN1_Rx0_Intr=20, CAN1_Rx1_Intr=21, CAN1_SCE_Intr=22,
	EXTI9_5_Intr=23,
	TIM1_BRK_TIM15_Intr=24, TIM1_UP_TIM16_Intr=25, TIM1_TRG_COM_TIM17_Intr=26,
	TIM1_CC_Intr=27, TIM2_Intr=28, TIM3_Intr=29, TIM4_Intr=30,
	I2C1_EV_Intr=31, I2C1_ER_Intr=32,  I2C2_EV_Intr=33, I2C2_ER_Intr=34,
	SPI1_Intr=35, SPI2_Intr=36,
	USART1_Intr=37, USART2_Intr=38, USART3_Intr=39,
	EXTI15_10_Intr=40, RTC_Alarm_Intr=41,
	CEC_Intr=42, USBWakeUp_Intr=42,		/* Alt: USB On-The-Go FS Wakeup */
	TIM12_Intr=43, TIM13_Intr=44, TIM14_Intr=45,
	/* Only on some chips. */
	TIM8_CC_Intr=46, ADC3_Intr=47, 
	FSMC_Intr=48,
	SDIO_Intr=49, USB_OTG_FS_WKUP_Intr=49, /* Overlap, dependent on chip */
	TIM5_Intr=50, SPI3_Intr=51, UART4_Intr=52, UART5_Intr=53,
	TIM6_DAC_Intr=54, TIM7_Intr=55,
	DMA2_Channel1_Intr=56, DMA2_Channel2_Intr=57, DMA2_Channel3_Intr=58,
	DMA2_Channel4_5_Intr=59, DMA2_Channel5_Intr=60,
	ETH_Intr=61, ETH_WkUp_Intr=62,
	CAN2_Tx_Intr=63, CAN2_Rx0_Intr=64, CAN2_Rx1_Intr=65, CAN2_SCE_Intr=66,
	USB_OTG_FS_Intr=67
};

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
