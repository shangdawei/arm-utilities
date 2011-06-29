/* crt-stm.c: C run-time initialization for the STM32. */
/* This file implements the system initialization for the STM32 family.
 *
 * It avoids the most common beginner's problem with the STM32 family
 * by initializing vital system features such register clocks.
 *
 * Where compatibility can be maintained it initializes all known parts.
 * If the register settings are dependent on the specific chip, it
 * defaults to the chip and configuration on the Discovery board
 * (STM32F100 with 8MHz+32KHz crystals, no battery power.
 */

#if defined(USE_ALT_HEADER_FILES)
/* Prefer stand-alone to avoid change dependencies. */
#include <armduino.h>
typedef unsigned int uint32_t;
#else
#include <ARM-core.h>
#define _MMIO_DWORD(mem_addr) (*(volatile uint32_t *)(mem_addr))
#define APB2ENR		_MMIO_DWORD(0x40021018)	/* Enable peripheral clocks */
#define APB1ENR		_MMIO_DWORD(0x4002101C)
#endif

/* Reset and interrupt vectors start at address 0.
 * There are many vector values, but the only two that are always used
 * are the first two: the reset stack pointer and reset entry point.
 *
 * We put names with weak bindings into the other vectors.
 * The default handler does an immediate return.
 * Note that a robust program may override the default-default handler.
 */
void _start(void);
extern void main(void);

/* This should eventually be defined as a weak constant so that it can
 * be overridden . */
#define STACK_TOP 0x20002000
/* These values are defined by the linker. */
extern int _bss_start, _bss_end;
extern int _initdata_start, _initdata_end;
extern int _initdata_flash;

typedef void _intr_handler(void);
/* A robust program may choose to provide its own default handler.
 * Our default-default counts unhandled interrupt events so that
 * at least the debugger can always see the count. */
int __unhandled_interrupts = 0;
void __unhandled_interrupt(void) __attribute__((weak,interrupt,used));
void __unhandled_interrupt(void) {
	__unhandled_interrupts++;
}

void NMI_Handler(void) __attribute__((weak,interrupt,alias("sysNMI_Handler")));
void HardFault_Handler(void) __attribute__((weak,interrupt,alias("sysMemfault_Handler")));
void MemManage_Handler(void) __attribute__((weak,interrupt,alias("sysMemfault_Handler")));
void sysNMI_Handler(void) __attribute__((interrupt,used));
void sysNMI_Handler(void) {
	__unhandled_interrupts++;
}
void sysMemfault_Handler(void) {
	__unhandled_interrupts++;
}

#define INTR_HAND_ATTRIBUTES \
	__attribute__((weak,interrupt,alias ("__unhandled_interrupt")))

/* These names are the standard ones from ARM.  See STM32F100xx-RM.pdf 8.1.2 */
void Reset_Handler(void) __attribute__((interrupt,alias ("_start")));
void BusFault_Handler(void)  INTR_HAND_ATTRIBUTES;
void UsageFault_Handler(void) INTR_HAND_ATTRIBUTES;
void SVC_Handler(void) INTR_HAND_ATTRIBUTES;
void DebugMon_Handler(void) INTR_HAND_ATTRIBUTES;
void PendSV_Handler(void) INTR_HAND_ATTRIBUTES;
void SysTick_Handler(void) INTR_HAND_ATTRIBUTES;
/* The remainder are STM32-specific names. */
void WWDG_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void PVD_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TAMPER_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void RTC_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void FLASH_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void RCC_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXT0_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXT1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXT2_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXT3_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXT4_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel1(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel2(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel3(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel4(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel5(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel6(void) INTR_HAND_ATTRIBUTES;
void DMA1_Channel7(void) INTR_HAND_ATTRIBUTES;
void ADC1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN1_Tx_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN1_Rx0_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN1_Rx1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN1_SCE_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXTI9_5(void) INTR_HAND_ATTRIBUTES;
void TIM1_BRK_TIM15(void) INTR_HAND_ATTRIBUTES;
void TIM1_UP_TIM16_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM1_TRG_COM_TIM17(void) INTR_HAND_ATTRIBUTES;
void TIM1_CC(void) INTR_HAND_ATTRIBUTES;
void TIM2_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM3_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM4_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void I2C1_EV(void) INTR_HAND_ATTRIBUTES;
void I2C1_ER(void) INTR_HAND_ATTRIBUTES;
void I2C2_EV(void) INTR_HAND_ATTRIBUTES;
void I2C2_ER(void) INTR_HAND_ATTRIBUTES;
void SPI1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void SPI2_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void USART1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void USART2_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void USART3_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void EXTI15_10(void) INTR_HAND_ATTRIBUTES;
void RTC_Alarm(void) INTR_HAND_ATTRIBUTES;
void CEC_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM12_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM13_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM14_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void FSMC_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM5_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void SPI3_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void UART4_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void UART5_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM6_DAC_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void TIM7_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void DMA2_Channel1(void) INTR_HAND_ATTRIBUTES;
void DMA2_Channel2(void) INTR_HAND_ATTRIBUTES;
void DMA2_Channel3(void) INTR_HAND_ATTRIBUTES;
void DMA2_Channel4_5(void) INTR_HAND_ATTRIBUTES;
void DMA2_Channel5(void) INTR_HAND_ATTRIBUTES;
void ISR_ETH(void) INTR_HAND_ATTRIBUTES;
void ISR_ETH_WKUP(void) INTR_HAND_ATTRIBUTES;
void CAN2_Tx_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN2_Rx0_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN2_Rx1_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void CAN2_SCE_IRQHandler(void) INTR_HAND_ATTRIBUTES;
void ISR_OTG_FS(void) INTR_HAND_ATTRIBUTES;

/* Interrupt number used for enabling.  The generic ARM ones are defined
 * in ARM-core.h. */
enum STM_Interrupts	{
	WWDG_INTR=0,
	USART1_INTR=37,	USART2_INTR=38,	USART3_INTR=39,
};

_intr_handler *myvectors[0x56]
__attribute__ ((section("vectors")))= {
  /* The first entries are part of the ARM core and use the standard names. */
  (_intr_handler *)	STACK_TOP,
  Reset_Handler,
  NMI_Handler,
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler,
  UsageFault_Handler,
  0,
  0,0,0,						/* 0x001C..002B */
  SVC_Handler,
  DebugMon_Handler,
  0,
  PendSV_Handler,
  SysTick_Handler,
  /* The remainder are vendor-specific names. */
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_IRQHandler, RTC_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXT0_IRQHandler, EXT1_IRQHandler, EXT2_IRQHandler,
  EXT3_IRQHandler, EXT4_IRQHandler,
  DMA1_Channel1, DMA1_Channel2, DMA1_Channel3,
  DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7,
  ADC1_IRQHandler,
  CAN1_Tx_IRQHandler, CAN1_Rx0_IRQHandler,
  CAN1_Rx1_IRQHandler, CAN1_SCE_IRQHandler,
  EXTI9_5,
  TIM1_BRK_TIM15, TIM1_UP_TIM16_IRQHandler, TIM1_TRG_COM_TIM17, TIM1_CC,
  TIM2_IRQHandler, TIM3_IRQHandler, TIM4_IRQHandler,
  I2C1_EV, I2C1_ER,
  I2C2_EV, I2C2_ER,
  SPI1_IRQHandler, SPI2_IRQHandler,
  USART1_IRQHandler, USART2_IRQHandler, USART3_IRQHandler,
  EXTI15_10,
  RTC_Alarm,
  CEC_IRQHandler,
  TIM12_IRQHandler, TIM13_IRQHandler, TIM14_IRQHandler,
  0,0,							/* 64 */
  FSMC_IRQHandler,
  0,
  TIM5_IRQHandler, SPI3_IRQHandler, UART4_IRQHandler, UART5_IRQHandler,
  TIM6_DAC_IRQHandler, TIM7_IRQHandler,
  DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4_5, DMA2_Channel5,
  ISR_ETH, ISR_ETH_WKUP,								/* 0x00000134 */
  CAN2_Tx_IRQHandler, CAN2_Rx0_IRQHandler,
  CAN2_Rx1_IRQHandler, CAN2_SCE_IRQHandler,
  ISR_OTG_FS,
  0, 							/* 0x0150  */
};

/* The default settings enables the clock to all of the the timers
 * and GPIO ports.
 * This avoids the most common beginner's problem with the STM32 family:
 * not knowing about register clocks.
 * Every application must turn on some clocks.  If the application cares
 * about the trivial extra power use, it can explicitly turn them off.
 * Exactly what it would need to do anyway.  */
const unsigned int __attribute__((weak))
#if 0
_RCC_APB1ENR = APB1ENR_TIM3EN,
_RCC_APB2ENR = APB2ENR_IOPDEN | APB2ENR_IOPCEN | APB2ENR_IOPBEN
	| APB2ENR_IOPAEN | APB2ENR_AFIOEN;
#else
_RCC_APB1ENR = 0x7fffffff,
_RCC_APB2ENR = 0x003fffff;
#endif

/* This routine is the entry point after reset.
 * We configure a few essential registers, then leap into main();
 */
void __attribute__((naked, interrupt, used, noreturn))
_start(void)
{
	int *src, *dst;

	/* Disable interrupts. */
	INTR_CLRENA_BASE[0] = 0xffffffff;
	INTR_CLRENA_BASE[1] = 0xffffffff;
	INTR_CLRENA_BASE[2] = 0xffffffff;
	INTR_CLRENA_BASE[3] = 0xffffffff;
	/* Enable peripheral clocks.  This bumps the power use up slightly,
	 * but developers benefit from having peripherals work by default
	 * rather than mysteriously not respond.
	 * If the program is power sensitive it can override the weak bindings.
	 */
	APB1ENR = _RCC_APB1ENR;
	APB2ENR = _RCC_APB2ENR;
	/* Copy the initialized data from flash to RAM. */
	for (dst = &_initdata_start, src = &_initdata_flash; dst < &_initdata_end; )
		*dst++ = *src++;
	/* Zero the BSS segment in RAM. Overwriting the stack is not fatal yet. */
	for (dst = &_bss_start; dst < &_bss_end; *dst++ = 0)
		;
	/* You would call the constructor list here in a normal application.
	 * However embedded firmware should always explicitly control the
	 * initialization order. */
	main();
	/* Future: call destructor list here.  They
	 * may put external devices into sleep or power-off mode.
	 * But good embedded firmware should explicitly handle sleep and
	 * never exit. */
 infinity: goto infinity;		/* Returning would be bad */
}

/*
 * Local variables:
 *  compile-command: "make crt-stm32.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
