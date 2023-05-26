//
// Created by marijn on 5/26/23.
//
#include "main.h"
#include "gpio.h"
#include "exti.h"
#include "tim.h"
#include "sys.h"


// GPIO config
#define RX_PORT GPIOA
#define RX_PIN	0
#define TX_PORT GPIOA
#define TX_PIN	1
// UART config
#define BAUD			9600
#define OVERSAMPLING	16
#define DATA_BITS		8


uint8_t started = 0;
uint8_t transmission = 0;
uint16_t sample = 0;
uint16_t sampled = 0;
uint8_t rx_bits = 0;
uint8_t rx = 0;

uint8_t* buffer;
uint8_t* ptr;

// RX
extern void TIM1_UP_TIM10_IRQHandler(void) {
	TIM1->SR &= ~TIM_SR_UIF;
	sample <<= 1;
	sample |= GPIO_read(RX_PORT, RX_PIN);
	if (++sampled == OVERSAMPLING) {
		sampled = 0;
		register uint8_t bit = (sample >> 7 & 0b111);
		// if (bit != 0b111 && bit) { /* TODO: fix bit */ }
		bit = bit == 0b111;
		if (!transmission && !bit)			{ transmission = 1; return; }
		if (rx_bits == DATA_BITS && bit)	{  // TODO: multiple stop bits
			stop_TIM(TIM1); started = 0;
			*ptr++ = rx; return;
		} rx = ((rx << 1) | bit);
	}
}
extern void EXTI0_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR0;
	if (!started) {
		transmission = 0;
		sampled = 0;
		rx_bits = 0;
		started = 1;
		start_TIM(TIM1);
	}
}
// TX
extern void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
}


int main(void) {
	// SYS_CLOCK (100Mhz)
	SYS_CLK_Config_t* sys_config = new_SYS_CLK_config();
	set_SYS_PLL_config(sys_config, 15, 120, PLL_P_DIV2, 0, PLL_SRC_HSE);
	set_SYS_CLOCK_config(sys_config, SYS_CLK_SRC_PLL, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV, 0);
	set_SYS_FLASH_config(sys_config, FLASH_LATENCY4, 1, 1, 1);  // latency is set automatically (when need be)
	set_SYS_tick_config(sys_config, 0, 0, NULL);
	sys_clock_init(sys_config); free(sys_config);

	// GPIO
	fconfig_GPIO(RX_PORT, RX_PIN, GPIO_input, GPIO_pull_up, GPIO_open_drain, GPIO_very_high_speed, 0);
	fconfig_GPIO(TX_PORT, TX_PIN, GPIO_output, GPIO_pull_up, GPIO_open_drain, GPIO_very_high_speed, 0);

	// EXTI
	config_EXTI(RX_PIN, RX_PORT, 1, 0);

	// TIM
	uint32_t rx_psc = (100000000 / (BAUD * OVERSAMPLING - 1));
	uint32_t tx_psc = (50000000 / (BAUD - 1));
	config_TIM(TIM1, rx_psc, 1);
	config_TIM(TIM2, tx_psc, 1);
	start_TIM_update_irq(TIM1);
	start_TIM_update_irq(TIM2);

	// software serial
	buffer =	malloc(100);
	ptr =		buffer;
	start_EXTI(RX_PIN);  // start software serial

	// main loop
	for (;;) {
		__NOP();
	}

	// | baud	| OS  | sample_freq	|
	// |--------|-----|-------------|
	// | 9600	| x16 | 153600		|
	// | 38400	| x16 | 614400		|
	// | 115200	| x16 | 1843200		|
	// | 230400	| x8  | 1843200		|
	// | 460800	| x8  | 3686400		|
	// | 576000	| x8  | 4608000		|
	// | 921600	| x8  | 7372800		|
}
