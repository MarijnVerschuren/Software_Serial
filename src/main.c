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
#define RX_PIN	5
#define TX_PORT GPIOA
#define TX_PIN	6
#define DBA_PORT GPIOA
#define DBA_PIN	4
#define DBB_PORT GPIOA
#define DBB_PIN	3
// UART config
//#define BAUD			460800
//#define BAUD			115200
#define BAUD			9600
#define OVERSAMPLING	16
#define DATA_BITS		8


uint8_t* rx_buffer;
uint8_t* rx_ptr;

// RX
void SUART_read_until(uint8_t** buffer, uint32_t size, uint8_t until) {
	uint16_t uart_tick;
	uint8_t j, rx = 0;
	for (uint32_t i = 0; i < size; i++) {
		while (GPIO_read(RX_PORT, RX_PIN)); uart_tick = TIM1->CNT;  // wait until start bit
		GPIO_write(DBA_PORT, DBA_PIN, 1);
		for (j = 0; j < 8; j++) {
			while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
			rx = (rx >> 1) | (GPIO_read(RX_PORT, RX_PIN) << 7);
			GPIO_toggle(DBB_PORT, DBB_PIN);
		}
		while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
		GPIO_write(DBA_PORT, DBA_PIN, 0);
		if (GPIO_read(RX_PORT, RX_PIN)) {
			*(*buffer)++ = rx;
			if (rx == until) { return; }
		}
	}
}
// TX
void SUART_write(const uint8_t* buffer, uint32_t size) {
	uint16_t uart_tick = TIM1->CNT; uint8_t j;
	for (uint32_t i = 0; i < size; i++) {
		while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
		GPIO_write(TX_PORT, TX_PIN, 0);  // start bit
		for (j = 0; j < 8; j++) {
			while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
			GPIO_write(TX_PORT, TX_PIN, (buffer[i] >> j) & 0b1);
		}
		while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
		GPIO_write(TX_PORT, TX_PIN, 1);  // stop bit
		while (TIM1->CNT - uart_tick < 3); uart_tick = TIM1->CNT;
	}
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
	fconfig_GPIO(RX_PORT, RX_PIN, GPIO_input, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 0);
	fconfig_GPIO(TX_PORT, TX_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 0);
	fconfig_GPIO(DBA_PORT, DBA_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 0);
	fconfig_GPIO(DBB_PORT, DBB_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 0);
	GPIO_write(TX_PORT, TX_PIN, 1);  // let the line be floating

	// EXTI
	config_EXTI(RX_PIN, RX_PORT, 1, 1);

	// TIM
	uint32_t rx_psc = (APB2_clock_frequency / (BAUD + 1));
	// cycles_per_bit = (uint32_t)(F_CPU + speed / 2) / speed;
	// https://www.deeptronic.com/software-design/writing-simple-software-serial-function-in-arduino/
	config_TIM(TIM1, rx_psc, 0xffff);
	start_TIM(TIM1);

	// software serial
	rx_buffer =	calloc(100, 1);
	rx_ptr =	rx_buffer;

	// main loop
	const uint8_t* message = "hello world!\n";
	uint8_t message_length = 13;
	for (;;) {
		//while (tick - start < 10); start = tick;
		SUART_write(message, message_length);
		//SUART_read_until(&rx_ptr, 100, 0x00);
		//rx_ptr = rx_buffer;
		//__NOP();  // 891a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121a0a121afc
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
