//
// Created by marijn on 5/26/23.
//
#include "main.h"
#include "base.h"
#include "gpio.h"
#include "exti.h"
#include "tim.h"
#include "sys.h"

// software serial config
#define RECEIVE
#ifdef RECEIVE
#define RX_PORT GPIOB
#define RX_PIN	10
#define TX_PORT GPIOB
#define TX_PIN	2
#else
#define RX_PORT GPIOB
#define RX_PIN	2
#define TX_PORT GPIOB
#define TX_PIN	10
#endif
#define DBA_PORT GPIOB
#define DBA_PIN	1
// UART config
//#define BAUD			921600
//#define BAUD			576000
//#define BAUD			460800
//#define BAUD			230400
#define BAUD			115200
//#define BAUD			38400
//#define BAUD			9600


uint32_t tx_psc;
uint32_t rx_psc;
io_buffer_t* rx_buffer;

struct {
	io_buffer_t* buffer;
	// settings
	volatile uint8_t parity : 2;  // none(= 0,1), odd, even
	volatile uint8_t stop_bits: 1;  // 1 + x
	volatile uint8_t data_bits: 4;  // (5 - 9)
	// state
	volatile uint8_t started : 1;
	volatile uint16_t framing_error : 1;
	volatile uint16_t transfer_complete : 1;
	volatile uint16_t data : 10;
	volatile uint16_t cnt : 4;
} rx_state;


// RX
void SUART_start_receive(io_buffer_t* buffer) {
	start_EXTI(RX_PIN);
	start_TIM(TIM10);
	rx_state.buffer = buffer;
	rx_state.started = 0;
	rx_state.cnt = 0;
}
void SUART_stop_receive() {
	start_EXTI(RX_PIN);
	start_TIM(TIM10);
}
#ifdef RECEIVE
extern void EXTI15_10_IRQHandler() {
	EXTI->PR = EXTI_PR_PR10;
#else
extern void EXTI2_IRQHandler() {
	EXTI->PR = EXTI_PR_PR2;
#endif
	GPIO_write(DBA_PORT, DBA_PIN, 1);
	rx_state.transfer_complete = 0;
	rx_state.started = 1;
}
extern void TIM1_UP_TIM10_IRQHandler() {
	TIM10->SR &= ~TIM_SR_UIF;
	if (!rx_state.started) { return; }
	if (rx_state.cnt <= rx_state.data_bits) {
		rx_state.cnt++;
		rx_state.data >>= 1;
		rx_state.data |= GPIO_read(RX_PORT, RX_PIN) << 7;
	} else {
		rx_state.transfer_complete = GPIO_read(RX_PORT, RX_PIN);
		rx_state.framing_error = !rx_state.transfer_complete;
		rx_state.started = rx_state.cnt = 0;  // reset state
		GPIO_write(DBA_PORT, DBA_PIN, 0);
		if (rx_state.framing_error) { return; }
		((uint8_t*)rx_state.buffer->ptr)[rx_state.buffer->i] = rx_state.data;
		rx_state.buffer->i = (rx_state.buffer->i + 1) % rx_state.buffer->size;
	}
}
// TX (up to 921600 baud)
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
	// debug pin outputting 'rx_state.started'
	fconfig_GPIO(DBA_PORT, DBA_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 0);

	// EXTI
	config_EXTI(RX_PIN, RX_PORT, 1, 0);

	// TIM
	tx_psc = (APB2_clock_frequency / (BAUD + 1));
	rx_psc = (APB2_clock_frequency / (BAUD * 2 + 1));
	config_TIM(TIM1, tx_psc, 0xffff);
	start_TIM(TIM1);  // start TX timer
	config_TIM(TIM10, rx_psc, 1);  // RX timer (started when SUART_start_receive is called)
	start_TIM_update_irq(TIM10);  // TIM1_UP_TIM10_IRQHandler

	// software serial
	rx_buffer = new_buffer(100);
	rx_state.parity = 0;
	rx_state.stop_bits = 0;
	rx_state.data_bits = 8;

	// main loop
	#ifdef RECEIVE
	SUART_start_receive(rx_buffer);
	for (;;) {
		if (rx_state.transfer_complete) {
			SUART_write((uint8_t*)(rx_buffer->ptr + rx_buffer->i - 1), 1);
		}
	}
	#else
	const uint8_t* msg = "hello world!\n";
	const uint32_t msg_len = 14;
	for (;;) {
		SUART_write(msg, msg_len);
	}
	#endif

	// | mode | blocking | interrupt |
	// |------|----------|-----------|
	// | TX   | x        | 0         |
	// | RX   | 0        | x         |

	// | baud	| TX | RX |
	// |--------|----|----|
	// | 9600   | x  | x  |
	// | 38400  | x  | x  |
	// | 115200 | x  | x  |
	// | 230400 | x  | -  |  // RX errors start being a problem at this speed
	// | 460800 | x  | 0  |
	// | 576000 | x  | 0  |
	// | 921600 | x  | 0  |
}
