//
// Created by marijn on 5/26/23.
//
#include "usart.h"
#include "main.h"
#include "base.h"
#include "gpio.h"
#include "exti.h"
#include "tim.h"
#include "sys.h"

#include <string.h>

// software serial config
//#define RECEIVE
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
//#define BAUD			115200
//#define BAUD			38400
#define BAUD			9600


uint32_t tx_psc;
uint32_t rx_psc;
io_buffer_t* rx_buffer;

struct {
	uint16_t parity			: 1;	// even (0), odd (1)
	uint16_t parity_enable	: 1;	// send out parity bit if set
	uint16_t dual_stop_bit	: 1;	// enable second stop bit
	uint16_t data_bits		: 5;	// (normally: 5 - 9) (1 is added afterwards)
	uint16_t frame_size		: 8;	// total frame size
} settings;
struct {
	io_buffer_t* buffer;
	// state
	volatile uint32_t data;  // GCC wide-char is 4 bytes
	// transmission state
	volatile uint16_t cnt				: 6;
	volatile uint16_t started			: 1;
	volatile uint16_t rec_parity		: 1;
	volatile uint16_t rec_dual_stop		: 1;
	// frame state
	volatile uint16_t parity			: 1;
	volatile uint16_t framing_error		: 1;
	volatile uint16_t parity_error		: 1;
	uint16_t _							: 4;
} state;
volatile uint64_t dbg_frame;


const uint32_t* msg = L"hello world!\n";
const uint32_t msg_len = 14;


// RX
void SUART_start_receive(io_buffer_t* buffer) {
	start_EXTI(RX_PIN);
	start_TIM(TIM10);
	state.buffer = buffer;
	state.started = 0;
	state.cnt = 0;
}
void SUART_stop_receive() {
	start_EXTI(RX_PIN);
	start_TIM(TIM10);
}
void SUART_transfer_complete(void) {
	dbg_frame = 0;
	if (state.framing_error) { state.framing_error = 0; return; }  // TODO: handle fe
	if (state.parity_error) { state.parity_error = 0; return; }  // TODO: handle pe
	for (uint8_t i = 0; i < 4; i++) {  // store wide char in buffer
		((uint8_t*)state.buffer->ptr)[state.buffer->i] = ((uint8_t*)&state.data)[i];
		state.buffer->i = (state.buffer->i + 1) % state.buffer->size;
	}
}
#ifdef RECEIVE
extern void EXTI15_10_IRQHandler() {
	EXTI->PR = EXTI_PR_PR10;
#else
extern void EXTI2_IRQHandler() {
	EXTI->PR = EXTI_PR_PR2;
#endif
	GPIO_write(DBA_PORT, DBA_PIN, 1);
	state.started = 1;
}
extern void TIM1_UP_TIM10_IRQHandler() {
	// TODO: capture entire frame and process it later
	// (or try to do it the old way which is more memory efficient)
	TIM10->SR &= ~TIM_SR_UIF;
	dbg_frame = (dbg_frame << 1) | GPIO_read(RX_PORT, RX_PIN);
	if (!state.started) { return; }
	if (state.cnt <= (settings.data_bits + 1)) {
		if (!state.cnt) { state.parity = settings.parity; }  // reset parity before starting
		state.cnt++; state.data >>= 1;
		uint8_t d = GPIO_read(RX_PORT, RX_PIN);
		state.data |= d << settings.data_bits;
		state.parity ^= d;
	} else if (settings.parity_enable && !state.rec_parity) {
		state.parity_error = state.parity != GPIO_read(RX_PORT, RX_PIN);
		state.rec_parity = 1;
	} else if (settings.dual_stop_bit && !state.rec_dual_stop) {
		state.framing_error = !GPIO_read(RX_PORT, RX_PIN);
		state.rec_dual_stop = 1;
	} else {
		// reset transmission state variables
		state.framing_error |= !GPIO_read(RX_PORT, RX_PIN);
		state.started = state.cnt = 0;
		state.rec_parity = state.rec_dual_stop = 0;

		GPIO_write(DBA_PORT, DBA_PIN, 0);  // TODO: remove
		SUART_transfer_complete();
	}
}
// TX (up to 921600 baud)
void SUART_write(const uint32_t* buffer, uint32_t size) {
	uint16_t uart_tick = TIM1->CNT; uint8_t j, d, p;
	for (uint32_t i = 0; i < size; i++) {
		p = settings.parity;  // reset parity
		while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
		GPIO_write(TX_PORT, TX_PIN, 0);  // start bit
		for (j = 0; j < settings.data_bits + 1; j++) {
			while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
			d = (buffer[i] >> j) & 0b1u; p ^= d;
			GPIO_write(TX_PORT, TX_PIN, d & 0b1u);
		}
		if (settings.parity_enable) {
			while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
			GPIO_write(TX_PORT, TX_PIN, p);  // parity
		}
		while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
		GPIO_write(TX_PORT, TX_PIN, 1);  // stop bit
		if (settings.dual_stop_bit) {
			while (uart_tick == TIM1->CNT); uart_tick = TIM1->CNT;
			GPIO_write(TX_PORT, TX_PIN, 1);  // dual stop bit
		}
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
	rx_buffer = new_buffer(100 * sizeof(wchar_t));
	settings.parity = 0;
	settings.parity_enable = 0;
	settings.dual_stop_bit = 0;
	settings.data_bits = 7;  // 8 data bits
	settings.frame_size = settings.data_bits + 1 + settings.dual_stop_bit + 1 + settings.parity_enable;

	// main loop
	#ifdef RECEIVE
	SUART_start_receive(rx_buffer);
	const uint8_t* menu =
			"Options\n\r"\
		" - D: display digital pins A0 - B15\n\r"\
		" - S: display the system clock-speed\n\r"\
		" - C: clear screen and exit\n\r";

	const uint8_t* hex = "0123456789ABCDEF";
	const uint8_t* dec = "0123456789";

	const uint8_t* digital_message = "pins A0-B15: ";
	const uint8_t* speed_message = "system clock speed: ";
	const uint8_t* invalid_message = "invalid option\n\r";
	uint8_t* str_buffer = malloc(150);
	uint8_t len;

	uint32_t freq;
	for (;;) {
		if (state.transfer_complete) {
			memset(str_buffer, 0x00, 150);
			switch (*((char*)(rx_buffer->ptr + rx_buffer->i - 1))) {
				case 'D':
					len = strlen(digital_message);
					memcpy(str_buffer, digital_message, len);
					for (uint8_t i = 0; i < 8; i++) { str_buffer[len] = hex[GPIOA->ODR >> (28 - (i << 2)) & 0xful]; len++; }
					str_buffer[len] = ' '; len++;
					for (uint8_t i = 0; i < 8; i++) { str_buffer[len] = hex[GPIOB->ODR >> (28 - (i << 2)) & 0xful]; len++; }
					str_buffer[len] = '\n'; len++; str_buffer[len] = '\r';
					SUART_write(str_buffer, strlen(str_buffer));
					break;
				case 'S':
					len = strlen(speed_message);
					memcpy(str_buffer, speed_message, len);
					freq = SYS_clock_frequency;
					while (freq) { str_buffer[len] = dec[freq % 10]; len++; freq /= 10;	}
					str_buffer[len] = '\n'; len++; str_buffer[len] = '\r';
					SUART_write(str_buffer, strlen(str_buffer));
					break;
				case 'C':
					break;
				default:
					SUART_write(menu, strlen(menu));
					break;
			}
			//SUART_write((uint8_t*)(rx_buffer->ptr + rx_buffer->i - 1), 1);
			state.transfer_complete = 0;
		}
	}
	#else
	SUART_start_receive(rx_buffer);
	for (;;) {
		//SUART_write(msg, 1);  // only send 'h'
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