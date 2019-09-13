/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Provides main entry point.  Initialise subsystems and enter GDB
 * protocol loop.
 */

#include "general.h"


#include <libopencm3/stm32/timer.h>

uint8_t my_buf_rx1 = 'U';
uint8_t my_buf_rx2 = 'Y';
uint8_t my_buf_rx3 = 'W';

void usbuart_run(int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx, int CDCACM_UART_ENDPOINT);


int
main(int argc, char **argv)
{
#if defined(LIBFTDI)
	platform_init(argc, argv);
#else
	(void) argc;
	(void) argv;
	platform_init();
#endif

    for (int i = 0; i < 50000000; ++i)  asm("nop");

	while (true) {
//         for (int i = 0; i < 1000000; ++i)  asm("nop");
//         usbuart_run(TIM2, NULL, NULL, &my_buf_rx1, 5);

        for (int i = 0; i < 1000000; ++i)  asm("nop");
        usbuart_run(TIM3, NULL, NULL, &my_buf_rx2, 1);

//         for (int i = 0; i < 1000000; ++i)  asm("nop");
//         usbuart_run(TIM4, NULL, NULL, &my_buf_rx3, 3);
	}

	/* Should never get here */
	return 0;
}

