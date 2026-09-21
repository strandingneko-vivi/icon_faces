/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


int main(void)
{	
	while (1) {
		uint64_t now_ticks = z_nrf_grtc_timer_read();
		uint64_t now_sec   = now_ticks / sys_clock_hw_cycles_per_sec();

		k_sleep(K_MSEC(1000));
	}

	return 0;
}
