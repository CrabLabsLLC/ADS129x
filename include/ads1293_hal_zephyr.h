/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Hardware Abstraction Layer - Zephyr Implementation Header
 *
 * This header provides the Zephyr-specific HAL context structure definition
 * for use by the driver when compiled for Zephyr.
 */

#ifndef ADS1293_HAL_ZEPHYR_H_
#define ADS1293_HAL_ZEPHYR_H_

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "ads1293_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Zephyr-specific HAL context structure
 *
 * This structure contains all platform-specific handles and state
 * needed for the ADS1293 driver on Zephyr RTOS.
 */
struct ads1293_hal_ctx
{
	/* SPI device */
	const struct spi_dt_spec *spi;

	/* GPIO pins */
	const struct gpio_dt_spec *drdy_gpio;
	const struct gpio_dt_spec *alarm_gpio;
	const struct gpio_dt_spec *reset_gpio;

	/* CLKIN PWM (for MCU-provided clock) */
	const struct pwm_dt_spec *clkin_pwm;
	uint32_t clkin_period_ns;
	bool clkin_running;

	/* Interrupt callbacks */
	struct gpio_callback drdy_callback_data;
	struct gpio_callback alarm_callback_data;
	ads1293_drdy_callback_t drdy_callback;
	void *drdy_user_data;
	ads1293_alarm_callback_t alarm_callback;
	void *alarm_user_data;

	/* Initialization state */
	bool initialized;
};

#ifdef __cplusplus
}
#endif

#endif /* ADS1293_HAL_ZEPHYR_H_ */
