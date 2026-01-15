/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Hardware Abstraction Layer - Zephyr Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include "ads1293_hal_zephyr.h"
#include "ads1293_registers.h"

LOG_MODULE_REGISTER(ads1293_hal, CONFIG_SENSOR_LOG_LEVEL);


/* ============================================================================
 * SPI Operations
 * ============================================================================ */

int ads1293_hal_reg_read(ads1293_hal_ctx_t *const context,
                         const uint8_t register_address,
                         uint8_t *const value)
{
	if (!context || !context->spi || !value)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t transmit_buffer[2] = { register_address | ADS1293_SPI_READ_FLAG, 0x00 };
	uint8_t receive_buffer[2] = { 0xFF, 0xFF };  /* Pre-fill to detect if SPI actually writes */

	const struct spi_buf transmit_descriptors[] = {
		{ .buf = transmit_buffer, .len = 2 }
	};
	const struct spi_buf receive_descriptors[] = {
		{ .buf = receive_buffer, .len = 2 }
	};
	const struct spi_buf_set transmit_set = { .buffers = transmit_descriptors, .count = 1 };
	const struct spi_buf_set receive_set = { .buffers = receive_descriptors, .count = 1 };

	const int spi_result = spi_transceive_dt(context->spi, &transmit_set, &receive_set);
	if (spi_result < 0)
	{
		LOG_ERR("SPI read failed (reg 0x%02X): %d", register_address, spi_result);
		return ADS1293_ERR_IO;
	}

	*value = receive_buffer[1];
	return ADS1293_OK;
}

int ads1293_hal_reg_write(ads1293_hal_ctx_t *const context,
                          const uint8_t register_address,
                          const uint8_t value)
{
	if (!context || !context->spi)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t transmit_buffer[2] = { register_address & ADS1293_SPI_ADDR_MASK, value };

	const struct spi_buf transmit_descriptors[] = {
		{ .buf = transmit_buffer, .len = 2 }
	};
	const struct spi_buf_set transmit_set = { .buffers = transmit_descriptors, .count = 1 };

	const int spi_result = spi_write_dt(context->spi, &transmit_set);
	if (spi_result < 0)
	{
		LOG_ERR("SPI write failed (reg 0x%02X): %d", register_address, spi_result);
		return ADS1293_ERR_IO;
	}

	return ADS1293_OK;
}

int ads1293_hal_reg_read_burst(ads1293_hal_ctx_t *const context,
                               const uint8_t register_address,
                               uint8_t *const buffer,
                               const size_t length)
{
	if (!context || !context->spi || !buffer || length == 0)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t address_byte = register_address | ADS1293_SPI_READ_FLAG;

	const struct spi_buf transmit_descriptors[] = {
		{ .buf = &address_byte, .len = 1 },
		{ .buf = NULL, .len = length }
	};
	const struct spi_buf receive_descriptors[] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = buffer, .len = length }
	};
	const struct spi_buf_set transmit_set = { .buffers = transmit_descriptors, .count = 2 };
	const struct spi_buf_set receive_set = { .buffers = receive_descriptors, .count = 2 };

	const int spi_result = spi_transceive_dt(context->spi, &transmit_set, &receive_set);
	if (spi_result < 0)
	{
		LOG_ERR("SPI burst read failed (reg 0x%02X, len %zu): %d",
			register_address, length, spi_result);
		return ADS1293_ERR_IO;
	}

	return ADS1293_OK;
}

int ads1293_hal_reg_write_burst(ads1293_hal_ctx_t *const context,
                                const uint8_t register_address,
                                const uint8_t *const buffer,
                                const size_t length)
{
	if (!context || !context->spi || !buffer || length == 0)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t address_byte = register_address & ADS1293_SPI_ADDR_MASK;

	const struct spi_buf transmit_descriptors[] = {
		{ .buf = &address_byte, .len = 1 },
		{ .buf = (void *)buffer, .len = length }
	};
	const struct spi_buf_set transmit_set = { .buffers = transmit_descriptors, .count = 2 };

	const int spi_result = spi_write_dt(context->spi, &transmit_set);
	if (spi_result < 0)
	{
		LOG_ERR("SPI burst write failed (reg 0x%02X, len %zu): %d",
			register_address, length, spi_result);
		return ADS1293_ERR_IO;
	}

	return ADS1293_OK;
}


/* ============================================================================
 * GPIO Operations - Reset
 * ============================================================================ */

bool ads1293_hal_has_reset(ads1293_hal_ctx_t *const context)
{
	return context && context->reset_gpio && context->reset_gpio->port;
}

int ads1293_hal_reset_assert(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_reset(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Active low reset - assert by driving low (set to 1 with active-low flag) */
	const int gpio_result = gpio_pin_set_dt(context->reset_gpio, 1);
	if (gpio_result < 0)
	{
		LOG_ERR("Failed to assert reset: %d", gpio_result);
		return ADS1293_ERR_IO;
	}

	LOG_DBG("Reset asserted (pin should be LOW)");
	return ADS1293_OK;
}

int ads1293_hal_reset_release(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_reset(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Release reset by driving high (set to 0 with active-low flag) */
	const int gpio_result = gpio_pin_set_dt(context->reset_gpio, 0);
	if (gpio_result < 0)
	{
		LOG_ERR("Failed to release reset: %d", gpio_result);
		return ADS1293_ERR_IO;
	}

	/* Read back the pin to verify it's actually high */
	int pin_val = gpio_pin_get_dt(context->reset_gpio);
	LOG_DBG("Reset released (pin should be HIGH, reads: %d)", pin_val);
	return ADS1293_OK;
}


/* ============================================================================
 * GPIO Operations - DRDY
 * ============================================================================ */

bool ads1293_hal_has_drdy(ads1293_hal_ctx_t *const context)
{
	return context && context->drdy_gpio && context->drdy_gpio->port;
}

int ads1293_hal_drdy_read(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_drdy(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	return gpio_pin_get_dt(context->drdy_gpio);
}

static void drdy_gpio_callback(const struct device *const port,
                               struct gpio_callback *const callback_data,
                               const gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct ads1293_hal_ctx *const context =
		CONTAINER_OF(callback_data, struct ads1293_hal_ctx, drdy_callback_data);

	if (context->drdy_callback)
	{
		context->drdy_callback(context->drdy_user_data);
	}
}

int ads1293_hal_drdy_int_enable(ads1293_hal_ctx_t *const context,
                                const ads1293_drdy_callback_t callback,
                                void *const user_data)
{
#if !IS_ENABLED(CONFIG_ADS1293_DRDY_IRQ_ENABLE)
	LOG_WRN("DRDY IRQ support disabled via CONFIG_ADS1293_DRDY_IRQ_ENABLE");
	return ADS1293_ERR_INVALID_PARAM;
#else
	if (!ads1293_hal_has_drdy(context))
	{
		LOG_ERR("DRDY GPIO not configured");
		return ADS1293_ERR_INVALID_PARAM;
	}

	context->drdy_callback = callback;
	context->drdy_user_data = user_data;

	gpio_init_callback(&context->drdy_callback_data, drdy_gpio_callback,
	                   BIT(context->drdy_gpio->pin));

	int result = gpio_add_callback(context->drdy_gpio->port, &context->drdy_callback_data);
	if (result < 0)
	{
		LOG_ERR("Failed to add DRDY callback: %d", result);
		return ADS1293_ERR_IO;
	}

	result = gpio_pin_interrupt_configure_dt(context->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (result < 0)
	{
		LOG_ERR("Failed to configure DRDY interrupt: %d", result);
		gpio_remove_callback(context->drdy_gpio->port, &context->drdy_callback_data);
		return ADS1293_ERR_IO;
	}

	LOG_DBG("DRDY interrupt enabled on GPIO port %s pin %u (flags=0x%x)",
		context->drdy_gpio->port->name,
		context->drdy_gpio->pin,
		context->drdy_gpio->dt_flags);

	return ADS1293_OK;
#endif /* CONFIG_ADS1293_DRDY_IRQ_ENABLE */
}

int ads1293_hal_drdy_int_disable(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_drdy(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	gpio_pin_interrupt_configure_dt(context->drdy_gpio, GPIO_INT_DISABLE);
	gpio_remove_callback(context->drdy_gpio->port, &context->drdy_callback_data);

	context->drdy_callback = NULL;
	context->drdy_user_data = NULL;

	return ADS1293_OK;
}


/* ============================================================================
 * GPIO Operations - ALARM
 * ============================================================================ */

bool ads1293_hal_has_alarm(ads1293_hal_ctx_t *const context)
{
	return context && context->alarm_gpio && context->alarm_gpio->port;
}

int ads1293_hal_alarm_read(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_alarm(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	return gpio_pin_get_dt(context->alarm_gpio);
}

static void alarm_gpio_callback(const struct device *const port,
                                struct gpio_callback *const callback_data,
                                const gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct ads1293_hal_ctx *const context =
		CONTAINER_OF(callback_data, struct ads1293_hal_ctx, alarm_callback_data);

	if (context->alarm_callback)
	{
		/* Read error status and pass to callback */
		/* Note: In ISR context, we just pass 0; let app read status */
		context->alarm_callback(0, context->alarm_user_data);
	}
}

int ads1293_hal_alarm_int_enable(ads1293_hal_ctx_t *const context,
                                 const ads1293_alarm_callback_t callback,
                                 void *const user_data)
{
#if !IS_ENABLED(CONFIG_ADS1293_ALARM_IRQ_ENABLE)
	LOG_WRN("ALARM IRQ support disabled via CONFIG_ADS1293_ALARM_IRQ_ENABLE");
	return ADS1293_ERR_INVALID_PARAM;
#else
	if (!ads1293_hal_has_alarm(context))
	{
		LOG_ERR("ALARM GPIO not configured");
		return ADS1293_ERR_INVALID_PARAM;
	}

	context->alarm_callback = callback;
	context->alarm_user_data = user_data;

	gpio_init_callback(&context->alarm_callback_data, alarm_gpio_callback,
	                   BIT(context->alarm_gpio->pin));

	int result = gpio_add_callback(context->alarm_gpio->port, &context->alarm_callback_data);
	if (result < 0)
	{
		LOG_ERR("Failed to add ALARM callback: %d", result);
		return ADS1293_ERR_IO;
	}

	result = gpio_pin_interrupt_configure_dt(context->alarm_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (result < 0)
	{
		LOG_ERR("Failed to configure ALARM interrupt: %d", result);
		gpio_remove_callback(context->alarm_gpio->port, &context->alarm_callback_data);
		return ADS1293_ERR_IO;
	}

	LOG_DBG("ALARM interrupt enabled");
	return ADS1293_OK;
#endif /* CONFIG_ADS1293_ALARM_IRQ_ENABLE */
}

int ads1293_hal_alarm_int_disable(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_alarm(context))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	gpio_pin_interrupt_configure_dt(context->alarm_gpio, GPIO_INT_DISABLE);
	gpio_remove_callback(context->alarm_gpio->port, &context->alarm_callback_data);

	context->alarm_callback = NULL;
	context->alarm_user_data = NULL;

	return ADS1293_OK;
}


/* ============================================================================
 * Clock Operations - CLKIN PWM
 * ============================================================================ */

bool ads1293_hal_has_clkin(ads1293_hal_ctx_t *const context)
{
	return context && context->clkin_pwm && context->clkin_pwm->dev;
}

int ads1293_hal_clkin_start(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_clkin(context))
	{
		/* No CLKIN configured - assuming external crystal */
		return ADS1293_OK;
	}

	if (context->clkin_running)
	{
		/* Already running */
		return ADS1293_OK;
	}

	/* Set PWM to 50% duty cycle at the configured frequency */
	const uint32_t period_ns = context->clkin_period_ns;
	const uint32_t pulse_ns = period_ns / 2;

	LOG_DBG("Starting CLKIN PWM: period=%u ns, pulse=%u ns (50%% duty), freq=%u Hz",
		period_ns, pulse_ns, (uint32_t)(1000000000ULL / period_ns));
	LOG_DBG("  PWM device=%s, channel=%u", context->clkin_pwm->dev->name,
		context->clkin_pwm->channel);

	const int pwm_result = pwm_set_dt(context->clkin_pwm, period_ns, pulse_ns);
	if (pwm_result < 0)
	{
		LOG_ERR("Failed to start CLKIN PWM: %d", pwm_result);
		return ADS1293_ERR_IO;
	}

	context->clkin_running = true;
	LOG_DBG("CLKIN PWM started successfully (result=%d)", pwm_result);

	return ADS1293_OK;
}

int ads1293_hal_clkin_stop(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_clkin(context))
	{
		return ADS1293_OK;
	}

	if (!context->clkin_running)
	{
		return ADS1293_OK;
	}

	/* Stop PWM by setting pulse width to 0 */
	const int pwm_result = pwm_set_dt(context->clkin_pwm, context->clkin_period_ns, 0);
	if (pwm_result < 0)
	{
		LOG_ERR("Failed to stop CLKIN PWM: %d", pwm_result);
		return ADS1293_ERR_IO;
	}

	context->clkin_running = false;
	LOG_DBG("CLKIN stopped");

	return ADS1293_OK;
}

uint32_t ads1293_hal_get_clkin_frequency(ads1293_hal_ctx_t *const context)
{
	if (!ads1293_hal_has_clkin(context) || context->clkin_period_ns == 0)
	{
		/* Using external crystal - return 0 to indicate no MCU clock */
		return 0;
	}

	/* Convert period in nanoseconds to frequency in Hz */
	return (uint32_t)(1000000000ULL / context->clkin_period_ns);
}


/* ============================================================================
 * Timing Operations
 * ============================================================================ */

void ads1293_hal_delay_us(ads1293_hal_ctx_t *const context, const uint32_t microseconds)
{
	ARG_UNUSED(context);
	k_busy_wait(microseconds);
}

void ads1293_hal_delay_ms(ads1293_hal_ctx_t *const context, const uint32_t milliseconds)
{
	ARG_UNUSED(context);
	k_msleep(milliseconds);
}

uint32_t ads1293_hal_get_time_ms(ads1293_hal_ctx_t *const context)
{
	ARG_UNUSED(context);
	return k_uptime_get_32();
}


/* ============================================================================
 * HAL Lifecycle
 * ============================================================================ */

int ads1293_hal_init(ads1293_hal_ctx_t *const context)
{
	if (!context)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Verify SPI is ready */
	if (!context->spi || !spi_is_ready_dt(context->spi))
	{
		LOG_ERR("SPI device not ready");
		return ADS1293_ERR_IO;
	}

	/* Configure reset GPIO if present */
	if (ads1293_hal_has_reset(context))
	{
		if (!gpio_is_ready_dt(context->reset_gpio))
		{
			LOG_ERR("Reset GPIO not ready");
			return ADS1293_ERR_IO;
		}

		const int configure_result = gpio_pin_configure_dt(context->reset_gpio,
		                                                   GPIO_OUTPUT_INACTIVE);
		if (configure_result < 0)
		{
			LOG_ERR("Failed to configure reset GPIO: %d", configure_result);
			return ADS1293_ERR_IO;
		}
	}

	/* Configure DRDY GPIO if present */
	if (ads1293_hal_has_drdy(context))
	{
		if (!gpio_is_ready_dt(context->drdy_gpio))
		{
			LOG_ERR("DRDY GPIO not ready");
			return ADS1293_ERR_IO;
		}

		const int configure_result = gpio_pin_configure_dt(context->drdy_gpio, GPIO_INPUT);
		if (configure_result < 0)
		{
			LOG_ERR("Failed to configure DRDY GPIO: %d", configure_result);
			return ADS1293_ERR_IO;
		}
	}

	/* Configure ALARM GPIO if present */
	if (ads1293_hal_has_alarm(context))
	{
		if (!gpio_is_ready_dt(context->alarm_gpio))
		{
			LOG_ERR("ALARM GPIO not ready");
			return ADS1293_ERR_IO;
		}

		const int configure_result = gpio_pin_configure_dt(context->alarm_gpio, GPIO_INPUT);
		if (configure_result < 0)
		{
			LOG_ERR("Failed to configure ALARM GPIO: %d", configure_result);
			return ADS1293_ERR_IO;
		}
	}

	/* Configure CLKIN PWM if present */
	if (ads1293_hal_has_clkin(context))
	{
		if (!pwm_is_ready_dt(context->clkin_pwm))
		{
			LOG_ERR("CLKIN PWM device not ready");
			return ADS1293_ERR_IO;
		}

		/* Store the period from device tree for later use */
		context->clkin_period_ns = context->clkin_pwm->period;
		context->clkin_running = false;

		const uint32_t frequency_hz = ads1293_hal_get_clkin_frequency(context);
		LOG_DBG("CLKIN PWM configured: %u Hz", frequency_hz);

		/* Validate frequency is within ADS1293 acceptable range */
		if (frequency_hz < 128000 || frequency_hz > 512000)
		{
			LOG_WRN("CLKIN frequency %u Hz outside recommended range (128-512 kHz)",
			        frequency_hz);
		}
	}
	else
	{
		LOG_DBG("Using external crystal for clock source");
	}

	context->initialized = true;
	LOG_DBG("HAL initialized");

	return ADS1293_OK;
}

int ads1293_hal_deinit(ads1293_hal_ctx_t *const context)
{
	if (!context)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Stop CLKIN if running */
	if (ads1293_hal_has_clkin(context))
	{
		ads1293_hal_clkin_stop(context);
	}

	/* Disable interrupts */
	if (ads1293_hal_has_drdy(context))
	{
		ads1293_hal_drdy_int_disable(context);
	}

	if (ads1293_hal_has_alarm(context))
	{
		ads1293_hal_alarm_int_disable(context);
	}

	context->initialized = false;

	return ADS1293_OK;
}
