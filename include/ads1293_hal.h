/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Hardware Abstraction Layer
 *
 * Platform-independent interface for hardware operations.
 * This HAL abstracts SPI, GPIO, and timing operations to allow
 * the driver to be portable across different platforms.
 */

#ifndef ADS1293_HAL_H_
#define ADS1293_HAL_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ads1293_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * HAL Context (Opaque Handle)
 * ============================================================================ */

/**
 * @brief Opaque HAL context handle
 *
 * Platform-specific implementation provides the actual structure.
 * Users interact through this opaque pointer only.
 */
typedef struct ads1293_hal_ctx ads1293_hal_ctx_t;


/* ============================================================================
 * SPI Operations
 * ============================================================================ */

/**
 * @brief Read a single register
 *
 * @param ctx HAL context handle
 * @param reg Register address (0x00 - 0x50)
 * @param val Pointer to store read value
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reg_read(ads1293_hal_ctx_t *ctx, uint8_t reg, uint8_t *val);

/**
 * @brief Write a single register
 *
 * @param ctx HAL context handle
 * @param reg Register address (0x00 - 0x50)
 * @param val Value to write
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reg_write(ads1293_hal_ctx_t *ctx, uint8_t reg, uint8_t val);

/**
 * @brief Read multiple consecutive registers (burst read)
 *
 * @param ctx HAL context handle
 * @param reg Starting register address
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reg_read_burst(ads1293_hal_ctx_t *ctx, uint8_t reg,
			       uint8_t *buf, size_t len);

/**
 * @brief Write multiple consecutive registers (burst write)
 *
 * @param ctx HAL context handle
 * @param reg Starting register address
 * @param buf Data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reg_write_burst(ads1293_hal_ctx_t *ctx, uint8_t reg,
				const uint8_t *buf, size_t len);


/* ============================================================================
 * GPIO Operations
 * ============================================================================ */

/**
 * @brief Assert hardware reset (pull low)
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reset_assert(ads1293_hal_ctx_t *ctx);

/**
 * @brief Release hardware reset (return high)
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_reset_release(ads1293_hal_ctx_t *ctx);

/**
 * @brief Check if reset GPIO is available
 *
 * @param ctx HAL context handle
 * @return true if reset GPIO is configured
 */
bool ads1293_hal_has_reset(ads1293_hal_ctx_t *ctx);

/**
 * @brief Read DRDY pin state
 *
 * @param ctx HAL context handle
 * @return 1 if high, 0 if low, negative on error
 */
int ads1293_hal_drdy_read(ads1293_hal_ctx_t *ctx);

/**
 * @brief Check if DRDY GPIO is available
 *
 * @param ctx HAL context handle
 * @return true if DRDY GPIO is configured
 */
bool ads1293_hal_has_drdy(ads1293_hal_ctx_t *ctx);

/**
 * @brief Enable DRDY interrupt
 *
 * @param ctx HAL context handle
 * @param callback Function to call on DRDY falling edge
 * @param user_data Context passed to callback
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_drdy_int_enable(ads1293_hal_ctx_t *ctx,
				ads1293_drdy_callback_t callback,
				void *user_data);

/**
 * @brief Disable DRDY interrupt
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_drdy_int_disable(ads1293_hal_ctx_t *ctx);

/**
 * @brief Read ALARM pin state
 *
 * @param ctx HAL context handle
 * @return 1 if high, 0 if low, negative on error
 */
int ads1293_hal_alarm_read(ads1293_hal_ctx_t *ctx);

/**
 * @brief Check if ALARM GPIO is available
 *
 * @param ctx HAL context handle
 * @return true if ALARM GPIO is configured
 */
bool ads1293_hal_has_alarm(ads1293_hal_ctx_t *ctx);

/**
 * @brief Enable ALARM interrupt
 *
 * @param ctx HAL context handle
 * @param callback Function to call on ALARM edge
 * @param user_data Context passed to callback
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_alarm_int_enable(ads1293_hal_ctx_t *ctx,
				 ads1293_alarm_callback_t callback,
				 void *user_data);

/**
 * @brief Disable ALARM interrupt
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_alarm_int_disable(ads1293_hal_ctx_t *ctx);


/* ============================================================================
 * Clock Operations
 * ============================================================================ */

/**
 * @brief Check if CLKIN (MCU-provided clock) is available
 *
 * @param ctx HAL context handle
 * @return true if CLKIN PWM is configured, false if using external crystal
 */
bool ads1293_hal_has_clkin(ads1293_hal_ctx_t *ctx);

/**
 * @brief Start the CLKIN clock output
 *
 * Starts the PWM output to provide clock to the ADS1293.
 * Only valid if ads1293_hal_has_clkin() returns true.
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_clkin_start(ads1293_hal_ctx_t *ctx);

/**
 * @brief Stop the CLKIN clock output
 *
 * Stops the PWM output. The ADS1293 will not function without a clock.
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_clkin_stop(ads1293_hal_ctx_t *ctx);

/**
 * @brief Get the configured CLKIN frequency in Hz
 *
 * @param ctx HAL context handle
 * @return Clock frequency in Hz, or 0 if using external crystal
 */
uint32_t ads1293_hal_get_clkin_frequency(ads1293_hal_ctx_t *ctx);


/* ============================================================================
 * Timing Operations
 * ============================================================================ */

/**
 * @brief Delay in microseconds
 *
 * @param ctx HAL context handle (may be NULL)
 * @param us Microseconds to delay
 */
void ads1293_hal_delay_us(ads1293_hal_ctx_t *ctx, uint32_t us);

/**
 * @brief Delay in milliseconds
 *
 * @param ctx HAL context handle (may be NULL)
 * @param ms Milliseconds to delay
 */
void ads1293_hal_delay_ms(ads1293_hal_ctx_t *ctx, uint32_t ms);

/**
 * @brief Get current timestamp in milliseconds
 *
 * @param ctx HAL context handle (may be NULL)
 * @return Current time in milliseconds
 */
uint32_t ads1293_hal_get_time_ms(ads1293_hal_ctx_t *ctx);


/* ============================================================================
 * HAL Lifecycle
 * ============================================================================ */

/**
 * @brief Initialize the HAL
 *
 * Called by the driver during device initialization.
 * Platform-specific setup (SPI, GPIO configuration) happens here.
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_init(ads1293_hal_ctx_t *ctx);

/**
 * @brief Deinitialize the HAL
 *
 * Release any resources acquired during init.
 *
 * @param ctx HAL context handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_hal_deinit(ads1293_hal_ctx_t *ctx);


#ifdef __cplusplus
}
#endif

#endif /* ADS1293_HAL_H_ */
