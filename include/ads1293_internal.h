/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Driver Internals
 *
 * Internal structures and functions for the ADS1293 driver.
 * This header is not part of the public API.
 */

#ifndef ADS1293_INTERNAL_H_
#define ADS1293_INTERNAL_H_

#include <zephyr/sys/atomic.h>
#include "ads1293.h"
#include "ads1293_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Internal Device Structure
 * ============================================================================ */

/**
 * @brief Internal device structure
 *
 * This is the concrete implementation of ads1293_dev_t.
 * Users see only the opaque pointer.
 */
struct ads1293_dev {
	/* HAL context for hardware access */
	ads1293_hal_ctx_t *hal;

	/* Current configuration */
	ads1293_config_t config;

	/* Device state */
	ads1293_state_t state;
	uint8_t revision;

	/* Cached status */
	uint8_t data_status;
	uint8_t error_status;
	uint8_t lead_off_status;

	/* Interrupt callbacks */
	ads1293_drdy_callback_t drdy_callback;
	void *drdy_user_data;
	ads1293_alarm_callback_t alarm_callback;
	void *alarm_user_data;

	/* Sample counter (incremented on each DRDY interrupt) */
	atomic_t sample_count;

	/* Initialization flag */
	bool initialized;
};


/* ============================================================================
 * Internal Register Access
 * ============================================================================ */

/**
 * @brief Read a register
 */
int ads1293_read_reg(ads1293_dev_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Write a register
 */
int ads1293_write_reg(ads1293_dev_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief Modify specific bits in a register
 */
int ads1293_modify_reg(ads1293_dev_t *dev, uint8_t reg,
		       uint8_t mask, uint8_t val);

/**
 * @brief Read multiple consecutive registers
 */
int ads1293_read_regs(ads1293_dev_t *dev, uint8_t reg,
		      uint8_t *buf, size_t len);


/* ============================================================================
 * Internal Configuration
 * ============================================================================ */

/**
 * @brief Apply configuration to hardware
 */
int ads1293_apply_config(ads1293_dev_t *dev, const ads1293_config_t *config);


/* ============================================================================
 * Internal Interrupt Handlers
 * ============================================================================ */

/**
 * @brief DRDY interrupt handler (called from HAL)
 */
void ads1293_drdy_isr(void *user_data);

/**
 * @brief ALARM interrupt handler (called from HAL)
 */
void ads1293_alarm_isr(uint8_t error_status, void *user_data);


#ifdef __cplusplus
}
#endif

#endif /* ADS1293_INTERNAL_H_ */
