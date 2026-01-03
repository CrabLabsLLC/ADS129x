/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Type Definitions
 *
 * Data structures and type definitions for the ADS1293 ECG AFE driver.
 */

#ifndef ADS1293_TYPES_H_
#define ADS1293_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include "ads1293_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Error Codes
 * ============================================================================ */

#define ADS1293_OK                      0
#define ADS1293_ERR_INVALID_PARAM       (-1)
#define ADS1293_ERR_IO                  (-2)
#define ADS1293_ERR_TIMEOUT             (-3)
#define ADS1293_ERR_NOT_READY           (-4)
#define ADS1293_ERR_DEVICE_ID           (-5)
#define ADS1293_ERR_NOT_INITIALIZED     (-6)


/* ============================================================================
 * Enumerations
 * ============================================================================ */

/**
 * @brief ECG channel identifiers
 */
typedef enum {
	ADS1293_CHANNEL_1 = 0,
	ADS1293_CHANNEL_2 = 1,
	ADS1293_CHANNEL_3 = 2
} ads1293_channel_t;

/**
 * @brief Input electrode selection
 */
typedef enum {
	ADS1293_INPUT_NONE = 0,
	ADS1293_INPUT_IN1  = 1,
	ADS1293_INPUT_IN2  = 2,
	ADS1293_INPUT_IN3  = 3,
	ADS1293_INPUT_IN4  = 4,
	ADS1293_INPUT_IN5  = 5,
	ADS1293_INPUT_IN6  = 6
} ads1293_input_t;

/**
 * @brief R1 decimation rate values
 */
typedef enum {
	ADS1293_R1_DECIMATE_2 = ADS1293_R1_DIV2,
	ADS1293_R1_DECIMATE_4 = ADS1293_R1_DIV4,
	ADS1293_R1_DECIMATE_5 = ADS1293_R1_DIV5,
	ADS1293_R1_DECIMATE_6 = ADS1293_R1_DIV6,
	ADS1293_R1_DECIMATE_8 = ADS1293_R1_DIV8
} ads1293_r1_rate_t;

/**
 * @brief Data ready source selection
 */
typedef enum {
	ADS1293_DRDY_SRC_CH1_16BIT = 0x00,
	ADS1293_DRDY_SRC_CH2_16BIT = 0x01,
	ADS1293_DRDY_SRC_CH3_16BIT = 0x02,
	ADS1293_DRDY_SRC_CH1_24BIT = 0x04,
	ADS1293_DRDY_SRC_CH2_24BIT = 0x05,
	ADS1293_DRDY_SRC_CH3_24BIT = 0x06
} ads1293_drdy_src_t;

/**
 * @brief Device operating state
 */
typedef enum {
	ADS1293_STATE_UNINITIALIZED,
	ADS1293_STATE_STANDBY,
	ADS1293_STATE_CONVERTING,
	ADS1293_STATE_ERROR
} ads1293_state_t;


/* ============================================================================
 * Configuration Structures
 * ============================================================================ */

/**
 * @brief Single channel input routing configuration
 */
typedef struct {
	ads1293_input_t positive;   /**< Positive electrode input */
	ads1293_input_t negative;   /**< Negative electrode input */
} ads1293_channel_input_t;

/**
 * @brief Decimation filter configuration
 *
 * Sample rate = fCLK / (R1 * R2 * R3)
 * With internal 409.6kHz clock: sample_rate = 409600 / (R1 * R2 * R3)
 */
typedef struct {
	ads1293_r1_rate_t r1;       /**< R1 decimation (2, 4, 5, 6, or 8) */
	uint8_t r2;                 /**< R2 decimation (2-8) */
	uint8_t r3[3];              /**< R3 decimation per channel (2-255) */
} ads1293_decimation_t;

/**
 * @brief Lead-off detection configuration
 */
typedef struct {
	bool enable_dc;             /**< Enable DC lead-off detection */
	bool enable_ac;             /**< Enable AC lead-off detection */
	uint8_t threshold;          /**< Comparator threshold (0-7) */
	uint8_t current;            /**< Detection current magnitude */
	uint8_t input_mask;         /**< Which inputs to monitor */
} ads1293_lead_off_t;

/**
 * @brief Right Leg Drive (RLD) configuration
 */
typedef struct {
	bool enabled;               /**< Enable RLD amplifier */
	uint8_t input_select;       /**< RLD input source selection */
	uint8_t output_select;      /**< RLD output routing */
} ads1293_rld_t;

/**
 * @brief Common-mode detection configuration
 */
typedef struct {
	bool enabled;               /**< Enable common-mode detection */
	uint8_t input_mask;         /**< Input selection mask (IN1-IN6) */
} ads1293_cmdet_t;

/**
 * @brief Clock configuration
 */
typedef struct {
	bool use_external;          /**< Use external clock input */
	bool output_enable;         /**< Enable clock output pin */
	bool output_divide;         /**< Divide output clock by 2 */
} ads1293_clock_t;

/**
 * @brief Complete runtime configuration
 *
 * This structure contains all configurable parameters that can be
 * changed at runtime without recompiling.
 */
typedef struct {
	/* Channel configuration */
	uint8_t channel_mask;                           /**< Enabled channels (bit mask) */
	ads1293_channel_input_t inputs[ADS1293_NUM_CHANNELS];

	/* Signal processing */
	ads1293_decimation_t decimation;

	/* Auxiliary circuits */
	ads1293_lead_off_t lead_off;
	ads1293_rld_t rld;
	ads1293_cmdet_t cmdet;
	ads1293_clock_t clock;

	/* Interrupt configuration */
	ads1293_drdy_src_t drdy_source;
} ads1293_config_t;


/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief ECG sample data from all channels
 */
typedef struct {
	int32_t ch1;                /**< Channel 1 raw 24-bit data (sign-extended) */
	int32_t ch2;                /**< Channel 2 raw 24-bit data (sign-extended) */
	int32_t ch3;                /**< Channel 3 raw 24-bit data (sign-extended) */
	uint8_t status;             /**< Data status register snapshot */
} ads1293_ecg_data_t;

/**
 * @brief Device status information
 */
typedef struct {
	ads1293_state_t state;      /**< Current operating state */
	uint8_t revision;           /**< Device revision ID */
	uint8_t data_status;        /**< Latest data status */
	uint8_t lead_off_status;    /**< Lead-off detection status */
	uint8_t error_status;       /**< Error status register */
} ads1293_status_t;


/* ============================================================================
 * Callback Types
 * ============================================================================ */

/**
 * @brief Data ready callback function type
 *
 * @param user_data User-provided context pointer
 */
typedef void (*ads1293_drdy_callback_t)(void *user_data);

/**
 * @brief Alarm/error callback function type
 *
 * @param error_status Error status register value
 * @param user_data User-provided context pointer
 */
typedef void (*ads1293_alarm_callback_t)(uint8_t error_status, void *user_data);


#ifdef __cplusplus
}
#endif

#endif /* ADS1293_TYPES_H_ */
