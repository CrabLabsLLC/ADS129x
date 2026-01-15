/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 ECG Analog Front-End Driver
 *
 * Public API for the Texas Instruments ADS1293 3-Channel, 24-Bit
 * Low-Power Analog Front-End for Biopotential Measurements.
 *
 * This header provides the user-facing API. For internal driver
 * functions, see ads1293_internal.h.
 */

#ifndef ADS1293_H_
#define ADS1293_H_

#include "ads1293_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Opaque Device Handle
 * ============================================================================ */

/**
 * @brief Opaque handle to an ADS1293 device instance
 *
 * Users obtain this handle via ads1293_get_device() and pass it to
 * all API functions. The internal structure is hidden from users.
 */
typedef struct ads1293_dev ads1293_dev_t;


/* ============================================================================
 * Device Acquisition
 * ============================================================================ */

/**
 * @brief Get device handle by device tree node label
 *
 * @param name Device tree node label (e.g., "ads1293" from DT_NODELABEL)
 * @return Device handle, or NULL if not found/not ready
 *
 * @note In Zephyr, this wraps DEVICE_DT_GET(DT_NODELABEL(name))
 */
ads1293_dev_t *ads1293_get_device(const char *name);

/**
 * @brief Check if device is ready for use
 *
 * @param dev Device handle
 * @return true if device is initialized and ready
 */
bool ads1293_is_ready(const ads1293_dev_t *dev);


/* ============================================================================
 * Lifecycle Management
 * ============================================================================ */

/**
 * @brief Start ECG data acquisition
 *
 * Begins continuous conversion on all enabled channels.
 * Data can be read via ads1293_read_ecg() or through DRDY interrupt.
 *
 * @param dev Device handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_start(ads1293_dev_t *dev);

/**
 * @brief Stop ECG data acquisition
 *
 * Puts device into standby mode. Configuration is retained.
 *
 * @param dev Device handle
 * @return 0 on success, negative error code on failure
 */
int ads1293_stop(ads1293_dev_t *dev);

/**
 * @brief Perform hardware reset
 *
 * Resets device to power-on state. Requires reset GPIO to be configured.
 * After reset, device must be reconfigured.
 *
 * @param dev Device handle
 * @return 0 on success, ADS1293_ERR_* on failure
 */
int ads1293_reset(ads1293_dev_t *dev);


/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Apply complete runtime configuration
 *
 * Configures all device parameters from the provided configuration structure.
 * Device must be stopped before calling this function.
 *
 * @param dev Device handle
 * @param config Configuration to apply
 * @return 0 on success, negative error code on failure
 */
int ads1293_configure(ads1293_dev_t *dev, const ads1293_config_t *config);

/**
 * @brief Get current device configuration
 *
 * @param dev Device handle
 * @param config Output: current configuration
 * @return 0 on success, negative error code on failure
 */
int ads1293_get_config(const ads1293_dev_t *dev, ads1293_config_t *config);

/**
 * @brief Initialize configuration with sensible defaults
 *
 * Sets up standard 3-lead ECG configuration:
 * - All 3 channels enabled
 * - CH1: Lead I (LA - RA)
 * - CH2: Lead II (LL - RA)
 * - CH3: Lead III (LL - LA)
 * - ~256 Hz sample rate
 * - Common-mode detection enabled
 * - RLD enabled
 *
 * @param config Configuration structure to initialize
 */
void ads1293_config_defaults(ads1293_config_t *config);


/* ============================================================================
 * Channel Configuration (Convenience Functions)
 * ============================================================================ */

/**
 * @brief Set input electrodes for a channel
 *
 * @param dev Device handle
 * @param channel Channel number (0-2)
 * @param positive Positive input electrode
 * @param negative Negative input electrode
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_channel_inputs(ads1293_dev_t *dev,
			       ads1293_channel_t channel,
			       ads1293_input_t positive,
			       ads1293_input_t negative);

/**
 * @brief Enable or disable channels
 *
 * @param dev Device handle
 * @param channel_mask Bitmask of channels to enable (bit 0=CH1, bit 1=CH2, bit 2=CH3)
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_channels_enabled(ads1293_dev_t *dev, uint8_t channel_mask);

/**
 * @brief Configure sample rate via decimation settings
 *
 * Sample rate = f_CLK / (R1 * R2 * R3)
 * With internal 409.6 kHz clock, common rates:
 * - 256 Hz:  R1=5, R2=4, R3=80
 * - 512 Hz:  R1=5, R2=4, R3=40
 * - 1024 Hz: R1=5, R2=4, R3=20
 *
 * @param dev Device handle
 * @param r1 First stage decimation (use ADS1293_R1_DIVx)
 * @param r2 Second stage decimation (2-8)
 * @param r3 Third stage decimation (2-255, applied to all channels)
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_sample_rate(ads1293_dev_t *dev,
			    ads1293_r1_rate_t r1,
			    uint8_t r2,
			    uint8_t r3);

/**
 * @brief Configure AFE resolution per channel
 *
 * @param dev Device handle
 * @param hires_mask EN_HIRES_CHx mask (bit 0=CH1)
 * @param fs_high_mask FS_HIGH_CHx mask (bit 0=CH1)
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_afe_res(ads1293_dev_t *dev, uint8_t hires_mask, uint8_t fs_high_mask);

/**
 * @brief Set sensitivity for a single channel (EN_HIRES)
 *
 * @param dev Device handle
 * @param channel Channel index
 * @param sensitivity Sensitivity mode
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_channel_sensitivity(ads1293_dev_t *dev,
				    ads1293_channel_t channel,
				    ads1293_sensitivity_t sensitivity);

/**
 * @brief Set gain for a single channel (FS_HIGH)
 *
 * @param dev Device handle
 * @param channel Channel index
 * @param gain Gain mode
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_channel_gain(ads1293_dev_t *dev,
			     ads1293_channel_t channel,
			     ads1293_gain_t gain);


/* ============================================================================
 * Data Acquisition
 * ============================================================================ */

/**
 * @brief Read ECG data from all channels
 *
 * Reads the latest 24-bit ECG samples from all enabled channels.
 * For interrupt-driven operation, call this from the DRDY callback.
 *
 * @param dev Device handle
 * @param data Output: ECG sample data
 * @return 0 on success, negative error code on failure
 */
int ads1293_read_ecg(ads1293_dev_t *dev, ads1293_ecg_data_t *data);

/**
 * @brief Read single channel ECG data
 *
 * @param dev Device handle
 * @param channel Channel to read (0-2)
 * @param value Output: 24-bit signed ECG value
 * @return 0 on success, negative error code on failure
 */
int ads1293_read_channel(ads1293_dev_t *dev,
			 ads1293_channel_t channel,
			 int32_t *value);

/**
 * @brief Check if new data is ready
 *
 * @param dev Device handle
 * @param ready Output: true if data is ready
 * @return 0 on success, negative error code on failure
 */
int ads1293_data_ready(ads1293_dev_t *dev, bool *ready);


/* ============================================================================
 * Interrupt Handling
 * ============================================================================ */

/**
 * @brief Register callback for data ready interrupt
 *
 * The callback is invoked from interrupt context when DRDY pin goes low.
 * Keep callback processing minimal; defer heavy work to a thread.
 *
 * @param dev Device handle
 * @param callback Function to call on data ready, or NULL to disable
 * @param user_data Context pointer passed to callback
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_drdy_callback(ads1293_dev_t *dev,
			      ads1293_drdy_callback_t callback,
			      void *user_data);

/**
 * @brief Register callback for alarm/error interrupt
 *
 * @param dev Device handle
 * @param callback Function to call on alarm, or NULL to disable
 * @param user_data Context pointer passed to callback
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_alarm_callback(ads1293_dev_t *dev,
			       ads1293_alarm_callback_t callback,
			       void *user_data);

/**
 * @brief Get sample counter (incremented on each DRDY interrupt)
 *
 * Use this to check if samples are available without registering a callback.
 *
 * @param dev Device handle
 * @return Number of DRDY events since last reset/read
 */
uint32_t ads1293_get_sample_count(const ads1293_dev_t *dev);

/**
 * @brief Reset sample counter to zero
 *
 * @param dev Device handle
 */
void ads1293_reset_sample_count(ads1293_dev_t *dev);

/**
 * @brief Get and reset sample counter atomically
 *
 * Useful for checking how many samples accumulated since last check.
 *
 * @param dev Device handle
 * @return Number of DRDY events since last call
 */
uint32_t ads1293_get_and_reset_sample_count(ads1293_dev_t *dev);


/* ============================================================================
 * Status and Diagnostics
 * ============================================================================ */

/**
 * @brief Get device status
 *
 * @param dev Device handle
 * @param status Output: current device status
 * @return 0 on success, negative error code on failure
 */
int ads1293_get_status(ads1293_dev_t *dev, ads1293_status_t *status);

/**
 * @brief Get lead-off detection status
 *
 * Returns bitmask indicating which electrodes have detected lead-off.
 *
 * @param dev Device handle
 * @param lead_off_mask Output: lead-off status (bit per input)
 * @return 0 on success, negative error code on failure
 */
int ads1293_get_lead_off_status(ads1293_dev_t *dev, uint8_t *lead_off_mask);

/**
 * @brief Get device revision ID
 *
 * @param dev Device handle
 * @param revision Output: revision ID byte
 * @return 0 on success, negative error code on failure
 */
int ads1293_get_revision(ads1293_dev_t *dev, uint8_t *revision);


/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

/**
 * @brief Configure clock source and output
 *
 * @param dev Device handle
 * @param config Clock configuration
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_clock(ads1293_dev_t *dev, const ads1293_clock_t *config);


/* ============================================================================
 * Lead-Off Detection
 * ============================================================================ */

/**
 * @brief Configure lead-off detection
 *
 * @param dev Device handle
 * @param config Lead-off detection configuration
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_lead_off(ads1293_dev_t *dev, const ads1293_lead_off_t *config);


/* ============================================================================
 * Right Leg Drive (RLD)
 * ============================================================================ */

/**
 * @brief Configure Right Leg Drive amplifier
 *
 * @param dev Device handle
 * @param config RLD configuration
 * @return 0 on success, negative error code on failure
 */
int ads1293_set_rld(ads1293_dev_t *dev, const ads1293_rld_t *config);


/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert raw 24-bit ECG value to millivolts
 *
 * The ADS1293 has ±400mV full-scale range with 24-bit resolution.
 *
 * @param raw Raw 24-bit value from ads1293_read_ecg()
 * @return Voltage in millivolts (float)
 */
static inline float ads1293_raw_to_mv(int32_t raw)
{
	/* Full scale = 800mV (±400mV), 24-bit = 16777216 counts */
	return (float)raw * 800.0f / 16777216.0f;
}

/**
 * @brief Convert raw 24-bit ECG value to microvolts
 *
 * @param raw Raw 24-bit value from ads1293_read_ecg()
 * @return Voltage in microvolts (int32_t)
 */
static inline int32_t ads1293_raw_to_uv(int32_t raw)
{
	/* Full scale = 800000µV, 24-bit = 16777216 counts */
	/* Using 64-bit intermediate to avoid overflow */
	return (int32_t)(((int64_t)raw * 800000LL) / 16777216LL);
}


/* ============================================================================
 * GPIO Read Functions (for polling mode)
 * ============================================================================ */

/**
 * @brief Read the current state of the DRDY GPIO pin
 *
 * Useful for polling mode when DRDY interrupts are disabled.
 * DRDY is active-low (0 = data ready, 1 = no data).
 *
 * @param dev   Device handle
 * @param state Output: GPIO pin state (0 = active/low, 1 = inactive/high)
 * @return 0 on success, ADS1293_ERR_* on failure
 */
int ads1293_read_drdy_gpio(ads1293_dev_t *dev, int *state);

/**
 * @brief Read the current state of the ALARM GPIO pin
 *
 * Useful for polling mode when ALARM interrupts are disabled.
 * ALARM is active-low (0 = alarm active, 1 = no alarm).
 *
 * @param dev   Device handle
 * @param state Output: GPIO pin state (0 = active/low, 1 = inactive/high)
 * @return 0 on success, ADS1293_ERR_* on failure
 */
int ads1293_read_alarm_gpio(ads1293_dev_t *dev, int *state);

/**
 * @brief Check if DRDY GPIO is configured and available
 *
 * @param dev Device handle
 * @return true if DRDY GPIO is configured, false otherwise
 */
bool ads1293_has_drdy_gpio(ads1293_dev_t *dev);

/**
 * @brief Check if ALARM GPIO is configured and available
 *
 * @param dev Device handle
 * @return true if ALARM GPIO is configured, false otherwise
 */
bool ads1293_has_alarm_gpio(ads1293_dev_t *dev);


#ifdef __cplusplus
}
#endif

#endif /* ADS1293_H_ */
