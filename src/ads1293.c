/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 ECG Analog Front-End Driver
 *
 * Main driver implementation for the Texas Instruments ADS1293.
 */

#define DT_DRV_COMPAT ti_ads1293

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ads1293.h"
#include "ads1293_internal.h"
#include "ads1293_registers.h"

LOG_MODULE_REGISTER(ads1293, CONFIG_SENSOR_LOG_LEVEL);


/* ============================================================================
 * Internal Register Access
 * ============================================================================ */

int ads1293_read_reg(ads1293_dev_t *const device,
		     const uint8_t register_address,
		     uint8_t *const value)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}
	return ads1293_hal_reg_read(device->hal, register_address, value);
}

int ads1293_write_reg(ads1293_dev_t *const device,
		      const uint8_t register_address,
		      const uint8_t value)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}
	return ads1293_hal_reg_write(device->hal, register_address, value);
}

int ads1293_modify_reg(ads1293_dev_t *const device,
		       const uint8_t register_address,
		       const uint8_t mask,
		       const uint8_t value)
{
	uint8_t current_value;
	const int read_result = ads1293_read_reg(device, register_address, &current_value);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	const uint8_t new_value = (current_value & ~mask) | (value & mask);
	return ads1293_write_reg(device, register_address, new_value);
}

int ads1293_read_regs(ads1293_dev_t *const device,
		      const uint8_t start_register,
		      uint8_t *const buffer,
		      const size_t length)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}
	return ads1293_hal_reg_read_burst(device->hal, start_register, buffer, length);
}


/* ============================================================================
 * Device Acquisition
 * ============================================================================ */

bool ads1293_is_ready(const ads1293_dev_t *const device)
{
	return device && device->initialized && device->state != ADS1293_STATE_UNINITIALIZED;
}


/* ============================================================================
 * Lifecycle Management
 * ============================================================================ */

int ads1293_start(ads1293_dev_t *const device)
{
	if (!ads1293_is_ready(device))
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}

	const int write_result = ads1293_write_reg(device,
						   ADS1293_REG_CONFIG,
						   ADS1293_CONFIG_START);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	device->state = ADS1293_STATE_CONVERTING;
	LOG_DBG("Conversion started");
	return ADS1293_OK;
}

int ads1293_stop(ads1293_dev_t *const device)
{
	if (!ads1293_is_ready(device))
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}

	const int write_result = ads1293_write_reg(device,
						   ADS1293_REG_CONFIG,
						   ADS1293_CONFIG_STANDBY);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	device->state = ADS1293_STATE_STANDBY;
	LOG_DBG("Conversion stopped");
	return ADS1293_OK;
}

int ads1293_reset(ads1293_dev_t *const device)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}

	if (!ads1293_hal_has_reset(device->hal))
	{
		LOG_WRN("Reset GPIO not configured");
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Assert reset */
	int result = ads1293_hal_reset_assert(device->hal);
	if (result != ADS1293_OK)
	{
		return result;
	}

	ads1293_hal_delay_us(device->hal, 100);

	/* Release reset */
	result = ads1293_hal_reset_release(device->hal);
	if (result != ADS1293_OK)
	{
		return result;
	}

	/* Wait for device to stabilize */
	ads1293_hal_delay_ms(device->hal, 10);

	device->state = ADS1293_STATE_STANDBY;
	LOG_DBG("Hardware reset complete");
	return ADS1293_OK;
}


/* ============================================================================
 * Configuration
 * ============================================================================ */

void ads1293_config_defaults(ads1293_config_t *const config)
{
	if (!config)
	{
		return;
	}

	memset(config, 0, sizeof(*config));

	/* Enable all 3 channels */
	config->channel_mask = ADS1293_CH_ALL_EN_MASK;

	/* Standard 3-lead ECG:
	 * CH1: Lead I  = LA - RA (IN2 - IN1)
	 * CH2: Lead II = LL - RA (IN3 - IN1)
	 * CH3: Lead III = LL - LA (IN3 - IN2)
	 */
	config->inputs[0].positive = ADS1293_INPUT_IN2;
	config->inputs[0].negative = ADS1293_INPUT_IN1;
	config->inputs[1].positive = ADS1293_INPUT_IN3;
	config->inputs[1].negative = ADS1293_INPUT_IN1;
	config->inputs[2].positive = ADS1293_INPUT_IN3;
	config->inputs[2].negative = ADS1293_INPUT_IN2;

	/* Decimation for ~256 Hz sample rate:
	 * 409600 / (5 * 4 * 80) = 256 Hz
	 */
	config->decimation.r1 = ADS1293_R1_DECIMATE_5;
	config->decimation.r2 = 4;
	config->decimation.r3[0] = 80;
	config->decimation.r3[1] = 80;
	config->decimation.r3[2] = 80;

	/* Common-mode detection on IN1-IN4 */
	config->cmdet.enabled = true;
	config->cmdet.input_mask = 0x0F;

	/* Enable RLD */
	config->rld.enabled = true;

	/* Internal clock */
	config->clock.use_external = false;
	config->clock.output_enable = false;

	/* DRDY from CH1 24-bit data */
	config->drdy_source = ADS1293_DRDY_SRC_CH1_24BIT;
}

int ads1293_apply_config(ads1293_dev_t *const device, const ads1293_config_t *const config)
{
	int write_result;

	/* Configure channel inputs */
	for (int channel_index = 0; channel_index < ADS1293_NUM_CHANNELS; channel_index++)
	{
		const uint8_t channel_input_config = ADS1293_FLEX_CH(
			config->inputs[channel_index].positive,
			config->inputs[channel_index].negative);

		write_result = ads1293_write_reg(device,
						 ADS1293_REG_FLEX_CH1_CN + channel_index,
						 channel_input_config);
		if (write_result != ADS1293_OK)
		{
			LOG_ERR("Failed to configure CH%d inputs", channel_index + 1);
			return write_result;
		}
	}

	/* Configure decimation - R1 */
	write_result = ads1293_write_reg(device, ADS1293_REG_R1_RATE, config->decimation.r1);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure decimation - R2 (register value = R2 - 2) */
	const uint8_t r2_register_value = config->decimation.r2 - 2;
	write_result = ads1293_write_reg(device, ADS1293_REG_R2_RATE, r2_register_value);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure decimation - R3 per channel */
	write_result = ads1293_write_reg(device,
					 ADS1293_REG_R3_RATE_CH1,
					 config->decimation.r3[0]);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}
	write_result = ads1293_write_reg(device,
					 ADS1293_REG_R3_RATE_CH2,
					 config->decimation.r3[1]);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}
	write_result = ads1293_write_reg(device,
					 ADS1293_REG_R3_RATE_CH3,
					 config->decimation.r3[2]);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure common-mode detection */
	if (config->cmdet.enabled)
	{
		write_result = ads1293_write_reg(device,
						 ADS1293_REG_CMDET_EN,
						 config->cmdet.input_mask);
		if (write_result != ADS1293_OK)
		{
			return write_result;
		}
	}

	/* Configure RLD */
	const uint8_t rld_control_value =
		(config->rld.enabled ? ADS1293_RLD_EN_MASK : 0) |
		(config->rld.output_select & ADS1293_RLD_OUT_MASK);
	write_result = ads1293_write_reg(device, ADS1293_REG_RLD_CN, rld_control_value);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure clock */
	const uint8_t oscillator_config =
		(config->clock.use_external ? ADS1293_OSC_CLK_SEL_MASK : 0) |
		(config->clock.output_enable ? ADS1293_OSC_CLK_OUT_MASK : 0) |
		((config->clock.output_enable && config->clock.output_divide) ?
			ADS1293_OSC_CLK_DIV_MASK : 0);
	write_result = ads1293_write_reg(device, ADS1293_REG_OSC_CN, oscillator_config);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure DRDY source */
	write_result = ads1293_write_reg(device, ADS1293_REG_DRDYB_SRC, config->drdy_source);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Enable channels */
	write_result = ads1293_write_reg(device, ADS1293_REG_CH_CNFG, config->channel_mask);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	LOG_DBG("Configuration applied");
	return ADS1293_OK;
}

int ads1293_configure(ads1293_dev_t *const device, const ads1293_config_t *const config)
{
	if (!ads1293_is_ready(device) || !config)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Stop conversion before reconfiguring */
	if (device->state == ADS1293_STATE_CONVERTING)
	{
		const int stop_result = ads1293_stop(device);
		if (stop_result != ADS1293_OK)
		{
			return stop_result;
		}
	}

	const int apply_result = ads1293_apply_config(device, config);
	if (apply_result != ADS1293_OK)
	{
		return apply_result;
	}

	/* Store configuration */
	memcpy(&device->config, config, sizeof(device->config));

	return ADS1293_OK;
}

int ads1293_get_config(const ads1293_dev_t *const device, ads1293_config_t *const config)
{
	if (!device || !config)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	memcpy(config, &device->config, sizeof(*config));
	return ADS1293_OK;
}


/* ============================================================================
 * Channel Configuration Convenience Functions
 * ============================================================================ */

int ads1293_set_channel_inputs(ads1293_dev_t *const device,
			       const ads1293_channel_t channel,
			       const ads1293_input_t positive,
			       const ads1293_input_t negative)
{
	if (!ads1293_is_ready(device) || channel >= ADS1293_NUM_CHANNELS)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	const uint8_t channel_input_routing = ADS1293_FLEX_CH(positive, negative);
	const int write_result = ads1293_write_reg(device,
						   ADS1293_REG_FLEX_CH1_CN + channel,
						   channel_input_routing);

	if (write_result == ADS1293_OK)
	{
		device->config.inputs[channel].positive = positive;
		device->config.inputs[channel].negative = negative;
	}

	return write_result;
}

int ads1293_set_channels_enabled(ads1293_dev_t *const device, uint8_t channel_mask)
{
	if (!ads1293_is_ready(device))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	channel_mask &= ADS1293_CH_ALL_EN_MASK;
	const int write_result = ads1293_write_reg(device, ADS1293_REG_CH_CNFG, channel_mask);

	if (write_result == ADS1293_OK)
	{
		device->config.channel_mask = channel_mask;
	}

	return write_result;
}

int ads1293_set_sample_rate(ads1293_dev_t *const device,
			    const ads1293_r1_rate_t r1,
			    const uint8_t r2,
			    const uint8_t r3)
{
	if (!ads1293_is_ready(device))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Validate R2 range */
	if (r2 < ADS1293_R2_MIN || r2 > ADS1293_R2_MAX)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Validate R3 range */
	if (r3 < ADS1293_R3_MIN)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	int write_result = ads1293_write_reg(device, ADS1293_REG_R1_RATE, r1);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	write_result = ads1293_write_reg(device, ADS1293_REG_R2_RATE, r2 - 2);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Apply R3 to all channels */
	write_result = ads1293_write_reg(device, ADS1293_REG_R3_RATE_CH1, r3);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}
	write_result = ads1293_write_reg(device, ADS1293_REG_R3_RATE_CH2, r3);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}
	write_result = ads1293_write_reg(device, ADS1293_REG_R3_RATE_CH3, r3);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Update stored config */
	device->config.decimation.r1 = r1;
	device->config.decimation.r2 = r2;
	device->config.decimation.r3[0] = r3;
	device->config.decimation.r3[1] = r3;
	device->config.decimation.r3[2] = r3;

	return ADS1293_OK;
}


/* ============================================================================
 * Data Acquisition
 * ============================================================================ */

int ads1293_read_ecg(ads1293_dev_t *const device, ads1293_ecg_data_t *const data)
{
	if (!ads1293_is_ready(device) || !data)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Read status first */
	int read_result = ads1293_read_reg(device, ADS1293_REG_DATA_STATUS, &data->status);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	/* Read all 9 bytes of ECG data */
	uint8_t ecg_data_buffer[9];
	read_result = ads1293_read_regs(device,
					ADS1293_REG_DATA_CH1_ECG_H,
					ecg_data_buffer,
					sizeof(ecg_data_buffer));
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	/* Convert 24-bit two's complement to 32-bit signed */
	for (int channel_index = 0; channel_index < ADS1293_NUM_CHANNELS; channel_index++)
	{
		const int buffer_offset = channel_index * 3;
		int32_t raw_sample = ((int32_t)ecg_data_buffer[buffer_offset] << 16) |
				     ((int32_t)ecg_data_buffer[buffer_offset + 1] << 8) |
				     ((int32_t)ecg_data_buffer[buffer_offset + 2]);

		/* Sign extend from 24-bit */
		if (raw_sample & 0x800000)
		{
			raw_sample |= 0xFF000000;
		}

		switch (channel_index)
		{
		case 0:
			data->ch1 = raw_sample;
			break;
		case 1:
			data->ch2 = raw_sample;
			break;
		case 2:
			data->ch3 = raw_sample;
			break;
		}
	}

	return ADS1293_OK;
}

int ads1293_read_channel(ads1293_dev_t *const device,
			 const ads1293_channel_t channel,
			 int32_t *const value)
{
	if (!ads1293_is_ready(device) || !value || channel >= ADS1293_NUM_CHANNELS)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t channel_data_buffer[3];
	const uint8_t register_address = ADS1293_REG_DATA_CH1_ECG_H + (channel * 3);

	const int read_result = ads1293_read_regs(device,
						  register_address,
						  channel_data_buffer,
						  sizeof(channel_data_buffer));
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	int32_t raw_sample = ((int32_t)channel_data_buffer[0] << 16) |
			     ((int32_t)channel_data_buffer[1] << 8) |
			     ((int32_t)channel_data_buffer[2]);

	if (raw_sample & 0x800000)
	{
		raw_sample |= 0xFF000000;
	}

	*value = raw_sample;
	return ADS1293_OK;
}

int ads1293_data_ready(ads1293_dev_t *const device, bool *const ready)
{
	if (!ads1293_is_ready(device) || !ready)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	uint8_t data_status_register;
	const int read_result = ads1293_read_reg(device,
						 ADS1293_REG_DATA_STATUS,
						 &data_status_register);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	device->data_status = data_status_register;
	*ready = (data_status_register & ADS1293_STAT_ECG_ALL_RDY_MASK) != 0;

	return ADS1293_OK;
}


/* ============================================================================
 * Interrupt Handling
 * ============================================================================ */

int ads1293_set_drdy_callback(ads1293_dev_t *const device,
			      const ads1293_drdy_callback_t callback,
			      void *const user_data)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}

	if (!ads1293_hal_has_drdy(device->hal))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	if (callback)
	{
		device->drdy_callback = callback;
		device->drdy_user_data = user_data;
		return ads1293_hal_drdy_int_enable(device->hal, ads1293_drdy_isr, device);
	}
	else
	{
		device->drdy_callback = NULL;
		device->drdy_user_data = NULL;
		return ads1293_hal_drdy_int_disable(device->hal);
	}
}

int ads1293_set_alarm_callback(ads1293_dev_t *const device,
			       const ads1293_alarm_callback_t callback,
			       void *const user_data)
{
	if (!device || !device->hal)
	{
		return ADS1293_ERR_NOT_INITIALIZED;
	}

	if (!ads1293_hal_has_alarm(device->hal))
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	if (callback)
	{
		device->alarm_callback = callback;
		device->alarm_user_data = user_data;
		return ads1293_hal_alarm_int_enable(device->hal, ads1293_alarm_isr, device);
	}
	else
	{
		device->alarm_callback = NULL;
		device->alarm_user_data = NULL;
		return ads1293_hal_alarm_int_disable(device->hal);
	}
}

void ads1293_drdy_isr(void *const user_data)
{
	ads1293_dev_t *const device = (ads1293_dev_t *)user_data;

	if (device && device->drdy_callback)
	{
		device->drdy_callback(device->drdy_user_data);
	}
}

void ads1293_alarm_isr(void *const user_data)
{
	ads1293_dev_t *const device = (ads1293_dev_t *)user_data;

	if (device && device->alarm_callback)
	{
		device->alarm_callback(device->error_status, device->alarm_user_data);
	}
}


/* ============================================================================
 * Status and Diagnostics
 * ============================================================================ */

int ads1293_get_status(ads1293_dev_t *const device, ads1293_status_t *const status)
{
	if (!ads1293_is_ready(device) || !status)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	status->state = device->state;
	status->revision = device->revision;

	int read_result = ads1293_read_reg(device,
					   ADS1293_REG_DATA_STATUS,
					   &status->data_status);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	read_result = ads1293_read_reg(device,
				       ADS1293_REG_ERROR_LOD,
				       &status->lead_off_status);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	read_result = ads1293_read_reg(device,
				       ADS1293_REG_ERROR_STATUS,
				       &status->error_status);
	if (read_result != ADS1293_OK)
	{
		return read_result;
	}

	return ADS1293_OK;
}

int ads1293_get_lead_off_status(ads1293_dev_t *const device, uint8_t *const lead_off_mask)
{
	if (!ads1293_is_ready(device) || !lead_off_mask)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	return ads1293_read_reg(device, ADS1293_REG_ERROR_LOD, lead_off_mask);
}

int ads1293_get_revision(ads1293_dev_t *const device, uint8_t *const revision)
{
	if (!device || !revision)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	return ads1293_read_reg(device, ADS1293_REG_REVID, revision);
}


/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

int ads1293_set_clock(ads1293_dev_t *const device, const ads1293_clock_t *const config)
{
	if (!ads1293_is_ready(device) || !config)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	const uint8_t oscillator_config =
		(config->use_external ? ADS1293_OSC_CLK_SEL_MASK : 0) |
		(config->output_enable ? ADS1293_OSC_CLK_OUT_MASK : 0) |
		((config->output_enable && config->output_divide) ?
			ADS1293_OSC_CLK_DIV_MASK : 0);

	const int write_result = ads1293_write_reg(device,
						   ADS1293_REG_OSC_CN,
						   oscillator_config);
	if (write_result == ADS1293_OK)
	{
		memcpy(&device->config.clock, config, sizeof(device->config.clock));
	}

	return write_result;
}


/* ============================================================================
 * Lead-Off Detection
 * ============================================================================ */

int ads1293_set_lead_off(ads1293_dev_t *const device, const ads1293_lead_off_t *const config)
{
	if (!ads1293_is_ready(device) || !config)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	/* Configure LOD_CN */
	const uint8_t lead_off_control_value =
		(config->threshold & ADS1293_LOD_COMP_TH_MASK) |
		(config->enable_dc ? ADS1293_LOD_DC_EN_MASK : 0) |
		(config->enable_ac ? ADS1293_LOD_AC_EN_MASK : 0);

	int write_result = ads1293_write_reg(device, ADS1293_REG_LOD_CN, lead_off_control_value);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure LOD_EN */
	write_result = ads1293_write_reg(device, ADS1293_REG_LOD_EN, config->input_mask);
	if (write_result != ADS1293_OK)
	{
		return write_result;
	}

	/* Configure LOD_CURRENT */
	write_result = ads1293_write_reg(device, ADS1293_REG_LOD_CURRENT, config->current);
	if (write_result == ADS1293_OK)
	{
		memcpy(&device->config.lead_off, config, sizeof(device->config.lead_off));
	}

	return write_result;
}


/* ============================================================================
 * Right Leg Drive
 * ============================================================================ */

int ads1293_set_rld(ads1293_dev_t *const device, const ads1293_rld_t *const config)
{
	if (!ads1293_is_ready(device) || !config)
	{
		return ADS1293_ERR_INVALID_PARAM;
	}

	const uint8_t rld_control_value =
		(config->output_select & ADS1293_RLD_OUT_MASK) |
		(config->enabled ? ADS1293_RLD_EN_MASK : 0);

	const int write_result = ads1293_write_reg(device, ADS1293_REG_RLD_CN, rld_control_value);
	if (write_result == ADS1293_OK)
	{
		memcpy(&device->config.rld, config, sizeof(device->config.rld));
	}

	return write_result;
}


/* ============================================================================
 * Zephyr Device Driver Integration
 * ============================================================================ */

#if DT_HAS_COMPAT_STATUS_OKAY(ti_ads1293)

/*
 * SPI Mode 0: CPOL=0, CPHA=0 - data sampled on rising edge, clock idles low
 * Note: SPI_MODE_CPOL and SPI_MODE_CPHA are NOT set (both = 0 for Mode 0)
 */
#define ADS1293_SPI_MODE (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)

/* HAL context embedded in device data for each instance */
struct ads1293_zephyr_data
{
	struct ads1293_dev dev;
	struct ads1293_hal_ctx hal_ctx;
};

struct ads1293_zephyr_config
{
	struct spi_dt_spec spi;
	struct gpio_dt_spec drdy_gpio;
	struct gpio_dt_spec alarm_gpio;
	struct gpio_dt_spec reset_gpio;
	struct pwm_dt_spec clkin_pwm;
	bool has_clkin_pwm;
};

static int ads1293_zephyr_init(const struct device *const zephyr_device)
{
	struct ads1293_zephyr_data *const data = zephyr_device->data;
	const struct ads1293_zephyr_config *const cfg = zephyr_device->config;
	struct ads1293_dev *const device = &data->dev;
	struct ads1293_hal_ctx *const hal = &data->hal_ctx;

	LOG_INF("Initializing ADS1293...");

	/* Setup HAL context with device tree bindings */
	hal->spi = &cfg->spi;
	hal->drdy_gpio = cfg->drdy_gpio.port ? &cfg->drdy_gpio : NULL;
	hal->alarm_gpio = cfg->alarm_gpio.port ? &cfg->alarm_gpio : NULL;
	hal->reset_gpio = cfg->reset_gpio.port ? &cfg->reset_gpio : NULL;
	hal->clkin_pwm = cfg->has_clkin_pwm ? &cfg->clkin_pwm : NULL;

	/* Initialize HAL */
	int init_result = ads1293_hal_init(hal);
	if (init_result != ADS1293_OK)
	{
		LOG_ERR("HAL init failed: %d", init_result);
		return -EIO;
	}

	/* Link device to HAL */
	device->hal = hal;

	/* Start CLKIN PWM if configured (must be before reset/communication) */
	if (ads1293_hal_has_clkin(hal))
	{
		init_result = ads1293_hal_clkin_start(hal);
		if (init_result != ADS1293_OK)
		{
			LOG_ERR("CLKIN start failed: %d", init_result);
			return -EIO;
		}
		/* Wait for clock to stabilize */
		ads1293_hal_delay_ms(hal, 5);
	}

	/* Perform hardware reset if available */
	if (ads1293_hal_has_reset(hal))
	{
		init_result = ads1293_reset(device);
		if (init_result != ADS1293_OK)
		{
			LOG_ERR("Reset failed: %d", init_result);
			return -EIO;
		}
	}

	/* Verify device */
	init_result = ads1293_get_revision(device, &device->revision);
	if (init_result != ADS1293_OK)
	{
		LOG_ERR("Failed to read revision: %d", init_result);
		return -EIO;
	}
	LOG_INF("ADS1293 revision: 0x%02X", device->revision);

	/* Initialize with defaults */
	ads1293_config_defaults(&device->config);

	/* Apply device tree overrides - if using CLKIN PWM, set external clock mode */
	if (cfg->has_clkin_pwm)
	{
		device->config.clock.use_external = true;
	}

	/* Apply configuration */
	init_result = ads1293_apply_config(device, &device->config);
	if (init_result != ADS1293_OK)
	{
		LOG_ERR("Config failed: %d", init_result);
		return -EIO;
	}

	device->state = ADS1293_STATE_STANDBY;
	device->initialized = true;

	LOG_INF("ADS1293 initialized successfully");
	return 0;
}

/* Helper macro to check if clkin-pwms property exists */
#define ADS1293_HAS_CLKIN_PWMS(inst) DT_INST_NODE_HAS_PROP(inst, clkin_pwms)

/* Conditionally get PWM spec or use empty struct */
#define ADS1293_CLKIN_PWM_SPEC(inst)                                            \
	COND_CODE_1(ADS1293_HAS_CLKIN_PWMS(inst),                               \
		    (PWM_DT_SPEC_INST_GET(inst, clkin_pwms)),                   \
		    ({0}))

#define ADS1293_DEFINE(inst)                                                    \
	static struct ads1293_zephyr_data ads1293_data_##inst;                  \
                                                                                \
	static const struct ads1293_zephyr_config ads1293_config_##inst = {     \
		.spi = SPI_DT_SPEC_INST_GET(inst, ADS1293_SPI_MODE, 0),         \
		.drdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, {0}),  \
		.alarm_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, alarm_gpios, {0}),\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),\
		.clkin_pwm = ADS1293_CLKIN_PWM_SPEC(inst),                     \
		.has_clkin_pwm = ADS1293_HAS_CLKIN_PWMS(inst),                 \
	};                                                                      \
                                                                                \
	DEVICE_DT_INST_DEFINE(inst, ads1293_zephyr_init, NULL,                  \
			      &ads1293_data_##inst,                             \
			      &ads1293_config_##inst,                           \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADS1293_DEFINE)

/* Helper to get device by label */
ads1293_dev_t *ads1293_get_device(const char *const name)
{
	/* This is a simplified implementation - in practice you'd use
	 * DEVICE_DT_GET with proper label handling */
	(void)name;
	return NULL; /* User should use DEVICE_DT_GET directly */
}

#endif /* DT_HAS_COMPAT_STATUS_OKAY(ti_ads1293) */
