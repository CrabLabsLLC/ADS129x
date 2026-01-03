# ADS1293 Zephyr ECG Driver

Zephyr RTOS driver for the Texas Instruments ADS1293 3-Channel, 24-Bit Low-Power Analog Front-End for ECG applications.

## Features

- Clean, layered API design with public and internal interfaces
- Full Hardware Abstraction Layer (HAL) for portability
- 3-channel 24-bit ECG data acquisition
- Configurable sample rates (50 Hz - 25.6 kHz)
- Flexible input electrode routing
- Lead-off detection
- Right Leg Drive (RLD) support
- Common-mode detection
- Interrupt-driven data ready (DRDY)
- Flexible clock source: external crystal or MCU-provided PWM

## File Structure

```
ADS129x/
├── include/
│   ├── ads1293.h           # Public API header
│   ├── ads1293_types.h     # Type definitions
│   ├── ads1293_registers.h # Register map and bit fields
│   ├── ads1293_hal.h       # Hardware abstraction interface
│   └── ads1293_internal.h  # Internal driver structures
├── src/
│   ├── ads1293.c           # Main driver implementation
│   └── ads1293_hal_zephyr.c # Zephyr HAL implementation
├── dts/bindings/sensor/
│   └── ti,ads1293.yaml     # Device tree binding
├── zephyr/
│   └── module.yml          # Zephyr module definition
├── Kconfig                 # Configuration options
├── CMakeLists.txt          # Build configuration
└── README.md               # This file
```

## Clock Source Configuration

The ADS1293 requires a clock source. Two options are supported:

### Option 1: External Crystal (Default)

If no `clkin-pwms` property is specified, the driver assumes an external crystal is connected directly to the chip's XTAL1/XTAL2 pins. No MCU involvement is required.

```dts
&spi0 {
    ads1293: ads1293@0 {
        compatible = "ti,ads1293";
        reg = <0>;
        spi-max-frequency = <4000000>;
        drdy-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
        reset-gpios = <&gpio0 12 GPIO_ACTIVE_LOW>;
    };
};
```

### Option 2: MCU-Provided Clock via PWM

The MCU can generate the clock signal using a PWM peripheral connected to the chip's CLKIN pin. This is useful when:
- No crystal is available on the board
- Precise frequency control is needed
- Power savings are desired (clock can be stopped)

```dts
&spi0 {
    ads1293: ads1293@0 {
        compatible = "ti,ads1293";
        reg = <0>;
        spi-max-frequency = <4000000>;
        drdy-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
        reset-gpios = <&gpio0 12 GPIO_ACTIVE_LOW>;
        clkin-pwms = <&pwm0 0 PWM_HZ(409600) PWM_POLARITY_NORMAL>;
    };
};
```

The frequency is specified in the PWM specifier. The ADS1293 requires **409.6 kHz** clock frequency with 50% duty cycle.

## Device Tree Properties

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| `drdy-gpios` | phandle-array | Yes | Data ready interrupt (active-low) |
| `alarm-gpios` | phandle-array | No | Alarm/error interrupt |
| `reset-gpios` | phandle-array | No | Hardware reset |
| `clkin-pwms` | phandle-array | No | PWM channel for MCU-provided clock |
| `vin-supply` | phandle | No | Power supply regulator |

## Kconfig Options

```kconfig
CONFIG_ADS1293=y              # Enable driver
CONFIG_ADS1293_TRIGGER=y      # Enable interrupt support
CONFIG_ADS1293_AUTO_START=n   # Auto-start on init
CONFIG_ADS1293_LOG_LEVEL=3    # Log level (0-4)
```

## Usage Example

```c
#include <zephyr/device.h>
#include "ads1293.h"

/* Get device handle */
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ads1293));
ads1293_dev_t *ecg = (ads1293_dev_t *)dev->data;

/* Check device ready */
if (!ads1293_is_ready(ecg)) {
    printk("ECG device not ready\n");
    return -ENODEV;
}

/* Configure with defaults (3-lead ECG @ 256 Hz) */
ads1293_config_t config;
ads1293_config_defaults(&config);
ads1293_configure(ecg, &config);

/* Start acquisition */
ads1293_start(ecg);

/* Read ECG data (polling) */
ads1293_ecg_data_t data;
bool ready;

while (1) {
    ads1293_data_ready(ecg, &ready);
    if (ready) {
        ads1293_read_ecg(ecg, &data);

        /* Convert to microvolts */
        int32_t lead_i_uv = ads1293_raw_to_uv(data.ch1);
        int32_t lead_ii_uv = ads1293_raw_to_uv(data.ch2);
        int32_t lead_iii_uv = ads1293_raw_to_uv(data.ch3);

        printk("ECG: I=%d II=%d III=%d uV\n",
               lead_i_uv, lead_ii_uv, lead_iii_uv);
    }
    k_msleep(1);
}
```

### Interrupt-Driven Acquisition

```c
void ecg_drdy_callback(void *user_data)
{
    struct k_sem *sem = (struct k_sem *)user_data;
    k_sem_give(sem);
}

int main(void)
{
    struct k_sem data_ready;
    k_sem_init(&data_ready, 0, 1);

    /* Setup callback */
    ads1293_set_drdy_callback(ecg, ecg_drdy_callback, &data_ready);

    /* Start acquisition */
    ads1293_start(ecg);

    while (1) {
        /* Wait for data ready interrupt */
        k_sem_take(&data_ready, K_FOREVER);

        ads1293_ecg_data_t data;
        ads1293_read_ecg(ecg, &data);

        /* Process data... */
    }
}
```

## API Reference

### Lifecycle

| Function | Description |
|----------|-------------|
| `ads1293_is_ready()` | Check if device is initialized |
| `ads1293_start()` | Start ECG acquisition |
| `ads1293_stop()` | Stop acquisition (standby) |
| `ads1293_reset()` | Hardware reset |

### Configuration

| Function | Description |
|----------|-------------|
| `ads1293_config_defaults()` | Initialize config with defaults |
| `ads1293_configure()` | Apply full configuration |
| `ads1293_get_config()` | Read current configuration |
| `ads1293_set_channel_inputs()` | Set electrode routing |
| `ads1293_set_channels_enabled()` | Enable/disable channels |
| `ads1293_set_sample_rate()` | Set decimation rates |
| `ads1293_set_lead_off()` | Configure lead-off detection |
| `ads1293_set_rld()` | Configure Right Leg Drive |

### Data Acquisition

| Function | Description |
|----------|-------------|
| `ads1293_read_ecg()` | Read all channel data |
| `ads1293_read_channel()` | Read single channel |
| `ads1293_data_ready()` | Check if data available |
| `ads1293_raw_to_mv()` | Convert to millivolts |
| `ads1293_raw_to_uv()` | Convert to microvolts |

### Interrupts

| Function | Description |
|----------|-------------|
| `ads1293_set_drdy_callback()` | Register data ready callback |
| `ads1293_set_alarm_callback()` | Register alarm callback |

### Status

| Function | Description |
|----------|-------------|
| `ads1293_get_status()` | Get device status |
| `ads1293_get_lead_off_status()` | Get lead-off status |
| `ads1293_get_revision()` | Get device revision ID |

## Standard 12-Lead ECG Configuration

For clinical 12-lead ECG, configure electrodes as:

| Electrode | Input | Description |
|-----------|-------|-------------|
| RA | IN1 | Right Arm |
| LA | IN2 | Left Arm |
| LL | IN3 | Left Leg |
| RL | IN4 | Right Leg (RLD) |

Channel configuration:
- CH1 = IN2 - IN1 = Lead I (LA - RA)
- CH2 = IN3 - IN1 = Lead II (LL - RA)
- CH3 = IN3 - IN2 = Lead III (LL - LA)

Augmented leads (aVR, aVL, aVF) and precordial leads (V1-V6) are calculated in software.

## Hardware Notes

- Operating voltage: 2.7V - 5.5V (analog), 1.65V - 3.6V (digital I/O)
- Clock frequency: 409.6 kHz (from 4.096 MHz crystal / 10)
- Input voltage range: ±400 mV differential
- Resolution: 24-bit
- Power consumption: 0.3 mW per channel
- SPI clock: Up to 20 MHz (4 MHz recommended)

## References

- [ADS1293 Datasheet](https://www.ti.com/product/ADS1293)
- [TI Application Note SNAA211](https://www.ti.com/lit/an/snaa211/snaa211.pdf)

## License

SPDX-License-Identifier: Apache-2.0
