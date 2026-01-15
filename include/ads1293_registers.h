/*
 * Copyright (c) 2025 Makani Science
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADS1293 Register Definitions
 *
 * Register map and bit field definitions for the Texas Instruments
 * ADS1293 3-Channel, 24-Bit ECG Analog Front-End.
 *
 * Reference: TI ADS1293 Datasheet (SBAS502E)
 *
 * Note: Only verified bit fields from the datasheet are defined here.
 * Consult the datasheet for complete register documentation.
 */

#ifndef ADS1293_REGISTERS_H_
#define ADS1293_REGISTERS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Register Addresses
 * ============================================================================
 * From Table 8 (Register Map) in the ADS1293 Datasheet
 * ============================================================================ */

/* Control */
#define ADS1293_REG_CONFIG              0x00

/* Flexible Channel Input Routing */
#define ADS1293_REG_FLEX_CH1_CN         0x01
#define ADS1293_REG_FLEX_CH2_CN         0x02
#define ADS1293_REG_FLEX_CH3_CN         0x03
#define ADS1293_REG_FLEX_PACE_CN        0x04
#define ADS1293_REG_FLEX_VBAT_CN        0x05

/* Lead-Off Detection */
#define ADS1293_REG_LOD_CN              0x06
#define ADS1293_REG_LOD_EN              0x07
#define ADS1293_REG_LOD_CURRENT         0x08
#define ADS1293_REG_LOD_AC_CN           0x09

/* Common-Mode Detection and Right Leg Drive */
#define ADS1293_REG_CMDET_EN            0x0A
#define ADS1293_REG_CMDET_CN            0x0B
#define ADS1293_REG_RLD_CN              0x0C

/* Wilson Reference */
#define ADS1293_REG_WILSON_EN1          0x0D
#define ADS1293_REG_WILSON_EN2          0x0E
#define ADS1293_REG_WILSON_EN3          0x0F
#define ADS1293_REG_WILSON_CN           0x10

/* Reference and Oscillator */
#define ADS1293_REG_REF_CN              0x11
#define ADS1293_REG_OSC_CN              0x12

/* AFE Control */
#define ADS1293_REG_AFE_RES             0x13
#define ADS1293_REG_AFE_SHDN_CN         0x14
#define ADS1293_REG_AFE_FAULT_CN        0x15
#define ADS1293_REG_AFE_DITHER_EN       0x16
#define ADS1293_REG_AFE_PACE_CN         0x17

/* Error Status (Read-Only) */
#define ADS1293_REG_ERROR_LOD           0x18
#define ADS1293_REG_ERROR_STATUS        0x19
#define ADS1293_REG_ERROR_RANGE1        0x1A
#define ADS1293_REG_ERROR_RANGE2        0x1B
#define ADS1293_REG_ERROR_RANGE3        0x1C
#define ADS1293_REG_ERROR_SYNC          0x1D
#define ADS1293_REG_ERROR_MISC          0x1E

/* Reserved: 0x1F, 0x20 */

/* Decimation Rates */
#define ADS1293_REG_R1_RATE             0x21
#define ADS1293_REG_R2_RATE             0x22
#define ADS1293_REG_R3_RATE_CH1         0x23
#define ADS1293_REG_R3_RATE_CH2         0x24
#define ADS1293_REG_R3_RATE_CH3         0x25

/* Pace and Filter */
#define ADS1293_REG_P_DRATE             0x26
#define ADS1293_REG_DIS_EFILTER         0x27

/* Data Ready and Sync */
#define ADS1293_REG_DRDYB_SRC           0x28
#define ADS1293_REG_SYNCB_CN            0x29

/* Masks */
#define ADS1293_REG_MASK_DRDYB          0x2A
#define ADS1293_REG_MASK_ERR            0x2B

/* Reserved: 0x2C, 0x2D */

/* Alarm and Channel Enable */
#define ADS1293_REG_ALARM_FILTER        0x2E
#define ADS1293_REG_CH_CNFG             0x2F

/* Status and Data (Read-Only) */
#define ADS1293_REG_DATA_STATUS         0x30
#define ADS1293_REG_DATA_CH1_PACE_H     0x31
#define ADS1293_REG_DATA_CH1_PACE_L     0x32
#define ADS1293_REG_DATA_CH2_PACE_H     0x33
#define ADS1293_REG_DATA_CH2_PACE_L     0x34
#define ADS1293_REG_DATA_CH3_PACE_H     0x35
#define ADS1293_REG_DATA_CH3_PACE_L     0x36
#define ADS1293_REG_DATA_CH1_ECG_H      0x37
#define ADS1293_REG_DATA_CH1_ECG_M      0x38
#define ADS1293_REG_DATA_CH1_ECG_L      0x39
#define ADS1293_REG_DATA_CH2_ECG_H      0x3A
#define ADS1293_REG_DATA_CH2_ECG_M      0x3B
#define ADS1293_REG_DATA_CH2_ECG_L      0x3C
#define ADS1293_REG_DATA_CH3_ECG_H      0x3D
#define ADS1293_REG_DATA_CH3_ECG_M      0x3E
#define ADS1293_REG_DATA_CH3_ECG_L      0x3F

/* AFE_RES bit masks */
#define ADS1293_AFE_RES_FS_HIGH_CH3   (1u << 5)
#define ADS1293_AFE_RES_FS_HIGH_CH2   (1u << 4)
#define ADS1293_AFE_RES_FS_HIGH_CH1   (1u << 3)
#define ADS1293_AFE_RES_EN_HIRES_CH3  (1u << 2)
#define ADS1293_AFE_RES_EN_HIRES_CH2  (1u << 1)
#define ADS1293_AFE_RES_EN_HIRES_CH1  (1u << 0)

/* Revision ID (Read-Only) */
#define ADS1293_REG_REVID               0x40

/* Reserved: 0x41 - 0x4F */

/* Digital Test */
#define ADS1293_REG_DATA_LOOP           0x50


/* ============================================================================
 * CONFIG Register (0x00) Bit Fields
 * ============================================================================
 * Bit 0: START - Start conversion
 *        0 = Standby mode
 *        1 = Start conversion
 * ============================================================================ */

#define ADS1293_CONFIG_START_POS        0
#define ADS1293_CONFIG_START_MASK       (0x01 << ADS1293_CONFIG_START_POS)
#define ADS1293_CONFIG_START            ADS1293_CONFIG_START_MASK
#define ADS1293_CONFIG_STANDBY          0x00


/* ============================================================================
 * FLEX_CHx_CN Registers (0x01-0x03) Bit Fields
 * ============================================================================
 * Bits [7:4]: INP - Positive input selection
 * Bits [3:0]: INN - Negative input selection
 *
 * Input selection values:
 *   0x0 = Not connected
 *   0x1 = IN1
 *   0x2 = IN2
 *   0x3 = IN3
 *   0x4 = IN4
 *   0x5 = IN5 (KTD2027 only)
 *   0x6 = IN6 (KTD2027 only)
 * ============================================================================ */

#define ADS1293_FLEX_INP_POS            4
#define ADS1293_FLEX_INP_MASK           (0x0F << ADS1293_FLEX_INP_POS)
#define ADS1293_FLEX_INN_POS            0
#define ADS1293_FLEX_INN_MASK           (0x0F << ADS1293_FLEX_INN_POS)

/* Input electrode selection values */
#define ADS1293_INPUT_NC                0x00    /* Not connected */
#define ADS1293_INPUT_IN1               0x01
#define ADS1293_INPUT_IN2               0x02
#define ADS1293_INPUT_IN3               0x03
#define ADS1293_INPUT_IN4               0x04
#define ADS1293_INPUT_IN5               0x05
#define ADS1293_INPUT_IN6               0x06

/* Helper macro to construct FLEX_CHx_CN value */
#define ADS1293_FLEX_CH(inp, inn) \
	(((inp) << ADS1293_FLEX_INP_POS) | ((inn) << ADS1293_FLEX_INN_POS))


/* ============================================================================
 * FLEX_PACE_CN Register (0x04) Bit Fields
 * ============================================================================
 * Bits [7:4]: INP - Positive input for pace channel
 * Bits [3:0]: INN - Negative input for pace channel
 * ============================================================================ */

/* Uses same INP/INN format as FLEX_CHx_CN */


/* ============================================================================
 * FLEX_VBAT_CN Register (0x05) Bit Fields
 * ============================================================================
 * Bits [7:4]: INP - Positive input for VBAT channel
 * Bits [3:0]: INN - Negative input for VBAT channel
 * ============================================================================ */

/* Uses same INP/INN format as FLEX_CHx_CN */


/* ============================================================================
 * LOD_CN Register (0x06) Bit Fields - Lead-Off Detection Control
 * ============================================================================
 * Bits [2:0]: COMP_TH - Comparator threshold
 * Bit 3:      AC_LOD_EN - Enable AC lead-off detection
 * Bit 4:      DC_LOD_EN - Enable DC lead-off detection
 * ============================================================================ */

#define ADS1293_LOD_COMP_TH_POS         0
#define ADS1293_LOD_COMP_TH_MASK        (0x07 << ADS1293_LOD_COMP_TH_POS)
#define ADS1293_LOD_AC_EN_POS           3
#define ADS1293_LOD_AC_EN_MASK          (0x01 << ADS1293_LOD_AC_EN_POS)
#define ADS1293_LOD_DC_EN_POS           4
#define ADS1293_LOD_DC_EN_MASK          (0x01 << ADS1293_LOD_DC_EN_POS)


/* ============================================================================
 * LOD_EN Register (0x07) Bit Fields - Lead-Off Detection Enable
 * ============================================================================
 * Bits [5:0]: Enable lead-off detection on specific inputs
 *   Bit 0: IN1
 *   Bit 1: IN2
 *   Bit 2: IN3
 *   Bit 3: IN4
 *   Bit 4: IN5
 *   Bit 5: IN6
 * ============================================================================ */

#define ADS1293_LOD_EN_IN1_POS          0
#define ADS1293_LOD_EN_IN1_MASK         (0x01 << ADS1293_LOD_EN_IN1_POS)
#define ADS1293_LOD_EN_IN2_POS          1
#define ADS1293_LOD_EN_IN2_MASK         (0x01 << ADS1293_LOD_EN_IN2_POS)
#define ADS1293_LOD_EN_IN3_POS          2
#define ADS1293_LOD_EN_IN3_MASK         (0x01 << ADS1293_LOD_EN_IN3_POS)
#define ADS1293_LOD_EN_IN4_POS          3
#define ADS1293_LOD_EN_IN4_MASK         (0x01 << ADS1293_LOD_EN_IN4_POS)
#define ADS1293_LOD_EN_IN5_POS          4
#define ADS1293_LOD_EN_IN5_MASK         (0x01 << ADS1293_LOD_EN_IN5_POS)
#define ADS1293_LOD_EN_IN6_POS          5
#define ADS1293_LOD_EN_IN6_MASK         (0x01 << ADS1293_LOD_EN_IN6_POS)


/* ============================================================================
 * LOD_CURRENT Register (0x08) Bit Fields
 * ============================================================================
 * Bits [2:0]: ILOD - Lead-off detection current magnitude
 * ============================================================================ */

#define ADS1293_LOD_ILOD_POS            0
#define ADS1293_LOD_ILOD_MASK           (0x07 << ADS1293_LOD_ILOD_POS)


/* ============================================================================
 * CMDET_EN Register (0x0A) Bit Fields - Common-Mode Detection Enable
 * ============================================================================
 * Bits [5:0]: Enable CM detection on specific inputs
 *   Bit 0: IN1
 *   Bit 1: IN2
 *   Bit 2: IN3
 *   Bit 3: IN4
 *   Bit 4: IN5
 *   Bit 5: IN6
 * ============================================================================ */

#define ADS1293_CMDET_EN_IN1_POS        0
#define ADS1293_CMDET_EN_IN1_MASK       (0x01 << ADS1293_CMDET_EN_IN1_POS)
#define ADS1293_CMDET_EN_IN2_POS        1
#define ADS1293_CMDET_EN_IN2_MASK       (0x01 << ADS1293_CMDET_EN_IN2_POS)
#define ADS1293_CMDET_EN_IN3_POS        2
#define ADS1293_CMDET_EN_IN3_MASK       (0x01 << ADS1293_CMDET_EN_IN3_POS)
#define ADS1293_CMDET_EN_IN4_POS        3
#define ADS1293_CMDET_EN_IN4_MASK       (0x01 << ADS1293_CMDET_EN_IN4_POS)
#define ADS1293_CMDET_EN_IN5_POS        4
#define ADS1293_CMDET_EN_IN5_MASK       (0x01 << ADS1293_CMDET_EN_IN5_POS)
#define ADS1293_CMDET_EN_IN6_POS        5
#define ADS1293_CMDET_EN_IN6_MASK       (0x01 << ADS1293_CMDET_EN_IN6_POS)


/* ============================================================================
 * RLD_CN Register (0x0C) Bit Fields - Right Leg Drive Control
 * ============================================================================
 * Bit 2:      RLD_EN - Enable RLD amplifier
 * Bits [1:0]: RLD output selection
 * ============================================================================ */

#define ADS1293_RLD_OUT_POS             0
#define ADS1293_RLD_OUT_MASK            (0x03 << ADS1293_RLD_OUT_POS)
#define ADS1293_RLD_EN_POS              2
#define ADS1293_RLD_EN_MASK             (0x01 << ADS1293_RLD_EN_POS)


/* ============================================================================
 * OSC_CN Register (0x12) Bit Fields - Oscillator Control
 * ============================================================================
 * Bit 0: CLK_DIV   - Clock output divider (0=fCLK, 1=fCLK/2)
 * Bit 1: CLK_OUT   - Enable clock output on CLKOUT pin
 * Bit 2: CLK_SEL   - Clock source (0=internal, 1=external)
 * ============================================================================ */

#define ADS1293_OSC_CLK_DIV_POS         0
#define ADS1293_OSC_CLK_DIV_MASK        (0x01 << ADS1293_OSC_CLK_DIV_POS)
#define ADS1293_OSC_CLK_OUT_POS         1
#define ADS1293_OSC_CLK_OUT_MASK        (0x01 << ADS1293_OSC_CLK_OUT_POS)
#define ADS1293_OSC_CLK_SEL_POS         2
#define ADS1293_OSC_CLK_SEL_MASK        (0x01 << ADS1293_OSC_CLK_SEL_POS)

/* Clock source values */
#define ADS1293_CLK_INTERNAL            0
#define ADS1293_CLK_EXTERNAL            1


/* ============================================================================
 * AFE_SHDN_CN Register (0x14) Bit Fields - AFE Shutdown Control
 * ============================================================================
 * Bit 0: SHDN_CH1 - Shutdown channel 1
 * Bit 1: SHDN_CH2 - Shutdown channel 2
 * Bit 2: SHDN_CH3 - Shutdown channel 3
 * ============================================================================ */

#define ADS1293_SHDN_CH1_POS            0
#define ADS1293_SHDN_CH1_MASK           (0x01 << ADS1293_SHDN_CH1_POS)
#define ADS1293_SHDN_CH2_POS            1
#define ADS1293_SHDN_CH2_MASK           (0x01 << ADS1293_SHDN_CH2_POS)
#define ADS1293_SHDN_CH3_POS            2
#define ADS1293_SHDN_CH3_MASK           (0x01 << ADS1293_SHDN_CH3_POS)
#define ADS1293_SHDN_ALL_MASK           (ADS1293_SHDN_CH1_MASK | \
					 ADS1293_SHDN_CH2_MASK | \
					 ADS1293_SHDN_CH3_MASK)


/* ============================================================================
 * R1_RATE Register (0x21) Bit Fields - First Decimation Stage
 * ============================================================================
 * Bits [2:0]: R1 decimation factor selection
 * ============================================================================ */

#define ADS1293_R1_POS                  0
#define ADS1293_R1_MASK                 (0x07 << ADS1293_R1_POS)

/* R1 decimation values */
#define ADS1293_R1_DIV2                 0x00
#define ADS1293_R1_DIV4                 0x01
#define ADS1293_R1_DIV5                 0x02
#define ADS1293_R1_DIV6                 0x03
#define ADS1293_R1_DIV8                 0x04


/* ============================================================================
 * R2_RATE Register (0x22) Bit Fields - Second Decimation Stage
 * ============================================================================
 * Bits [2:0]: R2 decimation factor (value = register + 2, range 2-8)
 * ============================================================================ */

#define ADS1293_R2_POS                  0
#define ADS1293_R2_MASK                 (0x07 << ADS1293_R2_POS)

/* R2 value range: 2-8 (write value-2 to register) */
#define ADS1293_R2_MIN                  2
#define ADS1293_R2_MAX                  8


/* ============================================================================
 * R3_RATE_CHx Registers (0x23-0x25) - Third Decimation Stage
 * ============================================================================
 * Full 8-bit register, value range 2-255
 * ============================================================================ */

#define ADS1293_R3_MIN                  2
#define ADS1293_R3_MAX                  255


/* ============================================================================
 * DRDYB_SRC Register (0x28) Bit Fields - Data Ready Source
 * ============================================================================
 * Bits [2:0]: DRDYB source selection
 * ============================================================================ */

#define ADS1293_DRDY_SRC_POS            0
#define ADS1293_DRDY_SRC_MASK           (0x07 << ADS1293_DRDY_SRC_POS)

/* Data ready source values */
#define ADS1293_DRDY_CH1_16BIT          0x00
#define ADS1293_DRDY_CH2_16BIT          0x01
#define ADS1293_DRDY_CH3_16BIT          0x02
#define ADS1293_DRDY_CH1_24BIT          0x04
#define ADS1293_DRDY_CH2_24BIT          0x05
#define ADS1293_DRDY_CH3_24BIT          0x06


/* ============================================================================
 * CH_CNFG Register (0x2F) Bit Fields - Channel Configuration
 * ============================================================================
 * Bit 0: EN_CH1 - Enable channel 1
 * Bit 1: EN_CH2 - Enable channel 2
 * Bit 2: EN_CH3 - Enable channel 3
 * ============================================================================ */

#define ADS1293_CH1_EN_POS              0
#define ADS1293_CH1_EN_MASK             (0x01 << ADS1293_CH1_EN_POS)
#define ADS1293_CH2_EN_POS              1
#define ADS1293_CH2_EN_MASK             (0x01 << ADS1293_CH2_EN_POS)
#define ADS1293_CH3_EN_POS              2
#define ADS1293_CH3_EN_MASK             (0x01 << ADS1293_CH3_EN_POS)
#define ADS1293_CH_ALL_EN_MASK          (ADS1293_CH1_EN_MASK | \
					 ADS1293_CH2_EN_MASK | \
					 ADS1293_CH3_EN_MASK)


/* ============================================================================
 * DATA_STATUS Register (0x30) Bit Fields - Read Only
 * ============================================================================
 * Bit 0: CH1_DATA_RDY - Channel 1 ECG data ready
 * Bit 1: CH2_DATA_RDY - Channel 2 ECG data ready
 * Bit 2: CH3_DATA_RDY - Channel 3 ECG data ready
 * Bit 4: PACE1_DATA_RDY - Pace channel 1 data ready
 * Bit 5: PACE2_DATA_RDY - Pace channel 2 data ready
 * Bit 6: PACE3_DATA_RDY - Pace channel 3 data ready
 * ============================================================================ */

#define ADS1293_STAT_CH1_RDY_POS        0
#define ADS1293_STAT_CH1_RDY_MASK       (0x01 << ADS1293_STAT_CH1_RDY_POS)
#define ADS1293_STAT_CH2_RDY_POS        1
#define ADS1293_STAT_CH2_RDY_MASK       (0x01 << ADS1293_STAT_CH2_RDY_POS)
#define ADS1293_STAT_CH3_RDY_POS        2
#define ADS1293_STAT_CH3_RDY_MASK       (0x01 << ADS1293_STAT_CH3_RDY_POS)
#define ADS1293_STAT_PACE1_RDY_POS      4
#define ADS1293_STAT_PACE1_RDY_MASK     (0x01 << ADS1293_STAT_PACE1_RDY_POS)
#define ADS1293_STAT_PACE2_RDY_POS      5
#define ADS1293_STAT_PACE2_RDY_MASK     (0x01 << ADS1293_STAT_PACE2_RDY_POS)
#define ADS1293_STAT_PACE3_RDY_POS      6
#define ADS1293_STAT_PACE3_RDY_MASK     (0x01 << ADS1293_STAT_PACE3_RDY_POS)
#define ADS1293_STAT_ECG_ALL_RDY_MASK   (ADS1293_STAT_CH1_RDY_MASK | \
					 ADS1293_STAT_CH2_RDY_MASK | \
					 ADS1293_STAT_CH3_RDY_MASK)


/* ============================================================================
 * ERROR_LOD Register (0x18) Bit Fields - Lead-Off Status (Read-Only)
 * ============================================================================
 * Bits [5:0]: Lead-off detected on respective inputs
 * ============================================================================ */

#define ADS1293_ERR_LOD_IN1_POS         0
#define ADS1293_ERR_LOD_IN1_MASK        (0x01 << ADS1293_ERR_LOD_IN1_POS)
#define ADS1293_ERR_LOD_IN2_POS         1
#define ADS1293_ERR_LOD_IN2_MASK        (0x01 << ADS1293_ERR_LOD_IN2_POS)
#define ADS1293_ERR_LOD_IN3_POS         2
#define ADS1293_ERR_LOD_IN3_MASK        (0x01 << ADS1293_ERR_LOD_IN3_POS)
#define ADS1293_ERR_LOD_IN4_POS         3
#define ADS1293_ERR_LOD_IN4_MASK        (0x01 << ADS1293_ERR_LOD_IN4_POS)
#define ADS1293_ERR_LOD_IN5_POS         4
#define ADS1293_ERR_LOD_IN5_MASK        (0x01 << ADS1293_ERR_LOD_IN5_POS)
#define ADS1293_ERR_LOD_IN6_POS         5
#define ADS1293_ERR_LOD_IN6_MASK        (0x01 << ADS1293_ERR_LOD_IN6_POS)


/* ============================================================================
 * SPI Protocol
 * ============================================================================
 * Read:  Set bit 7 of register address
 * Write: Clear bit 7 of register address
 * ============================================================================ */

#define ADS1293_SPI_READ_FLAG           0x80
#define ADS1293_SPI_WRITE_FLAG          0x00
#define ADS1293_SPI_ADDR_MASK           0x7F


/* ============================================================================
 * Device Constants
 * ============================================================================ */

#define ADS1293_NUM_CHANNELS            3
#define ADS1293_ECG_DATA_SIZE           3       /* 24-bit = 3 bytes */
#define ADS1293_PACE_DATA_SIZE          2       /* 16-bit = 2 bytes */
#define ADS1293_REVID_DEFAULT           0x01


/* ============================================================================
 * Helper Macros
 * ============================================================================ */

/* Extract field from register value */
#define ADS1293_GET_FIELD(reg_val, field) \
	(((reg_val) & field##_MASK) >> field##_POS)

/* Set field in register value */
#define ADS1293_SET_FIELD(reg_val, field, val) \
	(((reg_val) & ~field##_MASK) | (((val) << field##_POS) & field##_MASK))


#ifdef __cplusplus
}
#endif

#endif /* ADS1293_REGISTERS_H_ */
