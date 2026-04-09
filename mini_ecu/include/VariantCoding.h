/**
 * @file    VariantCoding.h
 * @brief   Post-build Variant Coding (Vehicle Order Coding)
 *
 * The same ECU binary is installed in every vehicle variant.
 * A "coding block" stored in NVM tells the software which features are
 * active for THIS specific vehicle.  It is written once at end-of-line
 * in the factory (or updated by a workshop tool) via UDS service 0x2E.
 *
 * -----------------------------------------------------------------------
 * Coding block memory map  (16 bytes total):
 *
 *  Offset  Size  Name                  Values / meaning
 *  ------  ----  --------------------  --------------------------------
 *   0       1    magic                 0xEC = valid block
 *   1       1    vehicle_class         0x01=Sedan, 0x02=SUV, 0x03=Van
 *   2       1    engine_type           0x01=Petrol2L, 0x02=Diesel3L
 *   3       1    market                0x01=EU, 0x02=US, 0x03=UK
 *   4       1    drive_side            0x01=LHD (left), 0x02=RHD (right)
 *   5       1    feature_flags_0       bit field (see VC_FEAT_* defines)
 *   6       1    feature_flags_1       bit field (see VC_FEAT_* defines)
 *   7       1    max_speed_kph         e.g. 180, 250 (0=unlimited in market)
 *   8-13    6    vin_prefix            First 6 chars of VIN (ASCII)
 *  14       1    checksum_lo           XOR checksum bytes 0-13 low byte
 *  15       1    checksum_hi           XOR checksum bytes 0-13 high byte
 * -----------------------------------------------------------------------
 *
 * Feature flags byte 0 (feature_flags_0):
 *   Bit 0: Heated seats
 *   Bit 1: Sunroof / panorama roof
 *   Bit 2: Adaptive cruise control
 *   Bit 3: Lane keeping assist
 *   Bit 4: Trailer hitch installed
 *   Bit 5: Start/Stop system
 *   Bit 6: Sport mode
 *   Bit 7: Reserved
 *
 * Feature flags byte 1 (feature_flags_1):
 *   Bit 0: Remote start
 *   Bit 1: 4WD / AWD
 *   Bit 2: Air suspension
 *   Bit 3-7: Reserved
 *
 * MISRA-C:2012 compliance notes:
 *  - All bit-flag access via macros — no implicit boolean conversions
 *  - Coding block validated by checksum before use  (Dir 4.1)
 *  - No dynamic allocation                          (Rule 21.3)
 */

#ifndef VARIANT_CODING_H
#define VARIANT_CODING_H

#include "Std_Types.h"

/*============================================================================
 *  Coding block magic
 *===========================================================================*/
#define VC_MAGIC_VALID          (0xECU)
#define VC_BLOCK_SIZE           (16U)

/*============================================================================
 *  Vehicle class
 *===========================================================================*/
#define VC_CLASS_SEDAN          (0x01U)
#define VC_CLASS_SUV            (0x02U)
#define VC_CLASS_VAN            (0x03U)

/*============================================================================
 *  Engine type
 *===========================================================================*/
#define VC_ENGINE_PETROL_2L     (0x01U)
#define VC_ENGINE_DIESEL_3L     (0x02U)

/*============================================================================
 *  Market
 *===========================================================================*/
#define VC_MARKET_EU            (0x01U)
#define VC_MARKET_US            (0x02U)
#define VC_MARKET_UK            (0x03U)

/*============================================================================
 *  Drive side
 *===========================================================================*/
#define VC_DRIVE_LHD            (0x01U)   /* Left-hand drive  */
#define VC_DRIVE_RHD            (0x02U)   /* Right-hand drive */

/*============================================================================
 *  Feature flags byte 0 bitmasks
 *===========================================================================*/
#define VC_FEAT0_HEATED_SEATS   (0x01U)
#define VC_FEAT0_SUNROOF        (0x02U)
#define VC_FEAT0_ACC            (0x04U)   /* Adaptive Cruise Control */
#define VC_FEAT0_LANE_KEEP      (0x08U)
#define VC_FEAT0_TRAILER_HITCH  (0x10U)
#define VC_FEAT0_STARTSTOP      (0x20U)
#define VC_FEAT0_SPORT_MODE     (0x40U)

/*============================================================================
 *  Feature flags byte 1 bitmasks
 *===========================================================================*/
#define VC_FEAT1_REMOTE_START   (0x01U)
#define VC_FEAT1_AWD            (0x02U)
#define VC_FEAT1_AIR_SUSP       (0x04U)

/*============================================================================
 *  Coding data structure
 *===========================================================================*/
typedef struct {
    uint8_t magic;
    uint8_t vehicle_class;
    uint8_t engine_type;
    uint8_t market;
    uint8_t drive_side;
    uint8_t feature_flags_0;
    uint8_t feature_flags_1;
    uint8_t max_speed_kph;
    uint8_t vin_prefix[6];
    uint8_t checksum_lo;
    uint8_t checksum_hi;
} VariantCoding_BlockType;

/*============================================================================
 *  Public API
 *===========================================================================*/

/**
 * Load and validate the coding block from NVM.
 * If the block is invalid (bad magic or bad checksum), a safe default
 * "uncoded" configuration is used and the function returns E_NOT_OK.
 *
 * @return E_OK    — valid coding found and loaded
 * @return E_NOT_OK — coding invalid; defaults applied
 */
Std_ReturnType VC_Init(void);

/**
 * Check if a specific feature flag is active in the current coding.
 *
 * @param flag_byte  0 = feature_flags_0, 1 = feature_flags_1
 * @param bit_mask   VC_FEAT0_* or VC_FEAT1_* mask
 * @return TRUE if the feature is present, FALSE otherwise
 */
boolean VC_IsFeatureEnabled(uint8_t flag_byte, uint8_t bit_mask);

/** Get the configured maximum vehicle speed (kph). 0 = no software limit. */
uint8_t VC_GetMaxSpeedKph(void);

/** Get vehicle class (VC_CLASS_*). */
uint8_t VC_GetVehicleClass(void);

/** Get engine type (VC_ENGINE_*). */
uint8_t VC_GetEngineType(void);

/** Get market code (VC_MARKET_*). */
uint8_t VC_GetMarket(void);

/** Get drive side (VC_DRIVE_*). */
uint8_t VC_GetDriveSide(void);

/**
 * Write a new coding block to NVM (called from UDS service 0x2E).
 * Validates checksum before writing; rejects if invalid.
 *
 * @param block  Pointer to the new coding data (16 bytes)
 * @return E_OK on success, E_NOT_OK on invalid checksum or NULL input
 */
Std_ReturnType VC_WriteBlock(const VariantCoding_BlockType *block);

/** Get a read-only pointer to the active coding block. */
const VariantCoding_BlockType *VC_GetBlock(void);

/** Print current coding summary to stdout (simulation only). */
void VC_PrintSummary(void);

#endif /* VARIANT_CODING_H */
