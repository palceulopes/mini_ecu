/**
 * @file    VariantCoding.c
 * @brief   Post-build Variant Coding implementation
 *
 * NVM simulation: on a real ECU the coding block lives in a dedicated
 * NVM sector (e.g., EEPROM or Data Flash).  Here we use a static byte
 * array as a stand-in, pre-initialised with a default "EU Sedan Petrol"
 * configuration so the simulation works out of the box.
 *
 * Checksum algorithm: XOR of bytes 0-13 split into lo/hi.
 * (A real system would use CRC16-CCITT, but XOR is simpler to illustrate
 * the concept without adding a CRC library.)
 *
 * MISRA-C:2012 rules applied:
 *  - All pointer parameters NULL-checked              (Dir 4.1)
 *  - No implicit arithmetic on enum/bool types        (Rule 10.1)
 *  - Checksum verified before any data is used        (Dir 4.3)
 *  - memcpy used for struct copy (no punning)         (Rule 11.3)
 */

#include "VariantCoding.h"

#include <stdio.h>
#include <string.h>

/*============================================================================
 *  Default factory coding — "EU Sedan 2.0L Petrol, LHD, basic trim"
 *
 *  Bytes 0-13 then checksum at 14-15.
 *===========================================================================*/
static const uint8_t k_default_coding_raw[VC_BLOCK_SIZE] = {
    VC_MAGIC_VALID,        /*  0: magic                  */
    VC_CLASS_SEDAN,        /*  1: vehicle_class          */
    VC_ENGINE_PETROL_2L,   /*  2: engine_type            */
    VC_MARKET_EU,          /*  3: market                 */
    VC_DRIVE_LHD,          /*  4: drive_side             */
    VC_FEAT0_STARTSTOP,    /*  5: feature_flags_0        */
    0x00U,                 /*  6: feature_flags_1        */
    200U,                  /*  7: max_speed_kph (200 kph)*/
    'W', 'V', 'W', 'Z', 'Z', '7',  /* 8-13: VIN prefix  */
    0x00U,                 /* 14: checksum_lo (computed below) */
    0x00U                  /* 15: checksum_hi */
};

/*============================================================================
 *  Simulated NVM storage (static RAM in this demo)
 *===========================================================================*/
static VariantCoding_BlockType s_nvm_block;
static boolean                 s_initialized = FALSE;
static boolean                 s_coding_valid = FALSE;

/*============================================================================
 *  Static helpers
 *===========================================================================*/

/**
 * Compute the XOR checksum over bytes 0 to (VC_BLOCK_SIZE - 3).
 * Returns the XOR result in *lo. *hi always zero (reserved for CRC16 upgrade).
 */
static void compute_checksum(const VariantCoding_BlockType *block,
                              uint8_t *lo, uint8_t *hi)
{
    uint8_t       xor_val = 0U;
    const uint8_t *raw    = (const uint8_t *)(const void *)block;
    uint8_t        i;

    /* Cover bytes 0 through (VC_BLOCK_SIZE - 3): magic through vin_prefix */
    for (i = 0U; i < (VC_BLOCK_SIZE - 2U); i++)
    {
        xor_val ^= raw[i];
    }

    *lo = xor_val;
    *hi = 0x00U;
}

static boolean validate_block(const VariantCoding_BlockType *block)
{
    uint8_t chk_lo, chk_hi;
    boolean valid = FALSE;

    if (block == NULL_PTR)
    {
        return FALSE;
    }

    if (block->magic != VC_MAGIC_VALID)
    {
        printf("[VC]   Invalid magic byte: 0x%02X (expected 0x%02X)\n",
               (unsigned)block->magic, (unsigned)VC_MAGIC_VALID);
        return FALSE;
    }

    compute_checksum(block, &chk_lo, &chk_hi);

    if ((block->checksum_lo == chk_lo) && (block->checksum_hi == chk_hi))
    {
        valid = TRUE;
    }
    else
    {
        printf("[VC]   Checksum mismatch: got %02X/%02X expected %02X/%02X\n",
               (unsigned)block->checksum_lo, (unsigned)block->checksum_hi,
               (unsigned)chk_lo, (unsigned)chk_hi);
    }

    return valid;
}

static void apply_default_coding(void)
{
    uint8_t chk_lo, chk_hi;

    (void)memcpy(&s_nvm_block, k_default_coding_raw, sizeof(k_default_coding_raw));

    /* Compute correct checksum for the default block */
    compute_checksum(&s_nvm_block, &chk_lo, &chk_hi);
    s_nvm_block.checksum_lo = chk_lo;
    s_nvm_block.checksum_hi = chk_hi;

    s_coding_valid = TRUE;
    printf("[VC]   Default coding applied (EU Sedan Petrol, LHD).\n");
}

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType VC_Init(void)
{
    s_initialized = TRUE;

    /* In real HW: read s_nvm_block from NVM/EEPROM here */
    /* For simulation: check if the NVM already has a valid block or load default */
    if (validate_block(&s_nvm_block) == TRUE)
    {
        s_coding_valid = TRUE;
        printf("[VC]   Loaded valid coding block from NVM.\n");
        return E_OK;
    }

    /* NVM empty or corrupt — apply safe factory defaults */
    apply_default_coding();
    return E_NOT_OK;   /* Signal that defaults were used */
}

boolean VC_IsFeatureEnabled(uint8_t flag_byte, uint8_t bit_mask)
{
    boolean result = FALSE;

    if (s_initialized != TRUE)
    {
        return FALSE;
    }

    if (flag_byte == 0U)
    {
        result = ((s_nvm_block.feature_flags_0 & bit_mask) != 0U) ? TRUE : FALSE;
    }
    else if (flag_byte == 1U)
    {
        result = ((s_nvm_block.feature_flags_1 & bit_mask) != 0U) ? TRUE : FALSE;
    }
    else
    {
        result = FALSE;
    }

    return result;
}

uint8_t VC_GetMaxSpeedKph(void)
{
    return (s_coding_valid == TRUE) ? s_nvm_block.max_speed_kph : 0U;
}

uint8_t VC_GetVehicleClass(void)
{
    return (s_coding_valid == TRUE) ? s_nvm_block.vehicle_class : 0U;
}

uint8_t VC_GetEngineType(void)
{
    return (s_coding_valid == TRUE) ? s_nvm_block.engine_type : 0U;
}

uint8_t VC_GetMarket(void)
{
    return (s_coding_valid == TRUE) ? s_nvm_block.market : 0U;
}

uint8_t VC_GetDriveSide(void)
{
    return (s_coding_valid == TRUE) ? s_nvm_block.drive_side : 0U;
}

Std_ReturnType VC_WriteBlock(const VariantCoding_BlockType *block)
{
    if (block == NULL_PTR)
    {
        return E_NOT_OK;
    }

    if (validate_block(block) != TRUE)
    {
        printf("[VC]   Write rejected: invalid checksum or magic.\n");
        return E_NOT_OK;
    }

    (void)memcpy(&s_nvm_block, block, sizeof(VariantCoding_BlockType));
    s_coding_valid = TRUE;

    /* In real HW: flush s_nvm_block to EEPROM/Data Flash here */
    printf("[VC]   New coding block written to NVM.\n");
    return E_OK;
}

const VariantCoding_BlockType *VC_GetBlock(void)
{
    return &s_nvm_block;
}

void VC_PrintSummary(void)
{
    static const char *const class_str[]  = {"?", "Sedan", "SUV", "Van"};
    static const char *const engine_str[] = {"?", "Petrol 2.0L", "Diesel 3.0L"};
    static const char *const market_str[] = {"?", "EU", "US", "UK"};
    static const char *const side_str[]   = {"?", "LHD", "RHD"};

    uint8_t cls = (s_nvm_block.vehicle_class <= 3U) ?
                  s_nvm_block.vehicle_class : 0U;
    uint8_t eng = (s_nvm_block.engine_type   <= 2U) ?
                  s_nvm_block.engine_type : 0U;
    uint8_t mkt = (s_nvm_block.market        <= 3U) ?
                  s_nvm_block.market : 0U;
    uint8_t drv = (s_nvm_block.drive_side    <= 2U) ?
                  s_nvm_block.drive_side : 0U;

    printf("========== Variant Coding Summary ==========\n");
    printf("  Vehicle class : %s\n",   class_str[cls]);
    printf("  Engine        : %s\n",   engine_str[eng]);
    printf("  Market        : %s\n",   market_str[mkt]);
    printf("  Drive side    : %s\n",   side_str[drv]);
    printf("  Max speed     : %u kph\n", (unsigned)s_nvm_block.max_speed_kph);
    printf("  VIN prefix    : %.6s\n",   s_nvm_block.vin_prefix);
    printf("  Features      :\n");
    printf("    Heated seats  : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_HEATED_SEATS) ? "YES" : "no");
    printf("    Sunroof       : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_SUNROOF) ? "YES" : "no");
    printf("    ACC           : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_ACC) ? "YES" : "no");
    printf("    Lane keeping  : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_LANE_KEEP) ? "YES" : "no");
    printf("    Start/Stop    : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_STARTSTOP) ? "YES" : "no");
    printf("    Sport mode    : %s\n",
           VC_IsFeatureEnabled(0U, VC_FEAT0_SPORT_MODE) ? "YES" : "no");
    printf("    AWD           : %s\n",
           VC_IsFeatureEnabled(1U, VC_FEAT1_AWD) ? "YES" : "no");
    printf("    Remote start  : %s\n",
           VC_IsFeatureEnabled(1U, VC_FEAT1_REMOTE_START) ? "YES" : "no");
    printf("============================================\n");
}
