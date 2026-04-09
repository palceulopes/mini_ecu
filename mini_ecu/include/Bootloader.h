/**
 * @file    Bootloader.h
 * @brief   Software Bootloader — Boot Descriptor Block + validity check +
 *          UDS-triggered reprogramming flow simulation
 *
 * -----------------------------------------------------------------------
 * CONCEPT: Why does an ECU need a bootloader?
 * -----------------------------------------------------------------------
 *
 *  Every ECU needs to be updated in the field (bug fixes, new features,
 *  legal recalls). The bootloader provides a *trusted, minimal* piece of
 *  code that:
 *
 *  1. Always runs first after reset (lives at the lowest flash address).
 *  2. Validates the application via a CRC32 checksum stored in the
 *     Boot Descriptor Block (BDB) — never executes corrupt software.
 *  3. Optionally receives new firmware over CAN using the standardised
 *     UDS download sequence (ISO 14229 services 0x34/0x36/0x37).
 *  4. Writes the new firmware to flash, recalculates the CRC, updates
 *     the BDB, and resets.
 *
 * -----------------------------------------------------------------------
 * Flash memory layout (simulated):
 *
 *  Address       Region
 *  -----------   -----------------------------------------------
 *  0x00000000    Bootloader code       (read-only, never erased)
 *  0x00008000    Boot Descriptor Block (written by bootloader after OTA)
 *  0x00009000    Application code      (erased and reprogrammed)
 *  0x000FFFFF    End of flash
 *
 * -----------------------------------------------------------------------
 * Boot Descriptor Block fields:
 *
 *  Offset  Size  Field              Meaning
 *  ------  ----  -----------------  ------------------------------------
 *   0       4    magic              0xB007AB1E = valid descriptor
 *   4       4    app_start_addr     Where the app code begins
 *   8       4    app_size_bytes     Length of app image
 *  12       4    app_crc32          CRC32 over app region
 *  16       2    sw_version_major   Application version
 *  18       2    sw_version_minor
 *  20       4    prog_date          Unix timestamp of last flashing
 *  24       4    boot_counter       Incremented each boot (life counter)
 *  28       4    bdb_crc32          CRC32 over bytes 0-27 of the BDB
 *
 * -----------------------------------------------------------------------
 * Boot sequence (simulated as a function call sequence):
 *
 *   Reset
 *    │
 *    ▼
 *   Btl_Init()                   Hardware init (clocks, CAN, NVM)
 *    │
 *    ▼
 *   Btl_CheckReprogramRequest()  Did tester send UDS 0x10 0x02?
 *    │                           Is PROG_REQUEST_FLAG set in NVM?
 *    │
 *    ├─── YES ─→ Btl_RunDownloadSequence()  ← wait for UDS 0x34/0x36/0x37
 *    │
 *    └─── NO ──→ Btl_ValidateApp()          ← check BDB magic + CRC32
 *                 │
 *                 ├─── VALID   ─→ Btl_JumpToApp()
 *                 └─── INVALID ─→ Btl_RunDownloadSequence()
 *
 * -----------------------------------------------------------------------
 */

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "Std_Types.h"

/*============================================================================
 *  Magic constants
 *===========================================================================*/
#define BDB_MAGIC               (0xB007AB1EU)  /* "BOOT ABLE" */
#define BDB_PROG_REQUEST_FLAG   (0xDEADC0DEU)  /* Written to NVM by tester */

/* Simulated flash addresses (on PC these map to a byte array) */
#define BTL_APP_START_ADDR      (0x00009000UL)
#define BTL_APP_MAX_SIZE        (0x000F7000UL) /* ~988 KB */

/*============================================================================
 *  Boot Descriptor Block
 *===========================================================================*/
typedef struct {
    uint32_t magic;
    uint32_t app_start_addr;
    uint32_t app_size_bytes;
    uint32_t app_crc32;
    uint16_t sw_version_major;
    uint16_t sw_version_minor;
    uint32_t prog_date;          /* Unix timestamp */
    uint32_t boot_counter;
    uint32_t bdb_crc32;          /* CRC32 over bytes 0–27 */
} Btl_BdbType;

/*============================================================================
 *  Download session state (tracks the UDS 0x34/0x36/0x37 sequence)
 *===========================================================================*/
typedef uint8_t Btl_DlStateType;
#define BTL_DL_STATE_IDLE       ((Btl_DlStateType)0U)
#define BTL_DL_STATE_REQUESTED  ((Btl_DlStateType)1U)  /* 0x34 received */
#define BTL_DL_STATE_TRANSFER   ((Btl_DlStateType)2U)  /* 0x36 in progress */
#define BTL_DL_STATE_COMPLETE   ((Btl_DlStateType)3U)  /* 0x37 received */
#define BTL_DL_STATE_ERROR      ((Btl_DlStateType)4U)

/*============================================================================
 *  Bootloader result codes
 *===========================================================================*/
typedef uint8_t Btl_ResultType;
#define BTL_OK                  ((Btl_ResultType)0U)
#define BTL_ERR_NO_APP          ((Btl_ResultType)1U)
#define BTL_ERR_CRC_MISMATCH    ((Btl_ResultType)2U)
#define BTL_ERR_SIZE_INVALID    ((Btl_ResultType)3U)
#define BTL_ERR_NULL_PTR        ((Btl_ResultType)4U)
#define BTL_ERR_SEQ_VIOLATION   ((Btl_ResultType)5U)  /* Wrong service order */

/*============================================================================
 *  Public Bootloader API
 *===========================================================================*/

/** Initialise bootloader — hardware, CAN, NVM, boot counter increment. */
Std_ReturnType      Btl_Init(void);

/**
 * Validate the application image in flash.
 * Reads the BDB, checks magic, recalculates CRC32 over the app region.
 *
 * @return BTL_OK             — app is valid, safe to jump
 * @return BTL_ERR_NO_APP     — BDB magic invalid
 * @return BTL_ERR_CRC_MISMATCH — CRC mismatch; app is corrupt
 */
Btl_ResultType      Btl_ValidateApp(void);

/**
 * Check if a programming session was requested.
 * In real HW this reads a specific NVM flag cell that a tester wrote
 * before triggering the reset (via UDS 0x11 EcuReset).
 *
 * @return TRUE  — enter download mode
 * @return FALSE — proceed with normal boot
 */
boolean             Btl_CheckReprogramRequest(void);

/**
 * Simulate the UDS firmware download sequence.
 * Accepts raw UDS PDUs and state-machines through 0x34 → 0x36 → 0x37.
 *
 * @param request_buf  Raw UDS request bytes
 * @param request_len  Number of valid bytes
 * @param response_buf Caller buffer for response
 * @param response_len OUT: number of bytes written
 * @return BTL_OK or an error code
 */
Btl_ResultType      Btl_ProcessUdsRequest(const uint8_t *request_buf,
                                          uint8_t        request_len,
                                          uint8_t       *response_buf,
                                          uint8_t       *response_len);

/**
 * Jump to the application. On real HW this sets the stack pointer to the
 * app vector table and jumps to the reset vector.
 * In simulation it returns to main() which then calls Os_Run().
 */
void                Btl_JumpToApp(void);

/** Get a pointer to the currently active Boot Descriptor Block. */
const Btl_BdbType  *Btl_GetBdb(void);

/** Print a human-readable BDB summary (simulation only). */
void                Btl_PrintBdb(void);

#endif /* BOOTLOADER_H */
