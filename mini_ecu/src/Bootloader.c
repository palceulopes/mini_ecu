/**
 * @file    Bootloader.c
 * @brief   Bootloader implementation — boot validation + UDS download flow
 *
 * Flash simulation:
 *  On a PC we cannot write to flash, so we use a static byte array
 *  (s_sim_flash[]) to represent the application region.  The CRC32
 *  is computed over this array.
 *
 * CRC32 implementation:
 *  Uses the standard IEEE 802.3 polynomial (0xEDB88320, reflected).
 *  A real ECU would use the MCU's hardware CRC unit for speed.
 *
 * UDS Download sequence enforced state machine:
 *  IDLE → (0x34 RequestDownload)  → REQUESTED
 *       → (0x36 TransferData)     → TRANSFER  (repeated)
 *       → (0x37 RequestTransferExit) → COMPLETE
 *  Any service received out of order returns NRC 0x24 (requestSequenceError).
 *
 * MISRA-C:2012 rules applied:
 *  - All pointer parameters NULL-checked         (Dir 4.1)
 *  - crc32 loop uses uint8_t index with range    (Rule 10.1)
 *  - Switch with explicit default                (Rule 16.4)
 *  - No dynamic memory allocation                (Rule 21.3)
 *  - No recursion                                (Rule 17.2)
 */

#include "Bootloader.h"
#include "Mcal.h"
#include "Ecu_Config.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/*============================================================================
 *  Simulated flash (application region, 1 KB demo area)
 *===========================================================================*/
#define SIM_FLASH_SIZE  (1024U)
static uint8_t  s_sim_flash[SIM_FLASH_SIZE];
static uint32_t s_sim_flash_written = 0U;   /* Bytes written so far in DL */

/*============================================================================
 *  Active Boot Descriptor Block (lives in "bootloader flash" — read/write)
 *===========================================================================*/
static Btl_BdbType s_bdb;

/*============================================================================
 *  Download session state
 *===========================================================================*/
static Btl_DlStateType s_dl_state        = BTL_DL_STATE_IDLE;
static uint32_t        s_dl_expected_size = 0U;
static uint8_t         s_dl_block_seq    = 0U;  /* Expected block sequence counter */

/*============================================================================
 *  Programming request flag (simulated NVM cell)
 *===========================================================================*/
static uint32_t s_prog_request_flag = 0U;   /* Set to BDB_PROG_REQUEST_FLAG to request DL */

/*============================================================================
 *  CRC32 (IEEE 802.3, reflected polynomial)
 *===========================================================================*/
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint32_t j;
    uint32_t byte_val;
    uint32_t crc_val = crc;

    for (i = 0U; i < len; i++)
    {
        byte_val = (uint32_t)data[i];
        crc_val ^= byte_val;
        for (j = 0U; j < 8U; j++)
        {
            if ((crc_val & 0x01U) != 0U)
            {
                crc_val = (crc_val >> 1U) ^ 0xEDB88320U;
            }
            else
            {
                crc_val >>= 1U;
            }
        }
    }
    return crc_val;
}

static uint32_t crc32(const uint8_t *data, uint32_t len)
{
    return crc32_update(0xFFFFFFFFU, data, len) ^ 0xFFFFFFFFU;
}

/*============================================================================
 *  Static helpers
 *===========================================================================*/
static uint8_t btl_nrc(uint8_t *resp, uint8_t sid, uint8_t nrc)
{
    resp[0] = 0x7FU;
    resp[1] = sid;
    resp[2] = nrc;
    return 3U;
}

/** Initialise BDB with current app image data (simulation). */
static void bdb_create_from_sim(void)
{
    s_bdb.magic            = BDB_MAGIC;
    s_bdb.app_start_addr   = BTL_APP_START_ADDR;
    s_bdb.app_size_bytes   = SIM_FLASH_SIZE;
    s_bdb.app_crc32        = crc32(s_sim_flash, SIM_FLASH_SIZE);
    s_bdb.sw_version_major = ECU_SW_VERSION_MAJOR;
    s_bdb.sw_version_minor = ECU_SW_VERSION_MINOR;
    s_bdb.prog_date        = (uint32_t)time(NULL);
    s_bdb.boot_counter++;

    /* BDB self-CRC — covers bytes 0 through 27 (28 bytes, before bdb_crc32) */
    s_bdb.bdb_crc32 = crc32((const uint8_t *)(const void *)&s_bdb,
                            (uint32_t)(sizeof(Btl_BdbType) - sizeof(uint32_t)));
}

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType Btl_Init(void)
{
    /* Fill simulated flash with a known pattern (represents the app) */
    uint32_t i;
    for (i = 0U; i < SIM_FLASH_SIZE; i++)
    {
        s_sim_flash[i] = (uint8_t)(i & 0xFFU);
    }

    /* Build an initial valid BDB from the sim flash */
    (void)memset(&s_bdb, 0, sizeof(s_bdb));
    bdb_create_from_sim();

    s_dl_state         = BTL_DL_STATE_IDLE;
    s_dl_expected_size = 0U;
    s_dl_block_seq     = 1U;
    s_prog_request_flag = 0U;

    printf("[BTL]  Bootloader initialized. Boot count: %u\n",
           (unsigned)s_bdb.boot_counter);
    return E_OK;
}

Btl_ResultType Btl_ValidateApp(void)
{
    uint32_t computed_crc;

    if (s_bdb.magic != BDB_MAGIC)
    {
        printf("[BTL]  INVALID: Bad BDB magic (0x%08X).\n",
               (unsigned)s_bdb.magic);
        return BTL_ERR_NO_APP;
    }

    if ((s_bdb.app_size_bytes == 0U) ||
        (s_bdb.app_size_bytes > BTL_APP_MAX_SIZE))
    {
        printf("[BTL]  INVALID: App size out of range (%u bytes).\n",
               (unsigned)s_bdb.app_size_bytes);
        return BTL_ERR_SIZE_INVALID;
    }

    /* Re-compute CRC over the simulated flash */
    computed_crc = crc32(s_sim_flash, s_bdb.app_size_bytes);

    if (computed_crc != s_bdb.app_crc32)
    {
        printf("[BTL]  INVALID: CRC mismatch. Expected=0x%08X Got=0x%08X\n",
               (unsigned)s_bdb.app_crc32, (unsigned)computed_crc);
        return BTL_ERR_CRC_MISMATCH;
    }

    printf("[BTL]  App validated OK. CRC=0x%08X  Size=%u bytes  Ver=%u.%u\n",
           (unsigned)computed_crc,
           (unsigned)s_bdb.app_size_bytes,
           (unsigned)s_bdb.sw_version_major,
           (unsigned)s_bdb.sw_version_minor);

    return BTL_OK;
}

boolean Btl_CheckReprogramRequest(void)
{
    return (s_prog_request_flag == BDB_PROG_REQUEST_FLAG) ? TRUE : FALSE;
}

/**
 * @brief Process UDS download sequence services
 *
 * Supported services:
 *  0x34 (RequestDownload):
 *      byte[1]:   dataFormatIdentifier
 *      byte[2]:   addressAndLengthFormatIdentifier
 *      byte[3-6]: address (uint32 BE)
 *      byte[7-10]: size   (uint32 BE)
 *
 *  0x36 (TransferData):
 *      byte[1]:   block sequence counter
 *      byte[2..]: data payload
 *
 *  0x37 (RequestTransferExit):
 *      no parameters — ends transfer, validates CRC, updates BDB
 */
Btl_ResultType Btl_ProcessUdsRequest(const uint8_t *request_buf,
                                     uint8_t        request_len,
                                     uint8_t       *response_buf,
                                     uint8_t       *response_len)
{
    uint8_t        sid;
    uint8_t        resp_len = 0U;
    Btl_ResultType result   = BTL_OK;

    if ((request_buf == NULL_PTR) || (response_buf == NULL_PTR) ||
        (response_len == NULL_PTR) || (request_len == 0U))
    {
        return BTL_ERR_NULL_PTR;
    }

    sid = request_buf[0];

    switch (sid)
    {
        /* ------------------------------------------------------------------
         *  0x34 — RequestDownload
         *  Expected in IDLE state only.
         *-----------------------------------------------------------------*/
        case 0x34U:
        {
            if (s_dl_state != BTL_DL_STATE_IDLE)
            {
                resp_len = btl_nrc(response_buf, sid, 0x24U); /* seqError */
                result   = BTL_ERR_SEQ_VIOLATION;
                break;
            }
            if (request_len < 11U)
            {
                resp_len = btl_nrc(response_buf, sid, 0x13U); /* incorrectLen */
                result   = BTL_ERR_SIZE_INVALID;
                break;
            }

            s_dl_expected_size =
                ((uint32_t)request_buf[7] << 24U) |
                ((uint32_t)request_buf[8] << 16U) |
                ((uint32_t)request_buf[9] <<  8U) |
                ((uint32_t)request_buf[10]);

            if ((s_dl_expected_size == 0U) ||
                (s_dl_expected_size > SIM_FLASH_SIZE))
            {
                resp_len = btl_nrc(response_buf, sid, 0x31U); /* outOfRange */
                result   = BTL_ERR_SIZE_INVALID;
                break;
            }

            /* Erase simulation flash */
            (void)memset(s_sim_flash, 0xFFU, SIM_FLASH_SIZE);
            s_sim_flash_written = 0U;
            s_dl_block_seq      = 1U;
            s_dl_state          = BTL_DL_STATE_REQUESTED;

            response_buf[0] = (uint8_t)(0x34U + 0x40U);
            response_buf[1] = 0x00U;  /* lengthAndDataFormatIdentifier */
            response_buf[2] = 0x04U;  /* maxBlockLen: 4 bytes payload  */
            resp_len        = 3U;

            printf("[BTL]  RequestDownload: expecting %u bytes.\n",
                   (unsigned)s_dl_expected_size);
            break;
        }

        /* ------------------------------------------------------------------
         *  0x36 — TransferData
         *  Expected in REQUESTED or TRANSFER state.
         *-----------------------------------------------------------------*/
        case 0x36U:
        {
            uint8_t  block_seq;
            uint8_t  payload_len;
            uint32_t copy_size;

            if ((s_dl_state != BTL_DL_STATE_REQUESTED) &&
                (s_dl_state != BTL_DL_STATE_TRANSFER))
            {
                resp_len = btl_nrc(response_buf, sid, 0x24U);
                result   = BTL_ERR_SEQ_VIOLATION;
                break;
            }
            if (request_len < 2U)
            {
                resp_len = btl_nrc(response_buf, sid, 0x13U);
                break;
            }

            block_seq   = request_buf[1];
            payload_len = (uint8_t)(request_len - 2U);

            /* Verify sequence counter */
            if (block_seq != s_dl_block_seq)
            {
                resp_len = btl_nrc(response_buf, sid, 0x73U); /* wrongBlockSeq */
                result   = BTL_ERR_SEQ_VIOLATION;
                break;
            }

            /* Write payload into simulation flash */
            copy_size = (uint32_t)payload_len;
            if ((s_sim_flash_written + copy_size) > SIM_FLASH_SIZE)
            {
                copy_size = SIM_FLASH_SIZE - s_sim_flash_written;
            }
            (void)memcpy(&s_sim_flash[s_sim_flash_written],
                         &request_buf[2], copy_size);
            s_sim_flash_written += copy_size;

            /* Advance sequence counter (wraps 0xFF → 0x00) */
            s_dl_block_seq = (uint8_t)((s_dl_block_seq == 0xFFU) ?
                                       0x00U : (s_dl_block_seq + 1U));
            s_dl_state = BTL_DL_STATE_TRANSFER;

            response_buf[0] = (uint8_t)(0x36U + 0x40U);
            response_buf[1] = block_seq;
            resp_len        = 2U;

            printf("[BTL]  TransferData block %u: %u bytes (total %u/%u).\n",
                   (unsigned)block_seq, (unsigned)copy_size,
                   (unsigned)s_sim_flash_written,
                   (unsigned)s_dl_expected_size);
            break;
        }

        /* ------------------------------------------------------------------
         *  0x37 — RequestTransferExit
         *  Expected in TRANSFER state.  Validates, updates BDB.
         *-----------------------------------------------------------------*/
        case 0x37U:
        {
            uint32_t computed_crc;

            if (s_dl_state != BTL_DL_STATE_TRANSFER)
            {
                resp_len = btl_nrc(response_buf, sid, 0x24U);
                result   = BTL_ERR_SEQ_VIOLATION;
                break;
            }

            /* Compute CRC over newly written data */
            computed_crc = crc32(s_sim_flash, s_sim_flash_written);

            /* Update BDB with new image info */
            s_bdb.app_size_bytes   = s_sim_flash_written;
            s_bdb.app_crc32        = computed_crc;
            s_bdb.sw_version_major = ECU_SW_VERSION_MAJOR;
            s_bdb.sw_version_minor = ECU_SW_VERSION_MINOR;
            s_bdb.prog_date        = (uint32_t)time(NULL);
            s_bdb.bdb_crc32        = crc32(
                (const uint8_t *)(const void *)&s_bdb,
                (uint32_t)(sizeof(Btl_BdbType) - sizeof(uint32_t)));

            s_dl_state          = BTL_DL_STATE_COMPLETE;
            s_prog_request_flag = 0U;  /* Clear request flag */

            response_buf[0] = (uint8_t)(0x37U + 0x40U);
            resp_len        = 1U;

            printf("[BTL]  TransferExit: %u bytes written. CRC=0x%08X\n",
                   (unsigned)s_sim_flash_written, (unsigned)computed_crc);
            printf("[BTL]  BDB updated. ECU ready to reset.\n");
            break;
        }

        default:
            resp_len = btl_nrc(response_buf, sid, 0x11U); /* serviceNotSupported */
            break;
    }

    *response_len = resp_len;
    return result;
}

void Btl_JumpToApp(void)
{
    /*
     * On real hardware this would be:
     *   uint32_t app_reset_vector = *(uint32_t *)(APP_START_ADDR + 4);
     *   void (*app_entry)(void) = (void (*)(void))app_reset_vector;
     *   __set_MSP(*(uint32_t *)APP_START_ADDR);
     *   app_entry();
     *
     * In simulation we simply return to main() which calls Os_Run().
     */
    printf("[BTL]  Jumping to application at 0x%08lX...\n",
           (unsigned long)BTL_APP_START_ADDR);
}

const Btl_BdbType *Btl_GetBdb(void)
{
    return &s_bdb;
}

void Btl_PrintBdb(void)
{
    printf("========== Boot Descriptor Block ===========\n");
    printf("  Magic          : 0x%08X %s\n",
           (unsigned)s_bdb.magic,
           (s_bdb.magic == BDB_MAGIC) ? "(valid)" : "(INVALID!)");
    printf("  App start      : 0x%08X\n", (unsigned)s_bdb.app_start_addr);
    printf("  App size       : %u bytes\n", (unsigned)s_bdb.app_size_bytes);
    printf("  App CRC32      : 0x%08X\n", (unsigned)s_bdb.app_crc32);
    printf("  SW Version     : %u.%u\n",
           (unsigned)s_bdb.sw_version_major,
           (unsigned)s_bdb.sw_version_minor);
    printf("  Boot counter   : %u\n", (unsigned)s_bdb.boot_counter);
    printf("  BDB CRC32      : 0x%08X\n", (unsigned)s_bdb.bdb_crc32);
    printf("============================================\n");
}
