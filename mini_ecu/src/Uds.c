/**
 * @file    Uds.c
 * @brief   Unified Diagnostic Services (ISO 14229) — subset implementation
 *
 * Supported services:
 *  0x10 — DiagnosticSessionControl  (switch between Default / Extended)
 *  0x11 — ECUReset                  (triggers soft reset in simulation)
 *  0x22 — ReadDataByIdentifier      (DIDs: F101, F200, F201)
 *  0x2E — WriteDataByIdentifier     (DID: VC_CODING = 0xF190 variant block)
 *
 * Frame format on CAN: ISO 15765-2 Single Frame (SF) only (DLC ≤ 7).
 * Multi-frame (ISO-TP) is outside the scope of this demo.
 *
 * Positive response: SID + 0x40 as first byte (e.g. 0x62 for 0x22).
 * Negative response: 0x7F <SID> <NRC>.
 *
 * MISRA-C:2012 rules applied:
 *  - All pointer parameters NULL-checked before dereference   (Dir 4.1)
 *  - switch/case has explicit default                         (Rule 16.4)
 *  - No dynamic memory allocation                            (Rule 21.3)
 *  - No recursion                                            (Rule 17.2)
 */

#include "Uds.h"
#include "Can_Driver.h"
#include "EngineCtrl.h"
#include "VariantCoding.h"
#include "Ecu_Config.h"

#include <stdio.h>
#include <string.h>

/*============================================================================
 *  Module state
 *===========================================================================*/
static Uds_SessionType s_session    = UDS_SESSION_DEFAULT;
static boolean         s_initialized = FALSE;

/*============================================================================
 *  Static helpers
 *===========================================================================*/

/** Build a negative response into buf. Returns the length (3 bytes). */
static uint8_t build_nrc(uint8_t *buf, uint8_t sid, uint8_t nrc)
{
    buf[0] = 0x7FU;
    buf[1] = sid;
    buf[2] = nrc;
    return 3U;
}

/*----------------------------------------------------------------------------
 *  Service 0x22 — ReadDataByIdentifier
 *---------------------------------------------------------------------------*/
static uint8_t handle_rdbi(const uint8_t *req, uint8_t req_len,
                            uint8_t *resp)
{
    uint16_t did;
    uint8_t  len = 0U;

    if (req_len < 3U)   /* SID + 2 DID bytes */
    {
        len = build_nrc(resp, UDS_SID_READ_DATA_BY_ID,
                        UDS_NRC_REQ_OUT_OF_RANGE);
        return len;
    }

    did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);

    resp[0] = (uint8_t)(UDS_SID_READ_DATA_BY_ID + 0x40U);  /* positive */
    resp[1] = req[1];   /* echo DID high byte */
    resp[2] = req[2];   /* echo DID low  byte */

    switch (did)
    {
        case UDS_DID_ECU_INFO:
            /* Returns SW version: Major, Minor, Patch */
            resp[3] = (uint8_t)ECU_SW_VERSION_MAJOR;
            resp[4] = (uint8_t)ECU_SW_VERSION_MINOR;
            resp[5] = (uint8_t)ECU_SW_VERSION_PATCH;
            len = 6U;
            break;

        case UDS_DID_ENGINE_RPM:
            /* Returns current RPM as uint16 big-endian */
            resp[3] = (uint8_t)((g_EngCtrl_Data.rpm >> 8U) & 0xFFU);
            resp[4] = (uint8_t)(g_EngCtrl_Data.rpm & 0xFFU);
            len = 5U;
            break;

        case UDS_DID_COOLANT_TEMP:
            /* Returns coolant temp as int16 big-endian (0.1 °C units) */
            resp[3] = (uint8_t)(((uint16_t)g_EngCtrl_Data.coolant_temp >> 8U) & 0xFFU);
            resp[4] = (uint8_t)((uint16_t)g_EngCtrl_Data.coolant_temp & 0xFFU);
            len = 5U;
            break;

        default:
            len = build_nrc(resp, UDS_SID_READ_DATA_BY_ID,
                            UDS_NRC_REQ_OUT_OF_RANGE);
            break;
    }

    return len;
}

/*----------------------------------------------------------------------------
 *  Service 0x2E — WriteDataByIdentifier (Variant Coding)
 *---------------------------------------------------------------------------*/
static uint8_t handle_wdbi(const uint8_t *req, uint8_t req_len,
                            uint8_t *resp)
{
    uint16_t did;
    uint8_t  len = 0U;

    /* Minimum: SID(1) + DID(2) + at least 1 data byte */
    if (req_len < 4U)
    {
        len = build_nrc(resp, 0x2EU, UDS_NRC_REQ_OUT_OF_RANGE);
        return len;
    }

    /* Only allowed in Extended Diagnostic Session */
    if (s_session != UDS_SESSION_EXTENDED)
    {
        len = build_nrc(resp, 0x2EU, 0x31U); /* requestOutOfRange */
        return len;
    }

    did = (uint16_t)(((uint16_t)req[1] << 8U) | (uint16_t)req[2]);

    if (did == 0xF190U)  /* VC coding block DID */
    {
        /* Expect exactly VC_BLOCK_SIZE bytes after SID+DID */
        if (req_len >= (uint8_t)(3U + VC_BLOCK_SIZE))
        {
            const VariantCoding_BlockType *block =
                (const VariantCoding_BlockType *)(const void *)&req[3];

            if (VC_WriteBlock(block) == E_OK)
            {
                resp[0] = (uint8_t)(0x2EU + 0x40U);
                resp[1] = req[1];
                resp[2] = req[2];
                len = 3U;
                printf("[UDS]  VariantCoding written via 0x2E (DID 0xF190).\n");
            }
            else
            {
                len = build_nrc(resp, 0x2EU, 0x22U); /* conditionsNotCorrect */
            }
        }
        else
        {
            len = build_nrc(resp, 0x2EU, UDS_NRC_REQ_OUT_OF_RANGE);
        }
    }
    else
    {
        len = build_nrc(resp, 0x2EU, UDS_NRC_REQ_OUT_OF_RANGE);
    }

    return len;
}

/*----------------------------------------------------------------------------
 *  Service 0x10 — DiagnosticSessionControl
 *---------------------------------------------------------------------------*/
static uint8_t handle_session_ctrl(const uint8_t *req, uint8_t req_len,
                                   uint8_t *resp)
{
    uint8_t len = 0U;

    if (req_len < 2U)
    {
        len = build_nrc(resp, UDS_SID_SESSION_CTRL, UDS_NRC_REQ_OUT_OF_RANGE);
        return len;
    }

    switch (req[1])
    {
        case UDS_SESSION_DEFAULT:
            s_session = UDS_SESSION_DEFAULT;
            resp[0]   = (uint8_t)(UDS_SID_SESSION_CTRL + 0x40U);
            resp[1]   = req[1];
            len       = 2U;
            printf("[UDS]  Session: Default.\n");
            break;

        case UDS_SESSION_EXTENDED:
            s_session = UDS_SESSION_EXTENDED;
            resp[0]   = (uint8_t)(UDS_SID_SESSION_CTRL + 0x40U);
            resp[1]   = req[1];
            len       = 2U;
            printf("[UDS]  Session: Extended Diagnostic.\n");
            break;

        default:
            len = build_nrc(resp, UDS_SID_SESSION_CTRL, UDS_NRC_SUBF_NOT_SUP);
            break;
    }

    return len;
}

/*----------------------------------------------------------------------------
 *  Service 0x11 — ECUReset
 *---------------------------------------------------------------------------*/
static uint8_t handle_ecu_reset(const uint8_t *req, uint8_t req_len,
                                uint8_t *resp)
{
    uint8_t len = 0U;

    if (req_len < 2U)
    {
        len = build_nrc(resp, UDS_SID_ECU_RESET, UDS_NRC_REQ_OUT_OF_RANGE);
        return len;
    }

    /* Sub-function 0x01 = hardReset, 0x03 = softReset */
    if ((req[1] == 0x01U) || (req[1] == 0x03U))
    {
        resp[0] = (uint8_t)(UDS_SID_ECU_RESET + 0x40U);
        resp[1] = req[1];
        len     = 2U;
        printf("[UDS]  ECU Reset requested (sub=0x%02X). Simulating reset...\n",
               (unsigned)req[1]);
        /* On real HW: trigger watchdog or NVIC_SystemReset() */
    }
    else
    {
        len = build_nrc(resp, UDS_SID_ECU_RESET, UDS_NRC_SUBF_NOT_SUP);
    }

    return len;
}

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType Uds_Init(void)
{
    s_session     = UDS_SESSION_DEFAULT;
    s_initialized = TRUE;
    printf("[UDS]  Diagnostics initialized. Session: Default.\n");
    return E_OK;
}

Std_ReturnType Uds_ProcessRequest(const uint8_t *request_buf,
                                  uint8_t        request_len,
                                  uint8_t       *response_buf,
                                  uint8_t       *response_len)
{
    uint8_t resp_len = 0U;

    /* Null pointer guard — security boundary */
    if ((request_buf == NULL_PTR) || (response_buf == NULL_PTR) ||
        (response_len == NULL_PTR) || (request_len == 0U))
    {
        return E_NOT_OK;
    }

    printf("[UDS]  Request SID=0x%02X len=%u\n",
           (unsigned)request_buf[0], (unsigned)request_len);

    switch (request_buf[0])
    {
        case UDS_SID_SESSION_CTRL:
            resp_len = handle_session_ctrl(request_buf, request_len,
                                           response_buf);
            break;

        case UDS_SID_ECU_RESET:
            resp_len = handle_ecu_reset(request_buf, request_len,
                                        response_buf);
            break;

        case UDS_SID_READ_DATA_BY_ID:
            resp_len = handle_rdbi(request_buf, request_len, response_buf);
            break;

        case 0x2EU:  /* WriteDataByIdentifier */
            resp_len = handle_wdbi(request_buf, request_len, response_buf);
            break;

        default:
            resp_len = build_nrc(response_buf, request_buf[0],
                                 UDS_NRC_SERVICE_NOT_SUP);
            break;
    }

    *response_len = resp_len;
    return E_OK;
}

void Uds_MainFunction(void)
{
    Can_FrameType frame;
    uint8_t       response_buf[UDS_MAX_RESPONSE_LEN];
    uint8_t       response_len = 0U;
    Can_FrameType resp_frame;

    if (s_initialized != TRUE)
    {
        return;
    }

    /* Check for incoming diagnostic frames */
    while (Can_Receive(CAN_CHANNEL_0, &frame) == E_OK)
    {
        if (frame.id != CAN_ID_DIAG_REQUEST)
        {
            /* Re-inject non-diagnostic frames back so other modules see them.
             * A real AUTOSAR COM module would use separate RX PDU routing. */
#if (ECU_PLATFORM_NATIVE == 1U)
            (void)Can_InjectFrame(&frame);
#endif
            break;
        }

        if (frame.dlc < 1U)
        {
            continue;
        }

        (void)Uds_ProcessRequest(frame.data, frame.dlc,
                                 response_buf, &response_len);

        /* Send response on diagnostic response ID */
        if (response_len > 0U)
        {
            resp_frame.id  = CAN_ID_DIAG_RESPONSE;
            resp_frame.dlc = (response_len <= CAN_FRAME_DLC_MAX) ?
                             response_len : CAN_FRAME_DLC_MAX;
            (void)memcpy(resp_frame.data, response_buf, resp_frame.dlc);
            (void)Can_Transmit(CAN_CHANNEL_0, &resp_frame);
        }
    }
}
