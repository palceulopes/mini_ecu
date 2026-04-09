/**
 * @file    Uds.h
 * @brief   Unified Diagnostic Services (ISO 14229) — subset implementation
 *
 * Supported services:
 *  0x10 — DiagnosticSessionControl
 *  0x11 — ECUReset
 *  0x22 — ReadDataByIdentifier  (DIDs: F101, F200, F201)
 *
 * Frames arrive via the CAN stack (ID 0x7DF), responses sent on 0x7E8.
 */

#ifndef UDS_H
#define UDS_H

#include "Std_Types.h"
#include "Ecu_Config.h"

typedef uint8_t Uds_SessionType;
#define UDS_SESSION_DEFAULT     ((Uds_SessionType)0x01U)
#define UDS_SESSION_EXTENDED    ((Uds_SessionType)0x03U)

Std_ReturnType  Uds_Init(void);

/**
 * Process a raw request buffer received from the CAN stack.
 * Builds the response into response_buf and sets *response_len.
 *
 * @param request_buf   Pointer to raw request bytes (not modified)
 * @param request_len   Number of valid bytes in request_buf
 * @param response_buf  Caller-supplied buffer for the response
 * @param response_len  OUT: number of bytes written to response_buf
 * @return E_OK if a positive or negative response was built,
 *         E_NOT_OK if request_buf or response_buf is NULL.
 */
Std_ReturnType  Uds_ProcessRequest(const uint8_t *request_buf,
                                   uint8_t        request_len,
                                   uint8_t       *response_buf,
                                   uint8_t       *response_len);

/** Called from OS 10 ms task — checks for pending CAN diag frames. */
void            Uds_MainFunction(void);

#endif /* UDS_H */
