/**
 * @file    main.c
 * @brief   Mini ECU — top-level entry point
 *
 * Start-up sequence:
 *
 *  1.  Bootloader phase
 *      a. Btl_Init()                   — hardware bring-up, boot counter
 *      b. Btl_CheckReprogramRequest()  — is a new firmware waiting?
 *      c. Btl_ValidateApp()            — CRC check on application image
 *      d. Btl_PrintBdb()               — show descriptor block
 *      e. Btl_JumpToApp()              — hand over to application
 *
 *  2.  Application initialisation
 *      a. SysTimer_Init / Gpio_Init / Adc_Init / Pwm_Init / Wdg_Init
 *      b. Can_Init
 *      c. VariantCoding — read NVM coding block
 *      d. EngCtrl_Init
 *      e. Uds_Init
 *      f. Os_Init + register tasks
 *
 *  3.  Demo CAN injection
 *      - Inject a throttle command frame (simulates another ECU sending)
 *      - Inject a UDS diagnostic request (simulate workshop tester)
 *
 *  4.  Os_Run(max_cycles) — runs the scheduler for a limited number of
 *      cycles so the demo terminates on its own.
 *
 *  5.  Demo sequence: UDS firmware download simulation
 *
 * Compile with:
 *   gcc -Wall -Wextra -std=c99 -Iinclude src\*.c -o mini_ecu
 *   (or use the provided Makefile)
 */

#include "Std_Types.h"
#include "Ecu_Config.h"
#include "Mcal.h"
#include "Can_Driver.h"
#include "Os.h"
#include "EngineCtrl.h"
#include "Uds.h"
#include "Bootloader.h"
#include "VariantCoding.h"

#include <stdio.h>
#include <string.h>

/*============================================================================
 *  Forward declarations for OS task wrappers
 *===========================================================================*/
static void Task_2ms(void);
static void Task_10ms(void);
static void Task_100ms(void);

/*============================================================================
 *  2 ms task — CAN main function (flush TX, service RX)
 *===========================================================================*/
static void Task_2ms(void)
{
    Can_MainFunction();
}

/*============================================================================
 *  10 ms task — engine sensing/control + diagnostics
 *===========================================================================*/
static void Task_10ms(void)
{
    EngCtrl_Runnable_10ms();
    Uds_MainFunction();
}

/*============================================================================
 *  100 ms task — CAN engine status broadcast
 *===========================================================================*/
static void Task_100ms(void)
{
    EngCtrl_Runnable_100ms();
}

/*============================================================================
 *  Demo: inject test CAN frames into the RX queue
 *===========================================================================*/
#if (ECU_PLATFORM_NATIVE == 1U)
static void demo_inject_can_frames(void)
{
    Can_FrameType frame;

    printf("\n--- Injecting demo CAN frames ---\n");

    /* Throttle command: 55% throttle from another ECU */
    (void)memset(&frame, 0, sizeof(frame));
    frame.id      = CAN_ID_THROTTLE_CMD;
    frame.dlc     = 1U;
    frame.data[0] = 55U;
    (void)Can_InjectFrame(&frame);

    /* UDS: DiagnosticSessionControl — switch to Extended session */
    (void)memset(&frame, 0, sizeof(frame));
    frame.id      = CAN_ID_DIAG_REQUEST;
    frame.dlc     = 2U;
    frame.data[0] = UDS_SID_SESSION_CTRL;
    frame.data[1] = UDS_SESSION_EXTENDED;
    (void)Can_InjectFrame(&frame);

    /* UDS: ReadDataByIdentifier — read ECU SW version */
    (void)memset(&frame, 0, sizeof(frame));
    frame.id      = CAN_ID_DIAG_REQUEST;
    frame.dlc     = 3U;
    frame.data[0] = UDS_SID_READ_DATA_BY_ID;
    frame.data[1] = (uint8_t)((UDS_DID_ECU_INFO >> 8U) & 0xFFU);
    frame.data[2] = (uint8_t)(UDS_DID_ECU_INFO & 0xFFU);
    (void)Can_InjectFrame(&frame);

    /* UDS: ReadDataByIdentifier — read coolant temperature */
    (void)memset(&frame, 0, sizeof(frame));
    frame.id      = CAN_ID_DIAG_REQUEST;
    frame.dlc     = 3U;
    frame.data[0] = UDS_SID_READ_DATA_BY_ID;
    frame.data[1] = (uint8_t)((UDS_DID_COOLANT_TEMP >> 8U) & 0xFFU);
    frame.data[2] = (uint8_t)(UDS_DID_COOLANT_TEMP & 0xFFU);
    (void)Can_InjectFrame(&frame);

    printf("--- End of injected frames ---\n\n");
}

/*============================================================================
 *  Demo: simulate a full UDS firmware download sequence
 *===========================================================================*/
static void demo_uds_download(void)
{
    uint8_t request[16];
    uint8_t response[16];
    uint8_t resp_len = 0U;
    uint8_t i;

    printf("\n===== Simulating UDS Firmware Download =====\n");

    /* Step 1: 0x34 RequestDownload — address=0x9000, size=16 bytes */
    (void)memset(request, 0, sizeof(request));
    request[0] = 0x34U;
    request[1] = 0x00U;   /* dataFormatIdentifier */
    request[2] = 0x44U;   /* addressAndLengthFormatIdentifier (4+4 bytes) */
    request[3] = 0x00U; request[4] = 0x00U;
    request[5] = 0x90U; request[6] = 0x00U;  /* address BE: 0x00009000 */
    request[7] = 0x00U; request[8] = 0x00U;
    request[9] = 0x00U; request[10] = 0x10U; /* size BE: 16 bytes */
    (void)Btl_ProcessUdsRequest(request, 11U, response, &resp_len);
    printf("  0x34 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Step 2: 0x36 TransferData — block 1, 4 bytes payload */
    (void)memset(request, 0, sizeof(request));
    request[0] = 0x36U;
    request[1] = 0x01U;             /* block sequence counter */
    request[2] = 0xDE; request[3] = 0xAD;
    request[4] = 0xBE; request[5] = 0xEF;
    (void)Btl_ProcessUdsRequest(request, 6U, response, &resp_len);
    printf("  0x36 block 1 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Step 3: 0x36 TransferData — block 2, 4 bytes payload */
    request[1] = 0x02U;
    request[2] = 0xCA; request[3] = 0xFE;
    request[4] = 0xBA; request[5] = 0xBE;
    (void)Btl_ProcessUdsRequest(request, 6U, response, &resp_len);
    printf("  0x36 block 2 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Step 4: 0x36 TransferData — block 3, 4 bytes payload */
    request[1] = 0x03U;
    request[2] = 0x12U; request[3] = 0x34U;
    request[4] = 0x56U; request[5] = 0x78U;
    (void)Btl_ProcessUdsRequest(request, 6U, response, &resp_len);
    printf("  0x36 block 3 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Step 5: 0x36 TransferData — block 4, 4 bytes payload */
    request[1] = 0x04U;
    request[2] = 0x9AU; request[3] = 0xBCU;
    request[4] = 0xDEU; request[5] = 0xF0U;
    (void)Btl_ProcessUdsRequest(request, 6U, response, &resp_len);
    printf("  0x36 block 4 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Step 6: 0x37 RequestTransferExit */
    (void)memset(request, 0, sizeof(request));
    request[0] = 0x37U;
    (void)Btl_ProcessUdsRequest(request, 1U, response, &resp_len);
    printf("  0x37 response (%u bytes):", (unsigned)resp_len);
    for (i = 0U; i < resp_len; i++) { printf(" %02X", (unsigned)response[i]); }
    printf("\n");

    /* Re-validate the newly "downloaded" image */
    printf("\nRe-validating app after download:\n");
    (void)Btl_ValidateApp();
    Btl_PrintBdb();

    printf("===== Download sequence complete =====\n\n");
}
#endif /* ECU_PLATFORM_NATIVE */

/*============================================================================
 *  main
 *===========================================================================*/
int main(void)
{
    Btl_ResultType btl_result;

    printf("============================================================\n");
    printf("   Mini ECU Simulation  v%u.%u.%u\n",
           (unsigned)ECU_SW_VERSION_MAJOR,
           (unsigned)ECU_SW_VERSION_MINOR,
           (unsigned)ECU_SW_VERSION_PATCH);
    printf("   Node address: 0x%02X\n", (unsigned)ECU_NODE_ADDRESS);
    printf("============================================================\n\n");

    /*------------------------------------------------------------------------
     *  PHASE 1: BOOTLOADER
     *-----------------------------------------------------------------------*/
    printf("--- BOOTLOADER PHASE ---\n");
    (void)Btl_Init();

    if (Btl_CheckReprogramRequest() == TRUE)
    {
        printf("[BTL]  Programming session requested — entering download mode.\n");
        /* In a real system we would enter download mode here and never reach
           the application. For demo, we fall through. */
    }

    btl_result = Btl_ValidateApp();
    Btl_PrintBdb();

    if (btl_result != BTL_OK)
    {
        printf("[BTL]  App invalid (code=%u) — would enter emergency download.\n",
               (unsigned)btl_result);
        /* Real system: stay in DL mode, wait for CAN firmware */
    }

    Btl_JumpToApp();

    /*------------------------------------------------------------------------
     *  PHASE 2: APPLICATION INIT
     *-----------------------------------------------------------------------*/
    printf("\n--- APPLICATION INIT ---\n");

    /* MCAL */
    (void)SysTimer_Init();
    (void)Gpio_Init();
    (void)Adc_Init();
    (void)Pwm_Init();
    (void)Wdg_Init();

    /* CAN */
    (void)Can_Init();

    /* Variant Coding — read NVM, gate features */
    {
        Std_ReturnType vc_ret = VC_Init();
        if (vc_ret != E_OK)
        {
            printf("[MAIN] VariantCoding: Using factory defaults.\n");
        }
        VC_PrintSummary();
    }

    /* Application SWCs */
    (void)EngCtrl_Init();
    (void)Uds_Init();

    /* OS scheduler */
    (void)Os_Init();
    (void)Os_RegisterTask(OS_TASK_ID_2MS,   Task_2ms,   OS_TASK_2MS_PERIOD);
    (void)Os_RegisterTask(OS_TASK_ID_10MS,  Task_10ms,  OS_TASK_10MS_PERIOD);
    (void)Os_RegisterTask(OS_TASK_ID_100MS, Task_100ms, OS_TASK_100MS_PERIOD);

    /*------------------------------------------------------------------------
     *  PHASE 3: DEMO — inject CAN frames to exercise the system
     *-----------------------------------------------------------------------*/
#if (ECU_PLATFORM_NATIVE == 1U)
    demo_inject_can_frames();
#endif

    /*------------------------------------------------------------------------
     *  PHASE 4: RUN SCHEDULER (200 cycles ≈ 200 ms of simulated time)
     *-----------------------------------------------------------------------*/
    printf("--- SCHEDULER RUNNING (200 cycles) ---\n\n");
    Os_Run(200U);

    /*------------------------------------------------------------------------
     *  PHASE 5: DEMO — UDS download sequence
     *-----------------------------------------------------------------------*/
#if (ECU_PLATFORM_NATIVE == 1U)
    demo_uds_download();
#endif

    printf("\n============================================================\n");
    printf("   Simulation complete.\n");
    printf("============================================================\n");

    return 0;
}
