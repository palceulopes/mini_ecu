/**
 * @file    Ecu_Config.h
 * @brief   Central ECU configuration — feature flags, limits, identifiers
 *
 * This is the single place to tune the system. All magic numbers live here.
 */

#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include "Std_Types.h"

/*============================================================================
 *  MCU / Platform
 *  Set ECU_PLATFORM_NATIVE=1 when building on a PC for simulation.
 *  Set ECU_PLATFORM_NATIVE=0 for real embedded targets.
 *  Allow the build system to override this symbol.
 *===========================================================================*/
#ifndef ECU_PLATFORM_NATIVE
#define ECU_PLATFORM_NATIVE     (1U)   /* PC simulation mode */
#endif

/*============================================================================
 *  Software identification
 *===========================================================================*/
#define ECU_SW_VERSION_MAJOR    (1U)
#define ECU_SW_VERSION_MINOR    (0U)
#define ECU_SW_VERSION_PATCH    (0U)

/* ECU hardware address on the CAN network */
#define ECU_NODE_ADDRESS        (0x10U)

/*============================================================================
 *  OS / Scheduler timing
 *  All times in milliseconds. Scheduler tick = 1 ms.
 *===========================================================================*/
#define OS_TICK_MS              (1U)
#define OS_TASK_2MS_PERIOD      (2U)
#define OS_TASK_10MS_PERIOD     (10U)
#define OS_TASK_100MS_PERIOD    (100U)

/*============================================================================
 *  Engine limits
 *===========================================================================*/
#define ENGINE_RPM_MAX          (7000U)   /* rpm       */
#define ENGINE_RPM_IDLE         (800U)    /* rpm       */
#define ENGINE_TEMP_OVERHEAT    (1050)    /* 0.1 °C → 105.0 °C */
#define ENGINE_TEMP_WARM        (800)     /* 0.1 °C →  80.0 °C */
#define ENGINE_TEMP_MIN         (-400)    /* 0.1 °C → -40.0 °C */
#define ENGINE_TEMP_MAX         (1500)    /* 0.1 °C → 150.0 °C */

/*============================================================================
 *  Fan control PWM limits
 *===========================================================================*/
#define FAN_DUTY_OFF            (0U)    /* % */
#define FAN_DUTY_LOW            (30U)   /* % */
#define FAN_DUTY_HIGH           (100U)  /* % */

/*============================================================================
 *  CAN identifiers (11-bit standard frame)
 *===========================================================================*/
#define CAN_ID_ENGINE_STATUS    (0x100U)  /* TX: RPM + Temp + Flags  */
#define CAN_ID_THROTTLE_CMD     (0x200U)  /* RX: throttle target     */
#define CAN_ID_DIAG_REQUEST     (0x7DFU)  /* RX: OBD/UDS request     */
#define CAN_ID_DIAG_RESPONSE    (0x7E8U)  /* TX: OBD/UDS response    */

/*============================================================================
 *  CAN buffer sizes
 *===========================================================================*/
#define CAN_TX_QUEUE_SIZE       (8U)
#define CAN_RX_QUEUE_SIZE       (8U)
#define CAN_FRAME_DLC_MAX       (8U)

/*============================================================================
 *  Diagnostics (UDS)
 *===========================================================================*/
#define UDS_MAX_REQUEST_LEN     (64U)
#define UDS_MAX_RESPONSE_LEN    (64U)

/* UDS service IDs */
#define UDS_SID_READ_DATA_BY_ID (0x22U)
#define UDS_SID_ECU_RESET       (0x11U)
#define UDS_SID_SESSION_CTRL    (0x10U)

/* UDS Data Identifiers */
#define UDS_DID_ECU_INFO        (0xF101U)
#define UDS_DID_ENGINE_RPM      (0xF200U)
#define UDS_DID_COOLANT_TEMP    (0xF201U)

/* UDS Negative Response Codes */
#define UDS_NRC_POSITIVE        (0x00U)
#define UDS_NRC_SERVICE_NOT_SUP (0x11U)
#define UDS_NRC_SUBF_NOT_SUP    (0x12U)
#define UDS_NRC_REQ_OUT_OF_RANGE (0x31U)

/*============================================================================
 *  Watchdog
 *===========================================================================*/
#define WDG_TIMEOUT_MS          (50U)   /* Watchdog kicked every task cycle */

#endif /* ECU_CONFIG_H */
