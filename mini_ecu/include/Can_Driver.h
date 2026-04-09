/**
 * @file    Can_Driver.h
 * @brief   CAN Bus driver — frame definition, TX/RX queues, send/receive API
 *
 * Implements a software CAN queue that, on a real target, would interface
 * with the MCU's CAN peripheral registers. In simulation mode, TX frames
 * are printed to stdout and RX frames are injected via Can_InjectFrame().
 *
 * MISRA-C:2012 compliance notes:
 *  - Fixed-size queue with no dynamic allocation   (Rule 21.3)
 *  - All error paths return Std_ReturnType         (Dir 4.7)
 *  - No recursion used anywhere                    (Rule 17.2)
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "Std_Types.h"
#include "Ecu_Config.h"

/*============================================================================
 *  CAN Frame
 *===========================================================================*/
typedef struct {
    uint32_t id;                        /* 11-bit standard CAN ID */
    uint8_t  dlc;                       /* Data Length Code: 0-8  */
    uint8_t  data[CAN_FRAME_DLC_MAX];   /* Payload                */
} Can_FrameType;

/*============================================================================
 *  CAN Channel
 *===========================================================================*/
typedef uint8_t Can_ChannelType;
#define CAN_CHANNEL_0   ((Can_ChannelType)0U)

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType  Can_Init(void);

/** Queue a frame for transmission. Returns E_NOT_OK if TX queue is full. */
Std_ReturnType  Can_Transmit(Can_ChannelType ch, const Can_FrameType *frame);

/** Dequeue one received frame. Returns E_NOT_OK if RX queue is empty. */
Std_ReturnType  Can_Receive(Can_ChannelType ch, Can_FrameType *frame);

/** Called from the OS 2 ms task — flushes TX queue, processes RX. */
void            Can_MainFunction(void);

/** Simulation-only: inject a frame into the RX queue for testing. */
#if (ECU_PLATFORM_NATIVE == 1U)
Std_ReturnType  Can_InjectFrame(const Can_FrameType *frame);
#endif

#endif /* CAN_DRIVER_H */
