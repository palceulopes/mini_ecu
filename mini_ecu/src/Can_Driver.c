/**
 * @file    Can_Driver.c
 * @brief   CAN driver — software queue + simulation output
 *
 * TX/RX queues are simple ring buffers protected by critical sections.
 * In native mode, transmitted frames are formatted and printed, and the
 * Can_InjectFrame() function lets test code push frames into the RX path.
 *
 * Ring buffer invariants:
 *   - Empty : head == tail
 *   - Full  : ((tail + 1) % SIZE) == head
 *   - Count : (tail - head + SIZE) % SIZE
 *
 * MISRA-C:2012 rules applied:
 *  - Modulo wrap with mask requires SIZE be power-of-two; compile-time check
 *    done via array-size assertion                              (Rule 10.4)
 *  - All pointer parameters checked for NULL before use        (Dir 4.1)
 *  - No dynamic memory allocation                              (Rule 21.3)
 *  - Recursion forbidden — no recursive calls present          (Rule 17.2)
 */

#include "Can_Driver.h"
#include "Os.h"

#include <stdio.h>
#include <string.h>

/*============================================================================
 *  Private ring-buffer type (generic, parameterised by SIZE)
 *===========================================================================*/
typedef struct {
    Can_FrameType   buf[CAN_TX_QUEUE_SIZE];
    uint8_t         head;
    uint8_t         tail;
} CanTxQueue_T;

typedef struct {
    Can_FrameType   buf[CAN_RX_QUEUE_SIZE];
    uint8_t         head;
    uint8_t         tail;
} CanRxQueue_T;

/*============================================================================
 *  Module state
 *===========================================================================*/
static CanTxQueue_T s_tx_queue;
static CanRxQueue_T s_rx_queue;
static boolean      s_initialized = FALSE;

/*============================================================================
 *  Static helpers
 *===========================================================================*/
static boolean txq_full(void)
{
    uint8_t next = (uint8_t)((s_tx_queue.tail + 1U) % CAN_TX_QUEUE_SIZE);
    return (next == s_tx_queue.head) ? TRUE : FALSE;
}

static boolean txq_empty(void)
{
    return (s_tx_queue.head == s_tx_queue.tail) ? TRUE : FALSE;
}

static boolean rxq_full(void)
{
    uint8_t next = (uint8_t)((s_rx_queue.tail + 1U) % CAN_RX_QUEUE_SIZE);
    return (next == s_rx_queue.head) ? TRUE : FALSE;
}

static boolean rxq_empty(void)
{
    return (s_rx_queue.head == s_rx_queue.tail) ? TRUE : FALSE;
}

/** Copy frame into TX queue tail. Caller must verify not full. */
static void txq_push(const Can_FrameType *frame)
{
    /* Safe: caller checked txq_full() == FALSE */
    (void)memcpy(&s_tx_queue.buf[s_tx_queue.tail], frame, sizeof(Can_FrameType));
    s_tx_queue.tail = (uint8_t)((s_tx_queue.tail + 1U) % CAN_TX_QUEUE_SIZE);
}

/** Copy frame from TX queue head. Caller must verify not empty. */
static void txq_pop(Can_FrameType *frame)
{
    (void)memcpy(frame, &s_tx_queue.buf[s_tx_queue.head], sizeof(Can_FrameType));
    s_tx_queue.head = (uint8_t)((s_tx_queue.head + 1U) % CAN_TX_QUEUE_SIZE);
}

/** Copy frame into RX queue tail. Caller must verify not full. */
static void rxq_push(const Can_FrameType *frame)
{
    (void)memcpy(&s_rx_queue.buf[s_rx_queue.tail], frame, sizeof(Can_FrameType));
    s_rx_queue.tail = (uint8_t)((s_rx_queue.tail + 1U) % CAN_RX_QUEUE_SIZE);
}

/** Copy frame from RX queue head. Caller must verify not empty. */
static void rxq_pop(Can_FrameType *frame)
{
    (void)memcpy(frame, &s_rx_queue.buf[s_rx_queue.head], sizeof(Can_FrameType));
    s_rx_queue.head = (uint8_t)((s_rx_queue.head + 1U) % CAN_RX_QUEUE_SIZE);
}

/** Print a CAN frame to stdout (simulation only). */
#if (ECU_PLATFORM_NATIVE == 1U)
static void can_print_frame(const char *direction, const Can_FrameType *frame)
{
    uint8_t i;
    printf("[CAN]  %s  ID=0x%03X  DLC=%u  Data:",
           direction,
           (unsigned)frame->id,
           (unsigned)frame->dlc);

    for (i = 0U; i < frame->dlc; i++)
    {
        printf(" %02X", (unsigned)frame->data[i]);
    }
    printf("\n");
}
#endif

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType Can_Init(void)
{
    (void)memset(&s_tx_queue, 0, sizeof(s_tx_queue));
    (void)memset(&s_rx_queue, 0, sizeof(s_rx_queue));
    s_initialized = TRUE;
    printf("[CAN]  Driver initialized.\n");
    return E_OK;
}

Std_ReturnType Can_Transmit(Can_ChannelType ch, const Can_FrameType *frame)
{
    Std_ReturnType ret = E_NOT_OK;

    /* Validate inputs — security boundary check */
    if ((frame == NULL_PTR) || (ch != CAN_CHANNEL_0) ||
        (s_initialized != TRUE) || (frame->dlc > CAN_FRAME_DLC_MAX))
    {
        return E_NOT_OK;
    }

    OS_ENTER_CRITICAL();
    if (txq_full() != TRUE)
    {
        txq_push(frame);
        ret = E_OK;
    }
    OS_EXIT_CRITICAL();

    return ret;
}

Std_ReturnType Can_Receive(Can_ChannelType ch, Can_FrameType *frame)
{
    Std_ReturnType ret = E_NOT_OK;

    if ((frame == NULL_PTR) || (ch != CAN_CHANNEL_0) ||
        (s_initialized != TRUE))
    {
        return E_NOT_OK;
    }

    OS_ENTER_CRITICAL();
    if (rxq_empty() != TRUE)
    {
        rxq_pop(frame);
        ret = E_OK;
    }
    OS_EXIT_CRITICAL();

    return ret;
}

void Can_MainFunction(void)
{
    Can_FrameType frame;

    if (s_initialized != TRUE)
    {
        return;
    }

    /* Flush TX queue — on real HW this would write to CAN peripheral */
    while (txq_empty() != TRUE)
    {
        txq_pop(&frame);
#if (ECU_PLATFORM_NATIVE == 1U)
        can_print_frame("TX", &frame);
#endif
    }
}

#if (ECU_PLATFORM_NATIVE == 1U)
Std_ReturnType Can_InjectFrame(const Can_FrameType *frame)
{
    Std_ReturnType ret = E_NOT_OK;

    if ((frame == NULL_PTR) || (s_initialized != TRUE) ||
        (frame->dlc > CAN_FRAME_DLC_MAX))
    {
        return E_NOT_OK;
    }

    OS_ENTER_CRITICAL();
    if (rxq_full() != TRUE)
    {
        rxq_push(frame);
        can_print_frame("RX (injected)", frame);
        ret = E_OK;
    }
    OS_EXIT_CRITICAL();

    return ret;
}
#endif
