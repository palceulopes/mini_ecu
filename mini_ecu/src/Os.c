/**
 * @file    Os.c
 * @brief   Cooperative OSEK-style scheduler implementation
 *
 * Scheduling algorithm:
 *  1. Os_TickHandler() increments a global ms counter every tick.
 *  2. Os_Run() loops continuously; on each iteration it reads the current
 *     tick and dispatches any task whose (tick - last_run) >= period_ms.
 *  3. All tasks run to completion before the next is considered (cooperative).
 *
 * On a real MCU the tick would be driven by a hardware timer ISR.
 * In simulation we call Os_TickHandler() from a tight loop gated by
 * SysTimer_GetTickMs(), giving 1 ms resolution without OS sleep calls.
 *
 * MISRA-C:2012 rules applied:
 *  - Global tick counter is volatile (Rule 8.6 / shared with "ISR")
 *  - Task IDs range-checked before array index                  (Rule 18.1)
 *  - No dynamic allocation                                      (Rule 21.3)
 *  - Single return per function                                 (Rule 15.5)
 */

#include "Os.h"
#include "Mcal.h"

#include <stdio.h>

/*============================================================================
 *  Module state
 *===========================================================================*/
static volatile uint32_t   s_tick_ms          = 0U;
static Os_TaskDescType     s_tasks[OS_TASK_COUNT];
static boolean             s_initialized      = FALSE;
static uint32_t            s_last_wall_ms     = 0U;

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType Os_Init(void)
{
    uint8_t i;

    for (i = 0U; i < OS_TASK_COUNT; i++)
    {
        s_tasks[i].func      = NULL_PTR;
        s_tasks[i].period_ms = 0U;
        s_tasks[i].last_run  = 0U;
        s_tasks[i].enabled   = FALSE;
    }

    s_tick_ms      = 0U;
    s_last_wall_ms = 0U;
    s_initialized  = TRUE;

    printf("[OS]   Scheduler initialized. %u task slots available.\n",
           (unsigned)OS_TASK_COUNT);

    return E_OK;
}

Std_ReturnType Os_RegisterTask(Os_TaskIdType    id,
                               Os_TaskFuncType  func,
                               uint32_t         period_ms)
{
    Std_ReturnType ret = E_NOT_OK;

    if ((id < OS_TASK_COUNT) && (func != NULL_PTR) && (period_ms > 0U) &&
        (s_initialized == TRUE))
    {
        s_tasks[id].func      = func;
        s_tasks[id].period_ms = period_ms;
        s_tasks[id].last_run  = 0U;
        s_tasks[id].enabled   = TRUE;

        printf("[OS]   Task %u registered. Period=%u ms.\n",
               (unsigned)id, (unsigned)period_ms);
        ret = E_OK;
    }

    return ret;
}

void Os_TickHandler(void)
{
    s_tick_ms++;   /* Called from timer ISR — or simulated equivalent */
}

void Os_Run(uint32_t max_cycles)
{
    uint32_t         cycles = 0U;
    uint32_t         last_counted_tick = 0U;
    uint8_t          i;
    uint32_t         now;
    Os_TaskDescType *task;

    if (s_initialized != TRUE)
    {
        printf("[OS]   ERROR: Os_Run() called before Os_Init().\n");
        return;
    }

    printf("[OS]   Scheduler running...\n");

    for (;;)
    {
        /*--------------------------------------------------------------------
         *  Advance the software tick counter in sync with wall-clock time.
         *  SysTimer_GetTickMs() never goes backwards, so this is monotonic.
         *-------------------------------------------------------------------*/
        now = SysTimer_GetTickMs();
        while (s_last_wall_ms < now)
        {
            Os_TickHandler();
            s_last_wall_ms++;
        }

        /*--------------------------------------------------------------------
         *  Dispatch any task whose time has come
         *-------------------------------------------------------------------*/
        for (i = 0U; i < OS_TASK_COUNT; i++)
        {
            task = &s_tasks[i];
            if (task->enabled != TRUE)
            {
                continue;
            }

            if ((s_tick_ms - task->last_run) >= task->period_ms)
            {
                task->last_run = s_tick_ms;
                task->func();   /* Run the task */
            }
        }

        /*--------------------------------------------------------------------
         *  Exit condition: count elapsed scheduler ticks, not busy-loop spins.
         *-------------------------------------------------------------------*/
        if (max_cycles > 0U)
        {
            if (s_tick_ms != last_counted_tick)
            {
                cycles += (s_tick_ms - last_counted_tick);
                last_counted_tick = s_tick_ms;
            }

            if (cycles >= max_cycles)
            {
                printf("[OS]   Reached max_cycles (%u). Stopping.\n",
                       (unsigned)max_cycles);
                break;
            }
        }
    }
}
