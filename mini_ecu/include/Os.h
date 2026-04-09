/**
 * @file    Os.h
 * @brief   OSEK/AUTOSAR-OS inspired cooperative scheduler
 *
 * Provides:
 *  - Fixed task table with configurable periods (2 ms, 10 ms, 100 ms)
 *  - Counting semaphores for ISR→task signalling
 *  - Critical section macros (disable/enable interrupts)
 *
 * This is a *cooperative* (non-preemptive) implementation suitable for
 * simulation and for simple ECUs.  A real AUTOSAR OS is preemptive and
 * priority-based, but the task API is identical.
 *
 * MISRA-C:2012 compliance notes:
 *  - Function pointers stored as const table entries  (Rule 11.1)
 *  - No dynamic allocation                            (Rule 21.3)
 *  - All task IDs validated before use                (Dir 4.1)
 */

#ifndef OS_H
#define OS_H

#include "Std_Types.h"
#include "Ecu_Config.h"

/*============================================================================
 *  Task IDs
 *===========================================================================*/
typedef uint8_t Os_TaskIdType;

#define OS_TASK_ID_2MS      ((Os_TaskIdType)0U)
#define OS_TASK_ID_10MS     ((Os_TaskIdType)1U)
#define OS_TASK_ID_100MS    ((Os_TaskIdType)2U)
#define OS_TASK_COUNT       (3U)

/*============================================================================
 *  Task function pointer type
 *===========================================================================*/
typedef void (*Os_TaskFuncType)(void);

/*============================================================================
 *  Task descriptor
 *===========================================================================*/
typedef struct {
    Os_TaskFuncType func;       /* Runnable to call             */
    uint32_t        period_ms;  /* How often to run (ms)        */
    uint32_t        last_run;   /* Tick counter of last run     */
    boolean         enabled;    /* Task active flag             */
} Os_TaskDescType;

/*============================================================================
 *  Semaphore (binary, for ISR→task signalling)
 *===========================================================================*/
typedef volatile uint8_t Os_SemType;

#define OS_SEM_POST(sem)    ((sem) = 1U)
#define OS_SEM_WAIT(sem)    ((sem) != 0U)
#define OS_SEM_CLEAR(sem)   ((sem) = 0U)

/*============================================================================
 *  Critical section (on native build maps to nothing; on target = disable IRQ)
 *===========================================================================*/
#if (ECU_PLATFORM_NATIVE == 1U)
#define OS_ENTER_CRITICAL()   /* not needed on single-threaded sim */
#define OS_EXIT_CRITICAL()    /* not needed on single-threaded sim */
#else
#define OS_ENTER_CRITICAL()  __disable_irq()
#define OS_EXIT_CRITICAL()   __enable_irq()
#endif

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType  Os_Init(void);

/**
 * Register a task into a time slot.
 * @param id        One of OS_TASK_ID_*
 * @param func      Function to call each period
 * @param period_ms Period in milliseconds
 */
Std_ReturnType  Os_RegisterTask(Os_TaskIdType id,
                                Os_TaskFuncType func,
                                uint32_t period_ms);

/**
 * Main scheduler loop — call once from main().
 * Runs forever, dispatching tasks based on elapsed time.
 * @param max_cycles  0 = run forever; N = run N scheduler cycles (for tests)
 */
void Os_Run(uint32_t max_cycles);

/** Called by SysTimer ISR (or sim timer) every OS_TICK_MS. */
void Os_TickHandler(void);

#endif /* OS_H */
