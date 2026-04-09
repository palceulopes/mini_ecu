/**
 * @file    EngineCtrl.h
 * @brief   Engine Control SWC — reads RPM + temperature, drives fan, sends CAN
 */

#ifndef ENGINE_CTRL_H
#define ENGINE_CTRL_H

#include "Std_Types.h"

/** Engine state machine states */
typedef uint8_t EngCtrl_StateType;
#define ENG_STATE_OFF       ((EngCtrl_StateType)0U)
#define ENG_STATE_CRANKING  ((EngCtrl_StateType)1U)
#define ENG_STATE_RUNNING   ((EngCtrl_StateType)2U)
#define ENG_STATE_FAULT     ((EngCtrl_StateType)3U)

/** Engine data snapshot (shared via extern for diagnostics) */
typedef struct {
    Rpm_T           rpm;
    Temp_T          coolant_temp;   /* tenths of °C */
    Voltage_mV_T    battery_mv;
    DutyCycle_T     fan_duty;
    EngCtrl_StateType state;
    boolean         overtemp_flag;
} EngCtrl_DataType;

extern EngCtrl_DataType g_EngCtrl_Data;

/*============================================================================
 *  Public API — called by OS tasks
 *===========================================================================*/
Std_ReturnType  EngCtrl_Init(void);
void            EngCtrl_Runnable_10ms(void);   /* Sense + compute + actuate */
void            EngCtrl_Runnable_100ms(void);  /* CAN TX of engine status   */

#endif /* ENGINE_CTRL_H */
