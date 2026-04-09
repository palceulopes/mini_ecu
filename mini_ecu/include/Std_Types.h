/**
 * @file    Std_Types.h
 * @brief   AUTOSAR-style standard types — MISRA-C:2012 compliant
 *
 * MISRA-C:2012 Rules applied:
 *  - Rule 2.5:  All typedefs used
 *  - Rule 8.1:  Explicit types on all declarations
 *  - Rule 21.1: No reserved identifiers used
 */

#ifndef STD_TYPES_H
#define STD_TYPES_H

#include <stdint.h>   /* uint8_t, uint16_t, uint32_t, int16_t ... */
#include <stddef.h>   /* NULL, size_t */

/*============================================================================
 *  Boolean
 *===========================================================================*/
typedef uint8_t boolean;

#ifndef TRUE
#define TRUE  ((boolean)1U)
#endif

#ifndef FALSE
#define FALSE ((boolean)0U)
#endif

/*============================================================================
 *  AUTOSAR standard return type
 *===========================================================================*/
typedef uint8_t Std_ReturnType;

#define E_OK        ((Std_ReturnType)0U)
#define E_NOT_OK    ((Std_ReturnType)1U)

/*============================================================================
 *  Physical value scaling helpers (avoid floats — MISRA Rule 14.x)
 *  All physical values carried as integers with implicit scale factors.
 *
 *  RPM     : raw uint16_t, 1 LSB = 1 RPM
 *  Temp    : raw int16_t,  1 LSB = 0.1 °C  (e.g. 250 = 25.0 °C)
 *  Voltage : raw uint16_t, 1 LSB = 1 mV
 *  Duty    : raw uint8_t,  0-100 = 0%-100%
 *===========================================================================*/
typedef uint16_t Rpm_T;
typedef int16_t  Temp_T;        /* tenths of °C */
typedef uint16_t Voltage_mV_T;
typedef uint8_t  DutyCycle_T;   /* 0–100 */

/*============================================================================
 *  Null pointer constant (MISRA Rule 11.9)
 *===========================================================================*/
#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#endif /* STD_TYPES_H */
