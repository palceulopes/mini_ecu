/**
 * @file    Mcal.h
 * @brief   Microcontroller Abstraction Layer — ADC, GPIO, PWM, Timer
 *
 * On a real target each function maps directly to peripheral registers.
 * In ECU_PLATFORM_NATIVE mode they return deterministic simulated values
 * so the full application stack can be validated on a PC without hardware.
 *
 * MISRA-C:2012 compliance notes:
 *  - All pointers validated before dereference           (Rule 11.5, 14.3)
 *  - No dynamic memory allocation                        (Rule 21.3)
 *  - All function parameters checked where appropriate   (Dir 4.1)
 */

#ifndef MCAL_H
#define MCAL_H

#include "Std_Types.h"
#include "Ecu_Config.h"

/*============================================================================
 *  GPIO
 *===========================================================================*/
typedef uint8_t Gpio_PinType;
typedef uint8_t Gpio_LevelType;

#define GPIO_HIGH   ((Gpio_LevelType)1U)
#define GPIO_LOW    ((Gpio_LevelType)0U)

/* Simulated pin numbers */
#define GPIO_PIN_FAN_RELAY      ((Gpio_PinType)0U)
#define GPIO_PIN_STATUS_LED     ((Gpio_PinType)1U)
#define GPIO_PIN_FAULT_LED      ((Gpio_PinType)2U)

Std_ReturnType  Gpio_Init(void);
Std_ReturnType  Gpio_WritePin(Gpio_PinType pin, Gpio_LevelType level);
Gpio_LevelType  Gpio_ReadPin(Gpio_PinType pin);

/*============================================================================
 *  ADC  (simulated sensors)
 *===========================================================================*/
typedef uint8_t  Adc_ChannelType;
typedef uint16_t Adc_ValueType;   /* 12-bit ADC: 0–4095 */

#define ADC_CH_COOLANT_TEMP     ((Adc_ChannelType)0U)
#define ADC_CH_BATTERY_VOLTAGE  ((Adc_ChannelType)1U)
#define ADC_CH_THROTTLE_POS     ((Adc_ChannelType)2U)

Std_ReturnType  Adc_Init(void);
Std_ReturnType  Adc_StartConversion(Adc_ChannelType channel);
Std_ReturnType  Adc_GetResult(Adc_ChannelType channel, Adc_ValueType *result);

/* Conversion helpers (avoid float — fixed-point) */
/* ADC raw 0-4095 → temperature in tenths-of-°C  */
/* Formula: Temp(0.1°C) = (raw * 2000 / 4095) - 400  covers -40…+160°C */
static inline Temp_T Adc_RawToTemp(Adc_ValueType raw)
{
    return (Temp_T)(((int32_t)raw * 2000) / 4095) - 400;
}

/* ADC raw 0-4095 → voltage in mV ( 0…20000 mV range ) */
static inline Voltage_mV_T Adc_RawToVoltage(Adc_ValueType raw)
{
    return (Voltage_mV_T)(((uint32_t)raw * 20000U) / 4095U);
}

/*============================================================================
 *  PWM (fan, injectors, etc.)
 *===========================================================================*/
typedef uint8_t Pwm_ChannelType;

#define PWM_CH_FAN              ((Pwm_ChannelType)0U)
#define PWM_CH_FUEL_PUMP        ((Pwm_ChannelType)1U)

Std_ReturnType  Pwm_Init(void);
Std_ReturnType  Pwm_SetDutyCycle(Pwm_ChannelType channel, DutyCycle_T duty);

/*============================================================================
 *  System Timer
 *===========================================================================*/
Std_ReturnType  SysTimer_Init(void);
uint32_t        SysTimer_GetTickMs(void);

/*============================================================================
 *  Watchdog
 *===========================================================================*/
Std_ReturnType  Wdg_Init(void);
void            Wdg_Kick(void);   /* Must be called within WDG_TIMEOUT_MS */

#endif /* MCAL_H */
