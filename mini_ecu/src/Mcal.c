/**
 * @file    Mcal.c
 * @brief   MCAL implementation — native (PC simulation) build
 *
 * Hardware abstractions are replaced by deterministic simulated behaviour:
 *  - ADC channels ramp slowly to exercise the control algorithms
 *  - GPIO writes print a human-readable log line
 *  - PWM writes store the duty cycle for inspection
 *  - SysTimer uses the C standard clock() function
 *  - Watchdog is a software counter (no hardware reset available on PC)
 */

#include "Mcal.h"

#include <stdio.h>   /* printf  */
#include <time.h>    /* clock() */

/*============================================================================
 *  Module-private state
 *===========================================================================*/
/* GPIO shadow register — bit N = pin N level */
static uint8_t s_gpio_state = 0U;

/* ADC simulated raw values (12-bit, 0-4095) */
static Adc_ValueType s_adc_raw[3] = {
    1500U,  /* CH0: coolant temp (maps to ~33 °C) */
    3000U,  /* CH1: battery ~14.7 V               */
    2000U   /* CH2: throttle ~49%                 */
};

/* ADC conversion-complete flags */
static boolean s_adc_ready[3] = {FALSE, FALSE, FALSE};

/* PWM duty shadows */
static DutyCycle_T s_pwm_duty[2] = {0U, 0U};

/* Watchdog software counter */
static uint32_t s_wdg_counter = 0U;

/* Timer base — tick at init */
static clock_t  s_timer_base = 0;

/* Simulation step counter — incremented each Adc_StartConversion call */
static uint32_t s_sim_step = 0U;

/*============================================================================
 *  GPIO
 *===========================================================================*/
Std_ReturnType Gpio_Init(void)
{
    s_gpio_state = 0U;
    printf("[MCAL] GPIO initialized.\n");
    return E_OK;
}

Std_ReturnType Gpio_WritePin(Gpio_PinType pin, Gpio_LevelType level)
{
    if (pin > 7U)
    {
        return E_NOT_OK;
    }

    if (level == GPIO_HIGH)
    {
        s_gpio_state |= (uint8_t)(1U << pin);
    }
    else
    {
        s_gpio_state &= (uint8_t)(~(uint8_t)(1U << pin));
    }

    printf("[GPIO] Pin %u -> %s\n", (unsigned)pin,
           (level == GPIO_HIGH) ? "HIGH" : "LOW");
    return E_OK;
}

Gpio_LevelType Gpio_ReadPin(Gpio_PinType pin)
{
    if (pin > 7U)
    {
        return GPIO_LOW;
    }
    return (Gpio_LevelType)((s_gpio_state >> pin) & 0x01U);
}

/*============================================================================
 *  ADC
 *  Simulation: each start-conversion nudges the coolant temperature
 *  upward slowly until it triggers the overtemp threshold, then resets.
 *===========================================================================*/
Std_ReturnType Adc_Init(void)
{
    s_adc_raw[ADC_CH_COOLANT_TEMP]    = 1500U;
    s_adc_raw[ADC_CH_BATTERY_VOLTAGE] = 3000U;
    s_adc_raw[ADC_CH_THROTTLE_POS]    = 2000U;
    s_adc_ready[0] = FALSE;
    s_adc_ready[1] = FALSE;
    s_adc_ready[2] = FALSE;
    s_sim_step = 0U;
    printf("[MCAL] ADC initialized.\n");
    return E_OK;
}

Std_ReturnType Adc_StartConversion(Adc_ChannelType channel)
{
    if (channel > ADC_CH_THROTTLE_POS)
    {
        return E_NOT_OK;
    }

    /*
     * Simulate slow temperature rise:
     * Every 5 conversions on CH0 raise raw by 60 (~2.9 °C step).
     * Reset when raw exceeds 3600 (~136 °C).
     */
    if (channel == ADC_CH_COOLANT_TEMP)
    {
        s_sim_step++;
        if ((s_sim_step % 5U) == 0U)
        {
            s_adc_raw[ADC_CH_COOLANT_TEMP] += 60U;
            if (s_adc_raw[ADC_CH_COOLANT_TEMP] > 3600U)
            {
                s_adc_raw[ADC_CH_COOLANT_TEMP] = 1000U; /* reset to ~9 °C */
            }
        }
    }

    s_adc_ready[channel] = TRUE;
    return E_OK;
}

Std_ReturnType Adc_GetResult(Adc_ChannelType channel, Adc_ValueType *result)
{
    if ((result == NULL_PTR) || (channel > ADC_CH_THROTTLE_POS))
    {
        return E_NOT_OK;
    }

    if (s_adc_ready[channel] != TRUE)
    {
        return E_NOT_OK;   /* Conversion not finished yet */
    }

    *result = s_adc_raw[channel];
    s_adc_ready[channel] = FALSE;
    return E_OK;
}

/*============================================================================
 *  PWM
 *===========================================================================*/
Std_ReturnType Pwm_Init(void)
{
    s_pwm_duty[PWM_CH_FAN]       = 0U;
    s_pwm_duty[PWM_CH_FUEL_PUMP] = 0U;
    printf("[MCAL] PWM initialized.\n");
    return E_OK;
}

Std_ReturnType Pwm_SetDutyCycle(Pwm_ChannelType channel, DutyCycle_T duty)
{
    if (channel > PWM_CH_FUEL_PUMP)
    {
        return E_NOT_OK;
    }
    if (duty > 100U)
    {
        return E_NOT_OK;
    }

    if (s_pwm_duty[channel] != duty)   /* Log only on change */
    {
        printf("[PWM]  Channel %u -> %u%%\n", (unsigned)channel, (unsigned)duty);
        s_pwm_duty[channel] = duty;
    }
    return E_OK;
}

/*============================================================================
 *  System Timer  (PC: milliseconds via clock())
 *===========================================================================*/
Std_ReturnType SysTimer_Init(void)
{
    s_timer_base = clock();
    printf("[MCAL] SysTimer initialized.\n");
    return E_OK;
}

uint32_t SysTimer_GetTickMs(void)
{
    clock_t now   = clock();
    clock_t delta = now - s_timer_base;
    /* CLOCKS_PER_SEC is at least 1 — safe division */
    return (uint32_t)((delta * 1000UL) / (uint32_t)CLOCKS_PER_SEC);
}

/*============================================================================
 *  Watchdog  (software implementation for simulation)
 *===========================================================================*/
Std_ReturnType Wdg_Init(void)
{
    s_wdg_counter = 0U;
    printf("[MCAL] Watchdog initialized.\n");
    return E_OK;
}

void Wdg_Kick(void)
{
    s_wdg_counter = 0U;   /* Reset the counter — we're alive */
}
