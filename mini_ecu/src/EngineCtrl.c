/**
 * @file    EngineCtrl.c
 * @brief   Engine Control SWC — the heart of this mini ECU demo
 *
 * Responsibilities:
 *  1. Read coolant temperature and battery voltage from MCAL/ADC
 *  2. Run a simple fan control algorithm (bang-bang + hysteresis)
 *  3. Implement a small engine state machine (OFF → CRANKING → RUNNING → FAULT)
 *  4. Broadcast engine status on CAN every 100 ms
 *  5. Receive throttle command frames from CAN and clamp to safe range
 *
 * All arithmetic uses integer types only — no floating point (MISRA Dir 1.1).
 * Physical values carry an implicit scale (see Std_Types.h).
 *
 * MISRA-C:2012 rules applied:
 *  - Rule 10.1/10.4 : Implicit type conversions avoided; explicit casts used
 *  - Rule 13.5       : No side-effects in right-hand operand of &&/||
 *  - Rule 15.5       : Single return per function
 *  - Dir 4.7         : Return value of called functions checked
 */

#include "EngineCtrl.h"
#include "Mcal.h"
#include "Can_Driver.h"
#include "Ecu_Config.h"

#include <stdio.h>
#include <string.h>

/*============================================================================
 *  Global data (shared with UDS diagnostics module)
 *===========================================================================*/
EngCtrl_DataType g_EngCtrl_Data;

/*============================================================================
 *  Module-private state
 *===========================================================================*/
static uint8_t  s_throttle_target = 0U;   /* 0–100 %, set via CAN */
static boolean  s_fan_was_on      = FALSE; /* Hysteresis state     */

/*============================================================================
 *  Static (private) helpers
 *===========================================================================*/

/**
 * @brief Fan control — bang-bang with 3 °C hysteresis
 *
 *  ON  threshold:  ENGINE_TEMP_OVERHEAT  (105.0 °C → 1050 in 0.1 °C units)
 *  LOW threshold:  ENGINE_TEMP_WARM      ( 80.0 °C →  800)
 *  OFF below warm by >3 °C (770 = 77.0 °C)
 *
 * Using hysteresis prevents rapid relay chattering near the threshold.
 */
static DutyCycle_T compute_fan_duty(Temp_T temp)
{
    DutyCycle_T duty;

    if (temp >= ENGINE_TEMP_OVERHEAT)
    {
        duty         = FAN_DUTY_HIGH;
        s_fan_was_on = TRUE;
    }
    else if (temp >= ENGINE_TEMP_WARM)
    {
        duty         = FAN_DUTY_LOW;
        s_fan_was_on = TRUE;
    }
    else if ((s_fan_was_on == TRUE) && (temp > (ENGINE_TEMP_WARM - 30)))
    {
        /* Hysteresis: keep fan on until 3 °C below warm threshold */
        duty = FAN_DUTY_LOW;
    }
    else
    {
        duty         = FAN_DUTY_OFF;
        s_fan_was_on = FALSE;
    }

    return duty;
}

/** @brief Simple state machine transition logic */
static EngCtrl_StateType update_state(EngCtrl_StateType current,
                                      Temp_T            temp,
                                      boolean           overtemp)
{
    EngCtrl_StateType next = current;

    switch (current)
    {
        case ENG_STATE_OFF:
            /* Auto-start for simulation — engine starts after init */
            next = ENG_STATE_CRANKING;
            break;

        case ENG_STATE_CRANKING:
            /* Transition to RUNNING when temp is sensible (above -20 °C) */
            if (temp > -200)
            {
                next = ENG_STATE_RUNNING;
                printf("[ENG]  Engine running.\n");
            }
            break;

        case ENG_STATE_RUNNING:
            if (overtemp == TRUE)
            {
                next = ENG_STATE_FAULT;
                printf("[ENG]  FAULT: Overtemperature! Temp=%d (0.1degC)\n",
                       (int)temp);
            }
            break;

        case ENG_STATE_FAULT:
            /* Latch in FAULT — requires external reset (UDS 0x11) */
            break;

        default:
            next = ENG_STATE_FAULT;
            break;
    }

    return next;
}

/** @brief Process incoming CAN frames addressed to this SWC */
static void process_can_rx(void)
{
    Can_FrameType frame;

    /* Drain all queued RX frames this cycle */
    while (Can_Receive(CAN_CHANNEL_0, &frame) == E_OK)
    {
        if (frame.id == CAN_ID_THROTTLE_CMD)
        {
            if (frame.dlc >= 1U)
            {
                uint8_t raw_throttle = frame.data[0];
                /* Clamp to valid range — never trust external data */
                s_throttle_target = (raw_throttle <= 100U) ? raw_throttle : 100U;
                printf("[ENG]  Throttle command received: %u%%\n",
                       (unsigned)s_throttle_target);
            }
        }
        /* Other IDs are silently ignored — not our concern */
    }
}

/*============================================================================
 *  Public API
 *===========================================================================*/
Std_ReturnType EngCtrl_Init(void)
{
    (void)memset(&g_EngCtrl_Data, 0, sizeof(g_EngCtrl_Data));
    g_EngCtrl_Data.state       = ENG_STATE_OFF;
    g_EngCtrl_Data.fan_duty    = FAN_DUTY_OFF;
    s_throttle_target          = 0U;
    s_fan_was_on               = FALSE;

    printf("[ENG]  Engine Control initialized.\n");
    return E_OK;
}

/**
 * @brief 10 ms runnable — sense → compute → actuate
 *
 * Execution order:
 *  1. Start ADC conversions for this cycle
 *  2. Retrieve results from previous cycle (pipeline — avoids blocking)
 *  3. Convert raw ADC to physical units
 *  4. Update state machine
 *  5. Compute fan duty
 *  6. Write actuator outputs (PWM, GPIO)
 *  7. Process CAN RX
 *  8. Kick watchdog
 */
void EngCtrl_Runnable_10ms(void)
{
    Adc_ValueType   adc_raw_temp;
    Adc_ValueType   adc_raw_vbat;
    Std_ReturnType  adc_ret;

    /* -- Step 1: Start new conversions for next cycle ----------------------*/
    (void)Adc_StartConversion(ADC_CH_COOLANT_TEMP);
    (void)Adc_StartConversion(ADC_CH_BATTERY_VOLTAGE);

    /* -- Step 2: Read results from this cycle (simulation: results are
       available immediately after StartConversion)                          */
    adc_ret = Adc_GetResult(ADC_CH_COOLANT_TEMP, &adc_raw_temp);
    if (adc_ret == E_OK)
    {
        g_EngCtrl_Data.coolant_temp = Adc_RawToTemp(adc_raw_temp);
    }

    adc_ret = Adc_GetResult(ADC_CH_BATTERY_VOLTAGE, &adc_raw_vbat);
    if (adc_ret == E_OK)
    {
        g_EngCtrl_Data.battery_mv = Adc_RawToVoltage(adc_raw_vbat);
    }

    /* -- Step 3: Validate sensor ranges (detect stuck/shorted sensors) -----*/
    if ((g_EngCtrl_Data.coolant_temp < ENGINE_TEMP_MIN) ||
        (g_EngCtrl_Data.coolant_temp > ENGINE_TEMP_MAX))
    {
        printf("[ENG]  WARNING: Coolant temp out of range: %d\n",
               (int)g_EngCtrl_Data.coolant_temp);
    }

    /* -- Step 4: Overtemp flag + state machine ----------------------------*/
    g_EngCtrl_Data.overtemp_flag =
        (g_EngCtrl_Data.coolant_temp >= ENGINE_TEMP_OVERHEAT) ? TRUE : FALSE;

    g_EngCtrl_Data.state = update_state(g_EngCtrl_Data.state,
                                        g_EngCtrl_Data.coolant_temp,
                                        g_EngCtrl_Data.overtemp_flag);

    /* -- Step 5: Fan control ----------------------------------------------*/
    if (g_EngCtrl_Data.state == ENG_STATE_RUNNING)
    {
        g_EngCtrl_Data.fan_duty = compute_fan_duty(g_EngCtrl_Data.coolant_temp);
    }
    else
    {
        g_EngCtrl_Data.fan_duty = FAN_DUTY_OFF;
    }

    /* -- Step 6: Actuate ---------------------------------------------------*/
    (void)Pwm_SetDutyCycle(PWM_CH_FAN, g_EngCtrl_Data.fan_duty);

    /* Fault LED */
    (void)Gpio_WritePin(GPIO_PIN_FAULT_LED,
                        (g_EngCtrl_Data.state == ENG_STATE_FAULT) ?
                        GPIO_HIGH : GPIO_LOW);

    /* Status LED toggles every 10 ms while running */
    {
        static boolean s_led_toggle = FALSE;
        s_led_toggle = (s_led_toggle == FALSE) ? TRUE : FALSE;
        if (g_EngCtrl_Data.state == ENG_STATE_RUNNING)
        {
            (void)Gpio_WritePin(GPIO_PIN_STATUS_LED,
                                (s_led_toggle == TRUE) ? GPIO_HIGH : GPIO_LOW);
        }
    }

    /* -- Step 7: CAN RX processing ----------------------------------------*/
    process_can_rx();

    /* -- Step 8: Watchdog kick --------------------------------------------*/
    Wdg_Kick();
}

/**
 * @brief 100 ms runnable — broadcast engine status on CAN
 *
 * Frame layout for CAN_ID_ENGINE_STATUS (0x100), DLC=8:
 *  Byte 0-1 : RPM          (uint16, big-endian, 1 LSB = 1 RPM)
 *  Byte 2-3 : Coolant temp (int16,  big-endian, 1 LSB = 0.1 °C)
 *  Byte 4-5 : Battery mV   (uint16, big-endian, 1 LSB = 1 mV)
 *  Byte 6   : Fan duty     (uint8, %)
 *  Byte 7   : State + flags
 *               Bit 7: overtemp_flag
 *               Bit 1-0: engine state (0=OFF,1=CRANK,2=RUN,3=FAULT)
 */
void EngCtrl_Runnable_100ms(void)
{
    Can_FrameType frame;
    uint8_t       flags;

    frame.id  = CAN_ID_ENGINE_STATUS;
    frame.dlc = 8U;

    /* Encode RPM — big-endian */
    frame.data[0] = (uint8_t)((g_EngCtrl_Data.rpm >> 8U) & 0xFFU);
    frame.data[1] = (uint8_t)(g_EngCtrl_Data.rpm & 0xFFU);

    /* Encode coolant temp — big-endian signed */
    frame.data[2] = (uint8_t)(((uint16_t)g_EngCtrl_Data.coolant_temp >> 8U) & 0xFFU);
    frame.data[3] = (uint8_t)((uint16_t)g_EngCtrl_Data.coolant_temp & 0xFFU);

    /* Encode battery voltage */
    frame.data[4] = (uint8_t)((g_EngCtrl_Data.battery_mv >> 8U) & 0xFFU);
    frame.data[5] = (uint8_t)(g_EngCtrl_Data.battery_mv & 0xFFU);

    /* Fan duty */
    frame.data[6] = g_EngCtrl_Data.fan_duty;

    /* Flags byte */
    flags = (uint8_t)(g_EngCtrl_Data.state & 0x03U);
    if (g_EngCtrl_Data.overtemp_flag == TRUE)
    {
        flags |= 0x80U;
    }
    frame.data[7] = flags;

    if (Can_Transmit(CAN_CHANNEL_0, &frame) != E_OK)
    {
        printf("[ENG]  WARNING: CAN TX queue full — status frame dropped.\n");
    }
}
