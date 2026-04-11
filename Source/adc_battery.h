/**
  ******************************************************************************
  * @file    adc_battery.h
  * @brief   ADC driver for battery voltage monitoring
  *          PB2 - Analog input, voltage divider ratio 42/(42+49)
  ******************************************************************************
  */

#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H

#include "stm8s.h"
#include "fixedpoint.h"

/* Battery voltage thresholds (Q16.16) */
/* 3.2V = 3.2 * 65536 = 209715 */
#define BATTERY_VOLTAGE_THRESHOLD_FP  ((fp_t)209715L)
/* 3.0V = 3.0 * 65536 = 196608 */
#define BATTERY_VOLTAGE_MIN_FP        ((fp_t)196608L)
/* 4.2V = 4.2 * 65536 = 275251 */
#define BATTERY_VOLTAGE_MAX_FP        ((fp_t)275251L)

/* ADC conversion */
/* Voltage divider: V_measure = V_battery * 42/(42+49) = V_battery * 42/91 */
/* V_battery = V_measure * 91/42 = V_measure * 2.1667 */
#define ADC_REF_VOLTAGE_FP    ((fp_t)216269L)  /* 3.3 * 65536 = 216269 */
#define ADC_RESOLUTION        1024    /* 10-bit ADC */
/* 91/42 * 65536 = 141888 */
#define VOLTAGE_DIVIDER_RATIO ((fp_t)141888L)

/* External state */
extern fp_t battery_voltage_fp;
extern uint8_t battery_low;

void ADC_Battery_Init(void);
uint16_t ADC_Battery_Read(void);
fp_t ADC_Battery_GetVoltage(void);

#endif /* __ADC_BATTERY_H */
