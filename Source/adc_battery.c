/**
  ******************************************************************************
  * @file    adc_battery.c
  * @brief   ADC driver for battery voltage monitoring
  ******************************************************************************
  */

#include "adc_battery.h"

/* External state */
fp_t battery_voltage_fp = FP_ZERO;
uint8_t battery_low = 0;

void ADC_Battery_Init(void)
{
    /* Enable ADC clock */
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    
    /* Configure PB2 as analog input */
    GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
    
    /* ADC configuration */
    ADC1_DeInit();
    ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
              ADC1_CHANNEL_2,          /* PB2 = Channel 2 */
              ADC1_PRESSEL_FCPU_D2,    /* fADC = fCPU/2 = 8MHz */
              ADC1_EXTTRIG_TIM,
              DISABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_CHANNEL2,
              DISABLE);
    
    /* Enable ADC */
    ADC1_Cmd(ENABLE);
    
    /* Start conversion */
    ADC1_StartConversion();
}

uint16_t ADC_Battery_Read(void)
{
    /* Wait for EOC flag */
    while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
    
    /* Read 10-bit result */
    return ADC1_GetConversionValue();
}

fp_t ADC_Battery_GetVoltage(void)
{
    uint16_t adc_value;
    fp_t voltage_measured;
    fp_t vref_fp;
    fp_t ratio_fp;
    
    adc_value = ADC_Battery_Read();
    
    /* Convert ADC to voltage: V = ADC * Vref / 1024 */
    vref_fp = ADC_REF_VOLTAGE_FP;
    voltage_measured = FP_DIV(FP_MUL(INT_TO_FP(adc_value), vref_fp), INT_TO_FP(ADC_RESOLUTION));
    
    /* Apply voltage divider ratio */
    ratio_fp = VOLTAGE_DIVIDER_RATIO;
    battery_voltage_fp = FP_MUL(voltage_measured, ratio_fp);
    
    /* Check battery level */
    if (FP_ABS(battery_voltage_fp) <= BATTERY_VOLTAGE_THRESHOLD_FP) {
        battery_low = 1;
    } else {
        battery_low = 0;
    }
    
    return battery_voltage_fp;
}
