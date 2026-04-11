/**
  ******************************************************************************
  * @file    pwm_motors.c
  * @brief   PWM output driver for 4 motors at 16KHz
  ******************************************************************************
  */

#include "pwm_motors.h"
#include "timer.h"

/* Motor duty cycles (0-100%) */
uint8_t motor_duty[MOTOR_COUNT] = {0, 0, 0, 0};

/* TIM1 period for 16KHz: 16MHz/16000 = 1000 */
#define TIM1_PERIOD 999  /* ARR = 999 for 1000 counts */

/* TIM2 period for 16KHz */
#define TIM2_PERIOD 999

void PWM_Motors_Init(void)
{
    /* TIM1 configuration for PC1 (CH1) and PC2 (CH2) */
    TIM1_DeInit();
    
    /* Time base: 16MHz / 1 = 16MHz, period = 1000 -> 16KHz */
    TIM1_TimeBaseInit(0,              /* Prescaler */
                      TIM1_COUNTERMODE_UP,
                      TIM1_PERIOD,    /* Period */
                      0);             /* Repetition counter */
    
    /* Channel 1 (PC1) - PWM Mode 1 */
    TIM1_OC1Init(TIM1_OCMODE_PWM1,
                 TIM1_OUTPUTSTATE_ENABLE,
                 TIM1_OUTPUTNSTATE_ENABLE,
                 0,                   /* Initial pulse = 0 */
                 TIM1_OCPOLARITY_HIGH,
                 TIM1_OCNPOLARITY_HIGH,
                 TIM1_OCIDLESTATE_RESET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC1PreloadConfig(ENABLE);
    
    /* Channel 2 (PC2) - PWM Mode 1 */
    TIM1_OC2Init(TIM1_OCMODE_PWM1,
                 TIM1_OUTPUTSTATE_ENABLE,
                 TIM1_OUTPUTNSTATE_ENABLE,
                 0,                   /* Initial pulse = 0 */
                 TIM1_OCPOLARITY_HIGH,
                 TIM1_OCNPOLARITY_HIGH,
                 TIM1_OCIDLESTATE_RESET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC2PreloadConfig(ENABLE);
    
    /* Enable auto-reload preload */
    TIM1_ARRPreloadConfig(ENABLE);
    
    /* Main output enable */
    TIM1_CtrlPWMOutputs(ENABLE);
    
    /* TIM2 configuration for PD3 (CH1) and PD2 (CH2) */
    TIM2_DeInit();
    
    /* Time base: 16MHz / 1 = 16MHz, period = 1000 -> 16KHz */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1,
                      TIM2_PERIOD);
    
    /* Channel 1 (PD3) - PWM Mode 1 */
    TIM2_OC1Init(TIM2_OCMODE_PWM1,
                 TIM2_OUTPUTSTATE_ENABLE,
                 0,                  /* Initial pulse = 0 */
                 TIM2_OCPOLARITY_HIGH);
    TIM2_OC1PreloadConfig(ENABLE);
    
    /* Channel 2 (PD2) - PWM Mode 1 */
    TIM2_OC2Init(TIM2_OCMODE_PWM1,
                 TIM2_OUTPUTSTATE_ENABLE,
                 0,                  /* Initial pulse = 0 */
                 TIM2_OCPOLARITY_HIGH);
    TIM2_OC2PreloadConfig(ENABLE);
    
    /* Enable auto-reload preload */
    TIM2_ARRPreloadConfig(ENABLE);
    
    /* Start timers */
    TIM1_Cmd(ENABLE);
    TIM2_Cmd(ENABLE);
}

void PWM_SetMotorDuty(uint8_t motor, uint8_t duty)
{
    uint16_t pulse;
    
    if (motor >= MOTOR_COUNT) return;
    if (duty > 100) duty = 100;
    
    motor_duty[motor] = duty;
    
    /* Convert duty % to pulse width (0-999) */
    pulse = (uint16_t)((uint32_t)duty * TIM1_PERIOD / 100);
    
    switch (motor) {
        case MOTOR_RIGHT_FRONT:
            TIM1_SetCompare1(pulse);
            break;
        case MOTOR_RIGHT_BACK:
            TIM1_SetCompare2(pulse);
            break;
        case MOTOR_LEFT_FRONT:
            TIM2_SetCompare1(pulse);
            break;
        case MOTOR_LEFT_BACK:
            TIM2_SetCompare2(pulse);
            break;
    }
}

void PWM_SetAllMotors(uint8_t duty)
{
    uint8_t i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        PWM_SetMotorDuty(i, duty);
    }
}

void PWM_Motors_Enable(void)
{
    TIM1_Cmd(ENABLE);
    TIM2_Cmd(ENABLE);
}

void PWM_Motors_Disable(void)
{
    TIM1_SetCompare1(0);
    TIM1_SetCompare2(0);
    TIM2_SetCompare1(0);
    TIM2_SetCompare2(0);
}
