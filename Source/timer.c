/**
  ******************************************************************************
  * @file    timer.c
  * @brief   Timer configuration for control loop timing
  ******************************************************************************
  */

#include "timer.h"

/* Timer tick flags */
volatile uint8_t timer_tick_500hz = 0;
volatile uint8_t timer_tick_20hz = 0;
volatile uint8_t timer_tick_1hz = 0;

/* Counter for 20Hz tick (500/20 ≈ 25) */
//static volatile uint8_t tick_counter = 0;
#define TICK_20HZ_DIVIDER (500/20)
#define TICK_1HZ_DIVIDER (20)

void Timer4_Init(void)
{
    /* TIM4 configuration for 500Hz interrupt */
    /* f_TIMER = 16MHz, prescaler = 128 -> 125KHz */
    /* Period = 125000/500 ≈ 250 */
    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_128, 250);
    
    /* Enable update interrupt */
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    
    /* Start timer */
    TIM4_Cmd(ENABLE);
}

void Timer1_PWM_Init(void)
{
    /* TIM1 is configured in pwm_motors.c */
    /* This function is a placeholder if additional TIM1 config is needed */
}

void Timer2_PWM_Init(void)
{
    /* TIM2 is configured in pwm_motors.c */
    /* This function is a placeholder if additional TIM2 config is needed */
}

/* TIM4 Update Interrupt Handler */
/* This should be called from stm8s_it.c */
void TIM4_Update_Handler(void)
{
    static uint8_t counter_20hz = 0;
    static uint8_t counter_1hz = 0;
    
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    
    /* 500Hz tick */
    timer_tick_500hz = 1;
    
    /* Divide down for 20Hz tick */
    counter_20hz++;
    if (counter_20hz >= TICK_20HZ_DIVIDER) {
        counter_20hz = 0;
        timer_tick_20hz = 1;

        /* Divide down for 1Hz tick */
        counter_1hz++;
        if (counter_1hz >= TICK_1HZ_DIVIDER) {
            counter_1hz = 0;
            timer_tick_1hz = 1;
        }
    }
}
