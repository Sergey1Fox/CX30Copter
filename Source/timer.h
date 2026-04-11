/**
  ******************************************************************************
  * @file    timer.h
  * @brief   Timer configuration for control loop timing
  *          TIM4 for system tick (330Hz control loop)
  ******************************************************************************
  */

#ifndef __TIMER_H
#define __TIMER_H

#include "stm8s.h"

/* Control loop rates */
#define FLIGHT_LOOP_RATE      500   /* Hz when in flight mode */
#define IDLE_LOOP_RATE        20    /* Hz when idle */

/* External flags */
extern volatile uint8_t timer_tick_500hz;
extern volatile uint8_t timer_tick_20hz;

void Timer4_Init(void);
void Timer1_PWM_Init(void);
void Timer2_PWM_Init(void);
void TIM4_Update_Handler(void);

#endif /* __TIMER_H */
