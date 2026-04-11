/**
  ******************************************************************************
  * @file    pwm_motors.h
  * @brief   PWM output driver for 4 motors at 16KHz
  *          PC1 - Right Front (TIM1 CH1)
  *          PC2 - Right Back  (TIM1 CH2)
  *          PD3 - Left Front  (TIM2 CH1)
  *          PD2 - Left Back   (TIM2 CH2)
  ******************************************************************************
  */

#ifndef __PWM_MOTORS_H
#define __PWM_MOTORS_H

#include "stm8s.h"

/* PWM Configuration */
#define PWM_FREQUENCY         16000   /* 16KHz */
#define PWM_DUTY_MIN        20      /* 20% normal flight */
#define PWM_DUTY_MAX        85      /* 85% max normal */
#define PWM_DUTY_AFTERBURN  95      /* 95% afterburn */

/* Motor indices */
#define MOTOR_RIGHT_FRONT   0
#define MOTOR_RIGHT_BACK    1
#define MOTOR_LEFT_FRONT    2
#define MOTOR_LEFT_BACK     3
#define MOTOR_COUNT         4

/* External state */
extern uint8_t motor_duty[MOTOR_COUNT];  /* Duty cycle 0-100 */

void PWM_Motors_Init(void);
void PWM_SetMotorDuty(uint8_t motor, uint8_t duty);
void PWM_SetAllMotors(uint8_t duty);
void PWM_Motors_Enable(void);
void PWM_Motors_Disable(void);

#endif /* __PWM_MOTORS_H */
