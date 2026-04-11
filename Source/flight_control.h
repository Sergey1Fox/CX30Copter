/**
  ******************************************************************************
  * @file    flight_control.h
  * @brief   Sliding mode control loops for quadcopter
  *          - Pitch control (Right stick X -> roll angle, max +-30 deg)
  *          - Roll control (Right stick Y -> roll angle, max +-30 deg)
  *          - Vertical speed control (Left stick Y, dead zone +-10)
  ******************************************************************************
  */

#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "stm8s.h"
#include "fixedpoint.h"

/* Control constants (pre-computed Q16.16 values) */
/* 30 degrees = 30 * 65536 = 1966080 */
#define MAX_ANGLE_FP          ((fp_t)1966080L)
/* 3 degrees deadzone = 3 * 65536 = 196608 */
#define ANGLE_DEADZONE_FP     ((fp_t)196608L)
/* Vertical speed deadzone 10 = 10 * 65536 = 655360 */
#define VERTICAL_DEADZONE_FP  ((fp_t)655360L)

/* Stick input range */
#define STICK_RIGHT_MIN_FP    ((fp_t)6291456L)   /* 96 * 65536 */
#define STICK_RIGHT_MAX_FP    ((fp_t)10485760L)  /* 160 * 65536 */
#define STICK_RIGHT_CENTER_FP ((fp_t)8388608L)   /* 128 * 65536 */
#define STICK_LEFT_Y_MIN_FP   ((fp_t)0L)
#define STICK_LEFT_Y_MAX_FP   ((fp_t)16711680L)  /* 255 * 65536 */
#define STICK_LEFT_Y_CENTER_FP ((fp_t)8388608L)  /* 128 * 65536 */

/* Sliding mode controller output: motor duty adjustments */
typedef struct {
    fp_t target_roll;
    fp_t target_pitch;
    fp_t target_vertical_speed;
} FC_Targets_t;

typedef struct {
    fp_t roll_error;
    fp_t pitch_error;
    fp_t vertical_error;
    fp_t roll_control;
    fp_t pitch_control;
    fp_t vertical_control;
} FC_Control_t;

void FlightControl_Init(void);
void FlightControl_UpdateTargets(uint8_t right_stick_x, uint8_t right_stick_y,
                                   uint8_t left_stick_y, uint8_t left_stick_x);
void FlightControl_ComputeMotorDuty(fp_t current_roll, fp_t current_pitch,
                                     fp_t current_yaw_rate, fp_t vertical_speed);
void FlightControl_GetMotorOutputs(fp_t *rf, fp_t *rb, fp_t *lf, fp_t *lb);

#endif /* __FLIGHT_CONTROL_H */
