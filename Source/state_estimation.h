/**
  ******************************************************************************
  * @file    state_estimation.h
  * @brief   State estimation for quadcopter
  *          Calculates: Roll, Pitch, Yaw angles
  *                      Wroll, Wpitch, Yaw angular speeds
  *                      x, y, z center mass position
  *                      Vx, Vy, Vz center mass speed
  *                      Ax, Ay, Az center mass acceleration
  ******************************************************************************
  */

#ifndef __STATE_ESTIMATION_H
#define __STATE_ESTIMATION_H

#include "stm8s.h"
#include "fixedpoint.h"

/* Quadcopter state vector */
typedef struct {
    /* Angles (degrees in Q16.16) */
    fp_t roll;
    fp_t pitch;
    fp_t yaw;
    
    /* Angular rates (degrees/sec in Q16.16) */
    fp_t wroll;
    fp_t wpitch;
    fp_t wyaw;
    
    /* Position (meters in Q16.16, initial = 0,0,0 at takeoff) */
    fp_t x;
    fp_t y;
    fp_t z;
    
    /* Velocity (m/s in Q16.16) */
    fp_t Vx;
    fp_t Vy;
    fp_t Vz;
    
    /* Acceleration (m/s^2 in Q16.16) */
    fp_t Ax;
    fp_t Ay;
    fp_t Az;
} QuadState_t;

/* External state */
extern QuadState_t quad_state;

void StateEstimation_Init(void);
void StateEstimation_Update(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                            int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

#endif /* __STATE_ESTIMATION_H */
