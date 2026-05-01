/**
  ******************************************************************************
  * @file    state_estimation.c
  * @brief   State estimation for quadcopter
  ******************************************************************************
  */

#include "state_estimation.h"

/* MPU6050 scaling factors (Q16.16) */
/* ACCEL: +/-8g -> 4096 LSB/g */
/* 1/4096 = 16 in Q16.16 */
#define ACCEL_SCALE_FP  ((fp_t)16L)
/* 9.81 m/s^2 = 643048 in Q16.16 */
#define G_ACCEL_FP       ((fp_t)643048L)

/* GYRO: +/-2000 deg/s -> 16.4 LSB/deg/s */
/* 1/16.4 = 4005 in Q16.16 */
#define GYRO_SCALE_FP   ((fp_t)4005L)

/* Complementary filter constants (Q16.16) */
/* 0.98 = 64225 */
#define COMP_ALPHA_FP   ((fp_t)64225L)
/* 0.02 = 1311 */
#define COMP_BETA_FP    ((fp_t)1311L)

/* Radians to degrees: 180/pi = 57.2958 = 3754967 in Q16.16 */
#define RAD_TO_DEG_FP   ((fp_t)3754967L)

/* dt for 330Hz: 1/330 = 199 in Q16.16 */
#define DT_330HZ_FP     ((fp_t)199L)
#define DT_500HZ_FP     ((fp_t)131L)

/* External state */
QuadState_t quad_state;

void StateEstimation_Init(void)
{
    /* Initialize all state variables to zero */
    quad_state.roll = FP_ZERO;
    quad_state.pitch = FP_ZERO;
    quad_state.yaw = FP_ZERO;
    
    quad_state.wroll = FP_ZERO;
    quad_state.wpitch = FP_ZERO;
    quad_state.wyaw = FP_ZERO;
    
    quad_state.x = FP_ZERO;
    quad_state.y = FP_ZERO;
    quad_state.z = FP_ZERO;
    
    quad_state.Vx = FP_ZERO;
    quad_state.Vy = FP_ZERO;
    quad_state.Vz = FP_ZERO;
    
    quad_state.Ax = FP_ZERO;
    quad_state.Ay = FP_ZERO;
    quad_state.Az = FP_ZERO;
}

void StateEstimation_Update(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                            int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                            fp_t dt_fp)
{
    fp_t accel_roll, accel_pitch;
    fp_t gyro_roll, gyro_pitch, gyro_yaw;
    fp_t accel_roll_fp, accel_pitch_fp;
    fp_t dt;
    (void)dt_fp;  /* Use fixed dt = 1/500 */
    
    dt = DT_500HZ_FP;
    
    /* Convert raw accel to m/s^2 */
    quad_state.Ax = FP_MUL(FP_MUL(INT_TO_FP(accel_x), ACCEL_SCALE_FP), G_ACCEL_FP);
    quad_state.Ay = FP_MUL(FP_MUL(INT_TO_FP(accel_y), ACCEL_SCALE_FP), G_ACCEL_FP);
    quad_state.Az = FP_MUL(FP_MUL(INT_TO_FP(accel_z), ACCEL_SCALE_FP), G_ACCEL_FP);
    
    /* Convert raw gyro to deg/s */
    gyro_roll = FP_MUL(INT_TO_FP(gyro_x), GYRO_SCALE_FP);
    gyro_pitch = FP_MUL(INT_TO_FP(gyro_y), GYRO_SCALE_FP);
    gyro_yaw = FP_MUL(INT_TO_FP(gyro_z), GYRO_SCALE_FP);
    
    /* Update angular rates */
    quad_state.wroll = gyro_roll;
    quad_state.wpitch = gyro_pitch;
    quad_state.wyaw = gyro_yaw;
    
    /* Calculate roll and pitch from accelerometer (atan2 approximation) */
    /* Roll from Y and Z accel: atan2(Ay, Az) ~ Ay/Az for small angles */
    if (quad_state.Az != 0) {
        accel_roll_fp = FP_DIV(quad_state.Ay, quad_state.Az);
        accel_roll = FP_MUL(accel_roll_fp, RAD_TO_DEG_FP);
    } else {
        accel_roll = quad_state.roll;
    }
    
    /* Pitch from X and Z accel */
    if (quad_state.Az != 0) {
        accel_pitch_fp = FP_DIV(quad_state.Ax, quad_state.Az);
        accel_pitch = FP_MUL(accel_pitch_fp, RAD_TO_DEG_FP);
    } else {
        accel_pitch = quad_state.pitch;
    }
    
    /* Complementary filter: gyro + accel fusion */
    quad_state.roll = FP_ADD(FP_MUL(quad_state.roll, COMP_ALPHA_FP),
                              FP_MUL(accel_roll, COMP_BETA_FP));
    quad_state.roll = FP_ADD(quad_state.roll, FP_MUL(gyro_roll, dt));
    
    quad_state.pitch = FP_ADD(FP_MUL(quad_state.pitch, COMP_ALPHA_FP),
                               FP_MUL(accel_pitch, COMP_BETA_FP));
    quad_state.pitch = FP_ADD(quad_state.pitch, FP_MUL(gyro_pitch, dt));
    
    /* Yaw integration from gyro */
    quad_state.yaw = FP_ADD(quad_state.yaw, FP_MUL(gyro_yaw, dt));
    
    /* Integrate acceleration to get velocity */
    quad_state.Vx = FP_ADD(quad_state.Vx, FP_MUL(quad_state.Ax, dt));
    quad_state.Vy = FP_ADD(quad_state.Vy, FP_MUL(quad_state.Ay, dt));
    quad_state.Vz = FP_ADD(quad_state.Vz, FP_MUL(quad_state.Az, dt));
    
    /* Integrate velocity to get position */
    quad_state.x = FP_ADD(quad_state.x, FP_MUL(quad_state.Vx, dt));
    quad_state.y = FP_ADD(quad_state.y, FP_MUL(quad_state.Vy, dt));
    quad_state.z = FP_ADD(quad_state.z, FP_MUL(quad_state.Vz, dt));
}
