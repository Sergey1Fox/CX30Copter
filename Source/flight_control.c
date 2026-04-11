/**
  ******************************************************************************
  * @file    flight_control.c
  * @brief   Sliding mode control loops for quadcopter
  ******************************************************************************
  */

#include "flight_control.h"
#include "pwm_motors.h"

/* Sliding mode control gains (Q16.16) */
/* 0.5 = 32768 */
#define SLIDE_ROLL_GAIN     ((fp_t)32768L)
#define SLIDE_PITCH_GAIN    ((fp_t)32768L)
/* 0.3 = 19661 */
#define SLIDE_VERT_GAIN     ((fp_t)19661L)

/* Scaling: stick range is ~32 (0x60 to 0xA0), angle range is 30 deg */
/* scale = 30/32 = 0.9375 = 61440 in Q16.16 */
#define STICK_TO_ANGLE_SCALE ((fp_t)61440L)

/* External state */
static FC_Targets_t fc_targets;
static FC_Control_t fc_control;

void FlightControl_Init(void)
{
    fc_targets.target_roll = FP_ZERO;
    fc_targets.target_pitch = FP_ZERO;
    fc_targets.target_vertical_speed = FP_ZERO;
    
    fc_control.roll_error = FP_ZERO;
    fc_control.pitch_error = FP_ZERO;
    fc_control.vertical_error = FP_ZERO;
    fc_control.roll_control = FP_ZERO;
    fc_control.pitch_control = FP_ZERO;
    fc_control.vertical_control = FP_ZERO;
}

void FlightControl_UpdateTargets(uint8_t right_stick_x, uint8_t right_stick_y,
                                  uint8_t left_stick_y, uint8_t left_stick_x)
{
    fp_t right_x_fp, right_y_fp, left_y_fp, left_x_fp;
    fp_t roll_cmd, pitch_cmd, vert_cmd;
    
    /* Convert stick inputs to fixed-point */
    right_x_fp = INT_TO_FP(right_stick_x);
    right_y_fp = INT_TO_FP(right_stick_y);
    left_y_fp = INT_TO_FP(left_stick_y);
    left_x_fp = INT_TO_FP(left_stick_x);
    
    /* Right stick X -> Roll angle target */
    /* Dead zone +-3 from center (0x80 = 128) */
    roll_cmd = FP_SUB(right_x_fp, STICK_RIGHT_CENTER_FP);
    roll_cmd = fp_deadzone(roll_cmd, INT_TO_FP(3));
    /* Scale to +-30 degrees */
    fc_targets.target_roll = FP_MUL(roll_cmd, STICK_TO_ANGLE_SCALE);
    
    /* Right stick Y -> Pitch angle target */
    pitch_cmd = FP_SUB(right_y_fp, STICK_RIGHT_CENTER_FP);
    pitch_cmd = fp_deadzone(pitch_cmd, INT_TO_FP(3));
    fc_targets.target_pitch = FP_MUL(pitch_cmd, STICK_TO_ANGLE_SCALE);
    
    /* Left stick Y -> Vertical speed target */
    /* Dead zone +-10 from center */
    vert_cmd = FP_SUB(left_y_fp, STICK_LEFT_Y_CENTER_FP);
    vert_cmd = fp_deadzone(vert_cmd, INT_TO_FP(10));
    /* Scale: full stick = moderate vertical speed */
    /* 128 * 0.01 = 1.28, use 1/128 = 512 in Q16.16 */
    fc_targets.target_vertical_speed = FP_MUL(vert_cmd, INT_TO_FP(1) / INT_TO_FP(128));
    
    /* Left stick X -> Afterburn (affects all motors equally) */
    /* Above center (0x80) triggers afterburn */
    fp_t afterburn = FP_SUB(left_x_fp, STICK_LEFT_Y_CENTER_FP);
    afterburn = fp_deadzone(afterburn, INT_TO_FP(3));
    if (afterburn > 0) {
        /* Add afterburn contribution to vertical speed */
        fc_targets.target_vertical_speed = FP_ADD(fc_targets.target_vertical_speed,
                                                   FP_MUL(afterburn, INT_TO_FP(1) / INT_TO_FP(256)));
    }
}

void FlightControl_ComputeMotorDuty(fp_t current_roll, fp_t current_pitch,
                                     fp_t current_yaw_rate, fp_t vertical_speed)
{
    fp_t roll_err, pitch_err, vert_err;
    fp_t roll_ctrl, pitch_ctrl, yaw_ctrl, vert_ctrl;
    fp_t sign_roll, sign_pitch, sign_vert;
    
    /* Roll error */
    roll_err = FP_SUB(fc_targets.target_roll, current_roll);
    fc_control.roll_error = roll_err;
    
    /* Sliding mode roll control: u = -k * sign(e) * |e| */
    sign_roll = fp_sign(roll_err);
    roll_ctrl = FP_MUL(sign_roll, FP_MUL(SLIDE_ROLL_GAIN, FP_ABS(roll_err)));
    /* Saturate to +-30% duty */
    roll_ctrl = fp_sat(roll_ctrl, -INT_TO_FP(30), INT_TO_FP(30));
    fc_control.roll_control = roll_ctrl;
    
    /* Pitch error */
    pitch_err = FP_SUB(fc_targets.target_pitch, current_pitch);
    fc_control.pitch_error = pitch_err;
    
    /* Sliding mode pitch control */
    sign_pitch = fp_sign(pitch_err);
    pitch_ctrl = FP_MUL(sign_pitch, FP_MUL(SLIDE_PITCH_GAIN, FP_ABS(pitch_err)));
    pitch_ctrl = fp_sat(pitch_ctrl, -INT_TO_FP(30), INT_TO_FP(30));
    fc_control.pitch_control = pitch_ctrl;
    
    /* Yaw control (from gyro, stabilize) */
    yaw_ctrl = FP_MUL(current_yaw_rate, INT_TO_FP(1) / INT_TO_FP(10));
    (void)yaw_ctrl;  /* Reserved for future yaw stabilization */
    
    /* Vertical speed error */
    vert_err = FP_SUB(fc_targets.target_vertical_speed, vertical_speed);
    fc_control.vertical_error = vert_err;
    
    /* Sliding mode vertical control */
    sign_vert = fp_sign(vert_err);
    vert_ctrl = FP_MUL(sign_vert, FP_MUL(SLIDE_VERT_GAIN, FP_ABS(vert_err)));
    /* Saturate to 20%-85% duty cycle range */
    vert_ctrl = fp_sat(vert_ctrl, INT_TO_FP(20), INT_TO_FP(85));
    fc_control.vertical_control = vert_ctrl;
}

void FlightControl_GetMotorOutputs(fp_t *rf, fp_t *rb, fp_t *lf, fp_t *lb)
{
    fp_t base_throttle, roll_adj, pitch_adj, yaw_adj;
    
    /* Base throttle from vertical control */
    base_throttle = fc_control.vertical_control;
    
    /* Roll adjustment: RF/LB same direction, RB/LF opposite */
    roll_adj = FP_MUL(fc_control.roll_control, FP_HALF);
    
    /* Pitch adjustment: RF/RB same direction, LF/LB opposite */
    pitch_adj = FP_MUL(fc_control.pitch_control, FP_HALF);
    
    /* Yaw adjustment: diagonal pairs opposite */
    yaw_adj = FP_MUL(fc_control.vertical_control, INT_TO_FP(1) / INT_TO_FP(20));
    (void)yaw_adj;
    
    /* Compute motor outputs (X configuration) */
    *rf = FP_ADD(FP_ADD(base_throttle, roll_adj), pitch_adj);
    *rb = FP_SUB(FP_SUB(base_throttle, roll_adj), pitch_adj);
    *lf = FP_SUB(FP_ADD(base_throttle, roll_adj), pitch_adj);
    *lb = FP_ADD(FP_SUB(base_throttle, roll_adj), pitch_adj);
    
    /* Saturate to valid range */
    *rf = fp_sat(*rf, INT_TO_FP(PWM_DUTY_MIN), INT_TO_FP(PWM_DUTY_AFTERBURN));
    *rb = fp_sat(*rb, INT_TO_FP(PWM_DUTY_MIN), INT_TO_FP(PWM_DUTY_AFTERBURN));
    *lf = fp_sat(*lf, INT_TO_FP(PWM_DUTY_MIN), INT_TO_FP(PWM_DUTY_AFTERBURN));
    *lb = fp_sat(*lb, INT_TO_FP(PWM_DUTY_MIN), INT_TO_FP(PWM_DUTY_AFTERBURN));
}
