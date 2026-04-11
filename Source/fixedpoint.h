/**
  ******************************************************************************
  * @file    fixedpoint.h
  * @brief   Fixed-point arithmetic library for STM8 quadcopter
  *          Uses Q16.16 format (16 bits integer, 16 bits fractional)
  ******************************************************************************
  */

#ifndef __FIXEDPOINT_H
#define __FIXEDPOINT_H

#include "stm8s.h"

/* Q16.16 fixed-point type */
typedef int32_t fp_t;

#define FP_ONE   ((fp_t)0x00010000)
#define FP_ZERO  ((fp_t)0)
#define FP_HALF  ((fp_t)0x00008000)

/* Conversion macros */
#define INT_TO_FP(x)   ((fp_t)((x) << 16))
#define FP_TO_INT(x)   ((int32_t)((x) >> 16))

/* Pre-computed constants for common float values (avoid floating-point on STM8) */
/* Usage: instead of FLOAT_TO_FP(0.5f), use FP_0_5 */
#define FP_0_01   ((fp_t)655)      /* 0.01 */
#define FP_0_05   ((fp_t)3277)     /* 0.05 */
#define FP_0_1    ((fp_t)6554)     /* 0.1 */
#define FP_0_3    ((fp_t)19661)    /* 0.3 */
#define FP_0_5    ((fp_t)32768)    /* 0.5 */
#define FP_0_98   ((fp_t)64225)    /* 0.98 */
#define FP_1_0    ((fp_t)65536)    /* 1.0 */
#define FP_9_81   ((fp_t)642908)   /* 9.81 */
#define FP_57_3   ((fp_t)3754967)  /* 57.3 (rad to deg) */
#define FP_TO_FLOAT(x) ((float)(x) / 65536.0f)  /* Debug only */

/* Basic operations using 32-bit arithmetic only (STM8 is 8-bit, no int64_t support) */
#define FP_ADD(a, b)   ((a) + (b))
#define FP_SUB(a, b)   ((a) - (b))

/* Q16.16 multiplication: (a * b) >> 16 using 32-bit intermediate
   Uses split multiplication to avoid overflow */
#define FP_MUL(a, b)   fp_mul((a), (b))

/* Q16.16 division: (a << 16) / b using 32-bit intermediate */
#define FP_DIV(a, b)   fp_div((a), (b))

/* Absolute value */
#define FP_ABS(x)      ((x) >= 0 ? (x) : -(x))

/* Sign function: returns FP_ONE, FP_ZERO, or -FP_ONE */
static inline fp_t fp_sign(fp_t x) {
    if (x > 0) return FP_ONE;
    if (x < 0) return -FP_ONE;
    return FP_ZERO;
}

/* Dead zone check: returns 0 if |x| <= threshold, else x */
static inline fp_t fp_deadzone(fp_t x, fp_t threshold) {
    if (FP_ABS(x) <= threshold) return FP_ZERO;
    return x;
}

/* Saturate: clamp x between min and max */
static inline fp_t fp_sat(fp_t x, fp_t min, fp_t max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* Square root using Newton's method (integer part) */
int32_t fp_sqrt(int32_t x);

/* Q16.16 multiply: (a * b) >> 16, safe for 32-bit */
fp_t fp_mul(fp_t a, fp_t b);

/* Q16.16 divide: (a << 16) / b, safe for 32-bit */
fp_t fp_div(fp_t a, fp_t b);

#endif /* __FIXEDPOINT_H */
