/**
  ******************************************************************************
  * @file    fixedpoint.c
  * @brief   Fixed-point arithmetic library implementation
  *          Q16.16 format, 32-bit only (STM8 has no int64_t support)
  ******************************************************************************
  */

#include "fixedpoint.h"

/**
  * @brief  Q16.16 multiply: result = (a * b) >> 16
  *
  *         Split into 16-bit halves to avoid overflow:
  *         a = a_hi * 65536 + a_lo  (a_lo is unsigned 0..65535)
  *         b = b_hi * 65536 + b_lo
  *
  *         (a * b) >> 16 = a_hi*b_hi*65536 + a_hi*b_lo + b_hi*a_lo + (a_lo*b_lo)>>16
  *
  *         All intermediate terms fit in 32-bit signed.
  */
fp_t fp_mul(fp_t a, fp_t b)
{
    int32_t a_hi = a >> 16;
    uint32_t a_lo = (uint32_t)a & 0xFFFFu;
    int32_t b_hi = b >> 16;
    uint32_t b_lo = (uint32_t)b & 0xFFFFu;
    
    /* Cross terms: each fits in 32-bit since a_hi, b_hi are ~16-bit and b_lo, a_lo are 16-bit */
    int32_t t1 = a_hi * (int32_t)b_lo;
    int32_t t2 = b_hi * (int32_t)a_lo;
    
    /* Low * low / 65536 */
    int32_t t3 = (int32_t)((a_lo * b_lo) >> 16);
    
    /* High * high << 16 */
    int32_t t4 = (a_hi * b_hi) << 16;
    
    return (fp_t)(t4 + t1 + t2 + t3);
}

/**
  * @brief  Q16.16 divide: result = (a << 16) / b
  *
  *         Long-division approach to avoid overflow.
  *         Handles signs explicitly.
  */
fp_t fp_div(fp_t a, fp_t b)
{
    int8_t sign = 1;
    
    if (b == 0) return FP_ZERO;
    
    /* Handle signs */
    if (a < 0) { sign = (int8_t)-sign; a = -a; }
    if (b < 0) { sign = (int8_t)-sign; b = -b; }
    
    /* Now a, b are non-negative */
    uint32_t ua = (uint32_t)a;
    uint32_t ub = (uint32_t)b;
    
    /* We want (ua << 16) / ub without actually shifting ua by 16.
     * ua_hi = ua >> 16  (upper 16 bits of a)
     * ua_lo = ua & 0xFFFF (lower 16 bits of a)
     * 
     * Result = (ua_hi * 2^32 + ua_lo * 2^16) / ub
     *        = ua_hi * (2^32/ub) + ua_lo * (2^16/ub)   [not quite, need long division]
     * 
     * Use iterative approach:
     * Step 1: high_16 = (ua_hi << 16) / ub  -> integer part of result >> 16
     * Step 2: rem = (ua_hi << 16) % ub
     * Step 3: low_16 = ((rem + ua_lo) << 16) / ub  -> fractional part
     */
    
    uint32_t ua_hi = ua >> 16;
    uint32_t ua_lo = ua & 0xFFFFu;
    
    /* ua_hi << 16 can overflow 32-bit, so we do it carefully */
    /* Actually ua_hi is at most 16 bits, ua_hi << 16 fits in 32-bit unsigned */
    uint32_t shifted_hi = ua_hi << 16;
    
    uint32_t q1 = shifted_hi / ub;
    uint32_t r1 = shifted_hi % ub;
    
    /* Combine remainder with low part, then shift by 16 */
    /* r1 + ua_lo can be at most ~32 bits, then << 16 may overflow
     * So compute: ((r1 + ua_lo) / ub) << 16 + (((r1 + ua_lo) % ub) << 16) / ub
     */
    uint32_t combined = r1 + ua_lo;
    uint32_t q2_int = combined / ub;
    uint32_t r2 = combined % ub;
    uint32_t q2_frac = (r2 << 16) / ub;
    
    uint32_t result = (q1 << 16) + (q2_int << 16) + q2_frac;
    
    return sign < 0 ? -(fp_t)result : (fp_t)result;
}

/**
  * @brief  Square root using Newton's method
  */
int32_t fp_sqrt(int32_t x)
{
    int32_t res;
    int32_t temp;
    
    if (x <= 0) return 0;
    if (x == 1) return 1;
    
    res = x / 2;
    do {
        temp = res;
        res = (res + x / res) / 2;
    } while (res < temp);
    
    return temp;
}
