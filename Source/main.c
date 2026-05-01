/**
  ******************************************************************************
  * @file    main.c
  * @brief   Main program body for CX30Copter
  *          STM8S005K6T6C quadcopter controller
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "gpio_config.h"
#include "spi_bk2425.h"
#include "i2c_mpu6050.h"
#include "adc_battery.h"
#include "pwm_motors.h"
#include "flight_control.h"
#include "state_estimation.h"
#include "timer.h"
#include "fixedpoint.h"
#include "stm8s_it.h"

/* Private defines -----------------------------------------------------------*/
/* LED timing constants */
#define LED1_INIT_FLASH_PERIOD    300   /* ms, flash period during init */
#define LED2_NO_PAYLOAD_THRESHOLD 10    /* requests before flash starts */
#define LED3_FLASH_PERIOD         500   /* ms, low battery flash */

/* System state */
typedef enum {
    SYS_INIT = 0,
    SYS_IDLE,
    SYS_MOTOR_ON,
    SYS_FLIGHT,
    SYS_FLIGHT_AUTO,
    SYS_FLIGHT_RETURN
} SystemState_t;

typedef enum {
    SYS_LOSSCONTROL_HOLD = 0,
    SYS_LOSSCONTROL_HOLD_AND_DOWN,
    SYS_LOSSCONTROL_RETURN,
} SystemLossControl_t;

/* Private variables ---------------------------------------------------------*/
static SystemState_t sys_state = SYS_INIT;
static SystemLossControl_t sys_losscontrol = SYS_LOSSCONTROL_HOLD;
static volatile uint32_t sys_tick_ms = 0;
static uint8_t led1_flash_counter = 0;
static uint8_t led2_no_payload_counter = 0;
static uint8_t led3_flash_counter = 0;

/* Flight mode flag */
static uint8_t flight_mode = 0;

/* Function prototypes -------------------------------------------------------*/
static void SystemClock_Config(void);
static void SysTick_Init(void);
static void ProcessLEDs(void);
static void ProcessFlightLoop(void);
static void ProcessIdleLoop(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure system clock (HSI 16MHz)
  */
static void SystemClock_Config(void)
{
    /* HSI 16MHz is default, but ensure it's configured */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);  /* 16MHz */
}

/**
  * @brief  Initialize millisecond system tick using TIM4
  */
static void SysTick_Init(void)
{
    /* Use a separate timer for ms tick, or use TIM4 for both */
    /* For simplicity, we'll use a software counter based on TIM4 */
    sys_tick_ms = 0;
}

/**
  * @brief  Process LED indicators
  */
static void ProcessLEDs(void)
{
    /* LED1 (PA1): Flash during init, solid after payload received */
    if (sys_state == SYS_INIT) {
        /* Flash at 300ms period */
        if ((sys_tick_ms % LED1_INIT_FLASH_PERIOD) < (LED1_INIT_FLASH_PERIOD / 2)) {
            LED1_On();
        } else {
            LED1_Off();
        }
    } else {
        /* Solid on after init */
        LED1_On();
    }
    
    /* LED2 (PC3): Off during init, on when payload received */
    if (sys_state == SYS_INIT) {
        LED2_Off();
    } else if (flight_mode) {
        LED2_On();
    } else {
        /* Flash if no payload for >10 requests */
        if (bk_no_payload_count > LED2_NO_PAYLOAD_THRESHOLD) {
            led2_no_payload_counter++;
            if (led2_no_payload_counter % 20 == 0) {  /* Flash every ~60ms at 330Hz */
                LED2_Toggle();
            }
        } else {
            LED2_Off();
        }
    }
    
    /* LED3 (PD0): Battery voltage indicator */
    if (battery_low) {
        /* Flash at 500ms period */
        led3_flash_counter++;
        if (led3_flash_counter >= 165) {  /* ~500ms at 330Hz */
            led3_flash_counter = 0;
            LED3_Toggle();
        }
    } else {
        LED3_On();
    }
}

/**
  * @brief  Process flight control loop (500Hz)
  */
static void ProcessFlightLoop(void)
{
    static uint8_t loop_counter = 0;
    fp_t motor_rf, motor_rb, motor_lf, motor_lb;
    /* dt = 1/500 sec, pre-computed */
    fp_t dt_fp = FP_DIV(INT_TO_FP(1), INT_TO_FP(500));
    
    /* Read BK2425 payload */
    SPI_BK_ReadPayload();
    
    /* Check if in flight mode (payload received) */
    if (bk_payload_received) {
        flight_mode = 1;
        
        /* Read MPU6050 data */
        MPU6050_ReadAllData();
        
        /* Update state estimation */
        StateEstimation_Update(mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z,
                               mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z,
                               dt_fp);
        
        /* Update flight control targets from stick inputs */
        FlightControl_UpdateTargets(bk_payload.right_stick_x, bk_payload.right_stick_y,
                                     bk_payload.left_stick_y, bk_payload.left_stick_x);
        
        /* Compute motor duty cycles */
        FlightControl_ComputeMotorDuty(quad_state.roll, quad_state.pitch,
                                        quad_state.wyaw, quad_state.Vz);
        
        /* Get motor outputs and apply */
        FlightControl_GetMotorOutputs(&motor_rf, &motor_rb, &motor_lf, &motor_lb);
        
        PWM_SetMotorDuty(MOTOR_RIGHT_FRONT, FP_TO_INT(motor_rf));
        PWM_SetMotorDuty(MOTOR_RIGHT_BACK, FP_TO_INT(motor_rb));
        PWM_SetMotorDuty(MOTOR_LEFT_FRONT, FP_TO_INT(motor_lf));
        PWM_SetMotorDuty(MOTOR_LEFT_BACK, FP_TO_INT(motor_lb));
    } else {
        /* No payload - idle */
        flight_mode = 0;
        PWM_SetAllMotors(0);
    }
}

/**
  * @brief  Process idle loop (20Hz)
  */
static void ProcessIdleLoop(void)
{
    /* Read battery voltage */
    ADC_Battery_GetVoltage();
    
    /* Poll BK2425 at lower rate */
    SPI_BK_ReadPayload();
    
    /* If payload received, transition to flight mode */
    if (bk_payload_received) {
        sys_state = SYS_FLIGHT;
    }
}

/**
  * @brief  Main program body
  */
void main(void)
{
    /* System initialization */
    SystemClock_Config();
    
    /* Configure GPIO */
    GPIO_Config();
    
    /* Initialize peripherals */
    SPI_BK_Init();
    I2C_MPU_Init();
    ADC_Battery_Init();
    PWM_Motors_Init();
    
    /* Initialize subsystems */
    FlightControl_Init();
    StateEstimation_Init();
    
    /* Initialize timer for control loop */
    Timer4_Init();
    
    /* Enable interrupts */
    enableInterrupts();
    
    /* Set initial state */
    sys_state = SYS_IDLE;
    
    /* Main loop */
    while (1)
    {
        /* Process LEDs */
        ProcessLEDs();
        
        /* Check system state */
        if (sys_state > SYS_IDLE) {
            /* Flight mode: 500Hz loop */
            if (timer_tick_500hz) {
                timer_tick_500hz = 0;
                ProcessFlightLoop();
            }
        } else {
            /* Idle mode: 20Hz loop */
            if (timer_tick_20hz) {
                timer_tick_20hz = 0;
                ProcessIdleLoop();
            }
        }
        
        /* Watchdog: if no payload for extended period, return to idle */
        if (bk_no_payload_count > 100 && sys_state > SYS_MOTOR_ON) {
            sys_state = SYS_IDLE;
            flight_mode = 0;
            PWM_SetAllMotors(0);
        }
    }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
