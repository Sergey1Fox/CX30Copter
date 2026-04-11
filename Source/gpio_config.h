/**
  ******************************************************************************
  * @file    gpio_config.h
  * @brief   GPIO pin configuration for CX30Copter
  ******************************************************************************
  */

#ifndef __GPIO_CONFIG_H
#define __GPIO_CONFIG_H

#include "stm8s.h"

/* LED Pin Definitions */
/* PA1 - LED1: Flashes during init, solid after BK2425 payload received */
#define LED1_GPIO_PORT  (GPIOA)
#define LED1_GPIO_PIN   (GPIO_PIN_1)

/* PC3 - LED2: Off during init, on when payload received, flashes if no payload >10 requests */
#define LED2_GPIO_PORT  (GPIOC)
#define LED2_GPIO_PIN   (GPIO_PIN_3)

/* PD0 - LED3: Battery voltage indicator. On if >3.2V, flashes 500ms if <=3.2V */
#define LED3_GPIO_PORT  (GPIOD)
#define LED3_GPIO_PIN   (GPIO_PIN_0)

/* SPI Pins for BK2425 */
#define SPI_SCK_GPIO_PORT   (GPIOC)
#define SPI_SCK_GPIO_PIN    (GPIO_PIN_5)
#define SPI_MOSI_GPIO_PORT  (GPIOC)
#define SPI_MOSI_GPIO_PIN   (GPIO_PIN_6)
#define SPI_MISO_GPIO_PORT  (GPIOC)
#define SPI_MISO_GPIO_PIN   (GPIO_PIN_7)
#define SPI_SS_GPIO_PORT    (GPIOE)
#define SPI_SS_GPIO_PIN     (GPIO_PIN_5)

/* I2C Pins for MPU6050 */
#define I2C_SCL_GPIO_PORT   (GPIOB)
#define I2C_SCL_GPIO_PIN    (GPIO_PIN_4)
#define I2C_SDA_GPIO_PORT   (GPIOB)
#define I2C_SDA_GPIO_PIN    (GPIO_PIN_5)

/* ADC Pin for Battery Voltage */
#define ADC_BATTERY_GPIO_PORT   (GPIOB)
#define ADC_BATTERY_GPIO_PIN    (GPIO_PIN_2)

/* PWM Motor Outputs */
/* PC1 - Right Front Motor (Timer 1 Channel 1) */
#define PWM_RF_GPIO_PORT    (GPIOC)
#define PWM_RF_GPIO_PIN     (GPIO_PIN_1)

/* PC2 - Right Back Motor (Timer 1 Channel 2) */
#define PWM_RB_GPIO_PORT    (GPIOC)
#define PWM_RB_GPIO_PIN     (GPIO_PIN_2)

/* PD3 - Left Front Motor (Timer 2 Channel 1) */
#define PWM_LF_GPIO_PORT    (GPIOD)
#define PWM_LF_GPIO_PIN     (GPIO_PIN_3)

/* PD2 - Left Back Motor (Timer 2 Channel 2) */
#define PWM_LB_GPIO_PORT    (GPIOD)
#define PWM_LB_GPIO_PIN     (GPIO_PIN_2)

void GPIO_Config(void);
void LED1_On(void);
void LED1_Off(void);
void LED1_Toggle(void);
void LED2_On(void);
void LED2_Off(void);
void LED2_Toggle(void);
void LED3_On(void);
void LED3_Off(void);
void LED3_Toggle(void);

#endif /* __GPIO_CONFIG_H */
