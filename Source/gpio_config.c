/**
  ******************************************************************************
  * @file    gpio_config.c
  * @brief   GPIO configuration for CX30Copter
  ******************************************************************************
  */

#include "gpio_config.h"

/* LED state tracking */
static uint8_t led1_state = 0;
static uint8_t led2_state = 0;
static uint8_t led3_state = 0;

void GPIO_Config(void)
{
    /* Initialize GPIO ports */
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOE);
    
    /* PA1 - LED1 output push-pull */
    GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);
    
    /* PC3 - LED2 output push-pull */
    GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
    
    /* PD0 - LED3 output push-pull */
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
    
    /* SPI pins: PC5(SCK), PC6(MOSI), PC7(MISO) - alternate function push-pull */
    GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);  /* SCK */
    GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST);  /* MOSI */
    GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_FL_NO_IT);       /* MISO */
    
    /* SPI SS: PE5 output */
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_WriteHigh(GPIOE, GPIO_PIN_5);  /* Deselect BK2425 */
    
    /* I2C pins: PB4(SCL), PB5(SDA) - open drain */
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_HIZ_FAST);
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);
    
    /* PB2 - ADC input for battery voltage */
    GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
    
    /* PWM outputs: PC1, PC2, PD2, PD3 - alternate function push-pull */
    GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
}

void LED1_On(void)
{
    GPIO_WriteHigh(LED1_GPIO_PORT, LED1_GPIO_PIN);
    led1_state = 1;
}

void LED1_Off(void)
{
    GPIO_WriteLow(LED1_GPIO_PORT, LED1_GPIO_PIN);
    led1_state = 0;
}

void LED1_Toggle(void)
{
    if (led1_state) {
        LED1_Off();
    } else {
        LED1_On();
    }
}

void LED2_On(void)
{
    GPIO_WriteHigh(LED2_GPIO_PORT, LED2_GPIO_PIN);
    led2_state = 1;
}

void LED2_Off(void)
{
    GPIO_WriteLow(LED2_GPIO_PORT, LED2_GPIO_PIN);
    led2_state = 0;
}

void LED2_Toggle(void)
{
    if (led2_state) {
        LED2_Off();
    } else {
        LED2_On();
    }
}

void LED3_On(void)
{
    GPIO_WriteHigh(LED3_GPIO_PORT, LED3_GPIO_PIN);
    led3_state = 1;
}

void LED3_Off(void)
{
    GPIO_WriteLow(LED3_GPIO_PORT, LED3_GPIO_PIN);
    led3_state = 0;
}

void LED3_Toggle(void)
{
    if (led3_state) {
        LED3_Off();
    } else {
        LED3_On();
    }
}
