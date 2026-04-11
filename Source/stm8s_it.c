/**
  ******************************************************************************
  * @file    stm8s_it.c
  * @brief   Interrupt Service Routines for CX30Copter
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s_it.h"
#include "timer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy Interrupt routine
  */
INTERRUPT_HANDLER(NonHandledInterrupt, 25)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*_COSMIC_*/

/**
  * @brief TRAP Interrupt routine
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Top Level Interrupt routine.
  */
INTERRUPT_HANDLER(TLI_IRQHandler, 0)
{
}

/**
  * @brief Auto Wake Up Interrupt routine.
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
}

/**
  * @brief Clock Controller Interrupt routine.
  */
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
}

/**
  * @brief External Interrupt PORTA Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
}

/**
  * @brief External Interrupt PORTB Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
}

/**
  * @brief External Interrupt PORTC Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
}

/**
  * @brief External Interrupt PORTD Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
}

/**
  * @brief External Interrupt PORTE Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
}

#if defined (STM8S903) || defined (STM8AF622x)
/**
  * @brief External Interrupt PORTF Interrupt routine.
  */
INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{
}
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined (STM8AF52Ax)
/**
  * @brief CAN RX Interrupt routine.
  */
INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{
}

/**
  * @brief CAN TX Interrupt routine.
  */
INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{
}
#endif /* (STM8S208) || (STM8AF52Ax) */

/**
  * @brief SPI Interrupt routine.
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
}

/**
  * @brief Timer1 Update/Overflow/Trigger/Break Interrupt routine.
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  TIM1_ClearFlag(TIM1_FLAG_UPDATE);
}

/**
  * @brief Timer1 Capture/Compare Interrupt routine.
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
}

#if defined (STM8S903) || defined (STM8AF622x)
/**
  * @brief Timer5 Update/Overflow/Break/Trigger Interrupt routine.
  */
INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{
}

/**
  * @brief Timer5 Capture/Compare Interrupt routine.
  */
INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{
}
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */
/**
  * @brief Timer2 Update/Overflow/Break Interrupt routine.
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);
}

/**
  * @brief Timer2 Capture/Compare Interrupt routine.
  */
INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{
}
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8AF626x)
/**
  * @brief Timer3 Update/Overflow/Break Interrupt routine.
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
}

/**
  * @brief Timer3 Capture/Compare Interrupt routine.
  */
INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{
}
#endif /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined (STM8S003) || defined(STM8S001) || defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
/**
  * @brief UART1 TX Interrupt routine.
  */
INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
}

/**
  * @brief UART1 RX Interrupt routine.
  */
INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
}
#endif /* (STM8S208) || (STM8S207) || (STM8S103) || (STM8S001) || (STM8S903) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8AF622x)
/**
  * @brief UART4 TX Interrupt routine.
  */
INTERRUPT_HANDLER(UART4_TX_IRQHandler, 17)
{
}

/**
  * @brief UART4 RX Interrupt routine.
  */
INTERRUPT_HANDLER(UART4_RX_IRQHandler, 18)
{
}
#endif /* (STM8AF622x) */

/**
  * @brief I2C Interrupt routine.
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
}

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
/**
  * @brief UART2 TX interrupt routine.
  */
INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
}

/**
  * @brief UART2 RX interrupt routine.
  */
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
}
#endif /* (STM8S105) || (STM8AF626x) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief UART3 TX interrupt routine.
  */
INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{
}

/**
  * @brief UART3 RX interrupt routine.
  */
INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
}
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief ADC2 interrupt routine.
  */
INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{
}
#else /* STM8S105 or STM8S103 or STM8S903 or STM8AF626x or STM8AF622x */
/**
  * @brief ADC1 interrupt routine.
  */
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
}
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8S903) || defined (STM8AF622x)
/**
  * @brief Timer6 Update/Overflow/Trigger Interrupt routine.
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{
}
#else /* STM8S208 or STM8S207 or STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */
/**
  * @brief Timer4 Update/Overflow Interrupt routine.
  *        This handles the 500Hz control loop tick
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  TIM4_Update_Handler();
}
#endif /* (STM8S903) || (STM8AF622x)*/

/**
  * @brief Eeprom EEC Interrupt routine.
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
