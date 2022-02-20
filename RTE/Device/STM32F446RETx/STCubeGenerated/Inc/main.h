/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOC
#define DWM1000_MOSI_Pin GPIO_PIN_1
#define DWM1000_MOSI_GPIO_Port GPIOC
#define DWM1000_MISO_Pin GPIO_PIN_2
#define DWM1000_MISO_GPIO_Port GPIOC
#define DWM1000_IRQ_Pin GPIO_PIN_3
#define DWM1000_IRQ_GPIO_Port GPIOC
#define DWM1000_IRQ_EXTI_IRQn EXTI3_IRQn
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define SENSOR_NSS_Pin GPIO_PIN_4
#define SENSOR_NSS_GPIO_Port GPIOA
#define SENSOR_SCK_Pin GPIO_PIN_5
#define SENSOR_SCK_GPIO_Port GPIOA
#define SENSOR_MISO_Pin GPIO_PIN_6
#define SENSOR_MISO_GPIO_Port GPIOA
#define SENSOR_MOSI_Pin GPIO_PIN_7
#define SENSOR_MOSI_GPIO_Port GPIOA
#define SPI1_INT_Pin GPIO_PIN_4
#define SPI1_INT_GPIO_Port GPIOC
#define SENSOR_NSS2_Pin GPIO_PIN_5
#define SENSOR_NSS2_GPIO_Port GPIOC
#define DWM1000_SCK_Pin GPIO_PIN_10
#define DWM1000_SCK_GPIO_Port GPIOB
#define DWM1000_CSN_Pin GPIO_PIN_12
#define DWM1000_CSN_GPIO_Port GPIOB
#define SENSOR_NSS3_Pin GPIO_PIN_13
#define SENSOR_NSS3_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOB
#define SPI3_INT_Pin GPIO_PIN_5
#define SPI3_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* Note: when enable operating system uCOS-III, you can't use function HAL_Delay(); */
#define USE_OS_UCOS_III 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
