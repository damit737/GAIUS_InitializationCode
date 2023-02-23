/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
typedef struct
{
	unsigned char screen_view;
	unsigned short index;
	unsigned short subindex;
	unsigned int   val;

}ReceiverData_t;

typedef struct {
	uint16_t cob_id;	/**< message's ID */
	uint8_t rtr;		/**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
	uint8_t len;		/**< message's length (0 to 8) */
	uint8_t data[8]; /**< message's datas */
} Message;
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
#define LTDC_DISP_Pin GPIO_PIN_9
#define LTDC_DISP_GPIO_Port GPIOF
#define CTP_RST_Pin GPIO_PIN_3
#define CTP_RST_GPIO_Port GPIOA
#define CTP_WAKE_Pin GPIO_PIN_4
#define CTP_WAKE_GPIO_Port GPIOC
#define CTP_INT_Pin GPIO_PIN_5
#define CTP_INT_GPIO_Port GPIOC
#define GPIO_OUT_BEEP_Pin GPIO_PIN_14
#define GPIO_OUT_BEEP_GPIO_Port GPIOB
#define BACKLIGHT_Pin GPIO_PIN_15
#define BACKLIGHT_GPIO_Port GPIOB
#define RTS_Pin GPIO_PIN_12
#define RTS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define FLASH_CS_PIN  GPIO_PIN_15
#define FLASH_GPIO_CS GPIOA
#define CACHE_SIZE 5*1024*1024
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
