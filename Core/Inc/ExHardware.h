/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXHAREWARE_H
#define __EXHAREWARE_H

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#include "stdint.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum{
	BL_ON = 0x00,
	BL_OFF,
}BACKLIGHT_CONTROL;

typedef enum{
	Beep_ON = 0x00,
	Beep_OFF,
}BEEP_CONTROL;

typedef enum{
	access_read = 0x00,
	access_write,
}access_type;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint8_t *brightness;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

void Backlight_Control ( BACKLIGHT_CONTROL BL_Ctrl );
void Set_Backlight_Duty ( uint8_t d );
uint8_t Get_Backlight_Duty ( void );
void beep_control ( BEEP_CONTROL beep_ctrl );
void buzzer_process ( void );

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif /* __EXHAREWARE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
