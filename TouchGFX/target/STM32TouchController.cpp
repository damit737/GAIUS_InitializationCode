/**
  ******************************************************************************
  * File Name          : STM32TouchController.cpp
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* USER CODE BEGIN STM32TouchController */

#include <STM32TouchController.hpp>
#include <main.h>
#include "cmsis_os.h"
extern "C" I2C_HandleTypeDef hi2c2;

//extern "C"
//{
//	uint32_t LCD_GetXSize();
//	uint32_t LCD_GetYSize();
//}
typedef enum{
	NTP = 0x00,
	CTP,
	RTP,
}TP_TYPE;

TP_TYPE touch_type;

typedef enum{
	unknown = 0x00,
	FT5426,
	ILI2130,
}ctp_driver_t;

ctp_driver_t ctp_driver = unknown;

//I2C_HandleTypeDef *phi2c = &hi2c2;

int check_flag = 0; uint8_t wakeUp = 0,wakeUp_flag = 0;

void STM32TouchController::init()
{
    /**
     * Initialize touch controller and driver
     *
     */
	touch_type = CTP;

	HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_RESET); // CTP reset pin set low
	HAL_Delay( 20 );
	HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_SET); // CTP reset pin set high
	HAL_Delay( 20 );


	/* ----------------------------- */
	// Identify the CTP drive model
	uint8_t Buf[ 64 ];

	/* FT5426 */
	if(HAL_I2C_Mem_Read( &hi2c2, 0x70, 0x02, I2C_MEMADD_SIZE_8BIT, &Buf[0], 5, 10 ) == HAL_OK)
		ctp_driver = FT5426;
	/* ILI2130 */
	else if( HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x10, I2C_MEMADD_SIZE_8BIT, &Buf[0], 64, 10 ) == HAL_OK )
		ctp_driver = ILI2130;
}

bool STM32TouchController::sampleTouch(int32_t& x, int32_t& y)
{
    /**
     * By default sampleTouch returns false,
     * return true if a touch has been detected, otherwise false.
     *
     * Coordinates are passed to the caller by reference by x and y.
     *
     * This function is called by the TouchGFX framework.
     * By default sampleTouch is called every tick, this can be adjusted by HAL::setTouchSampleRate(int8_t);
     *
     */
	switch( touch_type )
	{
		//====================================================================//
		case RTP:
			break;
		//====================================================================//
		case CTP:

			if( ctp_driver == unknown )
			{
				uint8_t Buf[ 64 ];

				/* FT5426 */
				if(HAL_I2C_Mem_Read( &hi2c2, 0x70, 0x02, I2C_MEMADD_SIZE_8BIT, &Buf[0], 5, 10 ) == HAL_OK)
					ctp_driver = FT5426;
				/* ILI2130 */
				else if( HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x10, I2C_MEMADD_SIZE_8BIT, &Buf[0], 64, 10 ) == HAL_OK )
					ctp_driver = ILI2130;
			}


			if( ctp_driver == FT5426 )
			{
				// +------+-----------+--------+--------+--------+--------+--------+--------+--------+--------+
				// | Addr |    Name   |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
				// +------+-----------+--------+--------+--------+--------+--------+--------+--------+--------+
				// | 0x02 | Cur Point |      			  Number of touch points[7:0]  						  |
				// +------+-----------+-----------------+--------+--------+-----------------------------------+
				// | 0x03 | TOUCH1_XH | 1st Event Flag  |        |		  | 	1st Touch X Position[11:8]    |
				// +------+-----------+-----------------+--------+--------+-----------------------------------+
				// | 0x04 | TOUCH1_XL | 				  1st Touch X Position[7:0]							  |
				// +------+-----------+-----------------------------------------------------------------------+
				// | 0x05 | TOUCH1_YH | 	  1st Touch ID[3:0]			  |     1st Touch Y Position[11:8]    |
				// +------+-----------+-----------------------------------+-----------------------------------+
				// | 0x06 | TOUCH1_YL | 				  1st Touch Y Position[7:0]							  |
				// +------+-----------+-----------------------------------------------------------------------+
				uint8_t Buf[ 10 ];
				uint8_t event;
				uint8_t ID;
				uint32_t sx,sy;

				if(HAL_I2C_Mem_Read( &hi2c2, 0x70, 0x02, I2C_MEMADD_SIZE_8BIT, &Buf[0], 5, 10 ) == HAL_OK)
				{
					event = (Buf[1] >> 6) & 0x03;
					ID = (Buf[3] >> 4) & 0x0F;

					if( ID == 0 )
					{
						if( ( event == 0x00 ) || ( event == 0x02 ) )
						{
							sx = ((Buf[1] & 0x0F) << 8) | Buf[2];
							sy = ((Buf[3] & 0x0F) << 8) | Buf[4];
							x = (sx * 100) / 224;
							y = (sy * 15) / 32;
							// printf("x %d,sx %d, y %d,sy %d\r\n", x, sx, y, sy );
							return true;
						}
					}
				}
			}
			else if( ctp_driver == ILI2130 )
			{
				uint8_t Buf[ 64 ];
				uint32_t sx,sy;
				uint32_t tempX, tempY;

				if( HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x10, I2C_MEMADD_SIZE_8BIT, &Buf[0], 32, 10 ) == HAL_OK )
				{
					if( Buf[0] == 0x48 )
					{
						if( Buf[1] & 0x40 ) // Tip switch
						{
							if( (Buf[1] & 0x3F) == 0x00 ) // Point ID = 0
							{
								sx = ( Buf[3] << 8 ) | Buf[2];
								sy = ( Buf[5] << 8 ) | Buf[4];

								x = ( sx * 800 ) / 0x4000;
								y = ( sy * 480 ) / 0x4000;
								check_flag = 1;
							}
						}
					}
//					else
//					{
//						if(wakeUp_flag == 0)
//						wakeUp ++;
//					}
					HAL_I2C_Master_Receive( &hi2c2, 0x83, &Buf[0], 32, 10 );

					if(check_flag == 1)
					{
						check_flag = 0;
						return true;
					}
//					if(wakeUp == 255 && wakeUp_flag == 0)
//					{
//						wakeUp_flag = 1;
//						HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x30, I2C_MEMADD_SIZE_8BIT, &Buf[0], 10, 10 );
//					}
				}
				else
				{
//					HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x31, I2C_MEMADD_SIZE_8BIT, &Buf[0], 10, 10 );


					HAL_I2C_DeInit(&hi2c2);
					HAL_I2C_Init(&hi2c2);
					HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_RESET);
					HAL_Delay( 20 );
					HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_SET);
					HAL_Delay( 1000 );
//					HAL_I2C_Mem_Read( &hi2c2, 0x82, 0x31, I2C_MEMADD_SIZE_8BIT, &Buf[0], 10, 10 );
				}
//				osDelay(1);


			}
			else {}

		break;
	//====================================================================//
	default:
		break;
	//====================================================================//
	}
	return false;
}

/* USER CODE END STM32TouchController */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
