/**
  ******************************************************************************
  * File Name          : TouchGFXHAL.cpp
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include <TouchGFXHAL.hpp>

/* USER CODE BEGIN TouchGFXHAL.cpp */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_ltdc.h"
#include "stm32f4xx_hal_spi.h"
#include "cmsis_os.h"
#include "w25q128jv.h"
#include <STM32DMA.hpp>

extern SPI_HandleTypeDef hspi1;

using namespace touchgfx;

void TouchGFXHAL::initialize()
{
    // Calling parent implementation of initialize().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::initialize() must be called to initialize the framework.

    TouchGFXGeneratedHAL::initialize();
}

/**
 * Gets the frame buffer address used by the TFT controller.
 *
 * @return The address of the frame buffer currently being displayed on the TFT.
 */
uint16_t* TouchGFXHAL::getTFTFrameBuffer() const
{
    // Calling parent implementation of getTFTFrameBuffer().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    return TouchGFXGeneratedHAL::getTFTFrameBuffer();
}

/**
 * Sets the frame buffer address used by the TFT controller.
 *
 * @param [in] address New frame buffer address.
 */
void TouchGFXHAL::setTFTFrameBuffer(uint16_t* address)
{
    // Calling parent implementation of setTFTFrameBuffer(uint16_t* address).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::setTFTFrameBuffer(address);
}

/**
 * This function is called whenever the framework has performed a partial draw.
 *
 * @param rect The area of the screen that has been drawn, expressed in absolute coordinates.
 *
 * @see flushFrameBuffer().
 */
void TouchGFXHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    // Calling parent implementation of flushFrameBuffer(const touchgfx::Rect& rect).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::flushFrameBuffer(const touchgfx::Rect& rect) must
    // be called to notify the touchgfx framework that flush has been performed.

    TouchGFXGeneratedHAL::flushFrameBuffer(rect);
}

bool TouchGFXHAL::blockCopy(void* RESTRICT dest, const void* RESTRICT src, uint32_t numBytes)
{
	if ((uint32_t) src >= 0x90000000 && (uint32_t) src < 0x91000000) {

		uint32_t dataOffset = (uint32_t) src - 0x90000000;

		// tickstart = HAL_GetTick();
		// In this example we assume graphics is placed in SPI Flash,
		// and that we have an appropriate function	for copying data from there.

		uint32_t i	= 0, count = 0, check = 0;
		uint32_t dst = (uint32_t) dest;
		uint32_t src = dataOffset;
		uint32_t size = numBytes;
		uint8_t CMD_ADD[5];

		check	= size % 0xFFFF;
		count	= (size - check) / 0xFFFF;

		CMD_ADD[0] = Fast_Read_Data;
		CMD_ADD[1] = ( ( dataOffset & 0xFF0000 ) >> 16);
		CMD_ADD[2] = ( ( dataOffset & 0x00FF00 ) >> 8 );
		CMD_ADD[3] = ( ( dataOffset & 0x0000FF ) >> 0 );
		CMD_ADD[4] = 0x00;  // Dummy

		disableInterrupts();

		FLASH_CS_Select;

		// Send Command and Address
		HAL_SPI_Transmit_DMA( &hspi1, CMD_ADD, 5 );

		while( ( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) );

		for( i = 0; i < count; i++) {

			HAL_SPI_Receive_DMA( &hspi1, (uint8_t *)dst, 0xFFFF );

			while( ( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) );

			dst += 0xFFFF;
		}

		if (check != 0)
			HAL_SPI_Receive_DMA( &hspi1, (uint8_t *)dst, check );

		while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY );

		FLASH_CS_DeSelect;

		enableInterrupts();

		// tickend = HAL_GetTick() - tickstart;

		return true;
	}
	else {

		// For all other addresses, just use the default implementation.
		// This is important, as blockCopy is also used for other things in the core framework.
		return HAL::blockCopy(dest, src, numBytes);
	}

//    return TouchGFXGeneratedHAL::blockCopy(dest, src, numBytes);
}

/**
 * Configures the interrupts relevant for TouchGFX. This primarily entails setting
 * the interrupt priorities for the DMA and LCD interrupts.
 */
void TouchGFXHAL::configureInterrupts()
{
    // Calling parent implementation of configureInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::configureInterrupts();
}

/**
 * Used for enabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::enableInterrupts()
{
    // Calling parent implementation of enableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::enableInterrupts();
}

/**
 * Used for disabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::disableInterrupts()
{
    // Calling parent implementation of disableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::disableInterrupts();
}

/**
 * Configure the LCD controller to fire interrupts at VSYNC. Called automatically
 * once TouchGFX initialization has completed.
 */
void TouchGFXHAL::enableLCDControllerInterrupt()
{
    // Calling parent implementation of enableLCDControllerInterrupt().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    TouchGFXGeneratedHAL::enableLCDControllerInterrupt();
}

/* USER CODE END TouchGFXHAL.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
