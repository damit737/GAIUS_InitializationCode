#include "ExHardware.h"
//#include "SmartDisplay.h"
#include "main.h"
#include "cmsis_os.h"
//#include "model_setting.h"

/**
 * usd in backlight function
 **/
extern TIM_HandleTypeDef htim12;
//---------------------------------

extern osTimerId_t BuzzerTimerHandle;

extern osSemaphoreId_t BuzzerTaskCountingSemHandle;


/**
 * usd in buzzer function
 **/
bool TimerFlag = false;

//---------------------------------


void Backlight_Control ( BACKLIGHT_CONTROL BL_Ctrl )
{
	switch( BL_Ctrl )
	{
		//==============================================
		case BL_ON:
			
			HAL_GPIO_WritePin( BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET ); // Set the EN pin of BL driver IC to high, enable it
			Set_Backlight_Duty( *brightness );
			HAL_TIM_PWM_Start( &htim12, TIM_CHANNEL_2 );
			
			break;
		//==============================================
		case BL_OFF:
		default:
			
			HAL_TIM_PWM_Stop( &htim12, TIM_CHANNEL_2 );
			Set_Backlight_Duty( 0 );
			HAL_GPIO_WritePin( BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_RESET ); // Set the EN pin of BL driver IC to low, disable it
			
			break;
		//==============================================
	}
}

void Set_Backlight_Duty ( uint8_t d )
{
	#define MaxTrueDuty 100
	#define minTrueDuty 7
	
	d = ( d > 100 ) ? 100 : d;
	
	/*if( d < 20 )
	{
		d = 0;
		goto set_duty;
	}*/
	
	*brightness = d;
	
	if( d == 0 )
		goto set_duty;

	d = ((MaxTrueDuty - minTrueDuty) * d ) / 100 + minTrueDuty;
	
	set_duty:
	__HAL_TIM_SET_COMPARE( &htim12, TIM_CHANNEL_2, d );
}

uint8_t Get_Backlight_Duty ( void )
{
	return *brightness;
}
