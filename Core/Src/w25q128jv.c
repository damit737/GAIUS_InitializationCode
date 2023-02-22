#include "w25q128jv.h"

extern SPI_HandleTypeDef hspi1;

#define FLASH_MAXSIZE   ( 1024*1024*16 )
#define SIZEBLOCK ( 64*1024 )
#define SIZESECTOR ( 4*1024 )

uint8_t Flash_SPI_Send_Byte(uint8_t Flash_Command)
{
  uint8_t rxBf = 0;
  uint8_t txBf = Flash_Command;

  FLASH_CS_Select;

  HAL_SPI_TransmitReceive( &hspi1, &txBf, &rxBf, 1, 1 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect; 
  
  return rxBf;  
}


uint8_t Inital_SPI_Flash(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.Pin = FLASH_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init( FLASH_GPIO_CS, &GPIO_InitStruct );
    
  FLASH_CS_Select;
    
	Flash_SPI_Send_Byte( Enable_Reset );

	HAL_Delay( 5 );

	Flash_SPI_Send_Byte( Reset_Device );

	HAL_Delay( 5 );
	
	#if 1
	while( 1 ) {

		HAL_Delay( 10 );
		
		if( _Manufacturer_ID == Flash_Read_Status( JEDEC_ID ) ) {
			break;			
		}
	}
	#endif
	
	// W25Q SPI FLASH Manufacturer ID -> 0xEF
	if ( 0xEF != Flash_Read_Status( JEDEC_ID ) ) {

		return 1;
	}

	// Hardware Unprotected   
	// When /WP pin is high the Status register is unlocked and can 
	// be written to after a Write Enable instruction, WEL=1.  

	// SRL SRP /WP
	// 0   1   1 
	
  #if 0
	uint8_t status_1 = 0xFF;
	uint8_t status_2 = 0xFF;

	status_1 = Flash_Read_Status(Read_Status_Register_1);
	status_2 = Flash_Read_Status(Read_Status_Register_2);

	Flash_Write_Enable();
	Flash_Write_Status( status_1 | 0x80, status_2 & 0xFE ); 	
	#endif	
    
  #if 0
  // Flash_Sector_Erase( 0 );
  // Flash_Byte_Program( 0, 0x77 );
  uint8_t array[] = { 0, 1, 2, 3, 4, 5 };
  
  Flash_Block_Erase( 0 );
  
  Flash_Program( 0, array, sizeof( array ) );
  
  uint8_t tmp[ 6 ] = { 0, 0, 0, 0, 0, 0 };
  
  Flash_Read( 0, tmp, sizeof( array ) );
  
  #endif
    
	return 0;
}

uint8_t Flash_Read_Status( uint8_t Register )
{
  uint8_t rxBf[ 2 ] = { 0, 0 };
  uint8_t txBf[ 2 ] = { Register, 0 };
  
  FLASH_CS_Select;

  if(HAL_SPI_TransmitReceive_DMA( &hspi1, txBf, rxBf, 2 ) != HAL_OK) {
  
  }

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect;
	
  return rxBf[1];
}

void Flash_Write_Status(uint8_t Flash_Status_1,uint8_t Flash_Status_2)
{
  uint8_t RX_Bf[3] = {0,0};
  uint8_t TX_Bf[3] = {0,0};
    
  TX_Bf[0] = Volatile_SR_Write_Enable;
  
  FLASH_CS_Select;

  HAL_SPI_TransmitReceive_DMA( &hspi1, TX_Bf, RX_Bf, 1 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect; 

  TX_Bf[0] = Write_Status_Register_1;
  TX_Bf[1] = Flash_Status_1;
  TX_Bf[2] = Flash_Status_2;

  FLASH_CS_Select;

  HAL_SPI_TransmitReceive_DMA( &hspi1, TX_Bf, RX_Bf, 3 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect; 
}
 
void Flash_Chip_Erase( void )
{
	Flash_Write_Enable();
	Flash_SPI_Send_Byte( Chip_Erase ); 
	
	// Check_Busy	
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) );
}

void Flash_Block_Erase(unsigned long Block_Address)
{
	Flash_Write_Enable();
  
	uint8_t txBuf[4] = { 

		_64K_Black_Erase, 
		Block_Address >> 16, 
		Block_Address >> 8, 
		Block_Address 
	};

	FLASH_CS_Select;

  HAL_SPI_Transmit_DMA( &hspi1, txBuf, 4 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect;

	// Check_Busy
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ); 
}


void Flash_Sector_Erase( unsigned long sector_addr )
{
  Flash_Write_Enable();

	uint8_t txBuf[] = { 

		Sector_Erase, 
		sector_addr >> 16, 
		sector_addr >> 8, 
		sector_addr 
	};

	FLASH_CS_Select;

  HAL_SPI_Transmit_DMA( &hspi1, txBuf, 4 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect; 

	// Check_Busy
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ); 
}


void Flash_Write_Enable(void)
{
  Flash_SPI_Send_Byte(Write_Enable);  
}

void Flash_Write_Disable(void)
{
  Flash_SPI_Send_Byte(Write_Disable);  
}

void Flash_Read( unsigned int ADDR, unsigned char *out, unsigned int size )
{
	uint8_t CMD_ADD[5];
	
	CMD_ADD[0] = Fast_Read_Data;
	CMD_ADD[1] = ( ( ADDR & 0xFF0000 ) >> 16);
	CMD_ADD[2] = ( ( ADDR & 0x00FF00 ) >> 8 );
	CMD_ADD[3] = ( ( ADDR & 0x0000FF ) >> 0 );
	CMD_ADD[4] = 0x00;  // Dummy
	
	FLASH_CS_Select;

	// Send Command and Address
	if( HAL_SPI_Transmit_DMA( &hspi1, CMD_ADD, 5 ) != HAL_OK) {

		// goto _Fail;
	} 

	while( ( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) );
	
	// receiver Data  
	if( HAL_SPI_Receive_DMA( &hspi1, out, size ) != HAL_OK) {

		// goto _Fail;
	}    

	while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY );

	FLASH_CS_DeSelect;
}

unsigned char Flash_Byte_Program(unsigned long Flash_ADD,uint8_t Program_Data)
{
  
	if(Flash_ADD > 0x01000000) return 1;
	
	Flash_Write_Enable();

	while(0x01 & Flash_Read_Status(Read_Status_Register_1));

	uint8_t txBuf[ 5 ] = { 

		0x02, 
		Flash_ADD >> 16, 
		Flash_ADD >> 8, 
		Flash_ADD,
		Program_Data
	};

	FLASH_CS_Select;

  HAL_SPI_Transmit_DMA( &hspi1, txBuf, 5 );

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) );

  FLASH_CS_DeSelect;

  // 等待寫入結束
	while(0x01 & Flash_Read_Status(Read_Status_Register_1));    

	return 0;
}

unsigned char Flash_Program( unsigned long ADDR, uint8_t *Data, uint16_t size )
{
	// 16Mbyte 
	if( ( ADDR + size ) > FLASH_MAXSIZE )
		return 0;

	uint8_t buf[ FLASH_PAGE_SIZE + 4 ];
	uint32_t addrCurrent = ADDR;
	uint32_t check = size % FLASH_PAGE_SIZE;
	uint32_t count = ( size - check ) / FLASH_PAGE_SIZE;
	uint32_t i;

	for ( i = 0; i < count; i++ ) { 

		buf[0] = 0x02;
		buf[1] = (addrCurrent >> 16);
		buf[2] = (addrCurrent >> 8);
		buf[3] = (addrCurrent & 0xFF);
		memcpy( &buf[4], Data, FLASH_PAGE_SIZE );

		// The WEL bit must be set prior to every Page Program,
		Flash_Write_Enable();

		// The WEL status bit is cleared to 0 when the device is write disabled
		while( ( 0x02 & Flash_Read_Status(Read_Status_Register_1) ) == 0 );

		FLASH_CS_Select;

		// Send Data, page prog time max 3mSec
		if( HAL_SPI_Transmit( &hspi1, buf, FLASH_PAGE_SIZE + 4, 5 ) != HAL_OK )
			return 0;

		FLASH_CS_DeSelect;

		// check flash busy flag ( write in progress )
		while( 0x01 & Flash_Read_Status(Read_Status_Register_1) );    

		Data += FLASH_PAGE_SIZE;
		addrCurrent += FLASH_PAGE_SIZE;
	}  

	if( check != 0 ) {

		buf[0] = 0x02;
		buf[1] = (addrCurrent >> 16);
		buf[2] = (addrCurrent >> 8);
		buf[3] = (addrCurrent & 0xFF);

		memcpy( &buf[4], Data, check );

		Flash_Write_Enable();

		// The WEL status bit is cleared to 0 when the device is write disabled
		while( ( 0x02 & Flash_Read_Status(Read_Status_Register_1) ) == 0 );

		FLASH_CS_Select; 

		// Send Data, page prog time max 3mSec
		if( HAL_SPI_Transmit( &hspi1, buf, check + 4, 5 ) != HAL_OK )
		return 0;

		FLASH_CS_DeSelect;

		// check flash busy flag ( write in progress )
		while( 0x01 & Flash_Read_Status(Read_Status_Register_1) );
	}

	return 1;
}

