
#include "AS5048A.h"

/*   SPI parity calculator   */
uint8_t CalcParityEven(uint16_t data) {

	uint8_t parity = 0;

	while (data){
		parity ^= ( data & 1 );
		data >>= 1;
	}

	return parity;

}


/*   Initialisation   */
AS5048A_Error AS5048A_Init(AS5048A *encoder, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint16_t zeroPos) {

	/*   Store parameters in struct   */
	encoder->spiHandle 	= 	spiHandle;
	encoder->csPort		=	csPort;
	encoder->csPin		=	csPin;

	encoder->zeroPos	=	zeroPos;

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*	Encoder startup   */
	HAL_Delay(10);

	/*   Read Offset compensation bit as communications check  */
	uint16_t OCF;
	status += AS5048A_ReadRegister(encoder, AS5048A_AGCR, &OCF);
	status += ( ( OCF & 0x0200 ) == 0 );

	/*   Clear error register   */
	status += AS5048A_ClearErrorFlags(encoder);

	/*   Set zero position   */
	status += AS5048A_SetZeroPosition(encoder, zeroPos);

	/*   SPI communications check   */
	if ( status != 0 ) {
				return SPI_ERROR_INIT;
			}

	return SPI_SUCCESS;

}


AS5048A_Error AS5048A_ReadRegister(AS5048A *encoder, uint16_t regAddr, uint16_t * data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[4] = {AS5048A_NOP};
	uint8_t RxBuf[4] = {AS5048A_NOP};

	/*   Initialise SPI package variables   */
	uint16_t comPackage;
	uint16_t readPackage;

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Set read transmission package   */
	comPackage = ( regAddr     | ( 0x0001 << 14 ) );					    // Sets Command Package address and read bit
	comPackage = ( comPackage  | CalcParityEven( comPackage ) << 15 );	// Sets Command Package parity bit

	/*   Command package byte segmentation   */
	TxBuf[0] = comPackage >> 8;				// Set 8 MSBs via logical left shift
	TxBuf[1] = comPackage;					// Set 8 LSBs via truncation

	/*   Transmit and receive   */
	status += HAL_SPI_TransmitReceive(encoder->spiHandle, TxBuf, RxBuf, 4, AS5048A_MAX_DELAY);

	/*   Read package byte aggregation   */
	readPackage = ( ( RxBuf[2] << 8 ) | RxBuf[3] );

	/*   SPI error flag check   */
	status += ( ( readPackage & ( 1 << 14 ) ) != 0 );

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR_INIT;
		}

	*data = readPackage & 0x3FFF;		// Save only 14 data bits
	return SPI_SUCCESS;

}


AS5048A_Error AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[6] = {AS5048A_NOP};
	uint8_t RxBuf[6] = {AS5048A_NOP};

	/*   Initialise SPI package variables   */
	uint16_t comPackage;
	uint16_t readPackage;
	uint16_t dataPackage;

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Set read transmission package   */
	comPackage = ( regAddr  | CalcParityEven( regAddr ) << 15 );	// Sets Command Package parity bit

	/*   Command package byte segmentation   */
	TxBuf[0] = comPackage >> 8;					// Set 8 MSBs via logical left shift
	TxBuf[1] = comPackage;						// Set 8 LSBs via truncation

	/*   Set read transmission package   */
	dataPackage = ( data  | CalcParityEven( data ) << 15 );	// Sets Command Package parity bit

	/*   Command package byte segmentation   */
	TxBuf[2] = dataPackage >> 8;				// Set 8 MSBs via logical left shift
	TxBuf[3] = dataPackage;					    // Set 8 LSBs via truncation

	/*   Transmit and receive   */
	status += HAL_SPI_TransmitReceive(encoder->spiHandle, TxBuf, RxBuf, 6, AS5048A_MAX_DELAY);

	/*   Read package byte aggregation   */
	readPackage = ( ( RxBuf[4] << 8 ) | RxBuf[5] );

	/*   SPI error flag check   */
	status += ( ( readPackage & ( 1 << 14 ) ) != 0 );

	/*   SPI data return check   */
	status += ( ( readPackage & 0x3FFF ) != data );		// 14 data bits

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR_INIT;
		}

	return SPI_SUCCESS;
}




AS5048A_Error AS5048A_ClearErrorFlags(AS5048A *encoder) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[6] = {AS5048A_NOP};

	/*   Initialise SPI package variables   */
	uint16_t comPackage;

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Set read transmission package   */
	comPackage = ( AS5048A_CEFR | ( 0x0001 << 14 ) );				// Sets Command Package address and read bit
	comPackage = ( comPackage   | CalcParityEven( comPackage ) << 15 );	// Sets Command Package parity bit

	/*   Command package byte segmentation   */
	TxBuf[0] = comPackage >> 8;				// Set 8 MSBs via logical left shift
	TxBuf[1] = comPackage;					// Set 8 LSBs via truncation

	/*   Transmit and receive   */
	status += HAL_SPI_Transmit(encoder->spiHandle, TxBuf, 6, AS5048A_MAX_DELAY);

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR_INIT;
		}

	return SPI_SUCCESS;

}

AS5048A_Error AS5048A_SetZeroPosition(AS5048A *encoder, uint16_t zeroPos) {

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Calculate high/low register split   */
	uint16_t highRegisterData = ( zeroPos & 0x3FC0 ) >> 6;
	uint16_t lowRegisterData  = ( zeroPos & 0x003F );

	/*   Write position data to registers   */
	status += AS5048A_WriteRegister(encoder, AS5048A_ZPHR, highRegisterData);
	status += AS5048A_WriteRegister(encoder, AS5048A_ZPLR, lowRegisterData);

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	return SPI_SUCCESS;

}

