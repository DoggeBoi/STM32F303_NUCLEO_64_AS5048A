
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

	/*   Chip select default inactive high   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);

	/*	Encoder startup   */
	HAL_Delay(10);

	/*   Clear error flag register   */
	status += AS5048A_ClearErrorFlags(encoder);

	/*   Read Offset compensation bit as communications check  */
	uint16_t OCF;
	status += AS5048A_ReadRegister(encoder, AS5048A_AGCR, &OCF);
	status += ( ( OCF & ( 1 << 8 ) ) == 0 );

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
	uint16_t TxBuf[] = {AS5048A_NOP};
	uint16_t RxBuf[] = {AS5048A_NOP};

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Set command transmission package   */
	TxBuf[0]  = ( regAddr | ( 0x0001 << 14 ) );					// Sets Command Package address and read bit
	TxBuf[0] |= ( CalcParityEven( TxBuf[0] ) << 15 );			// Sets Command Package parity bit

	/*   Transmit command package   */
	status += AS5048A_Transmit(encoder, TxBuf);

	/*   Receive read package   */
	status += AS5048A_Receive(encoder, RxBuf);

	/*   SPI error flag check   */
	status += ( ( RxBuf[0] & ( 1 << 14 ) ) != 0 );

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	*data = RxBuf[0] & 0x3FFF;		// Save only 14 data bits

	return SPI_SUCCESS;

}


AS5048A_Error AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data) {

	/*   Initialise SPI buffers   */
	uint16_t TxBuf[] = {AS5048A_NOP};
	uint16_t RxBuf[] = {AS5048A_NOP};

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Set read transmission package   */
	TxBuf[0] = ( regAddr | CalcParityEven( regAddr ) << 15 );	// Sets Command Package parity bit

	/*   Transmit command package   */
	status += AS5048A_Transmit(encoder, TxBuf);

	/*   Set data transmission package   */
	TxBuf[0] = ( data | CalcParityEven( data ) << 15 );	// Sets Command Package parity bit

	/*   Transmit data package   */
	status += AS5048A_Transmit(encoder, TxBuf);

	/*   Receive new register read package   */
	status += AS5048A_Receive(encoder, RxBuf);

	/*   SPI error flag check   */
	status += ( ( RxBuf[0] & ( 1 << 14 ) ) != 0 );

	/*   SPI data return check   */
	status += ( ( RxBuf[0] & 0x3FFF ) != data );		// 14 data bits check

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	return SPI_SUCCESS;
}


AS5048A_Error AS5048A_ClearErrorFlags(AS5048A *encoder) {

	/*   Initialise dummy buffer   */
	uint16_t dumBuf[1] = {AS5048A_NOP};

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Read error register   */
	status += AS5048A_ReadRegister(encoder, AS5048A_CEFR, dumBuf);	// Error register cleared by reading, do nothing with old EF register data.

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
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


AS5048A_Error AS5048A_GetAngleRad(AS5048A *encoder, float *angle){

	/*   Initialise raw angle variable   */
	uint16_t angleRaw;

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Read angle register   */
	status += AS5048A_ReadRegister(encoder, AS5048A_ANGR, &angleRaw);

	/*   Convert angleRaw to angle (radians)   */
	*angle = angleRaw * 0.00038349519;

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	return SPI_SUCCESS;
}


AS5048A_Error AS5048A_Transmit(AS5048A *encoder, uint16_t *TxBuf){

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_RESET);
	status += HAL_SPI_Transmit(encoder->spiHandle, (uint8_t*)TxBuf, 1, AS5048A_MAX_DELAY);
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	return SPI_SUCCESS;
}


AS5048A_Error AS5048A_Receive(AS5048A *encoder, uint16_t *RxBuf){

	/*   Initialise status variable   */
	uint8_t	status = 0;

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_RESET);
	status += HAL_SPI_Receive(encoder->spiHandle, (uint8_t*)RxBuf, 1, AS5048A_MAX_DELAY);
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);

	/*   SPI communications check   */
	if ( status != 0 ) {
			return SPI_ERROR;
		}

	return SPI_SUCCESS;

}

