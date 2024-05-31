
#ifndef STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H
#define STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H

#include "stm32f3xx_hal.h"

/*	Register defines*/
#define	AS5048A_NOP				0x0000		//	Dummy Command
#define	AS5048A_CEFR			0x0001		//	Clear error flag, errors cleared by access
//#define	AS5048A_PCR			0x0003		//	Program control, can burn settings, no touchy!
#define	AS5048A_ZPHR			0x0016		//  Zero Position value MSB(s)
#define	AS5048A_ZPLR			0x0017		//	Zero Position value 6 LSB(s)
#define	AS5048A_AGCR			0x3FFD		//	Diagnostics + Automatic Gain Control
#define	AS5048A_MAGR			0x3FFE		//	Magnitude output value of the CORDIC
#define	AS5048A_ANGR			0x3FFF		//	Angle output value including zero position correction


/*   Max SPI timeout define*/
#define	AS5048A_MAX_DELAY		50


/*	SPI error defines   */
typedef enum {
    SPI_SUCCESS,           		// No error
	SPI_ERROR,					// General error
	SPI_INIT_FAIL,				// Initialisation fail
	SPI_CONNECTION_FAIL,		// Failed to reestablish connection
	SPI_PARITY_ERROR			// Parity bit is incorrect
} AS5048A_Error;


typedef struct {

	/*	SPI	 */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPort;
	uint16_t 		   csPin;

	/*   Conversion constants (raw to degrees)	*/
	float angleConversion;

	/*   Angle measured   */
	float angleDegrees;

} AS5048A;


/*   Initialisation   */
AS5048A_Error AS5048A_Init(AS5048A *encoder, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t);


/*   SPI transmit / receive and chip select control function   */
AS5048A_Error AS5048A_Transmit(AS5048A *encoder, uint16_t *TxBuf);
AS5048A_Error AS5048A_Receive(AS5048A *encoder, uint16_t *RxBuf);


/*   Lower level register functions   */
AS5048A_Error AS5048A_ReadRegister(AS5048A *encoder, uint16_t regAddr, uint16_t *data);
AS5048A_Error AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data);
AS5048A_Error AS5048A_ClearErrorFlags(AS5048A *encoder);


/*	Angle read function   */
AS5048A_Error AS5048A_ReadAngle(AS5048A *encoder, uint16_t *angle);


/*   Set home position   */
AS5048A_Error AS5048A_SetZeroPosition(AS5048A *encoder, uint16_t zeroPos);


/*   In case of error check connection   */
AS5048A_Error AS5048A_ConnectionCheck(AS5048A *encoder);


/*   SPI parity calculator   */
uint8_t CalcParityEven(uint16_t data);


/*   Check parity calculator   */
AS5048A_Error CheckParityEven(uint16_t data);


#endif /* STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H */
