#ifndef INC_DRIVERS_AS5048A_H
#define INC_DRIVERS_AS5048A_H

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
    SPI_ERROR_INIT,             // Initialization error
	SPI_ERROR,					// General error
} AS5048A_Error;


typedef struct {

	/*	SPI	 */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPort;
	uint16_t 		   csPin;

	/*  DMA  */
	uint16_t TxData;
	uint16_t RxData;

	/*   Conversion constants (raw to degrees)	*/
	float angleConversion;

	/*   Angle measured   */
	float angleDegrees;

	/*   Zero Position   */
	uint16_t zeroPos;

} AS5048A;


/*   Initialisation   */
AS5048A_Error AS5048A_Init(AS5048A *encoder, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t, uint16_t zeroPos);


/*   Lower level register functions   */
AS5048A_Error AS5048A_ReadRegister(AS5048A *encoder, uint16_t regAddr, uint16_t *data);
AS5048A_Error AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data);
AS5048A_Error AS5048A_ClearErrorFlags(AS5048A *encoder);

/*   Set home position   */
AS5048A_Error AS5048A_SetZeroPosition(AS5048A *encoder, uint16_t zeroPos);

/*   DMA SPI angle   */
uint8_t AS5048A_ReadAngleDMA(AS5048A *encoder);
void AS5048A_ReadAngleDMA_Complete(AS5048A *encoder);

/*   Error Clear and report   */
void AS5048A_ErrorHandeler(uint8_t errorCode);

/*   SPI parity calculator   */
uint8_t CalcParityEven(uint16_t data);

#endif /* INC_DRIVERS_AS5048A_H */
