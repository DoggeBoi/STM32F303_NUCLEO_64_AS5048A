# STM32F303_NUCLEO_64_AS5048A
Prototype Library for AS5048A Encoder with NSS Pulse mode and DMA. WIP!

STM32 IDE Config

Mode Full_duplex Master

Hardware NSS Signal Hardware NSS Output Signal

Frame Format:   Motorola

Data Size:      16 Bits

First Bit:      MSB

Prescaler:      8, Baud 4.5MBits/s

CPOL:           Low

CPHA:           1 Edge

CRC:            Disabled

NSS:            Output Hardware

USER CODE BEGIN SPI2_Init 2

hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;	//Enables pulse mode

