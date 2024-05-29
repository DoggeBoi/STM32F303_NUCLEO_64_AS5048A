# STM32F303_NUCLEO_64_AS5048A
Prototype driver for AS5048A Encoder. WIP!
For future use in a custom servo motor.
Currently not planning the use of SPI interrupts, DMA, or NSS as it makes a neglige impact on read time and adds a lot of complexity. 
Pulse mode NSS dosent work with CPHA 1 / "2 Edge", which is used by the AS5048A.
