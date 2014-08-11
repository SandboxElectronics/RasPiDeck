#ifndef _H_PERIPHERALS_
#define _H_PERIPHERALS_



void RasPiDeckPeripheralsReset(void);
void RasPiDeckSPI3v3(void);
void RasPiDeckSPI5v(void);
void RasPiDeckI2C3v3(void);
void RasPiDeckI2C5v(void);
void RasPiDeckSPIEnable(void);
void RasPiDeckSPIDisable(void);
void RasPiDeckI2CEnable(void);
void RasPiDeckI2CDisable(void);
void RasPiDeckReference5v(void);
void RasPiDeckReferenceExt(void);
void RasPiDeckGPIO5v(int pin);
void RasPiDeckGPIO3v3(int pin);

#endif

