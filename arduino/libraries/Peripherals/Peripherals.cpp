
#include "../includes.h"



void RasPiDeckPeripheralsReset(void)
{
    digitalWrite(PL, 0);
    pinMode(PL,OUTPUT);

    Wire.begin();
    RasPiDeckI2CIOSetPortStatus(0xff);//Set the output register of PCA9554 to its POR status
    RasPiDeckI2CIOSetPortMode(0x3F);
    RasPiDeckI2CIO16SetPortStatus(0,0xff);//Set the port 0 output register of PCA9555 to its POR status
    RasPiDeckI2CIO16SetPortStatus(1,0xff);//Set the port 1 output register of PCA9555 to its POR status
    RasPiDeckI2CIO16SetPortMode(0,0); //Set port 0 of PCA9555 to output
    RasPiDeckI2CIO16SetPortMode(1,0); //Set port 1 of PCA9555 to output
    
    
}

void RasPiDeckSPI3v3(void)
{
    RasPiDeckI2CIO16SetPinStatus(15, 1);
}

void RasPiDeckSPI5v(void)
{
    RasPiDeckI2CIO16SetPinStatus(15, 0);
}

void RasPiDeckI2C3v3(void)
{
    RasPiDeckI2CIO16SetPinStatus(14, 1);
}

void RasPiDeckI2C5v(void)
{
    RasPiDeckI2CIO16SetPinStatus(14, 0);
}

void RasPiDeckSPIEnable(void)
{
    digitalWrite(PL,1);
}

void RasPiDeckSPIDisable(void)
{
    digitalWrite(PL,0);
}

void RasPiDeckI2CEnable(void)
{
    RasPiDeckI2CIOSetPinStatus(6,1);
}

void RasPiDeckI2CDisable(void)
{
    RasPiDeckI2CIOSetPinStatus(6,0);
}

void RasPiDeckReference5v(void)
{
    RasPiDeckI2CIOSetPinStatus(7,0);
}

void RasPiDeckReferenceExt(void)
{
    RasPiDeckI2CIOSetPinStatus(7,1);
}

void RasPiDeckGPIO5v(int pin)
{
  if (pin<=7) {
    RasPiDeckI2CIO16SetPinStatus(7-pin,0); //write 0 to enable 5V output
  } else {
    RasPiDeckI2CIO16SetPinStatus(pin,0); 
  }
}

void RasPiDeckGPIO3v3(int pin)
{
  if (pin<=7) {
    RasPiDeckI2CIO16SetPinStatus(7-pin,1); //write 1 to disable 5V output
  } else {
    RasPiDeckI2CIO16SetPinStatus(pin, 1);
  }
}

