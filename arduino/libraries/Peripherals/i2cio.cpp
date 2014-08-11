
#include "../includes.h"


uint8_t  RasPiDeckI2CIOReadRegister(uint8_t command)
{
    uint8_t  value;    

    Wire.beginTransmission( RASPIDECK_I2CIO_ADDR );
    Wire.write(command);
    Wire.endTransmission();
    Wire.requestFrom(RASPIDECK_I2CIO_ADDR,1);
    value = Wire.read();
    
    return value;
}

void RasPiDeckI2CIOWriteRegister(uint8_t command, uint8_t value)
{

    Wire.beginTransmission(RASPIDECK_I2CIO_ADDR);
    Wire.write(command);
    Wire.write(value);
    Wire.endTransmission();

}

void RasPiDeckI2CIOSetPinMode(uint8_t pin, uint8_t io_mode)
{
    uint8_t reg_val;

    reg_val =  RasPiDeckI2CIOReadRegister(RASPIDECK_I2CIO_REG_CFG);

    if (io_mode == INPUT) {
	reg_val |= (0x01<<pin);
    } else {
        reg_val &= (uint8_t)~(0x01<<pin);
    }

    RasPiDeckI2CIOWriteRegister(RASPIDECK_I2CIO_REG_CFG, reg_val);

    return;

}

void RasPiDeckI2CIOSetPinStatus(uint8_t pin, uint8_t pin_status)
{
    uint8_t reg_val;

    reg_val =  RasPiDeckI2CIOReadRegister(RASPIDECK_I2CIO_REG_OUT);
    if (pin_status != 0) {
        reg_val |= (0x01<<pin);
    } else {
        reg_val &= (uint8_t)~(0x01<<pin);
    }

    RasPiDeckI2CIOWriteRegister(RASPIDECK_I2CIO_REG_OUT, reg_val);
    return;
}

uint8_t  RasPiDeckI2CIOGetPinStatus(uint8_t pin)
{
    uint8_t reg_val;

    reg_val =  RasPiDeckI2CIOReadRegister(RASPIDECK_I2CIO_REG_IN);

    if (reg_val&(0x01<<pin) != 0) {
        return 1;
    } 
    return 0;
}



void RasPiDeckI2CIOSetPortMode(uint8_t io_mode)  //0:output 1:input
{
    RasPiDeckI2CIOWriteRegister(RASPIDECK_I2CIO_REG_CFG, io_mode);
    return;
}

void RasPiDeckI2CIOSetPortStatus(uint8_t port_status)
{
    RasPiDeckI2CIOWriteRegister(RASPIDECK_I2CIO_REG_OUT, port_status);
    return;
}


uint8_t RasPiDeckI2CIOGetPortStatus(void)
{
    return RasPiDeckI2CIOReadRegister(RASPIDECK_I2CIO_REG_IN);
}

