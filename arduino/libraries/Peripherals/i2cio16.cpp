//#define PIN_UNDER_TEST (2)

//#include "stdio.h"
#include "../includes.h"


/*
void setup()

{
    Wire.begin();

    RasPiDeckI2CIO16SetPortStatus(0,0xff);
    RasPiDeckI2CIO16SetPortStatus(1,0xff);

      
    RasPiDeckI2CIO16SetPortMode(0,0x00);
    RasPiDeckI2CIO16SetPortMode(1,0x00);

    RasPiDeckI2CIO16SetPinStatus(14, 0);

    pinMode(14,OUTPUT);
    digitalWrite(14,0);
}

void loop()
{

uint8_t i;

i++;


if (i&0x01) {
   RasPiDeckI2CIO16SetPinStatus(7, 1 );
} else {
   RasPiDeckI2CIO16SetPinStatus(7, 0);
}

}
*/
uint8_t  RasPiDeckI2CIO16ReadRegister(uint8_t command, uint8_t *i2c_buf)
{
    uint8_t  value;
    Wire.beginTransmission( RASPIDECK_I2CIO16_ADDR );
    Wire.write(command);
    Wire.endTransmission();
   // printf ("Available data: %d\n", Wire.requestFrom(RASPIDECK_I2CIO16_ADDR,2));

    i2c_buf[0] = (uint8_t)Wire.read();
    i2c_buf[1] = (uint8_t)Wire.read();

   // printf("READING from  device 0x%02x,register 0x%02x, value 0x%02x:0x%02x\n",RASPIDECK_I2CIO16_ADDR, command,i2c_buf[0],i2c_buf[1]);
    return value;
}

void RasPiDeckI2CIO16WriteRegister(uint8_t command, uint8_t *i2c_buf)
{
    
    Wire.beginTransmission(RASPIDECK_I2CIO16_ADDR);
    Wire.write(command);
    Wire.write(i2c_buf[0]);
    Wire.write(i2c_buf[1]);

    Wire.endTransmission();
    //printf("WRITING to    device 0x%02x,register 0x%02x, value 0x%02x:0x%02x\n",RASPIDECK_I2CIO16_ADDR, command,i2c_buf[0],i2c_buf[1]);

}

void RasPiDeckI2CIO16SetPinMode(uint8_t pin, uint8_t io_mode)
{
    uint8_t port_reg=RASPIDECK_I2CIO16_REG_CFG0;
    uint8_t buf[2];

    if (pin > 7) {
	port_reg = RASPIDECK_I2CIO16_REG_CFG1;
    }

    RasPiDeckI2CIO16ReadRegister(port_reg,buf);

    if (io_mode == INPUT) {
	buf[0] |= (0x01<<(pin%8));
    } else {
        buf[0] &= (uint8_t)~(0x01<<(pin%8));
    }

    RasPiDeckI2CIO16WriteRegister(port_reg, buf);

    return;

}

void RasPiDeckI2CIO16SetPinStatus(uint8_t pin, uint8_t pin_status)
{
    uint8_t buf[2];

    if (pin > 7) {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN1,buf);
    } else {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN0,buf);
    } 


    if (pin_status != 0) {
        buf[0] |= (0x01<<(pin%8));
    } else {
        buf[0] &= (uint8_t)~(0x01<<(pin%8));
    }

    if (pin > 7) {
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_OUT1, buf);
    } else {
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_OUT0, buf);
    }

    return;
}

uint8_t  RasPiDeckI2CIO16GetPinStatus(uint8_t pin)
{
    uint8_t buf[2];
    uint8_t port_reg=RASPIDECK_I2CIO16_REG_IN0; 


    if (pin > 7) {
        port_reg = RASPIDECK_I2CIO16_REG_IN1;
    }

    RasPiDeckI2CIO16ReadRegister(port_reg,buf);


    if (buf[0]&(0x01<<(pin%8)) != 0) {
        return 1;
    } 
    return 0;
}



void RasPiDeckI2CIO16SetPortMode(uint8_t port_number,  uint8_t io_mode)  //0:output 1:input
{
    uint8_t buf[2];

    if ( port_number == 0) {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_CFG0, buf);
        buf[0] = io_mode;   
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_CFG0,buf);
    } else {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_CFG1, buf);
        buf[0] = io_mode;
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_CFG1,buf);

    }
    return;

}

void RasPiDeckI2CIO16SetPortStatus(uint8_t port_number, uint8_t port_status)
{

    uint8_t buf[2];

    if ( port_number == 0) {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN0, buf);
        buf[0] = port_status;
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_OUT0,buf);
    } else {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN1, buf);
        buf[0] = port_status;
        RasPiDeckI2CIO16WriteRegister(RASPIDECK_I2CIO16_REG_OUT1,buf);

    }

    return;

}


uint8_t RasPiDeckI2CIO16GetPortStatus(uint8_t port_number)
{
    uint8_t buf[2];
    if ( port_number == 0) {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN0, buf);
    } else {
        RasPiDeckI2CIO16ReadRegister(RASPIDECK_I2CIO16_REG_IN1, buf);
    }

    return buf[0];

}

