#ifndef _H_I2CIO16_
#define _H_I2CIO16_

#define		    RASPIDECK_I2CIO16_REG_IN0       (0x00)
#define         RASPIDECK_I2CIO16_REG_IN1       (0X01)
#define		    RASPIDECK_I2CIO16_REG_OUT0      (0X02)
#define         RASPIDECK_I2CIO16_REG_OUT1      (0X03)
#define		    RASPIDECK_I2CIO16_REG_INV0      (0X04)
#define         RASPIDECK_I2CIO16_REG_INV1      (0X05)
#define         RASPIDECK_I2CIO16_REG_CFG0      (0X06)
#define         RASPIDECK_I2CIO16_REG_CFG1      (0X07)

#define         RASPIDECK_I2CIO16_ADDR          (0x20)



void RasPiDeckI2CIO16SetPinMode(uint8_t pin, uint8_t io_mode);
void RasPiDeckI2CIO16SetPinStatus(uint8_t pin, uint8_t pin_status);
uint8_t RasPiDeckI216CIOGetPinStatus(uint8_t pin);
void RasPiDeckI2CIO16SetPortStatus(uint8_t port_number, uint8_t port_status);
void RasPiDeckI2CIO16SetPortMode(uint8_t port_number,uint8_t port_mode);
uint8_t RasPiDeckI2CIO16GetPortStatus(uint8_t port_number);
uint8_t  RasPiDeckI2CIO16ReadRegister(uint8_t command, uint8_t *i2c_buf);
void RasPiDeckI2CIO16WriteRegister(uint8_t command, uint8_t *i2c_buf);

#endif
