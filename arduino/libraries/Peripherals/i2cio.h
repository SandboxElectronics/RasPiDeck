#ifndef _H_I2CIO_
#define _H_I2CIO_

#define         RASPIDECK_I2CIO_REG_CFG	     (0X03)
#define		RASPIDECK_I2CIO_REG_IN	     (0X00)
#define		RASPIDECK_I2CIO_REG_OUT	     (0X01)
#define		RASPIDECK_I2CIO_REG_INV	     (0X02)
#define         RASPIDECK_I2CIO_ADDR          (0x25)



void RasPiDeckI2CIOSetPinMode(uint8_t pin, uint8_t io_mode);
void RasPiDeckI2CIOSetPinStatus(uint8_t pin, uint8_t pin_status);
uint8_t RasPiDeckI2CIOGetPinStatus(uint8_t pin);
void RasPiDeckI2CIOSetPortStatus(uint8_t port_status);
void RasPiDeckI2CIOSetPortMode(uint8_t port_mode);
uint8_t RasPiDeckI2CIOGetPortStatus(void);

#endif
