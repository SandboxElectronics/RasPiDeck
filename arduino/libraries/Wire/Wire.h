#ifndef WIRE_HEADER
#define WIRE_HEADER

#define BCM2835_BSC1_BASE (0x20804000)

// Defines for I2C
// GPIO register offsets from BCM2835_BSC*_BASE.
// Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
#define BCM2835_BSC_C 							0x0000 ///< BSC Master Control
#define BCM2835_BSC_S 							0x0004 ///< BSC Master Status
#define BCM2835_BSC_DLEN						0x0008 ///< BSC Master Data Length
#define BCM2835_BSC_A 							0x000c ///< BSC Master Slave Address
#define BCM2835_BSC_FIFO						0x0010 ///< BSC Master Data FIFO
#define BCM2835_BSC_DIV							0x0014 ///< BSC Master Clock Divider
#define BCM2835_BSC_DEL							0x0018 ///< BSC Master Data Delay
#define BCM2835_BSC_CLKT						0x001c ///< BSC Master Clock Stretch Timeout

// Register masks for BSC_C
#define BCM2835_BSC_C_I2CEN 					0x00008000 ///< I2C Enable, 0 = disabled, 1 = enabled
#define BCM2835_BSC_C_INTR 						0x00000400 ///< Interrupt on RX
#define BCM2835_BSC_C_INTT 						0x00000200 ///< Interrupt on TX
#define BCM2835_BSC_C_INTD 						0x00000100 ///< Interrupt on DONE
#define BCM2835_BSC_C_ST 						0x00000080 ///< Start transfer, 1 = Start a new transfer
#define BCM2835_BSC_C_CLEAR_1 					0x00000020 ///< Clear FIFO Clear
#define BCM2835_BSC_C_CLEAR_2 					0x00000010 ///< Clear FIFO Clear
#define BCM2835_BSC_C_READ 						0x00000001 ///<	Read transfer

// Register masks for BSC_S
#define BCM2835_BSC_S_CLKT 						0x00000200 ///< Clock stretch timeout
#define BCM2835_BSC_S_ERR 						0x00000100 ///< ACK error
#define BCM2835_BSC_S_RXF 						0x00000080 ///< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_TXE 						0x00000040 ///< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_RXD 						0x00000020 ///< RXD FIFO contains data
#define BCM2835_BSC_S_TXD 						0x00000010 ///< TXD FIFO can accept data
#define BCM2835_BSC_S_RXR 						0x00000008 ///< RXR FIFO needs reading (full)
#define BCM2835_BSC_S_TXW 						0x00000004 ///< TXW FIFO needs writing (full)
#define BCM2835_BSC_S_DONE 						0x00000002 ///< Transfer DONE
#define BCM2835_BSC_S_TA 						0x00000001 ///< Transfer Active

#define BCM2835_BSC_FIFO_SIZE   				16 ///< BSC FIFO size
#define BCM2835_CORE_CLK_HZ				250000000	///< 250 MHz

#define BSC_C_I2CEN   BIT_15
#define BSC_C_INTR    BIT_10
#define BSC_C_INTT    BIT_9
#define BSC_C_INTD    BIT_8
#define BSC_C_ST      BIT_7
#define BSC_C_CLEAR   BIT_4
#define BSC_C_READ    BIT_0

#define START_READ    BSC_C_I2CEN | BSC_C_ST | BSC_C_CLEAR | BSC_C_READ
#define START_WRITE   BSC_C_I2CEN | BSC_C_ST

#define BSC_S_CLKT    BIT_9
#define BSC_S_ERR     BIT_8
#define BSC_S_RXF     BIT_7
#define BSC_S_TXE     BIT_6
#define BSC_S_RXD     BIT_5
#define BSC_S_TXD     BIT_4
#define BSC_S_RXR     BIT_3
#define BSC_S_TXW     BIT_2
#define BSC_S_DONE    BIT_1
#define BSC_S_TA      BIT_0

#define CLEAR_STATUS  BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE

#define BUFFER_LENGTH 32

/// \brief bcm2835I2CClockDivider
/// Specifies the divider used to generate the I2C clock from the system clock.
/// Clock divided is based on nominal base clock rate of 250MHz
typedef enum
{
    BCM2835_I2C_CLOCK_DIVIDER_2500   = 2500,      ///< 2500 = 10us = 100 kHz
    BCM2835_I2C_CLOCK_DIVIDER_626    = 626,       ///< 622 = 2.504us = 399.3610 kHz
    BCM2835_I2C_CLOCK_DIVIDER_150    = 150,       ///< 150 = 60ns = 1.666 MHz (default at reset)
    BCM2835_I2C_CLOCK_DIVIDER_148    = 148,       ///< 148 = 59ns = 1.689 MHz
} bcm2835I2CClockDivider;

/// \brief bcm2835I2CReasonCodes
/// Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
typedef enum
{
    BCM2835_I2C_REASON_OK   		 = 0x00,      ///< Success
    BCM2835_I2C_REASON_ERROR_NACK    = 0x01,      ///< Received a NACK
    BCM2835_I2C_REASON_ERROR_CLKT    = 0x02,      ///< Received Clock Stretch Timeout
    BCM2835_I2C_REASON_ERROR_DATA    = 0x04,      ///< Not all data is sent / received
} bcm2835I2CReasonCodes;

typedef enum
{
	RPI_V2_GPIO_P1_03     =  2,  ///< Version 2, Pin P1-03
	RPI_V2_GPIO_P1_05     =  3,  ///< Version 2, Pin P1-05
}RPiGPIOPin;

/* WirePi Class
 * Class that provides the functionality of arduino Wire library
 */
class WirePi{
	private:
	    volatile uint32_t* reg_dlen;
	    volatile uint32_t* reg_fifo;
	    volatile uint32_t* reg_stat;
	    volatile uint32_t* reg_ctrl;
	    volatile uint32_t* reg_addr;

        uint8_t rxBuffer[BUFFER_LENGTH];
        uint8_t rxBufferIndex;
        uint8_t rxBufferLength;

        uint8_t txAddress;
        uint8_t txBuffer[BUFFER_LENGTH];
        uint8_t txBufferIndex;
        uint8_t txBufferLength;

		int  memfd;
		int  map_peripheral   (struct bcm2835_peripheral *p);
		void unmap_peripheral (struct bcm2835_peripheral *p);
	public:
		WirePi();
		void    begin();
		void    beginTransmission (uint8_t address);
		void    beginTransmission (int address);
		size_t  write             (uint8_t data);
		size_t  write             (const uint8_t *data, size_t quantity);
		uint8_t endTransmission   (uint8_t sendStop);
		uint8_t endTransmission   (void);
		uint8_t requestFrom       (uint8_t address, uint8_t quantity);
		int     available         (void);
		int     read              (void);
		int     peek              (void);
};

extern WirePi Wire;

#endif // WIRE_HEADER
