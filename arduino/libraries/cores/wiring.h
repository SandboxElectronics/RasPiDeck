#ifndef WIRING_HEADER
#define WIRING_HEADER

#define SERIAL_IO_ADDR 0x20

#define IOBASE   0x20000000


#define GPIO_BASE (IOBASE + 0x200000)
#define BCM2835_SPI0_BASE (IOBASE + 0x204000)



#define GPFSEL0    *(gpio.addr + 0)
#define GPFSEL1    *(gpio.addr + 1)
#define GPFSEL2    *(gpio.addr + 2)
#define GPFSEL3    *(gpio.addr + 3)
#define GPFSEL4    *(gpio.addr + 4)
#define GPFSEL5    *(gpio.addr + 5)

// Reserved @ word offset 6
#define GPSET0    *(gpio.addr + 7)
#define GPSET1    *(gpio.addr + 8)
// Reserved @ word offset 9
#define GPCLR0    *(gpio.addr + 10)
#define GPCLR1    *(gpio.addr + 11)
// Reserved @ word offset 12
#define GPLEV0    *(gpio.addr + 13)
#define GPLEV1    *(gpio.addr + 14)





#define PAGESIZE 4096
#define BLOCK_SIZE 4096


/// Defines for SPI
/// GPIO register offsets from BCM2835_SPI0_BASE.
/// Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
#define BCM2835_SPI0_CS                      0x0000 ///< SPI Master Control and Status
#define BCM2835_SPI0_FIFO                    0x0004 ///< SPI Master TX and RX FIFOs
#define BCM2835_SPI0_CLK                     0x0008 ///< SPI Master Clock Divider
#define BCM2835_SPI0_DLEN                    0x000c ///< SPI Master Data Length
#define BCM2835_SPI0_LTOH                    0x0010 ///< SPI LOSSI mode TOH
#define BCM2835_SPI0_DC                      0x0014 ///< SPI DMA DREQ Controls

// Register masks for SPI0_CS
#define BCM2835_SPI0_CS_LEN_LONG             0x02000000 ///< Enable Long data word in Lossi mode if DMA_LEN is set
#define BCM2835_SPI0_CS_DMA_LEN              0x01000000 ///< Enable DMA mode in Lossi mode
#define BCM2835_SPI0_CS_CSPOL2               0x00800000 ///< Chip Select 2 Polarity
#define BCM2835_SPI0_CS_CSPOL1               0x00400000 ///< Chip Select 1 Polarity
#define BCM2835_SPI0_CS_CSPOL0               0x00200000 ///< Chip Select 0 Polarity
#define BCM2835_SPI0_CS_RXF                  0x00100000 ///< RXF - RX FIFO Full
#define BCM2835_SPI0_CS_RXR                  0x00080000 ///< RXR RX FIFO needs Reading ( full)
#define BCM2835_SPI0_CS_TXD                  0x00040000 ///< TXD TX FIFO can accept Data
#define BCM2835_SPI0_CS_RXD                  0x00020000 ///< RXD RX FIFO contains Data
#define BCM2835_SPI0_CS_DONE                 0x00010000 ///< Done transfer Done
#define BCM2835_SPI0_CS_TE_EN                0x00008000 ///< Unused
#define BCM2835_SPI0_CS_LMONO                0x00004000 ///< Unused
#define BCM2835_SPI0_CS_LEN                  0x00002000 ///< LEN LoSSI enable
#define BCM2835_SPI0_CS_REN                  0x00001000 ///< REN Read Enable
#define BCM2835_SPI0_CS_ADCS                 0x00000800 ///< ADCS Automatically Deassert Chip Select
#define BCM2835_SPI0_CS_INTR                 0x00000400 ///< INTR Interrupt on RXR
#define BCM2835_SPI0_CS_INTD                 0x00000200 ///< INTD Interrupt on Done
#define BCM2835_SPI0_CS_DMAEN                0x00000100 ///< DMAEN DMA Enable
#define BCM2835_SPI0_CS_TA                   0x00000080 ///< Transfer Active
#define BCM2835_SPI0_CS_CSPOL                0x00000040 ///< Chip Select Polarity
#define BCM2835_SPI0_CS_CLEAR                0x00000030 ///< Clear FIFO Clear RX and TX
#define BCM2835_SPI0_CS_CLEAR_RX             0x00000020 ///< Clear FIFO Clear RX
#define BCM2835_SPI0_CS_CLEAR_TX             0x00000010 ///< Clear FIFO Clear TX
#define BCM2835_SPI0_CS_CPOL                 0x00000008 ///< Clock Polarity
#define BCM2835_SPI0_CS_CPHA                 0x00000004 ///< Clock Phase
#define BCM2835_SPI0_CS_CS                   0x00000003 ///< Chip Select

#define BCM2835_GPFSEL0                      0x0000 ///< GPIO Function Select 0

#define BCM2835_GPEDS0                       0x0040 ///< GPIO Pin Event Detect Status 0
#define BCM2835_GPREN0                       0x004c ///< GPIO Pin Rising Edge Detect Enable 0
#define BCM2835_GPFEN0                       0x0048 ///< GPIO Pin Falling Edge Detect Enable 0
#define BCM2835_GPHEN0                       0x0064 ///< GPIO Pin High Detect Enable 0
#define BCM2835_GPLEN0                       0x0070 ///< GPIO Pin Low Detect Enable 0

#define CS		10
#define MOSI	11
#define MISO	12
#define SCK		13


static int REV = 0;

/// \brief bcm2835SPIBitOrder
/// Specifies the SPI data bit ordering
typedef enum
{
    LSBFIRST = 0,  ///< LSB First
    MSBFIRST = 1   ///< MSB First
}bcm2835SPIBitOrder;

/// \brief bcm2835SPIMode
/// Specify the SPI data mode
typedef enum
{
    SPI_MODE0 = 0,  ///< CPOL = 0, CPHA = 0
    SPI_MODE1 = 1,  ///< CPOL = 0, CPHA = 1
    SPI_MODE2 = 2,  ///< CPOL = 1, CPHA = 0
    SPI_MODE3 = 3,  ///< CPOL = 1, CPHA = 1
}bcm2835SPIMode;

/// \brief bcm2835SPIChipSelect
/// Specify the SPI chip select pin(s)
typedef enum
{
    SPI_CS0 = 0,     ///< Chip Select 0
    SPI_CS1 = 1,     ///< Chip Select 1
    SPI_CS2 = 2,     ///< Chip Select 2 (ie pins CS1 and CS2 are asserted)
    SPI_CS_NONE = 3, ///< No CS, control it yourself
} bcm2835SPIChipSelect;

/// \brief bcm2835SPIClockDivider
/// Specifies the divider used to generate the SPI clock from the system clock.
/// Figures below give the divider, clock period and clock frequency.
typedef enum
{
    SPI_CLOCK_DIV65536 = 0,       ///< 65536 = 256us = 4kHz
    SPI_CLOCK_DIV32768 = 32768,   ///< 32768 = 126us = 8kHz
    SPI_CLOCK_DIV16384 = 16384,   ///< 16384 = 64us = 15.625kHz
    SPI_CLOCK_DIV8192  = 8192,    ///< 8192 = 32us = 31.25kHz
    SPI_CLOCK_DIV4096  = 4096,    ///< 4096 = 16us = 62.5kHz
    SPI_CLOCK_DIV2048  = 2048,    ///< 2048 = 8us = 125kHz
    SPI_CLOCK_DIV1024  = 1024,    ///< 1024 = 4us = 250kHz
    SPI_CLOCK_DIV512   = 512,     ///< 512 = 2us = 500kHz
    SPI_CLOCK_DIV256   = 256,     ///< 256 = 1us = 1MHz
    SPI_CLOCK_DIV128   = 128,     ///< 128 = 500ns = = 2MHz
    SPI_CLOCK_DIV64    = 64,      ///< 64 = 250ns = 4MHz
    SPI_CLOCK_DIV32    = 32,      ///< 32 = 125ns = 8MHz
    SPI_CLOCK_DIV16    = 16,      ///< 16 = 50ns = 20MHz
    SPI_CLOCK_DIV8     = 8,       ///< 8 = 25ns = 40MHz
    SPI_CLOCK_DIV4     = 4,       ///< 4 = 12.5ns 80MHz
    SPI_CLOCK_DIV2     = 2,       ///< 2 = 6.25ns = 160MHz
    SPI_CLOCK_DIV1     = 1,       ///< 0 = 256us = 4kHz
} bcm2835SPIClockDivider;

typedef enum
{
    BCM2835_GPIO_FSEL_INPT  = 0b000,   ///< Input
    BCM2835_GPIO_FSEL_OUTP  = 0b001,   ///< Output
    BCM2835_GPIO_FSEL_ALT0  = 0b100,   ///< Alternate function 0
    BCM2835_GPIO_FSEL_ALT1  = 0b101,   ///< Alternate function 1
    BCM2835_GPIO_FSEL_ALT2  = 0b110,   ///< Alternate function 2
    BCM2835_GPIO_FSEL_ALT3  = 0b111,   ///< Alternate function 3
    BCM2835_GPIO_FSEL_ALT4  = 0b011,   ///< Alternate function 4
    BCM2835_GPIO_FSEL_ALT5  = 0b010,   ///< Alternate function 5
    BCM2835_GPIO_FSEL_MASK  = 0b111    ///< Function select bits mask
} bcm2835FunctionSelect;


namespace unistd {
	//All functions of unistd.h must be called like this: unistd::the_function()
    #include <unistd.h>
}


enum Representation{
	BIN,
	OCT,
	DEC,
	HEX,
	BYTE
};

typedef enum {
	INPUT,
	OUTPUT
}Pinmode;

typedef enum {
	LOW = 0,
	HIGH = 1,
	RISING = 2,
	FALLING = 3,
	BOTH = 4
}Digivalue;

typedef bool boolean;
typedef unsigned char byte;

struct bcm2835_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};

struct ThreadArg{
    void (*func)();
    int pin;
};



/* SerialPi Class
 * Class that provides the functionality of arduino Serial library
 */
class SerialPi {

private:
	int sd,status;
	const char *serialPort;
	unsigned char c;
	struct termios options;
	int speed;
	long timeOut;
	timespec time1, time2;
	timespec timeDiff(timespec start, timespec end);
	char * int2bin(int i);
	char * int2hex(int i);
	char * int2oct(int i);

public:

	SerialPi();
	void begin(int serialSpeed);
	int available();
	char read();
	int readBytes(char message[], int size);
	int readBytesUntil(char character,char buffer[],int length);
	bool find(const char *target);
	bool findUntil(const char *target, const char *terminal);
	long parseInt();
	float parseFloat();
	char peek();
	void print(const char *message);
	void print(char message);
	void print(unsigned char i,Representation rep);
	void print(float f, int precission);
	void println(const char *message);
	void println(char message);
	void println(int i, Representation rep);
	void println(float f, int precission);
	int write(unsigned char message);
	int write(const char *message);
	int write (char *message, int size);
	void flush();
	void setTimeout(long millis);
	void end();
};

class SPIPi{
	public:
		SPIPi();
  		void begin();
    	void end();
    	void setBitOrder(uint8_t order);
 		void setClockDivider(uint16_t divider);
		void setDataMode(uint8_t mode);
 		void chipSelect(uint8_t cs);
		void setChipSelectPolarity(uint8_t cs, uint8_t active);
 		uint8_t transfer(uint8_t value);
 		void transfernb(char* tbuf, char* rbuf, uint32_t len);
};

/* Some useful arduino functions */
void pinMode(int pin, Pinmode mode);
//void pinEn5v(int pin);
//void pinDis5v(int pin);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead (int pin);
void delay(long millis);
void delayMicroseconds(long micros);
uint8_t shiftIn  (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order);
void shiftOut (uint8_t dPin, uint8_t cPin, bcm2835SPIBitOrder order, uint8_t val);
void attachInterrupt(int p,void (*f)(), Digivalue m);
void detachInterrupt(int p);
void setup();
void loop();
long millis();

/* Helper functions */
int getBoardRev();
uint32_t *mapmem(const char *msg, size_t size, int fd, off_t off);
void setBoardRev(int rev);
int raspberryPinNumber(int arduinoPin);
pthread_t *getThreadIdFromPin(int pin);
uint32_t bcm2835_peri_read(volatile uint32_t* paddr);
uint32_t bcm2835_peri_read_nb(volatile uint32_t* paddr);
void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value);
void bcm2835_peri_write_nb(volatile uint32_t* paddr, uint32_t value);
void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask);
void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode);
void * threadFunction(void *args);

extern SerialPi Serial;
extern SPIPi SPI;

#endif // WIRING_HEADER
