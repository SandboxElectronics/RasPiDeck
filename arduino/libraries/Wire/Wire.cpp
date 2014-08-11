#include "../includes.h"

volatile uint32_t *bcm2835_bsc1;

extern timeval start_program;
extern struct bcm2835_peripheral gpio;

/******************
 * Public methods *
 ******************/

//Constructor
WirePi::WirePi(){
	if(map_peripheral(&gpio) == -1) {
		printf("Failed to map the physical GPIO registers into the virtual memory space.\n");
	}

	memfd = -1;

	// Open the master /dev/memory device
    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
        fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n",
		strerror(errno)) ;
        exit(1);
    }

	bcm2835_bsc1 = mapmem("bsc1", BLOCK_SIZE, memfd, BCM2835_BSC1_BASE);
    if (bcm2835_bsc1 == MAP_FAILED) {
        exit(1);
    }

    // start timer
    gettimeofday(&start_program, NULL);

    reg_dlen = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    reg_fifo = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    reg_stat = bcm2835_bsc1 + BCM2835_BSC_S/4;
    reg_ctrl = bcm2835_bsc1 + BCM2835_BSC_C/4;
    reg_addr = bcm2835_bsc1 + BCM2835_BSC_A/4;
}

//Initiate the Wire library and join the I2C bus.
void WirePi::begin() {
    rxBufferIndex  = 0;
    rxBufferLength = 0;

    txBufferIndex  = 0;
    txBufferLength = 0;

    // Set the I2C/BSC1 pins to the Alt 0 function to enable I2C access on them
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_03, BCM2835_GPIO_FSEL_ALT0); // SDA
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_05, BCM2835_GPIO_FSEL_ALT0); // SCL

    // Enable I2C Operation
    bcm2835_peri_write_nb(reg_ctrl, BCM2835_BSC_C_I2CEN);
}

//Begin a transmission to the I2C slave device with the given address
void WirePi::beginTransmission (uint8_t address) {
    // set address of targeted slave
    txAddress = address;
    // reset tx buffer iterator vars
    txBufferIndex  = 0;
    txBufferLength = 0;
}


void WirePi::beginTransmission (int address) {
    beginTransmission((uint8_t)address);
}


uint8_t WirePi::endTransmission (uint8_t sendStop) {
    // Wait until I2C is ready, become master receiver
    while ((bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_TA));

	// Set Slave Address
	bcm2835_peri_write_nb(reg_addr, txAddress);
    // Clear FIFO
    bcm2835_peri_set_bits(reg_ctrl, BCM2835_BSC_C_CLEAR_1, BCM2835_BSC_C_CLEAR_1);
    // Clear Status
	bcm2835_peri_write_nb(reg_stat, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    bcm2835_peri_write_nb(reg_dlen, txBufferLength);

    // pre populate FIFO with max buffer
    txBufferIndex = 0;
    while (txBufferIndex != txBufferLength) {
        if (bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_TXD) {
            bcm2835_peri_write_nb(reg_fifo, txBuffer[txBufferIndex]);
            txBufferIndex++;

            if (txBufferIndex < BCM2835_BSC_FIFO_SIZE) {
                if (txBufferIndex == txBufferLength) {
                    // Enable I2C opertation and start transfer
                    bcm2835_peri_write_nb(reg_ctrl, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
                }
            } else if (txBufferIndex == BCM2835_BSC_FIFO_SIZE) {
                // Enable I2C opertation and start transfer
                bcm2835_peri_write_nb(reg_ctrl, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
            }
        }
    }

    // Transfer is over when BCM2835_BSC_S_DONE
    while(!(bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_DONE));

    // Received an Error
    if (bcm2835_peri_read(reg_stat) & BCM2835_BSC_S_ERR) {
		return 4;
    }

    // Received Clock Stretch Timeout
    else if (bcm2835_peri_read(reg_stat) & BCM2835_BSC_S_CLKT) {
		return 4;
    }

    // reset tx buffer iterator vars
    txBufferIndex  = 0;
    txBufferLength = 0;
    return 0;
}


uint8_t WirePi::endTransmission (void) {
    return endTransmission(true);
}


size_t WirePi::write (uint8_t data) {
    // don't bother if buffer is full
    if (txBufferLength >= BUFFER_LENGTH) {
        return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;

    return 1;
}


size_t WirePi::write (const uint8_t *data, size_t quantity) {
    for(size_t i = 0; i < quantity; ++i){
        write(data[i]);
    }

    return quantity;
}


uint8_t WirePi::requestFrom (uint8_t address, uint8_t quantity){
    if (quantity > BUFFER_LENGTH) {
        quantity = BUFFER_LENGTH;
    }

    // Wait until I2C is ready, become master receiver
    while ((bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_TA)){/* printf("DEBUG INFO:I2C busy\n");*/ };

	// Set Slave Address
	bcm2835_peri_write_nb(reg_addr, address);
    // Clear Status Flags
	bcm2835_peri_write_nb(reg_stat, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    bcm2835_peri_write_nb(reg_dlen, quantity);
    // Start Read
    bcm2835_peri_write_nb(reg_ctrl, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_CLEAR_1 | BCM2835_BSC_C_ST | BCM2835_BSC_C_READ);

    // Reset rxBuffer Index
    rxBufferIndex  = 0;
    rxBufferLength = 0;
    // Wait for transfer to complete
    while (!(bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_DONE)) {
        // Empty the FIFO as it is populated
        while (bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_RXD) {
    		rxBuffer[rxBufferIndex] = (uint8_t)bcm2835_peri_read_nb(reg_fifo);
    		rxBufferIndex++;
    		rxBufferLength++;
    	}
    }

    // Empty the FIFO after all I2C transfer has been DONE
    while (bcm2835_peri_read_nb(reg_stat) & BCM2835_BSC_S_RXD) {
        rxBuffer[rxBufferIndex] = (uint8_t)bcm2835_peri_read_nb(reg_fifo);
        rxBufferIndex++;
        rxBufferLength++;
    }

    // Received an ERR
    if (bcm2835_peri_read(reg_stat) & BCM2835_BSC_S_ERR) {
        printf("Error Occured\n");
    }

    // Clock Stretch Timeout
    else if (bcm2835_peri_read(reg_stat) & BCM2835_BSC_S_CLKT) {
        printf("Slave Timeout\n");
    }

    rxBufferIndex = 0;
    //    printf ("DEBUG INFO: rxBufferIndex=0x%02x, rxBufferLength=0x%02x\n",  rxBufferIndex, rxBufferLength);
    return rxBufferLength;
}


int WirePi::available(){
	return rxBufferLength - rxBufferIndex;
}


int WirePi::read(){
    int value = -1;

    // get each successive byte on each call
    if(rxBufferIndex < rxBufferLength){
        value = rxBuffer[rxBufferIndex];
        rxBufferIndex++;
    }

    return value;
}


int WirePi::peek(void)
{
    int value = -1;

    if(rxBufferIndex < rxBufferLength){
        value = rxBuffer[rxBufferIndex];
    }

    return value;
}


/*******************
 * Private methods *
 *******************/

// Exposes the physical address defined in the passed structure using mmap on /dev/mem
int WirePi::map_peripheral(struct bcm2835_peripheral *p) {
   // Open /dev/mem
   if ((p->mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("Failed to open /dev/mem, try checking permissions.\n");
      return -1;
   }

   p->map = mmap(
      NULL,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED,
      p->mem_fd,  // File descriptor to physical memory virtual file '/dev/mem'
      p->addr_p   // Address in physical map that we want this memory block to expose
   );

   if (p->map == MAP_FAILED) {
        perror("mmap");
        return -1;
   }

   p->addr = (volatile unsigned int *)p->map;

   return 0;
}

void WirePi::unmap_peripheral(struct bcm2835_peripheral *p) {
    munmap(p->map, BLOCK_SIZE);
    unistd::close(p->mem_fd);
}

