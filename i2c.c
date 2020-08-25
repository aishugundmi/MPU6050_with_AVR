#include <avr/io.h>
#include "i2c.h"

#define F_CPU 16000000UL
#define SCL_CLOCK  100000L

void i2c_init()
{
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  // (must be > 10 for stable operation) SCL freq = CPU clock / (16+2*TWBR*prescaler), 16+2(TWBR)*1 = 16MHz / 100KHz. TWBR=72=48HEX
    TWCR = (1<<TWEN);   // Enable I2C, changes the pin's from ADC to i2c pin's
    TWSR = 0x00;        // Prescaler set to 1
}

void i2c_start()
{
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA); // Start Condition
    while (!(TWCR & (1<<TWINT))); // Check for Start condition successful
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);     /* send stop condition */
	while(TWCR & (1<<TWSTO));                      // wait until stop condition is executed and bus released
}

unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

    return TWDR;
}

void i2c_write(char data)
{
    TWDR = data; // Move value to I2C register
    TWCR = (1<<TWINT) | (1<<TWEN); // Enable I2C and Clear Interrupt
    while (!(TWCR & (1<<TWINT))); // Write Successful with TWDR Empty
}


// read one byte from dev, stored in value, return 1 for error
void i2c_read_byte(uint8_t reg_addr, uint8_t* buf)
{
	i2c_start();
	i2c_write(reg_addr);			//write address of register to read
	i2c_start();
	*buf = i2c_readNak();
    i2c_stop();
}


// write one byte to dev
void i2c_write_byte(uint8_t reg_addr, uint8_t data)
{
	i2c_start();
 	i2c_write(reg_addr);
    i2c_write(data);
	i2c_stop();
}




