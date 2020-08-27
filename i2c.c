#include <avr/io.h>
#include <inttypes.h>

#include "i2c.h"

#define F_CPU 16000000UL
#define SCL_CLOCK  100000L


#define TW_START		    0x08
#define TW_REP_START		0x10
/* .........................Master Transmitter.............................. */
#define TW_MT_SLA_ACK		0x18
#define TW_MT_SLA_NACK		0x20
#define TW_MT_DATA_ACK		0x28
#define TW_MT_DATA_NACK		0x30
#define TW_MT_ARB_LOST		0x38
/*......................... Master Receiver................................. */
#define TW_MR_ARB_LOST		0x38
#define TW_MR_SLA_ACK		0x40
#define TW_MR_SLA_NACK		0x48
#define TW_MR_DATA_ACK		0x50
#define TW_MR_DATA_NACK		0x58
/*.......................... Slave Transmitter................................ */
#define TW_ST_SLA_ACK		0xA8
#define TW_ST_ARB_LOST_SLA_ACK	0xB0
#define TW_ST_DATA_ACK		0xB8
#define TW_ST_DATA_NACK		0xC0
#define TW_ST_LAST_DATA		0xC8
/*.......................... Slave Receiver ..................................*/
#define TW_SR_SLA_ACK		    0x60
#define TW_SR_ARB_LOST_SLA_ACK	0x68
#define TW_SR_GCALL_ACK		    0x70
#define TW_SR_ARB_LOST_GCALL_ACK 0x78
#define TW_SR_DATA_ACK		    0x80
#define TW_SR_DATA_NACK		    0x88
#define TW_SR_GCALL_DATA_ACK	0x90
#define TW_SR_GCALL_DATA_NACK	0x98
#define TW_SR_STOP		        0xA0
#define TW_STATUS_MASK		(_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
#define TW_STATUS		(TWSR & TW_STATUS_MASK)
/*..........................R/~W bit in SLA+R/W address field...................*/
#define TW_READ		1
#define TW_WRITE	0




void i2c_init()
{
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  // (must be > 10 for stable operation) SCL freq = CPU clock / (16+2*TWBR*prescaler), 16+2(TWBR)*1 = 16MHz / 100KHz. TWBR=72=48HEX
    TWCR = (1<<TWEN);                 // Enable I2C, changes the pin's from ADC to i2c pin's
    TWSR = 0x00;                      // Prescaler set to 1
}

unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}


void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));

    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);

    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));

    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
    	{
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));

    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}


void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}


unsigned char i2c_write( unsigned char data )
{
    uint8_t   twst;

	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}


unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

    return TWDR;

}


unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

    return TWDR;

}



void i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{
	i2c_start_wait(dev_addr+TW_WRITE); 	//start i2c to write register address
	i2c_write(reg_addr);			        //write address of register to read
	i2c_start(dev_addr+TW_READ);	    //restart i2c to start reading
	*data = i2c_readNak();
    i2c_stop();
}


// read multiple bytes
void i2c_read_bytes(uint8_t dev_addr, uint8_t first_reg_addr, uint8_t length, uint8_t* data)
{
	i2c_start_wait(dev_addr+TW_WRITE); 	//start i2c to write register address
	i2c_write(first_reg_addr);		//write address of register to read
	i2c_start(dev_addr+TW_READ);	//restart i2c to start reading

	uint8_t i;
	for(i=0; i<length-1; i++)
    {
		*(data+i) = i2c_readAck();
	}

	*(data+i) = i2c_readNak();
    i2c_stop();
}

void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	i2c_start_wait(dev_addr+TW_WRITE);
 	i2c_write(reg_addr);
    i2c_write(data);
	i2c_stop();
}



