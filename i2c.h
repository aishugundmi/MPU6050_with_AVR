#ifndef I2C_H_
#define I2C_H_


void i2c_init();             // Change ADC pin 4 and 5 to i2c
void i2c_start();            // Send a start condition
void i2c_stop();             // Send a stop condition
unsigned char i2c_readNak(void);


void i2c_write(char data);   // Send a byte
void i2c_write_byte(uint8_t reg_addr, uint8_t data);
void i2c_read_byte(uint8_t reg_addr, uint8_t* buf);


#endif /* I2C_H_ */
