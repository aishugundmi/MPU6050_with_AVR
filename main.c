#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#include "i2c.h"


#define BAUDRATE 115200UL
#define BAUD_PRESCALLER 16


#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40

#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_RA_WHO_AM_I 0x75

/*........................................................................................................*/
// MPU6050 Slave Device Address
const uint8_t MPU6050_Address = 0x68;


// MPU6050  configuration register addresses
const uint8_t MPU6050_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_USER_CTRL    =  0x6A;
const uint8_t MPU6050_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_CONFIG       =  0x1A;
const uint8_t MPU6050_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_FIFO_EN      =  0x23;
const uint8_t MPU6050_INT_ENABLE   =  0x38;
const uint8_t MPU6050_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_SIGNAL_PATH_RESET  = 0x68;


uint8_t ser[14];
/*...........................................................................................................*/

void MPU6050_Init();
void mpu6050_read_gyro_X(uint8_t * buff);
void mpu6050_read_gyro_Y(uint8_t * buff);
void mpu6050_read_gyro_Z(uint8_t * buff);
void mpu6050_read_accel_X(uint8_t * buff);
void mpu6050_read_accel_Y(uint8_t * buff);
void mpu6050_read_accel_Z(uint8_t * buff);


void USART_init(void);
void USART_send(unsigned char data);
void USART_putstring(char* StringPtr);

/*...........................................................................................................*/


//configure MPU6050
void MPU6050_Init()
{
  _delay_ms(150);
  i2c_write_byte(MPU6050_SMPLRT_DIV, 0x07);
  i2c_write_byte(MPU6050_PWR_MGMT_1, 0x01);
  i2c_write_byte(MPU6050_PWR_MGMT_2, 0x00);
  i2c_write_byte(MPU6050_CONFIG, 0x00);
  i2c_write_byte(MPU6050_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  i2c_write_byte(MPU6050_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  i2c_write_byte(MPU6050_FIFO_EN, 0x00);
  i2c_write_byte(MPU6050_INT_ENABLE, 0x01);
  i2c_write_byte(MPU6050_SIGNAL_PATH_RESET, 0x00);
  i2c_write_byte(MPU6050_USER_CTRL, 0x00);
}



void mpu6050_read_gyro_X(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_GYRO_XOUT_H, buff);
	i2c_read_byte(MPU6050_RA_GYRO_XOUT_L, buff+1);
}


void mpu6050_read_gyro_Y(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_GYRO_YOUT_H, buff);
	i2c_read_byte(MPU6050_RA_GYRO_YOUT_L, buff+1);
}


void mpu6050_read_gyro_Z(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_GYRO_ZOUT_H, buff);
	i2c_read_byte(MPU6050_RA_GYRO_ZOUT_L, buff+1);
}


void mpu6050_read_accel_X(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_ACCEL_XOUT_H, buff);
	i2c_read_byte(MPU6050_RA_ACCEL_XOUT_L, buff+1);
}


void mpu6050_read_accel_Y(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_ACCEL_YOUT_H, buff);
	i2c_read_byte(MPU6050_RA_ACCEL_YOUT_L, buff+1);
}


void mpu6050_read_accel_Z(uint8_t * buff)
{
	i2c_read_byte(MPU6050_RA_ACCEL_ZOUT_H, buff);
	i2c_read_byte(MPU6050_RA_ACCEL_ZOUT_L, buff+1);
}



int main(void)
{

    DDRD |= (1<<DDD3);

    MPU6050_Init();
    i2c_init();
    USART_init();

        uint8_t res;
        i2c_read_byte(MPU6050_RA_WHO_AM_I, &res);
		if(res==0x68)
        {
			PORTD ^= (1<<PD3);
		}


    while(1)
    {

            ser[0]=0xF7;

            uint8_t tmp1[2], tmp2[2], tmp3[2], tmp4[2], tmp5[2], tmp6[2];

            mpu6050_read_gyro_X(tmp1);
            ser[1]= tmp1[0];     // high byte
			ser[2]= tmp1[1];     // low byte

			mpu6050_read_gyro_Y(tmp2);
			ser[3]= tmp2[0];    // high byte
			ser[4]= tmp2[1];    // low byte

			mpu6050_read_gyro_Z(tmp3);
			ser[5]= tmp3[0];     // high byte
			ser[6]= tmp3[1];     // low byte



			mpu6050_read_accel_X(tmp4);
			ser[7]= tmp4[0];     // high byte
			ser[8]= tmp4[1];     // low byte

            mpu6050_read_accel_Y(tmp5);
            ser[9]= tmp5[0];     // high byte
			ser[10]= tmp5[1];    // low byte

            mpu6050_read_accel_Z(tmp6);
            ser[11]= tmp6[0];     // high byte
			ser[12]= tmp6[1];    // low byte


			USART_send(ser[0]);
			USART_send(ser[1]);
			USART_send(ser[2]);
			USART_send(ser[3]);
			USART_send(ser[4]);
			USART_send(ser[5]);
			USART_send(ser[6]);
			USART_send(ser[7]);
			USART_send(ser[8]);
			USART_send(ser[9]);
			USART_send(ser[10]);
			USART_send(ser[11]);
			USART_send(ser[12]);

            _delay_ms(10);
    }

    return 0;
}





void USART_init(void)
{
    UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER);
    UCSR0A |= (1<<U2X0);      //U2X0=1
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_send( unsigned char data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void USART_putstring(char* StringPtr)
{
    while(*StringPtr != 0x00){
    USART_send(*StringPtr);
    StringPtr++;}
}







