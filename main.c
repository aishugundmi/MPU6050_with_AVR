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

#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42

#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_RA_WHO_AM_I 0x75


/*........................................................................................................*/
// MPU6050 Slave Device Address
const uint8_t MPU6050_Address = (0x68 << 1);


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
/**............................................................................................................**/

typedef struct
{
    int16_t axh;
	int16_t axl;
	int16_t ayh;
	int16_t ayl;
	int16_t azh;
	int16_t azl;

	int16_t temph;
	int16_t templ;

	int16_t gxh;
	int16_t gxl;
	int16_t gyh;
	int16_t gyl;
	int16_t gzh;
	int16_t gzl;

} mpu_data_t;

/**...........................................................................................................**/

void MPU6050_Init();
int read_mpu(mpu_data_t *tmp);


void USART_init(void);
void USART_send(unsigned char data);
void USART_putstring(char* StringPtr);

/*...........................................................................................................*/


//configure MPU6050
void MPU6050_Init()
{
  i2c_write_byte(MPU6050_Address, MPU6050_SMPLRT_DIV, 0x07);
  i2c_write_byte(MPU6050_Address, MPU6050_PWR_MGMT_1, 0x01);
  i2c_write_byte(MPU6050_Address, MPU6050_PWR_MGMT_2, 0x00);
  i2c_write_byte(MPU6050_Address, MPU6050_CONFIG, 0x00);
  i2c_write_byte(MPU6050_Address, MPU6050_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  i2c_write_byte(MPU6050_Address, MPU6050_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  i2c_write_byte(MPU6050_Address, MPU6050_FIFO_EN, 0x00);
  i2c_write_byte(MPU6050_Address, MPU6050_INT_ENABLE, 0x01);
  i2c_write_byte(MPU6050_Address, MPU6050_SIGNAL_PATH_RESET, 0x00);
  i2c_write_byte(MPU6050_Address, MPU6050_USER_CTRL, 0x00);
}



int read_mpu(mpu_data_t *tmp)
{

    uint8_t buff[14];
    i2c_read_bytes(MPU6050_Address, 0x3B, 14, buff);

    uint8_t i=0;
	tmp->axh=buff[i++];
	tmp->axl=buff[i++];
	tmp->ayh=buff[i++];
	tmp->ayl=buff[i++];
	tmp->azh=buff[i++];
	tmp->azl=buff[i++];

	tmp->temph=buff[i++];
	tmp->templ=buff[i++];

	tmp->gxh=buff[i++];
	tmp->gxl=buff[i++];
	tmp->gyh=buff[i++];
	tmp->gyl=buff[i++];
	tmp->gzh=buff[i++];
	tmp->gzl=buff[i++];

	return 0;
}


int main(void)
{

    DDRD |= (1<<DDD3);

    i2c_init();
    USART_init();
    MPU6050_Init();

    uint8_t res;
    i2c_read_byte(MPU6050_Address, MPU6050_RA_WHO_AM_I, &res);
    if(res==0x68)
    {
        PORTD |= (1<<PD3);
    }

    uint8_t u;

    while(1)
    {

            mpu_data_t tmp;
            read_mpu(&tmp);

            ser[0]=0xF7;

            ser[1]= tmp.axh;     // Ax high byte
			ser[2]= tmp.axl;     // Ax low byte
			ser[3]= tmp.ayh;    // Ay high byte
			ser[4]= tmp.ayl;    // Ay low byte
			ser[5]= tmp.azh;     //Az high byte
			ser[6]= tmp.azl;     //Az low byte

			ser[7]= tmp.gxh;     //Gx high byte
			ser[8]= tmp.gxl;     //Gx low byte
            ser[9]= tmp.gyh;     //Gy high byte
			ser[10]= tmp.gyl;    //Gy low byte
            ser[11]= tmp.gzh;     // high byte
			ser[12]= tmp.gzl;    // low byte


			for(u=0; u<13; u++)
            {
                USART_send(ser[u]);
            }

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







