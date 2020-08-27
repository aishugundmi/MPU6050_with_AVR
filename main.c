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
    int16_t Ax;
	int16_t Ay;
	int16_t Az;

	int16_t temp;

	int16_t Gx;
	int16_t Gy;
	int16_t Gz;

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
    int16_t res_16;
    i2c_read_bytes(MPU6050_Address, 0x3B, 14, buff);

    res_16=(int16_t)buff[0];
    res_16=((res_16 << 8) & 0xFF00)| buff[1];
	tmp->Ax=res_16;

    res_16=(int16_t)buff[2];
    res_16=((res_16 << 8) & 0xFF00)| buff[3];
	tmp->Ay=res_16;

    res_16=(int16_t)buff[4];
    res_16=((res_16 << 8) & 0xFF00)| buff[5];
	tmp->Az=res_16;

	res_16=(int16_t)buff[6];
    res_16=((res_16 << 8) & 0xFF00)| buff[7];
	tmp->temp=res_16;

	res_16=(int16_t)buff[8];
    res_16=((res_16 << 8) & 0xFF00)| buff[9];
	tmp->Gx=res_16;

    res_16=(int16_t)buff[10];
    res_16=((res_16 << 8) & 0xFF00)| buff[11];
	tmp->Gy=res_16;

    res_16=(int16_t)buff[12];
    res_16=((res_16 << 8) & 0xFF00)| buff[13];
	tmp->Gz=res_16;

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


    while(1)
    {
            char print_buf1[100];
            mpu_data_t tmp;
            read_mpu(&tmp);

            sprintf(print_buf1, "Ax= %d    Ay= %d    Az= %d\t\t\tGx= %d     Gy= %d      Gz= %d\n", tmp.Ax, tmp.Ay, tmp.Az, tmp.Gx, tmp.Gy, tmp.Gz);
            USART_putstring(print_buf1);

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







