#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "i2c.h"


#define BAUDRATE 115200UL
//#define BAUD_PRESCALLER 16

#define BAUD_PRESCALLER 1


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

#define PI  3.141592


/**............................................................................................................*/
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
char print_buf1[100];
volatile uint8_t int_rupt = 0;


/**............................................................................................................*/

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

/**............................................................................................................*/

void MPU6050_Init();
int read_mpu(mpu_data_t *tmp);


void USART_init(void);
void USART_send(unsigned char data);
void USART_putstring(char* StringPtr);

/**.............................................................................................................*/

ISR(INT0_vect)
{
	int_rupt = 1;
}

void timer0pwm_init()
{
    TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS00);  //pre scaling 1
    sei();
}


// configure MPU6050
void MPU6050_Init()
{
    i2c_write_byte(MPU6050_Address, MPU6050_SMPLRT_DIV, 0x4F);  //(8000/100)-1=79
    i2c_write_byte(MPU6050_Address, MPU6050_PWR_MGMT_1, 0x01);
    i2c_write_byte(MPU6050_Address, MPU6050_PWR_MGMT_2, 0x00);
    i2c_write_byte(MPU6050_Address, MPU6050_CONFIG, 0x00);
    //  i2c_write_byte(MPU6050_Address, MPU6050_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
    i2c_write_byte(MPU6050_Address, MPU6050_GYRO_CONFIG, (1<<3)|(1<<4));//set +/-2000degree/second full scale
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

    tmp->Ax = ((((uint16_t)buff[0] << 8) & 0xFF00) | buff[1]);
    tmp->Ay = ((((uint16_t)buff[2] << 8) & 0xFF00) | buff[3]);
    tmp->Az = ((((uint16_t)buff[4] << 8) & 0xFF00) | buff[5]);

	tmp->temp = ((((uint16_t)buff[6] << 8) & 0xFF00) | buff[7]);

	tmp->Gx = ((((uint16_t)buff[8] << 8) & 0xFF00) | buff[9]);
	tmp->Gy = ((((uint16_t)buff[10] << 8) & 0xFF00) | buff[11]);
	tmp->Gz = ((((uint16_t)buff[12] << 8) & 0xFF00) | buff[13]);

	return 0;
}

/**............................................................................................................*/



int main(void)
{

    DDRD |= (0<<PD2);    //PORTD as input INT0
    DDRD |= (1<<PD3);    //ouput LED

    DDRD |= (1<<PD5);
    DDRD |= (1<<PD6);

    EICRA |= (1 << ISC00) | (1 << ISC01);    // set INT0 to trigger on the rising edge
    EIMSK |= (1 << INT0);                    // Turns on INT0
	sei();			                         // Enable Global Interrupt

    i2c_init();
    USART_init();
    MPU6050_Init();
    timer0pwm_init();

    uint8_t res;
    i2c_read_byte(MPU6050_Address, MPU6050_RA_WHO_AM_I, &res);
    if(res==0x68)
    {
        PORTD |= (1<<PD3);
    }

    mpu_data_t tmp;

    uint8_t var=0;
    uint16_t z=0;
    int32_t add_Gx=0, add_Gy=0, add_Gz=0;
    int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
    float angle_Gx = 0, angle_Gy = 0, angle_Gz = 0;
    float G_pitch = 0, G_roll = 0, G_yaw = 0;
    int16_t A_roll = 0, A_pitch = 0;

    while(z < 500)
    {
        read_mpu(&tmp);
        add_Gx += tmp.Gx;
        add_Gy += tmp.Gy;
        add_Gz += tmp.Gz;

        _delay_ms(10);
        z++;
    }
    Gx_offset =  (add_Gx/500);
    Gy_offset =  (add_Gy/500);
    Gz_offset =  (add_Gz/500);

    while(1)
    {
        if(int_rupt == 1)
        {
                int_rupt = 0;

                read_mpu(&tmp);

         //       w_Gx = (tmp.Gx - Gx_offset) * (2000.0/32768.0);
         //       angle_Gx += (w_Gx * 0.01);

                angle_Gx =  ((float)(tmp.Gx - Gx_offset) * 20.0/32768.0);
                angle_Gy =  ((float)(tmp.Gy - Gy_offset) * 20.0/32768.0);
                angle_Gz =  ((float)(tmp.Gz - Gz_offset) * 20.0/32768.0);

                G_roll += angle_Gx;
                G_pitch  -= angle_Gy;
                G_yaw += angle_Gz;

                ///  Calculating Roll and Pitch from the accelerometer data
                A_roll = (atan2((float)(tmp.Ay), (float)tmp.Az) * 180.0)/PI;
                A_pitch = (atan2((float)tmp.Ax, sqrt((float) tmp.Ay*tmp.Ay +  (float) tmp.Az*tmp.Az)) * 180.0)/PI;


				///  COMPLEMENTARY FILTERING of G_roll and G_pitch
				G_roll = (float)G_roll*.98 + (float)A_roll*.02;
				G_pitch = (float)G_pitch*.98 + (float)A_pitch*.02;


                int8_t a = (int8_t)G_roll;

                if((0 < a) && (a < 91) )
                {
                    OCR0A = (uint8_t)((a * 255)/90);
                    OCR0B = 0;
                }
                else if((-91 < a) && (a < 0))
                {
                    OCR0B = (uint8_t)(-1 * (a * 255)/90);
                    OCR0A = 0;
                }
                else
                {
                    OCR0A = 0;
                    OCR0B = 0;
                }

                var++;


            if(var == 10)
            {
               sprintf(print_buf1, "Ar= %d\tAp= %d\tGr= %d\tGp= %d\tGy= %d\n", A_roll, A_pitch, (int)(G_roll), (int)(G_pitch), (int)(G_yaw));
               USART_putstring(print_buf1);
               var = 0;
            }
        }
    }

    return 0;
}


/**..............................................................................................................*/

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


