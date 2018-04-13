#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Port for the I2C
#define I2C_DDR DDRD
#define I2C_PIN PIND
#define I2C_PORT PORTD

#define I2C_ERROR_ADDRESS_NO_ACK 1
#define I2C_ERROR_DATA_NO_ACK 2

// Pins to be used in the bit banging
#define I2C_CLK 2
#define I2C_DAT 6

#define I2C_DATA_HI()\
I2C_DDR &= ~ (1 << I2C_DAT);
#define I2C_DATA_LO()\
I2C_DDR |= (1 << I2C_DAT);

#define I2C_STRETCH_LIMIT 500 // in us(1) tick

static void delay1() { _delay_us(1); }

static void delay(uint8_t n)
{
	while(n--) _delay_us(10);
}

static void I2C_CLOCK_HI()
{
	I2C_DDR &= ~ (1 << I2C_CLK);
	uint16_t count = I2C_STRETCH_LIMIT;
	while(!(I2C_PIN & (1<<I2C_CLK)) && --count)
		delay1();
}


#define I2C_CLOCK_LO()\
I2C_DDR |= (1 << I2C_CLK);



static void I2C_WriteBit(unsigned char c)
{
    if (c > 0)
    {
        I2C_DATA_HI();
    }
    else
    {
        I2C_DATA_LO();
    }

    delay(1);
    I2C_CLOCK_HI();
    delay(1);

    I2C_CLOCK_LO();
    delay(1);

    if (c > 0)
    {
        I2C_DATA_LO();
    }

    delay(1);
}

static unsigned char I2C_ReadBit()
{
    I2C_DATA_HI();

    delay(1);
    I2C_CLOCK_HI();
    delay(1);

    unsigned char c = !!(I2C_PIN & (1<<I2C_DAT));

    I2C_CLOCK_LO();
    delay(1);

    return c;
}

// Inits bitbanging port, must be called before using the functions below
//
void I2C_Init()
{
    I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));

    I2C_CLOCK_HI();
    I2C_DATA_HI();

    delay(1);
}

// Send a START Condition
//
void I2C_Start()
{
	I2C_DATA_HI();

    delay(1);
	I2C_CLOCK_HI();

    delay(1);

    I2C_DATA_LO();
    delay(1);

    I2C_CLOCK_LO();
    delay(1);
}

// Send a STOP Condition
//
void I2C_Stop()
{
	I2C_DATA_LO();
    delay(1);

    I2C_CLOCK_HI();
    delay(1);

    I2C_DATA_HI();
    delay(1);
}

// write a byte to the I2C slave device
//
unsigned char I2C_Write(unsigned char c)
{
	char i;
    for (i = 0; i < 8; i++)
    {
        I2C_WriteBit(c & 128);

        c <<= 1;
    }

	return I2C_ReadBit();
}


// read a byte from the I2C slave device
//
unsigned char I2C_Read(unsigned char ack)
{
    unsigned char res = 0;
    char i;

    for (i = 0; i < 8; i++)
    {
        res <<= 1;
        res |= I2C_ReadBit();
    }

    if (ack > 0)
    {
        I2C_WriteBit(0);
    }
    else
    {
        I2C_WriteBit(1);
    }

    delay(1);

    return res;
}

uint8_t I2C_WriteData(uint8_t slave_addr_7bit, uint8_t *buf, uint8_t size, uint8_t stop)
{
	uint8_t nack;
	I2C_Start();
	nack = I2C_Write((slave_addr_7bit << 1));
	if(nack)
	{
		I2C_Stop();
		return I2C_ERROR_ADDRESS_NO_ACK;
	}
	while(size--)
	{
		nack = I2C_Write(*(buf++));
		if(nack)
		{
			I2C_Stop();
			return I2C_ERROR_DATA_NO_ACK;
		}
	}
	if(stop) I2C_Stop();
	return 0;
}

uint8_t I2C_ReadData(uint8_t slave_addr_7bit, uint8_t *buf, uint8_t size)
{
	uint8_t nack;
	I2C_Start();
	nack = I2C_Write((slave_addr_7bit << 1) | 1);
	if(nack)
	{
		I2C_Stop();
		return I2C_ERROR_ADDRESS_NO_ACK;
	}
	while(size--) { *(buf++) = I2C_Read(size == 0 ? 0 : 1); }
	I2C_Stop();
	return 0;
}



