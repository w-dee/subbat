#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>	/* for sei() */
#include <util/delay.h>		/* for _delay_ms() */
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <setr.h>
#include "USI_TWI_Slave.h"


#define LCD_CONTRAST_PIN 3 // PORT B
#define BACKLIGHT_PIN 4 // PORT B
#define STATUS_0_PIN 6 // PORT B
#define STATUS_1_PIN 0 // PORT A
#define STATUS_2_PIN 1 // PORT A

#define SWITCH_0_PIN 1 // PORT B
#define SWITCH_1_PIN 2 // PORT B

#define CHARGE_PIN 0 // PORT D
#define SENS_PIN 0 // PORT B


/* ------------------------------------------------------------------------- */
// LCD functions

static void lcd_data(uint8_t nibble)
{
	nibble &= 0x0f;
	nibble <<= 3;
	// output nibble to PORTD  3...6
	PORTD &= 0b10000111;
	PORTD |= nibble;
}

static void lcd_set_rs(uint8_t n)
{
	if(n)
		SETR(PORTD, 2);
	else
		SETR(PORTD, ~2);
}

static void lcd_set_e(uint8_t n)
{
	if(n)
		SETR(PORTD, 1);
	else
		SETR(PORTD, ~1);
}

static void __attribute__((noinline))  lcd_delay() { _delay_us(20); }
static void __attribute__((noinline))  lcd_delay4ms()  { _delay_ms(4); }

static void lcd_8bit_command(uint8_t n)
{
	lcd_set_e(0);
	lcd_data(n);
	lcd_set_rs(0);
	lcd_set_e(1);
	lcd_delay();
	lcd_set_rs(0);
}

static void __attribute__((noinline))  _lcd_4bit(uint8_t n)
{
	lcd_data(n >> 4);
	lcd_set_e(1);
	lcd_delay();
	lcd_set_e(0);
	lcd_delay();
	lcd_data(n);
	lcd_set_e(1);
	lcd_delay();
	lcd_set_e(0);
	lcd_delay();
	lcd_delay();
}

static void lcd_4bit_command(uint8_t n)
{
	lcd_set_rs(0);
	_lcd_4bit(n);
}

static void lcd_4bit_data(uint8_t n)
{
	lcd_set_rs(1);
	_lcd_4bit(n);
}

static void init_lcd()
{
	SETR(PORTD, 1, 2, 3, 4, 5, 6); // 1=e, 2=rs, 3..6=d[4..7]

	lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();
	lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();

	lcd_8bit_command(0x03); // 8bit mode
	lcd_delay4ms();

	lcd_8bit_command(0x03); // 8bit mode
	lcd_delay4ms();

	lcd_8bit_command(0x03); // 8bit mode
	lcd_delay4ms();

	lcd_8bit_command(0x02); // 4bit mode
	lcd_delay4ms();

	lcd_4bit_command(0x08); // two line mode
	lcd_delay4ms();

	lcd_4bit_command(0x0c); // display on/off
	lcd_delay4ms();

	lcd_4bit_command(0x06); // entry mode
	lcd_delay4ms();

	lcd_4bit_command(0x01); // clear
	lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();lcd_delay4ms();
	lcd_delay4ms();
}

static void lcd_home()
{
	lcd_4bit_command(0x80); // set DDRAM address
}

static void lcd_next_line()
{
	// since currently this module only support two-line LCDs... TODO: support more
	lcd_4bit_command(0x80 + 0x40); // set DDRAM address
}

static void lcd_print(uint8_t *str, uint8_t len)
{
	for(uint8_t i = 0; i < len; ++i)
	{
		uint8_t ch = str[i];
		switch(ch)
		{
		case 0x0c: // ^V ; used to move to home
			lcd_home();
			break;

		case '\n': // next line
			lcd_next_line();
			break;

		case '\r': // ignore
			break;

		default:
			lcd_4bit_data(ch); // write to ddram
			break;
		}
	}
}

/* ------------------------------------------------------------------------- */
// Timer0 setup

static uint8_t pwm_count;
static uint8_t lcd_contrast = 12;
static uint8_t brightness = 50;
static uint8_t status_led;


static void init_timer0()
{
	/* Normal mode, prescaler = clkio/1, 31250.0Hz interrupt */
	SETR(TCCR0A, ~COM0A1, ~COM0A0, ~COM0B1, ~COM0B0, ~WGM01, ~WGM00);
	SETR(TCCR0B, ~FOC0A, ~FOC0B, ~WGM02, ~CS02, ~CS01, CS00);
	SETR(TIMSK, TOIE0); // enable overflow interrupt
}

static void light_on()
{
	SETR(PORTB, BACKLIGHT_PIN);

	uint8_t led = status_led;

	if(led & 1)
		SETR(PORTB, STATUS_0_PIN);
	else
		SETR(PORTB, ~STATUS_0_PIN);

	if(led & 2)
		SETR(PORTA, STATUS_1_PIN);
	else
		SETR(PORTA, ~STATUS_1_PIN);

	if(led & 4)
		SETR(PORTA, STATUS_2_PIN);
	else
		SETR(PORTA, ~STATUS_2_PIN);
}

static void light_off()
{
	SETR(PORTB, ~BACKLIGHT_PIN);
	SETR(PORTB, ~STATUS_0_PIN);
	SETR(PORTA, ~STATUS_1_PIN);
	SETR(PORTA, ~STATUS_2_PIN);
}


ISR(TIMER0_OVF_vect)
{
	uint8_t count = ++ pwm_count;

	if(count <= lcd_contrast)
		SETR(PORTB, LCD_CONTRAST_PIN);
	else
		SETR(PORTB, ~LCD_CONTRAST_PIN);

	if((uint8_t)(~count) <= brightness)
		light_on();
	else
		light_off();
}

/* ------------------------------------------------------------------------- */
volatile static uint8_t ambient_brightness;
volatile static uint8_t switch_positions;

static uint8_t _read_ambient()
{
	// force discharge capacitor
	SETR(PORTB, ~SENS_PIN);
	SETR(DDRB, SENS_PIN);
	SETR(PORTD, ~CHARGE_PIN);
	SETR(DDRD, CHARGE_PIN);

	// wait for discharging capacitor
	_delay_us(100);

	// float SENS pin and set CHARGE pin HIGH
	SETR(DDRB, ~SENS_PIN);
	SETR(PORTD, CHARGE_PIN);

	// measure time to SENS_PIN gets high
	cli();
	volatile uint8_t n;
	for(n = 0; n < 255; n++)
		if(PINB & (1<<SENS_PIN)) break;
	sei();

	return ~n;
}

// read ambient brightness and switches
static void read_switches()
{
	// read switch position
	switch_positions = 
		(PINB & (1<<SWITCH_0_PIN) ? 1 : 0 )+
		(PINB & (1<<SWITCH_1_PIN) ? 2 : 0 );

	// store the result
	uint16_t avg = 0;
	for(uint8_t i = 0; i < 16; ++i)
		avg += _read_ambient();
	ambient_brightness = avg / 16;

	// force discharge capacitor
	SETR(PORTB, ~SENS_PIN);
	SETR(DDRB, SENS_PIN);
	SETR(PORTD, ~CHARGE_PIN);
	SETR(DDRD, CHARGE_PIN);
}


/* ------------------------------------------------------------------------- */

#define I2C_SLAVE_ADDR 0x12


int main(void)
{
	wdt_disable();

	PORTB = 0xff;
	PORTA = 0xff;
	PORTD = 0xff;
	DDRB = 0;
	DDRA = 0;
	DDRD = 0;

	SETR(DDRB, BACKLIGHT_PIN, LCD_CONTRAST_PIN, STATUS_0_PIN);
	SETR(DDRA, STATUS_1_PIN, STATUS_2_PIN);

	SETR(DDRB, ~SWITCH_0_PIN, ~SWITCH_1_PIN);
	SETR(PORTB, SWITCH_0_PIN, SWITCH_1_PIN); // weak pull-up

	init_timer0();
	init_lcd();

	USI_TWI_Slave_Initialise(I2C_SLAVE_ADDR);
	sei();

	for(;;)
	{
		if(USI_TWI_Data_In_Receive_Buffer())
		{
			uint8_t cmd = USI_TWI_Receive_Byte();
			switch(cmd >> 4)
			{
			case 0x00: // ping
				USI_TWI_Transmit_Byte(~cmd); // pong
				break;

			case 0x01: // LCD write
			  {
				uint8_t n = cmd & 4;
				uint8_t buf[16];
				for(uint8_t i = 0; i < n; ++i)
					buf[i] = USI_TWI_Receive_Byte();
				lcd_print(buf, n);
				break;
			  }

			case 0x02: // LCD contrast
				lcd_contrast = USI_TWI_Receive_Byte();
				break;

			case 0x03: // Backlight brightness
				brightness = USI_TWI_Receive_Byte();
				break;

			case 0x05: // write led status
				status_led = USI_TWI_Receive_Byte();
				break;

			case 0x06: // read status
				read_switches();
				USI_TWI_Transmit_Byte(ambient_brightness);
				USI_TWI_Transmit_Byte(switch_positions);
				break;

			case 0x07: // read eeprom
			  {
				uint8_t start = USI_TWI_Receive_Byte();
				uint8_t length = USI_TWI_Receive_Byte();
				while(length -- )
				{
					USI_TWI_Transmit_Byte(eeprom_read_byte( (void*)(int)start));
					++ start;
				}
				break;
			  }
			}
		}
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
