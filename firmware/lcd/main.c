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
#define PUSH_SW_PIN 6 // PORT B

#define CHARGE_PIN 0 // PORT D
#define SENS_PIN 0 // PORT B


/* ------------------------------------------------------------------------- */
// LCD functions

#include "dots.h"

// user-defined character data
static const  uint8_t cg_data[][8] PROGMEM = {

{
/* code 0x08 */
XX________,
__________,
____XXXXXX,
__XX______,
__XX______,
__XX______,
____XXXXXX,
0,
},

{
/* code 0x09 */
____XX____,
__XX______,
__XXXXXXXX,
________XX,
XX__XXXX__,
XXXX______,
XXXXXX____,

0,
},

};

static void lcd_data(uint8_t nibble)
{
	nibble &= 0x0f;
	nibble <<= 3;
	// output nibble to PORTD  3...6
	uint8_t tmp;
	cli(); // prevent changing PORTD state in interrupt routine
	tmp = PORTD;
	tmp &= 0b10000111;
	tmp += nibble;
	PORTD = tmp;
	sei();
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

	lcd_4bit_command(0x40); // set CGRAM address

	// register user-defined glyphs
	for(uint8_t c = 0; c < sizeof(cg_data); ++c)
	{
		//assumes cg_data array data is continuous on progmem
		lcd_4bit_data(pgm_read_byte((uint8_t*)(& ( cg_data[0][0] )) + c));
	}


	lcd_4bit_command(0x80); // set DDRAM address
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

static uint8_t pwm_count = 255;
static uint8_t lcd_contrast = 12;
static uint8_t brightness = 50;
static uint8_t led_status;


static void init_timer0()
{
	/* Normal mode, prescaler = clkio/1, 31250.0Hz interrupt */
	SETR(TCCR0A, ~COM0A1, ~COM0A0, ~COM0B1, ~COM0B0, ~WGM01, ~WGM00);
	SETR(TCCR0B, ~FOC0A, ~FOC0B, ~WGM02, ~CS02, ~CS01, CS00);
	SETR(TIMSK, TOIE0); // enable overflow interrupt
}

volatile static uint8_t push_button_debounder = 0;
volatile static uint8_t button_pressed = 0;


#define DISCHARGE_START_PHASE  0 // start discharge at this timing
#define CHARGE_START_PHASE 64 // start charge at this timing
volatile static uint8_t measured_time = 0;
static uint8_t measuring = 0;

ISR(TIMER0_OVF_vect)
{
	uint8_t count = ++ pwm_count;

	uint8_t pb = PORTB;
	uint8_t pa = PORTA;

	pb &= ~(
		(1<<BACKLIGHT_PIN) |
		(1<<STATUS_0_PIN) |
		(1<<LCD_CONTRAST_PIN));

	pa &= ~(
		(1<<STATUS_1_PIN) |
		(1<<STATUS_2_PIN));

	if(count <= lcd_contrast)
		pb |= (1<<LCD_CONTRAST_PIN);

	if((uint8_t)(~count) <= brightness)
	{
		uint8_t led = led_status;
		pb |= (1<<BACKLIGHT_PIN);
		if(led & 1)
			pb |= (1<<STATUS_0_PIN);
		if(led & 2)
			pa |= (1<<STATUS_1_PIN);
		if(led & 4)
			pa |= (1<<STATUS_2_PIN);
	}

	PORTB = pb;
	PORTA = pa;

	// charge/discharge to measure phototransistor's current
	if(count == DISCHARGE_START_PHASE)
	{
		// start discharge
		// force discharge capacitor
		SETR(PORTB, ~SENS_PIN);
		SETR(DDRB, SENS_PIN);
		SETR(PORTD, ~CHARGE_PIN);
		SETR(DDRD, CHARGE_PIN);
	}
	else if(count == CHARGE_START_PHASE)
	{
		// start charge
		SETR(DDRB, ~SENS_PIN);
		SETR(PORTD, CHARGE_PIN);
		measuring = 1;	
	}
	else if(measuring && count > CHARGE_START_PHASE)
	{
		if(PINB & (1<<SENS_PIN) || count == 255)
		{
			// charge completed. measuered time is to be IIR filtered.
			uint8_t new_measured_time;
			uint8_t old_measured_time;
			new_measured_time = count - CHARGE_START_PHASE;
			old_measured_time = measured_time;
			uint8_t updated_measured_time;
			updated_measured_time =
				old_measured_time + ((int16_t)(
					(int16_t)new_measured_time - (int16_t)old_measured_time) >> 1);
			measured_time = updated_measured_time;
			measuring = 0;
		}
	}


	// check push button
	if(count == 0)
	{
		// approx. 122Hz intervally executed block;
		// push_button_debounder is latest 8 point history
		// of button pressed-state sampling
		uint8_t deb = push_button_debounder; 
		deb <<= 1;
		if(!(PINB & (1<<PUSH_SW_PIN))) deb |= 1;

		if(deb == 0b00000001) button_pressed = 1;

		push_button_debounder = deb;
	}
}

static uint8_t get_button_pressed()
{
	uint8_t pressed;
	cli();
	pressed = button_pressed;
	button_pressed = 0;
	sei();
	return pressed;
}

/* ------------------------------------------------------------------------- */
volatile static uint8_t ambient_brightness;
volatile static uint8_t switch_positions;

// read ambient brightness and switches
static void read_switches()
{
	// read switch position
	switch_positions = 
		((PINB & (1<<SWITCH_0_PIN) )? 1 : 0 )+
		((PINB & (1<<SWITCH_1_PIN) )? 2 : 0 );


	// store the result
	ambient_brightness = ~measured_time;
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

	SETR(DDRB, ~SWITCH_0_PIN, ~SWITCH_1_PIN, ~PUSH_SW_PIN);
	SETR(PORTB, SWITCH_0_PIN, SWITCH_1_PIN, PUSH_SW_PIN); // weak pull-up

	DDRD = 0xff; // all output

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
				led_status = USI_TWI_Receive_Byte();
				break;

			case 0x06: // read status
				read_switches();
				USI_TWI_Transmit_Byte(ambient_brightness);
				USI_TWI_Transmit_Byte(switch_positions + (get_button_pressed() ? 4 : 0));
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
