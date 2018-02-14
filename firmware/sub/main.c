#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "setr.h"
#include "I2C_slave.h"

#define I2C_SLAVE_ADDR 0x16
//--------------------------------------------------------------------

/**
 * デバッグ用シリアルポートを初期化する
 */

#define SERIAL_COM_CONCAT2(A) A##0 // rewrite '0' to your favorite port
#define SERIAL_COM_CONCAT3(A,C) A##0##C // rewrite '0' to your favorite port


static void init_serial()
{
	
	SERIAL_COM_CONCAT3(UBRR, L) = 10; // apprx. 115200baud
	SERIAL_COM_CONCAT3(UBRR, H) = 0;
	SETR(SERIAL_COM_CONCAT3(UCSR, A),
		~SERIAL_COM_CONCAT2(U2X)); // normal speed
	SETR(SERIAL_COM_CONCAT3(UCSR, B),
		~SERIAL_COM_CONCAT2(RXCIE),
		~SERIAL_COM_CONCAT2(TXCIE),
		SERIAL_COM_CONCAT2(TXEN),
		SERIAL_COM_CONCAT2(RXEN),
		~SERIAL_COM_CONCAT3(UCSZ, 2));
	SETR(SERIAL_COM_CONCAT3(UCSR, C),
		SERIAL_COM_CONCAT3(UCSZ, 1),
		SERIAL_COM_CONCAT3(UCSZ, 0)); // 8bit, disable rx/tx interrupt
}


static void send(unsigned char x)
{
	if(x == '\n') send('\r');
	while ( !( SERIAL_COM_CONCAT3(UCSR, A) & (1<<SERIAL_COM_CONCAT2(UDRE))) ) /**/ ;
	SERIAL_COM_CONCAT2(UDR)  = x;
}


/**
 * 文字列を送信する
 */
static void debug_send_P(const char * str)
{
	for(;;)
	{
		uint8_t ch = pgm_read_byte(str);
		if(!ch) break;
		send(ch);
		str++;
	}
}
/**
 * 文字列を送信する
 */
static void debug_send(const char * str)
{
	for(;;)
	{
		uint8_t ch = *str;
		if(!ch) break;
		send(ch);
		str++;
	}
}

/**
 * 数値を送信する
 */
static void debug_send_n(long int n)
{
	char buf[16];
	ltoa(n, buf, 10);
	debug_send(buf);
}


static uint8_t receive( void )
{
	if(!(SERIAL_COM_CONCAT3(UCSR, A) & (1<<SERIAL_COM_CONCAT2(RXC)))) return 0;

	
	if(SERIAL_COM_CONCAT3(UCSR, A) & (1<<SERIAL_COM_CONCAT2(FE)) )
	{
		// frame error
		(void) SERIAL_COM_CONCAT2(UDR); // ダミーリード
		return 0;
	}
	else 
	{
		return SERIAL_COM_CONCAT2(UDR);
	}
}


//--------------------------------------------------------------------
// ADC 関連

static uint8_t adc_last_ch;


static void adc_setup()
{
	/* adc を初期化する */
	/*
		ADC auto trigger disabled
		disable ADC interrupt, x128 prescaler = 156k conversion clock
	*/
	SETR(ADCSRA, ADEN, ~ADATE, ADIF, ~ADIE, ADPS2, ADPS1, ADPS0);

	/*
		No free running mode
	*/
	SETR(ADCSRB, ~ADTS2, ~ADTS1, ~ADTS0);

	/*
		AVcc with external capacitor at AREF pin
		result in left adjusted
	*/
	SETR(ADMUX, ~REFS1, REFS0, ~ADLAR, ~MUX3, ~MUX2, ~MUX1, ~MUX0);
	_delay_ms(10); // VREFが安定するまで待つ
	adc_last_ch = -1;
}

__attribute__((noinline)) static uint16_t _get_adc()
{
	SETR(ADCSRA, ADSC);
	while((ADCSRA & (1<<ADSC)));
	uint16_t adc_last_value =
			({
			typedef union
			{
				struct
				{
					uint8_t L;
					uint8_t H;
				};
				uint16_t S;
			} sd;
			sd v;

			cli(); // 16bit register 保護
				v.L = ADCL;
				v.H = ADCH;
			sei();

			v.S;
		});

	return adc_last_value;
}

__attribute__((noinline)) static uint16_t get_adc(uint8_t ch)
{
	if(ch != adc_last_ch)
	{
		adc_last_ch = ch;
		ADMUX = (ADMUX & 0xf0) | ch;
		_get_adc(); // ダミーリード
	}
	return _get_adc();
}

static float get_voltage(uint8_t ch)
{
	uint16_t v = get_adc(ch);
	float res = v * ((1.0 / 1024.0) * 1.1 / ( 1.0 / (1.0 + 20.0))); 
	return res;
}

//--------------------------------------------------------------------
/* EEPROM 関連 */
#if 0


typedef struct eeprom_tag
{
	uint16_t size;
r
} eeprom_t;
static eeprom_t eeprom;

void eeprom_all_reset()
{
	eeprom.vth_vs = 13.0f;
	eeprom.vth_va = 11.0f;
	eeprom.vth_vd = 10.0f;
	eeprom.vth_vr = 10.0f;

}

static uint16_t calc_checksum()
{
	uint8_t * p = (uint8_t *)&eeprom;
	int i;
	uint8_t s1 = 0, s2 = 0;
	for(i = 0; i < sizeof(eeprom); i++)
	{
		s1 += p[i];
		s2 += s1;
	}
	return ((uint16_t)s2<<8) + s1;
}


void eeprom_setup()
{
	// 内容を読み込む
	uint16_t sum;
	while(!eeprom_is_ready()) /**/;
	eeprom_read_block(&eeprom, (void*)0, sizeof(eeprom));
	eeprom_read_block(&sum, (void*)sizeof(eeprom), sizeof(sum));
	if(calc_checksum() != sum || eeprom.size != sizeof(eeprom))
	{
		debug_send_P(PSTR("EEPROM content broken, force initialize\n"));
		eeprom_all_reset();
	}
}

void eeprom_write_force()
{
	while(!eeprom_is_ready()) /**/;
	eeprom.size = sizeof(eeprom);
	uint16_t sum = calc_checksum();
	// TODO: eeprom書き込み中における do_busy_poll の呼び出し
	eeprom_write_block(&eeprom, (void*)0, sizeof(eeprom));
	eeprom_write_block(&sum, (void*)sizeof(eeprom), sizeof(sum));
}

#endif
//--------------------------------------------------------------------
#if 0
static char sbuf[80];

static void console_status()
{
	sprintf_P(sbuf, PSTR("system voltage: %2.2f   battery voltage: %2.2f    h<enter> for help\r\n"),
		vs, va);
	debug_send(sbuf);
}

static void console_usage()
{
	sprintf_P(sbuf, PSTR("Current settings:\r\n")); 
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("Deep sleep shutdown battery voltage   (vd): %2.2f\r\n"), eeprom.vth_vd);
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("Charge start system input voltage     (vs): %2.2f\r\n"), eeprom.vth_vs);
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("Charge terimnation battery voltage    (va): %2.2f\r\n"), eeprom.vth_va);
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("Active system input voltage threshold (vr): %2.2f\r\n"), eeprom.vth_vr);
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("\r\nCommands:\r\n")); 
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("vd <voltage>    Set vd.\r\n")); 
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("vs <voltage>    Set vs.\r\n")); 
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("va <voltage>    Set va.\r\n")); 
	debug_send(sbuf);
	sprintf_P(sbuf, PSTR("vr <voltage>    Set vr.\r\n")); 
	debug_send(sbuf);
}

static void console_command(const char *p)
{
	if(p[0] == 'v' && p[1] == 'd' && p[2] == ' ')
	{
		float t = atof(p + 3);
		if(t > 0.0f)
		{
			eeprom.vth_vd = t;
			eeprom_write_force();
		}
	}
	else if(p[0] == 'v' && p[1] == 's' && p[2] == ' ')
	{
		float t = atof(p + 3);
		if(t > 0.0f)
		{
			eeprom.vth_vs = t;
			eeprom_write_force();
		}
	}
	else if(p[0] == 'v' && p[1] == 'a' && p[2] == ' ')
	{
		float t = atof(p + 3);
		if(t > 0.0f)
		{
			eeprom.vth_va = t;
			eeprom_write_force();
		}
	}
	else if(p[0] == 'v' && p[1] == 'r' && p[2] == ' ')
	{
		float t = atof(p + 3);
		if(t > 0.0f)
		{
			eeprom.vth_vr = t;
			eeprom_write_force();
		}
	}
	else
	{
		console_usage();
	}
}


#define LINE_BUF_LEN 80
static char line_buf[LINE_BUF_LEN + 1];
static uint8_t receive_index = 0;


void poll_serial()
{
	uint8_t ch = receive();

	if(ch > 0)
	{
		switch(ch)
		{
		case 0x08: // bs
		case 127: // del
			if(receive_index > 0)
			{
				-- receive_index;
				send((char)'\x08');
				send((char)' ');
				send((char)'\x08');
			}
			break;

		case '\r':
			send((char)'\r');
			send((char)'\n');
			line_buf[receive_index] = 0;
			console_command(line_buf);
			receive_index = 0;
			console_status();
			send((char)'>');
			send((char)' ');
			break;

		case '\n':
			break;// discard

		default:
			if(ch >= 0x20)
			{
				if(receive_index < LINE_BUF_LEN)
				{
					line_buf[receive_index] = ch;
					++ receive_index;
					send((char)ch);
				}
			}
			else
			{
				send(' ');
			}
		}
	}
}


#endif
/* -------------------------------------------------------------------- */

uint8_t in_isr_callback_prepare_i2c_write(uint8_t addr)
{
	return 1;
}

void in_isr_callback_done_i2c_write()
{
}

// this is called during i2c isr; user code must
// prepare data in i2c_txbuffer[] and set i2c_txlen.
void in_isr_callback_prepare_i2c_read(uint8_t addr)
{
	if(addr < 128)
	{
		// read eeprom
		i2c_tx_len = 16;
		eeprom_read_block(&(i2c_txbuffer[0]), (void*)(addr * 32), 32);
		return;
	}
/*
	switch(addr)
	{
		case 0:
		
	}
*/
}


/* -------------------------------------------------------------------- */


int main(void) __attribute__((noreturn));
int main(void)
{
	/* disable WDT */
	wdt_disable();

	// wait for power stabilization
	_delay_ms(50);

	// setup
	I2C_init(I2C_SLAVE_ADDR);


	// enter infinite loop
	sei();
	for(;;)
	{
	}

}

/* -------------------------------------------------------------------- */

