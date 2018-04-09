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
#include "TWI_slave.h"
#include "settings.h"
//--------------------------------------------------------------------

/**
 * デバッグ用シリアルポートを初期化する
 */

#define SERIAL_COM_CONCAT2(A) A##0 // rewrite '0' to your favorite port
#define SERIAL_COM_CONCAT3(A,C) A##0##C // rewrite '0' to your favorite port

static void init_serial()
{
	
	SERIAL_COM_CONCAT3(UBRR, L) = 8; // apprx. 115200baud
	SERIAL_COM_CONCAT3(UBRR, H) = 0;
	SETR(SERIAL_COM_CONCAT3(UCSR, A),
		SERIAL_COM_CONCAT2(U2X)); // High speed
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
void debug_send_P(const char * str)
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
void debug_send(const char * str)
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
void debug_send_n(long int n)
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
		disable ADC interrupt, x64 prescaler = 125k conversion clock
	*/
	SETR(ADCSRA, ADEN, ~ADATE, ADIF, ~ADIE, ADPS2, ADPS1, ~ADPS0);

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


#define LINE_BUF_LEN 80
static char line_buf[LINE_BUF_LEN + 1];
static uint8_t receive_index = 0;


static void poll_serial()
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
			parse_command(line_buf); // in settings.c
			receive_index = 0;

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


/* -------------------------------------------------------------------- */



/* -------------------------------------------------------------------- */
static unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.

	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
	PORTB = TWIerrorMsg;
	TWI_Start_Transceiver();
	
	return TWIerrorMsg; 
}

static unsigned char messageBuf[TWI_BUFFER_SIZE];


// TWI polling
static void poll_TWI()
{

	// Check if the TWI Transceiver has completed an operation.
	if ( ! TWI_Transceiver_Busy() )
	{
		// Check if the last operation was successful
		if ( TWI_statusReg.lastTransOK )
		{
			// Check if the last operation was a reception
			if ( TWI_statusReg.RxDataInBuf )
			{
				TWI_Get_Data_From_Transceiver(messageBuf, 3);
				// Check if the last operation was a reception as General Call
				if ( TWI_statusReg.genAddressCall )
				{
					; /* do nothing */
				}
				else // Ends up here if the last operation was a reception as Slave Address Match
				{
					uint8_t return_size = 0;
					// interpret I2C commands
					switch(messageBuf[0])
					{
					case TWI_CMD_PING: // ping
						messageBuf[0] = 0xaa; // pong
						return_size = 1;
						break;

					case TWI_CMD_QUERY_EEPROM_SIZE: // eeprom size query
						messageBuf[0] = get_eeprom_size();
						messageBuf[1] = TWI_BUFFER_SIZE;
						return_size = 2;
						break;	

					case TWI_CMD_COPY_EEPROM: // eeprom block transfer
					  {
					    uint8_t start = messageBuf[1];
					    uint8_t size = messageBuf[2]; // TODO: parameter check
						copy_shadow_eeprom(messageBuf, start, size);
						return_size = size;
					  }
						break;
					}

					if(return_size)
						TWI_Start_Transceiver_With_Data( messageBuf, return_size );

				}
			}
			else // Ends up here if the last operation was a transmission
			{
				; // Put own code here.
			}
			// Check if the TWI Transceiver has already been started.
			// If not then restart it to prepare it for new receptions.
			if ( ! TWI_Transceiver_Busy() )
			{
			  TWI_Start_Transceiver();
			}
		}
		else // Ends up here if the last operation completed unsuccessfully
		{
			TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info() );
		}
	}
}

int main(void) __attribute__((noreturn));
int main(void)
{
	/* disable WDT */
	wdt_disable();

	// wait for power stabilization
	_delay_ms(50);

	// setup
	DDR

	adc_setup();
	init_serial();
	TWI_Slave_Initialise( (unsigned char)((I2C_SUB_SLAVE_ADDR<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); 
	TWI_Start_Transceiver();
	eeprom_setup();

	// enter infinite loop
	sei();
	for(;;)
	{
		// poll actions
		poll_serial();
		poll_TWI();
	}
}

/* -------------------------------------------------------------------- */

