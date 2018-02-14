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


/*

 Vs : System input voltage (from main battery)
 Va : Aux battery voltage

 Vth_vs : Threshold voltage; the system begin charging if Vs >= Vth_vs
 Vth_vr : Threshold voltage; the system recognize the input is active
          (regardless of the power from system main battery or
          alternator) if Vs >= Vth_vr
 Vth_va : Threshold voltage; the system does not pull power from aux battery 
          if Va < Vth_va
 Vth_vd : Threshold voltage; the system will go to deep sleep if there is no
          power supply from Vs and Va < Vth_vd
*/


/*

  Vs  ---+--- SW_VR1 ------ DC/DC ------ SW_VR2 ----+--------------- AUX BAT
         |                                          |
         |                                          +--- SW_BAT --+- POWER OUT
         |                                                        |
         +-----------------------------------------------SW_VS  --+



OC1A -- CC control
OC1B -- CV control

                      |\
 ICHARGE_SENS    ---- |+\            ^
 OC1A(filtered)  ---- |- > ----+     |
                      | /      |     <
                      |/       |     >
                               |     < 10k
                      |\       |     >
 VA_SENS_ADC     ---- |+\      |     |
 OC1B(filtered)  ---- |- > ----+-----+------[inverter]---> reglator feedback
                      | /
                      |/

reglator feedback >= 0.8V   decrease output
reglator feedback <  0.8V   increase output

reglator feedback = !( ICHARGE_SENS < OC1A || VA_SENS_ADC < OC1B)

*/


/* ポートマップの定義 */
MAP_START(pmap)
		MAP_MATCH( B, 0, DIDR0, -1 );
		MAP_MATCH( B, 1, DIDR0, -1 );
		MAP_MATCH( B, 2, DIDR0, -1 );
		MAP_MATCH( B, 3, DIDR0, -1 );
		MAP_MATCH( B, 4, DIDR0, -1 );
		MAP_MATCH( B, 5, DIDR0, -1 );
		MAP_MATCH( B, 6, DIDR0, -1 );
		MAP_MATCH( B, 7, DIDR0, -1 );

		MAP_MATCH( C, 0, DIDR0, ADC0D );
		MAP_MATCH( C, 1, DIDR0, ADC1D );
		MAP_MATCH( C, 2, DIDR0, ADC2D );
		MAP_MATCH( C, 3, DIDR0, ADC3D );
		MAP_MATCH( C, 4, DIDR0, ADC4D );
		MAP_MATCH( C, 5, DIDR0, ADC5D );
		MAP_MATCH( C, 6, DIDR0, -1 );
//		MAP_MATCH( C, 7, DIDR0, -1 );

		MAP_MATCH( D, 0, DIDR0, -1 );
		MAP_MATCH( D, 1, DIDR0, -1 );
		MAP_MATCH( D, 2, DIDR0, -1 );
		MAP_MATCH( D, 3, DIDR0, -1 );
		MAP_MATCH( D, 4, DIDR0, -1 );
		MAP_MATCH( D, 5, DIDR0, -1 );
		MAP_MATCH( D, 6, DIDR0, -1 );
		MAP_MATCH( D, 7, DIDR0, -1 );
MAP_END


static void port_setup()
{


pmap('C', 6, MDO0); // 1 PC6 (PCINT14/RESET)      RESET
pmap('D', 0, MDO0); // 2 PD0 (PCINT16/RXD)        RX
pmap('D', 1, MDO0); // 3 PD1 (PCINT17/TXD)        TX
pmap('D', 2, MDO0); // 4 PD2 (PCINT18/INT0)
pmap('D', 3, MDO0); // 5 PD3 (PCINT19/OC2B/INT1)
pmap('D', 4, MDO0); // 6 PD4 (PCINT20/XCK/T0)
                    // 7 VCC
                    // 8 GND
pmap('B', 6, MDO0); // 9 PB6 (PCINT6/XTAL1/TOSC1)     
pmap('B', 7, MDO0); //10 PB7 (PCINT7/XTAL2/TOSC2)
#define SW_VR1 'D', 5
pmap('D', 5, MDO0); //11 PD5 (PCINT21/OC0B/T1)    SW_VR1
#define SW_VR2 'D', 6
pmap('D', 6, MDO0); //12 PD6 (PCINT22/OC0A/AIN0)  SW_VR2
#define SW_BAT 'D', 7
pmap('D', 7, MDO0); //13 PD7 (PCINT23/AIN1)       SW_BAT
#define SW_VS  'B', 0
pmap('B', 0, MDO0); //14 PB0 (PCINT0/CLKO/ICP1)   SW_VS
#define DEEP_SLEEP_EN 'B', 1
pmap('B', 1, MDO1); //15 PB1 (OC1A/PCINT1)        DEEP_SLEEP_EN
#define VA_SENS_EN 'B', 2
pmap('B', 2, MDO0); //16 PB2 (SS/OC1B/PCINT2)     
pmap('B', 3, MDO0); //17 PB3 (MOSI/OC2A/PCINT3)   
pmap('B', 4, MDO0); //18 PB4 (MISO/PCINT4)        
pmap('B', 5, MDO0); //19 PB5 (SCK/PCINT5)         
                    //20 AVCC
                    //21 AREF
                    //22 GND
#define ADC_CH_VS_SENSE 0
pmap('C', 0, MAI);  //23 PC0 (ADC0/PCINT8)        VS Voltage sense
#define ADC_CH_VA_SENSE 0
pmap('C', 1, MAI);  //24 PC1 (ADC1/PCINT9)        VA Voltage sense
pmap('C', 2, MDO0); //25 PC2 (ADC2/PCINT10)       
pmap('C', 3, MDO0); //26 PC3 (ADC3/PCINT11)       
pmap('C', 4, MDO0); //27 PC4 (ADC4/SDA/PCINT12)   
pmap('C', 5, MDO0); //28 PC5 (ADC5/SCL/PCINT13)   

}



//--------------------------------------------------------------------
// global params

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
// I/O wrapup

static void set_sw_vr1(uint8_t b) { if(b) pmap(SW_VR1, MDO1); else pmap(SW_VR1, MDO0); }
static void set_sw_vr2(uint8_t b) { if(b) pmap(SW_VR2, MDO1); else pmap(SW_VR2, MDO0); }
static void set_sw_bat(uint8_t b) { if(b) pmap(SW_BAT, MDO1); else pmap(SW_BAT, MDO0); }
static void set_sw_vs (uint8_t b) { if(b) pmap(SW_VS , MDO1); else pmap(SW_VS , MDO0); }

static void  __attribute__((noreturn)) enter_deep_sleep() 
{
	cli();
	set_sw_vr1(0);
	set_sw_vr2(0);
	set_sw_bat(0);
	set_sw_vs(0);
	_delay_ms(100);
	pmap(DEEP_SLEEP_EN , MDO0);
	for(;;) ; // infinite loop
}

static uint8_t battery_charging = 0;
static uint8_t battery_measureing = 0;

static void set_vr1_vr2()
{
	set_sw_vr1(battery_charging);
	set_sw_vr2(battery_charging && !battery_measureing);
}

static void set_battery_charging(uint8_t b)
{
	battery_charging = b;
	set_vr1_vr2();
}
static void set_battery_measureing(uint8_t b)
{
	battery_measureing = b;
	set_vr1_vr2();
}

static void use_power_from_vs()
{
	set_sw_vs(1);
	set_sw_bat(0);
}

static void use_power_from_bat()
{
	set_sw_bat(1);
	set_sw_vs(0);
}

static void enable_va_sens()
{
	pmap(VA_SENS_EN, MDO1);
	_delay_us(50); // to charge sample-and-hold caps
}

static void disable_va_sens()
{
	pmap(VA_SENS_EN, MDO0);
}

//--------------------------------------------------------------------
// タイマー関連

static void timer0_setup()
{
	/* Normal mode, prescaler = clkio/64, 1220.703125 Hz interrupt */
	SETR(TCCR0A, ~COM0A1, ~COM0A0, ~COM0B1, ~COM0B0, ~WGM01, ~WGM00);
	SETR(TCCR0B, ~FOC0A, ~FOC0B, ~WGM02, ~CS02, CS01, CS00);
	SETR(TIMSK0, TOIE0); // enable overflow interrupt
}

static volatile uint8_t flag_start_va_measure = 1;
static volatile uint8_t flag_end_va_measure = 0;
#define TIMER_1SEC_TICK 1221

ISR(TIMER0_OVF_vect)
{
	static uint16_t _timer;
	++ _timer;
	if(_timer == 1)
	{
		flag_start_va_measure = 1;
	} 
	else if(_timer == TIMER_1SEC_TICK)
	{
		flag_end_va_measure = 1;
	}
	else if(_timer == TIMER_1SEC_TICK* 10)
	{
		_timer = 0;
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
static float vs = 0.0f;
static float va = 0.0f;

static void poll_power()
{
	vs = get_voltage(ADC_CH_VS_SENSE);

	// va の測定は、一旦 vr2 を切って1秒待ってから測定する
	// そのため頻繁に測定はできない
	if(flag_start_va_measure)
	{
		flag_start_va_measure = 0;
		set_battery_measureing(1);
	}

	if(flag_end_va_measure)
	{
		flag_end_va_measure = 0;
		enable_va_sens();
		va = get_voltage(ADC_CH_VA_SENSE);
		disable_va_sens();
		set_battery_measureing(0);
	}

	// 充電を開始すべきか？
	uint8_t charging = vs >= eeprom.vth_vs;
	uint8_t input_active = vs >= eeprom.vth_vr;
	set_battery_charging(charging);

	// 出力はどちらから採るべきか？
	if(va != 0.0f && va >= eeprom.vth_va && !charging)
	{
		// va が測定完了していて、かつ
		// va が Vth_va 以上で、かつ
		// 充電中でない
		use_power_from_bat();
	}
	else
	{
		// それ以外
		if(input_active)
		{
			// 入力が検出されていれば
			// vs からとる
			use_power_from_vs();
		}
		else
		{
			// そうでなければ

			// ディープスリープに移行すべきか？
			// Vth_vd を va が下回っていればシャットダウンし、
			// ディープスリープに移行する
			if(va != 0.0f && va < eeprom.vth_vd)
				enter_deep_sleep();

			// バッテリーから得る
			use_power_from_bat();
		}
	}
}
//--------------------------------------------------------------------

int main(void) __attribute__((noreturn));
int main(void)
{
	/* disable WDT */
	port_setup();
	wdt_disable();

	// wait for power stabilization
	_delay_ms(50);

	// setup
	init_serial();
	eeprom_setup();
	adc_setup();
	timer0_setup();

	// banner
	debug_send_P(PSTR("\n\nAux power controller\n\n"));


	// enter infinite loop
	sei();
	for(;;)
	{
		poll_serial();
		poll_power();
	}

}

/* -------------------------------------------------------------------- */

