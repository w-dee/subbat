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
#include "TinyI2CMaster.h"
#include "../sub/main_sub_common.h"
#include "bme280.h"

static void  __attribute__((noreturn)) enter_deep_sleep();
static float get_nct75_value();
typedef enum tag_device_existence_status
{
	deNone, // non-existent
	deFound, // found but not yet working
	deWorking // found and working
} device_existence_status_t;
static uint8_t bme280_exist = deNone;
static struct bme280_data bme280_last_result;

uint8_t nct75_exist = deNone;
static float nct75_temperature; // measuread temperature

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

  Vs  ---+--- SW_VR ------- DC/DC ------ SW_CHG ----+--------------- AUX BAT
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


----------++++++++++----------++++++++++
SYS->OUT 10.0A | AMB 20.1C 54% 1023h
C-CHARGE 12.0V 10.3A | BAT 11.0V 19C

C: constant current charge
V: constant voltage charge
F: floating charge
*/

#define CURRENT_SENS_REGISTER 0.01 // current sens register value
#define CURRENT_SENS_GAIN ((4700.0 + 680.0) / 120.0)

static float current_to_reading_voltage(float current)
{
	return current * (CURRENT_SENS_REGISTER * CURRENT_SENS_GAIN);
}

#define VOLTAGE_SENSE_GAIN (10.0 / (56.0 + 10.0 + 10.0))

static float voltage_to_reading_voltage(float voltage)
{
	return voltage * VOLTAGE_SENSE_GAIN;
}

#define VR_FB_VOLTAGE_GAIN (10.0 / (56.0 + 10.0 + 10.0)) // for comparator
#define VR_FB_CURRENT_PWM_GAIN (1.0) // for pwm LPF

#define VREF_VOLTAGE 4.7405

#define VDD_SENS_GAIN (10.0 / (10.0 + 22.0))

static float voltage_to_voltage_comparator_voltage(float voltage)
{
	return voltage * VR_FB_VOLTAGE_GAIN;
}

#define BATTERY_MIN_VOLTAGE 0.0 //9.0 // battery voltage (no matter the battery is loaded/charging/open) below this voltage
								// immediately shuts down the system

#define BATTERY_CHARGE_END_CURRENT 0.03 // battery charge finish current

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
		MAP_MATCH( C, 6, DIDR0, 6 );
		MAP_MATCH( C, 7, DIDR0, 7 );

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
pmap('D', 0, MDI);  // 2 PD0 (PCINT16/RXD)        RX
pmap('D', 1, MDO0); // 3 PD1 (PCINT17/TXD)        TX
pmap('D', 2, MDI);  // 4 PD2 (PCINT18/INT0)       SCL
#define OUTPUT_CHARGE 'D', 3
pmap('D', 3, MDO0); // 5 PD3 (PCINT19/OC2B/INT1)  OUTPUT_CHARGE
#define SW_BAT 'D', 4
pmap('D', 4, MDO0); // 6 PD4 (PCINT20/XCK/T0)     SW_BAT
                    // 7 VCC
                    // 8 GND
#define DEEP_SLEEP_EN 'B', 6
pmap('B', 6, MDO1); // 9 PB6 (PCINT6/XTAL1/TOSC1) /DEEP_SLEEP_EN
#define SW_CHG 'B', 7
pmap('B', 7, MDI);  //10 PB7 (PCINT7/XTAL2/TOSC2) SW_CHG
#define SW_VR 'D', 5
pmap('D', 5, MDO0); //11 PD5 (PCINT21/OC0B/T1)    SW_VR
pmap('D', 6, MDI);  //12 PD6 (PCINT22/OC0A/AIN0)  SDA
pmap('D', 7, MDI);  //13 PD7 (PCINT23/AIN1)       DIGITAL_IN0
#define SW_VS  'B', 0
pmap('B', 0, MDO0); //14 PB0 (PCINT0/CLKO/ICP1)   SW_VS
pmap('B', 1, MDO0); //15 PB1 (OC1A/PCINT1)        CC_CTRL
pmap('B', 2, MDO0); //16 PB2 (SS/OC1B/PCINT2)     CV_CTRL
pmap('B', 3, MDO0); //17 PB3 (MOSI/OC2A/PCINT3)   MOSI
pmap('B', 4, MDO0); //18 PB4 (MISO/PCINT4)        MISO
pmap('B', 5, MDO0); //19 PB5 (SCK/PCINT5)         SCK
                    //20 AVCC
                    //21 AREF
                    //22 GND
#define ADC_CH_VS_SENSE 0
pmap('C', 0, MAI);  //23 PC0 (ADC0/PCINT8)        VS Voltage sense
#define ADC_CH_VA_SENSE 1
pmap('C', 1, MAI);  //24 PC1 (ADC1/PCINT9)        VA Voltage sense
#define ADC_CH_IOUT_SENSE 2
pmap('C', 2, MAI);  //25 PC2 (ADC2/PCINT10)       IOUT sense
#define ADC_CH_ISYS_SENSE 3
pmap('C', 3, MAI);  //26 PC3 (ADC3/PCINT11)       ISYS sense
#define ADC_CH_VR_SENSE 4
pmap('C', 4, MAI);  //27 PC4 (ADC4/SDA/PCINT12)   VR output voltage sens
#define ADC_CH_VDD_SENSE 5
pmap('C', 5, MAI);  //28 PC5 (ADC5/SCL/PCINT13)   
#define ADC_CH_IUSB_SENSE 6
pmap('C', 6, MAI);  //28 PC6 
#define ADC_CH_ICHARGE_SENSE 7
pmap('C', 7, MAI);  //28 PC7

}


void do_panic(uint8_t reason);

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
/**
 * 数値を送信する
 */
static void debug_send_f(float f)
{
	char buf[32];
	sprintf_P(buf, PSTR("%.5f"), f);
	debug_send(buf);
}
/**
 * send hex string
 */
static void debug_send_h(long int n, uint8_t width)
{
	char buf[16];
	ltoa(n, buf, 16);
	uint8_t l = strlen(buf);
	if(width - l > 0)
	{
		uint8_t c = width - l;
		while(c--) send('0');
	}

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


void do_panic(uint8_t reason)
{
	debug_send_P(PSTR("\r\n\r\n!!!!! PANIC !!!!!\r\n\r\npanic code = "));
	debug_send_n(reason);
	debug_send_P(PSTR("\r\n\r\n"));
	enter_deep_sleep();
}

//--------------------------------------------------------------------
// EEPROM
static eeprom_t eeprom=	{
		.size = sizeof(eeprom),
		.vth_vd = 10.0,
		.vth_vs = 13.2,
		.vth_vr = 9.0,
		.chg_cc = 2.0,
		.chg_cv = 13.0,
		.chg_float = 12.4,
		.float_cc = 0.2,
		.coeff_cv = -3*6,
		.coeff_float = -5*6,
		.chg_restart = 12.2,
		.limit_temp_h = 55,
		.limit_temp_l = -6,
		.dark_ambient = 120,
		.pwm_light = 50,
		.pwm_dark = 12,
		.lcd_contrast = 20,
	};

static void eeprom_copy()
{
	uint8_t eeprom_size;
	uint8_t buffer_size;
	uint8_t buf[32];

	// eeprom is located on sub processor;
	// eeprom content must be copied from the slave using I2C.
	buf[0] = TWI_CMD_QUERY_EEPROM_SIZE;
	I2C_WriteData(I2C_SUB_SLAVE_ADDR, buf, 1, 1); // TODO: error check

	I2C_ReadData(I2C_SUB_SLAVE_ADDR, buf, 2); // TODO: error check
	eeprom_size = buf[0];
	buffer_size = buf[1];

	if(eeprom_size != sizeof(eeprom))
	{
		do_panic(2); // eeprom size mismatch
	}
	if(buffer_size > sizeof(buf))
	{
		do_panic(3); // buffer size too large
	}

	// start block transfer using TWI_CMD_COPY_EEPROM command
	uint8_t remain = eeprom_size;
	uint8_t index = 0;
	while(remain > 0)
	{
		uint8_t one_size = remain > buffer_size ? buffer_size : remain;
		buf[0] = TWI_CMD_COPY_EEPROM;
		buf[1] = index;
		buf[2] = one_size;
		I2C_WriteData(I2C_SUB_SLAVE_ADDR, buf, 3, 1); // TODO: error check

		I2C_ReadData(I2C_SUB_SLAVE_ADDR, (uint8_t*)(&eeprom) + index, one_size);  // TODO: error check

		remain -= one_size;
	}

}


//--------------------------------------------------------------------
// タイマー関連

static void timer0_setup()
{
	/* Fast PWM mode, prescaler = clkio/64, 1k Hz interrupt */
	SETR(TCCR0A, ~COM0A1, ~COM0A0, ~COM0B1, ~COM0B0, WGM01, WGM00);
	SETR(TCCR0B, ~FOC0A, ~FOC0B, WGM02, ~CS02, CS01, CS00);
	SETR(TIMSK0, TOIE0); // enable overflow interrupt
	OCR0A = 124;
}

static volatile uint32_t __do_not_touch_this___tick;

ISR(TIMER0_OVF_vect)
{
	++__do_not_touch_this___tick;
}

static uint32_t get_tick()
{
	uint32_t t;
	cli();
	t = __do_not_touch_this___tick;
	sei();
	return t;
}

static uint8_t tick_elapsed(uint32_t tick)
{
	int32_t diff = get_tick() - tick;
	return !(diff < 0);
}

// execute corresponding block intervally by specified interval time
#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)
#define _INTERVAL(ms, LINE) static uint32_t TOKENPASTE2(tick,LINE); static uint8_t TOKENPASTE2(tick_init,LINE); \
	if(!TOKENPASTE2(tick_init,LINE)) TOKENPASTE2(tick,LINE) = get_tick() + (-1), TOKENPASTE2(tick_init,LINE) = 1; \
	uint8_t TOKENPASTE2(do,LINE) = 0; \
	if(tick_elapsed(TOKENPASTE2(tick,LINE))) TOKENPASTE2(tick,LINE) += (ms), TOKENPASTE2(do,LINE) = 1; \
	if(TOKENPASTE2(do,LINE)) 
#define INTERVAL(ms) _INTERVAL(ms, __LINE__)

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
		AREF, Internal V ref turned off
		result in left adjusted
	*/
	SETR(ADMUX, ~REFS1, ~REFS0, ~ADLAR, ~MUX3, ~MUX2, ~MUX1, ~MUX0);

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
	float res = v * ((1.0 / 1024.0) * VREF_VOLTAGE / VOLTAGE_SENSE_GAIN); 
	return res;
}

static float get_voltage_avg4(uint8_t ch)
{
	uint16_t v = get_adc(ch) + get_adc(ch) + get_adc(ch) + get_adc(ch);
	float res = v * ((1.0 / 1024.0) * VREF_VOLTAGE / VOLTAGE_SENSE_GAIN / 4.0); 
	return res;
}

static float get_current(uint8_t ch)
{
	uint16_t v = get_adc(ch);
	float res = v * ((VREF_VOLTAGE / 1024.0) * (1.0 / CURRENT_SENS_GAIN) * (1.0 / CURRENT_SENS_REGISTER));
	return res;
}

static float get_current_avg4(uint8_t ch)
{
	uint16_t v = get_adc(ch) + get_adc(ch) + get_adc(ch) + get_adc(ch);
	float res = v * ((VREF_VOLTAGE / 1024.0 / 4.0) * (1.0 / CURRENT_SENS_GAIN) * (1.0 / CURRENT_SENS_REGISTER));
	return res;
}

//--------------------------------------------------------------------
// CC/CV control

static void timer1_setup()
{

	/* timer1とWGMを初期化 */
	/*
		Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
		WGM mode = 1110 (Fast PWM, TOP = ICR1)
		clock source = CLKio
	*/
	SETR(TCCR1A, COM1A1, ~COM1A0, COM1B1, ~COM1B0, WGM11, ~WGM10);
	SETR(TCCR1B, WGM13, WGM12, ~FOC1A, ~FOC1B, ~CS12, ~CS11, CS10);
	SETR(DDRB, 1, 2); // PB1, PB2 = output
	SETR(PORTB, ~1, ~2); 
	ICR1 = 1023;
	OCR1A = 0;
	OCR1B = 0;
}

typedef enum tag_vr_status_t
{
	vr_off, // vr is off
	vr_wait, // wait for battery voltage get lower than the target voltage
	vr_calib, // in voltage calibration
	vr_stabilization, // wait for current limit stabilization
	vr_run, // vr is running
} vr_status_t;
uint8_t vr_status = vr_off;

static float vr_target_current; // target current
#define VR_VOLT_RIPPLE_RANGE 0.04 // in [V]
#define VR_CURRENT_RIPPLE_RANGE 0.02 // in [A]
static uint32_t vr_stabled_tick;
static float vr_target_voltage; // target voltage
typedef uint32_t pwm_shifted_type;
static pwm_shifted_type computed_voltage_pwm_shifted; // computed voltage pwm value
#define OCR_SHIFT 6
#define VR_CALIBRARTION_MAX_PWM_DIFFERENCE (22<<OCR_SHIFT) // approx. +/- 0.1 V
static float mcu_voltage; // MCU Vdd voltage
static uint8_t vr_connected_to_battery = 0;
static pwm_shifted_type OCR1A_shadow_shifted = 0;
static pwm_shifted_type OCR1B_shadow_shifted = 0;
static float vr_now_current;// now output current from VR
static float battery_raw_voltage = -1.0; // battery raw voltage (negative value for not-yet-measured)


// measure mcu voltage
static void measure_mcu_voltage()
{
	uint16_t vdd = 0;
	for(uint8_t i = 0; i < 16; ++i)
		vdd += get_adc(ADC_CH_VDD_SENSE);

	mcu_voltage = ((float)vdd / (16.0f * 1024.0f)) / VDD_SENS_GAIN * VREF_VOLTAGE;

	debug_send("\r\nMCU voltage :");
	debug_send_f(mcu_voltage);
	debug_send("\r\n");
}

#define VR_VOLTAGE_STABILIZE_TIME 300
#define VR_CURRENT_STABILIZE_TIME 300

static void set_target_current(float v)
{
	if(mcu_voltage == 0.0) measure_mcu_voltage();

	uint16_t pwm_width = current_to_reading_voltage(v) /
		(mcu_voltage * VR_FB_CURRENT_PWM_GAIN) * (ICR1 + 1);
	vr_target_current = v;

	debug_send("\r\nset_target_current:"); debug_send_f(vr_target_current); debug_send("  ");
	debug_send("width:"); debug_send_n(pwm_width); debug_send("  ");
	debug_send("ICR1:"); debug_send_n(ICR1); debug_send("  ");
	debug_send("\r\n");

	if(pwm_width >= ICR1) do_panic(4); // wtf ? out of range

	OCR1A = pwm_width;
	OCR1A_shadow_shifted = (pwm_shifted_type)pwm_width << OCR_SHIFT;

	if(vr_status == vr_run)
	{
		// wait for stabilization
		vr_stabled_tick = get_tick() + VR_CURRENT_STABILIZE_TIME;
		vr_status = vr_stabilization;
	}
}

static void set_target_current_if_changed(float v)
{
	if(vr_target_current != v)
		set_target_current(v);
}


static void set_calibration_current()
{
	// set current limit to no limit, to calibrate voltage
	OCR1A = ICR1 - 1;
	OCR1A_shadow_shifted = OCR1A << OCR_SHIFT;
}

static void start_vr();
static void set_target_voltage(float v)
{
	if(mcu_voltage == 0.0) measure_mcu_voltage();

	uint16_t pwm_width = voltage_to_voltage_comparator_voltage(v) / mcu_voltage * (ICR1 + 1);
	vr_target_voltage = v;

	debug_send("\r\nset_target_voltage:"); debug_send_f(vr_target_voltage); debug_send("  ");
	debug_send("width:"); debug_send_n(pwm_width); debug_send("  ");
	debug_send("ICR1:"); debug_send_n(ICR1); debug_send("  ");
	debug_send("\r\n");

	if(pwm_width >= ICR1) do_panic(5); // wtf ? out of range

	OCR1B = pwm_width;
	OCR1B_shadow_shifted = (pwm_shifted_type)pwm_width << OCR_SHIFT;
	computed_voltage_pwm_shifted = OCR1B_shadow_shifted;

	if(vr_status == vr_run)
	{
		// reset VR
		start_vr();
	}
}

static void set_target_voltage_if_changed(float v)
{
	if(vr_target_voltage != v)
		set_target_voltage(v);
}

static void start_vr()
{
	vr_connected_to_battery = 0;
	pmap(SW_CHG, MDO0); // VR to battery switch off
	pmap(SW_VR , MDO1); // VR run signal = on
	vr_status = vr_wait; // first, do calibration of voltage
	set_calibration_current(); // no current limit
	set_target_voltage(vr_target_voltage); // reset target voltage
	vr_stabled_tick = get_tick() + VR_VOLTAGE_STABILIZE_TIME;
}

static void set_use_battery_voltage_for_mcu(uint8_t t);

static void connect_vr_to_battery()
{
	// check whether VR output is above battery raw voltage
	if(vr_connected_to_battery)
	{
		vr_connected_to_battery = 0;
		pmap(SW_CHG, MDO0); // disconnect
		_delay_us(600); // wait for a while
	}


	// connect VR
	vr_connected_to_battery = 1;
	pmap(SW_CHG, MDO1); // VR to battery switch on
	set_use_battery_voltage_for_mcu(0); // not use battery for MCU power
}

static void stop_vr()
{
	vr_connected_to_battery = 0;
	pmap(SW_CHG, MDO0); // VR to battery switch off
	pmap(SW_VR , MDO0); // VR run signal = off
	vr_status = vr_off;
	set_use_battery_voltage_for_mcu(1); // use also battery for MCU power
}

static uint8_t is_vr_running_well() { return vr_status == vr_run; }

static uint8_t vr_disconnect_temporarily()
{
	if(vr_connected_to_battery)
	{
		pmap(SW_CHG, MDO0); // VR to battery switch off
		set_use_battery_voltage_for_mcu(0); // also disconnect mcu power from battery
	}
	return vr_connected_to_battery;
}

static void vr_recover_disconnected_temporarily()
{
	if(vr_connected_to_battery)
	{
		set_use_battery_voltage_for_mcu(1); // use also battery for MCU power
		pmap(SW_CHG, MDO1); // restore the state
	}
}

static void debug_sp()
{
	debug_send_P(PSTR("  "));
}

static void dump_voltages(float now_voltage)
{
	float c = get_current_avg4(ADC_CH_ICHARGE_SENSE);
	uint8_t log = 0;

	INTERVAL(2000)
	{
		// do logging for graphing
		log = 1;
		debug_send_P(PSTR("\r\n"));
		debug_send_P(PSTR("Log: "));
	}

	debug_send_P(PSTR("tick: "));     debug_send_n(get_tick());         debug_sp();
	debug_send_P(PSTR("state: "));    debug_send_n(vr_status);          debug_sp();
	debug_send_P(PSTR("v_targ: "));   debug_send_f(vr_target_voltage);  debug_sp();
	debug_send_P(PSTR("v_now: "));    debug_send_f(now_voltage);        debug_sp();
	debug_send_P(PSTR("c_targ: "));   debug_send_f(vr_target_current);  debug_sp();
	debug_send_P(PSTR("c_now: "));    debug_send_f(c);                  debug_sp();
	debug_send_P(PSTR("bat_temp: ")); debug_send_f(nct75_temperature);  debug_sp();
	debug_send("pwm:"); debug_send_n(OCR1B);debug_send("  ");

	if(log)
		debug_send_P(PSTR("\r\n"));
	else
		debug_send_P(PSTR("\r"));
}

static void calibrate_voltate(float now_voltage)
{
	// do voltage calibration
	pwm_shifted_type lower = computed_voltage_pwm_shifted > VR_CALIBRARTION_MAX_PWM_DIFFERENCE ?
		computed_voltage_pwm_shifted - VR_CALIBRARTION_MAX_PWM_DIFFERENCE : 0;
	pwm_shifted_type upper = computed_voltage_pwm_shifted + VR_CALIBRARTION_MAX_PWM_DIFFERENCE;
	if(upper >= (ICR1<<OCR_SHIFT)) upper = (ICR1<<OCR_SHIFT);

	pwm_shifted_type width = OCR1B_shadow_shifted;

	if(now_voltage < vr_target_voltage)
	{
		// voltage too low
		if(width < upper) ++width;
	}
	else
	{
		// voltage too high
		if(width > lower) --width;		
	}

	OCR1B_shadow_shifted = width;
	OCR1B = width >> OCR_SHIFT;	
	dump_voltages(now_voltage);
}


static void poll_vr()
{
	INTERVAL(10000)
	{
		measure_mcu_voltage();
	}

	// voltage regulater needs some startup time after 'RUN' has issued...
	INTERVAL(10)
	{
		float voltage;
		float current =  get_current_avg4(ADC_CH_ICHARGE_SENSE);
		vr_now_current = current;
		switch(vr_status)
		{
		case vr_off: // off state
			break; // do nothing

		case vr_wait: // wait battery voltage gets lower than target
			//if(battery_raw_voltage <= 0) break; // raw voltage not yet measured
			if(vr_target_voltage >= battery_raw_voltage)
			{
				vr_status = vr_calib; // ok to calibrate
			}
			break;

		case vr_calib: // calibration state
			voltage = get_voltage_avg4(ADC_CH_VR_SENSE);
			if(voltage < vr_target_voltage - VR_VOLT_RIPPLE_RANGE ||
				voltage > vr_target_voltage + VR_VOLT_RIPPLE_RANGE)
			{
				// voltage out of range
				calibrate_voltate(voltage);
				vr_stabled_tick = get_tick() + VR_VOLTAGE_STABILIZE_TIME;
			}
			else
			{
				if(tick_elapsed(vr_stabled_tick))
				{
					// voltage well stabilized;
					// set current limit
					set_target_current(vr_target_current);
					// step toward next phase
					vr_status = vr_stabilization;
					vr_stabled_tick = get_tick() + VR_CURRENT_STABILIZE_TIME;
				}
			}
			break;

		case vr_stabilization:
			// wait for vr_stabled_tick and wait again,
			// that the target voltage get higher than the battery voltage
		{
			float target = get_voltage_avg4(ADC_CH_VR_SENSE);
			float bat    = get_voltage_avg4(ADC_CH_VA_SENSE);
			calibrate_voltate(target);
			if(tick_elapsed(vr_stabled_tick) &&
				target >= bat)
			{
				// connect voltage regulator to the battery
				connect_vr_to_battery();
				// step toward next phase
				vr_status = vr_run;
			}
			break;
		}

		case vr_run:
			// voltage regulator running
			// check whether the mode is CC mode or CV mode
			voltage = get_voltage_avg4(ADC_CH_VA_SENSE);
			if(voltage < vr_target_voltage - VR_VOLT_RIPPLE_RANGE)
			{
				// voltage lower than expected:
				if(current < vr_target_current - VR_CURRENT_RIPPLE_RANGE)
				{
					// current seems below limit; CV mode
					calibrate_voltate(voltage);
				}
				else
				{
					// current seems to hit the limit;
					// seems to be in current limitting mode (CC mode)
					dump_voltages(voltage);
				}
			}
			else
			{
				// seems to be in constant voltage mode
				calibrate_voltate(voltage);
			}
			break;

		default:
			;
		}
	}
}

//--------------------------------------------------------------------
// control of switch, and more

static void set_sw_bat(uint8_t b) { if(b) pmap(SW_BAT, MDO1); else pmap(SW_BAT, MDO0); }
static void set_sw_vs (uint8_t b) { if(b) pmap(SW_VS , MDO1); else pmap(SW_VS , MDO0); }

static uint8_t output_charge_capacitor_connected = 0;

static void set_output_charge_capacitor_connected (uint8_t b)
{
	if(b && !output_charge_capacitor_connected)
	{
		pmap(OUTPUT_CHARGE , MDO1);
		output_charge_capacitor_connected = 1;
		debug_send_P(PSTR("\r\nOutput capacitor connected\r\n"));
	}
	else if(!b && output_charge_capacitor_connected)
	{
		pmap(OUTPUT_CHARGE , MDO0);
		output_charge_capacitor_connected = 0;
		debug_send_P(PSTR("\r\nOutput capacitor disconnected\r\n"));
	}
}

static uint32_t output_charge_capacitor_connect_tick;
static uint8_t output_charge_capacitor_connect_triggered_state = 0;
static void trigger_delayed_output_charge()
{
	if(output_charge_capacitor_connected) return ; // already connected
	if(output_charge_capacitor_connect_triggered_state) return ; // already trigerred
	output_charge_capacitor_connect_tick = get_tick() + 1000; // after 1s, connect capacitor
	output_charge_capacitor_connect_triggered_state = 1;
}

static void poll_output_charge_capacitor()
{
	// the output capacitor is so large that may induces in-rush current.
	// the output capacitor is connected to output terminal through
	// 10R register in parallel with low-R(on) MOS FET.
	// first 1000ms the register restricts the in-rush current, then
	// MOS FET makes, then FET conducts and connects the capacitor to the terminal
	// completely.
	if(output_charge_capacitor_connect_triggered_state == 1)
	{
		if(tick_elapsed(output_charge_capacitor_connect_tick))
		{
			output_charge_capacitor_connect_triggered_state = 2;
			set_output_charge_capacitor_connected(1);
		}
	}
}

static uint8_t output_charge_capacitor_ok() { return output_charge_capacitor_connect_triggered_state == 2; }


typedef enum _output_source
{
	osNone, // none output
	osSystem, // from system
	osBattery, // from battery
} output_source_t;


static uint8_t output_source = osNone;

static void use_power_from_vs()
{
	set_sw_bat(0);
	set_sw_vs(1);
	output_source = osSystem;
	trigger_delayed_output_charge();
}

static void use_power_from_bat()
{
	set_sw_vs(0);
	set_sw_bat(1);
	output_source = osBattery;
	trigger_delayed_output_charge();
}

static void disconnect_output()
{
	stop_vr();
	set_sw_bat(0);
	set_sw_vs(0);
	output_source = osNone;
	set_output_charge_capacitor_connected(0); // disconnect output capacitor
}

static void set_use_battery_voltage_for_mcu(uint8_t t)
{
	// despite of the name, DEEP_SLEEP_EN is a switch
	// controlling wheter mcu power pulling from the battery or not.
	// no use of battery and no voltage from system causes
	// deep shutdown of MCU untill the system voltage rises again.
	if(t)
		pmap(DEEP_SLEEP_EN , MDO1); // yes, use
	else
		pmap(DEEP_SLEEP_EN , MDO1); // no, not use.
}
static void set_charge_state(uint8_t v);
static void  __attribute__((noreturn)) enter_deep_sleep() 
{
	cli();
	set_charge_state(0);
	disconnect_output();
	_delay_ms(100);
	set_use_battery_voltage_for_mcu(0);
	for(;;) ; // infinite loop
}

void poll_battery_raw_voltage()
{
	// measure battery voltage
	float v = get_voltage(ADC_CH_VA_SENSE);
	if(v < BATTERY_MIN_VOLTAGE)
	{
		do_panic(6); // system shutdown; battery is not connected or short-circuited
	}

	// to measure battery raw (unloaded) voltage, 
	// we need to disconnect the battery very short-time (500us).
	// this is only able while output capacitor is active.

	INTERVAL(5000)
	{
		debug_send_P(PSTR("\r\nOutput cap: cap_state:"));
		debug_send_n(output_charge_capacitor_connect_triggered_state);
		debug_send_P(PSTR("  "));
		if(output_charge_capacitor_ok())
		{
			// output charge capacitor is connected properly

			// disconnect power lines
			uint8_t source_disconnected = 0;
			if(output_source == osBattery)
			{
				// output power is taken from the battery;
				// disconnect it
				set_sw_bat(0);
				source_disconnected = 1;
			}

			// voltage_regulator is running, disconnect it from the battery
			vr_disconnect_temporarily();

			// wait for a short time
			_delay_us(500);

			// measure
			battery_raw_voltage = get_voltage_avg4(ADC_CH_VA_SENSE);

			// reconnect power lines
			if(source_disconnected)
			{
				set_sw_bat(1);
			}

			// reconnect battery and wait for a short time
			vr_recover_disconnected_temporarily();
			_delay_us(100);

			debug_send("battery raw:"); debug_send_n(battery_raw_voltage);
		}
		debug_send("\r\n");
	}
}

//--------------------------------------------------------------------
// battery charge logic state machine
static void set_battery_cc_cv_param()
{
	float v = eeprom.chg_cv + (nct75_temperature - 25.0) * eeprom.coeff_cv * 0.001;
	float c = eeprom.chg_cc;
	set_target_voltage_if_changed(v);
	set_target_current_if_changed(c);
	debug_send_P(PSTR("\r\nCharge set voltage:")); debug_send_f(v); debug_send_P(PSTR("\r\n"));
	debug_send_P(PSTR("\r\nCharge set current:")); debug_send_f(c); debug_send_P(PSTR("\r\n"));
}

static void set_battery_float_param()
{
	float v = eeprom.chg_float + (nct75_temperature - 25.0) * eeprom.coeff_float * 0.001;
	float c = eeprom.float_cc;
	set_target_voltage_if_changed(v);
	set_target_current_if_changed(c);
	debug_send_P(PSTR("\r\nFloating set voltage:")); debug_send_f(v); debug_send_P(PSTR("\r\n"));
	debug_send_P(PSTR("\r\nFloating set current:")); debug_send_f(c); debug_send_P(PSTR("\r\n"));
}


typedef enum _battery_charge_fsm_state
{
	bcfsNone,
	bcfsMeasureBatteryVoltage,
	bcfsChargeCCCV, // CC CV charging
	bcfsChargeFloat // floating charging
} battery_charge_fsm_state_t;

static uint8_t battery_charge_fsm_state = bcfsNone;

static void poll_battery_charge()
{
	switch(battery_charge_fsm_state)
	{
	case bcfsNone: // the stop state
		break;

	case bcfsMeasureBatteryVoltage: // measure battery voltage
		if(battery_raw_voltage > 0 && nct75_exist == deWorking)
		{
			// battery raw voltage has already measured
			if(battery_raw_voltage < eeprom.chg_restart)
			{
				// the battery raw voltage is below restarting voltage;
				// do constant-current, constant voltage charge
				battery_charge_fsm_state = bcfsChargeCCCV;
				debug_send_P(PSTR("\r\nbcfsChargeCCCV\r\n"));
				set_battery_cc_cv_param();
				start_vr();
			}
			else
			{
				// the battery raw voltage is above restarting voltage;
				// do floating charge
				battery_charge_fsm_state = bcfsChargeFloat; 
				debug_send_P(PSTR("\r\nbcfsChargeFloat\r\n"));
				set_battery_float_param();
				start_vr();
			}
		}
		break;

	case bcfsChargeCCCV: // CCCV charging
		if(is_vr_running_well() && vr_now_current <= BATTERY_CHARGE_END_CURRENT)
		{
			// battery charge end; begin float charging
			battery_charge_fsm_state = bcfsChargeFloat;
			debug_send_P(PSTR("\r\nbcfsChargeFloat\r\n"));
		}


		{
			INTERVAL(10000)
			{
				set_battery_cc_cv_param();
			}
		}
		break;
	
	case bcfsChargeFloat: // floating charging

		{
			INTERVAL(10000)
			{
				set_battery_float_param();
			}
		}
		break;

	
	}
}

static void set_charge_state(uint8_t v)
{
	// enable or disable the battery charge
	if(v)
	{
		if(battery_charge_fsm_state == bcfsNone)
		{
			battery_charge_fsm_state = bcfsMeasureBatteryVoltage; // next, measure battery voltage
			debug_send_P(PSTR("\r\nbcfsMeasureBatteryVoltage\r\n"));
		}
	}
	else
	{
		if(battery_charge_fsm_state != bcfsNone)
		{
			stop_vr();
			battery_charge_fsm_state = bcfsNone;
			debug_send_P(PSTR("\r\nbcfsNone\r\n"));
		}
	}
}

//--------------------------------------------------------------------


static float voltage_sys = -1.0;
typedef enum _system_voltage_state
{
	svsDown, // system voltage is down
	svsRun, // system voltage is running, but not so powerful to pull the power
	svsRunAlt, // system voltage is running and the alternator seems to be working
} system_voltage_state_t;

static uint8_t system_voltage_state;

static void poll_system_and_battery_voltage()
{
	// check battery voltage
	if(nct75_exist == deWorking)
	{
		if(eeprom.limit_temp_l >= nct75_temperature)
			do_panic(7); // battery temp too low
		
		if(eeprom.limit_temp_h <= nct75_temperature)
			do_panic(8); // battery temp too high
	}

	// check system voltage
	voltage_sys = get_voltage_avg4(ADC_CH_VS_SENSE);

	if(voltage_sys < eeprom.vth_vr)
		system_voltage_state = svsDown;
	else if(voltage_sys < eeprom.vth_vs)
		system_voltage_state = svsRun;
	else
		system_voltage_state = svsRunAlt;

	switch(system_voltage_state)
	{
		case svsDown:
			// stop charging
			set_charge_state(0);

			// we must change output path from system to battery
			// if the system voltage drops below vth_vr
			if(output_source == osSystem)
			{
				use_power_from_bat();
			}

			// check battery voltage
			if(output_source == osBattery && battery_raw_voltage > 0 &&
				battery_raw_voltage < eeprom.vth_vd)
			{
				// too low battery voltage; goto deep shutdown.
				// this shutdown will disconnect MCU power too.
				enter_deep_sleep();
			}

			break;

		case svsRun:
			set_charge_state(0);
			use_power_from_bat();
			break;

		case svsRunAlt:
			set_charge_state(1);
			use_power_from_vs();
			break;
	}
}

__attribute__((noinline)) static void poll_power()
{
	poll_output_charge_capacitor();
	poll_battery_raw_voltage();
	poll_battery_charge();
	poll_system_and_battery_voltage();
	poll_vr();
}

//--------------------------------------------------------------------


#define LINE_BUF_LEN 80
static char line_buf[LINE_BUF_LEN + 1];
static uint8_t receive_index = 0;

//--------------------------------------------------------------------
__attribute__((noinline)) static uint8_t parse_float(const char *p, float *dest)
{
	return sscanf_P(p, PSTR("%f"), dest) == 1;
}
//--------------------------------------------------------------------

static void parse_command(const char *buf)
{
	float param = 0;
	if(buf[0] == 'c' && buf[1] == 'c' && parse_float(buf+2, &param))
	{
		set_target_current(param);
	}
	else
	if(buf[0] == 'c' && buf[1] == 'v' && parse_float(buf+2, &param))
	{
		set_target_voltage(param);
	}
	else
	if(buf[0] == 'o' && buf[1] == 'n')
	{
		start_vr();
	}
	else
	if(buf[0] == 'o' && buf[1] == 'f' && buf[2] == 'f')
	{
		stop_vr();
	}
	else
	if(buf[0] == 'c' && buf[1] == 'h' && buf[2] == 'a')
	{
		set_charge_state(1);
	}
	else
	if(buf[0] == 's' && buf[1] == 't' && buf[2] == 'o')
	{
		set_charge_state(0);
	}
	else
	{
		debug_send_P(PSTR("?\r\n"));
	}
}

//--------------------------------------------------------------------

__attribute__((noinline)) static void poll_serial()
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

//--------------------------------------------------------------------
// BME280 interface

static int8_t bme280_read(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len)
{
	uint8_t res;
	res = I2C_WriteData(dev_id, &reg_addr, 1, I2C_STOP_AT_END);
	if(res != 0) return BME280_E_COMM_FAIL;
	res = I2C_ReadData(dev_id, data, len);
	if(res != 0) return BME280_E_COMM_FAIL;
	return BME280_OK;
}

static int8_t bme280_write(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len)
{
	uint8_t nack;
	I2C_Start();
	nack = I2C_Write((reg_addr << 1));
	if(nack)
		return BME280_E_COMM_FAIL;
	nack = I2C_Write(reg_addr);
	while(len--)
	{
		nack = I2C_Write(*(data++));
		if(len != 0 && nack) return BME280_E_COMM_FAIL;
	}
	I2C_Stop();
	return BME280_OK;
}


static void bme280_delay(uint32_t period)
{
	while(period--)	_delay_ms(1);
}

static struct bme280_dev bme280;

void bme280_setup()
{
	bme280.dev_id = BME280_I2C_ADDR_PRIM;
	bme280.intf = BME280_I2C_INTF;
	bme280.read = bme280_read;
	bme280.write = bme280_write;
	bme280.delay_ms = bme280_delay;

	if(BME280_OK != bme280_init(&bme280))
	{
		debug_send_P(PSTR("\r\nBME280 not found.\r\n"));
		return;
	}
	else
	{
		bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
		bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
		bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
		bme280.settings.filter = BME280_FILTER_COEFF_16;

		uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

		if(BME280_OK != bme280_set_sensor_settings(settings_sel, &bme280))
		{
			debug_send_P(PSTR("\r\nBME280 setting register write failed.\r\n"));
			return;
		}

		bme280_exist = deFound;

	}
}

static uint8_t bme280_measure_started = 0;
static uint32_t bme280_measure_end_tick;
#define BME280_MEASURE_TIME 40

void bme280_poll()
{
	INTERVAL(8000)
	{
		// every 8000ms
		// trigger measurement start
		if(bme280_exist != deNone)
		{
			bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
			bme280_measure_end_tick = get_tick() + BME280_MEASURE_TIME;
			bme280_measure_started = 1;
		}
	}

	if(bme280_measure_started && tick_elapsed(bme280_measure_end_tick))
	{
		// get result
		bme280_measure_started = 0;
		if(BME280_OK == bme280_get_sensor_data(BME280_ALL, &bme280_last_result, &bme280))
		{
			bme280_exist = deWorking;
			debug_send_P(PSTR("\r\nBME280 Temp : "));
			debug_send_n(bme280_last_result.temperature);
			debug_send_P(PSTR(" deg C\r\n"));

			debug_send_P(PSTR("BME280 Humid: "));
			debug_send_n(bme280_last_result.humidity);
			debug_send_P(PSTR(" %\r\n"));

			debug_send_P(PSTR("BME280 Press: "));
			debug_send_n(bme280_last_result.pressure);
			debug_send_P(PSTR(" hPa\r\n"));
		}
		else
		{
			bme280_exist = deFound;
		}
	}

}
//--------------------------------------------------------------------
// NCT75 interface

/*
	NCT75 is a variant of LM75 with higher accuracy, and some
	extended features like one-shot mode.
*/

#define NCT75_ADDRESS  0b1001001 
#define NCT75_MEASURE_TIME 80

static uint8_t nct75_measure_started = 0;
static uint32_t nct75_measure_end_tick;

static void init_nct75()
{
	uint8_t res;
	uint8_t retry = 4;

	while(retry --)
	{
		I2C_Start();
		res = I2C_Write((NCT75_ADDRESS << 1) | 1); // attempt to read
		I2C_Stop();
		if(!res) goto next;
	}
	goto not_responding;

next:
	debug_send_P(PSTR("\r\nNCT75 seems to be exist ... trying to configure ...\r\n"));

	retry = 4;
	while(retry --)
	{
		// write config bytes to enable shutdown
		uint8_t config_bytes[2];
		config_bytes[0] = 0x01; // pointer: configuration register
		config_bytes[1] = (1<<5); // CR: one-shot mode on
		res = I2C_WriteData(NCT75_ADDRESS, config_bytes, 2, I2C_STOP_AT_END);
		if(res == 0)
		{
			nct75_exist = deFound;
			return;
		}
	}

not_responding:
	debug_send_P(PSTR("\r\nNCT75 not responding.\r\n"));
}


static void nct75_poll()
{
	INTERVAL(7000)
	{
		// every 7000ms
		// trigger measurement start
		if(nct75_exist != deNone)
		{

			uint8_t trigger_byte[2];
			trigger_byte[0] = 0x04; // pointer: one-shot register
			trigger_byte[1] = 0b11111111; // dummy data; data content is ignored
			I2C_WriteData(NCT75_ADDRESS, trigger_byte, 2, I2C_STOP_AT_END);

			nct75_measure_end_tick = get_tick() + NCT75_MEASURE_TIME;
			nct75_measure_started = 1;
		}
	}

	if(nct75_measure_started && tick_elapsed(nct75_measure_end_tick))
	{
		// get result
		nct75_measure_started = 0;

		uint8_t tmp[2];
		tmp[0] = 0; // temerature register
		uint8_t res;
		res = I2C_WriteData(NCT75_ADDRESS, tmp, 1, I2C_NOT_STOP_AT_END);
		if(res == 0) res = I2C_ReadData(NCT75_ADDRESS, tmp, 2);
		if(res == 0)
		{
			int16_t d = (tmp[0] << 8 ) | (tmp[1]);
			d >>= 4; 
			nct75_temperature = d * 0.0625;
			nct75_exist = deWorking;
			debug_send_P(PSTR("NCT75 : "));
			debug_send_f(nct75_temperature);
			debug_send_P(PSTR(" deg C\r\n"));
		}
		else
		{
			nct75_exist = deFound; // last temperature measurement is invalid
		}
	}

}

//--------------------------------------------------------------------
static void enum_i2c()
{
	debug_send_P(PSTR("I2C map:\r\n"));
	for(uint8_t i = 0; i < 128; ++i)
	{
		debug_send_h(i, 2);
		debug_send_P(PSTR(":"));
		if(i == 0 )
		{
			send('?');
		}
		else
		{
			I2C_Start();
			uint8_t res = I2C_Write((i << 1) | 1); // attempt to read
			I2C_Stop();
			if(res)
				send('.');
			else
				send('!');
		}
		debug_send_P(PSTR("  "));
		if((i & 7) == 7) debug_send_P(PSTR("\r\n"));
	}
}

//--------------------------------------------------------------------

__attribute__((noreturn))
int main(void)
{
	/* disable WDT */
	port_setup();
	wdt_disable();
	init_serial();

	// wait for power stabilization
	_delay_ms(50);
	// banner
	debug_send_P(PSTR("\r\n\r\nAux power controller\r\n\r\n"));

	// setup
	I2C_Init();
	enum_i2c();
//	bme280_setup();
	init_nct75();
	//eeprom_setup(); TODO: transfer eeprom content from sub processor
//	eeprom_copy(); // TODO: proper eeprom initialization
	adc_setup();
	timer0_setup();
	timer1_setup();



	// enter infinite loop
	for(;;)
	{
//		bme280_poll();
		nct75_poll();
		poll_power();
		poll_serial();
	}

}

/* -------------------------------------------------------------------- */

