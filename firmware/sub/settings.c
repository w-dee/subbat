#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>

#ifdef SETTINGS_DEBUG

#define PSTR(X) X
#define PROGMEM
#define sscanf_P sscanf
#define memcpy_P memcpy
#define sprintf_P sprintf

#define pgm_read_byte(p) (*(uint8_t*)(p))
#define pgm_read_word(p) (*(uint16_t*)(p))
#define pgm_read_dword(p) (*(uint32_t*)(p))
#define pgm_read_ptr(p) (*(void**)(p))
static void debug_send_P(const char * str)
{
	printf("%s", str);
}

static void debug_send(const char * str)
{
	debug_send_P(str);
}

#endif


// include eeprom.inc multiple time to complete eeprom structure definitions

// first, CODE() section
#define CMD(A, B) ((A)+(B)*256)
#define DISP(X) PSTR(X)
#define UNIT(X) PSTR(X)
#define DESC(X) PSTR(X)
#define CODE(X) X
#define DECL(...) 
#include "eeprom.inc"

// eeprom_t structure
#undef CODE
#define CODE(X)
#undef DECL
#define DECL(TYPE, NAME, ...) TYPE NAME;

struct tag_eeprom_t
{
	uint8_t size;
	#include "eeprom.inc"
};

typedef struct tag_eeprom_t eeprom_t;


// settings descriptor
typedef struct tag_eeprom_desc_t {
	uint8_t offset; // member offset
	uint8_t datum_size; // member size
	uint16_t command; // two character setting command (first byte in lower byte, second in higher byte)
	const PROGMEM char *disp; // display format in PROGMEM
	const PROGMEM char *unit; // unit string in PROGMEM
	int (*scanner)(void *dest, const char *value); // scanner. returns 1 for ok, otherwize else
	void (*printer)(char * dest, const void * src, const PROGMEM char *disp); // value printer
	const PROGMEM char * (*validator)(void *data); // validator (returns null for ok, PROGMEM string for error)
	const PROGMEM char *desc; // description string in PROGMEM
} eeprom_desc_t;

static int scan_float(void *dest, const char *value)
{
	return sscanf_P(value, PSTR("%f"), (float*)dest); 
}
static int scan_int(void *dest, const char *value)
{
	return sscanf_P(value, PSTR("%d"), (int*)dest); 
}

#define VALUE_BUF_MIN 30
#define MAX_DATUM_SIZE 4

static void print_float(char * dest, const void * src, const PROGMEM char *disp)
{
	sprintf_P(dest, disp, *(float*)src);
}

static void print_int(char * dest, const void * src, const PROGMEM char *disp)
{
	sprintf_P(dest, disp, *(int*)src);
}

#undef DECL
#define DECL(_TYPE, _NAME, _CMD, _DISP, _UNIT, _VALIDATOR, _DESC) { \
	(uint8_t) offsetof(eeprom_t, _NAME), \
	(uint8_t) sizeof(_TYPE), \
	_CMD, \
	_DISP, \
	_UNIT, \
	scan_##_TYPE, \
	print_##_TYPE, \
	_VALIDATOR, \
	_DESC },


static const eeprom_desc_t eeprom_desc[] PROGMEM = {
#include "eeprom.inc"
};

#define NUM_EEPROM_DESC (sizeof(eeprom_desc) / sizeof(eeprom_desc[0]))

// the eeprom content shadow
static eeprom_t eeprom = {
.vth_vd = 10.0,
.vth_vs = 13.2,
.vth_vr = 10.0,
.chg_cc = 3.0,
.chg_cv = 13.0,
.chg_float = 12.4,
.chg_coeff = -3*6,
.chg_restart = 12.2,
.dark_ambient = 120,
.pwm_light = 50,
.pwm_dark = 12,
.lcd_contrast = 20,
};

// load one eeprom_desc content from progmem
static void load_eeprom_desc(uint8_t index, eeprom_desc_t * dest)
{
	memcpy_P(dest, &(eeprom_desc[index]), sizeof(eeprom_desc[0]));
}


// dump current shadow content
static void dump_eeprom_shadow()
{
	for(uint8_t i = 0; i < NUM_EEPROM_DESC; ++i)
	{
		char buf[VALUE_BUF_MIN];
		eeprom_desc_t desc;
		load_eeprom_desc(i, &desc);
		// dump description
		debug_send_P(desc.desc);
		// print short command
		debug_send_P(PSTR(" ("));
		buf[0] = desc.command & 0xff;
		buf[1] = desc.command >> 8;
		buf[2] = 0;
		debug_send(buf);
		debug_send_P(PSTR(")"));
		// print colon
		debug_send_P(PSTR(": "));
		// print value
		desc.printer(buf, (char*)&eeprom + desc.offset, desc.disp);
		debug_send(buf);
		// print unit
		debug_send_P(PSTR(" "));
		debug_send_P(desc.unit);
		// print return
		debug_send_P(PSTR("\r\n"));
	}
}

// parse input and update shadow eeprom
static void parse_command(const char * buf)
{
	const char *s = buf;
	uint8_t parameter_found = 0;
	uint16_t cmd = 0;
	const PROGMEM char * msg = NULL;
	uint8_t tmp[MAX_DATUM_SIZE];

	// skip ws
	while(*s && isspace(*s)) ++s;
	if(!*s) goto err;

	// two or one character must be the command
	cmd = *s;
	++s;
	if(*s && !isspace(*s)) { cmd += *s * 256; ++s; }

	// extract parameter
	while(*s && isspace(*s)) ++s;
	if(*s)
	{
		// parameter found.
		// s points the start of the parameter.
		parameter_found = 1;
	}

	// execute command
	if(cmd == 'p')
	{
		// print current settings
		dump_eeprom_shadow();
		return;
	}
	else
	{
		// is command found in eeprom desc?
		for(uint8_t i = 0; i < NUM_EEPROM_DESC; ++i)
		{
			eeprom_desc_t desc;
			load_eeprom_desc(i, &desc);
			if(desc.command == cmd)
			{
				if(!parameter_found) goto need_param;
				// scan the input
				if(1 != desc.scanner(tmp, s))
					goto invalid_param;
				// validate the input
				msg = desc.validator(tmp);
				if(msg) goto validate_failed;
				// copy the result to shadow
				memcpy((char*)&eeprom + desc.offset, tmp, desc.datum_size);
				return;
			}
		}
	}

err:
	// not a command
	///////////////////// TODO: help()
	debug_send_P("?\r\n");
	return; // ok

need_param:
	debug_send_P(PSTR("Error: parameter required.\r\n"));
	return;

invalid_param:
	debug_send_P(PSTR("Error: invalid parameter.\r\n"));
	return;

validate_failed:
	debug_send_P(PSTR("Error: "));
	debug_send_P(msg);
	debug_send_P(PSTR("\r\n"));
	return;

}

#ifdef SETTINGS_DEBUG

int main(void)
{
	char buf[256];
	for(;;)
	{
		fgets(buf, sizeof(buf), stdin);
		parse_command(buf);
	}
	return 0;
}

#endif

