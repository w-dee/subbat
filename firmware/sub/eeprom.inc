
CODE(  


// validators

const char * check_sys_voltage(void * data) {
	float v = *(float*)data;
	if(v >= 6.0 && v <= 30) return NULL;
	return PSTR("value must be within [6.0, 30.0]");
}

const char * check_charge_ampare(void * data) {
	float v = *(float*)data;
	if(v > 0 && v <= 10) return NULL;
	return PSTR("value must be within (0.0, 10.0]");
}

const char * check_coeff(void *data) {
	float v = *(float*)data;
	if(v > -1000.0 && v < 1000.0) return NULL;
	return PSTR("value must be within (-1000.0, 1000.0)");
}

const char * check_byte(void *data)
{
	int v = *(int*)data;
	if(v >= 0 && v < 256) return NULL;
	return PSTR("value must be within [0, 255]");
}

const char * check_temperature(void *data)
{
	int v = *(int*)data;
	if(v >= -30 && v < 120) return NULL;
	return PSTR("value must be within [-30, 120]");
}


// string constants

static const char desc_vth_vd[] PROGMEM = "Deep sleep shutdown battery voltage";
static const char desc_vth_vs[] PROGMEM = "Charge start system input voltage";
static const char desc_vth_vr[] PROGMEM = "Active system input voltage threshold";
static const char desc_chg_cc[] PROGMEM = "Constant current: charge current";
static const char desc_chg_cv[] PROGMEM = "Constant voltage: topping voltage @ 25 deg C";
static const char desc_chg_float[] PROGMEM = "Floating voltage @ 25 deg C";
static const char desc_float_cc[] PROGMEM = "Floating maximum current";
static const char desc_coeff_cv[] PROGMEM = "Temperature coefficient of cv charge";
static const char desc_coeff_float[] PROGMEM = "Temperature coefficient of floating charge";
static const char desc_chg_restart[] PROGMEM = "Recharge start battery voltage";
static const char desc_limit_temp_h[] PROGMEM = "Shutdown temperature higher limit";
static const char desc_limit_temp_l[] PROGMEM = "Shutdown temperature lower limit";
static const char desc_dark_ambient[] PROGMEM = "Ambient threshold value for dark/light switch";
static const char desc_pwm_light[] PROGMEM = "LED pwm value for light ambient";
static const char desc_pwm_dark[] PROGMEM = "LED pwm value for dark ambient";
static const char desc_lcd_contrast[] PROGMEM = "LCD contrast value";

static const char disp_2_2f[] PROGMEM = "%2.2f";
static const char disp_3d[] PROGMEM = "%3d";

static const char unit_V[] PROGMEM = "V";
static const char unit_A[] PROGMEM = "A";
static const char unit_coeff[] PROGMEM = "mV/deg C";
static const char unit_uint8[] PROGMEM = "[0-255]";
static const char unit_degC[] PROGMEM = "deg C";



 )


DECL(float,   vth_vd,       CMD('v','d'), disp_2_2f, unit_V,  check_sys_voltage)
DECL(float,   vth_vs,       CMD('v','s'), disp_2_2f, unit_V,  check_sys_voltage)
DECL(float,   vth_vr,       CMD('v','r'), disp_2_2f, unit_V,  check_sys_voltage)
DECL(float,   chg_cc,       CMD('c','c'), disp_2_2f, unit_A,  check_charge_ampare)
DECL(float,   chg_cv,       CMD('c','v'), disp_2_2f, unit_V,  check_sys_voltage)
DECL(float,   chg_float,    CMD('f','l'), disp_2_2f, unit_V,  check_sys_voltage)
DECL(float,   float_cc,     CMD('f','c'), disp_2_2f, unit_A,  check_charge_ampare)
DECL(float,   coeff_cv,     CMD('t','v'), disp_2_2f, unit_coeff,  check_coeff)
DECL(float,   coeff_float,  CMD('t','f'), disp_2_2f, unit_coeff,  check_coeff)
DECL(float,   chg_restart,  CMD('r','e'), disp_2_2f, unit_V,      check_sys_voltage)
DECL(int,     limit_temp_h, CMD('l','h'), disp_3d,   unit_degC,   check_temprature)
DECL(int,     limit_temp_l, CMD('l','l'), disp_3d,   unit_degC,   check_temprature)
DECL(int,     dark_ambient, CMD('d','a'), disp_3d,   unit_uint8,  check_byte)
DECL(int,     pwm_light,    CMD('p','l'), disp_3d,   unit_uint8,  check_byte)
DECL(int,     pwm_dark,     CMD('p','d'), disp_3d,   unit_uint8,  check_byte)
DECL(int,     lcd_contrast, CMD('l','c'), disp_3d,   unit_uint8,  check_byte)

