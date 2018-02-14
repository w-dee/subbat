CODE(

const PROGMEM char * check_sys_voltage(void * data) {
	float v = *(float*)data;
	if(v >= 6.0 && v <= 30) return NULL;
	return PSTR("value must be within [6.0, 30.0]");
}

const PROGMEM char * check_charge_ampare(void * data) {
	float v = *(float*)data;
	if(v > 0 && v <= 10) return NULL;
	return PSTR("value must be within (0.0, 10.0]");
}

const PROGMEM char * check_coeff(void *data) {
	float v = *(float*)data;
	if(v > -1000.0 && v < 1000.0) return NULL;
	return PSTR("value must be within (-1000.0, 1000.0)");
}

const PROGMEM char * check_byte(void *data)
{
	int v = *(int*)data;
	if(v >= 0 && v < 256) return NULL;
	return PSTR("value must be within [0, 255]");
}

)


DECL(float,   vth_vd,       CMD('v','d'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Deep sleep shutdown battery voltage"))
DECL(float,   vth_vs,       CMD('v','s'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Charge start system input voltage"))
DECL(float,   vth_vr,       CMD('v','r'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Active system input voltage threshold"))
DECL(float,   chg_cc,       CMD('c','c'), DISP("%2.2f"), UNIT("A"),  check_charge_ampare, DESC("Constant current: charge current"))
DECL(float,   chg_cv,       CMD('c','v'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Constant voltage: topping voltage @ 25 deg C"))
DECL(float,   chg_float,    CMD('f','l'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Floating voltage @ 25 deg C"))
DECL(float,   chg_coeff,    CMD('c','o'), DISP("%2.2f"), UNIT("mV/deg C"), check_coeff,   DESC("Temperature coefficient of cc and cv"))
DECL(float,   chg_restart,  CMD('r','e'), DISP("%2.2f"), UNIT("V"),  check_sys_voltage,   DESC("Recharge start battery voltage"))
DECL(int,     dark_ambient, CMD('d','a'), DISP("%3d"),   UNIT("[0-255]"),  check_byte,  DESC("Ambient threshold value for dark/light switch"))
DECL(int,     pwm_light,    CMD('p','l'), DISP("%3d"),   UNIT("[0-255]"),  check_byte,  DESC("LED pwm value for light ambient"))
DECL(int,     pwm_dark,     CMD('p','d'), DISP("%3d"),   UNIT("[0-255]"),  check_byte,  DESC("LED pwm value for dark ambient"))
DECL(int,     lcd_contrast, CMD('l','c'), DISP("%3d"),   UNIT("[0-255]"),  check_byte,  DESC("LCD contrast value"))

