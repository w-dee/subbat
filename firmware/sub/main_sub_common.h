#ifndef MAIN_SUB_COMMON_H__
#define MAIN_SUB_COMMON_H__


// eeprom_t structure
#define CODE(X)
#define DECL(TYPE, NAME, ...) TYPE NAME;

struct tag_eeprom_t
{
	uint8_t size;
	#include "eeprom.inc"
};

typedef struct tag_eeprom_t eeprom_t;


#define SS_FLAGS_EEPROM_CHANGED 4 // whether eeprom content has been changed


typedef struct tag_sub_status_t
{
	float sub_5v_current; // sub module 5V consumption current
	uint8_t flags; // flags
} sub_status_t;



#define I2C_SUB_SLAVE_ADDR 0x16


#define TWI_CMD_PING 0
#define TWI_CMD_QUERY_EEPROM_SIZE 4
#define TWI_CMD_COPY_EEPROM 5

#endif

