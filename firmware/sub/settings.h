#ifndef SETTINGS_H__
#define SETTINGS_H__

// eeprom_t structure
#include "main_sub_common.h"

extern eeprom_t eeprom;
void parse_command(const char * buf);
void eeprom_setup();
void eeprom_write_force();
uint8_t is_eeprom_changed();
uint8_t get_eeprom_size();
void copy_shadow_eeprom(void * dest, uint8_t start, uint8_t size);

#endif

