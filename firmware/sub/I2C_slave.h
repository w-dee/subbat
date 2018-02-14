#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#define BUFFER_LEN 32 // (must be less than 128 )

extern int8_t i2c_buffer_address;
extern uint8_t i2c_txbuffer[BUFFER_LEN ];
extern uint8_t i2c_rxbuffer[BUFFER_LEN ];
uint8_t i2c_tx_len;
uint8_t i2c_rx_len;

void I2C_init(uint8_t address);
void I2C_stop(void);
ISR(TWI_vect);

// this is called during i2c isr; user code must
// return expected payload bytes.
uint8_t in_isr_callback_prepare_i2c_write(uint8_t addr);
// this is called from i2c isr; this is called when
// master did writing all bytes specified by previous
// call of in_isr_callback_prepare_i2c_write().
// the data is in i2c_rxbuffer[].
void in_isr_callback_done_i2c_write();

// this is called during i2c isr; user code must
// prepare data in i2c_txbuffer[] and set i2c_txlen.
void in_isr_callback_prepare_i2c_read(uint8_t addr);

#endif // I2C_SLAVE_H
