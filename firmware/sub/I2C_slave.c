#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

/*
adapted from https://github.com/g4lvanix/I2C-slave-lib

Don't use this library because I'dont test well enough.
*/

#include "I2C_slave.h"

int8_t i2c_buffer_address;
#define I2C_BUFFER_ADDR_INVALID -1
uint8_t i2c_txbuffer[BUFFER_LEN ];
uint8_t i2c_tx_len;
uint8_t i2c_rxbuffer[BUFFER_LEN ];
uint8_t i2c_rx_len;

void I2C_init(uint8_t address){
	// load address into TWI address register
	TWAR = (address << 1);
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

ISR(TWI_vect){
	
	// temporary stores the received data
	uint8_t data;
	
	// own address has been acknowledged
	if( (TWSR & 0xF8) == TW_SR_SLA_ACK ){  
		i2c_buffer_address = I2C_BUFFER_ADDR_INVALID ;
		// clear TWI interrupt flag, prepare to receive next byte and acknowledge
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
	}
	else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ){ // data has been received in slave receiver mode
		
		// save the received byte inside data 
		data = TWDR;
		
		// check wether an address has already been transmitted or not
		if(i2c_buffer_address == I2C_BUFFER_ADDR_INVALID ){
			
			i2c_rx_len = in_isr_callback_prepare_i2c_write(data);
			i2c_buffer_address = 0;
			
			// clear TWI interrupt flag, prepare to receive next byte and acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{ // if a databyte has already been received
			
			// store the data at the current address
			i2c_rxbuffer[i2c_buffer_address] = data;
			
			// increment the buffer address
			i2c_buffer_address++;
			
			// if there is still enough space inside the buffer
			if(i2c_buffer_address == i2c_rx_len)
			{
				in_isr_callback_done_i2c_write();
			}

			if(i2c_buffer_address < i2c_rx_len){
				// clear TWI interrupt flag, prepare to receive next byte and acknowledge
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
			}
			else{
				// Don't acknowledge
				TWCR &= ~(1<<TWEA); 
				// clear TWI interrupt flag, prepare to receive last byte.
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN); 
			}
		}
	}
	else if( (TWSR & 0xF8) == TW_ST_DATA_ACK ){ // device has been addressed to be a transmitter
		
		// copy data from TWDR to the temporary memory
		data = TWDR;
		
		// if no buffer read address has been sent yet
		if( i2c_buffer_address == I2C_BUFFER_ADDR_INVALID  ){
			in_isr_callback_prepare_i2c_read(data);
			i2c_buffer_address = 0;
		}
		
		// copy the specified buffer address into the TWDR register for transmission
		TWDR = i2c_txbuffer[i2c_buffer_address];
		// increment buffer read address
		i2c_buffer_address++;
		
		// if there is another buffer address that can be sent
		if(i2c_buffer_address < i2c_tx_len ){
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
		}
		else{
			// Don't acknowledge
			TWCR &= ~(1<<TWEA); 
			// clear TWI interrupt flag, prepare to receive last byte.
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEN); 
		}
		
	}
	else{
		// if none of the above apply prepare TWI to be addressed again
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	} 
}
