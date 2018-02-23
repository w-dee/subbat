#ifndef TINYI2CMASTER_H__
#define TINYI2CMASTER_H__

void I2C_Init();
void I2C_Start();
void I2C_Stop();
unsigned char I2C_Write(unsigned char c);
unsigned char I2C_Read(unsigned char ack);
uint8_t I2C_WriteData(uint8_t slave_addr_7bit, uint8_t *buf, uint8_t size, uint8_t stop);
uint8_t I2C_ReadData(uint8_t slave_addr_7bit, uint8_t *buf, uint8_t size);

#endif

