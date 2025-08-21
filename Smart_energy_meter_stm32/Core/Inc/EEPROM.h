#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "stm32f0xx_hal.h"
#include <stdint.h>
extern I2C_HandleTypeDef hi2c1;
// Địa chỉ EEPROM (ví dụ cho AT24C04, A0/A1/A2 = 0 => 0xA0 hoặc 0xA4 nếu đã shift sẵn bên trái 1 bit)
#define EEPROM_ADDR  0xA4
//*** Dia chi cac thong so ghi trong EEPROM
#define ADD_ENERGY 0x20
#define ADD_BALANCE 0x40
#define ADD_ENERGY_RATE 0x60
void EEPROM_Write(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, uint8_t *data, uint16_t len);
void EEPROM_Read(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, uint8_t *data, uint16_t len);

void EEPROM_Write_Float(I2C_HandleTypeDef *hi2c, uint16_t mem_addr, float value);
float EEPROM_Read_Float(I2C_HandleTypeDef *hi2c, uint16_t mem_addr);

void test_EEPROM_Energy(void);
void read_initial_settings(void);

// void int_to_hex_array(int num);  

#endif
