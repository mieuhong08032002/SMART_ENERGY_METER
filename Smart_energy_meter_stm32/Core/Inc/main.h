/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "stm32f0xx_hal.h"
#include <stm32f030x8.h>
#include "max7219.h"
#include "ADE7753.h"
#include "rs485.h"
#include "kalman_filter.h"
#include "EEPROM.h"
#include "debug.h"
#define DEVICE_ID 0x0001  // ID cua thiet bi
#define ON 1  
#define OFF 0  

typedef enum {
    ADE_STATE_IDLE = 0,
    ADE_STATE_FREQ,
    ADE_STATE_VOLT,
    ADE_STATE_CURR,
    ADE_STATE_TEMP,
    ADE_STATE_PWR
} ADE_State;

// Dinh nghia ham
void get_Voltage(ADE_Name *test);
void get_Frequency(ADE_Name *test);
void get_temp(ADE_Name *test);
void get_Current(ADE_Name *test);
void get_Power(ADE_Name *test);
void ADE_Task_NonBlocking(void);

uint16_t crc16_modbus(const uint8_t *data, uint16_t length);
void send11ByteData(uint8_t key, uint32_t value);
void loop_rs485_check(void);
void RS485_Handle_Command(uint16_t id, uint8_t key, uint32_t value);
void reset_stm();
void uart_log();
void Update_Energy_And_Balance(void);
void addBalance(uint32_t amount);
void Check_Payment(void);
void Control_Relay(void);
void setEnergyRate(uint32_t energy_rate);
void Error_Handler(void);
#define RELAY(pin_value) \
    do { \
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, pin_value);\
    } while(0)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
