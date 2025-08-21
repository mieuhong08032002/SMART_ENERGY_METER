// #ifndef __RS485_H
// #define __RS485_H

// #include "stm32f0xx_hal.h"
// // Cấu trúc dữ liệu khung truyền
// #define FRAME_SIZE 11  // Độ dài khung dữ liệu

// typedef struct {
//     uint8_t start;       // 0xAA
//     uint16_t id;         // 2 byte ID
//     uint8_t key;         // 1 byte KEY
//     uint32_t value;      // 4 byte VALUE
//     uint16_t crc;        // 2 byte CRC
//     uint8_t end;         // 0xBB
// } RS485_Frame;

// // Biến UART sử dụng (chỉnh theo thực tế)
// extern UART_HandleTypeDef huart1;

// // Khai báo các hàm
// void RS485_Init();
// void RS485_Send(uint16_t id, uint8_t key, uint32_t value);
// void RS485_Start_Receive();
// //void RS485_Process_Data();
// void RS485_Process_Data(uint16_t *id, uint8_t *key, uint32_t *value);
// uint16_t RS485_Calculate_CRC(uint8_t *data, uint16_t length);

// #endif
#ifndef __RS485_H
#define __RS485_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
extern UART_HandleTypeDef huart1;
#define FRAME_SIZE 11

// Chân điều khiển DE của RS485
#define RS485_DE_PIN     GPIO_PIN_12
#define RS485_DE_PORT    GPIOA

// Bộ đệm nhận
extern uint8_t rx_buffer[FRAME_SIZE];
extern volatile uint8_t rs485_rx_done;
extern uint32_t last_byte_tick;

// Hàm khởi tạo
void RS485_Init(void);

// Gửi dữ liệu (gói 11 byte)
void RS485_Send(uint16_t id, uint8_t key, uint32_t value);

// Tính CRC-16 (Modbus)
uint16_t RS485_Calculate_CRC(uint8_t *data, uint16_t length);

#endif
