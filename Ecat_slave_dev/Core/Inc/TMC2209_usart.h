/*
 * TMC2209_usart.h
 *
 *  Created on: Mar 17, 2026
 *      Author: cubox
 */

#ifndef INC_TMC2209_USART_H_
#define INC_TMC2209_USART_H_

#include "main.h" // huart 설정 가져오기

// TMC2209의 Slave Address
#define TMC2209_ADDR 0x00

// TMC2209 레지스터 주소 예시
#define TMC2209_REG_GCONF  0x00
#define TMC2209_REG_CHOPCONF 0x6C
#define TMC2209_REG_IHOLD_IRUN 0x10

// =========================================================
// 1. TMC2209 전용 CRC8 계산 함수
// =========================================================
//static uint8_t TMC2209_CalcCRC(uint8_t* datagram, uint8_t datagramLength);

// =========================================================
// 2. 1선식 UART 레지스터 Write 함수
// =========================================================
void TMC2209_WriteRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr, uint32_t data);

// =========================================================
// 3. 모터 전류 설정 (IHOLD_IRUN) 예시 함수
// =========================================================
void TMC2209_SetCurrent(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t i_run, uint8_t i_hold);
void TMC2209_Update(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t target_speed, uint8_t direction);
void TMC2209_Init(UART_HandleTypeDef *huart, uint8_t motor_addr);
void TMC2209_WriteRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr, uint32_t data);
uint32_t TMC2209_ReadRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr);
#endif /* INC_TMC2209_USART_H_ */
