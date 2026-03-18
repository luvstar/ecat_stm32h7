/*
 * func.c
 *
 *  Created on: Mar 6, 2026
 *      Author: cubox
 */
//기본적인 기능 구현 모터 제어, 통신 등
// 모터 제어 후 ethercat에 상태 갱신 ->  object mapping 후 update
#include "func.h"

uint32_t motCtrl(){
	uint32_t state = 0;

	//command check function
	uint32_t count_before = TMC2209_ReadRegister(&huart1, 0x00, 0x02);
	// 1. ⭐️ [마스터 스위치 켜기] GCONF (0x00) 레지스터 세팅
	// 0x00000080을 넣으면: "십자 나사 무시해! MS1/MS2 핀 무시해! 이제부터 무조건 UART 명령만 따라!" 라는 뜻입니다.
	// 이 한 줄이 들어가야 비로소 진정한 256 마이크로스텝(무소음/무진동)이 켜집니다.
	TMC2209_WriteRegister(&huart1, 0x00, 0x00, 0x00000080);
	// 2. 질문자님이 찾으신 황금비율 전류 세팅 (IRUN=16)
	TMC2209_WriteRegister(&huart1, 0x00, 0x10, 0x00071003);
	// 3. 가장 안정적이었던 속도로 출발 (20만)
	TMC2209_WriteRegister(&huart1, 0x00, 0x22, 200000);
	// 3. 쓰기 후의 통신 카운트 읽기
	uint32_t count_after = TMC2209_ReadRegister(&huart1, 0x00, 0x02);

	return count_before == count_after ? 0 : 1;//true -> 명령 입력 실패
}

uint8_t setEcat(){
	uint8_t result = 0;


	return result;
}

void servo_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}
