/*
 * func.c
 *
 *  Created on: Mar 6, 2026
 *      Author: cubox
 */
//기본적인 기능 구현 모터 제어, 통신 등
// 모터 제어 후 ethercat에 상태 갱신 ->  object mapping 후 update
#include "func.h"
#include "global.h"


int32_t g_MaxSpeed[MAX_AXIS] = {0, };
GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t g_MotPIN[MAX_AXIS] = {GPIO_PIN_2, 0, };
void servo_on(int axis)
{
	if(axis > 4)
		return;

	HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_RESET);
}

void servo_off(int axis)
{
	HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_SET);
}

uint32_t motCtrl(int axis){ //모터 테스트 용

	//command check function
	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
	// 1. ⭐️ [마스터 스위치 켜기] GCONF (0x00) 레지스터 세팅
	// 0x00000080을 넣으면: "십자 나사 무시해! MS1/MS2 핀 무시해! 이제부터 무조건 UART 명령만 따라!" 라는 뜻입니다.
	// 이 한 줄이 들어가야 비로소 진정한 256 마이크로스텝(무소음/무진동)이 켜집니다.
	TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
	// 2. 질문자님이 찾으신 황금비율 전류 세팅 (IRUN=16)
	// 3. 가장 안정적이었던 속도로 출발 (20만)
	TMC2209_WriteRegister(&huart1, axis, 0x10, 0x00070E03);
	TMC2209_WriteRegister(&huart1, axis, 0x22, -200000);
	// 3. 쓰기 후의 통신 카운트 읽기
	uint32_t count_after = TMC2209_ReadRegister(&huart1, axis, 0x02);

	return count_before != count_after ? 1 : 0;//true -> command success
}

// Op 시 PDO를 통한 Motor Max speed update function
uint8_t mot_MaxSpdUpdate(int axis){

	//command check function
	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);

    if (ESCvar.ALstatus == ESC_OP) {												// OP모드 진입 시에만 작동
        if (Rb.target_speed[axis] != g_MaxSpeed[axis]) {										// TwinCAT에서 보낸 값이 이전과 다를 때(변경되었을 때)만 UART 전송
        	TMC2209_WriteRegister(&huart1, axis, 0x22, Rb.target_speed[axis]);
            //TMC2209_Update(&huart1, 0x00, Rb.target_speed[0], Rb.direction[0]);
            // 현재 값을 기억
        	g_MaxSpeed[axis] = Rb.target_speed[axis];
            //old_dir = (Rb.control_word << 16); 방향은 목표 위치나 목표 속도를 기준으로 결정
        }
    }
    else {																			// OP 가 아닌 경우
        if (g_MaxSpeed[axis] != 0) {														// 에러나 통신 끊김 시 정지 (이것도 중복 전송 방지)
        	//TMC2209_WriteRegister(&huart1, 0x00, 0x22, 0);						// 정지 전류 => 속도 0로 설정
        	servo_off(axis); 															// 통신 끊김 시 모터 전원 내림
        	g_MaxSpeed[axis] = 0;															// 중복 방지를 위해 old_speed 값 변경
        }
    }

	uint32_t count_after = TMC2209_ReadRegister(&huart1, 0x00, 0x02);

	return count_before != count_after ? 1 : -1;//true -> command success
}

// Op 시 PDO를 통한 Motor 가감속 프로파일 update function
uint8_t mot_ProfUpdate(int axis){

	//command check function
	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);



	uint32_t count_after = TMC2209_ReadRegister(&huart1, axis, 0x02);

	return count_before != count_after ? 1 : -1;//true -> command success
}

uint32_t get_MaxSpeed(int axis)
{
	return g_MaxSpeed[axis];
}

uint32_t CalcMotion(int axis)
{
	int pulse = 0;

	return pulse;
}

void ec_valinit()
{
	for(int i = 0; i < MAX_AXIS; i++){
		g_StatusWord[i] = 0x250;
		g_is_homed[i] = false;
		g_drv_status[i] = 0;
		gw_overtemp_pre[i] = false;
		g_MaxSpeed[i] = 0xFF;
	}
}
