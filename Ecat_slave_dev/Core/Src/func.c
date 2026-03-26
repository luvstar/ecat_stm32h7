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

// ==========================================================
// ⭐️ CiA 402 State Machine Parser (1ms ISR 내부에서 호출)
// ==========================================================
void CiA402_StateMachine(uint8_t axis) {
    uint32_t cw = Rb.control_word[axis];  // 마스터가 내린 명령 (Target)
    uint32_t sw = Wb.status_word[axis];   // 슬레이브가 보고할 현재 상태 (Actual)

    // ------------------------------------------------------
    // [관문 1] 하드웨어 에러(Fault) 상태 처리 (최우선 순위)
    // ------------------------------------------------------
    if (g_hwErr[axis] == true) {
        servo_off(axis);                      // 1. 즉시 모터 끄기 (물리적 차단)
        Wb.status_word[axis] |= (1 << 3);     // 2. Status Word Bit 3 (Fault) ON
        return;                               // 3. 아래 제어 로직 무시하고 함수 탈출
    }

    // 만약 에러 상태(Fault)에 빠져있는데, 마스터가 Bit 7 (Fault Reset)을 0->1로 쳤다면?
    if (sw & (1 << 3)) {
        if ((cw & (1 << 7)) != 0) {
            // 알람 해제 조건 충족! (TwinCAT에서 Reset 버튼 누름)
            Wb.status_word[axis] = 0x0250;    // 에러 비트 지우고 'Switch On Disabled'로 초기화
        }
        servo_off(axis); // 에러가 완전히 풀릴 때까지 물리적 구동 절대 금지
        return;
    }

    // ------------------------------------------------------
    // [관문 2] CiA 402 상태 전이 (하위 4비트 마스킹 검사)
    // 마스터의 Control Word(cw) 명령에 맞춰 Status Word(sw)를 응답
    // ------------------------------------------------------
    switch (cw & 0x000F) { // Bit 0~3만 추출해서 스위치문으로 검사

        case 0x0000: // [Disable Voltage 명령]
        case 0x0002: // [Quick Stop 명령]
            servo_off(axis);
            Wb.status_word[axis] = 0x0250; // 응답: Switch On Disabled
            break;

        case 0x0006: // [Shutdown 명령] - 서보 켜기 1단계
            servo_off(axis);
            Wb.status_word[axis] = 0x0231; // 응답: Ready to Switch On
            break;

        case 0x0007: // [Switch On 명령] - 서보 켜기 2단계 (또는 Disable Operation)
            servo_off(axis);
            Wb.status_word[axis] = 0x0233; // 응답: Switched On
            break;

        case 0x000F: // ⭐️ [Enable Operation 명령] - 최종 서보 ON 단계!
            servo_on(axis);
            Wb.status_word[axis] = 0x0237; // 응답: Operation Enabled

            // 이 상태에서만 목표 위치 연산 및 펄스 출력을 허용합니다.
            // CalcMotion(axis); // To do
            break;

        default:
            // 표준에 정의되지 않은 비정상 비트 조합이 들어온 경우 안전하게 차단
            servo_off(axis);
            Wb.status_word[axis] = 0x0250;
            break;
    }

    // ------------------------------------------------------
    // [관문 3] 커스텀 제어 비트 처리 (예: 홈 강제 초기화)
    // ------------------------------------------------------
    if ((Wb.status_word[axis] & 0x0237) == 0x0237) { // 서보가 정상적으로 켜진 상태에서만 허용
        if (cw & (1 << 11)) { // Custom Bit 11 (Force Homing)
            // 현재 위치 변수를 0으로 리셋 (하드웨어 타이머 카운터 리셋 함수 호출)
            //Reset_Actual_Position(axis); //  To do
            g_is_homed[axis] = true;
        }
    }
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
