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
#include "stdlib.h"

const uint32_t motor_channels[4] = {TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E};

// ==========================================================
// 1ms 주기로 실행되는 사다리꼴 가감속 프로파일 연산 및 펄스 생성
// ==========================================================
// ⭐️ 전역 상태 변수들 (1ms마다 유지되어야 함)
int32_t g_current_speed[4] = {0, 0, 0, 0};
int32_t g_current_accel[4] = {0, 0, 0, 0}; // S-Curve용 현재 가속도 상태

int32_t g_MaxSpeed[MAX_AXIS] = {0, };
GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOF, GPIOF, GPIOF};
GPIO_TypeDef* DIR_PORT[MAX_AXIS] = {GPIOA, GPIOF, GPIOF, GPIOF};
uint16_t g_MotPIN[MAX_AXIS] = {GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
uint16_t DIR_PIN[MAX_AXIS] = {GPIO_PIN_8, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};

TIM_TypeDef* motor_timers[4] = {TIM1, 0,};

// 1ms 마다 1000으로 나누어 떨어지지 않는 "나머지 펄스 조각"들을 모아두는 적분기입니다.
int32_t g_pos_accumulator[MAX_AXIS] = {0, 0, 0, 0};
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

//uint32_t CalcMotion(int axis)
//{
//	int pulse = 0;
//
//	return pulse;
//}

// ==========================================================
// ⭐️ CiA 402 State Machine Parser (1ms ISR 내부에서 호출)
// ==========================================================
void CiA402_StateMachine(uint8_t axis) {
	static uint32_t prev_cw[MAX_AXIS] = {0,};
    uint32_t cw = Rb.control_word[axis];  // 마스터가 내린 명령 (Target)
    uint32_t sw = g_StatusWord[axis];   // 슬레이브가 보고할 현재 상태 (Actual)
	if(axis >= 1)
		return;//임시 테스트용 1개 축 전용
    // ------------------------------------------------------
    // [관문 1] 하드웨어 에러(Fault) 상태 처리 (최우선 순위)
    // ------------------------------------------------------
    if (g_hwErr[axis] == true) {
        servo_off(axis);                      // 1. 즉시 모터 끄기 (물리적 차단)
        g_StatusWord[axis] = (g_StatusWord[axis] & 0xFFF0) | 0x0008;     // 2. Status Word Bit 3 (Fault) ON
        return;                               // 3. 아래 제어 로직 무시하고 함수 탈출
    }

    // 만약 에러 상태(Fault)에 빠져있는데, 마스터가 Bit 7 (Fault Reset)을 0->1로 쳤다면?
    if (sw & (1 << 3)) {
        // ⭐️ 현재 1이고, 이전에는 0이었을 때만 (Rising Edge)
        if (((cw & (1 << 7)) != 0) && ((prev_cw[axis] & (1 << 7)) == 0)) {
            g_StatusWord[axis] = 0x0250;
        }
        servo_off(axis);
        prev_cw[axis] = cw; // ⭐️ 현재 상태 기억
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
            g_StatusWord[axis] = 0x0250; // 응답: Switch On Disabled
            break;

        case 0x0006: // [Shutdown 명령] - 서보 켜기 1단계
            servo_off(axis);
            g_StatusWord[axis] = 0x0231; // 응답: Ready to Switch On
            break;

        case 0x0007: // [Switch On 명령] - 서보 켜기 2단계 (또는 Disable Operation)
            servo_off(axis);
            g_StatusWord[axis] = 0x0233; // 응답: Switched On
            break;

        case 0x000F: // ⭐️ [Enable Operation 명령] - 최종 서보 ON 단계!
            servo_on(axis);
            g_StatusWord[axis] = 0x0237; // 응답: Operation Enabled
            break;

        default: // 표준에 정의되지 않은 비정상 비트 조합이 들어온 경우 안전하게 차단
            servo_off(axis);
            g_StatusWord[axis] = 0x0250;
            break;
    }

    // ------------------------------------------------------
    // [관문 3] 커스텀 제어 비트 처리 (예: 홈 강제 초기화)
    // ------------------------------------------------------
    if ((g_StatusWord[axis] & 0x0237) == 0x0237) { // 서보가 정상적으로 켜진 상태에서만 허용
        if (cw & (1 << 11)) { // Custom Bit 11 (Force Homing)
            // 현재 위치 변수를 0으로 리셋 (하드웨어 타이머 카운터 리셋 함수 호출)
            Reset_Actual_Position(axis); //  To do
            g_is_homed[axis] = true;
        }
    }
    prev_cw[axis] = cw;
}



// ==========================================================
// 1ms 주기로 실행되는 실전용 가감속 프로파일 연산
// ==========================================================
void CalcMotion(int axis) {//CalcMotion
	if(axis >= 1)
		return;//임시 테스트용 1개 축 전용
    int32_t target_v = Rb.target_speed[axis];
    uint32_t max_v = Rb.max_speed[axis];
    uint32_t acc_time = Rb.accel_time[axis];
    uint32_t dec_time = Rb.decel_time[axis];
    uint8_t  pattern = Rb.speed_pattern[axis];    // 0: 사다리꼴, 1: S-Curve

    // [1] 베이스 가감속 변화량 계산 (안전장치 포함)
    int32_t dv_acc = (acc_time > 0) ? (max_v / acc_time) : max_v;
    int32_t dv_dec = (dec_time > 0) ? (max_v / dec_time) : max_v;
    if (dv_acc == 0) dv_acc = 1;
    if (dv_dec == 0) dv_dec = 1;

    // ⭐️ [2] 방향 전환 핵심 로직: 현재 '가속' 중인지 '감속' 중인지 절대적 판별
    bool is_accelerating = false;
    if (target_v > g_current_speed[axis]) {
        // 목표 속도를 향해 속도를 '올려야' 하는 상황
        is_accelerating = (g_current_speed[axis] >= 0); // 현재 정방향(+)이면 0에서 멀어지므로 가속!
    } else if (target_v < g_current_speed[axis]) {
        // 목표 속도를 향해 속도를 '내려야' 하는 상황
        is_accelerating = (g_current_speed[axis] <= 0); // 현재 역방향(-)이면 0에서 멀어지므로 가속!
    }

    // 적용할 최종 목표 가속도 결정 (가속 중이면 dv_acc, 감속 중이면 dv_dec)
    int32_t target_dv = is_accelerating ? dv_acc : dv_dec;

    // ⭐️ [3] 프로파일 패턴 적용 (사다리꼴 vs S-Curve)
    int32_t applied_dv = 0;

    if (pattern == 0) {
        // (A) 사다리꼴 모드: 목표 가속도를 즉시 100% 때려 넣음
        applied_dv = target_dv;
        g_current_accel[axis] = target_dv; // 상태 동기화
    }
    else {
        // (B) S-Curve 모드: 가속도 자체를 서서히 증가/감소시킴 (Jerk)
        // 저크 값(dj)은 가속 시간의 1/4 정도를 부드럽게 깎는 비율로 설정 (현업 튜닝 포인트)
        int32_t dj = (target_dv > 10) ? (target_dv / 10) : 1;

        if (g_current_accel[axis] < target_dv) {
            g_current_accel[axis] += dj;
            if (g_current_accel[axis] > target_dv) g_current_accel[axis] = target_dv;
        } else if (g_current_accel[axis] > target_dv) {
            g_current_accel[axis] -= dj;
            if (g_current_accel[axis] < target_dv) g_current_accel[axis] = target_dv;
        }
        applied_dv = g_current_accel[axis];
    }

    // ⭐️ [4] 실제 속도에 가감속 적용 (0점을 교차할 때의 안전장치 포함)
    if (g_current_speed[axis] < target_v) {
        g_current_speed[axis] += applied_dv;
        if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v; // 오버슈트 클램핑
    }
    else if (g_current_speed[axis] > target_v) {
        g_current_speed[axis] -= applied_dv;
        if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v; // 언더슈트 클램핑
    }

    // ------------------------------------------------------
    // [5] 방향 핀(DIR) 및 주파수 물리적 출력 (이전과 동일)
    // ------------------------------------------------------
    bool is_negative_speed = (g_current_speed[axis] < 0);
    bool is_reverse_bit_on = (Rb.control_word[axis] & (1 << 16)) != 0;

    if (is_negative_speed ^ is_reverse_bit_on) {
        HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], GPIO_PIN_RESET);
    }

    uint32_t output_speed_hz = abs(g_current_speed[axis]);
    if (output_speed_hz == 0) {
        stop_pulse_generator(axis);
    } else {
        update_timer_pwm_freq(axis, output_speed_hz);
    }

    Update_Actual_Position(axis);
}

// ==========================================================
// 실시간 주파수 변경 및 펄스 재개 (1ms ISR 내부에서 호출)
// ==========================================================
void update_timer_pwm_freq(uint8_t axis, uint32_t freq_hz) {
    if (freq_hz == 0) {
        stop_pulse_generator(axis);
        return;
    }

    TIM_TypeDef *TIMx = motor_timers[axis];

    if(TIMx == NULL)
    	return;
    // 1. 필요한 총 분주비 계산
    uint32_t total_div = TIM_CLOCK_FREQ / freq_hz;

    uint32_t psc = 0;
    uint32_t arr = 0;

    // 2. 16비트 타이머 한계 극복 로직 (오토 스케일링)
    // TIM1, TIM8 등은 ARR이 16비트(Max 65535)이므로,
    // 저속(총 분주비 > 65536)일 때는 반드시 PSC를 올려주어야 합니다.
    if (total_div <= 65536) {
        psc = 0;
        arr = total_div - 1;
    } else {
        // 저속 영역: PSC를 증가시켜서 ARR이 16비트 안에 들어오도록 계산
        psc = (total_div / 65536);
        arr = (total_div / (psc + 1)) - 1;
    }

    // 3. 레지스터에 계산값 직접 꽂아 넣기 (HAL보다 수십 배 빠름)
    TIMx->PSC = psc;
    TIMx->ARR = arr;

    // 4. STEP 핀 Duty Cycle 50% 유지 (ARR의 절반)
    // 참고: CubeMX에서 해당 채널의 CCR 레지스터입니다. (CH1 기준)
    TIMx->CCR1 = (arr + 1) / 2;

    // 5. 멈춰있던 타이머를 깨워서 다시 펄스 발사 시작
    TIMx->CCER |= motor_channels[axis]; // 핀 출력 허용
    TIMx->CR1 |= TIM_CR1_CEN;           // 카운터 런(Run)
}

// ==========================================================
// 목표 속도가 0이거나 서보 OFF 시 STEP 펄스 즉시 중단
// ==========================================================
void stop_pulse_generator(uint8_t axis) {
    TIM_TypeDef *TIMx = motor_timers[axis];

    if(TIMx == NULL)
    	return;
    // 1. 해당 채널의 PWM 출력 핀을 즉시 끔 (하드웨어 레벨 차단)
    TIMx->CCER &= ~motor_channels[axis];

    // 2. 타이머 카운터 자체를 일시 정지 (선택 사항이지만 안전함)
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // 3. 내부 카운터를 0으로 초기화하여 다음 출발 시 깔끔하게 나가도록 세팅
    TIMx->CNT = 0;
}

// ==========================================================
// ⭐️ 1ms 주기 현재 위치 누적 업데이트 함수
// ==========================================================
void Update_Actual_Position(uint8_t axis) {
    // 1. 현재 속도(Hz)를 누적기에 더함 (+방향이면 더해지고, -방향이면 빼짐)
    g_pos_accumulator[axis] += g_current_speed[axis];

    // 2. 1000ms(1초) 기준이므로, 1000으로 나눈 몫이 1ms 동안 출력된 "온전한 펄스 개수"가 됨
    int32_t delta_pulse = g_pos_accumulator[axis] / 1000;

    // 3. 실제 위치 변수에 이동한 펄스 수 반영
    g_Actual_Pos[axis] += delta_pulse;

    // 4. 온전한 펄스로 바꾸고 "남은 소수점(나머지)"은 다시 보관하여 다음 1ms에 합산!
    // (이렇게 해야 펄스 유실이 0%가 됩니다)
    g_pos_accumulator[axis] %= 1000;
}

// ==========================================================
// ⭐️ 위치 초기화 함수 (호밍 / 영점 셋업 시 호출)
// ==========================================================
void Reset_Actual_Position(uint8_t axis) {
    g_Actual_Pos[axis] = 0;
    g_pos_accumulator[axis] = 0; // 누적기 찌꺼기도 반드시 같이 비워주어야 함
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
