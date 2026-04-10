/*
 * func.c
 * Author: cubox
 */
#include "func.h"
#include "global.h"
#include "stdlib.h"
#include <stdbool.h>

// --- [추가된 부분] mm/s -> Hz 변환을 위한 기계적 상수 정의 ---
#define MOTOR_STEPS_PER_REV  200.0f  // 1.8도 스텝모터 기준 (1회전 = 200스텝)
#define MICROSTEPPING        16.0f   // TMC2209 마이크로스텝 설정값 (현재 설정에 맞게 변경)
#define MM_PER_REV           40.0f   // 1회전 시 직선 이동 거리 (예: 풀리 둘레 40mm 또는 리드스크류 피치)

// 1mm 이동하기 위해 필요한 펄스(Hz) 수 계산
#define PULSES_PER_MM        ((MOTOR_STEPS_PER_REV * MICROSTEPPING) / MM_PER_REV)
// -------------------------------------------------------------

const uint32_t motor_channels[4] = {TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E};

int32_t g_current_speed[4] = {0, 0, 0, 0};
int32_t g_current_accel[4] = {0, 0, 0, 0};

//방향 핀 : PB1, EN 핀 : PB2, PWM 핀 : PA8

int32_t g_MaxSpeed[MAX_AXIS] = {0, };

// ⭐️ 핀 매핑 수정 완료 ⭐️
GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOF, GPIOF, GPIOF}; // EN 포트
uint16_t g_MotPIN[MAX_AXIS]       = {GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0}; // EN 핀 (PB2)

GPIO_TypeDef* DIR_PORT[MAX_AXIS]  = {GPIOB, GPIOF, GPIOF, GPIOF}; // DIR 포트 (GPIOA -> GPIOB로 변경)
uint16_t DIR_PIN[MAX_AXIS]        = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0}; // DIR 핀 (PA8 -> PB1로 변경)

TIM_TypeDef* motor_timers[4] = {TIM1, 0,};
int32_t g_pos_accumulator[MAX_AXIS] = {0, 0, 0, 0};

void servo_on(int axis) {
    if(axis > 4) return;
    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_RESET);
}

void servo_off(int axis) {
    if(axis >= 4) return;
    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_SET);
    stop_pulse_generator(axis);
    g_current_speed[axis] = 0;
}

uint32_t motCtrl(uint8_t axis){
    // 1. GCONF 설정 (내부 제어 활성화)
    TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
    // 2. 모터 구동/유지 전류값 설정
    TMC2209_WriteRegister(&huart1, axis, 0x10, 0x00070E03);

    // ⭐️ 3. [추가] CHOPCONF 레지스터 설정 (16 마이크로스텝 강제 고정)
    // 이 코드가 없으면 256스텝으로 동작해 눈에 안 보일 정도로 느리게 돕니다.
    TMC2209_WriteRegister(&huart1, axis, 0x6C, 0x14000053);

    // 4. VACTUAL 초기화
    TMC2209_WriteRegister(&huart1, axis, 0x22, 0);
    return 1;
}

uint8_t mot_MaxSpdUpdate(int axis){
    uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
    if (ESCvar.ALstatus == ESC_OP) {
        if (Rb.axis[axis].target_speed != g_MaxSpeed[axis]) {
            TMC2209_WriteRegister(&huart1, axis, 0x22, Rb.axis[axis].target_speed);
            g_MaxSpeed[axis] = Rb.axis[axis].target_speed;
        }
    } else {
        if (g_MaxSpeed[axis] != 0) {
            servo_off(axis);
            g_MaxSpeed[axis] = 0;
        }
    }
    uint32_t count_after = TMC2209_ReadRegister(&huart1, 0x00, 0x02);
    return count_before != count_after ? 1 : -1;
}

uint8_t mot_ProfUpdate(int axis){
    uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
    uint32_t count_after = TMC2209_ReadRegister(&huart1, axis, 0x02);
    return count_before != count_after ? 1 : -1;
}

uint32_t get_MaxSpeed(int axis) {
    return g_MaxSpeed[axis];
}

void CiA402_StateMachine(uint8_t axis) {
    static uint32_t prev_cw[MAX_AXIS] = {0,};
    uint32_t cw = Rb.axis[axis].control_word;
    uint32_t sw = g_StatusWord[axis];
    if(axis >= 1) return;

    if (g_hwErr[axis] == true) {
        servo_off(axis);
        g_StatusWord[axis] = (g_StatusWord[axis] & 0xFFF0) | 0x0008;
        return;
    }

    if (sw & (1 << 3)) {
        if (((cw & (1 << 7)) != 0) && ((prev_cw[axis] & (1 << 7)) == 0)) {
            g_StatusWord[axis] = 0x0250;
        }
        servo_off(axis);
        prev_cw[axis] = cw;
        return;
    }

    if ((cw & 0x0002) == 0x0000) {
        servo_off(axis);
        g_StatusWord[axis] = 0x0250;
    } else if ((cw & 0x0006) == 0x0002) {
        servo_off(axis);
        g_StatusWord[axis] = 0x0250;
    } else if ((cw & 0x0007) == 0x0006) {
        servo_off(axis);
        g_StatusWord[axis] = 0x0231;
    } else if ((cw & 0x000F) == 0x0007) {
        servo_off(axis);
        g_StatusWord[axis] = 0x0233;
    } else if ((cw & 0x000F) == 0x000F) {
        // 최초로 Operation Enable 상태로 진입하는 순간 TMC2209 초기화!
        if ((g_StatusWord[axis] & 0x0237) != 0x0237) {
            motCtrl(axis);            // 코일 전류 세팅
            g_current_speed[axis] = 0; // 안전을 위해 현재 속도 초기화
        }
        servo_on(axis);
        g_StatusWord[axis] = 0x0237;
    } else {
        servo_off(axis);
        g_StatusWord[axis] = 0x0250;
    }

    if ((g_StatusWord[axis] & 0x0237) == 0x0237) {
        if (cw & (1 << 11)) {
            Reset_Actual_Position(axis);
            g_is_homed[axis] = true;
        }
    }
    prev_cw[axis] = cw;
}

void CalcMotion(int axis) {
    if(axis >= 1) return;

    // --- 동작 시작 명령 비트(Bit 4) 검사 ---
    bool is_start_cmd_on = (Rb.axis[axis].control_word & 0x0010) != 0;

//    // 🚨 [임시 하드코딩 테스트] TwinCAT 변수 매핑 오류를 무시하고 강제로 속도 주입
//    int32_t target_v = 0;
//    uint32_t max_v = (uint32_t)(100 * PULSES_PER_MM); // 강제 100mm/s
//    uint32_t acc_time = 1000; // 강제 1000ms
//    uint32_t dec_time = 1000; // 강제 1000ms

    int32_t target_v = Rb.axis[axis].target_speed; // ⭐️ 변경점
    uint32_t max_v = Rb.axis[axis].max_speed;
    uint32_t acc_time = Rb.axis[axis].accel_time;
    uint32_t dec_time = Rb.axis[axis].decel_time;

    if (is_start_cmd_on) {
        target_v = (int32_t)(20 * PULSES_PER_MM); // 강제 20mm/s로 회전
    }

    // ⭐️ 기존 클램핑 로직 그대로 유지
    if (max_v == 0) {
        target_v = 0;
    } else {
        if (target_v > (int32_t)max_v) {
            target_v = (int32_t)max_v;
        } else if (target_v < -(int32_t)max_v) {
            target_v = -(int32_t)max_v;
        }
    }

    int32_t dv_acc = 1;
    int32_t dv_dec = 1;

    if (max_v > 0) {
        dv_acc = (acc_time > 0) ? (max_v / acc_time) : max_v;
        dv_dec = (dec_time > 0) ? (max_v / dec_time) : max_v;
        if (dv_acc == 0) dv_acc = 1;
        if (dv_dec == 0) dv_dec = 1;
    }

    bool is_accelerating = false;
    if (target_v > g_current_speed[axis]) {
        is_accelerating = (g_current_speed[axis] >= 0);
    } else if (target_v < g_current_speed[axis]) {
        is_accelerating = (g_current_speed[axis] <= 0);
    }

    int32_t target_dv = is_accelerating ? dv_acc : dv_dec;
    int32_t applied_dv = 0;


    uint8_t pattern = Rb.axis[axis].speed_pattern;

    if (pattern == 0) {
        applied_dv = target_dv;
        g_current_accel[axis] = target_dv;
    } else {
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

    if (g_current_speed[axis] < target_v) {
        g_current_speed[axis] += applied_dv;
        if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v;
    }
    else if (g_current_speed[axis] > target_v) {
        g_current_speed[axis] -= applied_dv;
        if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v;
    }

    if (g_current_speed[axis] == target_v) {
        g_current_accel[axis] = 0;
    }

    bool is_negative_speed = (g_current_speed[axis] < 0);
    bool is_reverse_bit_on = (Rb.axis[axis].control_word & (1 << 15)) != 0;

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

void update_timer_pwm_freq(uint8_t axis, uint32_t freq_hz) {
    if (freq_hz == 0) {
        stop_pulse_generator(axis);
        return;
    }
    TIM_TypeDef *TIMx = motor_timers[axis];
    if(TIMx == NULL) return;

    // 1. 고정 프리스케일러 (타이머를 1MHz 속도로 고정)
    uint32_t timer_base_freq = 1000000; // 1MHz
    uint32_t psc = (TIM_CLOCK_FREQ / timer_base_freq) - 1;

    // 2. 주파수에 따른 ARR 계산 (오직 ARR만 바꿈)
    uint32_t arr = (timer_base_freq / freq_hz);
    if (arr > 0) arr -= 1;
    if (arr > 65535) arr = 65535; // 최저 속도 하한선 방어

    TIMx->PSC = psc;
    TIMx->ARR = arr;
    TIMx->CCR1 = (arr + 1) / 2;

    // 3. 타이머가 멈춰있을 때만 강제 리셋 (1ms마다 펄스가 죽는 현상 완벽 방지)
    if ((TIMx->CR1 & TIM_CR1_CEN) == 0) {
        TIMx->EGR = TIM_EGR_UG;
    } else {
        // 돌고 있는데 새로운 목표값이 현재 카운트보다 낮아지면 헛도는 것 방지
        if (TIMx->CNT > arr) {
            TIMx->EGR = TIM_EGR_UG;
        }
    }

    // 고급 타이머(TIM1) 핀 출력 강제 허용
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    TIMx->CCER |= motor_channels[axis];
    TIMx->CR1 |= TIM_CR1_CEN;
}

void stop_pulse_generator(uint8_t axis) {
    TIM_TypeDef *TIMx = motor_timers[axis];
    if(TIMx == NULL) return;
    TIMx->CCER &= ~motor_channels[axis];
    TIMx->CR1 &= ~TIM_CR1_CEN;
    TIMx->CNT = 0;
}

void Update_Actual_Position(uint8_t axis) {
    g_pos_accumulator[axis] += g_current_speed[axis];
    int32_t delta_pulse = g_pos_accumulator[axis] / 1000;
    g_Actual_Pos[axis] += delta_pulse;
    g_pos_accumulator[axis] %= 1000;
}

void Reset_Actual_Position(uint8_t axis) {
    g_Actual_Pos[axis] = 0;
    g_pos_accumulator[axis] = 0;
}

void ec_valinit() {
    for(int i = 0; i < MAX_AXIS; i++){
        g_StatusWord[i] = 0x250;
        g_is_homed[i] = false;
        g_drv_status[i] = 0;
        gw_overtemp_pre[i] = false;
        g_MaxSpeed[i] = 0xFF;
    }
}

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t count = us * (HAL_RCC_GetHCLKFreq() / 1000000) / 10; // 계수는 CPU마다 조정 필요
    while(count--) {
        __NOP(); // 아무 동작도 하지 않음 (최적화 방지)
    }
}

uint32_t DWT_GetTick(void) {
    return DWT->CYCCNT;
}


///*
// * func.c
// * Author: cubox
// */
//#include "func.h"
//#include "global.h"
//#include "stdlib.h"
//#include <stdbool.h>
//
//// --- [추가된 부분] mm/s -> Hz 변환을 위한 기계적 상수 정의 ---
//#define MOTOR_STEPS_PER_REV  200.0f  // 1.8도 스텝모터 기준 (1회전 = 200스텝)
//#define MICROSTEPPING        16.0f   // TMC2209 마이크로스텝 설정값 (현재 설정에 맞게 변경)
//#define MM_PER_REV           40.0f   // 1회전 시 직선 이동 거리 (예: 풀리 둘레 40mm 또는 리드스크류 피치)
//
//// 1mm 이동하기 위해 필요한 펄스(Hz) 수 계산
//#define PULSES_PER_MM        ((MOTOR_STEPS_PER_REV * MICROSTEPPING) / MM_PER_REV)
//// -------------------------------------------------------------
//
//const uint32_t motor_channels[4] = {TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E};
//
//int32_t g_current_speed[4] = {0, 0, 0, 0};
//int32_t g_current_accel[4] = {0, 0, 0, 0};
//
////방향 핀 : PB1, EN 핀 : PB2, PWM 핀 : PA8
//
//int32_t g_MaxSpeed[MAX_AXIS] = {0, };
//GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOF, GPIOF, GPIOF};
//GPIO_TypeDef* DIR_PORT[MAX_AXIS] = {GPIOA, GPIOF, GPIOF, GPIOF};
//uint16_t g_MotPIN[MAX_AXIS] = {GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
//uint16_t DIR_PIN[MAX_AXIS] = {GPIO_PIN_8, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
//
//TIM_TypeDef* motor_timers[4] = {TIM1, 0,};
//int32_t g_pos_accumulator[MAX_AXIS] = {0, 0, 0, 0};
//
//void servo_on(int axis) {
//	if(axis > 4) return;
//	HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_RESET);
//}
//
//void servo_off(int axis) {
//	if(axis >= 4) return;
//	HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_SET);
//	stop_pulse_generator(axis);
//	g_current_speed[axis] = 0;
//}
//
////uint32_t motCtrl(uint8_t axis){ // motor active test code
////	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
////	TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
////	TMC2209_WriteRegister(&huart1, axis, 0x10, 0x00070E03);// 전류값 설정
////	TMC2209_WriteRegister(&huart1, axis, 0x22, -200000);
////	uint32_t count_after = TMC2209_ReadRegister(&huart1, axis, 0x02);
////	return count_before != count_after ? 1 : 0;
////}
//
//uint32_t motCtrl(uint8_t axis){
//    // GCONF 설정
//    TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
//    // ⭐️ 모터 구동/유지 전류값 설정 (필수)
//    TMC2209_WriteRegister(&huart1, axis, 0x10, 0x00070E03);
//    // ⭐️ VACTUAL 초기화: Step/Dir 제어를 위해 내부 발생기는 반드시 0으로 꺼야 합니다.
//    TMC2209_WriteRegister(&huart1, axis, 0x22, 0);
//    return 1;
//}
//
//uint8_t mot_MaxSpdUpdate(int axis){
//	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
//    if (ESCvar.ALstatus == ESC_OP) {
//        if (Rb.axis[axis].target_speed != g_MaxSpeed[axis]) { // ⭐️ 변경점
//        	TMC2209_WriteRegister(&huart1, axis, 0x22, Rb.axis[axis].target_speed);
//        	g_MaxSpeed[axis] = Rb.axis[axis].target_speed;
//        }
//    } else {
//        if (g_MaxSpeed[axis] != 0) {
//        	servo_off(axis);
//        	g_MaxSpeed[axis] = 0;
//        }
//    }
//	uint32_t count_after = TMC2209_ReadRegister(&huart1, 0x00, 0x02);
//	return count_before != count_after ? 1 : -1;
//}
//
//uint8_t mot_ProfUpdate(int axis){
//	uint32_t count_before = TMC2209_ReadRegister(&huart1, axis, 0x02);
//	uint32_t count_after = TMC2209_ReadRegister(&huart1, axis, 0x02);
//	return count_before != count_after ? 1 : -1;
//}
//
//uint32_t get_MaxSpeed(int axis) {
//	return g_MaxSpeed[axis];
//}
//
//void CiA402_StateMachine(uint8_t axis) {
//	static uint32_t prev_cw[MAX_AXIS] = {0,};
//    uint32_t cw = Rb.axis[axis].control_word;  // ⭐️ 변경점
//    uint32_t sw = g_StatusWord[axis];
//	if(axis >= 1) return;
//
//    if (g_hwErr[axis] == true) {
//        servo_off(axis);
//        g_StatusWord[axis] = (g_StatusWord[axis] & 0xFFF0) | 0x0008;
//        return;
//    }
//
//    if (sw & (1 << 3)) {
//        if (((cw & (1 << 7)) != 0) && ((prev_cw[axis] & (1 << 7)) == 0)) {
//            g_StatusWord[axis] = 0x0250;
//        }
//        servo_off(axis);
//        prev_cw[axis] = cw;
//        return;
//    }
//
//    if ((cw & 0x0002) == 0x0000) {
//        servo_off(axis);
//        g_StatusWord[axis] = 0x0250;
//    } else if ((cw & 0x0006) == 0x0002) {
//        servo_off(axis);
//        g_StatusWord[axis] = 0x0250;
//    } else if ((cw & 0x0007) == 0x0006) {
//        servo_off(axis);
//        g_StatusWord[axis] = 0x0231;
//    } else if ((cw & 0x000F) == 0x0007) {
//        servo_off(axis);
//        g_StatusWord[axis] = 0x0233;
//    } else if ((cw & 0x000F) == 0x000F) {
//    	// ⭐️ 변경점: 최초로 Operation Enable 상태로 진입하는 순간 TMC2209 초기화!
//    	if ((g_StatusWord[axis] & 0x0237) != 0x0237) {
//    	    motCtrl(axis);            // 코일 전류 세팅
//    	    g_current_speed[axis] = 0; // 안전을 위해 현재 속도 초기화
//    	}
//        servo_on(axis);
//        g_StatusWord[axis] = 0x0237;
//    } else {
//        servo_off(axis);
//        g_StatusWord[axis] = 0x0250;
//    }
//
//    if ((g_StatusWord[axis] & 0x0237) == 0x0237) {
//        if (cw & (1 << 11)) {
//            Reset_Actual_Position(axis);
//            g_is_homed[axis] = true;
//        }
//    }
//    prev_cw[axis] = cw;
//}
//
//void CalcMotion(int axis) {
////	if(axis >= 1) return;
////
//////    int32_t target_v = Rb.axis[axis].target_speed; // ⭐️ 변경점
//////    uint32_t max_v = Rb.axis[axis].max_speed;
////	int32_t target_v = (int32_t)(Rb.axis[axis].target_speed * PULSES_PER_MM);
////	uint32_t max_v   = (uint32_t)(Rb.axis[axis].max_speed * PULSES_PER_MM);
////    uint32_t acc_time = Rb.axis[axis].accel_time;
////    uint32_t dec_time = Rb.axis[axis].decel_time;
////    uint8_t  pattern = Rb.axis[axis].speed_pattern;
////
////    int32_t dv_acc = (acc_time > 0) ? (max_v / acc_time) : max_v;
////    int32_t dv_dec = (dec_time > 0) ? (max_v / dec_time) : max_v;
////    if (dv_acc == 0) dv_acc = 1;
////    if (dv_dec == 0) dv_dec = 1;
////
////    bool is_accelerating = false;
////    if (target_v > g_current_speed[axis]) {
////        is_accelerating = (g_current_speed[axis] >= 0);
////    } else if (target_v < g_current_speed[axis]) {
////        is_accelerating = (g_current_speed[axis] <= 0);
////    }
////
////    int32_t target_dv = is_accelerating ? dv_acc : dv_dec;
////    int32_t applied_dv = 0;
////
////    if (pattern == 0) {
////        applied_dv = target_dv;
////        g_current_accel[axis] = target_dv;
////    } else {
////        int32_t dj = (target_dv > 10) ? (target_dv / 10) : 1;
////        if (g_current_accel[axis] < target_dv) {
////            g_current_accel[axis] += dj;
////            if (g_current_accel[axis] > target_dv) g_current_accel[axis] = target_dv;
////        } else if (g_current_accel[axis] > target_dv) {
////            g_current_accel[axis] -= dj;
////            if (g_current_accel[axis] < target_dv) g_current_accel[axis] = target_dv;
////        }
////        applied_dv = g_current_accel[axis];
////    }
////
////    if (g_current_speed[axis] < target_v) {
////        g_current_speed[axis] += applied_dv;
////        if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v;
////    }
////    else if (g_current_speed[axis] > target_v) {
////        g_current_speed[axis] -= applied_dv;
////        if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v;
////    }
////
////    if (g_current_speed[axis] == target_v) {
////            g_current_accel[axis] = 0;
////    }
//
//	if(axis >= 1) return;
//
//	// --- 동작 시작 명령 비트(Bit 4) 검사 ---
//	// TwinCAT에서 CW에 15(0x0F)를 넣으면 모터만 켜짐, 31(0x1F)을 넣어야 회전 시작.
//	bool is_start_cmd_on = (Rb.axis[axis].control_word & 0x0010) != 0;
//
//	int32_t target_v = 0;
//
//	// 동작 시작 비트가 활성화되었을 때만 사용자의 목표 속도를 읽어옵니다.
//	// 비트가 꺼지면 target_v가 0이 되므로 설정된 dcc 값에 맞춰 자연스럽게 감속 정지합니다.
//	if (is_start_cmd_on) {
//	    target_v = (int32_t)(Rb.axis[axis].target_speed * PULSES_PER_MM);
//	}
//	// mm/s -> Hz 변환 적용
//	//int32_t target_v = (int32_t)(Rb.axis[axis].target_speed * PULSES_PER_MM);
//	uint32_t max_v   = (uint32_t)(Rb.axis[axis].max_speed * PULSES_PER_MM);
//
//	// ⭐️ [안전 로직 1] Max Speed가 0이면 목표 속도도 0으로 강제 (순서 강제)
//	if (max_v == 0) {
//	    target_v = 0;
//	} else {
//	    // ⭐️ [안전 로직 2] 목표 속도가 최대 속도를 넘지 못하도록 클램핑(제한)
//	    if (target_v > (int32_t)max_v) {
//	        target_v = (int32_t)max_v;
//	    } else if (target_v < -(int32_t)max_v) {
//	        target_v = -(int32_t)max_v;
//	    }
//	}
//
//	uint32_t acc_time = Rb.axis[axis].accel_time;
//	uint32_t dec_time = Rb.axis[axis].decel_time;
//	uint8_t  pattern = Rb.axis[axis].speed_pattern;
//
//	// ⭐️ Max Speed가 0일 때 발생하던 가속도 계산 오류(0으로 나누기) 방지
//	int32_t dv_acc = 1;
//	int32_t dv_dec = 1;
//
//	if (max_v > 0) {
//	    dv_acc = (acc_time > 0) ? (max_v / acc_time) : max_v;
//	    dv_dec = (dec_time > 0) ? (max_v / dec_time) : max_v;
//	    if (dv_acc == 0) dv_acc = 1;
//	    if (dv_dec == 0) dv_dec = 1;
//	}
//
//	bool is_accelerating = false;
//	if (target_v > g_current_speed[axis]) {
//	    is_accelerating = (g_current_speed[axis] >= 0);
//	} else if (target_v < g_current_speed[axis]) {
//	    is_accelerating = (g_current_speed[axis] <= 0);
//	}
//
//	int32_t target_dv = is_accelerating ? dv_acc : dv_dec;
//	int32_t applied_dv = 0;
//
//	if (pattern == 0) {
//	    applied_dv = target_dv;
//	    g_current_accel[axis] = target_dv;
//	} else {
//	    int32_t dj = (target_dv > 10) ? (target_dv / 10) : 1;
//	    if (g_current_accel[axis] < target_dv) {
//	        g_current_accel[axis] += dj;
//	        if (g_current_accel[axis] > target_dv) g_current_accel[axis] = target_dv;
//	    } else if (g_current_accel[axis] > target_dv) {
//	        g_current_accel[axis] -= dj;
//	        if (g_current_accel[axis] < target_dv) g_current_accel[axis] = target_dv;
//	    }
//	    applied_dv = g_current_accel[axis];
//	}
//
//	if (g_current_speed[axis] < target_v) {
//	    g_current_speed[axis] += applied_dv;
//	    if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v;
//	}
//	else if (g_current_speed[axis] > target_v) {
//	    g_current_speed[axis] -= applied_dv;
//	    if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v;
//	}
//
//	if (g_current_speed[axis] == target_v) {
//	    g_current_accel[axis] = 0;
//	}
//
//    bool is_negative_speed = (g_current_speed[axis] < 0);
//    // ⭐️ 16비트 CW의 15번째 비트를 방향 반전용으로 사용
//    bool is_reverse_bit_on = (Rb.axis[axis].control_word & (1 << 15)) != 0;
//
//    if (is_negative_speed ^ is_reverse_bit_on) {
//        HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], GPIO_PIN_SET);
//    } else {
//        HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], GPIO_PIN_RESET);
//    }
//
//    uint32_t output_speed_hz = abs(g_current_speed[axis]);
//    if (output_speed_hz == 0) {
//        stop_pulse_generator(axis);
//    } else {
//        update_timer_pwm_freq(axis, output_speed_hz);
//    }
//
//    Update_Actual_Position(axis);
//}
//
//void update_timer_pwm_freq(uint8_t axis, uint32_t freq_hz) {
//    if (freq_hz == 0) {
//        stop_pulse_generator(axis);
//        return;
//    }
//    TIM_TypeDef *TIMx = motor_timers[axis];
//    if(TIMx == NULL) return;
//    uint32_t total_div = TIM_CLOCK_FREQ / freq_hz;
//    uint32_t psc = 0;
//    uint32_t arr = 0;
//
//    if (total_div <= 65536) {
//        psc = 0;
//        arr = total_div - 1;
//    } else {
//        psc = (total_div / 65536);
//        arr = (total_div / (psc + 1)) - 1;
//    }
//
//    TIMx->PSC = psc;
//    TIMx->ARR = arr;
//    TIMx->CCR1 = (arr + 1) / 2;
//
//    TIMx->EGR = TIM_EGR_UG;
//
//    TIMx->CCER |= motor_channels[axis];
//    TIMx->CR1 |= TIM_CR1_CEN;
//}
//
//void stop_pulse_generator(uint8_t axis) {
//    TIM_TypeDef *TIMx = motor_timers[axis];
//    if(TIMx == NULL) return;
//    TIMx->CCER &= ~motor_channels[axis];
//    TIMx->CR1 &= ~TIM_CR1_CEN;
//    TIMx->CNT = 0;
//}
//
//void Update_Actual_Position(uint8_t axis) {
//    g_pos_accumulator[axis] += g_current_speed[axis];
//    int32_t delta_pulse = g_pos_accumulator[axis] / 1000;
//    g_Actual_Pos[axis] += delta_pulse;
//    g_pos_accumulator[axis] %= 1000;
//}
//
//void Reset_Actual_Position(uint8_t axis) {
//    g_Actual_Pos[axis] = 0;
//    g_pos_accumulator[axis] = 0;
//}
//
//void ec_valinit() {
//	for(int i = 0; i < MAX_AXIS; i++){
//		g_StatusWord[i] = 0x250;
//		g_is_homed[i] = false;
//		g_drv_status[i] = 0;
//		gw_overtemp_pre[i] = false;
//		g_MaxSpeed[i] = 0xFF;
//	}
//}
//
//void DWT_Delay_Init(void) {
//    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//    DWT->CYCCNT = 0;
//    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//}
//
//void delay_us(uint32_t us) {
//    uint32_t count = us * (HAL_RCC_GetHCLKFreq() / 1000000) / 10; // 계수는 CPU마다 조정 필요
//    while(count--) {
//        __NOP(); // 아무 동작도 하지 않음 (최적화 방지)
//    }
//}
//
//uint32_t DWT_GetTick(void) {
//    return DWT->CYCCNT;
//}
