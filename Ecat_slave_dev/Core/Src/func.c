/*
 * func.c
 * TIM3 하드웨어 인코더 피드백 + 오버슛 자동 교정 폐루프(Closed-loop) 제어
 */
#include "func.h"
#include "global.h"
#include "stdlib.h"
#include <stdbool.h>
#include <math.h>

#define MOTOR_STEPS_PER_REV  200
#define MICROSTEPPING        16
#define DEG_PER_REV          360

const uint32_t motor_channels[4] = {TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E};

int32_t g_current_speed[4] = {0, 0, 0, 0};
int32_t g_MaxSpeed[MAX_AXIS] = {0, };

volatile int32_t g_target_pos_pulses[MAX_AXIS] = {0};
volatile int32_t g_actual_pos_pulses[MAX_AXIS] = {0};
volatile bool g_is_moving[MAX_AXIS] = {false};

// ⭐️ TIM3의 이전 카운트 값을 기억할 변수
volatile uint16_t g_last_tim3_cnt[MAX_AXIS] = {21};

GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOF, GPIOF, GPIOF};
uint16_t g_MotPIN[MAX_AXIS]       = {GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
GPIO_TypeDef* DIR_PORT[MAX_AXIS]  = {GPIOB, GPIOF, GPIOF, GPIOF};
uint16_t DIR_PIN[MAX_AXIS]        = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};

TIM_TypeDef* motor_timers[4] = {TIM1, 0,};

void servo_on(int axis) {
    if(axis > 4) return;
    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_RESET);
}

void servo_off(int axis) {
    if(axis >= 4) return;
    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_SET);
    stop_pulse_generator(axis);
}

uint32_t motCtrl(uint8_t axis){
    TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
    TMC2209_WriteRegister(&huart1, axis, 0x10, 0x0007180E); // 강력한 정지 전류 유지
    TMC2209_WriteRegister(&huart1, axis, 0x6C, 0x14000053);
    TMC2209_WriteRegister(&huart1, axis, 0x22, 0);
    return 1;
}

uint8_t mot_MaxSpdUpdate(int axis){ return 1; }
uint8_t mot_ProfUpdate(int axis){ return 1; }
uint32_t get_MaxSpeed(int axis) { return 0; }

void CiA402_StateMachine(uint8_t axis) {
    uint32_t cw = Rb.axis[axis].control_word;
    if(axis >= 1) return;

    if ((cw & 0x000F) == 0x000F) {
        if ((g_StatusWord[axis] & 0x0237) != 0x0237) motCtrl(axis);
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
}

// ⭐️ TIM3 기반 완벽한 절대 위치 추적 및 보정 함수
void Sync_Position_With_Hardware(uint8_t axis) {
    uint16_t current_tim3_cnt = TIM3->CNT;
    uint16_t delta = current_tim3_cnt - g_last_tim3_cnt[axis];

    if (delta > 0) {
        // 방향에 따라 더하거나 뺌
        if (HAL_GPIO_ReadPin(DIR_PORT[axis], DIR_PIN[axis]) == GPIO_PIN_RESET) {
            g_actual_pos_pulses[axis] += delta;
        } else {
            g_actual_pos_pulses[axis] -= delta;
        }
        g_last_tim3_cnt[axis] = current_tim3_cnt;
    }
}

void CalcMotion(int axis) {
    if(axis >= 1) return;

    bool current_start_bit = (Rb.axis[axis].control_word & 0x0010) != 0;

    // 매 1ms 루프마다 하드웨어 CCTV(TIM3)를 확인하여 위치를 완벽하게 동기화!
    Sync_Position_With_Hardware(axis);

    if (current_start_bit) {
        g_target_pos_pulses[axis] = (Rb.axis[axis].target_pos * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;

        int32_t error = g_target_pos_pulses[axis] - g_actual_pos_pulses[axis];
        int32_t remaining_dist = abs(error);

        if (remaining_dist > 0) {
            bool is_forward = (HAL_GPIO_ReadPin(DIR_PORT[axis], DIR_PIN[axis]) == GPIO_PIN_RESET);
            bool need_forward = (error > 0);

            uint32_t profile_v = (Rb.axis[axis].target_speed * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;
            uint32_t max_v     = (Rb.axis[axis].max_speed * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;
            uint32_t acc_time  = Rb.axis[axis].accel_time ? Rb.axis[axis].accel_time : 1;
            uint32_t dec_time  = Rb.axis[axis].decel_time ? Rb.axis[axis].decel_time : 1;

            if (profile_v > max_v) profile_v = max_v;
            if (profile_v < 16) profile_v = 16;

            int32_t dv_acc = max_v / acc_time;
            int32_t dv_dec = max_v / dec_time;
            if (dv_acc == 0) dv_acc = 1;
            if (dv_dec == 0) dv_dec = 1;

            uint32_t decel_rate_hz_s = dv_dec * 1000;
            int64_t v_sq = (int64_t)g_current_speed[axis] * g_current_speed[axis];
            int32_t decel_dist = v_sq / (2 * decel_rate_hz_s);
            decel_dist += (g_current_speed[axis] * 2) / 1000 + 5; // 예측 마진

            int32_t target_v = profile_v;

            // ⭐️ 오버슛 자동 교정 로직:
            // 만약 모터가 지나쳐버려서 반대 방향으로 돌아와야 한다면 초저속(16Hz)으로 징징징 후진해서 복귀!
            if (g_is_moving[axis] && (is_forward != need_forward)) {
                target_v = 16;
                if (g_current_speed[axis] <= 16) {
                    HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], need_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
                }
            } else {
                if (remaining_dist <= decel_dist) target_v = 16;
                if (!g_is_moving[axis]) {
                    HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], need_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
                }
            }

            if (!g_is_moving[axis]) {
                g_is_moving[axis] = true;
                g_current_speed[axis] = 16;

                // 출발 직전 TIM3 기준점 재설정
                g_last_tim3_cnt[axis] = TIM3->CNT;

                TIM_TypeDef *TIMx = motor_timers[axis];
                TIMx->SR = ~TIM_SR_UIF;
                TIMx->DIER |= TIM_DIER_UIE;
                NVIC_EnableIRQ(TIM1_UP_IRQn);
            }

            if (remaining_dist <= 10) {
                g_current_speed[axis] = 16;
                target_v = 16;
            } else {
                if (g_current_speed[axis] < target_v) {
                    g_current_speed[axis] += dv_acc;
                    if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v;
                } else if (g_current_speed[axis] > target_v) {
                    g_current_speed[axis] -= dv_dec;
                    if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v;
                }
            }

            update_timer_pwm_freq(axis, g_current_speed[axis]);

        } else {
            if (g_is_moving[axis]) stop_pulse_generator(axis);
        }
    } else {
        if (g_is_moving[axis]) stop_pulse_generator(axis);
    }

    Update_Actual_Position(axis);
}

void update_timer_pwm_freq(uint8_t axis, uint32_t freq_hz) {
    if (freq_hz == 0) return;
    TIM_TypeDef *TIMx = motor_timers[axis];
    if(TIMx == NULL) return;

    uint32_t actual_tim_clk = 240000000;
    uint32_t timer_base_freq = 1000000;
    uint32_t psc = (actual_tim_clk / timer_base_freq) - 1;

    uint32_t arr = (timer_base_freq / freq_hz);
    if (arr > 0) arr -= 1;
    if (arr > 65535) arr = 65535;

    TIMx->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIMx->CCMR1 |= (0x7 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIMx->CR1 |= TIM_CR1_ARPE;

    TIMx->PSC = psc;
    TIMx->ARR = arr;
    TIMx->CCR1 = (arr + 1) / 2;

    if ((TIMx->CR1 & TIM_CR1_CEN) == 0) {
        uint32_t uie_state = TIMx->DIER & TIM_DIER_UIE;
        TIMx->DIER &= ~TIM_DIER_UIE;
        TIMx->EGR = TIM_EGR_UG;
        TIMx->SR = ~TIM_SR_UIF;
        TIMx->DIER |= uie_state;
    }

    if (TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;
    TIMx->CCER |= motor_channels[axis];
    TIMx->CR1 |= TIM_CR1_CEN;
}

void stop_pulse_generator(uint8_t axis) {
    TIM_TypeDef *TIMx = motor_timers[axis];
    if(TIMx == NULL) return;
    TIMx->CCER &= ~motor_channels[axis];
    TIMx->CR1 &= ~TIM_CR1_CEN;
    g_current_speed[axis] = 0;
    g_is_moving[axis] = false;
}

void Update_Actual_Position(uint8_t axis) {
    g_Actual_Pos[axis] = (g_actual_pos_pulses[axis] * DEG_PER_REV) / (MOTOR_STEPS_PER_REV * MICROSTEPPING);
}

void Reset_Actual_Position(uint8_t axis) {
    g_Actual_Pos[axis] = 0;
    g_actual_pos_pulses[axis] = 0;
    g_target_pos_pulses[axis] = 0;
    g_last_tim3_cnt[axis] = TIM3->CNT; // 리셋할 때 TIM3 기준점도 같이 리셋
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

// ⭐️ 인터럽트 함수도 진실된 TIM3 데이터를 쓰도록 변경
void Motor_Pulse_ISR(uint8_t axis) {
    if (g_is_moving[axis]) {
        TIM_TypeDef *TIMx = motor_timers[axis];

        if (TIMx->SR & TIM_SR_UIF) {
            TIMx->SR = ~TIM_SR_UIF;

            // 멍청한 ++ 카운팅 삭제! TIM3의 진실된 변화량만 읽어옵니다.
            Sync_Position_With_Hardware(axis);

            // ⭐️ 오버슛 안전 정지 로직
            bool is_forward = (HAL_GPIO_ReadPin(DIR_PORT[axis], DIR_PIN[axis]) == GPIO_PIN_RESET);

            // 정방향 주행 중이면서 목표치를 넘었거나, 역방향 주행 중 목표치를 넘었을 때 즉시 차단!
            if ((is_forward && g_actual_pos_pulses[axis] >= g_target_pos_pulses[axis]) ||
                (!is_forward && g_actual_pos_pulses[axis] <= g_target_pos_pulses[axis])) {

                TIMx->CR1 &= ~TIM_CR1_CEN;
                TIMx->CCER &= ~TIM_CCER_CC1E;
                g_is_moving[axis] = false;
                g_current_speed[axis] = 0;
            }
        }
    }
}

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void delay_us(uint32_t us) {
    uint32_t count = us * (HAL_RCC_GetHCLKFreq() / 1000000) / 10;
    while(count--) { __NOP(); }
}
uint32_t DWT_GetTick(void) { return DWT->CYCCNT; }

///*
// * func.c
// * 스마트 서보 드라이브 (PWM Mode 2 적용, 누적 오차 0% 완벽 제어)
// */
//#include "func.h"
//#include "global.h"
//#include "stdlib.h"
//#include <stdbool.h>
//
//#define MOTOR_STEPS_PER_REV  200
//#define MICROSTEPPING        16
//#define DEG_PER_REV          360
//
//const uint32_t motor_channels[4] = {TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E, TIM_CCER_CC1E};
//
//int32_t g_current_speed[4] = {0, 0, 0, 0};
//int32_t g_MaxSpeed[MAX_AXIS] = {0, };
//
//volatile int32_t g_target_pos_pulses[MAX_AXIS] = {0};
//volatile int32_t g_actual_pos_pulses[MAX_AXIS] = {0};
//volatile bool g_is_moving[MAX_AXIS] = {false};
//
//// 핀 매핑
//GPIO_TypeDef* g_MotPORT[MAX_AXIS] = {GPIOB, GPIOF, GPIOF, GPIOF};
//uint16_t g_MotPIN[MAX_AXIS]       = {GPIO_PIN_2, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
//GPIO_TypeDef* DIR_PORT[MAX_AXIS]  = {GPIOB, GPIOF, GPIOF, GPIOF};
//uint16_t DIR_PIN[MAX_AXIS]        = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_0};
//
//TIM_TypeDef* motor_timers[4] = {TIM1, 0,};
//
//void servo_on(int axis) {
//    if(axis > 4) return;
//    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_RESET);
//}
//
//void servo_off(int axis) {
//    if(axis >= 4) return;
//    HAL_GPIO_WritePin(g_MotPORT[axis], g_MotPIN[axis], GPIO_PIN_SET);
//    stop_pulse_generator(axis);
//    g_current_speed[axis] = 0;
//    g_is_moving[axis] = false;
//}
//
//uint32_t motCtrl(uint8_t axis){
//    TMC2209_WriteRegister(&huart1, axis, 0x00, 0x00000080);
//    TMC2209_WriteRegister(&huart1, axis, 0x10, 0x0007180E);
//    TMC2209_WriteRegister(&huart1, axis, 0x6C, 0x14000053);
//    TMC2209_WriteRegister(&huart1, axis, 0x22, 0);
//    return 1;
//}
//
//uint8_t mot_MaxSpdUpdate(int axis){ return 1; }
//uint8_t mot_ProfUpdate(int axis){ return 1; }
//uint32_t get_MaxSpeed(int axis) { return 0; }
//
//void CiA402_StateMachine(uint8_t axis) {
//    uint32_t cw = Rb.axis[axis].control_word;
//    if(axis >= 1) return;
//
//    if ((cw & 0x000F) == 0x000F) {
//        if ((g_StatusWord[axis] & 0x0237) != 0x0237) motCtrl(axis);
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
//}
//// ⭐️ 관성 오버슛(탈조) 완벽 방어형 위치 제어
//void CalcMotion(int axis) {
//    if(axis >= 1) return;
//
//    bool current_start_bit = (Rb.axis[axis].control_word & 0x0010) != 0;
//
//    if (current_start_bit) {
//        g_target_pos_pulses[axis] = (Rb.axis[axis].target_pos * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;
//
//        int32_t error = g_target_pos_pulses[axis] - g_actual_pos_pulses[axis];
//        int32_t remaining_dist = abs(error);
//
//        if (remaining_dist > 0) {
//            bool is_forward = (HAL_GPIO_ReadPin(DIR_PORT[axis], DIR_PIN[axis]) == GPIO_PIN_RESET);
//            bool need_forward = (error > 0);
//
//            uint32_t profile_v = (Rb.axis[axis].target_speed * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;
//            uint32_t max_v     = (Rb.axis[axis].max_speed * MOTOR_STEPS_PER_REV * MICROSTEPPING) / DEG_PER_REV;
//            uint32_t acc_time  = Rb.axis[axis].accel_time ? Rb.axis[axis].accel_time : 1;
//            uint32_t dec_time  = Rb.axis[axis].decel_time ? Rb.axis[axis].decel_time : 1;
//
//            if (profile_v > max_v) profile_v = max_v;
//            if (profile_v < 16) profile_v = 16;
//
//            int32_t dv_acc = max_v / acc_time;
//            int32_t dv_dec = max_v / dec_time;
//            if (dv_acc == 0) dv_acc = 1;
//            if (dv_dec == 0) dv_dec = 1;
//
//            // 1. 순수 제동 거리 계산 (물리 공식)
//            uint32_t decel_rate_hz_s = dv_dec * 1000;
//            int64_t v_sq = (int64_t)g_current_speed[axis] * g_current_speed[axis];
//            int32_t decel_dist = v_sq / (2 * decel_rate_hz_s);
//
//            // ⭐️ 해결책 1: 예측 제동 (Look-ahead Margin)
//            // 1ms 제어 루프의 지연과 고속 관성을 고려하여, 현재 속도로 2ms 동안 날아갈 거리를 미리 더해줍니다.
//            int32_t safety_margin = (g_current_speed[axis] * 2) / 1000;
//            if (safety_margin < 5) safety_margin = 5; // 최소 5펄스는 미리 브레이크 밟기
//
//            decel_dist += safety_margin;
//
//            int32_t target_v = profile_v;
//
//            if (g_is_moving[axis] && (is_forward != need_forward)) {
//                target_v = 16;
//                if (g_current_speed[axis] <= 16) {
//                    HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], need_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
//                }
//            } else {
//                if (remaining_dist <= decel_dist) target_v = 16;
//                if (!g_is_moving[axis]) {
//                    HAL_GPIO_WritePin(DIR_PORT[axis], DIR_PIN[axis], need_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
//                }
//            }
//
//            if (!g_is_moving[axis]) {
//                g_is_moving[axis] = true;
//                g_current_speed[axis] = 16;
//
//                TIM_TypeDef *TIMx = motor_timers[axis];
//                TIMx->SR = ~TIM_SR_UIF;
//                TIMx->DIER |= TIM_DIER_UIE;
//                NVIC_EnableIRQ(TIM1_UP_IRQn);
//            }
//
//            // ⭐️ 해결책 2: 초저속 착륙 구역 (Creep Zone) 강제 진입
//            // 목표치까지 10펄스 이하로 남았다면, 가감속 로직을 무시하고 무조건 16Hz로 속도를 박아버립니다!
//            // 16Hz에서는 펄스 간격이 무려 62ms(0.062초)로 벌어지기 때문에, CPU가 아무리 바빠도 오버슛이 100% 차단됩니다.
//            if (remaining_dist <= 10) {
//                g_current_speed[axis] = 16;
//                target_v = 16;
//            } else {
//                // 일반 가감속
//                if (g_current_speed[axis] < target_v) {
//                    g_current_speed[axis] += dv_acc;
//                    if (g_current_speed[axis] > target_v) g_current_speed[axis] = target_v;
//                } else if (g_current_speed[axis] > target_v) {
//                    g_current_speed[axis] -= dv_dec;
//                    if (g_current_speed[axis] < target_v) g_current_speed[axis] = target_v;
//                }
//            }
//
//            update_timer_pwm_freq(axis, g_current_speed[axis]);
//
//        } else {
//            if (g_is_moving[axis]) stop_pulse_generator(axis);
//        }
//    } else {
//        if (g_is_moving[axis]) stop_pulse_generator(axis);
//    }
//
//    Update_Actual_Position(axis);
//}
