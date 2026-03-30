/*
 * global.h
 *
 *  Created on: Mar 23, 2026
 *      Author: cubox
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#define MAX_AXIS 4
#define ESC_AL_STATUS_OP	8
#define TIM_CLOCK_FREQ  240000000

//현재 모터 위치(보낸 펄스 수)
extern int32_t g_Actual_Pos[MAX_AXIS];
//모터 구동 최고 속도
extern int32_t g_MaxSpeed[MAX_AXIS];
extern int32_t g_current_speed[MAX_AXIS];
extern int32_t g_current_accel[MAX_AXIS]; // S-Curve용 현재 가속도 상태
//CiA402(0-15bit) + TMC2208 Register(16-31bit)
extern uint32_t g_StatusWord[MAX_AXIS];

//TMC2208 Register 32bit
extern uint32_t g_drv_status[MAX_AXIS];

//Motor position is Home now?
extern bool g_is_homed[MAX_AXIS];

//Hardware Error Flag for each Axis
extern volatile bool g_hwErr[MAX_AXIS];

//Warning that over pre-limit temp
extern bool gw_overtemp_pre[MAX_AXIS];
//Warning that over limit temp
extern bool gw_overtemp[MAX_AXIS];
//Warning that over temp 120°C
extern bool gw_overtemp_120[MAX_AXIS];
//Warning that over temp 143°C
extern bool gw_overtemp_143[MAX_AXIS];
//Warning that over temp 150°C
extern bool gw_overtemp_150[MAX_AXIS];
//Warning that over temp 157°C
extern bool gw_overtemp_157[MAX_AXIS];

//Warning that short GND A Phase
extern bool gw_shortGND_A[MAX_AXIS];
//Warning that short GND B Phase
extern bool gw_shortGND_B[MAX_AXIS];
//Warning that MOSFET short
extern bool gw_shortMos[MAX_AXIS];

//Warning that Open load Phase A
extern bool gw_openload_A[MAX_AXIS];
//Warning that Open load Phase B
extern bool gw_openload_B[MAX_AXIS];

//Actual motor current
extern uint16_t g_motcur[MAX_AXIS];

//State of Stealth => 0 : SpreadMode, 1: StealthMode
extern bool g_stealthMode[MAX_AXIS];

//Motor GPIO
extern GPIO_TypeDef* g_MotPORT[MAX_AXIS];
extern GPIO_TypeDef* DIR_PORT[MAX_AXIS];
extern uint16_t g_MotPIN[MAX_AXIS];
extern uint16_t DIR_PIN[MAX_AXIS];
extern TIM_TypeDef* motor_timers[MAX_AXIS];

//Motor Position
extern int32_t g_pos_accumulator[MAX_AXIS];
#endif /* INC_GLOBAL_H_ */
