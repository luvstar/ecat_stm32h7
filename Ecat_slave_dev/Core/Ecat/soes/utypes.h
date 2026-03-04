#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"
#include <stdint.h>

// 가독성을 높이기 위한 가감속 패턴 매크로 (TwinCAT 마스터에서도 동일한 숫자로 제어)
#define PROFILE_NONE        0  // 가감속 없음 (즉시 목표 속도 도달)
#define PROFILE_TRAPEZOIDAL 1  // 사다리꼴 패턴 (등가속도)
#define PROFILE_S_CURVE     2  // S자 패턴 (Jerk 제어, 부드러운 구동)

#define DIR_CW              0  // 정방향 (+)
#define DIR_CCW             1  // 역방향 (-)

// 구조체 패딩(Padding) 방지 - 이더캣 통신에서 바이트 밀림을 막기 위해 필수!
#pragma pack(push, 1)

// 1. RxPDO: 마스터(TwinCAT) -> 슬레이브(STM32) (총 32 바이트)
typedef struct {
    uint32_t target_pos[4];      // 1~4축 목표 위치
    uint8_t  direction[4];       // 1~4축 이동 방향 (0: +, 1: -)
    uint8_t  target_speed[4];    // 1~4축 목표 속도 (0 ~ 100%)
    uint8_t  accel_pattern[4];   // 1~4축 가감속 패턴 (0: None, 1: Trap, 2: S-curve)
    uint16_t control_word;       // 전체 모터 제어 상태 (Enable, Reset 등)
    uint16_t dummy;
} _Rbuffer;

// 2. TxPDO: 슬레이브(STM32) -> 마스터(TwinCAT) (총 20 바이트)
typedef struct {
    uint32_t actual_pos[4];      // 1~4축 현재 위치 (엔코더 피드백)
    uint16_t status_word;        // 전체 모터 상태 (0=정지, 1=동작중, 2=에러 등)
    uint16_t error_code;         // 에러 코드
} _Wbuffer;

#pragma pack(pop)

// SOES 코어 스택에서 사용할 전역 변수 선언
extern _Rbuffer Rb;
extern _Wbuffer Wb;

#endif /* __UTYPES_H__ */

