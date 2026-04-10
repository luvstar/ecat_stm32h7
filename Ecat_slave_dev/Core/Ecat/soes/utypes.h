#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"
#include <stdint.h>

#define PROFILE_NONE        0
#define PROFILE_TRAPEZOIDAL 1
#define PROFILE_S_CURVE     2

#define DIR_CW              0
#define DIR_CCW             1

#pragma pack(push, 1) // ⭐️ 강제 1바이트 정렬

// ⭐️ 1축당 정확히 24 Bytes (더미 포함) x 4축 = 96 Bytes
//typedef struct {
//    uint16_t control_word;
//    int32_t  target_pos;
//    int32_t  target_speed;
//    uint32_t max_speed;
//    uint32_t accel_time;
//    uint32_t decel_time;
//    uint8_t  speed_pattern;
//    uint8_t  dummy;          // TwinCAT 메모리 정렬(Padding) 강제 대응용!
//} AxisTarget_t;
typedef struct {
    int32_t  target_pos;     // Offset 0
    int32_t  target_speed;   // Offset 4
    uint32_t max_speed;      // Offset 8
    uint32_t accel_time;     // Offset 12
    uint32_t decel_time;     // Offset 16
    uint16_t control_word;   // Offset 20
    uint8_t  speed_pattern;  // Offset 22
    uint8_t  dummy;   // Offset 23 (배열 연속성을 위해 구조체 전체 크기를 24바이트로 맞추는 용도)
} AxisTarget_t;

typedef struct {
    AxisTarget_t axis[4];
} _Rbuffer;

// ⭐️ 1축당 8 Bytes x 4축 = 32 Bytes
typedef struct {
    uint16_t status_word;
    int32_t  actual_pos;
    uint16_t error_code;
} AxisActual_t;

typedef struct {
    AxisActual_t axis[4];
} _Wbuffer;

#pragma pack(pop)

typedef struct {
    uint32_t gear_ratio[4];
} _Cbuffer;

extern _Rbuffer Rb;
extern _Wbuffer Wb;
extern _Cbuffer Cb;

#endif /* __UTYPES_H__ */
