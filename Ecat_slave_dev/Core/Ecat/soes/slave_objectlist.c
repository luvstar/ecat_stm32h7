#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

_Rbuffer Rb;
_Wbuffer Wb;
_Cbuffer Cb;

#ifndef HW_REV
#define HW_REV "1.0"
#endif

#ifndef SW_REV
#define SW_REV "1.0"
#endif

// -------------------------------------------------------------
// [2] 통신 맵핑용 전역 변수 및 배열 선언
// -------------------------------------------------------------
uint8_t rxpdo_map_count = 33;  // 0x1600 매핑 개수
uint8_t txpdo_map_count = 6;   // 0x1A00 매핑 개수
uint8_t sm2_assign_count = 1;
uint8_t sm3_assign_count = 1;

uint8_t rxpdo_target_count = 25; // 0x7000 서브 인덱스 개수
uint8_t txpdo_actual_count = 6;  // 0x6000 서브 인덱스 개수
uint8_t config_param_count = 4;  // 0x8000 서브 인덱스 개수

uint16_t sm2_pdo_assign = 0x1600;
uint16_t sm3_pdo_assign = 0x1A00;

uint8_t sm_comm_type_count = 4;
uint8_t sm_comm_types[4] = {1, 2, 3, 4};

uint8_t id_count = 4;
uint32_t vendor_id = 0x1337;
uint32_t product_code = 1234;
uint32_t revision_num = 0;
uint32_t serial_num = 0;

// ⭐️ RxPDO (0x1600) 실제 매핑 데이터 배열 (끝자리가 20=32비트, 08=8비트)
// ⭐️ RxPDO (0x1600) 실제 매핑 데이터 배열 (총 33개)
uint32_t rxpdo_mapping_entries[33] = {
  0x70000120, 0x70000220, 0x70000320, 0x70000420, // Target Pos (32bit)
  0x70000520, 0x70000620, 0x70000720, 0x70000820, // Target Angle (32bit)
  0x70000920, 0x70000A20, 0x70000B20, 0x70000C20, // Target Speed (32bit)
  0x70000D20, 0x70000E20, 0x70000F20, 0x70001020, // Max Speed (32bit)
  0x70001120, 0x70001220, 0x70001320, 0x70001420, // Accel Time (32bit)
  0x70001520, 0x70001620, 0x70001720, 0x70001820, // Decel Time (32bit)
  0x70001908, 0x70001A08, 0x70001B08, 0x70001C08, // Control Mode (8bit)
  0x70001D08, 0x70001E08, 0x70001F08, 0x70002008, // Speed Pattern (8bit)
  0x70002120                                      // Control Word (32bit)
};

// ⭐️ TxPDO (0x1A00) 실제 매핑 데이터 배열
uint32_t txpdo_mapping_entries[6] = {
  0x60000120, 0x60000220, 0x60000320, 0x60000420, // Actual Pos (32bit * 4)
  0x60000520,                                     // Status Word (32bit * 1)
  0x60000610                                      // Error Code (16bit * 1)
};

// -------------------------------------------------------------
// [3] Object Dictionary 문자열 정의
// -------------------------------------------------------------
static const char acName1018[] = "Identity Object";
static const char acNameMapping[] = "PDO Mapping Entry";

static const char acName1600[] = "RxPDO Mapping";
static const char acName1A00[] = "TxPDO Mapping";
static const char acName1C00[] = "Sync Manager Comm Type";
static const char acName1C12[] = "Sync Manager 2 PDO Assign";
static const char acName1C13[] = "Sync Manager 3 PDO Assign";

static const char acName6000[] = "TxPDO 4-Axis Actual Data";
static const char acName7000[] = "RxPDO 4-Axis Target Data";
static const char acName8000[] = "Configuration Parameters";

// -------------------------------------------------------------
// [4] SDO 데이터 구조체 맵핑
// -------------------------------------------------------------
const _objd SDO1018[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RO, "Max SubIndex", 4, &id_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Vendor ID", 0, &vendor_id},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Product Code", 0, &product_code},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Revision", 0, &revision_num},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Serial", 0, &serial_num},
};

// ==========================================================
// ⭐️ 0x6000: TxPDO (Actual Data) - Status Word 32비트 적용
// ==========================================================
const _objd SDO6000[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Max SubIndex", 6, &txpdo_actual_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Actual Pos 1", 0, &Wb.actual_pos[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Actual Pos 2", 0, &Wb.actual_pos[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Actual Pos 3", 0, &Wb.actual_pos[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Actual Pos 4", 0, &Wb.actual_pos[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Status Word",  0, &Wb.status_word}, // 32비트로 변경
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Error Code",   0, &Wb.error_code},
};

// ==========================================================
// ⭐️ 0x7000: RxPDO (Target Data) - 신규 파라미터 통합
// ==========================================================
const _objd SDO7000[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Max SubIndex", 33, &rxpdo_target_count},
  // 목표 위치 (SubIndex 0x01 ~ 0x04)
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Pos 1", 0, &Rb.target_pos[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Pos 2", 0, &Rb.target_pos[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Pos 3", 0, &Rb.target_pos[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Pos 4", 0, &Rb.target_pos[3]},
  // 목표 각도 (SubIndex 0x05 ~ 0x08)
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Angle 1", 0, &Rb.target_angle[0]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Angle 2", 0, &Rb.target_angle[1]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Angle 3", 0, &Rb.target_angle[2]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Angle 4", 0, &Rb.target_angle[3]},
  // 목표 속도 (SubIndex 0x09 ~ 0x0C)
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Speed 1", 0, &Rb.target_speed[0]},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Speed 2", 0, &Rb.target_speed[1]},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Speed 3", 0, &Rb.target_speed[2]},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Target Speed 4", 0, &Rb.target_speed[3]},
  // 최대 속도 (SubIndex 0x0D ~ 0x10)
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Max Speed 1", 0, &Rb.max_speed[0]},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Max Speed 2", 0, &Rb.max_speed[1]},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Max Speed 3", 0, &Rb.max_speed[2]},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Max Speed 4", 0, &Rb.max_speed[3]},

  // ⭐️ 가속 시간 (SubIndex 0x11 ~ 0x14)
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Accel Time 1", 0, &Rb.accel_time[0]},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Accel Time 2", 0, &Rb.accel_time[1]},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Accel Time 3", 0, &Rb.accel_time[2]},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Accel Time 4", 0, &Rb.accel_time[3]},
  // ⭐️ 감속 시간 (SubIndex 0x15 ~ 0x18)
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Decel Time 1", 0, &Rb.decel_time[0]},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Decel Time 2", 0, &Rb.decel_time[1]},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Decel Time 3", 0, &Rb.decel_time[2]},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Decel Time 4", 0, &Rb.decel_time[3]},

  // 모터 제어 모드 (SubIndex 0x19 ~ 0x1C)
  {0x19, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Control Mode 1", 0, &Rb.control_mode[0]},
  {0x1A, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Control Mode 2", 0, &Rb.control_mode[1]},
  {0x1B, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Control Mode 3", 0, &Rb.control_mode[2]},
  {0x1C, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Control Mode 4", 0, &Rb.control_mode[3]},
  // 가감속 패턴 (SubIndex 0x1D ~ 0x20)
  {0x1D, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Speed Pattern 1", 0, &Rb.speed_pattern[0]},
  {0x1E, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Speed Pattern 2", 0, &Rb.speed_pattern[1]},
  {0x1F, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Speed Pattern 3", 0, &Rb.speed_pattern[2]},
  {0x20, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Speed Pattern 4", 0, &Rb.speed_pattern[3]},

  // 32비트 Control Word (SubIndex 0x21)
  {0x21, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Control Word", 0, &Rb.control_word},
};

// ==========================================================
// ⭐️ 0x8000: SDO Configuration Data - 감속비 (정적 파라미터)
// ==========================================================
const _objd SDO8000[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Max SubIndex", 4, &config_param_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Gear Ratio 1", 0, &Cb.gear_ratio[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Gear Ratio 2", 0, &Cb.gear_ratio[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Gear Ratio 3", 0, &Cb.gear_ratio[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Gear Ratio 4", 0, &Cb.gear_ratio[3]},
};

// ==========================================================
// 매핑 영역 (0x1600, 0x1A00, 0x1C00, 0x1C12, 0x1C13)
// ==========================================================
const _objd SDO1600[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Max SubIndex", 33, &rxpdo_map_count},

  // 목표 위치 (SubIndex 0x01 ~ 0x04) -> rxpdo_mapping_entries[0 ~ 3]
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[3]},

  // 목표 각도 (SubIndex 0x05 ~ 0x08) -> rxpdo_mapping_entries[4 ~ 7]
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[7]},

  // 목표 속도 (SubIndex 0x09 ~ 0x0C) -> rxpdo_mapping_entries[8 ~ 11]
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[8]},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[9]},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[10]},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[11]},

  // 최대 속도 (SubIndex 0x0D ~ 0x10) -> rxpdo_mapping_entries[12 ~ 15]
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[12]},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[13]},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[14]},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[15]},

  // ⭐️ 가속 시간 (SubIndex 0x11 ~ 0x14) -> rxpdo_mapping_entries[16 ~ 19]
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[16]},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[17]},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[18]},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[19]},

  // ⭐️ 감속 시간 (SubIndex 0x15 ~ 0x18) -> rxpdo_mapping_entries[20 ~ 23]
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[20]},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[21]},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[22]},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[23]},

  // 모터 제어 모드 (SubIndex 0x19 ~ 0x1C) -> rxpdo_mapping_entries[24 ~ 27]
  {0x19, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[24]},
  {0x1A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[25]},
  {0x1B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[26]},
  {0x1C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[27]},

  // 가감속 패턴 (SubIndex 0x1D ~ 0x20) -> rxpdo_mapping_entries[28 ~ 31]
  {0x1D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[28]},
  {0x1E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[29]},
  {0x1F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[30]},
  {0x20, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[31]},

  // 32비트 Control Word (SubIndex 0x21) -> rxpdo_mapping_entries[32]
  {0x21, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &rxpdo_mapping_entries[32]},
};

const _objd SDO1A00[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Max SubIndex", 6, &txpdo_map_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0, &txpdo_mapping_entries[5]},
};

const _objd SDO1C00[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Max SubIndex", 4, &sm_comm_type_count},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Comm Type SM0", 1, &sm_comm_types[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Comm Type SM1", 2, &sm_comm_types[1]},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Comm Type SM2", 3, &sm_comm_types[2]},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Comm Type SM3", 4, &sm_comm_types[3]},
};

const _objd SDO1C12[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, "Max SubIndex", 1, &sm2_assign_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, "PDO Mapping", 0x1600, &sm2_pdo_assign},
};

const _objd SDO1C13[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, "Max SubIndex", 1, &sm3_assign_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, "PDO Mapping", 0x1A00, &sm3_pdo_assign},
};

// -------------------------------------------------------------
// [5] 최종 SDO Object List (엔진 메인 포인터)
// -------------------------------------------------------------
const _objectlist SDOobjects[] = {
  {0x1018, OTYPE_RECORD,  4,  5, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 25, 26, acName1600, SDO1600}, // 개수 변경
  {0x1A00, OTYPE_RECORD,  6,  7, acName1A00, SDO1A00},
  {0x1C00, OTYPE_ARRAY,   4,  5, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY,   1,  2, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY,   1,  2, acName1C13, SDO1C13},
  {0x6000, OTYPE_RECORD,  6,  7, acName6000, SDO6000},
  {0x7000, OTYPE_RECORD, 25, 26, acName7000, SDO7000}, // 개수 변경
  {0x8000, OTYPE_RECORD,  4,  5, acName8000, SDO8000}, // 신규 감속비 오브젝트 추가
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};

//// -------------------------------------------------------------
//// [추가] SOES 내부 연산용 전역 변수 및 배열 선언 (NULL 방지)
//// -------------------------------------------------------------
//uint8_t rxpdo_map_count = 18;  // 0x1600의 SubIndex 개수
//uint8_t txpdo_map_count = 6;   // 0x1A00의 SubIndex 개수
//uint8_t sm2_assign_count = 1;  // 0x1C12에 할당된 PDO 개수
//uint8_t sm3_assign_count = 1;  // 0x1C13에 할당된 PDO 개수
//
//uint8_t rxpdo_target_count = 18; // 0x7000 개수
//uint8_t txpdo_actual_count = 6;  // 0x6000 개수
//
//uint16_t sm2_pdo_assign = 0x1600;
//uint16_t sm3_pdo_assign = 0x1A00;
//
//// SM 통신 타입 (0x1C00)
//uint8_t sm_comm_type_count = 4;
//uint8_t sm_comm_types[4] = {1, 2, 3, 4}; // SM0~SM3 통신 타입
//
//// Identity Object (0x1018) 변수
//uint8_t id_count = 4;
//uint32_t vendor_id = 0x1337;
//uint32_t product_code = 1234;
//uint32_t revision_num = 0;
//uint32_t serial_num = 0;
//
//// [핵심] RxPDO (0x1600) 실제 매핑 데이터 배열
//uint32_t rxpdo_mapping_entries[18] = {
//  0x70000120, 0x70000220, 0x70000320, 0x70000420, // Target Pos 1~4
//  0x70000508, 0x70000608, 0x70000708, 0x70000808, // Direction 1~4
//  0x70000908, 0x70000A08, 0x70000B08, 0x70000C08, // Target Speed 1~4
//  0x70000D08, 0x70000E08, 0x70000F08, 0x70001008, // Accel Pattern 1~4
//  0x70001110, 0x70001210                          // Control Word, Dummy
//};
//
//// [핵심] TxPDO (0x1A00) 실제 매핑 데이터 배열
//uint32_t txpdo_mapping_entries[6] = {
//  0x60000120, 0x60000220, 0x60000320, 0x60000420, // Actual Pos 1~4
//  0x60000510, 0x60000610                          // Status Word, Error Code
//};
//
//// -------------------------------------------------------------
//// 1. Object Dictionary 이름(String) 정의
//// -------------------------------------------------------------
//static const char acName1000[] = "Device Type";
//static const char acName1008[] = "Device Name";
//static const char acName1009[] = "Hardware Version";
//static const char acName100A[] = "Software Version";
//
//static const char acName1018[] = "Identity Object";
//static const char acName1018_00[] = "Max SubIndex";
//static const char acName1018_01[] = "Vendor ID";
//static const char acName1018_02[] = "Product Code";
//static const char acName1018_03[] = "Revision Number";
//static const char acName1018_04[] = "Serial Number";
//
//static const char acNameMapping[] = "PDO Mapping Entry";
//
//static const char acName1600[] = "RxPDO Mapping";
//static const char acName1600_00[] = "Max SubIndex";
//static const char acName1A00[] = "TxPDO Mapping";
//static const char acName1A00_00[] = "Max SubIndex";
//
//static const char acName1C00[] = "Sync Manager Communication Type";
//static const char acName1C00_00[] = "Max SubIndex";
//static const char acName1C00_01[] = "Communication Type SM0";
//static const char acName1C00_02[] = "Communication Type SM1";
//static const char acName1C00_03[] = "Communication Type SM2";
//static const char acName1C00_04[] = "Communication Type SM3";
//
//static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
//static const char acName1C12_00[] = "Max SubIndex";
//static const char acName1C12_01[] = "PDO Mapping";
//
//static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
//static const char acName1C13_00[] = "Max SubIndex";
//static const char acName1C13_01[] = "PDO Mapping";
//
//static const char acName6000[] = "TxPDO 4-Axis Actual Data";
//static const char acName6000_00[] = "Max SubIndex";
//static const char acName6000_01[] = "Actual Pos Axis 1";
//static const char acName6000_02[] = "Actual Pos Axis 2";
//static const char acName6000_03[] = "Actual Pos Axis 3";
//static const char acName6000_04[] = "Actual Pos Axis 4";
//static const char acName6000_05[] = "Status Word";
//static const char acName6000_06[] = "Error Code";
//
//static const char acName7000[] = "RxPDO 4-Axis Target Data";
//static const char acName7000_00[] = "Max SubIndex";
//static const char acName7000_01[] = "Target Pos Axis 1";
//static const char acName7000_02[] = "Target Pos Axis 2";
//static const char acName7000_03[] = "Target Pos Axis 3";
//static const char acName7000_04[] = "Target Pos Axis 4";
//static const char acName7000_05[] = "Direction Axis 1";
//static const char acName7000_06[] = "Direction Axis 2";
//static const char acName7000_07[] = "Direction Axis 3";
//static const char acName7000_08[] = "Direction Axis 4";
//static const char acName7000_09[] = "Target Speed Axis 1";
//static const char acName7000_0A[] = "Target Speed Axis 2";
//static const char acName7000_0B[] = "Target Speed Axis 3";
//static const char acName7000_0C[] = "Target Speed Axis 4";
//static const char acName7000_0D[] = "Accel Pattern Axis 1";
//static const char acName7000_0E[] = "Accel Pattern Axis 2";
//static const char acName7000_0F[] = "Accel Pattern Axis 3";
//static const char acName7000_10[] = "Accel Pattern Axis 4";
//static const char acName7000_11[] = "Control Word";
//
//// -------------------------------------------------------------
//// 2. SDO 데이터 구조체 맵핑
//// -------------------------------------------------------------
//const _objd SDO1000[] = {
//  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x01901389, NULL},
//};
//const _objd SDO1008[] = {
//  {0x0, DTYPE_VISIBLE_STRING, 216, ATYPE_RO, acName1008, 0, "STM32H753 4-Axis Controller"},
//};
//const _objd SDO1009[] = {
//  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName1009, 0, HW_REV},
//};
//const _objd SDO100A[] = {
//  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName100A, 0, SW_REV},
//};
//
//const _objd SDO1018[] = {
//  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, &id_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0x1337, &vendor_id},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 1234, &product_code},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, &revision_num},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0, &serial_num},
//};
//
//// =====================================================================
//// 0x6000: TxPDO 실제 메모리 연결 (bData는 0으로 둬도 무방)
//// =====================================================================
//const _objd SDO6000[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName6000_00, 6, &txpdo_actual_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_01, 0, &Wb.actual_pos[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_02, 0, &Wb.actual_pos[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_03, 0, &Wb.actual_pos[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_04, 0, &Wb.actual_pos[3]},
//  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_05, 0, &Wb.status_word},
//  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_06, 0, &Wb.error_code},
//};
//
//// =====================================================================
//// 0x7000: RxPDO 실제 메모리 연결
//// =====================================================================
//const _objd SDO7000[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_00, 18, &rxpdo_target_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_01, 0, &Rb.target_pos[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_02, 0, &Rb.target_pos[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_03, 0, &Rb.target_pos[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_04, 0, &Rb.target_pos[3]},
//  {0x05, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_05, 0, &Rb.direction[0]},
//  {0x06, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_06, 0, &Rb.direction[1]},
//  {0x07, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_07, 0, &Rb.direction[2]},
//  {0x08, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_08, 0, &Rb.direction[3]},
//  {0x09, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_09, 0, &Rb.target_speed[0]},
//  {0x0A, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0A, 0, &Rb.target_speed[1]},
//  {0x0B, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0B, 0, &Rb.target_speed[2]},
//  {0x0C, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0C, 0, &Rb.target_speed[3]},
//  {0x0D, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0D, 0, &Rb.accel_pattern[0]},
//  {0x0E, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0E, 0, &Rb.accel_pattern[1]},
//  {0x0F, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0F, 0, &Rb.accel_pattern[2]},
//  {0x10, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_10, 0, &Rb.accel_pattern[3]},
//  {0x11, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7000_11, 0, &Rb.control_word},
//  {0x12, DTYPE_UNSIGNED16, 16, ATYPE_RW, "Dummy", 0, &Rb.dummy},
//};
//
//// =====================================================================
//// [핵심 해결] TxPDO 매핑 설정 (0x1A00) -> 6번째, 7번째 더블 바인딩!
//// =====================================================================
//const _objd SDO1A00[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1A00_00, 6, &txpdo_map_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000120, &txpdo_mapping_entries[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000220, &txpdo_mapping_entries[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000320, &txpdo_mapping_entries[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000420, &txpdo_mapping_entries[3]},
//  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000510, &txpdo_mapping_entries[4]},
//  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000610, &txpdo_mapping_entries[5]},
//};
//
//// =====================================================================
//// [핵심 해결] RxPDO 매핑 설정 (0x1600) -> 6번째, 7번째 더블 바인딩!
//// =====================================================================
//const _objd SDO1600[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1600_00, 18, &rxpdo_map_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000120, &rxpdo_mapping_entries[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000220, &rxpdo_mapping_entries[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000320, &rxpdo_mapping_entries[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000420, &rxpdo_mapping_entries[3]},
//  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000508, &rxpdo_mapping_entries[4]},
//  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000608, &rxpdo_mapping_entries[5]},
//  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000708, &rxpdo_mapping_entries[6]},
//  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000808, &rxpdo_mapping_entries[7]},
//  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000908, &rxpdo_mapping_entries[8]},
//  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000A08, &rxpdo_mapping_entries[9]},
//  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000B08, &rxpdo_mapping_entries[10]},
//  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000C08, &rxpdo_mapping_entries[11]},
//  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000D08, &rxpdo_mapping_entries[12]},
//  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000E08, &rxpdo_mapping_entries[13]},
//  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000F08, &rxpdo_mapping_entries[14]},
//  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001008, &rxpdo_mapping_entries[15]},
//  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001110, &rxpdo_mapping_entries[16]},
//  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001210, &rxpdo_mapping_entries[17]},
//};
//
//// =====================================================================
//// SyncManager 파라미터 셋업 (0x1C00, 1C12, 1C13)
//// =====================================================================
//const _objd SDO1C00[] = {
//  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, &sm_comm_type_count},
//  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, &sm_comm_types[0]},
//  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, &sm_comm_types[1]},
//  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, &sm_comm_types[2]},
//  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, &sm_comm_types[3]},
//};
//
//const _objd SDO1C12[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C12_00, 1, &sm2_assign_count},
//  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C12_01, 0x1600, &sm2_pdo_assign},
//};
//const _objd SDO1C13[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C13_00, 1, &sm3_assign_count},
//  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C13_01, 0x1A00, &sm3_pdo_assign},
//};
//
//// -------------------------------------------------------------
//// 3. 최종 Object List (SOES 엔진이 읽어가는 메인 배열)
//// -------------------------------------------------------------
//const _objectlist SDOobjects[] = {
//  {0x1000, OTYPE_VAR,    0, 1, acName1000, SDO1000},
//  {0x1008, OTYPE_VAR,    0, 1, acName1008, SDO1008},
//  {0x1009, OTYPE_VAR,    0, 1, acName1009, SDO1009},
//  {0x100A, OTYPE_VAR,    0, 1, acName100A, SDO100A},
//  {0x1018, OTYPE_RECORD, 4, 5, acName1018, SDO1018},
//  {0x1600, OTYPE_RECORD, 18, 19, acName1600, SDO1600},
//  {0x1A00, OTYPE_RECORD, 6, 7, acName1A00, SDO1A00},
//  {0x1C00, OTYPE_ARRAY,  4, 5, acName1C00, SDO1C00},
//  {0x1C12, OTYPE_ARRAY,  1, 2, acName1C12, SDO1C12},
//  {0x1C13, OTYPE_ARRAY,  1, 2, acName1C13, SDO1C13},
//  {0x6000, OTYPE_RECORD, 6, 7, acName6000, SDO6000},
//  {0x7000, OTYPE_RECORD, 18, 19, acName7000, SDO7000},
//  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
//};
//
