/*
 * TMC2209_usart.c
 *
 * Created on: Mar 17, 2026
 * Author: cubox
 */

#include "TMC2209_usart.h"

// TMC2209 주요 레지스터 주소 (헤더 파일에 없다면 여기에 정의)
#define TMC2209_REG_GCONF      0x00
#define TMC2209_REG_IHOLD_IRUN 0x10
#define TMC2209_REG_VACTUAL    0x22
#define TMC2209_REG_CHOPCONF   0x6C

// =========================================================
// 1. TMC2209 전용 CRC8 계산 함수
// =========================================================
uint8_t TMC2209_CalcCRC(uint8_t* datagram, uint8_t datagramLength) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < datagramLength; i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    return crc;
}


// =========================================================
// 2. 1선식 UART 레지스터 Write 함수 (공용)
// =========================================================
void TMC2209_WriteRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr, uint32_t data) {
    uint8_t tx_datagram[8];

    // 1. Sync Byte & Address
    tx_datagram[0] = 0x05;
    tx_datagram[1] = motor_addr;

    // 2. ⭐️ 쓰기(Write) 명령 플래그 추가 (최상위 비트를 1로)
    tx_datagram[2] = reg_addr | 0x80;

    // 3. 32비트 데이터 분할 (MSB First)
    tx_datagram[3] = (data >> 24) & 0xFF;
    tx_datagram[4] = (data >> 16) & 0xFF;
    tx_datagram[5] = (data >> 8) & 0xFF;
    tx_datagram[6] = data & 0xFF;

    // 4. ⭐️ CRC 계산 (쓰기는 앞의 7바이트 전체를 대상으로 계산해야 합니다!)
    tx_datagram[7] = TMC2209_CalcCRC(tx_datagram, 7);

    // 5. 송신
    HAL_HalfDuplex_EnableTransmitter(huart);
    HAL_UART_Transmit(huart, tx_datagram, 8, HAL_MAX_DELAY);

    // 쓰기 명령은 모터가 대답(ACK)을 하지 않으므로 수신 대기 코드가 필요 없습니다!
}

// =========================================================
// 3. 모터 전류 설정 전용 함수
// =========================================================
void TMC2209_SetCurrent(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t i_run, uint8_t i_hold) {
    uint32_t reg_value = 0;

    // IHOLDDELAY (bit 16~19), IRUN (bit 8~12), IHOLD (bit 0~4)
    reg_value |= (0x02 << 16);    // IHOLDDELAY = 2
    reg_value |= (i_run << 8);    // 모터 구동 시 전류
    reg_value |= (i_hold << 0);   // 모터 정지 시 유지 전류

    TMC2209_WriteRegister(huart, motor_addr, TMC2209_REG_IHOLD_IRUN, reg_value);
}

// =========================================================
// 4. 모터 초기화 (어떤 UART 채널, 어떤 모터든 이 함수 하나로 끝!)
// =========================================================
void TMC2209_Init(UART_HandleTypeDef *huart, uint8_t motor_addr) {
    // 1. UART 제어 활성화 (pdn_disable=1)
    TMC2209_WriteRegister(huart, motor_addr, TMC2209_REG_GCONF, 0x00000040);

    // 2. 모터 구동 드라이버 켜기 (TOFF=3)
    TMC2209_WriteRegister(huart, motor_addr, TMC2209_REG_CHOPCONF, 0x10000053);

    // 3. 전류 설정 (기왕 만든 SetCurrent 함수를 재활용!)
    TMC2209_SetCurrent(huart, motor_addr, 16, 8);

    // 4. 초기 속도 0 (정지)
    TMC2209_WriteRegister(huart, motor_addr, TMC2209_REG_VACTUAL, 0);
}

// =========================================================
// 5. 모터 속도 및 방향 업데이트 (어떤 모터든 이거 하나로 제어!)
// =========================================================
void TMC2209_Update(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t target_speed, uint8_t direction) {
    int32_t vactual = 0;

    if (target_speed > 0) {
        // 속도 증폭 스케일링
        vactual = (int32_t)target_speed * 1000;

        // 방향 반전
        if (direction == 0) {
            vactual = -vactual;
        }
    }

    // UART로 실시간 속도 명령 전송
    TMC2209_WriteRegister(huart, motor_addr, TMC2209_REG_VACTUAL, (uint32_t)vactual);
}

// =========================================================
// [추가] 1선식 UART 레지스터 Read 함수 (가장 중요!)
// =========================================================
uint32_t TMC2209_ReadRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr) {
    uint8_t tx_datagram[4];
    uint8_t rx_datagram[8] = {0}; // 수신 버퍼
    uint32_t result = 0;

    // 1. 읽기 명령 데이터그램 조립
    tx_datagram[0] = 0x05;                        // Sync Byte
    tx_datagram[1] = motor_addr;                  // Slave Address
    tx_datagram[2] = reg_addr & 0x7F;             // Register Address (MSB 0 = Read)
    tx_datagram[3] = TMC2209_CalcCRC(tx_datagram, 3); // CRC 계산

    // 2. 송신 모드로 전환하여 데이터 발사
    HAL_HalfDuplex_EnableTransmitter(huart);
    HAL_UART_Transmit(huart, tx_datagram, 4, HAL_MAX_DELAY);

    // 3. 수신 모드 전환
    HAL_HalfDuplex_EnableReceiver(huart);

    // ⭐️ [가장 중요] 전환 과정에서 발생한 노이즈 및 에러 플래그 강제 삭제!!
    // 이거 없으면 STM32가 에러 났다고 착각하고 수신을 거부합니다.
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

    // 4. 수신 시도 및 결과 상태 저장 (타임아웃 20ms로 넉넉하게)
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, rx_datagram, 8, 20);

    // 5. 결과에 따른 디버깅 처리
    if (status == HAL_OK) {
        // 수신 성공! 데이터 조립
        result = (rx_datagram[3] << 24) | (rx_datagram[4] << 16) | (rx_datagram[5] << 8) | rx_datagram[6];
        return result;
    } else if (status == HAL_TIMEOUT) {
        // 모터가 대답을 안 함 (단선, 주소 불일치, 동기화 안됨 등)
        return 0xEEEEEEEE;
    } else {
        // STM32 자체 UART 에러 (노이즈, 핀 설정 등)
        return 0xDEADBEEF;
    }
}
