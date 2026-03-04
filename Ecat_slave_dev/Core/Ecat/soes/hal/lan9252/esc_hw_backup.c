///*
// * Licensed under the GNU General Public License version 2 with exceptions. See
// * LICENSE file in the project root for full license information
// */
//
// /** \file
// * \brief
// * ESC hardware layer functions for LAN9252.
// *
// * Function to read and write commands to the ESC. Used to read/write ESC
// * registers and memory.
// */

#include "esc.h"             // SOES 코어 헤더 (ESC_read, ESC_write 선언 포함)
#include "stm32h7xx_hal.h"   // STM32H7 시리즈 HAL 라이브러리

// 1. 큐브IDE에서 설정한 SPI 핸들 가져오기 (예: SPI1을 쓴다면 hspi1)
extern SPI_HandleTypeDef hspi1;

// 2. CS(Chip Select) 핀 매크로 정의 (회로도/CubeIDE 설정에 맞게 포트/핀 수정 필수)
#define LAN9252_CS_PORT GPIOE
#define LAN9252_CS_PIN  GPIO_PIN_2
#define CS_LOW()  HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_SET)

// 3. LAN9252 SPI 명령어
#define LAN9252_CMD_FAST_READ 0x0B
#define LAN9252_CMD_WRITE     0x02

/**
 * @brief LAN9252에서 데이터를 읽어오는 함수 (SOES 코어에서 호출됨)
 * @param address LAN9252 내부 레지스터 주소
 * @param buf 읽어온 데이터를 저장할 버퍼 포인터
 * @param len 읽어올 데이터 길이 (바이트 단위)
 */
// =========================================================================
// 1. 제어 레지스터(CSR) 간접 접근을 위한 32비트 전용 Helper 함수
// =========================================================================
uint32_t lan9252_read_direct_32(uint16_t lan_addr) {
    uint8_t tx_buf[8] = {0x0B, (lan_addr >> 8) & 0xFF, lan_addr & 0xFF, 0x00, 0, 0, 0, 0};
    uint8_t rx_buf[8] = {0};

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 8, HAL_MAX_DELAY);
    CS_HIGH();

    return ((uint32_t)rx_buf[4]) | ((uint32_t)rx_buf[5] << 8) | ((uint32_t)rx_buf[6] << 16) | ((uint32_t)rx_buf[7] << 24);
}

void lan9252_write_direct_32(uint16_t lan_addr, uint32_t val) {
    uint8_t tx_buf[7] = {0x02, (lan_addr >> 8) & 0xFF, lan_addr & 0xFF,
                         val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF};
    uint8_t rx_buf[7] = {0}; // Overrun 방지용

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 7, HAL_MAX_DELAY);
    CS_HIGH();
}

// =========================================================================
// 2. SOES 코어용 궁극의 하이브리드 Read / Write 함수
// =========================================================================
//void ESC_read(uint16_t address, void *buf, uint16_t len) {
//    uint8_t tx_buf[300] = {0};
//    uint8_t rx_buf[300] = {0};
//
//    // SOES 스택이 요청한 주소에 LAN9252 이더캣 메모리 시작점(0x1000)을 더함
//    uint16_t lan_addr = address;
//    if (lan_addr < 0x1000) {
//        lan_addr += 0x1000;
//    }
//
//    // Fast Read 구조: 명령어(0x0B) + 주소 2바이트 + 더미 1바이트
//    tx_buf[0] = 0x0B;
//    tx_buf[1] = (lan_addr >> 8) & 0xFF;
//    tx_buf[2] = lan_addr & 0xFF;
//    tx_buf[3] = 0x00;
//
//    CS_LOW();
//    // TransmitReceive를 한 번에 호출하여 통신 끊김과 버퍼 오버런을 원천 차단
//    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 4, HAL_MAX_DELAY);
//    CS_HIGH();
//
//    // 앞의 4바이트(헤더 송신 시 수신된 쓰레기값)를 건너뛰고 실제 데이터만 복사
//    memcpy(buf, &rx_buf[4], len);
//}
//
//void ESC_write(uint16_t address, void *buf, uint16_t len) {
//    uint8_t tx_buf[300] = {0};
//    uint8_t rx_buf[300] = {0}; // 수신 쓰레기값을 받아낼 더미 공간
//
//    uint16_t lan_addr = address;
//    if (lan_addr < 0x1000) {
//        lan_addr += 0x1000;
//    }
//
//    // Write 구조: 명령어(0x02) + 주소 2바이트
//    tx_buf[0] = 0x02;
//    tx_buf[1] = (lan_addr >> 8) & 0xFF;
//    tx_buf[2] = lan_addr & 0xFF;
//
//    // 헤더 바로 뒤에 쓸 데이터를 이어 붙임
//    memcpy(&tx_buf[3], buf, len);
//
//    CS_LOW();
//    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 3, HAL_MAX_DELAY);
//    CS_HIGH();
//}

// 1. 간접 제어기(CSR) 도우미 함수들
uint32_t lan9252_read_csr(uint16_t csr_addr) {
    uint8_t tx_buf[8] = {0x0B, (csr_addr >> 8) & 0xFF, csr_addr & 0xFF, 0x00, 0, 0, 0, 0};
    uint8_t rx_buf[8] = {0};
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 8, HAL_MAX_DELAY);
    CS_HIGH();
    return ((uint32_t)rx_buf[4]) | ((uint32_t)rx_buf[5] << 8) | ((uint32_t)rx_buf[6] << 16) | ((uint32_t)rx_buf[7] << 24);
}

void lan9252_write_csr(uint16_t csr_addr, uint32_t val) {
    uint8_t tx_buf[7] = {0x02, (csr_addr >> 8) & 0xFF, csr_addr & 0xFF,
                         val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF};
    uint8_t rx_buf[7] = {0};
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 7, HAL_MAX_DELAY);
    CS_HIGH();
}

// 2. SOES용 코어 함수
void ESC_read(uint16_t address, void *buf, uint16_t len) {
    if (address >= 0x1000) {
        // [Process RAM] 고속 Direct 통신 (0x1000 더하지 않고 그대로 사용!)
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};
        tx_buf[0] = 0x0B;
        tx_buf[1] = (address >> 8) & 0xFF;
        tx_buf[2] = address & 0xFF;
        tx_buf[3] = 0x00;
        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 4, HAL_MAX_DELAY);
        CS_HIGH();
        memcpy(buf, &rx_buf[4], len);
    } else {
        // [제어 레지스터] 우회 접근
        uint8_t *p = (uint8_t *)buf;
        uint16_t current_addr = address;
        uint16_t bytes_left = len;

        while (bytes_left > 0) {
            uint8_t chunk = (bytes_left > 4) ? 4 : bytes_left;
            uint8_t size_code = (chunk == 4) ? 0 : chunk;

            uint32_t cmd = 0x80000000 | 0x40000000 | ((uint32_t)size_code << 16) | current_addr;
            lan9252_write_csr(0x0304, cmd);

            // 🔥 [핵심] 멈춤 방지를 위한 Timeout 설정
            uint32_t timeout = 10000;
            while ((lan9252_read_csr(0x0304) & 0x80000000) && timeout > 0) {
                timeout--;
            }

            uint32_t data = lan9252_read_csr(0x0300);
            for (int i = 0; i < chunk; i++) {
                *p++ = (data >> (i * 8)) & 0xFF;
            }
            bytes_left -= chunk;
            current_addr += chunk;
        }
    }
}

void ESC_write(uint16_t address, void *buf, uint16_t len) {
    if (address >= 0x1000) {
        // [Process RAM] 고속 Direct 통신
        uint8_t tx_buf[256] = {0};
        uint8_t rx_buf[256] = {0};
        tx_buf[0] = 0x02;
        tx_buf[1] = (address >> 8) & 0xFF;
        tx_buf[2] = address & 0xFF;
        memcpy(&tx_buf[3], buf, len);
        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 3, HAL_MAX_DELAY);
        CS_HIGH();
    } else {
        // [제어 레지스터] 우회 접근
        uint8_t *p = (uint8_t *)buf;
        uint16_t current_addr = address;
        uint16_t bytes_left = len;

        while (bytes_left > 0) {
            uint8_t chunk = (bytes_left > 4) ? 4 : bytes_left;
            uint32_t data = 0;
            for (int i = 0; i < chunk; i++) {
                data |= ((uint32_t)(*p++) << (i * 8));
            }

            lan9252_write_csr(0x0300, data);

            uint8_t size_code = (chunk == 4) ? 0 : chunk;
            uint32_t cmd = 0x80000000 | ((uint32_t)size_code << 16) | current_addr;
            lan9252_write_csr(0x0304, cmd);

            // 🔥 [핵심] 멈춤 방지를 위한 Timeout 설정
            uint32_t timeout = 10000;
            while ((lan9252_read_csr(0x0304) & 0x80000000) && timeout > 0) {
                timeout--;
            }

            bytes_left -= chunk;
            current_addr += chunk;
        }
    }
}

// =========================================================================
// LAN9252 전용 하드웨어 초기화 API
// =========================================================================
uint8_t LAN9252_HW_Init(void) {
    uint32_t data = 0;
    uint32_t intMask = 0;

    // 1. 칩 부팅 및 통신 확인 (BYTE_ORDER_REG: 0x0064)
    // 칩이 완전히 켜질 때까지 0x87654321을 기다립니다.
    do {
        data = lan9252_read_csr(0x0064);
    } while (data != 0x87654321);

    // 2. 인터럽트 강제 비활성화 (INT_EN: 0x005C)
    lan9252_write_csr(0x005C, 0x00000000);

    // 3. AL Event Mask 레지스터(0x0204) 초기화
    // (Microchip 드라이버의 핵심: 찌꺼기 인터럽트를 확실히 지워줍니다)
    do {
        lan9252_write_csr(0x0204, 0x00000093);
        intMask = lan9252_read_csr(0x0204);
    } while (intMask != 0x00000093);
    lan9252_write_csr(0x0204, 0x00000000);

    // 4. IRQ 핀 설정 (INT_CONF: 0x0054)
    // IRQ enable, IRQ polarity, IRQ buffer type 설정 (0x00000101)
    lan9252_write_csr(0x0054, 0x00000101);

    // 5. 이더캣 글로벌 인터럽트 활성화 (INT_EN: 0x005C)
    lan9252_write_csr(0x005C, 0x00000001);

    // 6. 펜딩된 인터럽트 찌꺼기 비우기 (INT_STS: 0x0058 읽기)
    data = lan9252_read_csr(0x0058);

    // (선택) 칩 내부 이더캣 코어 리셋 튕기기 (안정성 향상)
    data = lan9252_read_csr(0x01F8);
    lan9252_write_csr(0x01F8, data | 0x00000040);
    HAL_Delay(50);

    return 1; // 초기화 대성공
}

//chip id read success
//void ESC_read(uint16_t address, void *buf, uint16_t len) {
//    uint8_t tx_buf[256] = {0}; // 송신 버퍼 (명령어+주소+더미+빈공간)
//    uint8_t rx_buf[256] = {0}; // 수신 버퍼
//
//    // Fast Read 구조: 명령어(0x0B) + 주소 2바이트 + 더미 1바이트
//    tx_buf[0] = 0x0B;
//    tx_buf[1] = (address >> 8) & 0xFF;
//    tx_buf[2] = address & 0xFF;
//    tx_buf[3] = 0x00;
//
//    CS_LOW();
//
//    // 🔥 [핵심] Transmit과 Receive를 분리하지 않고 한 번에 묶어서 클럭 끊김 차단!
//    // 길이: 헤더(4바이트) + 받을 데이터 길이(len)
//    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 4, HAL_MAX_DELAY);
//
//    CS_HIGH();
//
//    // 수신 버퍼의 앞 4바이트(명령어 보낼 때 들어온 쓰레기값)를 무시하고
//    // 실제 데이터가 있는 인덱스 4번부터 buf로 복사
//    memcpy(buf, &rx_buf[4], len);
//}
//
//void ESC_write(uint16_t address, void *buf, uint16_t len) {
//    uint8_t tx_buf[256] = {0};
//    uint8_t rx_buf[256] = {0}; // 쓰레기 값 받아낼 더미 버퍼 (Overrun 방지)
//
//    // Write 구조: 명령어(0x02) + 주소 2바이트
//    tx_buf[0] = 0x02;
//    tx_buf[1] = (address >> 8) & 0xFF;
//    tx_buf[2] = address & 0xFF;
//
//    // 쓸 데이터를 헤더 바로 뒤에 이어 붙이기
//    memcpy(&tx_buf[3], buf, len);
//
//    CS_LOW();
//
//    // 🔥 [핵심] 송신만 하더라도 반드시 TransmitReceive를 써서 RX FIFO 오버런 늪을 방지!
//    // 길이: 헤더(3바이트) + 쓸 데이터 길이(len)
//    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, len + 3, HAL_MAX_DELAY);
//
//    CS_HIGH();
//}

void ESC_init(const esc_cfg_t * cfg) {
    // 하드웨어 초기화 딜레이 (필요시 사용)
    HAL_Delay(10);
}


//
//extern SPI_HandleTypeDef hspi1; // 큐브IDE에서 생성한 SPI 핸들
//
//// =========================================================================
//// 1. LAN9252 하드웨어 CSR(간접 제어기) 안전한 1-Shot SPI 통신 함수
//// =========================================================================
//uint32_t LAN9252_CSR_Read(uint16_t csr_addr) {
//    // 0x03(Read) + Address MSB + Address LSB + 4바이트 Dummy
//    uint8_t tx[7] = {0x03, (csr_addr >> 8) & 0xFF, csr_addr & 0xFF, 0, 0, 0, 0};
//    uint8_t rx[7] = {0};
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS 핀 (자신의 핀에 맞게 변경!)
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY); // 끊김 없는 1-Shot 통신
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//
//    return ((uint32_t)rx[3]) | ((uint32_t)rx[4] << 8) | ((uint32_t)rx[5] << 16) | ((uint32_t)rx[6] << 24);
//}
//
//// 수정된 Write 함수: RX 버퍼를 추가해서 수신 FIFO Overrun을 방지합니다.
//void LAN9252_CSR_Write(uint16_t csr_addr, uint32_t val) {
//    uint8_t tx[7] = {0x02, (csr_addr >> 8) & 0xFF, csr_addr & 0xFF,
//                     val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF};
//
//    uint8_t dummy_rx[7] = {0}; // 쓰레기 값을 받아낼 더미(Dummy) 버퍼 추가!
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//
//    // Transmit 대신 TransmitReceive를 써서 TX/RX 밸런스를 강제로 맞춥니다.
//    HAL_SPI_TransmitReceive(&hspi1, tx, dummy_rx, 7, HAL_MAX_DELAY);
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//}
//
//// =========================================================================
//// 2. SOES 코어용 진짜 EtherCAT RAM 접근 함수 (여기서 0x0300, 0x0304를 씁니다!)
//// =========================================================================
//void ESC_read(uint16_t address, void *buf, uint16_t len) {
//    uint8_t *p = (uint8_t *)buf;
//    uint16_t bytes_left = len;
//    uint16_t current_addr = address;
//
//    while (bytes_left > 0) {
//        uint8_t chunk = (bytes_left > 4) ? 4 : bytes_left;
//
//        // 1. ECAT_CSR_CMD(0x0304)에 '읽기 명령 + 사이즈 + 주소' 쏘기
//        uint32_t cmd = 0x80000000 | 0x40000000 | (chunk << 16) | current_addr;
//        LAN9252_CSR_Write(0x0304, cmd);
//
//        // 2. LAN9252가 EtherCAT 메모리에서 값을 가져올 때까지 대기
//        while (LAN9252_CSR_Read(0x0304) & 0x80000000) {}
//
//        // 3. 가져온 값을 ECAT_CSR_DATA(0x0300)에서 빼오기
//        uint32_t data = LAN9252_CSR_Read(0x0300);
//
//        for (int i = 0; i < chunk; i++) {
//            *p++ = (data >> (i * 8)) & 0xFF;
//        }
//        bytes_left -= chunk;
//        current_addr += chunk;
//    }
//}
//
//void ESC_write(uint16_t address, void *buf, uint16_t len) {
//    uint8_t *p = (uint8_t *)buf;
//    uint16_t bytes_left = len;
//    uint16_t current_addr = address;
//
//    while (bytes_left > 0) {
//        uint8_t chunk = (bytes_left > 4) ? 4 : bytes_left;
//        uint32_t data = 0;
//
//        for (int i = 0; i < chunk; i++) {
//            data |= (*p++ << (i * 8));
//        }
//
//        // 1. 쓸 데이터를 ECAT_CSR_DATA(0x0300)에 먼저 밀어넣기
//        LAN9252_CSR_Write(0x0300, data);
//
//        // 2. ECAT_CSR_CMD(0x0304)에 '쓰기 명령 + 사이즈 + 주소' 쏘기
//        uint32_t cmd = 0x80000000 | (chunk << 16) | current_addr;
//        LAN9252_CSR_Write(0x0304, cmd);
//
//        // 3. 쓰기가 완료될 때까지 대기
//        while (LAN9252_CSR_Read(0x0304) & 0x80000000) {}
//
//        bytes_left -= chunk;
//        current_addr += chunk;
//    }
//}
//

////#include "esc.h"
////#include "stm32h7xx.h"
////#include <string.h>
////#include <stdlib.h>
////
////
////#define BIT(x)                   (1U << (x))
////
////#define ESC_CMD_SERIAL_WRITE     0x02
////#define ESC_CMD_SERIAL_READ      0x03
////#define ESC_CMD_FAST_READ        0x0B
////#define ESC_CMD_RESET_SQI        0xFF
////
////#define ESC_CMD_FAST_READ_DUMMY  1
////#define ESC_CMD_ADDR_INC         BIT(6)
////
////#define ESC_PRAM_RD_FIFO_REG     0x000
////#define ESC_PRAM_WR_FIFO_REG     0x020
////#define ESC_PRAM_RD_ADDR_LEN_REG 0x308
////#define ESC_PRAM_RD_CMD_REG      0x30C
////#define ESC_PRAM_WR_ADDR_LEN_REG 0x310
////#define ESC_PRAM_WR_CMD_REG      0x314
////
////#define ESC_PRAM_CMD_BUSY        BIT(31)
////#define ESC_PRAM_CMD_ABORT       BIT(30)
////
////#define ESC_PRAM_CMD_CNT(x)      ((x >> 8) & 0x1F)
////#define ESC_PRAM_CMD_AVAIL       BIT(0)
////
////#define ESC_PRAM_SIZE(x)         ((x) << 16)
////#define ESC_PRAM_ADDR(x)         ((x) << 0)
////
////#define ESC_CSR_DATA_REG         0x300
////#define ESC_CSR_CMD_REG          0x304
////
////#define ESC_CSR_CMD_BUSY         BIT(31)
////#define ESC_CSR_CMD_READ         (BIT(31) | BIT(30))
////#define ESC_CSR_CMD_WRITE        BIT(31)
////#define ESC_CSR_CMD_SIZE(x)      (x << 16)
////
////#define ESC_RESET_CTRL_REG       0x1F8
////#define ESC_RESET_CTRL_RST       BIT(6)
////
////static int lan9252 = -1;
////
/////* lan9252 singel write */
////static void lan9252_write_32 (uint16_t address, uint32_t val)
////{
////    uint8_t data[7];
////    ssize_t n;
////
////    data[0] = ESC_CMD_SERIAL_WRITE;
////    data[1] = (uint8_t)((address >> 8) & 0xFF);
////    data[2] = (uint8_t)(address & 0xFF);
////    data[3] = (uint8_t)(val & 0xFF);
////    data[4] = (uint8_t)((val >> 8) & 0xFF);
////    data[5] = (uint8_t)((val >> 16) & 0xFF);
////    data[6] = (uint8_t)((val >> 24) & 0xFF);
////
////    /* Write data */
////    n = write (lan9252, data, sizeof(data));
////    (void)n;
////}
////
/////* lan9252 single read */
////static uint32_t lan9252_read_32 (uint32_t address)
////{
////   uint8_t data[2];
////   uint8_t result[4];
////   uint16_t lseek_addr;
////   ssize_t n;
////
////   data[0] = ((address >>8) & 0xFF);
////   data[1] = (address & 0xFF);
////
////   lseek_addr=(uint16_t)((data[0] << 8) | data[1]);
////   lseek (lan9252, lseek_addr, SEEK_SET);
////   n = read (lan9252, result, sizeof(result));
////   (void)n;
////
////   return (uint32_t)((result[3] << 24) |
////           (result[2] << 16) |
////           (result[1] << 8) |
////            result[0]);
////}
////
/////* ESC read CSR function */
////static void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
////{
////   uint32_t value;
////
////   value = ESC_CSR_CMD_READ;
////   value |= (uint32_t)ESC_CSR_CMD_SIZE(len);
////   value |= address;
////   lan9252_write_32(ESC_CSR_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_CSR_CMD_REG);
////   } while(value & ESC_CSR_CMD_BUSY);
////
////   value = lan9252_read_32(ESC_CSR_DATA_REG);
////   memcpy(buf, (uint8_t *)&value, len);
////}
////
/////* ESC write CSR function */
////static void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
////{
////   uint32_t value;
////
////   memcpy((uint8_t*)&value, buf,len);
////   lan9252_write_32(ESC_CSR_DATA_REG, value);
////   value = ESC_CSR_CMD_WRITE;
////   value |= (uint32_t)ESC_CSR_CMD_SIZE(len);
////   value |= address;
////   lan9252_write_32(ESC_CSR_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_CSR_CMD_REG);
////   } while(value & ESC_CSR_CMD_BUSY);
////}
////
/////* ESC read process data ram function */
////static void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
////{
////   uint32_t value;
////   uint8_t * temp_buf = buf;
////   uint16_t byte_offset = 0;
////   uint8_t fifo_cnt, first_byte_position, temp_len;
////   uint8_t *buffer;
////   size_t i, array_size, size;
////   float quotient,remainder;
////   uint32_t temp;
////   ssize_t n;
////
////   value = ESC_PRAM_CMD_ABORT;
////   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
////   }while(value & ESC_PRAM_CMD_BUSY);
////
////   value = (uint32_t)(ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address));
////   lan9252_write_32(ESC_PRAM_RD_ADDR_LEN_REG, value);
////
////   value = ESC_PRAM_CMD_BUSY;
////   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
////   }while((value & ESC_PRAM_CMD_AVAIL) == 0);
////
////   /* Fifo count */
////   fifo_cnt = (uint8_t)ESC_PRAM_CMD_CNT(value);
////
////   /* Read first value from FIFO */
////   value = lan9252_read_32(ESC_PRAM_RD_FIFO_REG);
////   fifo_cnt--;
////
////   /* Find out first byte position and adjust the copy from that
////    * according to LAN9252 datasheet and MicroChip SDK code
////    */
////   first_byte_position = (address & 0x03);
////   temp_len = ((4 - first_byte_position) > len) ? (uint8_t)len : (uint8_t)(4 - first_byte_position);
////
////   memcpy(temp_buf ,((uint8_t *)&value + first_byte_position), temp_len);
////   len = (uint16_t)(len - temp_len);
////   byte_offset = (uint16_t)(byte_offset + temp_len);
////
////   /* Continue reading until we have read len */
////    if (len > 0){
////
////        quotient = (float)(len/4);
////        remainder = (float)(len%4);
////
////        if (remainder == 0)
////            array_size = (size_t)quotient;
////        else
////            array_size = (size_t)quotient+1;
////
////        size = 4*array_size;
////
////        buffer = (uint8_t *)malloc(size);
////        buffer[0] = (uint8_t)size;
////        memset(buffer,0,size);
////
////        lseek (lan9252, ESC_PRAM_RD_FIFO_REG, SEEK_SET);
////        n = read (lan9252, buffer, size);
////        (void)n;
////
////        while(len > 0)
////        {
////
////            for (i=0; i<size; i=i+4) {
////                temp_len = (len > 4) ? 4: (uint8_t)len;
////
////                temp = (uint32_t)(buffer[i] | (buffer[i+1] << 8) | (buffer[i+2] << 16) | (buffer[i+3] << 24));
////                memcpy(temp_buf + byte_offset ,&temp, temp_len);
////                fifo_cnt--;
////                len = (uint16_t)(len - temp_len);
////                byte_offset = (uint16_t)(byte_offset + temp_len);
////            }
////        }
////        free(buffer);
////    }
////}
////
/////* ESC write process data ram function */
////static void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
////{
////   uint32_t value;
////   uint8_t * temp_buf = buf;
////   uint16_t byte_offset = 0;
////   uint8_t fifo_cnt, first_byte_position, temp_len;
////   uint8_t *buffer;
////   size_t i, array_size, size;
////   float quotient,remainder;
////   ssize_t n;
////
////   value = ESC_PRAM_CMD_ABORT;
////   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
////   }while(value & ESC_PRAM_CMD_BUSY);
////
////   value = (uint32_t)(ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address));
////   lan9252_write_32(ESC_PRAM_WR_ADDR_LEN_REG, value);
////
////   value = ESC_PRAM_CMD_BUSY;
////   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);
////
////   do
////   {
////      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
////   }while((value & ESC_PRAM_CMD_AVAIL) == 0);
////
////   /* Fifo count */
////   fifo_cnt = (uint8_t)ESC_PRAM_CMD_CNT(value);
////
////   /* Find out first byte position and adjust the copy from that
////    * according to LAN9252 datasheet
////    */
////   first_byte_position = (address & 0x03);
////   temp_len = ((4 - first_byte_position) > len) ? (uint8_t)len : (uint8_t)(4 - first_byte_position);
////
////   memcpy(((uint8_t *)&value + first_byte_position), temp_buf, temp_len);
////
////   /* Write first value from FIFO */
////   lan9252_write_32(ESC_PRAM_WR_FIFO_REG, value);
////
////   len = (uint16_t)(len - temp_len);
////   byte_offset = (uint16_t)(byte_offset + temp_len);
////   fifo_cnt--;
////
////    if (len > 0){
////
////        quotient = len/4;
////        remainder = (float)(len%4);
////
////        if (remainder == 0)
////            array_size = (size_t)quotient;
////        else
////            array_size = (size_t)quotient+1;
////
////        size = 3+4*array_size;
////
////        buffer = (uint8_t *)malloc(size);
////        buffer[0] = (uint8_t)size;
////        memset(buffer,0,size);
////
////        buffer[0] = ESC_CMD_SERIAL_WRITE;
////        buffer[1] = ((ESC_PRAM_WR_FIFO_REG >> 8) & 0xFF);
////        buffer[2] = (ESC_PRAM_WR_FIFO_REG & 0xFF);
////        while(len > 0)
////        {
////            for (i=3; i<size; i=i+4) {
////                temp_len = (len > 4) ? 4 : (uint8_t)len;
////
////                memcpy((uint8_t *)&value, (temp_buf + byte_offset), temp_len);
////                buffer[i] = (uint8_t)(value & 0xFF);
////                buffer[i+1] = (uint8_t)((value >> 8) & 0xFF);
////                buffer[i+2] = (uint8_t)((value >> 16) & 0xFF);
////                buffer[i+3] = (uint8_t)((value >> 24) & 0xFF);
////
////                fifo_cnt--;
////                len = (uint16_t)(len - temp_len);
////                byte_offset= (uint16_t)(byte_offset + temp_len);
////            }
////        }
////        n = write (lan9252, buffer, size);
////        (void)n;
////        free(buffer);
////    }
////}
////
////
/////** ESC read function used by the Slave stack.
//// *
//// * @param[in]   address     = address of ESC register to read
//// * @param[out]  buf         = pointer to buffer to read in
//// * @param[in]   len         = number of bytes to read
//// */
////void ESC_read (uint16_t address, void *buf, uint16_t len)
////{
////   /* Select Read function depending on address, process data ram or not */
////   if (address >= 0x1000)
////   {
////      ESC_read_pram(address, buf, len);
////   }
////   else
////   {
////      uint16_t size;
////      uint8_t *temp_buf = (uint8_t *)buf;
////
////      while(len > 0)
////      {
////         /* We write maximum 4 bytes at the time */
////         size = (len > 4) ? 4 : len;
////         /* Make size aligned to address according to LAN9252 datasheet
////          * Table 12-14 EtherCAT CSR Address VS size and MicroChip SDK code
////          */
////         /* If we got an odd address size is 1 , 01b 11b is captured */
////         if(address & BIT(0))
////         {
////            size = 1;
////         }
////         /* If address 1xb and size != 1 and 3 , allow size 2 else size 1 */
////         else if (address & BIT(1))
////         {
////            size = (size & BIT(0)) ? 1 : 2;
////         }
////         /* size 3 not valid */
////         else if (size == 3)
////         {
////            size = 1;
////         }
////         /* else size is kept AS IS */
////         ESC_read_csr(address, temp_buf, size);
////
////         /* next address */
////         len = (uint16_t)(len - size);
////         temp_buf = (uint8_t *)(temp_buf + size);
////         address = (uint16_t)(address + size);
////      }
////   }
////   /* To mimic the ET1100 always providing AlEvent on every read or write */
////   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
////   ESCvar.ALevent = etohs (ESCvar.ALevent);
////
////}
////
/////** ESC write function used by the Slave stack.
//// *
//// * @param[in]   address     = address of ESC register to write
//// * @param[out]  buf         = pointer to buffer to write from
//// * @param[in]   len         = number of bytes to write
//// */
////void ESC_write (uint16_t address, void *buf, uint16_t len)
////{
////   /* Select Write function depending on address, process data ram or not */
////   if (address >= 0x1000)
////   {
////      ESC_write_pram(address, buf, len);
////   }
////   else
////   {
////      uint16_t size;
////      uint8_t *temp_buf = (uint8_t *)buf;
////
////      while(len > 0)
////      {
////         /* We write maximum 4 bytes at the time */
////         size = (len > 4) ? 4 : len;
////         /* Make size aligned to address according to LAN9252 datasheet
////          * Table 12-14 EtherCAT CSR Address VS size  and MicroChip SDK code
////          */
////         /* If we got an odd address size is 1 , 01b 11b is captured */
////         if(address & BIT(0))
////         {
////            size = 1;
////         }
////         /* If address 1xb and size != 1 and 3 , allow size 2 else size 1 */
////         else if (address & BIT(1))
////         {
////            size = (size & BIT(0)) ? 1 : 2;
////         }
////         /* size 3 not valid */
////         else if (size == 3)
////         {
////            size = 1;
////         }
////         /* else size is kept AS IS */
////         ESC_write_csr(address, temp_buf, size);
////
////         /* next address */
////         len = (uint16_t)(len - size);
////         temp_buf = (uint8_t *)(temp_buf + size);
////         address = (uint16_t)(address + size);
////      }
////   }
////
////   /* To mimic the ET1x00 always providing AlEvent on every read or write */
////   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
////   ESCvar.ALevent = etohs (ESCvar.ALevent);
////}
////
/////* Un-used due to evb-lan9252-digio not havning any possability to
//// * reset except over SPI.
//// */
////void ESC_reset (void)
////{
////
////}
////
////void ESC_init (const esc_cfg_t * config)
////{
////   uint32_t value;
////   const char * spi_name = (char *)config->user_arg;
////   lan9252 = open (spi_name, O_RDWR, 0);
////
////   /* Reset the ecat core here due to evb-lan9252-digio not having any GPIO
////    * for that purpose.
////    */
////   lan9252_write_32(ESC_RESET_CTRL_REG,ESC_RESET_CTRL_RST);
////   do
////   {
////      value = lan9252_read_32(ESC_CSR_CMD_REG);
////   } while(value & ESC_RESET_CTRL_RST);
////
////
////
////}
