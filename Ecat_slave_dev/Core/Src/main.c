/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ecat_slv.h"
#include "utypes.h"
#include "TMC2209_usart.h"
#include "global.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 1. 모터 음계 VACTUAL 속도 정의 (4옥타브 기준 주파수 계산값)
#define NOTE_C 94000   // 도
#define NOTE_D 105000  // 레
#define NOTE_E 118000  // 미
#define NOTE_G 140000  // 솔
#define REST   0       // 쉼표 (정지)

#define NOTE_A3 78000
#define NOTE_B3 88000
#define NOTE_C4 93000
#define NOTE_D4 105000
#define NOTE_E4 118000
#define NOTE_G4 140000
#define NOTE_A4 157000
#define REST    0
#define DRV_ERROR_MASK 0xFE // 이 비트들 중 1개라도 1인 경우 구동에 문제가 생기는 하드웨어적 오류임

// 2. 비행기 계명 배열 (미-레-도-레-미-미-미...)
uint32_t melody[] = {
    NOTE_E, NOTE_D, NOTE_C, NOTE_D, NOTE_E, NOTE_E, NOTE_E,
    NOTE_D, NOTE_D, NOTE_D, NOTE_E, NOTE_G, NOTE_G,
    NOTE_E, NOTE_D, NOTE_C, NOTE_D, NOTE_E, NOTE_E, NOTE_E,
    NOTE_D, NOTE_D, NOTE_E, NOTE_D, NOTE_C
};

// 3. 각 음표의 길이 배열 (박자, ms 단위)
// 400은 4분음표(짧게), 800은 2분음표(길게), 1200은 점2분음표입니다.
uint32_t noteDurations[] = {
    400, 400, 400, 400, 400, 400, 800,
    400, 400, 800, 400, 400, 800,
    400, 400, 400, 400, 400, 400, 800,
    400, 400, 400, 400, 1200
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t g_StatusWord[MAX_AXIS] = {0x250, 0x250, 0x250, 0x250}; // 축별 상태정보 변수
bool g_is_homed[MAX_AXIS] = {false, false, false, false}; // 축별 홈 포지션 도착 정보 변수
uint32_t g_drv_status[MAX_AXIS] = {0x0, }; // 축별 제어 드라이버 상태 레지스터 32비트 정보
int32_t g_Actual_Pos[MAX_AXIS] = {0, }; // 축별 현위치

//레지스터 32비트 종류
uint16_t g_motcur[MAX_AXIS] = {0, }; // 축별 모터 전류 값
bool g_stealthMode[MAX_AXIS] = {NULL, NULL, NULL, NULL};
bool g_hwErr[MAX_AXIS] = {0, };
bool gw_overtemp_pre[MAX_AXIS] = {false, false, false, false};
bool gw_overtemp[MAX_AXIS] 	   = {false, false, false, false};
bool gw_overtemp_120[MAX_AXIS] = {false, false, false, false};
bool gw_overtemp_143[MAX_AXIS] = {false, false, false, false};
bool gw_overtemp_150[MAX_AXIS] = {false, false, false, false};
bool gw_overtemp_157[MAX_AXIS] = {false, false, false, false};
bool gw_shortGND_A[MAX_AXIS] = {false, false, false, false};
bool gw_shortGND_B[MAX_AXIS] = {false, false, false, false};
bool gw_shortMos[MAX_AXIS] 	 = {false, false, false, false};
bool gw_openload_A[MAX_AXIS] = {false, false, false, false};
bool gw_openload_B[MAX_AXIS] = {false, false, false, false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void SPI_Read(uint16_t address, void *buf, uint16_t len);
uint8_t LAN9252_HW_Init(void);
void togglepower(void);
void estegg(void);
void ethercat_chip_check(void);
void TMC_statecheck(void);
extern void servo_on(int axis);
extern void servo_off(int axis);
extern void ec_valinit();
extern uint32_t CalcMotion(int axis);
extern uint32_t motCtrl();
extern void CiA402_StateMachine(int axis);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 1. 마스터로 보낼 데이터 업데이트 (TxPDO)
void cb_get_inputs(void) {
    // 예: 4축 모터의 현재 엔코더 위치를 읽어와서 Wb 구조체에 넣습니다.
    Wb.actual_pos[0] = 0;//Motor1_Get_Encoder_Count();
    Wb.actual_pos[1] = 0;//Motor2_Get_Encoder_Count();
    Wb.actual_pos[2] = 0;//Motor3_Get_Encoder_Count();
    Wb.actual_pos[3] = 0;//Motor4_Get_Encoder_Count();

    // 모터 드라이버의 상태를 읽어서 상태 워드 업데이트
    Wb.status_word[0] = 0;//Motor_Get_System_Status();
    Wb.status_word[1] = 0;//Motor_Get_System_Status();
    Wb.status_word[2] = 0;//Motor_Get_System_Status();
    Wb.status_word[3] = 0;//Motor_Get_System_Status();
}

// 2. 마스터에서 온 데이터 적용 (RxPDO)
void cb_set_outputs(void) {
    // 마스터가 새로운 목표 위치나 제어 명령을 보냈으므로, 이를 모터 제어 변수에 반영합니다.
//    if (Rb.control_word[0] == 1) { // 예: Enable 명령이 들어왔다면
//        for(int i = 0; i < 4; i++) {
////            Motor_Set_Target_Position(i, Rb.target_pos[i]);
////            Motor_Set_Speed(i, Rb.target_speed[i]);
////            Motor_Set_Direction(i, Rb.direction[i]);
//        }
//    }
    if (ESCvar.ALstatus == ESC_AL_STATUS_OP) {

            // 4개 축에 대해 CiA 402 상태 머신 처리 (마스터 명령 반영)
            for (int axis = 0; axis < 4; axis++) {
                CiA402_StateMachine(axis);

                // (만약 서보가 ON 상태라면 여기서 펄스 가감속도 연산)
                if ((Wb.status_word[axis] & 0x0237) == 0x0237) {
                    CalcMotion(axis);
                }
            }
        } else {
            // OP 상태가 아니면 무조건 모터 정지 (안전 규격 준수)
            for (int axis = 0; axis < 4; axis++) {
                servo_off(axis);
            }
        }
}

// 3. SOES 스택 설정 구조체 연결
static esc_cfg_t config = {
    .user_arg = NULL,
    .use_interrupt = 0,
    .watchdog_cnt = 1000,

    .application_hook = cb_set_outputs, // 루프마다 실행될 함수 (데이터 적용)
    .safeoutput_override = NULL,
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to switch on leds ---- */

  ethercat_chip_check();

  BSP_LED_On(LED_RED);

  ecat_slv_init(&config);

  BSP_LED_On(LED_BLUE);

  HAL_TIM_Base_Start_IT(&htim2); // ethercat timer init with interrupt

  ec_valinit();
  BSP_LED_On(LED_GREEN);
  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //servo_on();
//  static uint8_t old_speed = 255;
//  static uint8_t old_dir = 255;
  //estegg();
  //HAL_Delay(1000);

  //motCtrl();
  //HAL_Delay(1000);
  while (1)
  {

    //ecat_slv();
	  TMC_statecheck();
	  for(int i = 0; i < MAX_AXIS; i++){
		  if(g_hwErr[i]){
			  g_StatusWord[i] |= (1 << 3); // 3번 비트 1 (Fault bit True)
			  servo_off(i);
			  continue; // 에러 난 축은 아래 제어 로직 건너뜀
		  }

		  // 2. 해당 축의 Control Word 분석 및 상태 머신 구동
		  uint32_t ctrl = Rb.control_word[i];

		  if (ctrl & (1 << 11)) {
		      // 해당 축만 강제 호밍
		      g_Actual_Pos[i] = 0; // Feedback position ==> pwm timer counter로 확인
		      g_is_homed[i] = true;
		  }

		  if ((ctrl & 0x000F) == 0x000F) {
		      // 해당 축만 서보 ON
		      Wb.status_word[i] |= 0x0237; // Operation Enabled
		      CalcMotion(i); // 해당 축만 펄스 계산 == 구현 필요
		  } else {
		      // 해당 축만 서보 OFF
		      servo_off(i);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//    if (ESCvar.ALstatus == ESC_OP) {												// OP모드 진입 시에만 작동
//        if (Rb.target_speed[0] != old_speed) {										// TwinCAT에서 보낸 값이 이전과 다를 때(변경되었을 때)만 UART 전송
//        	TMC2209_WriteRegister(&huart1, 0x00, 0x22, Rb.target_speed[0]);
//            //TMC2209_Update(&huart1, 0x00, Rb.target_speed[0], Rb.direction[0]);
//            // 현재 값을 기억
//            old_speed = Rb.target_speed[0];
//            //old_dir = (Rb.control_word << 16); 방향은 목표 위치나 목표 속도를 기준으로 결정
//        }
//        togglepower(); 																// OP 시에는 버튼으로 수동 조작 가능
//    }
//    else {																			// OP 가 아닌 경우
//        if (old_speed != 0) {														// 에러나 통신 끊김 시 정지 (이것도 중복 전송 방지)
//        	//TMC2209_WriteRegister(&huart1, 0x00, 0x22, 0);						// 정지 전류 => 속도 0로 설정
//        	servo_off(); 															// 통신 끊김 시 모터 전원 내림
//            old_speed = 0;															// 중복 방지를 위해 old_speed 값 변경
//        }
//    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 6;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ecat_CS_GPIO_Port, Ecat_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ecat_RST_GPIO_Port, Ecat_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Ecat_CS_Pin */
  GPIO_InitStruct.Pin = Ecat_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ecat_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ecat_RST_Pin */
  GPIO_InitStruct.Pin = Ecat_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ecat_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ecat_INT_Pin */
  GPIO_InitStruct.Pin = Ecat_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ecat_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_EN_Pin */
  GPIO_InitStruct.Pin = TMC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ethercat_chip_check()
{
  //---------------------LAN9252 ID Check---------------------------------
  // 1. 송신 버퍼 세팅: Fast Read(0x0B) + 주소(0x0050) + 더미(0x00) + 데이터 받을 빈 공간 4칸
  uint8_t tx_buf[8] = {0x0B, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx_buf[8] = {0};

  // 2. CS 핀 Low (PE2)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 8, HAL_MAX_DELAY);
  // 4. CS 핀 High (PE2)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  // 5. 앞의 4바이트(명령어 구간) 쓰레기값 무시하고 뒤의 4바이트 조립
  volatile uint32_t verify_id = rx_buf[4] | (rx_buf[5] << 8) | (rx_buf[6] << 16) | (rx_buf[7] << 24);

  (void)verify_id;

  __NOP();
  //------------------------------------------------------
  if(verify_id != 0x92520001)
  {
  	while(1){
  		HAL_Delay(100);
  		BSP_LED_On(LED_RED);
  		HAL_Delay(100);
  		BSP_LED_Off(LED_RED);
  	}
  }
}

void togglepower()
{
    if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)){
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
    }
}

//EtherCAT Timer - 1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	static int cnt = 0;
    	cnt++;
        // 동작 확인을 위해 보드의 LED를 토글해볼 수 있습니다.
        //cnt%2 == 0 ? BSP_LED_On(LED_RED) : BSP_LED_Off(LED_RED);
    	ecat_slv();
    }
}

//Read TMC Status Register
void TMC_statecheck()
{
	for (int i = 0; i < 4; i++) {
	    // UART로 DRV_STATUS를 천천히 읽어옴
	    uint32_t drv_status = TMC2209_ReadRegister(&huart1, i, 0x6F);
	    // TMC2208 Chip Error check
	    if ((drv_status & DRV_ERROR_MASK) != 0){
	    	g_hwErr[i] = true; // 비정상 ==> 비정상인 경우 해결될때까지 모터 구동 대기 함수 필요
	    }
//	    else {
//	    	//g_hwErr[i] = false; // 정상화 되면 복귀 지양
//	    }
	}
}

void estegg()
{
/* ========================================================== */
  /* 🎵 스텝 모터 연주 프로그램: "비행기 (Airplane)" 🎵 */
  /* ========================================================== */
  int totalNotes = sizeof(melody) / sizeof(melody[0]);

  // 4. 연주 시작 전 1초 대기
  HAL_Delay(1000);

  // 5. 악보를 읽으며 연주 시작!
  for (int i = 0; i < totalNotes; i++) {
      // 음표의 속도로 모터 회전 (소리 내기)
      TMC2209_WriteRegister(&huart1, 0x00, 0x22, melody[i]);

      // 음표의 길이만큼 유지하되, 음과 음 사이를 구분하기 위해 끝부분 50ms는 뺍니다.
      HAL_Delay(noteDurations[i] - 50);

      // ⭐️ 핵심: 음이 뭉개지지 않도록(스타카토 느낌) 아주 잠깐 모터를 세워줍니다.
      TMC2209_WriteRegister(&huart1, 0x00, 0x22, REST);
      HAL_Delay(50);
  }

  // 6. 연주가 끝나면 모터 정지
  TMC2209_WriteRegister(&huart1, 0x00, 0x22, 0);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);

}

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
