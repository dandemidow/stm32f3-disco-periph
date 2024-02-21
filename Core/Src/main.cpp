/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Danila Demidov
  */
#include <cstdio>
#include <cmath>
#include <cstring>

extern "C" {
#include "main.h"
#include "pwm.h"
}
#include "position.h"
#include "ubx_state_machine.h"
#include "tx_message.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
static GpsState gps_state {};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_DMA_Init();
static void MX_USART2_UART_Init(uint32_t const);

extern "C" void initialise_monitor_handles(void);


typedef struct {
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tacc;
  int32_t nsec;
  uint8_t fixtype;
  uint8_t flags;
  uint8_t reserved1;
  uint8_t num_sv;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t heading;
  uint32_t sAcc;
  uint32_t headingAcc;
  uint16_t pDOP;
  uint16_t reserved2;
  uint32_t reserved3;
} nav_payload;

void navigate_parser(uint8_t *msg, size_t size) {
  printf("navigate message %d\r\n", size);
  nav_payload *p = (nav_payload *)msg;
  printf("date %d-%d-%d\r\n", p->day, p->month, p->year);
  printf("time %d-%d-%d\r\n", p->hour, p->min, p->sec);
  printf("sattl num %d\r\n", p->num_sv);
  printf("[%d, %d]\r\n", p->lon, p->lat);
  printf("height %d\r\n", p->height);
}

uint8_t aRxBuffer[kBufSize];
static int is_uart_ready = 0;
static int is_uart_tx_ready = 0;
static int is_uart_error = 0;
//LED_GREEN  = LED6,
//LED_ORANGE = LED5,
//LED_RED    = LED3,
//LED_BLUE   = LED4,
//LED_GREEN_2  = LED7,
//LED_ORANGE_2 = LED8,
//LED_RED_2    = LED10,
//LED_BLUE_2   = LED9
static Led_TypeDef leds[] = {LED_RED, LED_BLUE, LED_GREEN, LED_ORANGE_2, LED_RED_2, LED_BLUE_2, LED_GREEN_2, LED_ORANGE};

static void InitialGpsSetup() {
  std::array<uint8_t, 256> message {};

  MX_USART2_UART_Init(9600);

  CfgPort cfg {};
  cfg.port_id = 1U;
  cfg.mode = START | CH_8BIT | NO_PARITY;
  cfg.baudrate = 115200;
  cfg.in_proto_mask = Proto::kUbx | Proto::kNmea;
  cfg.out_proto_mask = Proto::kUbx;
  size_t const len {CreateMessage(cfg, message.data())};

  HAL_StatusTypeDef const result {HAL_UART_Transmit_DMA(&huart, message.data(), len)};
  if(result != HAL_OK) {
    Error_Handler();
  }
}

static void SetupGpsRate() {
  std::array<uint8_t, 256> message {};

  CfgRate rate {};
  rate.meas_rate = 5000;
  rate.nav_rate = 1;
  rate.time_ref = 1;
  size_t const len {CreateMessage(rate, message.data())};

  HAL_StatusTypeDef const result {HAL_UART_Transmit_DMA(&huart, message.data(), len)};
  if(result != HAL_OK) {
    Error_Handler();
  }
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  initialise_monitor_handles();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED6);
  BSP_PWM_Init();
  BSP_Position_Init();


  Led_TypeDef led_index {};


  InitialGpsSetup();
  while (!is_uart_tx_ready) {
    HAL_Delay(100);
    BSP_LED_Toggle(LED3);
  }
  is_uart_tx_ready = 0;
  if (HAL_UART_DeInit(&huart) != HAL_OK) {
    Error_Handler();
  }
  printf("tx done\r\n");
  MX_USART2_UART_Init(115200);
  memset(aRxBuffer, 0, kBufSize);
  gps_state.clear();
  HAL_StatusTypeDef result {HAL_UART_Receive_DMA(&huart, aRxBuffer, 2U)};
  if(result != HAL_OK) {
    Error_Handler();
  }
  size_t count {};
  while (!gps_state.data.is_ok) {
    HAL_Delay(1000);
    printf("read %d, write %d\r\n", gps_state.read_index_, gps_state.write_index_);
    if (gps_state.write_index_ - count < 5) {
       printf(" - %d   ", gps_state.write_index_ - count);
      for (size_t i {count}; i < gps_state.write_index_; i++) {
        printf("%02x ", gps_state.cyclic_[i]);
      }
      printf("\r\n");
    }
    count = gps_state.write_index_;
    gps_state.post_processing();
    BSP_LED_Toggle(LED3);
//    printf("st %d, %d\r\n", gps_state.st_index, gps_state.index);
//    for (size_t i {}; i < gps_state.index; i++) {
//      printf("%02x ", gps_state.test[i]);
//    }
//    printf("\r\n");
  }
  printf("receiving ok\r\n");

  gps_state.clear();
  SetupGpsRate();
  while (!is_uart_tx_ready) {
    HAL_Delay(100);
    BSP_LED_Toggle(LED3);
  }
  printf("setup ok\r\n");
  is_uart_tx_ready = 0;
  gps_state.clear();
  int dummy_counter = 50;

  while (1) {
//    BSP_LED_Toggle(led_index);
    if (led_index == 0) { BSP_LED_Toggle(LED4); }
    led_index = static_cast<Led_TypeDef>((static_cast<int>(led_index) + 1) % 8);
    printf("read %d, write %d\r\n", gps_state.read_index_, gps_state.write_index_);
    gps_state.post_processing();
    if (gps_state.data.is_ok) {
        printf("DONE!\r\n");
        for (size_t i {}; i < gps_state.data.size; i++) {
          printf("%02x ", gps_state.data.payload[i]);
        }
        printf("\r\n");
        if ((gps_state.st_msg_class.msg_class == 0x01U) &&
                (gps_state.st_msg_id.msg_id == 0x07)) {
          navigate_parser(gps_state.data.payload.data(), gps_state.data.size);
        }
//      HAL_UART_DMAPause(&huart);
        gps_state.clear();
    } else {
      HAL_Delay(1000);
    }
    if (dummy_counter-- == 0) {
        printf("read %d, write %d\r\n", gps_state.read_index_, gps_state.write_index_);
//        printf("st %d, size %d\r\n", gps_state.st_index, gps_state.data.size);
//        for (size_t i {}; i < 20; i++) {
//          printf("%02x ", gps_state.data.payload[i]);
//        }
//        printf("\r\n");
        BSP_LED_Toggle(LED10);
        dummy_counter = 50;
    }

    if (is_uart_error) {
        printf("error %d\r\n", huart.ErrorCode);
        is_uart_error = 0;
    }
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(uint32_t const baudrate) {
  huart.Instance = USART2;
  huart.Init.BaudRate = baudrate;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart) != HAL_OK) {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
  /* Turn LED3 on: Transfer in transmission process is correct */
  BSP_LED_On(LED3);
  is_uart_tx_ready = 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  gps_state.push(aRxBuffer[0]);
  BSP_LED_On(LED8);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  /* Turn LED5 on: Transfer in reception process is correct */
  BSP_LED_On(LED5);
  gps_state.push(aRxBuffer[1]);
  is_uart_ready = 1;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
  /* Turn LED6 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED6);
  is_uart_error = 1;
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

#ifdef  USE_FULL_ASSERT
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
