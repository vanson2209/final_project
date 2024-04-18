/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "math.h"
#include "rfid.h"
#include "flash.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {AUTO = 0, NONE_AUTO_REMOTE, NONE_AUTO_WEB, STOP} state_t;
typedef enum {PASS_START_1, LEFT_START_1, PASS_START_2, LEFT_START_2, PASS_START_3} state_start_t;
typedef enum {START = 0, PICK_UP, SHIP, COME_BACK, COME_HOME} state_process_t;
typedef enum {PRE_UP = 0, OPERATE, WAIT_DONE_PICK, DONE_PICK} state_pick_t;
typedef enum {PROCESS_SHIP = 0, GO_SHIP, PUT_DOWN, WAIT_DONE_SHIP, DONE_SHIP} state_ship_t;
typedef enum {PROCESS_GO = 0, WAIT_CROSS} state_go_t;
typedef enum {PRE_DOWN = 0, WAIT_ROBOT, PROCESS_PUT_DOWN, DONE_PUT_DOWN} state_put_down_t;
typedef enum {PRE_CB = 0, GO_CB} state_comeback_t;

typedef enum {CHECK_SIGN_LEFT, PASS_LEFT_1, TURN_LEFT, PASS_LEFT_2} state_turn_left_intersection_t;
typedef enum {CHECK_SIGN_RIGHT, TURN_RIGHT} state_turn_right_intersection_t;
typedef enum {CHECK_SIGN_CROSS, PASS_CROSS_1, PASS_CROSS_2} state_cross_intersection_t;

typedef enum {TS1 = 0, RS1 = 1, RS2 = 2, TS2 = 4, RS3 = 5, RS4 = 6} list_station_t;
typedef enum {UP = 0, DOWN, LEFT, RIGHT, PAUSE} direct_t;

typedef enum {reset = 0, set} bool_t;
//typedef enum{} state_ship_t;
//typedef enum{} state_comeback_t;
//typedef enum{} state_comeback_home_opp_t;
//typedef enum{} state_comeback_home_syn_t;

//typedef struct{
//	uint8_t start;
//	uint8_t pick;
//	uint8_t ship;
//	uint8_t come_back;
//	uint8_t come_home_opp;
//	uint8_t come_home_syn;
//} process_t;

typedef struct{
	bool_t goods_side[4];
	uint8_t goods_position[4];
	list_station_t goods_list[4];
	uint8_t goods_quantity;
	uint8_t goods_tmp;
} goods_infor_t;

typedef struct{
	uint8_t agv_position;
	direct_t agv_direct;
	direct_t agv_next_step_go;
	uint8_t agv_side;
	volatile bool_t agv_power;
} AGV_infor_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define line_sensor 	50
#define i_axis				6
#define j_axis				3
//#define GPIO_READ_PIN(GPIOx, GPIO_Pin_x) (GPIOx -> IDR & GPIO_Pin_x)
#define GPIO_SET_PIN(GPIOx, GPIO_Pin_x) (GPIOx -> ODR |= GPIO_Pin_x)
#define GPIO_RESET_PIN(GPIOx, GPIO_Pin_x) (GPIOx -> ODR &= ~GPIO_Pin_x)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t distance_tmp;
volatile uint32_t systick_count;
state_t state;
state_process_t process_state;
state_pick_t pick_state;
state_ship_t ship_state;
state_go_t state_go;
state_put_down_t state_put_down;
state_start_t start;
state_comeback_t state_comeback;

state_cross_intersection_t cross_interection_state = CHECK_SIGN_CROSS;
state_turn_left_intersection_t turn_left_interection_state = CHECK_SIGN_LEFT;
state_turn_right_intersection_t turn_right_interection_state = CHECK_SIGN_RIGHT;

volatile bool_t robot;

goods_infor_t goods_infor;
AGV_infor_t	agv_infor;

uint8_t Rx;
uint8_t Tx;
direct_t direct;

uint16_t data[5], data_max[5], data_min[5]; // gia tri mau tu
uint16_t sensor[5]; // gia tri doc mau tu
uint8_t check[5];	// gia tri kiem tra mau tu
uint16_t val1, val2; //gia tri bam xung pwm cho dng co

uint8_t str[5];
uint8_t ID_Scan[5]; // gia tri the tu
uint8_t ID[20]; // bo ma tu
//uint8_t direct;

uint8_t tmp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void V_Start_Motor(void);
void V_Go_Straight(void);
void V_Go_Back(void);
void V_Turn_Left(void);
void V_Turn_Right(void);
void V_Control_Remote(void);
void V_Control_Web(void);

uint8_t V_Check_Sensor_Init(void);
void V_Sensor_Init(void);
uint8_t V_Check_Sensor(void);

void V_Start(void);
void V_Pick_Up(void);
void V_Ship(void);
void V_Come_Back(void);
void V_Come_Home_Syn(void);
void V_Come_Home_Pos(void);

void V_Scan_ID(void);
int8_t V_Check_ID(void);

void V_Put_Down(void);
 
void V_Analysis_Goods_List(bool_t v_goods_side[4], uint8_t v_goods_position[4], list_station_t v_goods_list[4]);

void V_Cross_the_Intersection(void);
void V_Turn_Left_at_the_Intersection(void);
void V_Turn_Right_at_the_Intersection(void);

bool_t V_Check_Turn_Left(void);
bool_t V_Check_Go_Straight(void);

void V_Determine_Next_Step(void);
uint8_t V_Determine_Distance(uint8_t xo, uint8_t yo, uint8_t x1, uint8_t y1, direct_t v_direct);
void V_Remove_Duplicate(list_station_t v_goods_list[4],  uint8_t* goods_quantity);
void V_Remove_First_Goods(list_station_t v_goods_list[4],  uint8_t goods_quantity);
void V_GO(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	state = AUTO;
	process_state = START;
	pick_state = PRE_UP;
	ship_state = PROCESS_SHIP;
	state_go = PROCESS_GO;
	state_put_down = PRE_DOWN;
	start = PASS_START_1;
	robot = set;
	agv_infor.agv_power = reset;
	state_comeback = PRE_CB;

	cross_interection_state = CHECK_SIGN_CROSS;
	turn_left_interection_state = CHECK_SIGN_LEFT;
	turn_right_interection_state = CHECK_SIGN_RIGHT;
	V_Go_Straight();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	Lcd_Init();
	HAL_UART_Receive_IT(&huart1, &Rx, 1);
	Tx = '0';
	HAL_UART_Transmit(&huart1, &Tx, 1, 100);
	while(agv_infor.agv_power != set){}
	Tx = '1';
	HAL_UART_Transmit(&huart1, &Tx, 1, 100);
////	val1 = 9868;
////	val2 = 0;
////	V_Start_Motor();
//	//systick_count = HAL_GetTick();
	do
	{
//		Lcd_Send_String_XY(1, 4, "Sampling!");
//		Lcd_Goto_XY(2,1);
//		Lcd_Send_String("Wait, Please!");
		V_Sensor_Init();
	}
	while(V_Check_Sensor_Init());
	Tx = '2';
	HAL_UART_Transmit(&huart1, &Tx, 1, 100);
	htim2.Instance -> CCR1 = 8771;
	htim2.Instance -> CCR2 = 8888;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch(state)
		{
			case AUTO:
			{
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 5);
				HAL_Delay(1);
				HAL_ADC_Stop_DMA(&hadc1);
				switch(V_Check_Sensor())
				{
					case 0:
						htim2.Instance -> CCR1 = 9868;	//AGV1			//AGV1: val1:val2 = 9868:9999
						htim2.Instance -> CCR2 = 9999;
						break;
					case 1:
						htim2.Instance -> CCR1 = 9868;	//AGV2
						htim2.Instance -> CCR2 = 8888;
						break;
					case 2:
						htim2.Instance -> CCR1 = 9868;	//AGV2
						htim2.Instance -> CCR2 = 5555;
						break;
					case 3:
						htim2.Instance -> CCR1 = 9868;	//AGV2
						htim2.Instance -> CCR2 = 3333;
						break;
					case 4:
						htim2.Instance -> CCR1 = 9868;	//AGV2
						htim2.Instance -> CCR2 = 1666;
						break;
					case 5:
						htim2.Instance -> CCR1 = 8771;	//AGV2
						htim2.Instance -> CCR2 = 9999;
						break;
					case 6:
						htim2.Instance -> CCR1 = 5482;	//AGV2
						htim2.Instance -> CCR2 = 9999;
						break;
					case 7:
						htim2.Instance -> CCR1 = 3289;	//AGV2
						htim2.Instance -> CCR2 = 9999;
						break;
					case 8:
						htim2.Instance -> CCR1 = 1644;	//AGV2
						htim2.Instance -> CCR2 = 9999;
						break;
					case 9:
					{
						switch (process_state)
						{
							case START:
								V_Start();
								break;
							case PICK_UP:
								V_Pick_Up();
								break;
							case SHIP:
								V_Ship();
								break;
							case COME_BACK:
								V_Come_Back();
								break;
							case COME_HOME:
								break;
						}
						break;
					}
				}
				break;
			}
			case NONE_AUTO_REMOTE:
				V_Control_Remote();
				break;
			case NONE_AUTO_WEB:
				V_Control_Web();
				break;
			case STOP:
				htim2.Instance -> CCR1 = 0;
				htim2.Instance -> CCR2 = 0;
				break;
		}
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 69;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case GPIO_PIN_7:
			if(state == AUTO)
			{
				state = NONE_AUTO_REMOTE;
				htim2.Instance -> CCR1 = 0;
				htim2.Instance -> CCR2 = 0;
			}
			else
				state = AUTO;
			break;
		case GPIO_PIN_2:
			if(state == AUTO)
				state = STOP;
			else
				state = AUTO;
			break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	HAL_UART_Receive_IT(&huart1, &Rx, 1);
	switch (Rx)
	{
		case '0':
			agv_infor.agv_power = set;	//POWER_ON
			break;
		case '1':
			agv_infor.agv_power = reset;	//POWER_OFF
			break;
		case '2':		//off the automation
			state = NONE_AUTO_WEB;
			break;
		case '3':			//on the automation
			state = AUTO;
			break;
		case '4':
			robot = reset;
			break;
		case '5':
			goods_infor.goods_list[goods_infor.goods_quantity] = RS1;
			goods_infor.goods_quantity ++;
			break;
		case '6':
			goods_infor.goods_list[goods_infor.goods_quantity] = RS2;
			goods_infor.goods_quantity ++;
			break;
		case '7':
			goods_infor.goods_list[goods_infor.goods_quantity] = RS3;
			goods_infor.goods_quantity ++;
			break;
		case '8':
			goods_infor.goods_list[goods_infor.goods_quantity] = RS4;
			goods_infor.goods_quantity ++;
			break;
		case '9':
			direct = UP;
			break;
		case 'A':
			direct = DOWN;
			break;
		case 'B':
			direct = LEFT;
			break;
		case 'C':
			direct = RIGHT;
			break;
		case 'D':
			direct = PAUSE;
			break;
	}
}
void V_Go_Straight(void)
{
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_SET_PIN(GPIOA, GPIO_PIN_11);   //AIN1
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_12);	//AIN2
	GPIO_SET_PIN(GPIOB, GPIO_PIN_4);   //BIN2
	GPIO_RESET_PIN(GPIOB, GPIO_PIN_5);	//BIN1
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB`
}
void V_Go_Back(void)
{
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_11);   //AIN1
	GPIO_SET_PIN(GPIOA, GPIO_PIN_12);	//AIN2
	GPIO_RESET_PIN(GPIOB, GPIO_PIN_4);   //BIN2
	GPIO_SET_PIN(GPIOB, GPIO_PIN_5);	//BIN1
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB`
}
void V_Turn_Left(void)
{
	htim2.Instance -> CCR1 = 8771;
	htim2.Instance -> CCR2 = 8888;
	HAL_Delay(500);
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_11);   //AIN1
	GPIO_SET_PIN(GPIOA, GPIO_PIN_12);	//AIN2
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB
	HAL_Delay(350);
	systick_count = HAL_GetTick();
	do
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 5);
		HAL_Delay(1);
		HAL_ADC_Stop_DMA(&hadc1);
		tmp = V_Check_Sensor();
		if((tmp > 4) && (tmp < 9))
				break;
	} while((HAL_GetTick() - systick_count) < 500);
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_SET_PIN(GPIOA, GPIO_PIN_11);   //AIN1
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_12);	//AIN2
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB`
}
void V_Turn_Right(void)
{
	htim2.Instance -> CCR1 = 8771;
	htim2.Instance -> CCR2 = 8888;
	HAL_Delay(500);
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_RESET_PIN(GPIOB, GPIO_PIN_4);   //BIN2
	GPIO_SET_PIN(GPIOB, GPIO_PIN_5);	//BIN1
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	HAL_Delay(350);
	systick_count = HAL_GetTick();
	do
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 5);
		HAL_Delay(1);
		HAL_ADC_Stop_DMA(&hadc1);
		tmp = V_Check_Sensor();
		if((tmp > 4) && (tmp < 9))
				break;
	} while((HAL_GetTick() - systick_count) < 500);
	GPIO_RESET_PIN(GPIOA, GPIO_PIN_10);	//STB`
	GPIO_SET_PIN(GPIOB, GPIO_PIN_4);   //BIN1
	GPIO_RESET_PIN(GPIOB, GPIO_PIN_5);	//BIN2
	GPIO_SET_PIN(GPIOA, GPIO_PIN_10);	//STB
}
void V_Control_Remote(void)
{
	if((GPIOB->IDR & GPIO_PIN_1) != GPIO_PIN_RESET)		//UP
	{
		if((GPIOA->IDR & GPIO_PIN_12) != GPIO_PIN_RESET)
			V_Go_Straight();
		htim2.Instance -> CCR1 = 9900;
		htim2.Instance -> CCR2 = 9999;
	}
	else if((GPIOA->IDR & GPIO_PIN_5) != GPIO_PIN_RESET)	//DOWN
	{
		if((GPIOA->IDR & GPIO_PIN_11) != GPIO_PIN_RESET)
			V_Go_Back();
		htim2.Instance -> CCR1 = 9900;
		htim2.Instance -> CCR2 = 9999;
	}
	else if((GPIOB->IDR & GPIO_PIN_0) != GPIO_PIN_RESET)	//LEFT
	{
		htim2.Instance -> CCR1 = 2192;
		htim2.Instance -> CCR2 = 9999;
	}
	else if((GPIOA->IDR & GPIO_PIN_6) != GPIO_PIN_RESET)		//RIGHT
	{
		htim2.Instance -> CCR1 = 9868;
		htim2.Instance -> CCR2 = 2222;
	}
	else
	{
		htim2.Instance -> CCR1 = 0;
		htim2.Instance -> CCR2 = 0;
	}
}
void V_Control_Web(void)
{
	if((GPIOA->IDR & GPIO_PIN_5) == GPIO_PIN_SET)		//UP
	{
			V_Go_Straight();
	}
	else if((GPIOA->IDR & GPIO_PIN_6) == GPIO_PIN_SET)	//DOWN
	{
		V_Go_Back();
	}
	else if((GPIOA->IDR & GPIO_PIN_7) == GPIO_PIN_SET)	//LEFT
	{
		V_Turn_Left();
	}
	else if((GPIOB->IDR & GPIO_PIN_0) == GPIO_PIN_SET)		//RIGHT
	{
		V_Turn_Right();
	}
	else
	{
		htim2.Instance -> CCR1 = 0;
		htim2.Instance -> CCR2 = 0;
		V_Start_Motor();
	}
}
uint8_t V_Check_Sensor_Init(void) 
{
	uint8_t i, k = 0;
	for(i = 0; i < 5; i++)
	{
		if(((data_max[i] - data_min[i]) > 15) || ((sensor[i] - data[i]) > 10) || ((data[i] -sensor[i]) > 10))
			k++;
		else check[i] = 1;
	}
	if(k == 0) return 0;
	else return 1;
}
void V_Sensor_Init(void)
{
	uint8_t i;
	for( i = 0; i < 5; i++) 
	{
		if(check[i] == 0)
		{
			data_max[i] = 0;
			data_min[i] = ~0;
		}
	}
	systick_count = HAL_GetTick();
	while((HAL_GetTick() - systick_count) < 5000) 
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data, 5);
		HAL_Delay(1);
		HAL_ADC_Stop_DMA(&hadc1);
		if((HAL_GetTick() - systick_count) > 1000)
		{
			for(i = 0; i < 5; i++) 
			{
				if((check[i] == 0) && ((data[i] - data_min[i]) < 10) && ((data_max[i] - data[i]) < 10))
				{
					if(data[i] > data_max[i]) data_max[i] = data[i];
					if(data[i] < data_min[i]) data_min[i] = data[i];
				}
			}
		}
	}
	for(i = 0; i < 5; i++) sensor[i] = (55*data_max[i] + 45*data_min[i])/100; 
}
uint8_t V_Check_Sensor(void)
{
	if ((data[0] > sensor[0] + line_sensor) && (data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor) && (data[4] > sensor[4] + line_sensor))
		return 9;
	else if ((data[0] > sensor[0] + line_sensor) && (data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor))
		return 9;
	else if ((data[4] > sensor[4] + line_sensor) && (data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor))
		return 9;
	else if ((data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor))
		return 0;
	else if ((data[0] > sensor[0] + line_sensor) && (data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor))
		return 2;
	else if ((data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor) && (data[4] > sensor[4] + line_sensor))
		return 6;
	else if ((data[1] > sensor[1] + line_sensor) && (data[2] > sensor[2] + line_sensor))
		return 1;
	else if ((data[0] > sensor[0] + line_sensor) && (data[1] > sensor[1] + line_sensor))
		return 3;
	else if ((data[2] > sensor[2] + line_sensor) && (data[3] > sensor[3] + line_sensor))
		return 5;
	else if ((data[3] > sensor[3] + line_sensor) && (data[4] > sensor[4] + line_sensor))
		return 7;
	else if (data[0] > sensor[0] + line_sensor)
		return 4;
	else if (data[4] > sensor[4] + line_sensor)
		return 8;
	//else return 1;
}

void V_Start(void)
{
	switch(start)
	{
		case PASS_START_1:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			start = LEFT_START_1;
			break;
		case LEFT_START_1:
			V_Turn_Left();
			start = PASS_START_2;
			break;
		case PASS_START_2:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			start = LEFT_START_2;
			break;
		case LEFT_START_2:
			V_Turn_Left();
			start = PASS_START_3;
			break;
		case PASS_START_3	:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			agv_infor.agv_side = 0;
			agv_infor.agv_position = 0;
			agv_infor.agv_direct = LEFT;
		  process_state = PICK_UP;
			break;
	}
}

void V_Pick_Up(void)
{
	switch(pick_state)
	{	
		case PRE_UP:
				htim2.Instance -> CCR1 = 0;
				htim2.Instance -> CCR2 = 0;
				Lcd_Send_String_XY(1, 4, "PICKING!");
				Lcd_Goto_XY(2, 0);
			if(agv_infor.agv_side == 0)
				Tx = '3';
			else 
				Tx = '4';
			HAL_UART_Transmit(&huart1, &Tx, 1, 100);
			pick_state = OPERATE;
			break;
		case	OPERATE:
		{
			if(robot == reset)
			{
				goods_infor.goods_tmp = 0;
				V_Remove_Duplicate(goods_infor.goods_list, &goods_infor.goods_quantity);
				V_Analysis_Goods_List(goods_infor.goods_side, goods_infor.goods_position, goods_infor.goods_list);
				pick_state = WAIT_DONE_PICK;
			}
			else
			{
				if(goods_infor.goods_quantity > goods_infor.goods_tmp)
				{ // goods_infor.goods_list[goods_infor.goods_quantity_pre] = RS4;
					switch (goods_infor.goods_list[goods_infor.goods_tmp])
					{
						case RS1:
							Lcd_Send_String("-RS1");
							break;
						case RS2:
							Lcd_Send_String("-RS2");
							break;
						case RS3:
							Lcd_Send_String("-RS3");
							break;
						case RS4:
							Lcd_Send_String("-RS4");
							break;
						default:
							break;
					}
					goods_infor.goods_tmp = goods_infor.goods_quantity;
				}
			}
			break;
		}
		case WAIT_DONE_PICK:	//finish receiving
			Lcd_Send_String_XY(1, 0, "FINISH RECEIVING");
			systick_count = HAL_GetTick();
			pick_state = DONE_PICK;
			robot = set;
			break;
		case DONE_PICK:
//			HAL_UART_Transmit(&huart1, &Tx, 1, 100);
			if((HAL_GetTick() - systick_count) > 1500)
			{
				Lcd_Clear_Display();
				pick_state = PRE_UP;
				process_state = SHIP;
			}
			break;
	}
}

void V_Analysis_Goods_List(bool_t v_goods_side[4], uint8_t v_goods_position[4], list_station_t v_goods_list[4])
{
	for(uint8_t x = 0; x < goods_infor.goods_quantity; x++)
	{
		v_goods_position[x] = v_goods_list[x] & 3;
		v_goods_side[x] = 	(bool_t)((v_goods_list[x] & 4) >> 2);
	}
}

uint8_t V_Determine_Distance(uint8_t xo, uint8_t yo, uint8_t x1, uint8_t y1, direct_t v_direct)
{
	uint8_t result;
	if(v_direct == LEFT)
		result = pow((int8_t)((xo*i_axis + 4) - x1*i_axis), 2) + pow((int8_t)((yo*j_axis + 1) - y1*j_axis), 2);
	else
		result = pow((int8_t)(((xo - 1)*i_axis - 4) - x1*i_axis), 2) + pow((int8_t)((yo*j_axis - 1) - y1*j_axis), 2);
	return result;
}
void V_Remove_Duplicate(list_station_t v_goods_list[4],  uint8_t* goods_quantity)
{
	uint8_t x, y, z;
	for(x = 0; x < *goods_quantity; x++)			
	{
		for(y = x + 1; y < *goods_quantity; y++)
		{
			if(v_goods_list[x] == v_goods_list[y])
			{
				for(z = y; z < *goods_quantity - 1; z++)
				{
					v_goods_list[z] = v_goods_list[z+1];
				}
				(*goods_quantity)--;
				y--;	
			}
		}
				
	}
}
void V_Remove_First_Goods(list_station_t v_goods_list[4],  uint8_t goods_quantity)
{
	for(uint8_t x = 0;  x < goods_quantity; x++)
		v_goods_list[x] = v_goods_list[x+1];
}
bool_t V_Check_Turn_Left(void)	//set -> can turn
{
	if((GPIOB->IDR & GPIO_PIN_8) != (uint32_t)GPIO_PIN_RESET)
		return reset;
	else
		return set;
}
bool_t V_Check_Go_Straight(void)	//set -> can go
{
	if((GPIOB->IDR & GPIO_PIN_8) != (uint32_t)GPIO_PIN_RESET)
		return reset;
	else
		return set;
}

void V_Cross_the_Intersection(void)
{
	switch(cross_interection_state)
	{
		case CHECK_SIGN_CROSS:
			if(V_Check_Go_Straight() == set)
			{
				htim2.Instance -> CCR1 = 9868;
				htim2.Instance -> CCR2 = 9999;
				HAL_Delay(150);
				cross_interection_state = PASS_CROSS_1;
			}
			break;
		case PASS_CROSS_1:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			cross_interection_state = PASS_CROSS_2;
			break;
		case PASS_CROSS_2:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			if(agv_infor.agv_position == goods_infor.goods_position[0])
			{
				if(process_state == SHIP)
				{
					ship_state = PUT_DOWN;
				}
				else 
					process_state = PICK_UP;
			}
//			else if((agv_infor.agv_direct == LEFT) || agv_infor.agv_direct == RIGHT)
//				cross_interection_state = PASS_CROSS_3;
			state_go = PROCESS_GO;
			cross_interection_state = CHECK_SIGN_CROSS;
			break;
//		case PASS_CROSS_3:
//			state_go = PROCESS_GO;
//			cross_interection_state = CHECK_SIGN_CROSS;
//			htim2.Instance -> CCR1 = 9868;
//			val2 = 9999;
//			V_Start_Motor();
//			HAL_Delay(150);
//			break;
	}
}
void V_Turn_Left_at_the_Intersection(void)
{
	switch(turn_left_interection_state)
	{
		case CHECK_SIGN_LEFT:
			if(V_Check_Turn_Left() == set)
				htim2.Instance -> CCR1 = 9868;
				htim2.Instance -> CCR2 = 9999;
				HAL_Delay(150);
				turn_left_interection_state = PASS_LEFT_1;
			break;
		case PASS_LEFT_1:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			turn_left_interection_state = TURN_LEFT;
			break;
		case TURN_LEFT:
			V_Turn_Left();
			turn_left_interection_state = PASS_LEFT_2;
			break;
		case PASS_LEFT_2:
			htim2.Instance -> CCR1 = 9868;
			htim2.Instance -> CCR2 = 9999;
			HAL_Delay(150);
			if(agv_infor.agv_position == goods_infor.goods_position[0])
			{
				if(process_state == SHIP)
					ship_state = PUT_DOWN;
				else 
					process_state = PICK_UP;
			}
//			else if((agv_infor.agv_direct == LEFT) || agv_infor.agv_direct == RIGHT)
//				turn_left_interection_state = PASS_LEFT_3;
			state_go = PROCESS_GO;
			turn_left_interection_state = CHECK_SIGN_LEFT;
			break;
//		case PASS_LEFT_3:
//			state_go = PROCESS_GO;
//			turn_left_interection_state = CHECK_SIGN_LEFT;
//			val1 = 9868;
//			val2 = 9999;
//			V_Start_Motor();
//			HAL_Delay(150);
//			break;
	}
}

void V_Turn_Right_at_the_Intersection(void)
{
	switch(turn_right_interection_state)
	{
		case CHECK_SIGN_RIGHT:
			if(V_Check_Go_Straight() == set)
			{
				htim2.Instance -> CCR1 = 9868;
				htim2.Instance -> CCR2 = 9999;
				HAL_Delay(150);
				turn_right_interection_state = TURN_RIGHT;
			}
			break;
		case TURN_RIGHT:
			V_Turn_Right();
			if(agv_infor.agv_position == goods_infor.goods_position[0])
			{
				if(process_state == SHIP)
					ship_state = PUT_DOWN;
				else 
					process_state = PICK_UP;
			}
//			else if((agv_infor.agv_direct == LEFT) || agv_infor.agv_direct == RIGHT)
//				turn_right_interection_state = PASS_RIGHT_1;
			state_go = PROCESS_GO;
			turn_right_interection_state = CHECK_SIGN_RIGHT;
			break;
//		case PASS_RIGHT_1:
//			state_go = PROCESS_GO;
//			turn_right_interection_state = CHECK_SIGN_RIGHT;
//			val1 = 9868;
//			val2 = 9999;
//			V_Start_Motor();
//			HAL_Delay(150);
//			break;
	}
}

void V_Put_Down(void)
{
	switch(state_put_down)
	{
		case PRE_DOWN:
			htim2.Instance -> CCR1 = 0;
			htim2.Instance -> CCR2 = 0;
			if(agv_infor.agv_direct == LEFT)
			{
				if(agv_infor.agv_side == 0)
				{
					if(agv_infor.agv_position == 1)
						Tx = '5';
					else 
						Tx = '6';
				}
				else
				{
					if(agv_infor.agv_position == 1)
						Tx = '7';
					else 
						Tx = '8';
				}
			}
			else
			{
				if(agv_infor.agv_side == 1)
				{
					if(agv_infor.agv_position == 1)
						Tx = '5';
					else 
						Tx = '6';
				}
				else
				{
					if(agv_infor.agv_position == 1)
						Tx = '7';
					else 
						Tx = '8';
				}
			}
			HAL_UART_Transmit(&huart1, &Tx, 1, 100);
			Lcd_Send_String_XY(1, 2, "PUTTING DOWN");
			state_put_down = WAIT_ROBOT;
			break;
		case WAIT_ROBOT:
			if(robot == reset)
				state_put_down = PROCESS_PUT_DOWN;
			break;
		case PROCESS_PUT_DOWN:
			Lcd_Send_String_XY(1, 2, "DONE PUT DOWN");
			systick_count = HAL_GetTick();
			robot = set;
			goods_infor.goods_quantity--;
			ship_state = PROCESS_SHIP;
			V_Remove_First_Goods(goods_infor.goods_list, goods_infor.goods_quantity);
			V_Analysis_Goods_List(goods_infor.goods_side, goods_infor.goods_position, goods_infor.goods_list);
			state_put_down = DONE_PUT_DOWN;
			break;
		case DONE_PUT_DOWN:
			if((HAL_GetTick() -  systick_count) > 1500)
			{
				Lcd_Clear_Display();
				state_put_down = PRE_DOWN;
			}
			break;
	}
}
void V_Determine_Next_Step(void)
{
	switch(agv_infor.agv_direct)
	{
		case UP:
			if(goods_infor.goods_position[0] > (agv_infor.agv_position + 1))
			{
				agv_infor.agv_next_step_go = UP;
			}
			else
			{
				if(goods_infor.goods_side[0] < agv_infor.agv_side)
				{
					agv_infor.agv_next_step_go = RIGHT;
					agv_infor.agv_direct = RIGHT;
				}
				else
				{
					agv_infor.agv_direct = LEFT;
					agv_infor.agv_next_step_go = LEFT;
				}
			}
			agv_infor.agv_position++;
			break;
		case DOWN:
			if((goods_infor.goods_position[0] + 1)< agv_infor.agv_position)
			{
				agv_infor.agv_next_step_go = UP;
			}
			else
			{
				if(goods_infor.goods_side[0] < agv_infor.agv_side)
				{
					agv_infor.agv_next_step_go = LEFT;
					agv_infor.agv_direct = RIGHT;			
				}
				else
				{
					agv_infor.agv_direct = LEFT;
					agv_infor.agv_next_step_go = RIGHT;
				}
			}
			agv_infor.agv_position--;
			break;
		case LEFT:
			if(goods_infor.goods_position[0] == agv_infor.agv_position)
			{
				agv_infor.agv_next_step_go = UP;
			}
			else if(goods_infor.goods_position[0] < agv_infor.agv_position)
			{
				agv_infor.agv_next_step_go = LEFT;
				agv_infor.agv_direct = DOWN;
			}
			else
			{
				goods_infor.goods_tmp = 0;
				for(uint8_t x = 1; x < goods_infor.goods_quantity; x++)
				{
					if(goods_infor.goods_position[0] == goods_infor.goods_position[x])
					{
						goods_infor.goods_tmp = 1;
						break;
					}
				}
				if((goods_infor.goods_tmp == 1) && (goods_infor.goods_side[0] > agv_infor.agv_side))
				{	
					agv_infor.agv_next_step_go = UP;
				}
				else
				{
					agv_infor.agv_next_step_go = RIGHT;
					agv_infor.agv_direct = UP;
				}
			}
			agv_infor.agv_side++;
			break;
		case RIGHT:
			if(goods_infor.goods_position[0] == agv_infor.agv_position)
			{
				agv_infor.agv_next_step_go = UP;
			}
			else if(goods_infor.goods_position[0] < agv_infor.agv_position)
			{
				agv_infor.agv_next_step_go = RIGHT;
				agv_infor.agv_direct = DOWN;
			}
			else
			{
				goods_infor.goods_tmp = 0;
				for(uint8_t x = 1; x < goods_infor.goods_quantity; x++)
				{
					if(goods_infor.goods_position[0] == goods_infor.goods_position[x])
					{
						goods_infor.goods_tmp = 1;
						break;
					}
				}
				if((goods_infor.goods_tmp == 1) && ((goods_infor.goods_side[0] + 1) < agv_infor.agv_side))
				{	
					agv_infor.agv_next_step_go = UP;
				}
				else
				{
					agv_infor.agv_next_step_go = LEFT;
					agv_infor.agv_direct = UP;
				}
			}
			agv_infor.agv_side--;
			break;
		default:
			break;
	}
}
void V_GO(void)
{
	switch(state_go)
	{
		case PROCESS_GO:
		{	
			V_Determine_Next_Step();
			state_go = WAIT_CROSS;
			break;
		}
		case WAIT_CROSS:
		{
			switch(agv_infor.agv_next_step_go)
			{
				case UP:
					V_Cross_the_Intersection();
					break;
				case LEFT:
					V_Turn_Left_at_the_Intersection();
					break;
				case RIGHT:
					V_Turn_Right_at_the_Intersection();
					break;
				default:
					break;
			}
			break;
		}
	}
}
void V_Ship(void)
{
	uint8_t distance_min;
//	uint8_t distance_tmp;
	list_station_t tmp;
	switch (ship_state)
	{
		case PROCESS_SHIP:
		{	
			if(goods_infor.goods_quantity == 0)
				ship_state = WAIT_DONE_SHIP;
			else if(goods_infor.goods_quantity > 1)
			{
				distance_min = 255;
				for(uint8_t x = 0; x < goods_infor.goods_quantity; x++)
				{
					distance_tmp = V_Determine_Distance(agv_infor.agv_side, agv_infor.agv_position, goods_infor.goods_side[x], goods_infor.goods_position[x], agv_infor.agv_direct);
					if(distance_tmp < distance_min)
					{
							goods_infor.goods_tmp = x;
							distance_min = distance_tmp;
					}
				}
				tmp = goods_infor.goods_list[0];
				goods_infor.goods_list[0] = goods_infor.goods_list[goods_infor.goods_tmp];
				goods_infor.goods_list[goods_infor.goods_tmp] = tmp;
				V_Analysis_Goods_List(goods_infor.goods_side, goods_infor.goods_position, goods_infor.goods_list);
				Tx = 'D';
				HAL_UART_Transmit(&huart1, &Tx, 1, 100);
				ship_state = GO_SHIP;
			}
			else{
				ship_state = GO_SHIP;
				Tx = 'D';
				HAL_UART_Transmit(&huart1, &Tx, 1, 100);
			}
			break;
		}
		case GO_SHIP:
		{
			V_GO();
			break;
		}
		case PUT_DOWN:
			V_Put_Down();
			break;
		case WAIT_DONE_SHIP:
			if((agv_infor.agv_direct == RIGHT) && (agv_infor.agv_side == 0))
				goods_infor.goods_list[0] = TS1;
			else
				goods_infor.goods_list[0] = TS2;
			ship_state = DONE_SHIP;
			break;
		case DONE_SHIP:
			goods_infor.goods_quantity = 0;
			ship_state = PROCESS_SHIP;
			state_comeback = PRE_CB;
			process_state = COME_BACK;
			break;
	 }
}

void V_Come_Back(void)
{
	switch(state_comeback)
	{
	
		case PRE_CB:
			goods_infor.goods_position[0] = goods_infor.goods_list[0] & 3;
			goods_infor.goods_side[0] = 	(bool_t)((goods_infor.goods_list[0] & 4) >> 2);
			state_comeback = GO_CB;
			break;
		case GO_CB:
			V_GO();
			break;
	}
}
void V_Come_Home_Syn(void);
void V_Come_Home_Pos(void);
void V_Start_Motor(void)
{
	htim2.Instance -> CCR1 = val1;
	htim2.Instance -> CCR2 = val2;
}

void V_Scan_ID(void)
{
	for(uint8_t i = 0; i < 100; i++){
		if(!MFRC522_Request(0x26, str)){
			if(!MFRC522_Anticoll(str)){
				for(uint8_t i = 0; i < 5; i++){
					ID_Scan[i] = str[i];
				}
			}
		}
		if(ID_Scan[1] != 0) break;
	}
	Flash_Erase(0x800A000);  //ghi vao bo id
	Flash_Write_Array(0x800A000, ID_Scan, 5);
	for(uint8_t i = 0; i < 5; i++){
			str[i] = 0;
			ID_Scan[i] = 0;
	}
}
int8_t V_Check_ID(void)
{
	AntennaOn();
	uint8_t t = 0, m, n;
	int8_t cout = -1;
	uint32_t a = 0x800A000;
	uint32_t b = 0x800B000;
	V_Scan_ID();
	AntennaOff();
	for(uint8_t j = 0; j < 5; j++){
		for(uint8_t i = 0; i < 5; i++){
			m = Flash_Read_Int(a);
			n = Flash_Read_Int(b);
			if(m == n) t++;
			a++;
			b++;
		}
		cout++;
		if(t == 5) break;
		else{
			t = 0;
			a = 0x800A000;
		}
	}
	Flash_Erase(0x800A000);
	if(t == 5) return cout;
	else return -1;	
}
/* USER CODE END 4 */

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
