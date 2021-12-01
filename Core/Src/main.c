/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include "KeyboardHoldRepeat.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Sets the color assignment for each bin
typedef enum {
      RBG,
      BGR,
      GRB,
      GBR,
      RGB,
      BRG
} COLOR_MODE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Color Bin Values */
#define RED_LVL         18
#define GRN_LVL         45
#define BLUE_LVL        140

/* Minimum bucket value */
#define CUTOFF_LVL      7000

/* Max LED PWM Pulse (Everything else is too bright) */
#define LED_MAX         5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

// scheduler stuff
extern uint8_t ten_mS_Flag;
extern uint8_t twentyfive_mS_Flag;
extern uint8_t hundred_mS_Flag;
extern uint8_t one_S_Flag;

// DMA Flags for processing first/second half of data 
extern volatile uint32_t process_First;
extern volatile uint32_t process_Second;

// PWM Structs to easily configure the LED Outputs 
TIM_OC_InitTypeDef TRI_RED = {0};
TIM_OC_InitTypeDef TRI_GREEN = {0};
TIM_OC_InitTypeDef TRI_BLUE = {0};

// Flag set when DMA data has been processed and bins filled 
uint8_t FFT_DATA_READY = false;

// Buckets for adding together frequency bins 
uint16_t Bucket_Low = 0;
uint16_t Bucket_Med = 0;
uint16_t Bucket_High = 0;

COLOR_MODE MODE;

uint8_t Button_Code;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void Fill_Buckets ();
void Display_FFT(COLOR_MODE MODE);

uint16_t data[1024];
float FFT_OUT[512];
float FFT_data[1024];

volatile uint8_t ms25_Count = 0;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  hdma_adc1.Instance->CCR = 1409;
  HAL_TIM_Base_Start(&htim6);
  
  TRI_RED.OCMode = TIM_OCMODE_PWM1;
  TRI_RED.Pulse = 0;
  TRI_RED.OCPolarity = TIM_OCPOLARITY_HIGH;
  TRI_RED.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TRI_RED.OCFastMode = TIM_OCFAST_DISABLE;
  TRI_RED.OCIdleState = TIM_OCIDLESTATE_RESET;
  TRI_RED.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  TRI_GREEN.OCMode = TIM_OCMODE_PWM1;
  TRI_GREEN.Pulse = 0;
  TRI_GREEN.OCPolarity = TIM_OCPOLARITY_HIGH;
  TRI_GREEN.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TRI_GREEN.OCFastMode = TIM_OCFAST_DISABLE;
  TRI_GREEN.OCIdleState = TIM_OCIDLESTATE_RESET;
  TRI_GREEN.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  TRI_BLUE.OCMode = TIM_OCMODE_PWM1;
  TRI_BLUE.Pulse = 0;
  TRI_BLUE.OCPolarity = TIM_OCPOLARITY_HIGH;
  TRI_BLUE.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TRI_BLUE.OCFastMode = TIM_OCFAST_DISABLE;
  TRI_BLUE.OCIdleState = TIM_OCIDLESTATE_RESET;
  TRI_BLUE.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    
    //---------------------------------
    // 10mS Tasks 
    if (ten_mS_Flag) {
      ten_mS_Flag = false;

    }  // end of 10mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 25mS Tasks 
    if (twentyfive_mS_Flag) {
      twentyfive_mS_Flag = false;
      ms25_Count++;
      if(ms25_Count++ > 9){
        ms25_Count = 0;
        HAL_ADC_Start_DMA(&hadc1,(uint32_t*)data,1024);
      }
         
    }  // end of 25mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 100mS Tasks 
    if (hundred_mS_Flag) {
      hundred_mS_Flag = false;
      Button_Code = ScanKeyboard();
      
    }  // end of 100mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 1 Sec Tasks 
    if (one_S_Flag) {
      one_S_Flag = false;
      
    } // end of 1Sec Tasks
    //---------------------------------

    
    //---------------------------------
    // Every time through the loop

    if(process_First){
      process_First = 0;
      int j = 0;
      for(int i=0; i<512;i++){
        FFT_data[j] = (float)data[i]-1986.0;
        FFT_data[j+1] = 0.0;
        j+=2;
      }
      /* Process the data through the CFFT/CIFFT module */
      arm_cfft_f32(&arm_cfft_sR_f32_len512, FFT_data, 0, 1);

      /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin*/
      arm_cmplx_mag_f32(FFT_data, FFT_OUT, 512);
      FFT_DATA_READY = true;
    }
    else if(process_Second){
      process_Second = 0;
      int j = 0;
      for(int i=512; i<1024;i++){
        FFT_data[j] = (float)data[i]-1986.0;
        FFT_data[j+1] = 0.0;
        j+=2;
      }
      /* Process the data through the CFFT/CIFFT module */
         arm_cfft_f32(&arm_cfft_sR_f32_len512, FFT_data, 0, 1);

        /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin*/
        arm_cmplx_mag_f32(FFT_data, FFT_OUT, 512);
        FFT_DATA_READY = true;
    }
    
    if(FFT_DATA_READY){
      FFT_DATA_READY = false;
      Fill_Buckets();
      Display_FFT(MODE);
    }
    

    // end Every time through the loop
    //---------------------------------
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW4_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW4_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Fill_Buckets (){
  
  //Unfill all buckets
  Bucket_Low = 0;
  Bucket_Med = 0;
  Bucket_High = 0;

  //Skip DC bucket (It is always too large)
  int i= 1;
  
  //Fill each bucket with appropriate bins (Set in definitions section at top of main.c)
  for(;i<RED_LVL;i++){
    Bucket_Low+= (uint16_t) FFT_OUT[i];
  }
  for (;i<GRN_LVL;i++){
    Bucket_High+=(uint16_t)FFT_OUT[i];
  }

  for(;i<BLUE_LVL;i++){
    Bucket_Med+=(uint16_t)FFT_OUT[i];
  }
}


void Display_FFT(COLOR_MODE MODE){
  uint16_t Pulse_Low, Pulse_Med, Pulse_High;
  
  //Find total so you can weight the LED intensity
  uint16_t total = Bucket_Low+Bucket_High+Bucket_Med;
  
  //Make sure the bucket content is large enough to display
  if(Bucket_Low > CUTOFF_LVL){
    
    //Calculate the percentage of that bucket in relation to total frequency content
    Pulse_Low = (uint16_t)(((float)Bucket_Low/(float)total)*LED_MAX);
  }
  
  //If there is low audio signal then do not turn on LED
  else{
    Pulse_Low = 0;
  }
  if(Bucket_Med > CUTOFF_LVL){
    Pulse_Med = (uint16_t)(((float)Bucket_Med/(float)total)*LED_MAX);
  }    
  else{
    Pulse_Med = 0;
  }
  if(Bucket_High > CUTOFF_LVL){ 
    Pulse_High = (uint16_t)(((float)Bucket_High/(float)total)*LED_MAX);
  }  
  else{
    Pulse_High = 0;
  }
  
  //Set the color pattern for each bucket
  switch(MODE){
    case (RBG):
      TRI_RED.Pulse = Pulse_Low;
      TRI_BLUE.Pulse = Pulse_Med;
      TRI_GREEN.Pulse = Pulse_High;
      break;
    case (BGR):
      TRI_RED.Pulse = Pulse_High;
      TRI_BLUE.Pulse = Pulse_Low;
      TRI_GREEN.Pulse = Pulse_Med;
      break;
    case (GRB):
      TRI_RED.Pulse = Pulse_Med;
      TRI_BLUE.Pulse = Pulse_High;
      TRI_GREEN.Pulse = Pulse_Low;
      break;
    case (GBR):
      TRI_RED.Pulse = Pulse_High;
      TRI_BLUE.Pulse = Pulse_Med;
      TRI_GREEN.Pulse = Pulse_Low;
      break;
    case (RGB):
      TRI_RED.Pulse = Pulse_Low;
      TRI_BLUE.Pulse = Pulse_High;
      TRI_GREEN.Pulse = Pulse_Med;
      break;
    case (BRG):
      TRI_RED.Pulse = Pulse_Med;
      TRI_BLUE.Pulse = Pulse_Low;
      TRI_GREEN.Pulse = Pulse_High;
      break;
      
  default:
    TRI_RED.Pulse = Pulse_Low;
    TRI_BLUE.Pulse = Pulse_Med;
    TRI_GREEN.Pulse = Pulse_High;
    break;  
  }
  
  //Reset all the PWM signals for new frequency weights
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1, &TRI_RED, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &TRI_BLUE, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &TRI_GREEN, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
