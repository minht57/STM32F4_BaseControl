
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "PID.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_BUF_DATA_RECV   20

uint8_t ui8_BufLog[50];
uint8_t u8_rec_data;
uint8_t UART_Buf[MAX_BUF_DATA_RECV];
uint16_t UART_ReadIdx = 0;
uint16_t UART_WriteIdx = 0;
uint16_t u16_avail_byte = 0;
int32_t i32_countZeroCmd = 0;

int16_t i16_CountPulseLeft, i16_CountPulseRight;

uint16_t count_reset = 0;
uint8_t count_send = 0;

PID_PARAMETERS PID_ParaMotor_Left = {.Kp = 0.020, .Kd = 0.0, .Ki = 0.005,
             .Ts = 0.020, .PID_Saturation = 450, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS PID_ParaMotor_Right = {.Kp = 0.020, .Kd = 0.0, .Ki = 0.005,
             .Ts = 0.020, .PID_Saturation = 450, .e_=0, .e__=0, .u_=0};

LPF_PARAMETERS LPF_EncoderLeft = {.counter = 0, .result = 0, .alpha = 0.8};
LPF_PARAMETERS LPF_EncoderRight = {.counter = 0, .result = 0, .alpha = 0.8};

TARGET_VELOCITY_PARAMETERS Target_Velocity = {.angle_velocity = 0, .linear_velocity = 0,
                                              .i16_Pulse_Left = 0, .i16_Pulse_Right = 0,
                                              .radius = 0.035, .length = (0.3*1), .ratio = 1};
TARGET_VELOCITY_PARAMETERS Result_Velocity = {.angle_velocity = 0, .linear_velocity = 0,
                                              .i16_Pulse_Left = 0, .i16_Pulse_Right = 0,
                                              .radius = 0.035, .length = (0.3*1), .ratio = 1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int8_t UART_GetCmd(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void UART_Log(uint8_t* log)
{
  HAL_UART_Transmit_DMA(&huart2, log, strlen((const char *)log));
}

int16_t LPF_Encoder(LPF_PARAMETERS* LPF_params, int16_t counter)
{
  LPF_params->counter = counter;
  LPF_params->result = LPF_params->result + LPF_params->alpha * (LPF_params->counter - LPF_params->result);
  return LPF_params->result;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim3.Instance)
  {
    if(UART_GetCmd() == 0)
    {
      i32_countZeroCmd++;
    }
    else
    {
      i32_countZeroCmd = 0;
    }
    
    if(i32_countZeroCmd > 50)
    {
      Target_Velocity.linear_velocity = 0.05;
      Target_Velocity.angle_velocity = 0;
      Step_CalVelocity(&Target_Velocity);
    }
    
    if(count_send > 10)
    {
      count_send = 0;
      
      sprintf((char*) ui8_BufLog,"%d \t %d \t %d \t %d\n\r",(int16_t)(Target_Velocity.linear_velocity*1000),
                                                            (int16_t)(Result_Velocity.linear_velocity*1000),
                                                            (int16_t)(Target_Velocity.angle_velocity*1000),
                                                            (int16_t)(Result_Velocity.angle_velocity*1000));
  //    sprintf((char*) ui8_BufLog,"%f \t %f \t %f \t %f\n\r",Target_Velocity.linear_velocity*1000,Result_Velocity.linear_velocity*1000,
  //                                                          Target_Velocity.angle_velocity*1000, Result_Velocity.angle_velocity*1000);
//      sprintf((char*) ui8_BufLog,"%d \t %d \t %d \t %d\n\r",(int16_t)(Target_Velocity.linear_velocity*1000),
//                                                            (int16_t)(Result_Velocity.linear_velocity*1000),
//                                                            (int16_t)(Target_Velocity.angle_velocity*1000),
//                                                            (int16_t)(Result_Velocity.angle_velocity*1000));
      
//      sprintf((char*) ui8_BufLog,"%d \t %d\n\r", Target_Velocity.i16_Pulse_Right, i16_Counter_Right_LPF);
//      sprintf((char*) ui8_BufLog,"%d \t %d \t %d \t %d\n\r", i16_Error_Left, (int16_t)i16_PIDScale_Left, (int16_t)i16_Error_Right, (int16_t)i16_PIDScale_Right);
//      sprintf((char*) ui8_BufLog,"%d \t %d \t %d \t %d\n\r",i16_Counter_Left, i16_Counter_Left_LPF, i16_Counter_Right, i16_Counter_Right_LPF);
      UART_Log(ui8_BufLog);
    }
    else
    {
      count_send++;
    }
  }
  else if (htim->Instance == htim4.Instance)
  {
    if (count_reset >= 1000)
    {      
      if(Target_Velocity.i16_Pulse_Left > 0)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, STEP_FORWARD_LEFT);
        if(i16_CountPulseLeft > Target_Velocity.i16_Pulse_Left)
        {
          i16_CountPulseLeft = 0;
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        }
        else
        {
          i16_CountPulseLeft++;
        }
      }
      else if(Target_Velocity.i16_Pulse_Left < 0)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, STEP_BACK_LEFT);
        if(i16_CountPulseLeft > -Target_Velocity.i16_Pulse_Left)
        {
          i16_CountPulseLeft = 0;
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        }
        else
        {
          i16_CountPulseLeft++;
        }
      }
      else
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
      }
      
      if(Target_Velocity.i16_Pulse_Right > 0)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, STEP_FORWARD_RIGHT);
        if(i16_CountPulseRight > Target_Velocity.i16_Pulse_Right)
        {
          i16_CountPulseRight = 0;
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
        else
        {
          i16_CountPulseRight++;
        }
      }
      else if(Target_Velocity.i16_Pulse_Right < 0)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, STEP_BACK_RIGHT);
        if(i16_CountPulseRight > -Target_Velocity.i16_Pulse_Right)
        {
          i16_CountPulseRight = 0;
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
        else
        {
          i16_CountPulseRight++;
        }
      }
      else
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
      }
    }
    else 
    {
        count_reset++;
    }
  }
}

uint16_t UART_QueryData(void){
  return (u16_avail_byte);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart2.Instance)
	{
    if(((UART_WriteIdx == (MAX_BUF_DATA_RECV -1))&&(UART_ReadIdx != 0))||
       ((UART_WriteIdx != (MAX_BUF_DATA_RECV -1))&&((UART_WriteIdx+1) != UART_ReadIdx)))
    {
        UART_Buf[UART_WriteIdx++] = u8_rec_data;
        u16_avail_byte++;
        UART_WriteIdx %= MAX_BUF_DATA_RECV;
    }
  }
}

void UART_C1101_Read(uint8_t * buf, uint16_t len)
{
    uint16_t idx;
    if(UART_QueryData() >= len){
        for(idx = 0; idx < len; idx++){
            if (UART_ReadIdx != UART_WriteIdx)
            {
              *(buf + idx) = UART_Buf[UART_ReadIdx++];
              if (u16_avail_byte)
              {
                u16_avail_byte--;
              }
            }
            if(UART_ReadIdx >= MAX_BUF_DATA_RECV){
                UART_ReadIdx = 0;
            }
        }
    }
}

int8_t UART_GetCmd(void)
{
  static int8_t Buf_temp[20];
  static int8_t i8_HaveCmd = 0;
  static int8_t i8_InProcessing = 0;
  static int8_t i8_Idx = 0;
  static uint8_t u8_Temp;
  char* cToken;
  while(UART_QueryData() > 0)
  {
    UART_C1101_Read(&u8_Temp, 1);
    if(u8_Temp == ']')
    {
      i8_InProcessing = 0;
      //
      cToken = strtok((char*)Buf_temp, ",");
      if(cToken != NULL)
      {
          Target_Velocity.linear_velocity = atof(cToken);
          cToken = strtok(NULL,",");
          if(cToken != NULL)
          {
              Target_Velocity.angle_velocity = atof(cToken);
          }
      }
      if(Target_Velocity.linear_velocity > 0.5)
      {
        Target_Velocity.linear_velocity = 0.5;
      }
      if(Target_Velocity.angle_velocity > 3)
      {
        Target_Velocity.angle_velocity = 3;
      }
      else if(Target_Velocity.angle_velocity < -3)
      {
        Target_Velocity.angle_velocity = -3;
      }
      Step_CalVelocity(&Target_Velocity);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
      return 1;
    }
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    if(i8_InProcessing)
    {
      Buf_temp[i8_Idx++] = u8_Temp;
    }
    if(u8_Temp == '[')
    {
      i8_Idx = 0;
      i8_InProcessing = 1;
    }
  }
  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  pid_reset(&PID_ParaMotor_Left);
  pid_reset(&PID_ParaMotor_Right);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  
  HAL_UART_Receive_DMA(&huart2, &u8_rec_data,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE7 PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
