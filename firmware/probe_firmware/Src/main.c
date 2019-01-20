
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "tim_defines.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}
/////////////////////////////////////////////////
//Settings


#define CHANNELS 16
#define ID 1 //must be < 256 and > 0 or else line termination will break
            // A maximum of 4080 channels!

#define SCALE_THRESHOLD 150

/////////////////////////////////////////////////
//Board calibration factors
/////////////////////////////////////////////////
#define PACKET_LENGTH (CHANNELS*2)+1+1+4 //channels + checksum + id + terminator
#define ID_INDEX (CHANNELS*2)+1
#define CHECKSUM_INDEX ID_INDEX+1
#define TERMINATOR_INDEX_1 CHECKSUM_INDEX+1
#define TERMINATOR_INDEX_2 CHECKSUM_INDEX+2
#define TERMINATOR_INDEX_3 CHECKSUM_INDEX+3
uint8_t tx_buffer[PACKET_LENGTH];
uint16_t channel_values[CHANNELS][2];//buffer1/2, channel number, ADC input x1/x10
float channel_calibration_factors[16];

uint8_t current_ADC_channel = 0; //0: x1, 1: x10
uint8_t current_input_buffer = 0;
uint8_t current_input_channel = 0;
uint8_t print_complete = 0;
uint8_t adc_complete = 0;

/////////////////////////////////////////////////

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void wait_cycles(uint32_t cycles){ //0.19 us per cycle
  volatile int cycle_counter = 0;
  for(cycle_counter = 0;cycle_counter<cycles;cycle_counter++){
  }
}

void switch_adc_channel(uint32_t channel){
  HAL_ADC_DeInit(&hadc);
  MX_ADC_Init();
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
//
// //on the other side
void unsigned_to_buffer(uint16_t input,uint8_t index){
  tx_buffer[index] = input & 0xff;
  tx_buffer[index+1] = (input >> 8);
}

uint8_t chksum8(const uint8_t *buff, size_t len){
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

void faster_pin_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = (uint32_t)GPIO_Pin;
  }
}


void select_mux_channel(uint8_t address){
    //ah darn! Why didn't I make the channels zero-indexed on the silkscreen?
    //That was tremendously stupid!
    //Let's let the convention be that all channel numbers are zero-indexed until they're graphed.

    faster_pin_write(S0_GPIO_Port, S0_Pin,(address >> 0) & 1U);
    faster_pin_write(S1_GPIO_Port, S1_Pin,(address >> 1) & 1U);
    faster_pin_write(S2_GPIO_Port, S2_Pin,(address >> 2) & 1U);
    faster_pin_write(S3_GPIO_Port, S3_Pin,(address >> 3) & 1U);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
* @brief This function handles ADC interrupt.
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    // channel_values[current_input_channel][current_ADC_channel] = (&hadc)->Instance->DR;
    // if(current_ADC_channel == 1 && current_input_channel < CHANNELS){
    //   current_input_channel++;
    // }
    // current_ADC_channel = !current_input_channel;
    //select_mux_channel(current_input_channel);
}
//

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //
  //switch_adc_channel(ADC_CHANNEL_0);
  //HAL_ADC_Start(&hadc);
  // HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(ADC1_IRQn);
  //HAL_ADC_Start_IT(&hadc);

  // switch_adc_channel(ADC_CHANNEL_0);
  //HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(ADC1_IRQn);
  //select_mux_channel(current_input_channel);
  HAL_ADC_Start(&hadc);
  TIM17->CR1 |= TIM_CR1_CEN;

  while (1){

    // //HAL_IWDG_Refresh(&hiwdg);

    for(int i = 0; i < 1 ; i++){
      for(int t = 0; t < PACKET_LENGTH; t++){
        tx_buffer[t] = 0;
      }

      for(int channel_number = 0; channel_number < CHANNELS; channel_number++){
        //this section must not take more than ~30 clock cycles.
        select_mux_channel(channel_number);

        //x1
        while(HAL_IS_BIT_CLR(((&hadc)->Instance->ISR), ADC_FLAG_EOS));
        //EOS = End of sample
        channel_values[channel_number][0] = (&hadc)->Instance->DR;

        //x10
        while(HAL_IS_BIT_CLR(((&hadc)->Instance->ISR), ADC_FLAG_EOC));
        //EOC = End of conversion sequence
        channel_values[channel_number][1] = (&hadc)->Instance->DR;
      }

      //
      // //HAL_ADC_Stop(&hadc);
      for(int channel_number = 0; channel_number < CHANNELS; channel_number++){
        //threshold x1 to check if we need to switch to x10
        if(channel_values[channel_number][0] > SCALE_THRESHOLD || channel_values[channel_number][0] < (4096-SCALE_THRESHOLD)){
          unsigned_to_buffer(channel_values[channel_number][0],channel_number*2);
        }
        else{
          unsigned_to_buffer(channel_values[channel_number][1],channel_number*2);
          //set the 7th bit of the high byte to 1.
          tx_buffer[(channel_number*2)+1] |= 1UL << 6;
        }
      }

      tx_buffer[CHECKSUM_INDEX] = chksum8(tx_buffer,CHECKSUM_INDEX);
      tx_buffer[TERMINATOR_INDEX_1] = 0xff;
      tx_buffer[TERMINATOR_INDEX_2] = 0x00;
      tx_buffer[TERMINATOR_INDEX_3] = 0xff;
      //wait for previous transmission to complete, then TX
      // TIM17->CNT = 0;

      //debug messages
      // for(int i =0 ; i < PACKET_LENGTH; i++){
      //   printf("%i,",tx_buffer[i]);
      // }
      // printf("\r\n");

      HAL_UART_Transmit(&huart1, (uint8_t *)&tx_buffer,PACKET_LENGTH,100);
      // printf("\r\n%i\r\n",TIM17->CNT);
    }

    // tx_buffer[INDEX_CHECKSUM] = chksum8(tx_buffer,NUMBER_OF_VALUES);
    // printf(TIM17->CNT);
    //HAL_ADC_Stop_IT(&hadc);
    //printf("D\r\n");
    //while(HAL_UART_Transmit(&huart1, (uint8_t *)&tx_buffer, PACKET_BYTE_COUNT) == HAL_BUSY);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
  huart1.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S0_Pin|S1_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_Pin */
  GPIO_InitStruct.Pin = S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S3_GPIO_Port, &GPIO_InitStruct);

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
  // while(1)
  // {
  printf("HAL ERROR: File: %s Line: %i",file,line);
  // }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
