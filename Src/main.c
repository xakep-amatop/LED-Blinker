/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#define TIME_ON         (500) // in ms delay

#define BUFFER_RX_SIZE  (0x100)
#define BUFFER_TX_SIZE  (0x100)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint32_t     reset,  // change in interrupt handler for user button (blue)
                      state,
                      time_on     = TIME_ON,
                      was_command = 0;

volatile uint32_t _i = 0;

uint8_t   rx_buff[BUFFER_RX_SIZE] = {0};
uint8_t   tx_buff[BUFFER_TX_SIZE] = {0};

uint8_t   rx_data[2]              = {0};

volatile uint16_t  mesg_length    = 0;


// NOTE! don`t change command position in this array
static const uint8_t * commands[] =
  {
    (uint8_t *) "help",
    (uint8_t *) "set direction=left",
    (uint8_t *) "set direction=right",
    (uint8_t *) "set interval="
  };

#define number_commands (sizeof(commands) / sizeof(uint8_t * ))

static const uint8_t * info[] =
  {
    (uint8_t *) "\r\nUnknown command or incorrect argument!!!\r\n>",
    (uint8_t *) "\r\nOK!\r\n>"
  };

static const uint8_t msg_help[] =
  "\r\nInstructions for use!"
  "\r\n\r\n\t\tOPTIONS:\r\n"
  "    set direction=[left|right]\r\n"
  "    set interval=number_of_milliseconds_between_switching_of_LEDs\r\n\r\n"
  "    help - show this text\r\n\r\n>";

static uint32_t const leds[] =
   {
       GPIO_PIN_9  , GPIO_PIN_10 , GPIO_PIN_11 , GPIO_PIN_12 ,
       GPIO_PIN_13 , GPIO_PIN_14 , GPIO_PIN_15 , GPIO_PIN_8  ,

       GPIO_PIN_9  , GPIO_PIN_8  , GPIO_PIN_15 , GPIO_PIN_14 ,
       GPIO_PIN_13 , GPIO_PIN_12 , GPIO_PIN_11 , GPIO_PIN_10
   };

#define number_leds (sizeof(leds) / sizeof(leds[0]) / 2)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void RingBlinkerHandler(){
  static uint32_t i = 0;
  // turn on/off LEDs
  HAL_GPIO_TogglePin(GPIOE, leds[i + state * number_leds]);

  // turn off all LEDs if reset set
  if(reset){
    for(uint32_t j = 0; j < number_leds; ++j){
      HAL_GPIO_WritePin(GPIOE, leds[j], GPIO_PIN_RESET);
    }
    reset = 0;
    i     = 0;
  }
  else{
    i = (++i) % number_leds;
  }
}


static uint32_t SetDirectionLeft(uint32_t position, uint8_t * str){

  if(str[strlen((char*)commands[position])] != 0){
    return 0;
  }

  reset = 1;
  state = 1;
  return 1;
}

static uint32_t SetDirectionRight(uint32_t position, uint8_t * str){

  if(str[strlen((char*)commands[position])] != 0){
    return 0;
  }

  reset = 1;
  state = 0;
  return 1;
}

static uint32_t ShowHelp(uint32_t position, uint8_t * str){
  if(str[strlen((char*)commands[position])] != 0){
    return 0;
  }

  HAL_UART_Transmit_IT(&huart4, (uint8_t * ) msg_help, strlen((char*)msg_help));

  return 1;
}

static uint32_t SetInterval(uint32_t position, uint8_t * str){
  uint32_t start_nums = strlen((char*)commands[position]);

  uint32_t _time = strtoul((char*)(str + start_nums), (char**)&str, 10);

  if(!_time || *str != '\0' || errno == ERANGE){
    return 0;
  }

  time_on = _time;
  reset   = 1;
  return 1;
}

typedef uint32_t (*pfnCommands)(uint32_t, uint8_t *);

pfnCommands hCommands[] =
  {
    ShowHelp,
    SetDirectionLeft,
    SetDirectionRight,
    SetInterval
  };

void ParseCommand(){
  if(mesg_length == 0){
    HAL_UART_Transmit_IT(&huart4, (uint8_t * ) "\r\n>", sizeof "\r\n>");
    return;
  }

    uint8_t * substr_ptr = 0;

    for(uint32_t i = 0; i < number_commands; ++i){
      substr_ptr = (uint8_t *) strstr((char*) tx_buff, (char *) commands[i]);
      if (substr_ptr){
        if(substr_ptr != tx_buff){
          substr_ptr = 0;
          break;
        }
        substr_ptr = (uint8_t *) hCommands[i](i, substr_ptr);
        break;
      }
    }

    HAL_UART_Transmit_IT(&huart4, (uint8_t * )info[(uint32_t)substr_ptr], strlen((char*)info[(uint32_t)substr_ptr]));
}

/* USER CODE END 0 */

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
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
  sprintf((char*)tx_buff, "\r\n>");
  HAL_UART_Transmit_IT(&huart4, tx_buff, sizeof("\r\n>"));
  HAL_UART_Receive_IT(&huart4, rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tickstart = HAL_GetTick();
  while (1)
  {
    if((HAL_GetTick() - tickstart) >= time_on || reset){
      tickstart = HAL_GetTick();
      RingBlinkerHandler();
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if(was_command){
      was_command = 0;
      sprintf((char*)tx_buff, "%s", rx_buff);
      ParseCommand();
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
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

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
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
     PA11   ------> USB_DM
     PA12   ------> USB_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin 
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin 
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin 
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_0){
    state ^= 0x01;
    reset  = 0x01;
  }
}

volatile HAL_StatusTypeDef RX_status = HAL_OK;
volatile HAL_StatusTypeDef TX_status = HAL_OK;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == UART4){
    uint8_t sym = rx_data[0];

    if(isalnum((int)sym) || sym == '\n' || sym == '\r' || sym == '=' || sym == '\b' || sym == ' '){

      if(sym == '\b'){
        if(_i > 0){
          do{
          HAL_UART_AbortTransmit_IT(huart);
            TX_status = HAL_UART_Transmit_IT(huart, (uint8_t * )"\b \b", sizeof("\b \b"));
          }while(TX_status != HAL_OK);
          --_i;
        }
        goto _exit;
      }

      if(_i == sizeof(rx_buff) - 2 && sym != '\r'){
        goto _exit;
      }

      rx_buff[_i] = sym;

      if(rx_buff[_i] == '\r'){
        rx_buff[_i]     = 0;
        mesg_length     = _i;
        _i              = 0;
        was_command     = 1;
      }
      else{
        do{
          HAL_UART_AbortTransmit_IT(huart);
        TX_status = HAL_UART_Transmit_IT(huart, rx_data, 1);
      }while(TX_status != HAL_OK);
        ++_i;
      }
    }
    _exit:
   do{
     RX_status = HAL_UART_Receive_IT(huart,  rx_data, 1);
   }while(RX_status != HAL_OK);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
