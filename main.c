/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define  UART_ENABLE_RE(USARTx)       USARTx.Instance->CR1|= (uint32_t)0x0004            
#define  UART_DISABLE_RE(USARTx)      USARTx.Instance->CR1&= (~(uint32_t)0x0004)   
#define  ERROR_MAX 20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t adcbuf[6]={0};
uint8_t tx_buff[16] = {0x44, 0x52};

typedef	int filter_type;

extern uint8_t Uart1_RxFlag;
extern uint8_t Uart1_RxBuff[10];

float sum=0;
const float effective_value = 10.0;
float adcreal[6] = {0};
float R[6] = {0};
float temper[6] = {0};
float last_temprt[6] = {0};
  
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart1 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}

void rs485_trx(uint8_t input){
	if(input==1){
		HAL_GPIO_WritePin(rx_en_GPIO_Port, rx_en_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(tx_en_GPIO_Port, tx_en_Pin, GPIO_PIN_RESET);
	}
	if(input==0){
		HAL_GPIO_WritePin(tx_en_GPIO_Port, tx_en_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(rx_en_GPIO_Port, rx_en_Pin, GPIO_PIN_SET);
	}
}


/*
 普通定时器实现us延时
*/
void user_delaynus_tim(uint32_t nus)
{
 
 uint16_t  differ = 0xffff-nus-5;
 //设置定时器2的技术初始值
  __HAL_TIM_SetCounter(&htim2,differ);
  //开启定时器
  HAL_TIM_Base_Start(&htim2);
 
  while( differ<0xffff-5)
 {
  differ = __HAL_TIM_GetCounter(&htim2);
 };
 //关闭定时器
  HAL_TIM_Base_Stop(&htim2);
} 

/*
 普通定时器实现ms延时，可直接使用HAL库函数HAL_delay（）
*/
void delay_ms_tim(uint16_t nms)
{
 uint32_t i;
 for(i=0;i<nms;i++) user_delaynus_tim(1000);
}

unsigned int lowV(unsigned int com)
{
	static unsigned int iLastData;
	unsigned int iData;
	double dPower=0.5;
	iData = (com*dPower)+(1-dPower)*iLastData;//计算
	iLastData = iData;//jilu 
	return iData;//返回数据
}

int float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }



	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
 **/
	

		
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
   
//  HAL_UART_Receive_IT(&huart1,&aRxBuffer,1);
  HAL_ADCEx_Calibration_Start(&hadc1);    
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcbuf, 6);
  HAL_Delay(100);
	filter_type filter(filter_type effective_value, filter_type new_value, filter_type error_max);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t sum[6] = {0};
	uint32_t adc_offer[6] = {0};
		
	for(int i=0; i<6; i++){
		for(int j=0; j<10000; j++){
			sum[i] += adcbuf[i];
			user_delaynus_tim(10);
		}		
  adc_offer[i] = sum[i] /1000;		
	}
	
	
	for(int i=0; i<6; i++){
		  
		adcreal[i] = (float)adc_offer[i]*3.3f/4096;
		R[i] = 100*adcreal[i]/(3.3-adcreal[i]);
		
		for(int j=0;j<sizeof(PT100TAB)/sizeof(float)-1;j++)
		{
			if(R[i]>=PT100TAB[j] && R[i]<PT100TAB[j+1]){
			temper[i]=(float)j-50.0+(R[i]-PT100TAB[j])/(PT100TAB[j+1]-PT100TAB[j]);
			break;
			} 
		}
		
		if(temper[i]>T_MAX)temper[i] = T_MAX;
		if(R[i]<PT100TAB[0])temper[i] = T_MIN;
		last_temprt[i] = temper[i];
	}
			
	
  while (1)
  {
	  for(int i=0; i<6; i++){
		  
		adcreal[i] = (float)adcbuf[i]*3.3f/4096;
		R[i] = 100*adcreal[i]/(3.3-adcreal[i]);
		
		for(int j=0;j<sizeof(PT100TAB)/sizeof(float)-1;j++)
		{
			if(R[i]>=PT100TAB[j] && R[i]<PT100TAB[j+1]){
			temper[i]=(float)j-50.0+(R[i]-PT100TAB[j])/(PT100TAB[j+1]-PT100TAB[j]);
			break;
			} 
		}
		
		if(temper[i]>T_MAX)temper[i] = T_MAX;
		if(R[i]<PT100TAB[0])temper[i] = T_MIN;
		
		if(fabs(temper[i] - last_temprt[i]) > effective_value){
			temper[i] = last_temprt[i];
		}
		
		temper[i] = 0.05f*temper[i] + 0.95f*last_temprt[i];
		last_temprt[i] = temper[i];

	}
	  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
