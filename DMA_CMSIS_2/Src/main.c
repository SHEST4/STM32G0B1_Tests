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
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nmea-parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxBuff[RX_BUFFER_SIZE] = {0};
uint16_t Index = 0;
NMEA GPSData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UART1_Init(void);
void DMA_Init(void);
void DMA_Config(uint32_t scrAdd, uint32_t destAdd, uint16_t datasize);
void DMA1_Channel1_IRQHandler(void);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(PWR_GPS_EN_GPIO_Port, PWR_GPS_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VRTC_GPIO_Port, VRTC_Pin, GPIO_PIN_SET);
	
	NMEA_Init(&GPSData);	
	
	UART1_Init();
	
	DMA_Init();
	
	DMA_Config((uint32_t) &USART1->RDR, (uint32_t) RxBuff, RX_BUFFER_SIZE);
	
	
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if ((USART1->ISR & USART_ISR_IDLE) != 0) 
		{
			NMEA_Parse(RxBuff, &GPSData);
		}
		
		
		HAL_GPIO_TogglePin(GPIOB, LED_G_Pin);
		HAL_Delay(100);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
* CHANGE LL TO HAL
  * @brief System Clock Configuration 
  * @retval None
  */
void SystemClock_Config(void)
{
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI48_Enable();

   /* Wait till HSI48 is ready */
  while(LL_RCC_HSI48_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_CRS);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_CRS);
  LL_CRS_SetSyncDivider(LL_CRS_SYNC_DIV_1);
  LL_CRS_SetSyncPolarity(LL_CRS_SYNC_POLARITY_RISING);
  LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
  LL_CRS_SetReloadCounter(__LL_CRS_CALC_CALCULATE_RELOADVALUE(48000000,1000));
  LL_CRS_SetFreqErrorLimit(34);
  LL_CRS_SetHSI48SmoothTrimming(32);
}

/* USER CODE BEGIN 4 */

void UART1_Init(void)
{
	SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);
	
	SET_BIT(USART1->BRR, 0x008B);
	CLEAR_REG(USART1->CR1);
	CLEAR_REG(USART1->CR2);
	CLEAR_REG(USART1->CR3);
	SET_BIT(USART1->CR1, USART_CR1_RE);
	SET_BIT(USART1->CR3, USART_CR3_ONEBIT);
	SET_BIT(USART1->CR3, USART_CR3_DMAR);
	SET_BIT(USART1->CR1, USART_CR1_UE);
}


void DMA_Init(void)
{
	
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
	
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE | DMA_CCR_TEIE);
	
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR);
	
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC);
	
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0);
	
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
}

void DMA_Config(uint32_t scr, uint32_t dest, uint16_t datasize)
{
	WRITE_REG(DMA1_Channel1->CNDTR, datasize);
	
	WRITE_REG(DMA1_Channel1->CPAR, scr);
	
	WRITE_REG(DMA1_Channel1->CMAR, dest);
	
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);
	SET_BIT(USART1->CR3, USART_CR3_DMAR);
		
}


void DMA1_Channel1_IRQHandler(void)
{
	
		if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF1) == (DMA_ISR_TCIF1)) 
		{ 
				SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF1);
				DMA1->ISR &= ~DMA_CCR_EN;
				DMA_Config((uint32_t)&USART1->RDR, (uint32_t)RxBuff, RX_BUFFER_SIZE);
    }
		else if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF4) == (DMA_ISR_TEIF4))
		{
			CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);
		}
		SET_BIT(USART1->CR3, USART_CR3_DMAR);
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
