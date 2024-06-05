/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
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
void DMA1_Channel1_Config(void);
void USART1_IRQHandler(void);
//void DMA1_Channel1_IRQHandler(void);
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
  //MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  //MX_USB_Device_Init();
  //MX_I2C2_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	LL_GPIO_SetOutputPin(LED_R_GPIO_Port, LED_R_Pin);
	LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
	LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
	
	LL_GPIO_SetOutputPin(PWR_GPS_EN_GPIO_Port, PWR_GPS_EN_Pin);
	LL_GPIO_SetOutputPin(VRTC_GPIO_Port, VRTC_Pin);
	
	DMA1_Channel1_Config();
	 
	NMEA_Init(&GPSData);	
	
	LL_USART_EnableIT_RXNE(USART1); 
	LL_USART_EnableIT_IDLE(USART1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		
		LL_mDelay(500);
		
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

void DMA1_Channel1_Config(void)
{
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&USART1->RDR);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)RxBuff);

	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, RX_BUFFER_SIZE);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

}


void USART1_IRQHandler(void)
{
		if(Index >= RX_BUFFER_SIZE)
		{
			memset(RxBuff, 0, RX_BUFFER_SIZE);
			Index = 0;
		}
    
		if(LL_USART_IsActiveFlag_RXNE(USART1))
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART1);
				
				RxBuff[Index++] = received_data;
    }
		
		 if(LL_USART_IsActiveFlag_IDLE(USART1))
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART1);
			
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
      
        NMEA_Parse(RxBuff, &GPSData);
				
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, RX_BUFFER_SIZE);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
				
				LL_USART_ClearFlag_IDLE(USART1);
				Index = 0;
    }

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
