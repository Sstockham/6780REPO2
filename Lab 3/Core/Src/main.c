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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN ;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
__HAL_RCC_GPIOC_CLK_ENABLE(); 
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable timer 3
	
	TIM2->PSC = 7999; /* (1) */
  TIM2->ARR = 250; /* (2) */ //2. Configure the timer to trigger an update event (UEV) at 4 Hz.
	// register set -> register subset -> bitmasked
	
	TIM3->PSC = 31999; /* (1) */
  TIM3->ARR = 5; /* (2) */ //2. Configure the timer to trigger an update event (UEV) at 800 Hz. To do this we need 1.25 ms, so I think we need to scale division up 4x and have a arr of 5
	// register set -> register subset -> bitmasked
	TIM3-> CCMR1 |= ((0<<0)|(0<<1)|(1<<4)|(1<<5)|(1<<6)|(0<<12)|(1<<13)|(1<<14)|(0<<8)|(0<<9)|(1<<3)|(1<<11)); // Configuring Capture Compare Mode Register 1
	TIM3-> CCER |= ((1<<0)|(1<<4)); // Set output enable bits.
	
	TIM3-> CCR1 = 4;
	TIM3-> CCR2 = 1;
	
	
	//programming the bit pattern for AF1 into the bit region representing PB1 in the GPIOB alternate
//function registers we can select the capture/compare channel 4 of timer 3 to output on that pin. You will
//need to read the register map for the GPIO AFRH & AFRL registers,
	//GPIOC-> GPIO_AFRL = 0x2;
//GPIOC->AFR[1] &= ~(0x0F << ((9 - 8) * 4));
	
	//GPIOC-> AFR[0] |= (0x0000);
	
	GPIOC-> AFR[0] &= ~(0b1111);
  GPIOC-> AFR[1] &= ~(0b1111);
	//Where I am leaving off at is Trying to select AF0 (0000) on the AFSEL register as referenced in the GPIO AFRL and AFRH sections.
	TIM3 -> DIER |= (0x0001);//Use the DMA/Interrupt Enable Register (DIER) to enable the update interrupt
	TIM3 -> CR1 |=(0x0001); //4. Configure and enable/start the timer
	
	//NVIC_EnableIRQ(TIM2_IRQn); /* (1) */
  //NVIC_SetPriority(TIM2_IRQn,39); /* (2) *///5. Set up the timer’s interrupt handler, and enable in the NVIC.
	//for timer, prescale by 7999 and then arr by 10
	
	//LED Initialization
	
GPIOC->MODER|=(GPIO_MODER_MODER7_1);
GPIOC->MODER|=(GPIO_MODER_MODER6_1);

GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);

GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6);
GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7);

GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);	
GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);	

GPIOC->BSRR  = GPIO_BSRR_BS_6 ;
GPIOC->BSRR = GPIO_BSRR_BS_7 ;


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//5. Set up the timer’s interrupt handler, and enable in the NVIC.
void TIM2_IRQHandler(void){
	//always clear flags!!!
	//you always need to figure out how to clear flags when initializing a interupt handler
  //GPIOC->ODR ^= (GPIO_ODR_6);
  //GPIOC->ODR ^= (GPIO_ODR_7); 
 
  TIM2 -> SR &= ~(0x0001);
	
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
