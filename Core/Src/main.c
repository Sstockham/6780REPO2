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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN ;
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	
	RCC -> APB2RSTR |=(RCC_APB2RSTR_SYSCFGRST_Pos); //Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  /* USER CODE BEGIN SysInit */

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
GPIOC->MODER|=(GPIO_MODER_MODER6_0);
GPIOC->MODER|=(GPIO_MODER_MODER7_0);
GPIOC->MODER|=(GPIO_MODER_MODER8_0);
GPIOC->MODER|=(GPIO_MODER_MODER9_0);

GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6);
GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8);
GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);

GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6);
GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7);
GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8);
GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9);

GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);	
GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

GPIOC->BSRR  = GPIO_BSRR_BR_6 ;
GPIOC->BSRR  = GPIO_BSRR_BR_7 ;
GPIOC->BSRR = GPIO_BSRR_BR_8 ;
GPIOC->BSRR = GPIO_BSRR_BS_9 ;

//Configure the button pin (PA0) to input-mode at low-speed, with the internal pull-down
//resistor enabled.
GPIOA-> MODER &= ~(GPIO_MODER_MODER0);
//GPIOA -> MODER &= ~(GPIO_MODER_MODER0_1);
GPIOA -> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0);
GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR0;


// MY STUFF/ FIRST TRY
//Enable/unmask interrupt generation on EXTI input line 0 (EXTI0).
//EXTI-> IMR |= (0<<0);

//Configure the EXTI input line 0 to have a rising-edge trigger.
//EXTI-> RTSR |= (1<<0);

//SYSCFG->EXTICR[0] |= (0x000); //Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).


// APENDIX EXAMPLES
SYSCFG->EXTICR[1] &= (uint16_t)~SYSCFG_EXTICR1_EXTI0_PA; /* (2) */
EXTI->IMR = 0x0001; /* (3) */
EXTI->RTSR = 0x0001; /* (4) */
//EXTI->FTSR = 0x0001; /* (5) */



  NVIC_EnableIRQ(EXTI0_1_IRQn); /* (1) */
  NVIC_SetPriority(EXTI0_1_IRQn,0); // Enable the selected EXTI interrupt by passing its defined name to the NVIC_EnableIRQ(), Set the priority for the interrupt to 1 (high-priority) with the NVIC_SetPriority() function
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  while (1)
  {
    /* USER CODE END WHILE */
 HAL_Delay(200); // Delay 200ms
// Toggle the output state of both PC8 and PC9
//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
   // GPIOC -> ODR |= GPIO_ODR_6;
			//GPIOC->ODR ^= GPIO_ODR_6;
			//GPIOC ->BSRR = GPIO_BSRR_BS_6;
  // GPIOC->ODR ^= (GPIO_ODR_9);
	GPIOC->ODR ^= (GPIO_ODR_6);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
volatile uint32_t counter2;

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

/* USER CODE END 4 */
void EXTI0_1_IRQHandler(void){
	//always clear flags!!!
	//you always need to figure out how to clear flags when initializing a interupt handler
	counter2 ++;
	if (counter2 == 1500000){
		 
	counter2 = 0;
  GPIOC->ODR ^= (GPIO_ODR_8);
  GPIOC->ODR ^= (GPIO_ODR_9); 
 
  EXTI -> PR |= (0x0001);
	}
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
