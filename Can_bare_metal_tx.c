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
#define CAN_READ_REG(REG);
#define CAN_READ_REG(REG);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint8_t ubKeyNumber = 0x0;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8]; // Actual Payload
uint32_t TxMailbox;

uint8_t rx[2];   //store the data to be transmitted
uint8_t sx_x;
uint8_t sx_y;
uint8_t sx_z;
uint32_t potentiometerValue = 0;
uint32_t potValue = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void LED_Display(uint8_t LedStatus);
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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  CAN_filterConfig();

  // Start the CAN peripheral
  CAN1->MCR &= ~CAN_MCR_INRQ; // Leave initialization mode
  while (CAN1->MSR & CAN_MSR_INAK); // Wait for initialization mode to exit

  // Enable FIFO 0 message pending interrupt
  CAN1->IER |= CAN_IER_FMPIE0;

  // Set up the Tx header
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = 0x456;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 5;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET); // cs pulled low
	     	  rx[0]=0xA9; //0x29 | 0x80;
	     	  HAL_SPI_Transmit(&hspi1, rx, 1, 50);
	     	  HAL_SPI_Receive(&hspi1, &sx_x, 1, 50);
	     	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

	     	  //Read the OUT_Y (2Ah - 2Bh)
	     	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
	     	  rx[0]=0x2B | 0x80;
	     	  HAL_SPI_Transmit(&hspi1, rx, 1, 50);
	     	  HAL_SPI_Receive(&hspi1, &sx_y, 1, 50);
	     	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

	     	  //Read the OUT_Z (2Ch - 2Dh)
	     	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
	     	  rx[0]=0x2D | 0x80;
	     	  HAL_SPI_Transmit(&hspi1, rx, 1, 50);
	     	  HAL_SPI_Receive(&hspi1, &sx_z, 1, 50);
	     	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

	     	   HAL_ADC_Start(&hadc1);
	     	    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	     	    {
	     	      potValue = HAL_ADC_GetValue(&hadc1);
	     	      potValue = (int)potValue/10;
	     	     potentiometerValue = (int)potValue*20;
	     	    }
	     	    HAL_ADC_Stop(&hadc1);

	     	 if ((sx_y >= 10 && sx_y <= 30) && ( potentiometerValue >= 0 && potentiometerValue <= 1500 ))

	            {
	     		     	  	 /* Set the data to be transmitted */
	     		     	  	 TxData[0] = ubKeyNumber;
	     		     	  	 TxData[1] = 0x1F;
	     		     	  	 TxData[2] = sx_x;
	     		     	  	 TxData[3] = sx_y;
	     		     	  	 TxData[4] = sx_z;
	     		     	  	 /* Start the Transmission process */
	     		     	  	 if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) // Check if mailbox 0 is empty
	     		     	  	 {
	     		     	  	   CAN1->sTxMailBox[0].TIR = (TxHeader.StdId << 21) | (TxHeader.RTR << 1) | TxHeader.IDE;
	     		     	  	   CAN1->sTxMailBox[0].TDTR = TxHeader.DLC;
	     		     	  	   CAN1->sTxMailBox[0].TDHR = (TxData[4]);
	     		     	  	   CAN1->sTxMailBox[0].TDLR = (TxData[0] << 24) | (TxData[1] << 16) | (TxData[2] << 8) | TxData[3];
	     		     	  	   CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; // Request transmission
	     		     	  	 }
	     		  }


	     	 if(sx_x == 50 )
	     		 {

	     			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	     			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	     			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	     			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	     		 }
	     		else if (sx_x == 255)
	     			{
	     				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	     				//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	     				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	     				//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	     			}

	     		 else if(sx_y == 50)
	     			 {
	     				 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	     				 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	     			 }
	     			 else if (sx_y == 255)
	     			{

	     				 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	     				 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	     			 }

    if (GPIOA->IDR & GPIO_IDR_ID0)
    {
      for (volatile int i = 0; i < 100000; i++); // Debouncing delay
      if (GPIOA->IDR & GPIO_IDR_ID0)
      {
        if (ubKeyNumber == 0x4)
        {
          ubKeyNumber = 0x00;
        }
        else
        {
          LED_Display(++ubKeyNumber);
          /* Set the data to be transmitted */
          TxData[0] = ubKeyNumber;
          TxData[1] = 0x1F;
          TxData[2] = sx_x;
          TxData[3] = sx_y;
          TxData[4] = sx_z;
          /* Start the Transmission process */
          if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) // Check if mailbox 0 is empty
          {
            CAN1->sTxMailBox[0].TIR = (TxHeader.StdId << 21) | (TxHeader.RTR << 1) | TxHeader.IDE;
            CAN1->sTxMailBox[0].TDTR = TxHeader.DLC;
            CAN1->sTxMailBox[0].TDHR = (TxData[4]);
            CAN1->sTxMailBox[0].TDLR = (TxData[0] << 24) | (TxData[1] << 16) | (TxData[2] << 8) | TxData[3];
            CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; // Request transmission
          }
          for (volatile int i = 0; i < 200000; i++); // Delay just for better Tuning
        }
      }



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_filterConfig(void)
{
  // Enter CAN filter initialization mode
  CAN1->FMR |= CAN_FMR_FINIT;

  // Configure filter bank 0
  CAN1->FA1R &= ~(1 << 0); // Deactivate filter bank 0
  CAN1->FS1R |= (1 << 0); // Set filter scale to 32-bit
  CAN1->FM1R &= ~(1 << 0); // Set filter mode to mask
  CAN1->sFilterRegister[0].FR1 = 0x00000000; // Set filter ID
  CAN1->sFilterRegister[0].FR2 = 0x00000000; // Set filter mask
  CAN1->FFA1R &= ~(1 << 0); // Assign filter bank 0 to FIFO 0
  CAN1->FA1R |= (1 << 0); // Activate filter bank 0

  // Leave CAN filter initialization mode
  CAN1->FMR &= ~CAN_FMR_FINIT;
}

void CAN1_RX0_IRQHandler(CAN_HandleTypeDef *hcan1)
{
	/*USER CODE BEGIN CAN1_TX_IRQn 0 */
	uint32_t errorcode = HAL_CAN_ERROR_NONE;
	uint32_t interrupts = READ_REG(&hcan1->Instance->IER);
	uint32_t rf0rflags = READ_REG(&hcan1->Instance->RF0R);

	  /* Receive FIFO 0 overrun interrupt management *****************************/
	  if ((interrupts & CAN_IT_RX_FIFO0_OVERRUN) != 0U)
	  {
	    if ((rf0rflags & CAN_RF0R_FOVR0) != 0U)
	    {
	      /* Set CAN error code to Rx Fifo 0 overrun error */
	      errorcode |= HAL_CAN_ERROR_RX_FOV0;

	      /* Clear FIFO0 Overrun Flag */
	      __HAL_CAN_CLEAR_FLAG(hcan1, CAN_FLAG_FOV0);
	    }
	  }

	  /* Receive FIFO 0 full interrupt management ********************************/
	  //if ((interrupts & CAN_IT_RX_FIFO0_FULL) != 0U)
	  {
	    if ((rf0rflags & CAN_RF0R_FULL0) != 0U)
	    {
	      /* Clear FIFO 0 full Flag */
	      __HAL_CAN_CLEAR_FLAG(hcan1, CAN_FLAG_FF0);

	      /* Receive FIFO 0 full Callback */
	#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
	      /* Call registered callback*/
	      hcan1->RxFifo0FullCallback(hcan);
	#else
	      /* Call weak (surcharged) callback */
	      HAL_CAN_RxFifo0FullCallback(hcan1);
	#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
	    }
	  }

	/* Receive FIFO 0 message pending interrupt management *********************/
	  if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
	  {
	    /* Check if message is still pending */
	    if ((hcan1->Instance->RF0R & CAN_RF0R_FMP0) != 0U)
	    	/*Callback for receive function can*/

	    	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	    	 {
	    	   /* Reception Error*/
	    	   Error_Handler();
	    	 }
	    	 /* Display LEDx*/
	    	 if ((RxHeader.StdId == 0x456) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
	    	 {
	    	   LED_Display(RxData[0]);
	    	   ubKeyNumber = RxData[0];

	    	 }


}
}
void LED_Display(uint8_t LedStatus)
{
  /* Turn OFF all LEDs */
  GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
  switch (LedStatus)
  {
    case (1):
      /* Turn ON LED1 */
      GPIOD->ODR |= GPIO_ODR_OD12;
      break;
    case (2):
      /* Turn ON LED2 */
      GPIOD->ODR |= GPIO_ODR_OD13;
      break;
    case (3):
      /* Turn ON LED3 */
      GPIOD->ODR |= GPIO_ODR_OD14;
      break;
    case (4):
      /* Turn ON LED4 */
      GPIOD->ODR |= GPIO_ODR_OD15;
      break;
    default:
      break;
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
  /* User can add his own implementation to report the HAL error return state */
  GPIOD->ODR |= GPIO_ODR_OD14; // Turn on LED3 (PD14) in case of error
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
