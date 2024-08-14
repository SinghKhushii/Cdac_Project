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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "SEGGER_SYSVIEW.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "Disc_F407.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_READ_REG(REG);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t CANData1 = 0;  // Global variable to store first CAN data
uint8_t CANData2 = 0;  // Global variable to store second CAN data
uint8_t CANData3 = 0;  // Global variable to store third CAN data
uint8_t CANData4 = 0;  // Global variable to store fourth CAN data
uint8_t CANData5 = 0;  // Global variable to store fifth CAN data
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8]; // Actual Payload
uint32_t TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void LED_Display(uint8_t LedStatus);
void CANReceiveTask(void *pvParameters);
void I2C_OLED_Display(const char* message);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define STACK_SIZE 128
TaskHandle_t CANReceiveTaskHandle;
void WasteFullLED( void* NotUsed);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CAN_filterConfig();

  // Start the CAN peripheral
  CAN1->MCR &= ~CAN_MCR_INRQ; // Leave initialization mode
  while (CAN1->MSR & CAN_MSR_INAK); // Wait for initialization mode to exit
  SEGGER_SYSVIEW_Conf();
    	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//ensure proper priority grouping for freeRTOS


   // Enable FIFO 0 message pending interrupt
  CAN1->IER |= CAN_IER_FMPIE0;
   assert_param(xTaskCreate(CANReceiveTask, "CANReceive", STACK_SIZE, NULL,tskIDLE_PRIORITY +3, &CANReceiveTaskHandle)== pdPASS);
   assert_param(xTaskCreate(WasteFullLED, "LED_blink_low_priority", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
  vTaskStartScheduler();

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

void WasteFullLED(void *NotUsed)
{
	while(1)
	{
		BlueLed.On();
		vTaskDelay(100/portTICK_PERIOD_MS);
		BlueLed.Off();
		vTaskDelay(100/portTICK_PERIOD_MS);

	}
}


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

/*void CANReceiveTask(void *pvParameters)
{
    while (1)
    {
        // Wait for notification from ISR
    	BaseType_t result = xTaskGenericNotifyWait(
    	    0,               // uxIndexToWaitOn: index 0, or change according to your needs
    	    pdTRUE,          // ulBitsToClearOnEntry: usually pdTRUE
    	    0,               // ulBitsToClearOnExit: clear no bits on exit, or change according to your needs
    	    NULL,            // pulNotificationValue: NULL if you don't need the notification value
    	    portMAX_DELAY    // xTicksToWait: maximum delay to wait for the notification
    	);

        // Process received CAN data
    if ((RxHeader.StdId == 0x456) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 5))
        {
            // Store each received byte into the corresponding global variable
            CANData1 = RxData[0];
            CANData2 = RxData[1];
            CANData3 = RxData[2];
            CANData4 = RxData[3];
            CANData5 = RxData[4];

            // For example, display CANData1 on LED and display "automation" on OLED
            LED_Display(CANData1);
          //  ubKeyNumber = CANData1;

            // Send "automation activated" to the OLED display
            I2C_OLED_Display("automation activated");
        }
    }
}*/

/*void CANReceiveTask(void *pvParameters)
{
   uint32_t ulNotificationValue;
    for(;;)
    {
    	// Wait for the notification
    	if (xTaskGenericNotifyWait(
    	        		0,
    	                0x00,                   // Clear bits on entry
    	                0xFFFFFFFF,             // Clear bits on exit
    	                &ulNotificationValue,   // Store notified value
    	                portMAX_DELAY           // Wait indefinitely
    	            ) == pdTRUE){
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
            // Process received CAN data
           // if ((RxHeader.StdId == 0x456) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 5))
            //{
                // Store each received byte into the corresponding global variable
                CANData1 = RxData[0];
                CANData2 = RxData[1];
                CANData3 = RxData[2];
                CANData4 = RxData[3];
                CANData5 = RxData[4];

                // For example, display CANData1 on LED and display "automation" on OLED
               // LED_Display(CANData1);

                // Send "automation activated" to the OLED display
                I2C_OLED_Display("automation activated");
            //}
        }
        else
        {
        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
             // Handle timeout or error (if needed)
            // This is optional depending on your application needs
        }
    }
}*/
void CANReceiveTask(void *pvParameters)
{
   // uint32_t ulNotificationValue;
    while(1)
    {
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // Turn on the LED
    	    	            HAL_Delay(100); // Small delay for visibility
    	    	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);  // Turn off the LED
    	// Wait for any notification
    	    	            xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    	    	            //ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    	if ((RxHeader.StdId == 0x456) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 5))
    	            {
    	                // Store each received byte into the corresponding global variable
    	                CANData1 = RxData[0];
    	                CANData2 = RxData[1];
    	                CANData3 = RxData[2];
    	                CANData4 = RxData[3];
    	                CANData5 = RxData[4];

    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);  // Turn on the LED
    	            HAL_Delay(100); // Small delay for visibility
    	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);  // Turn off the LED
    	            I2C_OLED_Display("automation activated");
    	            vTaskDelay(100/portTICK_PERIOD_MS);
    	            }
//        else
//               {
//               	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
//                }
}
}


void I2C_OLED_Display(const char* message)
{
    // Clear the screen before displaying the message
    ssd1306_Fill(Black);

    // Set the cursor to the top-left corner (or adjust as needed)
    ssd1306_SetCursor(10, 20);

    // Display the provided message on the OLED
    ssd1306_WriteString(message, Font_11x18, White);

    // Update the screen to show the message
    ssd1306_UpdateScreen();
}
void CAN1_RX0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    SEGGER_SYSVIEW_RecordEnterISR();

    // Access the CAN1 interrupt enable register directly
    uint32_t interrupt = CAN1->IER;

    if ((interrupt & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
    {
        // Check if there are any messages pending in FIFO 0
        if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
        {
            // Get the Id
            uint8_t IDE = (uint8_t)(0x04 & CAN1->sFIFOMailBox[0].RIR);

            if (IDE == 0x00)
            {
                // Standard ID
                uint32_t StdId = (uint32_t)(0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21));
            }
            else
            {
                // Extended ID
                uint32_t ExtId = (uint32_t)(0x1FFFFFFF & (CAN1->sFIFOMailBox[0].RIR >> 3));
            }
        }
    }

    if (CAN1->RF0R & CAN_RF0R_FMP0) // FIFO 0 message pending
    {

        RxHeader.StdId = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
        RxHeader.IDE = (CAN1->sFIFOMailBox[0].RIR >> 2) & 0x03;
        RxHeader.DLC = CAN1->sFIFOMailBox[0].RDTR & 0x0F;
        RxData[4] = CAN1->sFIFOMailBox[0].RDHR & 0xFF;
        RxData[3] = (CAN1->sFIFOMailBox[0].RDLR) & 0xFF;
        RxData[2] = (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        RxData[1] = (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        RxData[0] = (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

        // Release the FIFO
        CAN1->RF0R |= CAN_RF0R_RFOM0;
		if ((RxHeader.StdId == 0xE) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 5))
			SEGGER_SYSVIEW_PrintfHost("In Interrupt\n");
			SEGGER_SYSVIEW_PrintfHost("Data: %c\n",RxData[1]);
			SEGGER_SYSVIEW_PrintfHost("Data: %c\n",RxData[2]);
        // Notify the CANReceiveTask
			vTaskNotifyGiveFromISR(CANReceiveTaskHandle,&xHigherPriorityTaskWoken);
			if (xHigherPriorityTaskWoken == pdTRUE)
			        {
			            SEGGER_SYSVIEW_PrintfHost("Task switch required.\n");
			        }
			        else
			        {
			            SEGGER_SYSVIEW_PrintfHost("No task switch required.\n");
			        }

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
			HAL_Delay(10); // Small delay for visibility
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);

        // Perform a context switch if needed
//
//    SEGGER_SYSVIEW_PrintfHost("Data Sent\n");
   SEGGER_SYSVIEW_RecordExitISR();
    // Perform a context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}

}


/*
void CAN1_RX0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (CAN1->RF0R & CAN_RF0R_FMP0) // FIFO 0 message pending
    {
        RxHeader.StdId = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
        RxHeader.IDE = (CAN1->sFIFOMailBox[0].RIR >> 2) & 0x03;
        RxHeader.DLC = CAN1->sFIFOMailBox[0].RDTR & 0x0F;
        RxData[4] = CAN1->sFIFOMailBox[0].RDHR & 0xFF;
        RxData[3] = (CAN1->sFIFOMailBox[0].RDLR) & 0xFF;
        RxData[2] = (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        RxData[1] = (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        RxData[0] = (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

        // Release the FIFO
        CAN1->RF0R |= CAN_RF0R_RFOM0;

        // Notify the CANReceiveTask
        xTaskGenericNotifyFromISR(
               CANReceiveTaskHandle,         // The handle of the task to notify
               0,                            // The index within the array to notify (usually 0)
               0,                            // The notification value (e.g., 0)
               eNoAction,                    // The action to perform on the notification value
               NULL,                         // Pointer to receive the previous notification value (NULL if not needed)
               &xHigherPriorityTaskWoken     // Pass the address of the flag that will determine if a context switch is needed
           );

        // Perform a context switch if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

*//*
void LED_Display(uint8_t LedStatus)
{
  // Turn OFF all LEDs
  GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
  switch (LedStatus)
  {
    case (1):
      // Turn ON LED1
      GPIOD->ODR |= GPIO_ODR_OD12;
      break;
    case (2):
      // Turn ON LED2
      GPIOD->ODR |= GPIO_ODR_OD13;
      break;
    case (3):
      // Turn ON LED3
      GPIOD->ODR |= GPIO_ODR_OD14;
      break;
    case (4):
   //Turn ON LED4
      GPIOD->ODR |= GPIO_ODR_OD15;
      break;
    default:
      break;
  }
}*/
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
