/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CANOPEN_NODE_ID 0x01
#define STEPS_PER_REV 12800  // 200 steps/rev * 64 microsteps
#define TARGET_REV 1
#define TARGET_POSITION (TARGET_REV * STEPS_PER_REV)  // 128000 steps

#define OD_CONTROLWORD 0x6040
#define OD_STATUSWORD 0x6041
#define OD_MODE_OPERATION 0x6060
#define OD_TARGET_POSITION 0x607A
#define OD_PROFILE_VELOCITY 0x6081
#define OD_PROFILE_ACCEL 0x6083
#define OD_PROFILE_DECEL 0x6084
#define OD_LIMIT_SWITCHES 0x2005
#define OD_ACTUAL_POSITION 0x6064
#define OD_HOMING_METHOD 0x6098
#define OD_ERROR_REGISTER 0x1001  // Standard CANopen error register

#define COB_ID_NMT 0x000
#define SDO_CLIENT_TX (0x600 + CANOPEN_NODE_ID)
#define SDO_SERVER_RX (0x580 + CANOPEN_NODE_ID)


// 6.2 How to move a Motor in pp Mode



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t rx_buffer[32]; // Buffer to store incoming data
uint8_t rx_index = 0;
int user_input = 0;    // Variable to store the parsed integer
volatile uint8_t rx_complete = 0;


uint32_t duty_cycle = 0; // Example: 50% duty cycle
// Since ARR = 99, the CCR value ranges from 0 to 99 to represent 0% to 100% duty cycle.
// Since ARR = 49, the CCR value ranges from 0 to 49 to represent 0% to 100% duty cycle.



uint32_t SPEED = 50000; //  The motor’s datasheet specifies a maximum velocity (e.g., 50000 inc/s for 200 steps/rev with 64 microsteps).

//#define REVOLUTIONS 20 // Change this for fixed revolutions (e.g., 3, 7)
//#define TARGET_POSITION (REVOLUTIONS * STEPS_PER_REV) // Override motor_control.h
// redefine TARGET_POSITION in main.c to override the one in motor_control.h.
// This avoids the initializer element is not constant error since macros are evaluated at compile time.





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef CANopen_SendFrame(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);
void CANopen_NMT_Start(uint8_t nodeId);
HAL_StatusTypeDef CANopen_SDO_Write8(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, int8_t value);
HAL_StatusTypeDef CANopen_SDO_Write16(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, uint16_t value);
HAL_StatusTypeDef CANopen_SDO_Write32(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, uint32_t value);
uint16_t CANopen_SDO_Read16(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx);
uint32_t CANopen_SDO_Read32(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx);
void Delay_ms(uint32_t ms);
void Debug_Print(const char *msg);




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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */


  // Start UART receive interrupt
  HAL_UART_Receive_IT(&huart3, rx_buffer, 1);
  // Prompt user for input
  char prompt[] = "Enter an integer: \r\n";


  HAL_UART_Transmit(&huart3, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);





  // motor control




/*
  // Full reset sequence with retries
  Debug_Print("Performing Full Motor Reset\r\n");
  uint8_t nmt_data[2];
  for (int i = 0; i < 3; i++) { // Retry reset 3 times
    nmt_data[0] = 0x82; nmt_data[1] = CANOPEN_NODE_ID;  // NMT Reset All
    if (CANopen_SendFrame(&hcan1, COB_ID_NMT, nmt_data, 2) != HAL_OK) {
      Debug_Print("NMT Reset Send Failed\r\n");
    }
    Delay_ms(1000);
    nmt_data[0] = 0x80; nmt_data[1] = CANOPEN_NODE_ID;  // NMT Pre-Operational
    if (CANopen_SendFrame(&hcan1, COB_ID_NMT, nmt_data, 2) != HAL_OK) {
      Debug_Print("NMT Pre-Op Send Failed\r\n");
    }
    Delay_ms(1000);
  }
  if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0080) != HAL_OK) {
    Debug_Print("SDO Fault Reset Failed\r\n");
  }
  Delay_ms(1000);
*/





  // Start CAN with error reporting
  Debug_Print("Starting CAN\r\n");
  HAL_StatusTypeDef can_status = HAL_CAN_Start(&hcan1);
  if (can_status != HAL_OK) {
    char buf[48];
    snprintf(buf, sizeof(buf), "CAN Start Failed: Error Code 0x%08lX\r\n", HAL_CAN_GetError(&hcan1));
    Debug_Print(buf);
    Error_Handler();
  }
  Delay_ms(200);


  Debug_Print("Starting CANopen Motor Control\r\n");

  // CAN filter: Accept all IDs
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    Debug_Print("CAN Filter Config Failed\r\n");
    Error_Handler();
  }
  Delay_ms(200);


      // Start NMT
        Debug_Print("Sending NMT Start\r\n");
        CANopen_NMT_Start(CANOPEN_NODE_ID);
        Delay_ms(200);

        // Check statusword
        uint16_t status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
        char buf[48];
        snprintf(buf, sizeof(buf), "Initial Statusword: 0x%04X\r\n", status);
        Debug_Print(buf);
        if (status == 0xFFFF) {
          Debug_Print("SDO Read Failed: No response\r\n");
          Error_Handler();
        }
        Delay_ms(200);



        // Check and clear fault
/*          if (status & 0x08) {
            uint32_t error_code = CANopen_SDO_Read32(&hcan1, OD_ERROR_CODE, 0x00);
            snprintf(buf, sizeof(buf), "Fault Code: 0x%08lX\r\n", error_code);
            Debug_Print(buf);
            CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0080);  // Reset Fault
            Delay_ms(200);
            status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
            snprintf(buf, sizeof(buf), "Status after reset: 0x%04X\r\n", status);
            Debug_Print(buf);
            if (status & 0x08) {
              Debug_Print("Fault persists\r\n");
              Error_Handler();
            }
          }
*/



          // Reset state machine
            if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0000) != HAL_OK) {  // Disable Voltage
              Debug_Print("Disable Voltage Failed\r\n");
              Error_Handler();
            }
            Delay_ms(200);
            status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
            snprintf(buf, sizeof(buf), "Status after Disable: 0x%04X\r\n", status);
            Debug_Print(buf);



/*
            // Check current position
              int32_t current_pos = (int32_t)CANopen_SDO_Read32(&hcan1, OD_ACTUAL_POSITION, 0x00);
              snprintf(buf, sizeof(buf), "Current Position: %ld steps\r\n", current_pos);
              Debug_Print(buf);
              if (current_pos == 0xFFFFFFFF) {
                Debug_Print("Read Position Failed\r\n");
                Error_Handler();
              }
*/



              // Homing: Set method 35 and execute
/*
            Debug_Print("Starting Homing\r\n");
            if (CANopen_SDO_Write8(&hcan1, OD_HOMING_METHOD, 0x00, 35) != HAL_OK) {  // Method 35: Current position = 0
              Debug_Print("Set Homing Method Failed\r\n");
              Error_Handler();
            }

                Delay_ms(50);
                if (CANopen_SDO_Write8(&hcan1, OD_MODE_OPERATION, 0x00, 0x06) != HAL_OK) {  // Homing mode
                  Debug_Print("Set Homing Mode Failed\r\n");
                  Error_Handler();
                }

                Delay_ms(100);
                if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {  // Switch On
                  Debug_Print("Homing Switch On Failed\r\n");
                  Error_Handler();
                }
                Delay_ms(100);
                status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                snprintf(buf, sizeof(buf), "Homing Status after Switch On: 0x%04X\r\n", status);
                Debug_Print(buf);

                if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0007) != HAL_OK) {  // Switched On
                  Debug_Print("Homing Switched On Failed\r\n");
                  Error_Handler();
                }
                Delay_ms(100);
                status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                snprintf(buf, sizeof(buf), "Homing Status after Switched On: 0x%04X\r\n", status);
                Debug_Print(buf);

                if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {  // Enable Operation
                  Debug_Print("Enable Homing Failed\r\n");
                  Error_Handler();
                }
                Delay_ms(100);
                status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                snprintf(buf, sizeof(buf), "Homing Status after Enable Op: 0x%04X\r\n", status);
                Debug_Print(buf);


                if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {  //start homing process
                Debug_Print("Start Homing Failed\r\n");
                Error_Handler();
                }
                Delay_ms(500);


                if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {  //finish homing
                  Debug_Print("Start Homing Failed\r\n");
                  Error_Handler();
                }
                Delay_ms(100);
*/




                      // Configure velocity and acceleration
                        if (CANopen_SDO_Write32(&hcan1, OD_PROFILE_VELOCITY, 0x00, 5000) != HAL_OK) {  // 5000 inc/s
                          Debug_Print("Set Velocity Failed\r\n");
                          Error_Handler();
                        }
                        if (CANopen_SDO_Write32(&hcan1, OD_PROFILE_ACCEL, 0x00, 500) != HAL_OK) {
                          Debug_Print("Set Acceleration Failed\r\n");
                          Error_Handler();
                        }
                        if (CANopen_SDO_Write32(&hcan1, OD_PROFILE_DECEL, 0x00, 500) != HAL_OK) {
                          Debug_Print("Set Deceleration Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(50);




                        status = CANopen_SDO_Read16(&hcan1, 0x2707, 0x00);
                        snprintf(buf, sizeof(buf), "CAN bit rate: 0x%04X\r\n", status);
                        Debug_Print(buf);
                        Delay_ms(50);
                        status = CANopen_SDO_Read16(&hcan1, 0x2708, 0x00);
                        snprintf(buf, sizeof(buf), "node ID: 0x%04X\r\n", status);
                        Debug_Print(buf);
                        Delay_ms(50);






                         // CiA 402 state machine





                        // homing start
                        Debug_Print("Starting CiA 402 Transition\r\n");
                        if (CANopen_SDO_Write8(&hcan1, OD_MODE_OPERATION, 0x00, 0x06) != HAL_OK) {
                        Debug_Print("Switch On Failed\r\n");
                        Error_Handler();
                        }
                        Delay_ms(100);

                        if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {
                          Debug_Print("Switch On Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(500);

                        if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0007) != HAL_OK) {
                          Debug_Print("Switch On Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(500);

                        if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {
                          Debug_Print("Switch On Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(500);

                        // homing to zero test
                        Debug_Print("Starting Homing\r\n");
                        if (CANopen_SDO_Write8(&hcan1, OD_HOMING_METHOD, 0x00, 0x23) != HAL_OK) {  // Method 35: Current position = 0
                          Debug_Print("Set Homing Method Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(100);

                        if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {
                          Debug_Print("Switch On Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(500);

                        if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {
                          Debug_Print("Switch On Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(500);

                        // Check homing completion by position (method 35 sets position to 0)
                        int32_t current_pos = (int32_t)CANopen_SDO_Read32(&hcan1, OD_ACTUAL_POSITION, 0x00);
                          snprintf(buf, sizeof(buf), "Position after homing: %ld steps\r\n", current_pos);
                          Debug_Print(buf);
                          if (current_pos == 0xFFFFFFFF) {
                            Debug_Print("Read Position Failed\r\n");
                            Error_Handler();
                          }
                          if (current_pos != 0) {
                            Debug_Print("Homing Failed: Position not zero\r\n");
                            // Proceed anyway, as motion works
                              } else {
                                Debug_Print("Homing Successful\r\n");
                              }
                              status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                              snprintf(buf, sizeof(buf), "Final Homing Statusword: 0x%04X\r\n", status);
                              Debug_Print(buf);



                        // homing end



                        // Disable limit switches
                        if (CANopen_SDO_Write32(&hcan1, OD_LIMIT_SWITCHES, 0x00, 0x00000003) != HAL_OK) {
                          Debug_Print("Set Limit Switches Failed\r\n");
                          Error_Handler();
                        }
                        Delay_ms(100);

                           Debug_Print("Starting CiA 402 Transition\r\n");
                           if (CANopen_SDO_Write8(&hcan1, OD_MODE_OPERATION, 0x00, 0x01) != HAL_OK) {
                           Debug_Print("Switch On Failed\r\n");
                           Error_Handler();
                           }
                           Delay_ms(100);

                           if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {
                             Debug_Print("Switch On Failed\r\n");
                             Error_Handler();
                           }
                           Delay_ms(500);
                           status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                           snprintf(buf, sizeof(buf), "Status after Switch On: 0x%04X\r\n", status);
                           Debug_Print(buf);

                           if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0007) != HAL_OK) {
                             Debug_Print("Switched On Failed\r\n");
                             Error_Handler();
                           }
                           Delay_ms(500);
                           status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                           snprintf(buf, sizeof(buf), "Status after Switched On: 0x%04X\r\n", status);
                           Debug_Print(buf);

                           if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {
                             Debug_Print("Enable Operation Failed\r\n");
                             Error_Handler();
                           }
                           Delay_ms(500);

                           // Set target position
                           if (CANopen_SDO_Write32(&hcan1, OD_TARGET_POSITION, 0x00, TARGET_POSITION) != HAL_OK) {
                             Debug_Print("Set Position Failed\r\n");
                             Error_Handler();
                           }


                           Delay_ms(100);
                           status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                           snprintf(buf, sizeof(buf), "Status after Enable Op: 0x%04X\r\n", status);
                           Debug_Print(buf);

                           if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {
                             Debug_Print("Mark the new target position as active Failed\r\n");
                             Error_Handler();
                           }
                           Delay_ms(100);
                           Debug_Print("Mark the new target position as active Sent\r\n");


                           if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {
                              Debug_Print("Reset the activationn Failed\r\n");
                              Error_Handler();
                           }
                           Delay_ms(100);


                           // Poll statusword for target reached
                             do {
                               status = CANopen_SDO_Read16(&hcan1, OD_STATUSWORD, 0x00);
                               snprintf(buf, sizeof(buf), "Statusword: 0x%04X\r\n", status);
                               Debug_Print(buf);
                               if (status == 0xFFFF) {
                                 Debug_Print("SDO Read Failed\r\n");
                                 Error_Handler();
                               }
                               current_pos = (int32_t)CANopen_SDO_Read32(&hcan1, OD_ACTUAL_POSITION, 0x00);
                               snprintf(buf, sizeof(buf), "Current Position: %ld steps\r\n", current_pos);
                               Debug_Print(buf);
                               Delay_ms(100);
                             } while (!(status & 0x0400));  // Bit 10: Target reached

                             // Stop
                             if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0002) != HAL_OK) {  // Quick Stop
                               Debug_Print("Quick Stop Failed\r\n");
                             }
                             Delay_ms(200);
                             if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0000) != HAL_OK) {  // Disable Voltage
                               Debug_Print("Disable Voltage Failed\r\n");
                             }
                             Debug_Print("Motion Complete\r\n");



                             // Reset for next motion
                             if (CANopen_SDO_Write16(&hcan1, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {
                               Debug_Print("reset for next motion Failed\r\n");
                             }
                             Debug_Print("Motion Complete\r\n");




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	      if (rx_complete)
	          {
	              // Process received integer
	              char response[64];
	              snprintf(response, sizeof(response), "Received integer: %d\r\n", user_input);
	              HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
	              // Prompt again
	              HAL_UART_Transmit(&huart3, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);
	              rx_complete = 0;
	              duty_cycle = user_input;

	          }

*/

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* Function definitions (unchanged) */
HAL_StatusTypeDef CANopen_SendFrame(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = id;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = len;

  HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
  if (ret != HAL_OK) {
    char buf[48];
    snprintf(buf, sizeof(buf), "CAN Tx Error: %lu\r\n", HAL_CAN_GetError(hcan));
    Debug_Print(buf);
    return ret;
  }

  uint32_t tickstart = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox) && (HAL_GetTick() - tickstart < 100)) {}
  return HAL_OK;
}

void CANopen_NMT_Start(uint8_t nodeId) {
  uint8_t nmt[2] = {0x01, nodeId};
  CANopen_SendFrame(&hcan1, COB_ID_NMT, nmt, 2);
}

HAL_StatusTypeDef CANopen_SDO_Write8(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, int8_t value) {
  uint8_t sdo[8] = {0x2F, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, (uint8_t)value, 0x00, 0x00, 0x00};
  return CANopen_SendFrame(hcan, SDO_CLIENT_TX, sdo, 8);
}

HAL_StatusTypeDef CANopen_SDO_Write16(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, uint16_t value) {
  uint8_t sdo[8] = {0x2B, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx,
                    (uint8_t)(value & 0xFF), (uint8_t)(value >> 8), 0x00, 0x00};
  return CANopen_SendFrame(hcan, SDO_CLIENT_TX, sdo, 8);
}

HAL_StatusTypeDef CANopen_SDO_Write32(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx, uint32_t value) {
  uint8_t sdo[8] = {0x23, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx,
                    (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF),
                    (uint8_t)((value >> 16) & 0xFF), (uint8_t)(value >> 24)};
  return CANopen_SendFrame(hcan, SDO_CLIENT_TX, sdo, 8);
}

uint16_t CANopen_SDO_Read16(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, 0x00, 0x00, 0x00, 0x00};
  if (CANopen_SendFrame(hcan, SDO_CLIENT_TX, req, 8) != HAL_OK) {
    Debug_Print("SDO Read Send Failed\r\n");
    return 0xFFFF;
  }

  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  uint32_t tickstart = HAL_GetTick();
  while ((HAL_GetTick() - tickstart < 500)) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
      if (RxHeader.StdId == SDO_SERVER_RX && RxData[0] == 0x4B) {
        return (RxData[4] | (RxData[5] << 8));
      }
    }
  }
  Debug_Print("SDO Read Timeout\r\n");
  return 0xFFFF;
}

uint32_t CANopen_SDO_Read32(CAN_HandleTypeDef *hcan, uint16_t index, uint8_t subidx) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, 0x00, 0x00, 0x00, 0x00};
  if (CANopen_SendFrame(hcan, SDO_CLIENT_TX, req, 8) != HAL_OK) {
    Debug_Print("SDO Read32 Send Failed\r\n");
    return 0xFFFFFFFF;
  }

  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  uint32_t tickstart = HAL_GetTick();
  while ((HAL_GetTick() - tickstart < 500)) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
      if (RxHeader.StdId == SDO_SERVER_RX && RxData[0] == 0x43) {
        return (RxData[4] | (RxData[5] << 8) | (RxData[6] << 16) | (RxData[7] << 24));
      }
    }
  }
  Debug_Print("SDO Read32 Timeout\r\n");
  return 0xFFFFFFFF;
}

void Delay_ms(uint32_t ms) {
  HAL_Delay(ms);
}

void Debug_Print(const char *msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case GPIO_PIN_13: // u cant make EXTI same pin on  other port so no need for port specification
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14, GPIO_PIN_SET); // on
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14);
		break;
	default:
		break;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_buffer[rx_index] == '\r' || rx_buffer[rx_index] == '\n')
        {
            rx_buffer[rx_index] = '\0'; // Null-terminate
            // Parse the received string to an integer
            user_input = atoi((char*)rx_buffer);
            rx_complete = 1;
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }
        else
        {
            rx_index++;
            if (rx_index >= sizeof(rx_buffer)) rx_index = 0; // Prevent overflow
        }
        // Restart interrupt for next byte
        HAL_UART_Receive_IT(&huart3, rx_buffer + rx_index, 1);
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
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14);
	  HAL_Delay(500);
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
