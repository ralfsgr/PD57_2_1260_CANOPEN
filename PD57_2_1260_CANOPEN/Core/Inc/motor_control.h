/*
 * motor_control.h
 *
 *  Created on: Sep 25, 2025
 *      Author: rgraholskis2
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>

// Motor configuration
#define CANOPEN_NODE_ID 0x01  // Motor node ID
#define STEPS_PER_REV 12800     // Full steps; multiply by microsteps (e.g., x64 = 12800)
#define TARGET_REV 1
#define TARGET_POSITION (TARGET_REV * STEPS_PER_REV)  // 128000 steps

/* CiA 402 Object Dictionary */
// Object dictionary indices
#define OD_CONTROLWORD 0x6040
#define OD_STATUSWORD 0x6041
#define OD_MODE_OPERATION 0x6060
#define OD_TARGET_POSITION 0x607A

#define OD_LIMIT_SWITCHES 0x2005


#define OD_ERROR_CODE 0x603F
#define OD_PROFILE_VELOCITY 0x6081
#define OD_PROFILE_ACCEL 0x6083
#define OD_PROFILE_DECEL 0x6084

// 6.2 How to move a Motor in pp Mode - how to get a motor running in pp mode

#define OD_ACTUAL_POSITION 0x6064
#define OD_HOMING_METHOD 0x6098
#define OD_HOMING_SPEEDS 0x6099
#define OD_HOMING_ACCEL 0x609A


/* CANopen COB-IDs */
#define COB_ID_NMT 0x000
#define SDO_CLIENT_TX (0x600 + CANOPEN_NODE_ID)  // Master to slave
#define SDO_SERVER_RX (0x580 + CANOPEN_NODE_ID)  // Slave to master


// Function prototypes
void Debug_Print(UART_HandleTypeDef *huart, const char *msg); // Added

HAL_StatusTypeDef Motor_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_NMT_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef Motor_CheckStatus(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t *status);
HAL_StatusTypeDef Motor_ClearFault(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_ResetState(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_ReadPosition(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, int32_t *position);
HAL_StatusTypeDef Motor_Home(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_SetProfilePositionMode(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_ConfigureMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint32_t velocity);
HAL_StatusTypeDef Motor_SetTargetPosition(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, int32_t position);
HAL_StatusTypeDef Motor_StartMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_PollMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Stop(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);



#endif /* INC_MOTOR_CONTROL_H_ */
