//motor_control.c

// motor_control.c
#include "motor_control.h"
#include <string.h>
#include <stdio.h>

void Debug_Print(UART_HandleTypeDef *huart, const char *msg) {
  HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 100);
}

static HAL_StatusTypeDef CANopen_SendFrame(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint32_t id, uint8_t *data, uint8_t len) {
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
    Debug_Print(huart, buf); // Use huart parameter
    return ret;
  }

  uint32_t tickstart = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox) && (HAL_GetTick() - tickstart < 100)) {}
  return HAL_OK;
}

static HAL_StatusTypeDef CANopen_SDO_Write8(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t index, uint8_t subidx, int8_t value) {
  uint8_t sdo[8] = {0x2F, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, (uint8_t)value, 0x00, 0x00, 0x00};
  return CANopen_SendFrame(hcan, huart, SDO_CLIENT_TX, sdo, 8);
}

static HAL_StatusTypeDef CANopen_SDO_Write16(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t index, uint8_t subidx, uint16_t value) {
  uint8_t sdo[8] = {0x2B, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx,
                    (uint8_t)(value & 0xFF), (uint8_t)(value >> 8), 0x00, 0x00};
  return CANopen_SendFrame(hcan, huart, SDO_CLIENT_TX, sdo, 8);
}

static HAL_StatusTypeDef CANopen_SDO_Write32(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t index, uint8_t subidx, uint32_t value) {
  uint8_t sdo[8] = {0x23, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx,
                    (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF),
                    (uint8_t)((value >> 16) & 0xFF), (uint8_t)(value >> 24)};
  return CANopen_SendFrame(hcan, huart, SDO_CLIENT_TX, sdo, 8);
}

static uint16_t CANopen_SDO_Read16(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t index, uint8_t subidx) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, 0x00, 0x00, 0x00, 0x00};
  if (CANopen_SendFrame(hcan, huart, SDO_CLIENT_TX, req, 8) != HAL_OK) {
    Debug_Print(huart, "SDO Read Send Failed\r\n");
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
  Debug_Print(huart, "SDO Read Timeout\r\n"); // Use huart parameter
  return 0xFFFF;
}

static uint32_t CANopen_SDO_Read32(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t index, uint8_t subidx) {
  uint8_t req[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subidx, 0x00, 0x00, 0x00, 0x00};
  if (CANopen_SendFrame(hcan, huart, SDO_CLIENT_TX, req, 8) != HAL_OK) {
    Debug_Print(huart, "SDO Read32 Send Failed\r\n");
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
  Debug_Print(huart, "SDO Read32 Timeout\r\n"); // Use huart parameter
  return 0xFFFFFFFF;
}

HAL_StatusTypeDef Motor_Init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  // Start CAN
  if (HAL_CAN_Start(hcan) != HAL_OK) {
    Debug_Print(huart, "CAN Start Failed\r\n");
    return HAL_ERROR;
  }

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
  if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
    Debug_Print(huart, "CAN Filter Config Failed\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef Motor_NMT_Start(CAN_HandleTypeDef *hcan) {
  uint8_t nmt[2] = {0x01, CANOPEN_NODE_ID};
  return CANopen_SendFrame(hcan, NULL, COB_ID_NMT, nmt, 2); // No UART needed for NMT
}

HAL_StatusTypeDef Motor_CheckStatus(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint16_t *status) {
  *status = CANopen_SDO_Read16(hcan, huart, OD_STATUSWORD, 0x00);
  char buf[48];
  snprintf(buf, sizeof(buf), "Statusword: 0x%04X\r\n", *status);
  Debug_Print(huart, buf);
  if (*status == 0xFFFF) {
    Debug_Print(huart, "SDO Read Failed: No response\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Motor_ClearFault(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  uint16_t status;
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (status & 0x08) {
    uint32_t error_code = CANopen_SDO_Read32(hcan,huart, OD_ERROR_CODE, 0x00);
    char buf[48];
    snprintf(buf, sizeof(buf), "Fault Code: 0x%08lX\r\n", error_code);
    Debug_Print(huart, buf);
    if (CANopen_SDO_Write16(hcan, huart, OD_CONTROLWORD, 0x00, 0x0080) != HAL_OK) {
          Debug_Print(huart, "Reset Fault Failed\r\n");
          return HAL_ERROR;
        }
    HAL_Delay(200);
    if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
      return HAL_ERROR;
    }
    if (status & 0x08) {
      Debug_Print(huart, "Fault persists\r\n");
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef Motor_ResetState(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0000) != HAL_OK) {
    Debug_Print(huart, "Disable Voltage Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  uint16_t status;
  return Motor_CheckStatus(hcan, huart, &status);
}

HAL_StatusTypeDef Motor_ReadPosition(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, int32_t *position) {
  *position = (int32_t)CANopen_SDO_Read32(hcan,huart, OD_ACTUAL_POSITION, 0x00);
  char buf[48];
  snprintf(buf, sizeof(buf), "Current Position: %ld steps\r\n", *position);
  Debug_Print(huart, buf);
  if (*position == 0xFFFFFFFF) {
    Debug_Print(huart, "Read Position Failed\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Motor_Home(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  Debug_Print(huart, "Starting Homing\r\n");
  if (CANopen_SDO_Write8(hcan,huart, OD_HOMING_METHOD, 0x00, 35) != HAL_OK) {
    Debug_Print(huart, "Set Homing Method Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(50);
  if (CANopen_SDO_Write8(hcan,huart, OD_MODE_OPERATION, 0x00, 0x06) != HAL_OK) {
    Debug_Print(huart, "Set Homing Mode Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {
    Debug_Print(huart, "Homing Switch On Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  uint16_t status;
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0007) != HAL_OK) {
    Debug_Print(huart, "Homing Switched On Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {
    Debug_Print(huart, "Enable Homing Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {
    Debug_Print(huart, "Start Homing Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);

  // Check homing completion (method 35: position = 0)
  int32_t position;
  if (Motor_ReadPosition(hcan, huart, &position) != HAL_OK) {
    return HAL_ERROR;
  }
  if (position != 0) {
    Debug_Print(huart, "Homing Failed: Position not zero\r\n");
    return HAL_OK; // Proceed anyway
  }
  Debug_Print(huart, "Homing Successful\r\n");
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef Motor_SetProfilePositionMode(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  if (CANopen_SDO_Write32(hcan,huart, OD_LIMIT_SWITCHES, 0x00, 0x00000003) != HAL_OK) {
    Debug_Print(huart, "Set Limit Switches Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(50);
  if (CANopen_SDO_Write8(hcan,huart, OD_MODE_OPERATION, 0x00, 0x01) != HAL_OK) {
    Debug_Print(huart, "Set Mode Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  return HAL_OK;
}

HAL_StatusTypeDef Motor_ConfigureMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, uint32_t velocity) {
  if (CANopen_SDO_Write32(hcan,huart, OD_PROFILE_VELOCITY, 0x00, velocity) != HAL_OK) {
    Debug_Print(huart, "Set Velocity Failed\r\n");
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write32(hcan,huart, OD_PROFILE_ACCEL, 0x00, 500) != HAL_OK) {
    Debug_Print(huart, "Set Acceleration Failed\r\n");
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write32(hcan,huart, OD_PROFILE_DECEL, 0x00, 500) != HAL_OK) {
    Debug_Print(huart, "Set Deceleration Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(50);
  return HAL_OK;
}

HAL_StatusTypeDef Motor_SetTargetPosition(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart, int32_t position) {
  if (CANopen_SDO_Write32(hcan,huart, OD_TARGET_POSITION, 0x00, position) != HAL_OK) {
    Debug_Print(huart, "Set Position Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(50);
  return HAL_OK;
}

HAL_StatusTypeDef Motor_StartMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  Debug_Print(huart, "Starting CiA 402 Transition\r\n");
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0006) != HAL_OK) {
    Debug_Print(huart, "Switch On Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  uint16_t status;
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0007) != HAL_OK) {
    Debug_Print(huart, "Switched On Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x000F) != HAL_OK) {
    Debug_Print(huart, "Enable Operation Failed\r\n");
    return HAL_ERROR;
  }
  HAL_Delay(100);
  if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x001F) != HAL_OK) {
    Debug_Print(huart, "Start Motion Failed\r\n");
    return HAL_ERROR;
  }
  Debug_Print(huart, "Motion Command Sent\r\n");
  return HAL_OK;
}

HAL_StatusTypeDef Motor_PollMotion(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  uint16_t status;
  int32_t position;
  do {
    if (Motor_CheckStatus(hcan, huart, &status) != HAL_OK) {
      return HAL_ERROR;
    }
    if (Motor_ReadPosition(hcan, huart, &position) != HAL_OK) {
      return HAL_ERROR;
    }
    HAL_Delay(100);
  } while (!(status & 0x0400));  // Bit 10: Target reached
  return HAL_OK;
}

HAL_StatusTypeDef Motor_Stop(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart) {
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0002) != HAL_OK) {
    Debug_Print(huart, "Quick Stop Failed\r\n");
    return HAL_OK; // Non-critical
  }
  HAL_Delay(200);
  if (CANopen_SDO_Write16(hcan,huart, OD_CONTROLWORD, 0x00, 0x0000) != HAL_OK) {
    Debug_Print(huart, "Disable Voltage Failed\r\n");
    return HAL_OK; // Non-critical
  }
  Debug_Print(huart, "Motion Complete\r\n");
  return HAL_OK;
}
