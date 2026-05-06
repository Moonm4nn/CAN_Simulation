#include "can_protocol.h"

static HAL_StatusTypeDef CAN_Protocol_SendRaw(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    uint8_t *data,
    uint8_t dlc
)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}


HAL_StatusTypeDef CAN_Protocol_SendThrottleStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t throttle_percent,
    uint16_t rpm,
    uint16_t speed_kph
)
{
    uint8_t data[8] = {0};

    if (throttle_percent > 100)
    {
        throttle_percent = 100;
    }

    data[0] = throttle_percent;

    data[1] = (uint8_t)(rpm >> 8);
    data[2] = (uint8_t)(rpm & 0xFF);

    data[3] = (uint8_t)(speed_kph >> 8);
    data[4] = (uint8_t)(speed_kph & 0xFF);

    return CAN_Protocol_SendRaw(hcan, CAN_ID_THROTTLE_STATUS, data, 5);
}


HAL_StatusTypeDef CAN_Protocol_SendEngineStatus(
    CAN_HandleTypeDef *hcan,
    uint16_t rpm,
    uint16_t speed_kph,
    uint8_t engine_temp_c,
    uint8_t engine_fault
)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(rpm >> 8);
    data[1] = (uint8_t)(rpm & 0xFF);

    data[2] = (uint8_t)(speed_kph >> 8);
    data[3] = (uint8_t)(speed_kph & 0xFF);

    data[4] = engine_temp_c;
    data[5] = engine_fault ? 1 : 0;

    return CAN_Protocol_SendRaw(hcan, CAN_ID_ENGINE_STATUS, data, 6);
}


HAL_StatusTypeDef CAN_Protocol_SendBrakeStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t brake_active,
    uint8_t brake_pressure,
    uint8_t brake_light,
    uint8_t brake_fault
)
{
    uint8_t data[8] = {0};

    if (brake_pressure > 100)
    {
        brake_pressure = 100;
    }

    data[0] = brake_active ? 1 : 0;
    data[1] = brake_pressure;
    data[2] = brake_light ? 1 : 0;
    data[3] = brake_fault ? 1 : 0;

    return CAN_Protocol_SendRaw(hcan, CAN_ID_BRAKE_STATUS, data, 4);
}


HAL_StatusTypeDef CAN_Protocol_SendFaultStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t source_node,
    uint8_t fault_code,
    uint8_t fault_active
)
{
    uint8_t data[8] = {0};

    data[0] = source_node;
    data[1] = fault_code;
    data[2] = fault_active ? 1 : 0;

    return CAN_Protocol_SendRaw(hcan, CAN_ID_FAULT_STATUS, data, 3);
}


uint8_t CAN_Protocol_Decode(
    CAN_RxHeaderTypeDef *rx_header,
    uint8_t rx_data[8],
    CAN_DecodedMessage_t *decoded_msg
)
{
    if (rx_header == 0 || rx_data == 0 || decoded_msg == 0)
    {
        return 0;
    }

    decoded_msg->type = CAN_MSG_UNKNOWN;

    switch (rx_header->StdId)
    {
        case CAN_ID_THROTTLE_STATUS:
            decoded_msg->type = CAN_MSG_THROTTLE_STATUS;

            decoded_msg->data.throttle.throttle_percent = rx_data[0];

            decoded_msg->data.throttle.rpm =
                ((uint16_t)rx_data[1] << 8) |
                ((uint16_t)rx_data[2]);

            decoded_msg->data.throttle.speed_kph =
                ((uint16_t)rx_data[3] << 8) |
                ((uint16_t)rx_data[4]);

            return 1;

        case CAN_ID_ENGINE_STATUS:
            decoded_msg->type = CAN_MSG_ENGINE_STATUS;

            decoded_msg->data.engine.rpm =
                ((uint16_t)rx_data[0] << 8) |
                ((uint16_t)rx_data[1]);

            decoded_msg->data.engine.speed_kph =
                ((uint16_t)rx_data[2] << 8) |
                ((uint16_t)rx_data[3]);

            decoded_msg->data.engine.engine_temp_c = rx_data[4];
            decoded_msg->data.engine.engine_fault = rx_data[5];

            return 1;

        case CAN_ID_BRAKE_STATUS:
            decoded_msg->type = CAN_MSG_BRAKE_STATUS;

            decoded_msg->data.brake.brake_active = rx_data[0];
            decoded_msg->data.brake.brake_pressure = rx_data[1];
            decoded_msg->data.brake.brake_light = rx_data[2];
            decoded_msg->data.brake.brake_fault = rx_data[3];

            return 1;

        case CAN_ID_FAULT_STATUS:
            decoded_msg->type = CAN_MSG_FAULT_STATUS;

            decoded_msg->data.fault.source_node = rx_data[0];
            decoded_msg->data.fault.fault_code = rx_data[1];
            decoded_msg->data.fault.fault_active = rx_data[2];

            return 1;

        default:
            decoded_msg->type = CAN_MSG_UNKNOWN;
            return 0;
    }
}