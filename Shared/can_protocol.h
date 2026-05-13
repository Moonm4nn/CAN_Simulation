#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * CAN Message IDs
 * Standard 11-bit CAN IDs are used.
 */
#define CAN_ID_THROTTLE_STATUS   0x100
#define CAN_ID_ENGINE_STATUS     0x101
#define CAN_ID_BRAKE_STATUS      0x200
#define CAN_ID_FAULT_STATUS      0x300
#define CAN_ID_DASHBOARD_COMMAND 0x400

/*
 * Generic decoded CAN message type.
 */
typedef enum
{
    CAN_MSG_UNKNOWN = 0,
    CAN_MSG_THROTTLE_STATUS,
    CAN_MSG_ENGINE_STATUS,
    CAN_MSG_BRAKE_STATUS,
    CAN_MSG_FAULT_STATUS,
    CAN_MSG_DASHBOARD_COMMAND
} CAN_MessageType_t;

/*
 * Throttle status packet
 * Sent by Node A.
 */
typedef struct
{
    uint8_t throttle_percent;   // 0-100
    uint16_t rpm;               // simulated engine RPM
    uint16_t speed_kph;         // simulated vehicle speed
} CAN_ThrottleStatus_t;

/*
 * Engine status packet
 * Sent by Node A or engine simulator logic.
 */
typedef struct
{
    uint16_t rpm;
    uint16_t speed_kph;
    uint8_t engine_temp_c;
    uint8_t engine_fault;
} CAN_EngineStatus_t;

/*
 * Brake status packet
 * Sent by Node C.
 */
typedef struct
{
    uint8_t brake_active;       // 0 or 1
    uint8_t brake_pressure;     // 0-100
    uint8_t brake_light;        // 0 or 1
    uint8_t brake_fault;        // 0 or 1
} CAN_BrakeStatus_t;

/*
 * Fault status packet
 * Not fully implemented yet
 */
typedef struct
{
    uint8_t source_node;        // example: 1 = throttle, 2 = dashboard, 3 = brake
    uint8_t fault_code;
    uint8_t fault_active;
} CAN_FaultStatus_t;

/*
 * Decoded message container.
 */
typedef struct
{
    CAN_MessageType_t type;

    union
    {
        CAN_ThrottleStatus_t throttle;
        CAN_EngineStatus_t engine;
        CAN_BrakeStatus_t brake;
        CAN_FaultStatus_t fault;
    } data;

} CAN_DecodedMessage_t;


/*
 * Transmit functions
 */
HAL_StatusTypeDef CAN_Protocol_SendThrottleStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t throttle_percent,
    uint16_t rpm,
    uint16_t speed_kph
);

HAL_StatusTypeDef CAN_Protocol_SendEngineStatus(
    CAN_HandleTypeDef *hcan,
    uint16_t rpm,
    uint16_t speed_kph,
    uint8_t engine_temp_c,
    uint8_t engine_fault
);

HAL_StatusTypeDef CAN_Protocol_SendBrakeStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t brake_active,
    uint8_t brake_pressure,
    uint8_t brake_light,
    uint8_t brake_fault
);

HAL_StatusTypeDef CAN_Protocol_SendFaultStatus(
    CAN_HandleTypeDef *hcan,
    uint8_t source_node,
    uint8_t fault_code,
    uint8_t fault_active
);


/*
 * Receive/decode function
 */
uint8_t CAN_Protocol_Decode(
    CAN_RxHeaderTypeDef *rx_header,
    uint8_t rx_data[8],
    CAN_DecodedMessage_t *decoded_msg
);

#ifdef __cplusplus
}
#endif

#endif