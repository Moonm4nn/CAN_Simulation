# STM32 CAN Vehicle Simulation

This project simulates a vehicle CAN network using three STM32 boards.

## Nodes

| Node | Role | Description |
|---|---|---|
| Node A | Throttle / Engine | Reads throttle input and sends simulated RPM/speed |
| Node B | Gateway / Logger | Receives CAN messages and logs system state |
| Node C | Brake | Reads brake input and sends brake pressure/status |

## Development Setup

- STM32CubeMX for project generation
- STM32 HAL for CAN handling
- VS Code for editing/building
- Git for collaboration

## Folder Structure

- `firmware/node_a_throttle/`
- `firmware/node_b_gateway/`
- `firmware/node_c_brake/`
- `shared/`
- `docs/`