cmake_minimum_required(VERSION 3.22)

# Top-level workspace project to avoid per-node CMake conflicts.
project(CAN_simulation_workspace)

# Options to select which node(s) to configure/build
option(BUILD_NODE_A "Build Node A Throttle" OFF)
option(BUILD_NODE_B "Build Node B Log" OFF)
option(BUILD_NODE_C "Build Node C Brake" OFF)

# If user didn't choose any, enable all by default for backwards compatibility
if(NOT BUILD_NODE_A AND NOT BUILD_NODE_B AND NOT BUILD_NODE_C)
    message(STATUS "No specific node selected — enabling all nodes by default")
    set(BUILD_NODE_A ON)
    set(BUILD_NODE_B ON)
    set(BUILD_NODE_C ON)
endif()

if(BUILD_NODE_A)
    add_subdirectory(Firmware/Node_A_Throttle)
endif()

if(BUILD_NODE_B)
    add_subdirectory(Firmware/Node_B_Log)
endif()

if(BUILD_NODE_C)
    add_subdirectory(Firmware/Node_C_Brake)
endif()

message(STATUS "Configured nodes: A=${BUILD_NODE_A} B=${BUILD_NODE_B} C=${BUILD_NODE_C}")
