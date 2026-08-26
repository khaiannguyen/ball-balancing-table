#ifndef OPERATING_MODE_HPP
#define OPERATING_MODE_HPP

#include <cstdint>

/*
 * Jetson operating mode values.
 *
 * These numeric values must remain synchronized with the STM32 operating
 * mode definition because the mode is exchanged directly as a uint8_t
 * through CAN message 0x103 ROBOT_STATE.
 *
 * Keep these values unchanged when modifying the mode definitions on either
 * platform.
 */
namespace opmode
{
    constexpr uint8_t HOME = 0;
    constexpr uint8_t CALIB = 1;
    constexpr uint8_t BALANCE = 2;
    constexpr uint8_t POSITION = 3;
    constexpr uint8_t MANUAL = 4;
}

#endif