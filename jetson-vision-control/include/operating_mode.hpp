#ifndef OPERATING_MODE_HPP
#define OPERATING_MODE_HPP
#include <cstdint>

/* Hang so mode phia Jetson - PHAI giu DUNG gia tri so voi enum
 * operating_mode_t ben STM32 (system_state.h), vi ca 2 cung doc/ghi chung
 * 1 truong uint8_t mode qua CAN 0x103 ROBOT_STATE
 * (telemetry_robot_state_t.mode, xem system_state.hpp/.cpp). Dat rieng 1
 * namespace hang so o day (khong dung enum class) de so sanh truc tiep
 * voi uint8_t mode doc duoc tu telemetry_robot_state_read() ma khong can
 * ep kieu dai dong. NEU STM32 doi enum operating_mode_t, phai sua LAI
 * DUNG cac gia tri nay cho khop. */
namespace opmode {
    constexpr uint8_t HOME     = 0;
    constexpr uint8_t CALIB    = 1;
    constexpr uint8_t BALANCE  = 2;
    constexpr uint8_t POSITION = 3;
    constexpr uint8_t MANUAL   = 4;
}

#endif