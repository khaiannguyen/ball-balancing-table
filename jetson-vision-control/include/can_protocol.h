#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

#include "can_id.h"

/*
 * CAN frame representation used by the Jetson CAN transport layer.
 *
 * The project uses Standard CAN identifiers with an 11-bit identifier and
 * Classic CAN payloads containing up to 8 data bytes.
 *
 * Multi-byte values use little-endian byte order as defined by the project
 * CAN protocol.
 */
typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

} can_frame_t;

/*
 * ROBOT_STATE (0x103) status flags.
 *
 * These flags are transmitted in byte 1 of the ROBOT_STATE frame and allow
 * the Jetson to determine the current STM32 runtime and camera status.
 */
#define ROBOT_STATE_BIT_RUN   (1u << 0)
#define ROBOT_STATE_BIT_FAULT (1u << 1)
#define ROBOT_STATE_BIT_CAM_OK (1u << 2)

 /*
  * Valid standard CAN identifier range used by the SocketCAN filter.
  *
  * The range covers both STM32 -> Jetson and Jetson -> STM32 application
  * messages.
  */
#define CAN_FILTER_ID_LOW  0x100u
#define CAN_FILTER_ID_HIGH 0x2FFu

  /*
   * Maximum allowed interval between heartbeat messages before the CAN link
   * is considered unhealthy.
   */
#define CAN_HEARTBEAT_TIMEOUT_MS 200u

   /*
    * Reads a signed 16-bit little-endian value from a CAN payload.
    */
static inline int16_t can_rd_i16le(
    const uint8_t* p)
{
    return (int16_t)(
        (uint16_t)p[0] |
        ((uint16_t)p[1] << 8)
        );
}

/*
 * Reads an unsigned 16-bit little-endian value from a CAN payload.
 */
static inline uint16_t can_rd_u16le(
    const uint8_t* p)
{
    return (uint16_t)(
        (uint16_t)p[0] |
        ((uint16_t)p[1] << 8)
        );
}

/*
 * Writes a signed 16-bit value to a CAN payload using little-endian order.
 */
static inline void can_wr_i16le(
    uint8_t* p,
    int16_t v)
{
    p[0] =
        (uint8_t)(
            (uint16_t)v & 0xFFu
            );

    p[1] =
        (uint8_t)(
            ((uint16_t)v >> 8) & 0xFFu
            );
}

/*
 * Writes an unsigned 16-bit value to a CAN payload using little-endian
 * order.
 */
static inline void can_wr_u16le(
    uint8_t* p,
    uint16_t v)
{
    p[0] =
        (uint8_t)(
            v & 0xFFu
            );

    p[1] =
        (uint8_t)(
            (v >> 8) & 0xFFu
            );
}

#endif