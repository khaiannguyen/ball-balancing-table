#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include "can_id.h"     /* All CAN_ID_* definitions are maintained in can_id.h.
                         * Keeping the definitions in one place avoids redefinition
                         * errors and keeps the CAN ID table centralized.
                         */

typedef struct
{
    uint32_t id;        /* Standard CAN ID (11-bit). */
    uint8_t  dlc;       /* Data Length Code, 0..8 bytes for Classic CAN. */
    uint8_t  data[8];   /* CAN payload encoded in little-endian format. */
} can_frame_t;


/* ROBOT_STATE (0x103) byte 1 bit definitions.
 * Used by task_can_tx.c when packing the status frame and by
 * task_control_loop.c when evaluating the current robot state.
 */
#define ROBOT_STATE_BIT_RUN     (1u << 0)
#define ROBOT_STATE_BIT_FAULT   (1u << 1)

/* Camera link status.
 * camera_state_is_ok() reports true when a valid 0x200 or 0x201
 * frame has been received within the last 500 ms.
 */
#define ROBOT_STATE_BIT_CAM_OK  (1u << 2)


/* Valid CAN ID range for the single FDCAN filter configured by
 * MX_FDCAN1_Init().
 *
 * The range covers both communication directions:
 *   STM32 -> Jetson : 0x100 - 0x1FF
 *   Jetson -> STM32 : 0x200 - 0x2FF
 *
 * The same filter range is used for both loopback bring-up testing
 * and normal communication.
 */
#define CAN_FILTER_ID_LOW     0x100u
#define CAN_FILTER_ID_HIGH    0x2FFu


/* Failsafe timeout for the CAN heartbeat. */
#define CAN_HEARTBEAT_TIMEOUT_MS  200u


/* Little-endian helpers shared by task_can_rx.c and task_can_tx.c. */
static inline int16_t can_rd_i16le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t can_rd_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void can_wr_i16le(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static inline void can_wr_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((uint16_t)v >> 8);
}

#endif
