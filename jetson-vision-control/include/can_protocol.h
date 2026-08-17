#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>
#include "can_id.h"     /* TẤT CẢ CAN_ID_* nằm ở đây, không định nghĩa lại ở file này
                           (trước đó bị trùng với can_id.h -> lỗi redefinition khi build) */

typedef struct {
    uint32_t id;      // Standard ID 11-bit
    uint8_t  dlc;      // Data Length Code, 0..8 (Classic CAN)
    uint8_t  data[8];  // little-endian theo đúng mục 4.2/4.3
} can_frame_t;

/* ROBOT_STATE (0x103) byte 1 - bitfield, dùng bởi task_can_tx.c khi đóng gói
 * và task_control_loop.c sau này khi cần biết trạng thái RUN/FAULT hiện tại */
#define ROBOT_STATE_BIT_RUN     (1u << 0)
#define ROBOT_STATE_BIT_FAULT   (1u << 1)
#define ROBOT_STATE_BIT_CAM_OK  (1u << 2)   /* THÊM: camera_state_is_ok() (system_state.c),
                                                true khi nhận 0x200/0x201 trong 500ms gần nhất */

/* Vùng ID hợp lệ để cấu hình 1 filter range duy nhất (StdFiltersNbr = 1
 * đã set sẵn trong MX_FDCAN1_Init) — bao trọn cả 2 chiều 0x100-0x2FF,
 * dùng chung không đổi cho B3.1 (loopback) lẫn B3.2 (ESP32 thật). */
#define CAN_FILTER_ID_LOW     0x100u
#define CAN_FILTER_ID_HIGH    0x2FFu

/* Failsafe timeout mục 4.4 */
#define CAN_HEARTBEAT_TIMEOUT_MS  200u

/* ---- Helper little-endian (dùng chung cho cả task_can_rx.c và task_can_tx.c) ---- */
static inline int16_t can_rd_i16le(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t can_rd_u16le(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline void can_wr_i16le(uint8_t *p, int16_t v) {
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}
static inline void can_wr_u16le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

#endif
