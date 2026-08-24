#ifndef CAN_ID_H
#define CAN_ID_H

/* STM32 -> Jetson (base range: 0x100) */
#define CAN_ID_ATTITUDE          0x100
#define CAN_ID_RATE              0x101
#define CAN_ID_SERVO_POS         0x102
#define CAN_ID_ROBOT_STATE       0x103
#define CAN_ID_BALL_DESIRED      0x104
#define CAN_ID_HEARTBEAT_TX      0x1FF

/* Jetson -> STM32 (base range: 0x200) */
#define CAN_ID_BALL_POS           0x200
#define CAN_ID_BALL_VEL           0x201
#define CAN_ID_BALL_STATE         0x202
#define CAN_ID_SERVO_CALIB        0x203
#define CAN_ID_ATTITUDE_DESIRED   0x204
#define CAN_ID_HEARTBEAT_RX       0x2FF

/* Reserved CAN ID for bring-up loopback testing.
 * This ID is not part of the production protocol table.
 */
#define CAN_ID_LOOPBACK_TEST      0x555

#endif
