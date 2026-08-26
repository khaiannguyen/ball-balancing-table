#include "control_mode_home.h"
#include "servo_actuator.h"
#include "calibration_data.h"
#include "system_state.h"
#include "task_state_machine.h"
#include "trajectory.h"
#include "cmsis_os2.h"
#include <stdlib.h>
#include <math.h>

extern osMessageQueueId_t StateRequestQueueHandle;

/*
 * Home is considered stable only when all three servos remain within
 * the configured tolerance for the required number of control cycles.
 */
#define HOME_TOLERANCE_US    5
#define HOME_SETTLE_CYCLES   20

/*
 * Motion limits for the synchronized home trajectory.
 *
 * V_MAX is kept below the actuator slew-rate limit so that the trajectory
 * remains the primary motion constraint and the slew limiter acts only
 * as a final safety boundary.
 *
 * A_MAX is selected to provide a short acceleration phase without an
 * abrupt change in servo motion.
 */
#define HOME_TRAJ_V_MAX_US_S    3000.0f
#define HOME_TRAJ_A_MAX_US_S2   8000.0f

static bool     s_entered    = false;
static bool     s_evt_sent   = false;
static uint16_t s_settle_ct  = 0;

static trajectory_t s_traj[3];
static bool         s_traj_done = false;

void control_mode_home_enter(void)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    /*
     * Use the actual actuator position as the trajectory start point.
     * The previous target is not sufficient because it may differ from
     * the physical servo position at the moment the mode is entered.
     */
    int32_t s1, s2, s3;

    servo_actuator_get_local(&s1, &s2, &s3);

    float from[3] = {
        (float)s1,
        (float)s2,
        (float)s3
    };

    float to[3] = {
        (float)c->S1_neutral,
        (float)c->S2_neutral,
        (float)c->S3_neutral
    };

    /*
     * Generate synchronized trajectories for all three servos.
     * Shorter axes are automatically scaled so that all axes reach
     * their neutral targets at the same time.
     */
    trajectory_start_synced3(
        s_traj,
        from,
        to,
        HOME_TRAJ_V_MAX_US_S,
        HOME_TRAJ_A_MAX_US_S2
    );

    s_entered   = true;
    s_evt_sent  = false;
    s_settle_ct = 0;
    s_traj_done = false;
}

void control_mode_home_step(float dt)
{
    if (!s_entered) {
        /*
         * Keep the mode self-initializing so that a missed enter()
         * call cannot leave the control loop without a valid trajectory.
         */
        control_mode_home_enter();
    }

    if (s_evt_sent) {
        /*
         * The completion event has already been accepted by the state
         * machine queue. No further actuator or queue updates are needed.
         */
        return;
    }

    if (!s_traj_done) {
        float p1, p2, p3;

        /*
         * Advance all three trajectories using the same control period.
         * Home only needs the position reference, so velocity and
         * acceleration outputs are not required.
         */
        trajectory_update(&s_traj[0], dt, &p1, NULL, NULL);
        trajectory_update(&s_traj[1], dt, &p2, NULL, NULL);
        trajectory_update(&s_traj[2], dt, &p3, NULL, NULL);

        servo_actuator_set_target(
            SERVO_CH_S1,
            (int32_t)lroundf(p1)
        );

        servo_actuator_set_target(
            SERVO_CH_S2,
            (int32_t)lroundf(p2)
        );

        servo_actuator_set_target(
            SERVO_CH_S3,
            (int32_t)lroundf(p3)
        );

        if (trajectory_is_done(&s_traj[0]) &&
            trajectory_is_done(&s_traj[1]) &&
            trajectory_is_done(&s_traj[2])) {
            s_traj_done = true;
        }
    }

    const calibration_data_t *c = calibration_data_get_ptr();

    int32_t s1, s2, s3;

    servo_actuator_get_local(&s1, &s2, &s3);

    /*
     * Trajectory completion only confirms that the commanded references
     * reached their targets. Verify the actual actuator positions
     * independently before declaring the home operation complete.
     */
    bool at_home =
        (abs((int)(s1 - c->S1_neutral)) < HOME_TOLERANCE_US) &&
        (abs((int)(s2 - c->S2_neutral)) < HOME_TOLERANCE_US) &&
        (abs((int)(s3 - c->S3_neutral)) < HOME_TOLERANCE_US);

    s_settle_ct = at_home
        ? (uint16_t)(s_settle_ct + 1)
        : 0;

    if (s_settle_ct >= HOME_SETTLE_CYCLES) {
        state_event_t evt = EVT_HOME_DONE;

        /*
         * Do not block the control loop while waiting for the state-event
         * queue. If the queue is temporarily full, retry on the next cycle
         * while preserving the completed settle condition.
         */
        osStatus_t st = osMessageQueuePut(
            StateRequestQueueHandle,
            &evt,
            0,
            0
        );

        if (st == osOK) {
            s_evt_sent = true;
        }
    }
}

bool control_mode_home_is_done(void)
{
    return s_evt_sent;
}
