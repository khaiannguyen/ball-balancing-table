#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>

/*
 * PID controller with anti-windup and velocity-based derivative feedback.
 *
 * The controller provides two update paths:
 *
 * - update():
 *   Computes the derivative term from the error difference.
 *
 * - update_with_velocity():
 *   Uses measured velocity directly for the derivative term. This is
 *   preferred when the measured position comes from a low-rate vision
 *   pipeline because numerical differentiation can amplify detection noise.
 *
 * Anti-windup protection combines:
 *
 * 1. Conditional integration:
 *    Prevents further integration when the output is saturated and the
 *    current error would drive the output farther into saturation.
 *
 * 2. Integral clamp:
 *    Limits the accumulated integral independently of the output limits.
 *
 * 3. Integral leak:
 *    Gradually reduces the stored integral to prevent long-term accumulation.
 *
 * 4. Zero-crossing reset:
 *    Clears the integral when the error crosses through zero, preventing
 *    residual integral action from driving the system further after the
 *    setpoint has been crossed.
 */
class PIDController
{
public:
    PIDController(
        float kp,
        float ki,
        float kd,
        float out_min,
        float out_max
    )
        : kp_(kp),
        ki_(ki),
        kd_(kd),
        out_min_(out_min),
        out_max_(out_max)
    {
        /*
         * Use the output range as the default integral limit.
         *
         * The limit can be overridden with set_integral_limit() when the
         * application requires a different integral authority.
         */
        integral_limit_ =
            std::fabs(out_max_);
    }

    /*
     * Sets the maximum absolute value allowed for the integral state.
     */
    void set_integral_limit(
        float limit)
    {
        integral_limit_ =
            std::fabs(limit);
    }

    /*
     * Sets the per-cycle integral decay factor.
     *
     * A value of 1.0 disables decay. Lower values gradually reduce the
     * stored integral before each new error contribution is added.
     */
    void set_integral_leak(
        float leak_per_cycle)
    {
        integral_leak_ =
            leak_per_cycle;
    }

    /*
     * Sets the error deadband.
     *
     * Errors inside the deadband are treated as zero. This prevents small
     * measurement noise around the setpoint from producing unnecessary
     * proportional and integral activity.
     */
    void set_error_deadband(
        float deadband)
    {
        error_deadband_ =
            std::fabs(deadband);
    }

    /*
     * Sets the EMA coefficient used to filter measured velocity.
     *
     * alpha = 1.0 disables filtering.
     * Lower values provide stronger smoothing at the cost of additional
     * response delay.
     */
    void set_velocity_filter_alpha(
        float alpha)
    {
        vel_filter_alpha_ =
            alpha;
    }

    /*
     * Updates the PID controller using the error derivative.
     *
     * The derivative term is calculated from the change in error between
     * consecutive control cycles.
     */
    float update(
        float setpoint,
        float measured,
        float dt)
    {
        float error =
            setpoint - measured;

        float p_term =
            kp_ * error;

        float d_term =
            (dt > 1e-6f)
            ? kd_ * (error - last_error_) / dt
            : 0.f;

        /*
         * Evaluate the output before integrating the new error.
         *
         * Integration is skipped when the resulting integral action would
         * increase an existing output saturation in the same direction as
         * the error.
         */
        float trial_output =
            p_term +
            ki_ * integral_ +
            d_term;

        bool saturated_high =
            trial_output > out_max_;

        bool saturated_low =
            trial_output < out_min_;

        bool would_worsen =
            (saturated_high && error > 0) ||
            (saturated_low && error < 0);

        if (!would_worsen)
        {
            integral_ +=
                error * dt;
        }

        float output =
            p_term +
            ki_ * integral_ +
            d_term;

        if (output > out_max_)
        {
            output =
                out_max_;
        }

        if (output < out_min_)
        {
            output =
                out_min_;
        }

        last_error_ =
            error;

        return output;
    }

    /*
     * Updates the PID controller using measured velocity for the derivative
     * term.
     *
     * This avoids differentiating the position signal directly. It is useful
     * when position measurements originate from a vision pipeline where
     * frame-to-frame quantization and detection noise can produce a noisy
     * numerical derivative.
     */
    float update_with_velocity(
        float setpoint,
        float measured,
        float measured_velocity,
        float dt)
    {
        float error =
            setpoint - measured;

        /*
         * Ignore small errors inside the configured deadband.
         *
         * This also prevents small noise-induced sign changes from
         * repeatedly triggering the zero-crossing integral reset.
         */
        if (std::fabs(error) <
            error_deadband_)
        {
            error = 0.f;
        }

        /*
         * Reset the integral when the error crosses zero.
         *
         * The reset is applied only after the deadband has been evaluated,
         * so measurement noise around the setpoint does not repeatedly clear
         * the accumulated integral.
         */
        bool sign_changed =
            (error > 0.f && last_error_ < 0.f) ||
            (error < 0.f && last_error_ > 0.f);

        if (sign_changed)
        {
            integral_ =
                0.f;
        }

        /*
         * Apply integral decay before accumulating the current error.
         *
         * With a value of 1.0, the integral remains unchanged.
         */
        integral_ *=
            integral_leak_;

        /*
         * Filter measured velocity before using it for the derivative term.
         *
         * The derivative contribution uses the negative measured velocity
         * because the velocity already represents the motion of the measured
         * variable rather than the change in control error.
         */
        filtered_velocity_ +=
            vel_filter_alpha_ *
            (measured_velocity -
                filtered_velocity_);

        float p_term =
            kp_ * error;

        float d_term =
            -kd_ *
            filtered_velocity_;

        /*
         * Conditional integration prevents the integral from increasing
         * output saturation in the direction of the current error.
         */
        float trial_output =
            p_term +
            ki_ * integral_ +
            d_term;

        bool saturated_high =
            trial_output > out_max_;

        bool saturated_low =
            trial_output < out_min_;

        bool would_worsen =
            (saturated_high && error > 0) ||
            (saturated_low && error < 0);

        if (!would_worsen)
        {
            integral_ +=
                error * dt;
        }

        /*
         * Clamp the integral independently of the output limits.
         *
         * This provides a second protection layer against long-term
         * accumulation when the error remains non-zero for an extended
         * period.
         */
        if (integral_ >
            integral_limit_)
        {
            integral_ =
                integral_limit_;
        }

        if (integral_ <
            -integral_limit_)
        {
            integral_ =
                -integral_limit_;
        }

        float output =
            p_term +
            ki_ * integral_ +
            d_term;

        if (output > out_max_)
        {
            output =
                out_max_;
        }

        if (output < out_min_)
        {
            output =
                out_min_;
        }

        last_error_ =
            error;

        return output;
    }

    /*
     * Clears all dynamic controller state.
     *
     * Call this when the controlled signal becomes invalid, when the
     * controller is disabled, or when the control loop restarts. Resetting
     * the velocity filter also prevents a stale derivative contribution from
     * appearing when valid measurements resume.
     */
    void reset()
    {
        integral_ =
            0.f;

        last_error_ =
            0.f;

        filtered_velocity_ =
            0.f;
    }

private:
    float kp_;
    float ki_;
    float kd_;

    float out_min_;
    float out_max_;

    float integral_ = 0.f;
    float last_error_ = 0.f;

    /*
     * Maximum absolute integral state.
     */
    float integral_limit_;

    /*
     * Per-cycle integral decay factor.
     *
     * 1.0 means no decay.
     */
    float integral_leak_ = 1.0f;

    /*
     * Position error deadband.
     *
     * 0.0 means no deadband.
     */
    float error_deadband_ = 0.f;

    /*
     * EMA coefficient for measured velocity.
     *
     * 1.0 means no filtering.
     */
    float vel_filter_alpha_ = 1.0f;

    float filtered_velocity_ = 0.f;
};

#endif