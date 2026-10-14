#pragma once

#include "LokYS/core/Types.hpp"
#include <algorithm>
#include <cmath>

namespace LokYS::Autopilot {

struct PidGains {
    f64 kp{ 1.0 };
    f64 ki{ 0.0 };
    f64 kd{ 0.0 };
    f64 minIntegral{ -1.0 };
    f64 maxIntegral{ 1.0 };
    f64 minOutput{ -1.0 };
    f64 maxOutput{ 1.0 };
    f64 maxRate{ 10.0 };              // Slew rate limit (units per second)
    f64 derivativeFilterAlpha{ 0.25 }; // Low-pass filter smoothing coefficient for derivative
};

class PidController {
public:
    PidController() = default;
    explicit PidController(const PidGains& gains) : m_gains(gains) {}

    void Reset() noexcept {
        m_integral = 0.0;
        m_prevError = 0.0;
        m_prevDerivative = 0.0;
        m_prevOutput = 0.0;
        m_initialized = false;
    }

    void SetGains(const PidGains& gains) noexcept { m_gains = gains; }
    [[nodiscard]] const PidGains& GetGains() const noexcept { return m_gains; }

    f64 Update(f64 error, f64 dt) noexcept {
        if (dt <= 0.0) {
            return m_prevOutput;
        }

        // Proportional term
        f64 pTerm = m_gains.kp * error;

        // Integral term with anti-windup clamping
        m_integral += error * dt;
        m_integral = std::clamp(m_integral, m_gains.minIntegral, m_gains.maxIntegral);
        f64 iTerm = m_gains.ki * m_integral;

        // Derivative term with first-order low-pass filter
        f64 derivative = 0.0;
        if (m_initialized) {
            f64 rawDerivative = (error - m_prevError) / dt;
            derivative = m_gains.derivativeFilterAlpha * rawDerivative +
                         (1.0 - m_gains.derivativeFilterAlpha) * m_prevDerivative;
        } else {
            m_initialized = true;
        }
        m_prevDerivative = derivative;
        m_prevError = error;
        f64 dTerm = m_gains.kd * derivative;

        // Raw PID output
        f64 rawOutput = pTerm + iTerm + dTerm;

        // Slew rate limiting to prevent actuator snap
        f64 maxDelta = m_gains.maxRate * dt;
        f64 rateLimited = std::clamp(rawOutput, m_prevOutput - maxDelta, m_prevOutput + maxDelta);

        // Saturation clamping
        f64 output = std::clamp(rateLimited, m_gains.minOutput, m_gains.maxOutput);
        m_prevOutput = output;
        return output;
    }

    [[nodiscard]] f64 GetIntegral() const noexcept { return m_integral; }
    [[nodiscard]] f64 GetPreviousOutput() const noexcept { return m_prevOutput; }

private:
    PidGains m_gains{};
    f64 m_integral{ 0.0 };
    f64 m_prevError{ 0.0 };
    f64 m_prevDerivative{ 0.0 };
    f64 m_prevOutput{ 0.0 };
    bool m_initialized{ false };
};

} // namespace LokYS::Autopilot
