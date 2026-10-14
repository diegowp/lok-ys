#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"

namespace LokYS::Physics {

struct RigidBodyState {
    Math::Vec3d positionNed{ 0.0, 0.0, 0.0 };         // Position in NED frame (meters)
    Math::Vec3d velocityBody{ 0.0, 0.0, 0.0 };        // Linear velocity [u, v, w] in Body frame (m/s)
    Math::Quat4d attitude{ Math::Quat4d::Identity() }; // Orientation quaternion (Body -> NED)
    Math::Vec3d angularVelocityBody{ 0.0, 0.0, 0.0 }; // Angular velocity [p, q, r] in Body frame (rad/s)

    [[nodiscard]] static RigidBodyState Interpolate(
        const RigidBodyState& a,
        const RigidBodyState& b,
        f64 alpha) noexcept
    {
        RigidBodyState result{};
        result.positionNed = Math::Vec3d::Lerp(a.positionNed, b.positionNed, alpha);
        result.velocityBody = Math::Vec3d::Lerp(a.velocityBody, b.velocityBody, alpha);
        result.attitude = Math::Quat4d::Slerp(a.attitude, b.attitude, alpha);
        result.angularVelocityBody = Math::Vec3d::Lerp(a.angularVelocityBody, b.angularVelocityBody, alpha);
        return result;
    }
};

struct RigidBodyDerivatives {
    Math::Vec3d dPositionNed{ 0.0, 0.0, 0.0 };        // dPosition/dt = q * v_body * q*
    Math::Vec3d dVelocityBody{ 0.0, 0.0, 0.0 };       // dv/dt = (1/m)*F_body - omega x v_body
    Math::Quat4d dAttitude{ 0.0, 0.0, 0.0, 0.0 };     // dq/dt = 0.5 * q * [0, p, q, r]
    Math::Vec3d dAngularVelocityBody{ 0.0, 0.0, 0.0 };// domega/dt = I^-1 * (M_body - omega x (I * omega))
};

} // namespace LokYS::Physics
