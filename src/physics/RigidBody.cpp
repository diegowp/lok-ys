#include "LokYS/physics/RigidBody.hpp"
#include <algorithm>

namespace LokYS::Physics {

RigidBody::RigidBody() {
    // Typical Cessna 172 baseline inertia tensor (kg*m^2):
    // Ixx ~ 1285, Iyy ~ 1824, Izz ~ 2667, Ixz ~ 0
    Math::Mat3d c172Inertia(
        1285.0, 0.0, 0.0,
        0.0, 1824.0, 0.0,
        0.0, 0.0, 2667.0
    );
    SetInertiaTensor(c172Inertia);
}

RigidBody::RigidBody(f64 massKg, const Math::Mat3d& inertiaTensor) {
    SetMass(massKg);
    SetInertiaTensor(inertiaTensor);
}

void RigidBody::SetMass(f64 massKg) noexcept {
    m_mass = std::max(massKg, 1e-3);
    m_invMass = 1.0 / m_mass;
}

void RigidBody::SetInertiaTensor(const Math::Mat3d& inertiaTensor) noexcept {
    m_inertiaTensor = inertiaTensor;
    m_invInertiaTensor = inertiaTensor.Inverse();
}

void RigidBody::SetState(const RigidBodyState& state) noexcept {
    m_state = state;
    m_state.attitude.Renormalize();
}

void RigidBody::AddForceBody(const Math::Vec3d& forceBody) noexcept {
    m_accumulatedForcesBody += forceBody;
}

void RigidBody::AddForceNed(const Math::Vec3d& forceNed) noexcept {
    // Transform NED force into Body frame using attitude quaternion conjugate: F_body = q* * F_ned * q
    m_accumulatedForcesBody += m_state.attitude.Unrotate(forceNed);
}

void RigidBody::AddMomentBody(const Math::Vec3d& momentBody) noexcept {
    m_accumulatedMomentsBody += momentBody;
}

void RigidBody::AddForceAtPointBody(const Math::Vec3d& forceBody, const Math::Vec3d& pointBody) noexcept {
    m_accumulatedForcesBody += forceBody;
    // Moment = r x F
    m_accumulatedMomentsBody += pointBody.Cross(forceBody);
}

void RigidBody::ClearForcesAndMoments() noexcept {
    m_accumulatedForcesBody = { 0.0, 0.0, 0.0 };
    m_accumulatedMomentsBody = { 0.0, 0.0, 0.0 };
}

RigidBodyDerivatives RigidBody::EvaluateDerivatives(
    const RigidBodyState& state,
    const Math::Vec3d& totalForcesBody,
    const Math::Vec3d& totalMomentsBody) const noexcept
{
    RigidBodyDerivatives deriv{};

    // 1. Linear velocity in Navigation (NED) frame: dPositionNed/dt = q * v_body * q*
    deriv.dPositionNed = state.attitude.Rotate(state.velocityBody);

    // 2. Linear acceleration in Body frame: dv/dt = (1/m)*F_body - omega x v_body
    Math::Vec3d coriolisAccel = state.angularVelocityBody.Cross(state.velocityBody);
    deriv.dVelocityBody = (totalForcesBody * m_invMass) - coriolisAccel;

    // 3. Attitude quaternion rate: dq/dt = 0.5 * q * [0, p, q, r]
    f64 p = state.angularVelocityBody.x;
    f64 q = state.angularVelocityBody.y;
    f64 r = state.angularVelocityBody.z;

    f64 qw = state.attitude.w;
    f64 qx = state.attitude.x;
    f64 qy = state.attitude.y;
    f64 qz = state.attitude.z;

    deriv.dAttitude.w = 0.5 * (-qx * p - qy * q - qz * r);
    deriv.dAttitude.x = 0.5 * ( qw * p + qy * r - qz * q);
    deriv.dAttitude.y = 0.5 * ( qw * q - qx * r + qz * p);
    deriv.dAttitude.z = 0.5 * ( qw * r + qx * q - qy * p);

    // 4. Angular acceleration: domega/dt = I^-1 * (M_body - omega x (I * omega))
    Math::Vec3d angularMomentum = m_inertiaTensor * state.angularVelocityBody;
    Math::Vec3d gyroTorque = state.angularVelocityBody.Cross(angularMomentum);
    deriv.dAngularVelocityBody = m_invInertiaTensor * (totalMomentsBody - gyroTorque);

    return deriv;
}

void RigidBody::IntegrateRK4(f64 dt) noexcept {
    if (dt <= 0.0) {
        return;
    }

    // Compute gravity force in body frame for current attitude
    auto ComputeTotalForces = [this](const Math::Quat4d& attitude) -> Math::Vec3d {
        Math::Vec3d forces = m_accumulatedForcesBody;
        if (m_gravityEnabled) {
            // Gravity vector in NED: [0, 0, g]
            Math::Vec3d gNed{ 0.0, 0.0, Math::Constants::STANDARD_GRAVITY };
            // Rotate NED gravity into Body frame: g_body = q* * g_ned * q
            Math::Vec3d gBody = attitude.Unrotate(gNed);
            forces += gBody * m_mass;
        }
        return forces;
    };

    const RigidBodyState s0 = m_state;

    // --- k1 ---
    Math::Vec3d forces0 = ComputeTotalForces(s0.attitude);
    RigidBodyDerivatives k1 = EvaluateDerivatives(s0, forces0, m_accumulatedMomentsBody);

    // --- k2 ---
    RigidBodyState s1{};
    s1.positionNed = s0.positionNed + k1.dPositionNed * (0.5 * dt);
    s1.velocityBody = s0.velocityBody + k1.dVelocityBody * (0.5 * dt);
    s1.attitude = Math::Quat4d(
        s0.attitude.w + k1.dAttitude.w * (0.5 * dt),
        s0.attitude.x + k1.dAttitude.x * (0.5 * dt),
        s0.attitude.y + k1.dAttitude.y * (0.5 * dt),
        s0.attitude.z + k1.dAttitude.z * (0.5 * dt)
    ).Normalized();
    s1.angularVelocityBody = s0.angularVelocityBody + k1.dAngularVelocityBody * (0.5 * dt);

    Math::Vec3d forces1 = ComputeTotalForces(s1.attitude);
    RigidBodyDerivatives k2 = EvaluateDerivatives(s1, forces1, m_accumulatedMomentsBody);

    // --- k3 ---
    RigidBodyState s2{};
    s2.positionNed = s0.positionNed + k2.dPositionNed * (0.5 * dt);
    s2.velocityBody = s0.velocityBody + k2.dVelocityBody * (0.5 * dt);
    s2.attitude = Math::Quat4d(
        s0.attitude.w + k2.dAttitude.w * (0.5 * dt),
        s0.attitude.x + k2.dAttitude.x * (0.5 * dt),
        s0.attitude.y + k2.dAttitude.y * (0.5 * dt),
        s0.attitude.z + k2.dAttitude.z * (0.5 * dt)
    ).Normalized();
    s2.angularVelocityBody = s0.angularVelocityBody + k2.dAngularVelocityBody * (0.5 * dt);

    Math::Vec3d forces2 = ComputeTotalForces(s2.attitude);
    RigidBodyDerivatives k3 = EvaluateDerivatives(s2, forces2, m_accumulatedMomentsBody);

    // --- k4 ---
    RigidBodyState s3{};
    s3.positionNed = s0.positionNed + k3.dPositionNed * dt;
    s3.velocityBody = s0.velocityBody + k3.dVelocityBody * dt;
    s3.attitude = Math::Quat4d(
        s0.attitude.w + k3.dAttitude.w * dt,
        s0.attitude.x + k3.dAttitude.x * dt,
        s0.attitude.y + k3.dAttitude.y * dt,
        s0.attitude.z + k3.dAttitude.z * dt
    ).Normalized();
    s3.angularVelocityBody = s0.angularVelocityBody + k3.dAngularVelocityBody * dt;

    Math::Vec3d forces3 = ComputeTotalForces(s3.attitude);
    RigidBodyDerivatives k4 = EvaluateDerivatives(s3, forces3, m_accumulatedMomentsBody);

    // --- Final State Update (RK4 weighted sum) ---
    const f64 dt6 = dt / 6.0;

    m_state.positionNed = s0.positionNed + (k1.dPositionNed + k2.dPositionNed * 2.0 + k3.dPositionNed * 2.0 + k4.dPositionNed) * dt6;
    m_state.velocityBody = s0.velocityBody + (k1.dVelocityBody + k2.dVelocityBody * 2.0 + k3.dVelocityBody * 2.0 + k4.dVelocityBody) * dt6;

    m_state.attitude = Math::Quat4d(
        s0.attitude.w + (k1.dAttitude.w + k2.dAttitude.w * 2.0 + k3.dAttitude.w * 2.0 + k4.dAttitude.w) * dt6,
        s0.attitude.x + (k1.dAttitude.x + k2.dAttitude.x * 2.0 + k3.dAttitude.x * 2.0 + k4.dAttitude.x) * dt6,
        s0.attitude.y + (k1.dAttitude.y + k2.dAttitude.y * 2.0 + k3.dAttitude.y * 2.0 + k4.dAttitude.y) * dt6,
        s0.attitude.z + (k1.dAttitude.z + k2.dAttitude.z * 2.0 + k3.dAttitude.z * 2.0 + k4.dAttitude.z) * dt6
    ).Normalized();

    m_state.angularVelocityBody = s0.angularVelocityBody + (k1.dAngularVelocityBody + k2.dAngularVelocityBody * 2.0 + k3.dAngularVelocityBody * 2.0 + k4.dAngularVelocityBody) * dt6;

    // Reset applied external forces and moments for next tick
    ClearForcesAndMoments();
}

} // namespace LokYS::Physics
