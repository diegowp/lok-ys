#include "LokYS/physics/RigidBody.hpp"
#include <algorithm>
#include <cmath>

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
    // Moment = r x F where r is lever arm from dynamic CG
    Math::Vec3d leverArm = pointBody - m_cgBody;
    m_accumulatedMomentsBody += leverArm.Cross(forceBody);
}

void RigidBody::SetMassDistribution(
    f64 emptyMassKg,
    const Math::Vec3d& emptyCgBody,
    const Math::Mat3d& emptyInertiaTensor,
    const Math::Vec3d& fuelTankLocationBody,
    const Math::Vec3d& payloadLocationBody) noexcept
{
    m_emptyMassKg = std::max(emptyMassKg, 1e-3);
    m_emptyCgBody = emptyCgBody;
    m_emptyInertiaTensor = emptyInertiaTensor;
    m_fuelTankLocationBody = fuelTankLocationBody;
    m_payloadLocationBody = payloadLocationBody;

    UpdateFuelAndPayload(m_fuelMassKg, m_payloadMassKg);
}

void RigidBody::UpdateFuelAndPayload(f64 fuelMassKg, f64 payloadMassKg) noexcept {
    m_fuelMassKg = std::max(0.0, fuelMassKg);
    m_payloadMassKg = std::max(0.0, payloadMassKg);

    m_mass = m_emptyMassKg + m_fuelMassKg + m_payloadMassKg;
    m_invMass = 1.0 / m_mass;

    // Instantaneous composite CG: r_cg = (m_empty*r_empty + m_fuel*r_fuel + m_payload*r_payload) / m_total
    m_cgBody = (m_emptyCgBody * m_emptyMassKg +
                m_fuelTankLocationBody * m_fuelMassKg +
                m_payloadLocationBody * m_payloadMassKg) * m_invMass;

    // Parallel Axis Theorem (Steiner) for Inertia Tensor:
    auto SteinerTerm = [](f64 m, const Math::Vec3d& d) -> Math::Mat3d {
        f64 x = d.x;
        f64 y = d.y;
        f64 z = d.z;
        return Math::Mat3d(
            m * (y * y + z * z), -m * x * y,          -m * x * z,
           -m * x * y,           m * (x * x + z * z), -m * y * z,
           -m * x * z,          -m * y * z,           m * (x * x + y * y)
        );
    };

    Math::Vec3d dEmpty = m_emptyCgBody - m_cgBody;
    Math::Vec3d dFuel = m_fuelTankLocationBody - m_cgBody;
    Math::Vec3d dPayload = m_payloadLocationBody - m_cgBody;

    Math::Mat3d totalInertia = m_emptyInertiaTensor +
                               SteinerTerm(m_emptyMassKg, dEmpty) +
                               SteinerTerm(m_fuelMassKg, dFuel) +
                               SteinerTerm(m_payloadMassKg, dPayload);

    SetInertiaTensor(totalInertia);
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

    // Safeguard: Clamp non-physical linear and angular acceleration blowups
    constexpr f64 MAX_LINEAR_ACCEL = 500.0; // ~50 G
    f64 linAccel = deriv.dVelocityBody.Length();
    if (!std::isfinite(linAccel) || linAccel > MAX_LINEAR_ACCEL) {
        if (linAccel > 1e-6) {
            deriv.dVelocityBody = deriv.dVelocityBody * (MAX_LINEAR_ACCEL / linAccel);
        } else {
            deriv.dVelocityBody = { 0.0, 0.0, 0.0 };
        }
    }

    constexpr f64 MAX_ANGULAR_ACCEL = 100.0; // ~5700 deg/s^2
    f64 angAccel = deriv.dAngularVelocityBody.Length();
    if (!std::isfinite(angAccel) || angAccel > MAX_ANGULAR_ACCEL) {
        if (angAccel > 1e-6) {
            deriv.dAngularVelocityBody = deriv.dAngularVelocityBody * (MAX_ANGULAR_ACCEL / angAccel);
        } else {
            deriv.dAngularVelocityBody = { 0.0, 0.0, 0.0 };
        }
    }

    return deriv;
}

void RigidBody::IntegrateRK4(f64 dt) noexcept {
    IntegrateRK4(dt, [this](
        [[maybe_unused]] const RigidBodyState& s,
        [[maybe_unused]] f64 subDt,
        Math::Vec3d& outF,
        Math::Vec3d& outM) noexcept
    {
        outF = m_accumulatedForcesBody;
        outM = m_accumulatedMomentsBody;
    });
}

} // namespace LokYS::Physics
