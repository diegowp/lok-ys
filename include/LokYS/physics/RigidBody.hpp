#pragma once

#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include "LokYS/math/Units.hpp"

namespace LokYS::Physics {

class RigidBody {
public:
    RigidBody();
    explicit RigidBody(f64 massKg, const Math::Mat3d& inertiaTensor);

    void SetMass(f64 massKg) noexcept;
    [[nodiscard]] f64 GetMass() const noexcept { return m_mass; }
    [[nodiscard]] f64 GetInvMass() const noexcept { return m_invMass; }

    void SetInertiaTensor(const Math::Mat3d& inertiaTensor) noexcept;
    [[nodiscard]] const Math::Mat3d& GetInertiaTensor() const noexcept { return m_inertiaTensor; }
    [[nodiscard]] const Math::Mat3d& GetInvInertiaTensor() const noexcept { return m_invInertiaTensor; }

    // Dynamic mass distribution, CG & parallel-axis tensor tracking (Phase F1)
    void SetMassDistribution(
        f64 emptyMassKg,
        const Math::Vec3d& emptyCgBody,
        const Math::Mat3d& emptyInertiaTensor,
        const Math::Vec3d& fuelTankLocationBody = Math::Vec3d{ 0.0, 0.0, 0.0 },
        const Math::Vec3d& payloadLocationBody = Math::Vec3d{ 0.0, 0.0, 0.0 }) noexcept;

    void UpdateFuelAndPayload(f64 fuelMassKg, f64 payloadMassKg = 0.0) noexcept;

    [[nodiscard]] const Math::Vec3d& GetCenterOfGravityBody() const noexcept { return m_cgBody; }

    void SetState(const RigidBodyState& state) noexcept;
    [[nodiscard]] const RigidBodyState& GetState() const noexcept { return m_state; }

    // Force and moment accumulation in Body (FRD) frame
    void AddForceBody(const Math::Vec3d& forceBody) noexcept;
    void AddForceNed(const Math::Vec3d& forceNed) noexcept;
    void AddMomentBody(const Math::Vec3d& momentBody) noexcept;
    void AddForceAtPointBody(const Math::Vec3d& forceBody, const Math::Vec3d& pointBody) noexcept;
    void ClearForcesAndMoments() noexcept;

    // Enable/disable gravitational force
    void SetGravityEnabled(bool enabled) noexcept { m_gravityEnabled = enabled; }
    [[nodiscard]] bool IsGravityEnabled() const noexcept { return m_gravityEnabled; }

    // Standard RK4 numerical integration step
    void IntegrateRK4(f64 dt) noexcept;

    // Phase F1: State-dependent RK4 integration evaluating forces/moments at trial states
    template<typename ForceEvaluatorFunc>
    void IntegrateRK4(f64 dt, ForceEvaluatorFunc&& forceEvaluator) noexcept {
        if (dt <= 0.0) return;

        auto ComputeGravityForces = [this](const Math::Quat4d& attitude, const Math::Vec3d& extForces) -> Math::Vec3d {
            Math::Vec3d forces = extForces;
            if (m_gravityEnabled) {
                Math::Vec3d gNed{ 0.0, 0.0, Math::Constants::STANDARD_GRAVITY };
                Math::Vec3d gBody = attitude.Unrotate(gNed);
                forces += gBody * m_mass;
            }
            return forces;
        };

        const RigidBodyState s0 = m_state;

        // k1 at s0, t0
        Math::Vec3d extForces0{ 0.0, 0.0, 0.0 };
        Math::Vec3d extMoments0{ 0.0, 0.0, 0.0 };
        forceEvaluator(s0, 0.0, extForces0, extMoments0);
        Math::Vec3d f0 = ComputeGravityForces(s0.attitude, extForces0);
        RigidBodyDerivatives k1 = EvaluateDerivatives(s0, f0, extMoments0);

        // k2 at s1, t0 + 0.5*dt
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

        Math::Vec3d extForces1{ 0.0, 0.0, 0.0 };
        Math::Vec3d extMoments1{ 0.0, 0.0, 0.0 };
        forceEvaluator(s1, 0.5 * dt, extForces1, extMoments1);
        Math::Vec3d f1 = ComputeGravityForces(s1.attitude, extForces1);
        RigidBodyDerivatives k2 = EvaluateDerivatives(s1, f1, extMoments1);

        // k3 at s2, t0 + 0.5*dt
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

        Math::Vec3d extForces2{ 0.0, 0.0, 0.0 };
        Math::Vec3d extMoments2{ 0.0, 0.0, 0.0 };
        forceEvaluator(s2, 0.5 * dt, extForces2, extMoments2);
        Math::Vec3d f2 = ComputeGravityForces(s2.attitude, extForces2);
        RigidBodyDerivatives k3 = EvaluateDerivatives(s2, f2, extMoments2);

        // k4 at s3, t0 + dt
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

        Math::Vec3d extForces3{ 0.0, 0.0, 0.0 };
        Math::Vec3d extMoments3{ 0.0, 0.0, 0.0 };
        forceEvaluator(s3, dt, extForces3, extMoments3);
        Math::Vec3d f3 = ComputeGravityForces(s3.attitude, extForces3);
        RigidBodyDerivatives k4 = EvaluateDerivatives(s3, f3, extMoments3);

        // Final weighted sum
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

        ClearForcesAndMoments();
    }

    // Derivative evaluation function
    [[nodiscard]] RigidBodyDerivatives EvaluateDerivatives(
        const RigidBodyState& state,
        const Math::Vec3d& totalForcesBody,
        const Math::Vec3d& totalMomentsBody) const noexcept;

private:
    f64 m_mass{ 1111.0 }; // Cessna 172 gross weight (~2450 lbs = 1111 kg) default baseline
    f64 m_invMass{ 1.0 / 1111.0 };

    Math::Mat3d m_inertiaTensor{ Math::Mat3d::Identity() };
    Math::Mat3d m_invInertiaTensor{ Math::Mat3d::Identity() };

    // Mass distribution & CG tracking (Phase F1)
    f64 m_emptyMassKg{ 1111.0 };
    Math::Vec3d m_emptyCgBody{ 0.0, 0.0, 0.0 };
    Math::Mat3d m_emptyInertiaTensor{ Math::Mat3d::Identity() };

    f64 m_fuelMassKg{ 0.0 };
    Math::Vec3d m_fuelTankLocationBody{ 0.0, 0.0, 0.0 };

    f64 m_payloadMassKg{ 0.0 };
    Math::Vec3d m_payloadLocationBody{ 0.0, 0.0, 0.0 };

    Math::Vec3d m_cgBody{ 0.0, 0.0, 0.0 };

    RigidBodyState m_state{};

    Math::Vec3d m_accumulatedForcesBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d m_accumulatedMomentsBody{ 0.0, 0.0, 0.0 };

    bool m_gravityEnabled{ true };
};

} // namespace LokYS::Physics
