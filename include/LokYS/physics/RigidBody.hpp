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

    // RK4 numerical integration step
    void IntegrateRK4(f64 dt) noexcept;

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

    RigidBodyState m_state{};

    Math::Vec3d m_accumulatedForcesBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d m_accumulatedMomentsBody{ 0.0, 0.0, 0.0 };

    bool m_gravityEnabled{ true };
};

} // namespace LokYS::Physics
