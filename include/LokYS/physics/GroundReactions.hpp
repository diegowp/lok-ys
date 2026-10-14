#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/physics/RigidBody.hpp"
#include <array>

namespace LokYS::Physics {

enum class GearId {
    Nose = 0,
    MainLeft = 1,
    MainRight = 2,
    TailBumper = 3
};

struct StrutProperties {
    GearId id{ GearId::Nose };
    Math::Vec3d mountPointBody{ 0.0, 0.0, 0.0 }; // Mounting hinge point in Body FRD
    f64 uncompressedLength{ 1.0 };                // Distance from mount to uncompressed tire bottom (m)
    f64 maxStroke{ 0.20 };                         // Max compression stroke (m)
    f64 springStiffness{ 50000.0 };                // Spring constant k (N/m)
    f64 dampingCompression{ 4500.0 };             // Compression damping coefficient c_comp (N*s/m)
    f64 dampingRebound{ 2500.0 };                 // Rebound damping coefficient c_rebound (N*s/m)
    f64 rollingFrictionCoeff{ 0.025 };            // Rolling resistance coefficient mu_roll
    f64 brakingFrictionCoeff{ 0.45 };             // Braking friction coefficient mu_brake
    f64 corneringStiffness{ 18000.0 };            // Cornering scrub stiffness C_alpha (N/rad)
    f64 maxSteerAngleRad{ 0.2618 };               // 15 degrees max steering angle (rad)
    bool isSteerable{ false };
    bool hasBrake{ true };
};

struct GearContactState {
    GearId id{ GearId::Nose };
    bool inContact{ false };
    f64 compression{ 0.0 };         // Compression distance delta (m)
    f64 compressionRate{ 0.0 };     // Compression velocity d(delta)/dt (m/s)
    f64 normalForceNewtons{ 0.0 };  // Normal reaction force (N)
    f64 frictionForceNewtons{ 0.0 };// Longitudinal tire force (N)
    f64 lateralForceNewtons{ 0.0 }; // Lateral scrub force (N)
    f64 slipAngleDeg{ 0.0 };        // Tire slip angle (deg)
    Math::Vec3d contactPointBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d forceBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d momentBody{ 0.0, 0.0, 0.0 };
};

struct GroundReactionsOutput {
    std::array<GearContactState, 3> gearStates{};
    GearContactState tailBumperState{};
    Math::Vec3d totalForceBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d totalMomentBody{ 0.0, 0.0, 0.0 };
    bool anyGearInContact{ false };
    bool noseInContact{ false };
    bool leftMainInContact{ false };
    bool rightMainInContact{ false };
    f64 totalNormalForce{ 0.0 };
    bool tailstrikeDetected{ false };
    bool wingtipStrikeDetected{ false };
};

struct AircraftLandingGearConfig {
    StrutProperties noseGear{
        GearId::Nose,
        Math::Vec3d{ 1.60, 0.0, 0.0 }, // 1.60m forward of CG
        1.0,                            // 1.0m uncompressed length
        0.20,                           // 0.20m oleo stroke
        45000.0,                        // 45 kN/m spring stiffness
        4000.0,                         // 4.0 kN*s/m compression damping
        2200.0,                         // 2.2 kN*s/m rebound damping
        0.025,                          // 0.025 rolling friction
        0.0,                            // No brake on nose wheel
        16000.0,                        // Cornering scrub stiffness
        0.2618,                         // 15 deg max steering
        true,                           // Steerable linked to rudder
        false                           // No brake
    };

    StrutProperties leftMainGear{
        GearId::MainLeft,
        Math::Vec3d{ -0.35, -1.05, 0.0 }, // 0.35m aft, 1.05m left
        1.0,                              // 1.0m uncompressed length
        0.18,                             // 0.18m stroke
        65000.0,                          // 65 kN/m spring-steel stiffness
        4500.0,                           // 4.5 kN*s/m compression damping
        2500.0,                           // 2.5 kN*s/m rebound damping
        0.025,                            // 0.025 rolling friction
        0.45,                             // 0.45 braking friction
        20000.0,                          // Cornering scrub stiffness
        0.0,                              // Fixed orientation
        false,                            // Non-steerable
        true                              // Left toe brake
    };

    StrutProperties rightMainGear{
        GearId::MainRight,
        Math::Vec3d{ -0.35, 1.05, 0.0 }, // 0.35m aft, 1.05m right
        1.0,                             // 1.0m uncompressed length
        0.18,                            // 0.18m stroke
        65000.0,                         // 65 kN/m spring-steel stiffness
        4500.0,                          // 4.5 kN*s/m compression damping
        2500.0,                          // 2.5 kN*s/m rebound damping
        0.025,                           // 0.025 rolling friction
        0.45,                            // 0.45 braking friction
        20000.0,                         // Cornering scrub stiffness
        0.0,                             // Fixed orientation
        false,                           // Non-steerable
        true                             // Right toe brake
    };

    Math::Vec3d tailBumperBody{ -5.5, 0.0, -0.1 };  // Tail cone bottom scrape point
    Math::Vec3d leftWingtipBody{ 0.2, -5.5, -0.6 }; // Left wingtip scrape point
    Math::Vec3d rightWingtipBody{ 0.2, 5.5, -0.6 }; // Right wingtip scrape point
};

class GroundReactionsSystem {
public:
    explicit GroundReactionsSystem(const AircraftLandingGearConfig& config = {});

    void SetConfig(const AircraftLandingGearConfig& config) noexcept { m_config = config; }
    [[nodiscard]] const AircraftLandingGearConfig& GetConfig() const noexcept { return m_config; }

    [[nodiscard]] GroundReactionsOutput Evaluate(
        const RigidBodyState& state,
        f64 rudderInput,
        f64 brakeInput,
        f64 dt,
        f64 terrainElevationNed = 0.0) const noexcept;

    void ApplyToRigidBody(RigidBody& body, const GroundReactionsOutput& out) const noexcept;

private:
    [[nodiscard]] GearContactState EvaluateStrut(
        const StrutProperties& strut,
        const RigidBodyState& state,
        f64 steerAngleRad,
        f64 brakeFactor,
        f64 terrainElevationNed) const noexcept;

    AircraftLandingGearConfig m_config{};
};

} // namespace LokYS::Physics
