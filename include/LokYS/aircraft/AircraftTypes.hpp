#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include <string>

namespace LokYS::Aircraft {

enum class AircraftType : u8 {
    Cessna172Skyhawk = 0, // Single-engine piston GA trainer (160 hp, high wing, fixed gear)
    Baron58 = 1,          // Multi-engine piston twin (2x 300 hp, low wing, retractable gear)
    Extra330SC = 2        // Unlimited competition aerobatic monoplane (315 hp, symmetric wing, +/-10G)
};

enum class AircraftCategory : u8 {
    SingleEnginePiston,
    MultiEnginePiston,
    Aerobatic
};

struct AircraftGeometryConfig {
    f64 wingAreaM2{ 16.165 };       // S: Wing reference area (m^2)
    f64 wingspanM{ 11.0 };          // b: Wingspan (m)
    f64 meanChordM{ 1.493 };        // c_bar: Mean aerodynamic chord (m)
    f64 aspectRatio{ 7.49 };        // AR = b^2 / S
    f64 oswaldEfficiency{ 0.75 };    // e: Oswald span efficiency factor
};

struct AeroDerivativesConfig {
    f64 CL0{ 0.307 };               // Lift coefficient at zero alpha
    f64 CLa{ 4.58 };                // Lift curve slope per radian
    f64 CLq{ 3.8 };                 // Pitch rate lift derivative
    f64 CLde{ 0.43 };               // Elevator lift effectiveness
    f64 CLdf{ 0.45 };               // Flap lift increment
    f64 stallAlphaRad{ 0.279 };     // Positive stall angle of attack (~16.0 deg)
    f64 stallAlphaNegRad{ -0.279 }; // Negative stall angle of attack (~ -16.0 deg)

    f64 CD0{ 0.027 };               // Parasite drag coefficient (clean)
    f64 CD0_gearDown{ 0.040 };      // Parasite drag with landing gear extended
    f64 CDdf{ 0.035 };              // Flap parasite drag increment

    f64 CYb{ -0.31 };               // Sideforce due to sideslip beta
    f64 CYdr{ 0.187 };              // Rudder sideforce effectiveness

    f64 Clb{ -0.089 };              // Dihedral effect (roll due to sideslip)
    f64 Clp{ -0.47 };               // Roll damping derivative
    f64 Clr{ 0.096 };               // Roll due to yaw rate
    f64 Clda{ 0.165 };              // Aileron roll effectiveness

    f64 Cm0{ 0.04 };                // Zero-alpha pitching moment
    f64 Cma{ -0.65 };               // Static longitudinal stability (pitch stiffness)
    f64 Cmq{ -12.4 };               // Pitch damping derivative
    f64 Cmde{ -1.122 };             // Elevator pitch effectiveness
    f64 Cmdf{ -0.05 };              // Flap pitching moment increment

    f64 Cnb{ 0.065 };               // Directional static stability (weathercocking)
    f64 Cnr{ -0.099 };              // Yaw damping derivative
    f64 Cnp{ -0.030 };              // Yaw due to roll rate
    f64 Cndr{ -0.074 };             // Rudder yaw effectiveness
    f64 criticalMach{ 0.65 };       // Transonic drag divergence onset Mach
};

struct AircraftEngineConfig {
    u32 engineCount{ 1 };           // 1 for C172/Extra330, 2 for Baron 58
    f64 horsepowerPerEngine{ 160.0 };
    f64 maxRatedRpm{ 2700.0 };
    f64 idleRpm{ 680.0 };
    f64 propDiameterM{ 1.905 };
    f64 propInertiaKgM2{ 0.38 };
    f64 lateralOffsetM{ 0.0 };      // Lateral distance from centerline (+/- 1.83m for Baron 58)
    f64 maxFuelCapacityKg{ 144.5 }; // Usable fuel capacity
    f64 fuelFlowGphCruise{ 8.5 };   // Cruise fuel flow (GPH)
    bool isAerobaticInvertedFuel{ false }; // Unlimited inverted fuel/oil system
};

struct VSpeeds {
    f64 Vso{ 40.0 };                // Stall speed dirty (flaps landing) [kt]
    f64 Vs{ 48.0 };                 // Stall speed clean [kt]
    f64 Vr{ 55.0 };                 // Rotation speed [kt]
    f64 Vx{ 62.0 };                 // Best angle of climb [kt]
    f64 Vy{ 74.0 };                 // Best rate of climb [kt]
    f64 Vfe{ 85.0 };                // Maximum flap extended speed [kt]
    f64 Vno{ 129.0 };               // Maximum structural cruising speed [kt]
    f64 Vne{ 163.0 };               // Never exceed speed [kt]
    f64 Vmc{ 0.0 };                 // Minimum control speed single engine (twin) [kt]
};

struct AircraftMassConfig {
    f64 emptyMassKg{ 757.0 };
    f64 maxTakeoffMassKg{ 1111.0 };
    f64 defaultFuelKg{ 120.0 };
    Math::Mat3d inertiaTensor{
        1285.0, 0.0, 0.0,
        0.0, 1824.0, 0.0,
        0.0, 0.0, 2667.0
    };
    Math::Vec3d emptyCgBody{ 0.0, 0.0, 0.0 };
    Math::Vec3d fuelTankLocationBody{ 0.0, 0.0, -0.70 };
    Math::Vec3d payloadLocationBody{ 0.35, 0.0, 0.10 };
};

struct AircraftConfig {
    AircraftType type{ AircraftType::Cessna172Skyhawk };
    std::string name{ "Cessna 172 Skyhawk" };
    std::string icaoCode{ "C172" };
    AircraftCategory category{ AircraftCategory::SingleEnginePiston };
    std::string glbPath{ "assets/models/cessna172.glb" };

    AircraftGeometryConfig geometry{};
    AeroDerivativesConfig aero{};
    AircraftEngineConfig propulsion{};
    AircraftMassConfig mass{};
    VSpeeds vSpeeds{};

    bool hasRetractableGear{ false };
    f32 chaseCameraDistance{ 12.0f };
    f32 chaseCameraHeight{ 3.2f };
};

[[nodiscard]] constexpr const char* AircraftTypeToString(AircraftType type) noexcept {
    switch (type) {
    case AircraftType::Cessna172Skyhawk: return "Cessna 172 Skyhawk";
    case AircraftType::Baron58:          return "Beechcraft Baron 58";
    case AircraftType::Extra330SC:        return "Extra 330SC";
    }
    return "Unknown Aircraft";
}

[[nodiscard]] constexpr const char* AircraftCategoryToString(AircraftCategory cat) noexcept {
    switch (cat) {
    case AircraftCategory::SingleEnginePiston: return "Single-Engine Piston (SEP)";
    case AircraftCategory::MultiEnginePiston:  return "Multi-Engine Piston (MEP)";
    case AircraftCategory::Aerobatic:          return "Unlimited Aerobatic";
    }
    return "General Aviation";
}

} // namespace LokYS::Aircraft
