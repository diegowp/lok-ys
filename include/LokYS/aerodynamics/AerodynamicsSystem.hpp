#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include "LokYS/math/Units.hpp"

namespace LokYS::Aerodynamics {

struct AircraftGeometry {
    f64 wingAreaM2{ 16.165 };       // S: Wing reference area (m^2) [C172: ~174 sq ft = 16.165 m^2]
    f64 wingspanM{ 11.0 };          // b: Wingspan (m) [C172: 36.1 ft = 11.0 m]
    f64 meanChordM{ 1.493 };        // c_bar: Mean aerodynamic chord (m) [C172: 4.9 ft = 1.493 m]
    f64 aspectRatio{ 7.49 };        // AR = b^2 / S
    f64 oswaldEfficiency{ 0.75 };    // e: Oswald span efficiency factor
};

struct ControlDeflections {
    f64 elevator{ 0.0 }; // delta_e: [-1.0, 1.0] (-1 nose down, +1 nose up)
    f64 aileron{ 0.0 };  // delta_a: [-1.0, 1.0] (-1 left bank, +1 right bank)
    f64 rudder{ 0.0 };   // delta_r: [-1.0, 1.0] (-1 left yaw, +1 right yaw)
    f64 flaps{ 0.0 };    // delta_f: [0.0, 1.0] (0 to 30 deg normalized)
};

struct AeroCoefficients {
    f64 CL{ 0.0 }; // Lift coefficient
    f64 CD{ 0.0 }; // Drag coefficient
    f64 CY{ 0.0 }; // Sideforce coefficient
    f64 Cl{ 0.0 }; // Roll moment coefficient
    f64 Cm{ 0.0 }; // Pitch moment coefficient
    f64 Cn{ 0.0 }; // Yaw moment coefficient
};

struct AeroOutput {
    AeroCoefficients coeffs{};
    Math::Vec3d forcesBody{ 0.0, 0.0, 0.0 };   // Total aerodynamic force in Body frame (N)
    Math::Vec3d momentsBody{ 0.0, 0.0, 0.0 };  // Total aerodynamic moment in Body frame (N*m)
    f64 liftNewtons{ 0.0 };
    f64 dragNewtons{ 0.0 };
    f64 sideForceNewtons{ 0.0 };
    f64 trueAirspeedMps{ 0.0 };
    f64 dynamicPressurePa{ 0.0 };
    f64 alphaDeg{ 0.0 };
    f64 betaDeg{ 0.0 };
    bool isStalled{ false };
};

class AerodynamicsSystem {
public:
    explicit AerodynamicsSystem(const AircraftGeometry& geometry = {});

    void SetGeometry(const AircraftGeometry& geometry) noexcept { m_geometry = geometry; }
    [[nodiscard]] const AircraftGeometry& GetGeometry() const noexcept { return m_geometry; }

    // Evaluates aerodynamic forces and moments from airframe state
    [[nodiscard]] AeroOutput Evaluate(
        const Math::Vec3d& relativeWindBody,
        const Math::Vec3d& angularVelocityBody,
        const ControlDeflections& controls,
        f64 airDensityKgM3 = Math::Constants::SEA_LEVEL_DENSITY,
        f64 speedOfSoundMps = 340.294) const noexcept;

private:
    [[nodiscard]] f64 ComputeLiftCoefficient(f64 alphaRad, f64 qHat, f64 elevatorNorm, f64 flapsNorm, bool& outStall) const noexcept;
    [[nodiscard]] f64 ComputeDragCoefficient(f64 alphaRad, f64 CL, f64 flapsNorm) const noexcept;
    [[nodiscard]] f64 ComputeSideForceCoefficient(f64 betaRad, f64 rudderNorm) const noexcept;
    [[nodiscard]] f64 ComputeRollMomentCoefficient(f64 betaRad, f64 pHat, f64 rHat, f64 aileronNorm) const noexcept;
    [[nodiscard]] f64 ComputePitchMomentCoefficient(f64 alphaRad, f64 qHat, f64 elevatorNorm, f64 flapsNorm) const noexcept;
    [[nodiscard]] f64 ComputeYawMomentCoefficient(f64 betaRad, f64 pHat, f64 rHat, f64 rudderNorm) const noexcept;

    AircraftGeometry m_geometry{};

    // --- Cessna 172 Aero Derivatives (NASA CR-2144 / Roskam Airplane Flight Dynamics) ---
    f64 m_CL0{ 0.307 };         // Zero-alpha lift coefficient
    f64 m_CLa{ 4.58 };          // Lift curve slope per radian (~0.08 / deg)
    f64 m_CLq{ 3.8 };           // Pitch rate lift derivative
    f64 m_CLde{ 0.43 };         // Elevator lift effectiveness
    f64 m_CLdf{ 0.45 };         // Flaps lift increment at full extension
    f64 m_stallAlphaRad{ 0.279 }; // 16.0 degrees stall angle of attack

    f64 m_CD0{ 0.027 };         // Parasite drag coefficient (clean)
    f64 m_CDdf{ 0.035 };        // Flap parasite drag increment

    f64 m_CYb{ -0.31 };         // Side force due to sideslip
    f64 m_CYdr{ 0.187 };        // Rudder side force effectiveness

    f64 m_Clb{ -0.089 };        // Dihedral effect (roll due to sideslip, stabilizing)
    f64 m_Clp{ -0.47 };         // Roll damping derivative (stabilizing)
    f64 m_Clr{ 0.096 };         // Roll due to yaw rate
    f64 m_Clda{ 0.165 };        // Aileron roll effectiveness

    f64 m_Cm0{ 0.04 };          // Zero-alpha pitching moment
    f64 m_Cma{ -0.65 };         // Longitudinal static stability (pitch stiffness, stabilizing)
    f64 m_Cmq{ -12.4 };         // Pitch damping derivative (stabilizing)
    f64 m_Cmde{ -1.122 };       // Elevator pitch effectiveness (elevator up -> nose up)
    f64 m_Cmdf{ -0.05 };        // Flap pitching moment increment

    f64 m_Cnb{ 0.065 };         // Weathercock directional stability (stabilizing)
    f64 m_Cnr{ -0.099 };        // Yaw damping derivative (stabilizing)
    f64 m_Cnp{ -0.030 };        // Yaw due to roll rate
    f64 m_Cndr{ -0.074 };       // Rudder yaw effectiveness
};

} // namespace LokYS::Aerodynamics
