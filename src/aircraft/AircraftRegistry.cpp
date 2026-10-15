#include "LokYS/aircraft/AircraftRegistry.hpp"
#include <cassert>

namespace LokYS::Aircraft {

AircraftRegistry::AircraftRegistry() {
    InitializeConfigurations();
}

void AircraftRegistry::InitializeConfigurations() {
    // -------------------------------------------------------------
    // 1. Cessna 172 Skyhawk (Single-Engine Piston GA Trainer)
    // -------------------------------------------------------------
    AircraftConfig c172{};
    c172.type = AircraftType::Cessna172Skyhawk;
    c172.name = "Cessna 172 Skyhawk";
    c172.icaoCode = "C172";
    c172.category = AircraftCategory::SingleEnginePiston;
    c172.glbPath = "assets/models/cessna172.glb";

    c172.geometry.wingAreaM2 = 16.165;
    c172.geometry.wingspanM = 11.0;
    c172.geometry.meanChordM = 1.493;
    c172.geometry.aspectRatio = 7.49;
    c172.geometry.oswaldEfficiency = 0.75;

    c172.aero.CL0 = 0.307;
    c172.aero.CLa = 4.58;
    c172.aero.CLq = 3.8;
    c172.aero.CLde = 0.43;
    c172.aero.CLdf = 0.45;
    c172.aero.stallAlphaRad = 0.279;     // 16.0 deg
    c172.aero.stallAlphaNegRad = -0.279;

    c172.aero.CD0 = 0.027;
    c172.aero.CD0_gearDown = 0.027;      // Fixed landing gear
    c172.aero.CDdf = 0.035;

    c172.aero.CYb = -0.31;
    c172.aero.CYdr = 0.187;

    c172.aero.Clb = -0.089;
    c172.aero.Clp = -0.47;
    c172.aero.Clr = 0.096;
    c172.aero.Clda = 0.165;

    c172.aero.Cm0 = 0.04;
    c172.aero.Cma = -0.65;
    c172.aero.Cmq = -12.4;
    c172.aero.Cmde = -1.122;
    c172.aero.Cmdf = -0.05;

    c172.aero.Cnb = 0.065;
    c172.aero.Cnr = -0.099;
    c172.aero.Cnp = -0.030;
    c172.aero.Cndr = -0.074;

    c172.propulsion.engineCount = 1;
    c172.propulsion.horsepowerPerEngine = 160.0;
    c172.propulsion.maxRatedRpm = 2700.0;
    c172.propulsion.idleRpm = 680.0;
    c172.propulsion.propDiameterM = 1.905;
    c172.propulsion.propInertiaKgM2 = 0.38;
    c172.propulsion.lateralOffsetM = 0.0;
    c172.propulsion.maxFuelCapacityKg = 144.5;
    c172.propulsion.fuelFlowGphCruise = 8.5;
    c172.propulsion.isAerobaticInvertedFuel = false;

    c172.mass.emptyMassKg = 757.0;
    c172.mass.maxTakeoffMassKg = 1111.0;
    c172.mass.defaultFuelKg = 120.0;
    c172.mass.inertiaTensor = Math::Mat3d(
        1285.0, 0.0, 0.0,
        0.0, 1824.0, 0.0,
        0.0, 0.0, 2667.0
    );
    c172.mass.emptyCgBody = { 0.0, 0.0, 0.0 };
    c172.mass.fuelTankLocationBody = { 0.0, 0.0, -0.70 };
    c172.mass.payloadLocationBody = { 0.35, 0.0, 0.10 };

    c172.vSpeeds.Vso = 40.0;
    c172.vSpeeds.Vs = 48.0;
    c172.vSpeeds.Vr = 55.0;
    c172.vSpeeds.Vx = 62.0;
    c172.vSpeeds.Vy = 74.0;
    c172.vSpeeds.Vfe = 85.0;
    c172.vSpeeds.Vno = 129.0;
    c172.vSpeeds.Vne = 163.0;
    c172.vSpeeds.Vmc = 0.0;

    c172.hasRetractableGear = false;
    c172.chaseCameraDistance = 12.0f;
    c172.chaseCameraHeight = 3.2f;

    m_configs[static_cast<size_t>(AircraftType::Cessna172Skyhawk)] = c172;

    // -------------------------------------------------------------
    // 2. Beechcraft Baron 58 (Twin-Engine High-Performance Piston)
    // -------------------------------------------------------------
    AircraftConfig b58{};
    b58.type = AircraftType::Baron58;
    b58.name = "Beechcraft Baron 58";
    b58.icaoCode = "B58";
    b58.category = AircraftCategory::MultiEnginePiston;
    b58.glbPath = "assets/models/baron58.glb";

    b58.geometry.wingAreaM2 = 18.51;
    b58.geometry.wingspanM = 11.53;
    b58.geometry.meanChordM = 1.60;
    b58.geometry.aspectRatio = 7.18;
    b58.geometry.oswaldEfficiency = 0.80;

    b58.aero.CL0 = 0.250;
    b58.aero.CLa = 4.85;
    b58.aero.CLq = 4.2;
    b58.aero.CLde = 0.48;
    b58.aero.CLdf = 0.50;
    b58.aero.stallAlphaRad = 0.262;      // 15.0 deg
    b58.aero.stallAlphaNegRad = -0.262;

    b58.aero.CD0 = 0.024;               // Clean (gear up)
    b58.aero.CD0_gearDown = 0.038;      // Retractable gear down (+0.014 parasite drag)
    b58.aero.CDdf = 0.040;

    b58.aero.CYb = -0.45;
    b58.aero.CYdr = 0.220;

    b58.aero.Clb = -0.110;
    b58.aero.Clp = -0.52;
    b58.aero.Clr = 0.120;
    b58.aero.Clda = 0.180;

    b58.aero.Cm0 = 0.03;
    b58.aero.Cma = -0.85;
    b58.aero.Cmq = -15.2;
    b58.aero.Cmde = -1.250;
    b58.aero.Cmdf = -0.06;

    b58.aero.Cnb = 0.120;               // Robust weathercocking for twin stability
    b58.aero.Cnr = -0.150;
    b58.aero.Cnp = -0.035;
    b58.aero.Cndr = -0.095;

    b58.propulsion.engineCount = 2;
    b58.propulsion.horsepowerPerEngine = 300.0; // 2x Continental IO-550 (600 hp total)
    b58.propulsion.maxRatedRpm = 2700.0;
    b58.propulsion.idleRpm = 700.0;
    b58.propulsion.propDiameterM = 1.98;
    b58.propulsion.propInertiaKgM2 = 0.45;
    b58.propulsion.lateralOffsetM = 1.83; // Engine nacelles at Y = +/- 1.83m
    b58.propulsion.maxFuelCapacityKg = 450.0; // ~166 usable gallons
    b58.propulsion.fuelFlowGphCruise = 28.0;  // Twin engine combined cruise fuel burn
    b58.propulsion.isAerobaticInvertedFuel = false;

    b58.mass.emptyMassKg = 1814.0;
    b58.mass.maxTakeoffMassKg = 2495.0;
    b58.mass.defaultFuelKg = 300.0;
    b58.mass.inertiaTensor = Math::Mat3d(
        6100.0, 0.0, 0.0,
        0.0, 7800.0, 0.0,
        0.0, 0.0, 11500.0
    );
    b58.mass.emptyCgBody = { 0.0, 0.0, 0.0 };
    b58.mass.fuelTankLocationBody = { 0.0, 0.0, 0.10 };
    b58.mass.payloadLocationBody = { 0.45, 0.0, 0.15 };

    b58.vSpeeds.Vso = 74.0;
    b58.vSpeeds.Vs = 84.0;
    b58.vSpeeds.Vmc = 84.0;              // Critical minimum control speed
    b58.vSpeeds.Vr = 85.0;
    b58.vSpeeds.Vx = 92.0;
    b58.vSpeeds.Vy = 105.0;
    b58.vSpeeds.Vfe = 152.0;
    b58.vSpeeds.Vno = 195.0;
    b58.vSpeeds.Vne = 223.0;

    b58.hasRetractableGear = true;
    b58.chaseCameraDistance = 15.0f;
    b58.chaseCameraHeight = 3.8f;

    m_configs[static_cast<size_t>(AircraftType::Baron58)] = b58;

    // -------------------------------------------------------------
    // 3. Extra 330SC (Unlimited Aerobatic Competition Monoplane)
    // -------------------------------------------------------------
    AircraftConfig e330{};
    e330.type = AircraftType::Extra330SC;
    e330.name = "Extra 330SC";
    e330.icaoCode = "E330";
    e330.category = AircraftCategory::Aerobatic;
    e330.glbPath = "assets/models/extra330.glb";

    e330.geometry.wingAreaM2 = 10.70;
    e330.geometry.wingspanM = 7.50;
    e330.geometry.meanChordM = 1.45;
    e330.geometry.aspectRatio = 5.25;
    e330.geometry.oswaldEfficiency = 0.85;

    // Symmetrical NACA 0012 airfoil: zero lift at zero alpha
    e330.aero.CL0 = 0.000;
    e330.aero.CLa = 5.20;
    e330.aero.CLq = 4.8;
    e330.aero.CLde = 0.65;
    e330.aero.CLdf = 0.00;              // Aerobatic aircraft without high-lift flaps
    e330.aero.stallAlphaRad = 0.384;     // 22.0 deg high-alpha envelope
    e330.aero.stallAlphaNegRad = -0.384; // Identical inverted stall angle

    e330.aero.CD0 = 0.022;               // Streamlined composite fuselage
    e330.aero.CD0_gearDown = 0.022;      // Fixed lightweight gear with wheel fairings
    e330.aero.CDdf = 0.00;

    e330.aero.CYb = -0.55;
    e330.aero.CYdr = 0.350;             // Oversized rudder for knife-edge flight

    e330.aero.Clb = 0.000;              // Zero dihedral: zero roll coupling with beta
    e330.aero.Clp = -0.38;              // Low roll damping -> blistering roll rate
    e330.aero.Clr = 0.050;
    e330.aero.Clda = 0.320;             // Full-span ailerons -> roll rate > 400 deg/s

    e330.aero.Cm0 = 0.00;               // Perfectly symmetrical pitch balance
    e330.aero.Cma = -0.55;
    e330.aero.Cmq = -8.5;
    e330.aero.Cmde = -1.550;            // Ultra-powerful elevator authority
    e330.aero.Cmdf = 0.00;

    e330.aero.Cnb = 0.080;
    e330.aero.Cnr = -0.070;
    e330.aero.Cnp = -0.020;
    e330.aero.Cndr = -0.140;

    e330.propulsion.engineCount = 1;
    e330.propulsion.horsepowerPerEngine = 315.0; // Lycoming AEIO-580
    e330.propulsion.maxRatedRpm = 2700.0;
    e330.propulsion.idleRpm = 750.0;
    e330.propulsion.propDiameterM = 1.95;
    e330.propulsion.propInertiaKgM2 = 0.30;
    e330.propulsion.lateralOffsetM = 0.0;
    e330.propulsion.maxFuelCapacityKg = 120.0;
    e330.propulsion.fuelFlowGphCruise = 16.5;
    e330.propulsion.isAerobaticInvertedFuel = true; // Unlimited inverted flight

    e330.mass.emptyMassKg = 650.0;
    e330.mass.maxTakeoffMassKg = 860.0;
    e330.mass.defaultFuelKg = 80.0;
    e330.mass.inertiaTensor = Math::Mat3d(
        520.0, 0.0, 0.0,
        0.0, 680.0, 0.0,
        0.0, 0.0, 1050.0
    );
    e330.mass.emptyCgBody = { 0.0, 0.0, 0.0 };
    e330.mass.fuelTankLocationBody = { -0.10, 0.0, 0.0 };
    e330.mass.payloadLocationBody = { -0.30, 0.0, 0.05 };

    e330.vSpeeds.Vso = 58.0;
    e330.vSpeeds.Vs = 60.0;
    e330.vSpeeds.Vmc = 0.0;
    e330.vSpeeds.Vr = 62.0;
    e330.vSpeeds.Vx = 78.0;
    e330.vSpeeds.Vy = 95.0;
    e330.vSpeeds.Vfe = 85.0;
    e330.vSpeeds.Vno = 160.0;
    e330.vSpeeds.Vne = 220.0;

    e330.hasRetractableGear = false;
    e330.chaseCameraDistance = 10.0f;
    e330.chaseCameraHeight = 2.5f;

    m_configs[static_cast<size_t>(AircraftType::Extra330SC)] = e330;
}

const AircraftConfig& AircraftRegistry::GetConfig(AircraftType type) const noexcept {
    size_t idx = static_cast<size_t>(type);
    assert(idx < m_configs.size());
    return m_configs[idx];
}

void AircraftRegistry::SetActiveType(AircraftType type) noexcept {
    m_activeType = type;
    // Set gear down by default when switching
    m_gearExtended = true;
}

AircraftType AircraftRegistry::CycleAircraft() noexcept {
    switch (m_activeType) {
    case AircraftType::Cessna172Skyhawk:
        SetActiveType(AircraftType::Baron58);
        break;
    case AircraftType::Baron58:
        SetActiveType(AircraftType::Extra330SC);
        break;
    case AircraftType::Extra330SC:
        SetActiveType(AircraftType::Cessna172Skyhawk);
        break;
    }
    return m_activeType;
}

void AircraftRegistry::ToggleGear() noexcept {
    const auto& config = GetActiveConfig();
    if (config.hasRetractableGear) {
        m_gearExtended = !m_gearExtended;
    } else {
        m_gearExtended = true; // Fixed gear is always extended
    }
}

void AircraftRegistry::ComputeMultiEngineForces(
    const AircraftConfig& config,
    f64 thrustLeftN,
    f64 thrustRightN,
    Math::Vec3d& outTotalForceBody,
    Math::Vec3d& outAsymMomentBody) noexcept
{
    if (config.propulsion.engineCount <= 1) {
        // Single engine along centerline
        outTotalForceBody = Math::Vec3d(thrustLeftN, 0.0, 0.0);
        outAsymMomentBody = Math::Vec3d(0.0, 0.0, 0.0);
        return;
    }

    // Twin engine configuration
    f64 totalThrust = thrustLeftN + thrustRightN;
    outTotalForceBody = Math::Vec3d(totalThrust, 0.0, 0.0);

    // Asymmetric thrust produces a yaw moment around Body Z (positive right yaw)
    // Left engine (+X thrust at -Y position): produces positive yaw moment (+Z)
    // Right engine (+X thrust at +Y position): produces negative yaw moment (-Z)
    f64 yOffset = config.propulsion.lateralOffsetM;
    f64 yawMoment = (thrustLeftN - thrustRightN) * yOffset;

    outAsymMomentBody = Math::Vec3d(0.0, 0.0, yawMoment);
}

} // namespace LokYS::Aircraft
