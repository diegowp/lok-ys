#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Aerodynamics {

AerodynamicsSystem::AerodynamicsSystem(const AircraftGeometry& geometry)
    : m_geometry(geometry)
{
}

AeroOutput AerodynamicsSystem::Evaluate(
    const Math::Vec3d& relativeWindBody,
    const Math::Vec3d& angularVelocityBody,
    const ControlDeflections& controls,
    f64 airDensityKgM3,
    f64 speedOfSoundMps) const noexcept
{
    AeroOutput out{};

    // Extract airflow kinematic state
    auto aeroState = Math::CoordinateFrames::ComputeAerodynamicState(
        relativeWindBody, airDensityKgM3, speedOfSoundMps);

    out.trueAirspeedMps = aeroState.trueAirspeed;
    out.dynamicPressurePa = aeroState.dynamicPressure;
    out.alphaDeg = Math::Units::RadToDeg(aeroState.alphaRad);
    out.betaDeg = Math::Units::RadToDeg(aeroState.betaRad);

    // If airspeed is near zero, aero forces vanish
    if (out.trueAirspeedMps < 0.5) {
        return out;
    }

    const f64 invTwoV = 1.0 / (2.0 * out.trueAirspeedMps);
    const f64 pHat = angularVelocityBody.x * m_geometry.wingspanM * invTwoV;
    const f64 qHat = angularVelocityBody.y * m_geometry.meanChordM * invTwoV;
    const f64 rHat = angularVelocityBody.z * m_geometry.wingspanM * invTwoV;

    // 1. Aerodynamic Coefficients
    out.coeffs.CL = ComputeLiftCoefficient(aeroState.alphaRad, qHat, controls.elevator, controls.flaps, out.isStalled);
    out.coeffs.CD = ComputeDragCoefficient(aeroState.alphaRad, out.coeffs.CL, controls.flaps);
    out.coeffs.CY = ComputeSideForceCoefficient(aeroState.betaRad, controls.rudder);

    out.coeffs.Cl = ComputeRollMomentCoefficient(aeroState.betaRad, pHat, rHat, controls.aileron);
    out.coeffs.Cm = ComputePitchMomentCoefficient(aeroState.alphaRad, qHat, controls.elevator, controls.flaps);
    out.coeffs.Cn = ComputeYawMomentCoefficient(aeroState.betaRad, pHat, rHat, controls.rudder);

    // 2. Dimensional Forces in Wind Frame
    const f64 qS = out.dynamicPressurePa * m_geometry.wingAreaM2;
    out.liftNewtons = qS * out.coeffs.CL;
    out.dragNewtons = qS * out.coeffs.CD;
    out.sideForceNewtons = qS * out.coeffs.CY;

    // 3. Transform Forces from Wind Frame into Body (FRD) Frame
    out.forcesBody = Math::CoordinateFrames::WindForcesToBody(
        out.liftNewtons,
        out.dragNewtons,
        out.sideForceNewtons,
        aeroState.alphaRad,
        aeroState.betaRad);

    // 4. Dimensional Moments in Body Frame
    out.momentsBody.x = qS * m_geometry.wingspanM * out.coeffs.Cl;   // Roll Moment (N*m)
    out.momentsBody.y = qS * m_geometry.meanChordM * out.coeffs.Cm;  // Pitch Moment (N*m)
    out.momentsBody.z = qS * m_geometry.wingspanM * out.coeffs.Cn;   // Yaw Moment (N*m)

    return out;
}

f64 AerodynamicsSystem::ComputeLiftCoefficient(
    f64 alphaRad,
    f64 qHat,
    f64 elevatorNorm,
    f64 flapsNorm,
    bool& outStall) const noexcept
{
    // Linear lift slope
    f64 clLinear = m_CL0 + (m_CLa * alphaRad) + (m_CLq * qHat)
                 + (m_CLde * elevatorNorm) + (m_CLdf * flapsNorm);

    // Stall boundary evaluation
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha <= m_stallAlphaRad) {
        outStall = false;
        return clLinear;
    }

    outStall = true;

    // Post-stall model: blend from peak linear CL down into flat-plate crossflow
    f64 clMax = m_CL0 + (m_CLa * m_stallAlphaRad) + (m_CLdf * flapsNorm);
    f64 stallFraction = (absAlpha - m_stallAlphaRad) / (Math::Constants::PI * 0.5 - m_stallAlphaRad);
    stallFraction = std::clamp(stallFraction, 0.0, 1.0);

    // Flat plate lift: CL_flat = 2 * sin(alpha) * cos(alpha) = sin(2 * alpha)
    f64 clFlat = std::sin(2.0 * alphaRad);

    // Cosine ease-in blend
    f64 blend = 0.5 * (1.0 - std::cos(stallFraction * Math::Constants::PI));
    f64 clPostStall = (1.0 - blend) * std::copysign(clMax, alphaRad) + blend * clFlat;

    return clPostStall + (m_CLq * qHat);
}

f64 AerodynamicsSystem::ComputeDragCoefficient(
    f64 alphaRad,
    f64 CL,
    f64 flapsNorm) const noexcept
{
    // Induced drag: CDi = CL^2 / (pi * AR * e)
    const f64 piARe = Math::Constants::PI * m_geometry.aspectRatio * m_geometry.oswaldEfficiency;
    const f64 CDi = (CL * CL) / piARe;

    // Parasite drag with flap penalty
    f64 CD_base = m_CD0 + (m_CDdf * flapsNorm) + CDi;

    // High alpha separation drag penalty
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha > m_stallAlphaRad) {
        f64 excessAlpha = absAlpha - m_stallAlphaRad;
        // Flat plate drag growth: CD_sep ~ 2.0 * sin^2(alpha)
        f64 cdSeparation = 1.8 * (std::sin(excessAlpha) * std::sin(excessAlpha));
        CD_base += cdSeparation;
    }

    return std::max(0.005, CD_base);
}

f64 AerodynamicsSystem::ComputeSideForceCoefficient(
    f64 betaRad,
    f64 rudderNorm) const noexcept
{
    return (m_CYb * betaRad) + (m_CYdr * rudderNorm);
}

f64 AerodynamicsSystem::ComputeRollMomentCoefficient(
    f64 betaRad,
    f64 pHat,
    f64 rHat,
    f64 aileronNorm) const noexcept
{
    return (m_Clb * betaRad) + (m_Clp * pHat) + (m_Clr * rHat) + (m_Clda * aileronNorm);
}

f64 AerodynamicsSystem::ComputePitchMomentCoefficient(
    f64 alphaRad,
    f64 qHat,
    f64 elevatorNorm,
    f64 flapsNorm) const noexcept
{
    return m_Cm0 + (m_Cma * alphaRad) + (m_Cmq * qHat) + (m_Cmde * elevatorNorm) + (m_Cmdf * flapsNorm);
}

f64 AerodynamicsSystem::ComputeYawMomentCoefficient(
    f64 betaRad,
    f64 pHat,
    f64 rHat,
    f64 rudderNorm) const noexcept
{
    return (m_Cnb * betaRad) + (m_Cnr * rHat) + (m_Cnp * pHat) + (m_Cndr * rudderNorm);
}

} // namespace LokYS::Aerodynamics
