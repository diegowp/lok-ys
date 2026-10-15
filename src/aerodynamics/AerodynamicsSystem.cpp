#include "LokYS/aerodynamics/AerodynamicsSystem.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Aerodynamics {

AerodynamicsSystem::AerodynamicsSystem(const AircraftGeometry& geometry)
    : m_geometry(geometry)
{
}

void AerodynamicsSystem::SetConfig(
    const Aircraft::AircraftGeometryConfig& geo,
    const Aircraft::AeroDerivativesConfig& aero,
    bool gearExtended) noexcept
{
    m_geometry.wingAreaM2 = geo.wingAreaM2;
    m_geometry.wingspanM = geo.wingspanM;
    m_geometry.meanChordM = geo.meanChordM;
    m_geometry.aspectRatio = geo.aspectRatio;
    m_geometry.oswaldEfficiency = geo.oswaldEfficiency;

    m_CL0 = aero.CL0;
    m_CLa = aero.CLa;
    m_CLq = aero.CLq;
    m_CLde = aero.CLde;
    m_CLdf = aero.CLdf;
    m_stallAlphaRad = aero.stallAlphaRad;

    m_CD0 = gearExtended ? aero.CD0_gearDown : aero.CD0;
    m_CDdf = aero.CDdf;

    m_CYb = aero.CYb;
    m_CYdr = aero.CYdr;

    m_Clb = aero.Clb;
    m_Clp = aero.Clp;
    m_Clr = aero.Clr;
    m_Clda = aero.Clda;

    m_Cm0 = aero.Cm0;
    m_Cma = aero.Cma;
    m_Cmq = aero.Cmq;
    m_Cmde = aero.Cmde;
    m_Cmdf = aero.Cmdf;

    m_Cnb = aero.Cnb;
    m_Cnr = aero.Cnr;
    m_Cnp = aero.Cnp;
    m_Cndr = aero.Cndr;
    m_criticalMach = aero.criticalMach;
}

AeroOutput AerodynamicsSystem::Evaluate(
    const Math::Vec3d& relativeWindBody,
    const Math::Vec3d& angularVelocityBody,
    const ControlDeflections& controls,
    f64 airDensityKgM3,
    f64 speedOfSoundMps) const noexcept
{
    return Evaluate(relativeWindBody, angularVelocityBody, controls, airDensityKgM3, speedOfSoundMps, 10000.0, 0.0, 1.905, 0.60);
}

AeroOutput AerodynamicsSystem::Evaluate(
    const Math::Vec3d& relativeWindBody,
    const Math::Vec3d& angularVelocityBody,
    const ControlDeflections& controls,
    f64 airDensityKgM3,
    f64 speedOfSoundMps,
    f64 altitudeAglM,
    f64 propThrustNewtons,
    f64 propDiameterM,
    f64 propwashImmersion) const noexcept
{
    AeroOutput out{};

    // Extract airflow kinematic state
    auto aeroState = Math::CoordinateFrames::ComputeAerodynamicState(
        relativeWindBody, airDensityKgM3, speedOfSoundMps);

    out.trueAirspeedMps = aeroState.trueAirspeed;
    out.dynamicPressurePa = aeroState.dynamicPressure;
    out.alphaDeg = Math::Units::RadToDeg(aeroState.alphaRad);
    out.betaDeg = Math::Units::RadToDeg(aeroState.betaRad);
    out.machNumber = (speedOfSoundMps > 1e-3) ? (out.trueAirspeedMps / speedOfSoundMps) : 0.0;

    // --- Phase F2.1: McCormick Ground Effect Formulation ---
    // Normalized height above terrain h/b = max(h_agl, 0.1 * c_bar) / b
    const f64 minGroundHeight = std::max(0.1 * m_geometry.meanChordM, 0.05);
    const f64 hEffective = std::max(altitudeAglM, minGroundHeight);
    const f64 hOverB = hEffective / std::max(m_geometry.wingspanM, 1.0);
    const f64 hOverB2 = hOverB * hOverB;
    // phi(h/b) = 16(h/b)^2 / (1 + 16(h/b)^2)
    const f64 phiGe = std::clamp((16.0 * hOverB2) / (1.0 + 16.0 * hOverB2), 0.05, 1.0);
    out.groundEffectFactor = phiGe;

    // --- Phase F2.2: Rankine-Froude Propwash Impingement ---
    // Actuator disk slipstream velocity V_slip = sqrt(V_inf^2 + 2*T / (rho * A_prop))
    const f64 propRadius = std::max(0.1, propDiameterM * 0.5);
    const f64 diskArea = Math::Constants::PI * propRadius * propRadius;
    const f64 thrustPositive = std::max(0.0, propThrustNewtons);
    const f64 rhoClamped = std::max(0.1, airDensityKgM3);

    const f64 vInf2 = out.trueAirspeedMps * out.trueAirspeedMps;
    const f64 slipstreamVel2 = vInf2 + (2.0 * thrustPositive) / (rhoClamped * diskArea);
    out.slipstreamVelocityMps = std::sqrt(std::max(0.0, slipstreamVel2));

    // Tail dynamic pressure: q_tail = 0.5 * rho * [k_wash * V_slip^2 + (1 - k_wash) * V_inf^2]
    const f64 kWash = std::clamp(propwashImmersion, 0.0, 1.0);
    out.tailDynamicPressurePa = 0.5 * rhoClamped * (kWash * slipstreamVel2 + (1.0 - kWash) * vInf2);

    // Propwash control effectiveness ratio
    const f64 etaTail = (out.dynamicPressurePa > 1e-3)
        ? (out.tailDynamicPressurePa / out.dynamicPressurePa)
        : 1.0;

    // --- Phase F2.4: Prandtl-Glauert Subsonic Compressibility Factor ---
    // Incompressible regime (M < 0.30): beta_comp = 1.0.
    // Subsonic compressible regime (0.30 <= M < 0.85): beta_comp = sqrt(max(0.20, 1 - M^2)).
    const f64 machSq = out.machNumber * out.machNumber;
    const f64 betaComp = (out.machNumber >= 0.30)
        ? std::sqrt(std::max(0.20, 1.0 - machSq))
        : 1.0;

    // If airspeed is near zero, evaluate purely propwash-induced static moments
    if (out.trueAirspeedMps < 0.5) {
        if (thrustPositive > 1.0 && kWash > 1e-3) {
            // Static propwash deflection moments over empennage
            const f64 qTailS = out.tailDynamicPressurePa * m_geometry.wingAreaM2 * kWash;
            out.momentsBody.y = qTailS * m_geometry.meanChordM * (m_Cmde * controls.elevator);
            out.momentsBody.z = qTailS * m_geometry.wingspanM * (m_Cndr * controls.rudder);
        }
        return out;
    }

    const f64 invTwoV = 1.0 / (2.0 * out.trueAirspeedMps);
    const f64 pHat = angularVelocityBody.x * m_geometry.wingspanM * invTwoV;
    const f64 qHat = angularVelocityBody.y * m_geometry.meanChordM * invTwoV;
    const f64 rHat = angularVelocityBody.z * m_geometry.wingspanM * invTwoV;

    // 1. Aerodynamic Coefficients with F2 Corrections
    out.coeffs.CL = ComputeLiftCoefficient(
        aeroState.alphaRad, qHat, controls.elevator, controls.flaps, phiGe, betaComp, out.isStalled);
    out.coeffs.CD = ComputeDragCoefficient(
        aeroState.alphaRad, out.coeffs.CL, controls.flaps, phiGe, out.machNumber);
    out.coeffs.CY = ComputeSideForceCoefficient(
        aeroState.betaRad, controls.rudder);

    out.coeffs.Cl = ComputeRollMomentCoefficient(
        aeroState.betaRad, pHat, rHat, controls.aileron, aeroState.alphaRad, out.trueAirspeedMps);
    out.coeffs.Cm = ComputePitchMomentCoefficient(
        aeroState.alphaRad, qHat, controls.elevator, controls.flaps, phiGe, etaTail, betaComp);
    out.coeffs.Cn = ComputeYawMomentCoefficient(
        aeroState.betaRad, pHat, rHat, controls.rudder, etaTail, aeroState.alphaRad);

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
    f64 phiGe,
    f64 betaComp,
    bool& outStall) const noexcept
{
    // Linear lift slope with compressibility correction
    f64 clLinear = (m_CL0 + (m_CLa * alphaRad) + (m_CLq * qHat)
                  + (m_CLde * elevatorNorm) + (m_CLdf * flapsNorm)) / betaComp;

    // McCormick Ground Effect Lift Increment:
    // Delta CL_ge = CL * [ (1 / phi^(0.25)) - 1 ] * 0.22
    if (phiGe < 0.999) {
        f64 phiFactor = std::pow(phiGe, -0.25) - 1.0;
        f64 deltaClGe = clLinear * std::clamp(phiFactor * 0.22, 0.0, 0.25);
        clLinear += deltaClGe;
    }

    // Stall boundary evaluation
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha <= m_stallAlphaRad) {
        outStall = false;
        return clLinear;
    }

    outStall = true;

    // Post-stall model: blend from peak linear CL down into flat-plate crossflow
    f64 clMax = (m_CL0 + (m_CLa * m_stallAlphaRad) + (m_CLdf * flapsNorm)) / betaComp;
    f64 stallFraction = (absAlpha - m_stallAlphaRad) / (Math::Constants::PI * 0.5 - m_stallAlphaRad);
    stallFraction = std::clamp(stallFraction, 0.0, 1.0);

    // Flat plate lift: CL_flat = sin(2 * alpha)
    f64 clFlat = std::sin(2.0 * alphaRad);

    // Cosine ease-in blend
    f64 blend = 0.5 * (1.0 - std::cos(stallFraction * Math::Constants::PI));
    f64 clPostStall = (1.0 - blend) * std::copysign(clMax, alphaRad) + blend * clFlat;

    return clPostStall + (m_CLq * qHat);
}

f64 AerodynamicsSystem::ComputeDragCoefficient(
    f64 alphaRad,
    f64 CL,
    f64 flapsNorm,
    f64 phiGe,
    f64 mach) const noexcept
{
    // Induced drag attenuated by McCormick ground effect factor phiGe:
    // CDi = (CL^2 / (pi * AR * e)) * phi(h/b)
    const f64 piARe = Math::Constants::PI * m_geometry.aspectRatio * m_geometry.oswaldEfficiency;
    const f64 CDi = ((CL * CL) / piARe) * phiGe;

    // Parasite drag with flap penalty
    f64 CD_base = m_CD0 + (m_CDdf * flapsNorm) + CDi;

    // High alpha separation drag penalty
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha > m_stallAlphaRad) {
        f64 excessAlpha = absAlpha - m_stallAlphaRad;
        f64 cdSeparation = 1.8 * (std::sin(excessAlpha) * std::sin(excessAlpha));
        CD_base += cdSeparation;
    }

    // Subsonic wave drag divergence onset beyond critical Mach
    if (mach > m_criticalMach) {
        f64 deltaM = mach - m_criticalMach;
        f64 cdWave = 20.0 * (deltaM * deltaM * deltaM * deltaM);
        CD_base += cdWave;
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
    f64 aileronNorm,
    f64 alphaRad,
    [[maybe_unused]] f64 tasMps) const noexcept
{
    f64 Cl_linear = (m_Clb * betaRad) + (m_Clp * pHat) + (m_Clr * rHat) + (m_Clda * aileronNorm);

    // Phase F2.3: Coupled Asymmetric Post-Stall & Autorotation (Spin Dynamics)
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha > m_stallAlphaRad && std::abs(pHat) > 1e-4) {
        // In the stalled regime, the descending wing has alpha > stall (lift drops)
        // while the rising wing has lower alpha (lift higher), creating positive feedback autorotation.
        f64 stallDepth = std::clamp((absAlpha - m_stallAlphaRad) / 0.15, 0.0, 1.0);
        f64 Cl_spin = -m_Clp * pHat * 1.65 * stallDepth;
        Cl_linear += Cl_spin;
    }

    return Cl_linear;
}

f64 AerodynamicsSystem::ComputePitchMomentCoefficient(
    f64 alphaRad,
    f64 qHat,
    f64 elevatorNorm,
    f64 flapsNorm,
    f64 phiGe,
    f64 etaTail,
    f64 betaComp) const noexcept
{
    // Compressibility scaled static pitch stability
    f64 Cm_base = (m_Cm0 + (m_Cma * alphaRad)) / betaComp + (m_Cmq * qHat) + (m_Cmdf * flapsNorm);

    // Elevator effectiveness scaled by propwash dynamic pressure ratio etaTail
    f64 Cm_elev = m_Cmde * elevatorNorm * std::clamp(etaTail, 0.5, 3.5);

    // Downwash suppression at horizontal tail in ground effect causing nose-down flare pitching moment
    f64 Cm_flare = -0.05 * std::abs(alphaRad) * (1.0 - phiGe);

    return Cm_base + Cm_elev + Cm_flare;
}

f64 AerodynamicsSystem::ComputeYawMomentCoefficient(
    f64 betaRad,
    f64 pHat,
    f64 rHat,
    f64 rudderNorm,
    f64 etaTail,
    f64 alphaRad) const noexcept
{
    // Post-stall yaw damping attenuation
    f64 cnrEff = m_Cnr;
    f64 absAlpha = std::abs(alphaRad);
    if (absAlpha > m_stallAlphaRad) {
        f64 excess = absAlpha - m_stallAlphaRad;
        cnrEff *= std::max(0.15, 1.0 - 2.5 * excess);
    }

    f64 Cn_base = (m_Cnb * betaRad) + (cnrEff * rHat) + (m_Cnp * pHat);

    // Rudder effectiveness scaled by propwash dynamic pressure ratio
    f64 Cn_rudder = m_Cndr * rudderNorm * std::clamp(etaTail, 0.5, 3.5);

    return Cn_base + Cn_rudder;
}

} // namespace LokYS::Aerodynamics
