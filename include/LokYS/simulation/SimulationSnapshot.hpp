#pragma once

#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/math/Units.hpp"

namespace LokYS::Simulation {

struct SimulationSnapshot {
    u64 tickNumber{ 0 };
    f64 simTimeSec{ 0.0 };

    Physics::RigidBodyState bodyState{};

    // Derived Flight Telemetry
    f64 yawDeg{ 0.0 };
    f64 pitchDeg{ 0.0 };
    f64 rollDeg{ 0.0 };

    f64 airspeedMps{ 0.0 };
    f64 airspeedKnots{ 0.0 };

    f64 altitudeMeters{ 0.0 }; // h = -Z_NED
    f64 altitudeFeet{ 0.0 };
    f64 verticalSpeedFpm{ 0.0 }; // VSI = -V_D * 196.85

    [[nodiscard]] static SimulationSnapshot CreateFromRigidBody(
        u64 tick,
        f64 simTime,
        const Physics::RigidBodyState& state) noexcept
    {
        SimulationSnapshot snap{};
        snap.tickNumber = tick;
        snap.simTimeSec = simTime;
        snap.bodyState = state;

        // Derive Euler angles
        f64 yRad = 0.0, pRad = 0.0, rRad = 0.0;
        state.attitude.ToEuler(yRad, pRad, rRad);
        snap.yawDeg = Math::Units::RadToDeg(yRad);
        snap.pitchDeg = Math::Units::RadToDeg(pRad);
        snap.rollDeg = Math::Units::RadToDeg(rRad);

        // Derive airspeeds
        snap.airspeedMps = state.velocityBody.Length();
        snap.airspeedKnots = Math::Units::MpsToKnots(snap.airspeedMps);

        // Derive altitude (-Z_NED)
        snap.altitudeMeters = -state.positionNed.z;
        snap.altitudeFeet = Math::Units::MetersToFeet(snap.altitudeMeters);

        // Derive vertical speed (NED V_D is positive downward, so V_up = -V_D)
        Math::Vec3d vNed = state.attitude.Rotate(state.velocityBody);
        f64 vUpMps = -vNed.z;
        snap.verticalSpeedFpm = Math::Units::MpsToFeetPerMinute(vUpMps);

        return snap;
    }

    [[nodiscard]] static SimulationSnapshot Interpolate(
        const SimulationSnapshot& a,
        const SimulationSnapshot& b,
        f64 alpha) noexcept
    {
        SimulationSnapshot res{};
        res.tickNumber = b.tickNumber;
        res.simTimeSec = a.simTimeSec + (b.simTimeSec - a.simTimeSec) * alpha;
        res.bodyState = Physics::RigidBodyState::Interpolate(a.bodyState, b.bodyState, alpha);

        f64 yRad = 0.0, pRad = 0.0, rRad = 0.0;
        res.bodyState.attitude.ToEuler(yRad, pRad, rRad);
        res.yawDeg = Math::Units::RadToDeg(yRad);
        res.pitchDeg = Math::Units::RadToDeg(pRad);
        res.rollDeg = Math::Units::RadToDeg(rRad);

        res.airspeedMps = res.bodyState.velocityBody.Length();
        res.airspeedKnots = Math::Units::MpsToKnots(res.airspeedMps);
        res.altitudeMeters = -res.bodyState.positionNed.z;
        res.altitudeFeet = Math::Units::MetersToFeet(res.altitudeMeters);

        Math::Vec3d vNed = res.bodyState.attitude.Rotate(res.bodyState.velocityBody);
        res.verticalSpeedFpm = Math::Units::MpsToFeetPerMinute(-vNed.z);

        return res;
    }
};

} // namespace LokYS::Simulation
