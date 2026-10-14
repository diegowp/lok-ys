#include "LokYS/physics/GroundReactions.hpp"
#include "LokYS/math/Units.hpp"
#include <cmath>
#include <algorithm>

namespace LokYS::Physics {

GroundReactionsSystem::GroundReactionsSystem(const AircraftLandingGearConfig& config)
    : m_config(config) {}

GearContactState GroundReactionsSystem::EvaluateStrut(
    const StrutProperties& strut,
    const RigidBodyState& state,
    f64 steerAngleRad,
    f64 brakeFactor,
    f64 terrainElevationNed) const noexcept
{
    GearContactState out{};
    out.id = strut.id;

    // 1. Mount point in Navigation (NED) frame
    const Math::Vec3d mountNed = state.positionNed + state.attitude.Rotate(strut.mountPointBody);

    // 2. Uncompressed tip point in NED frame
    const Math::Vec3d uncompressedStrutVectorBody{ 0.0, 0.0, strut.uncompressedLength };
    const Math::Vec3d tipNed = mountNed + state.attitude.Rotate(uncompressedStrutVectorBody);

    // 3. Compression delta in NED frame (NED +Z is downwards)
    f64 delta = tipNed.z - terrainElevationNed;

    if (delta <= 0.0) {
        // Airborne - no contact with ground
        out.inContact = false;
        return out;
    }

    out.inContact = true;
    out.compression = delta;

    // Clamped compression for contact point placement
    f64 clampedDelta = std::min(delta, strut.maxStroke * 1.35);

    // 4. Contact point in Body frame
    out.contactPointBody = strut.mountPointBody + Math::Vec3d{ 0.0, 0.0, strut.uncompressedLength - clampedDelta };

    // 5. Contact point velocity in Body and NED frames
    // V_contact = V_body + omega x r_contact
    const Math::Vec3d vContactBody = state.velocityBody + state.angularVelocityBody.Cross(out.contactPointBody);
    const Math::Vec3d vContactNed = state.attitude.Rotate(vContactBody);

    // Rate of compression: downward velocity into ground
    out.compressionRate = vContactNed.z;

    // 6. Suspension Normal Reaction Force
    // Progressive spring curve with bottoming out resistance
    f64 strokeRatio = std::clamp(delta / strut.maxStroke, 0.0, 1.5);
    f64 springForce = strut.springStiffness * delta * (1.0 + 0.6 * strokeRatio);

    // Hydraulic damping (higher on compression to dissipate energy, lower on rebound)
    f64 dampForce = (out.compressionRate > 0.0)
        ? (strut.dampingCompression * out.compressionRate)
        : (strut.dampingRebound * out.compressionRate);

    out.normalForceNewtons = std::max(0.0, springForce + dampForce);

    // Ground reaction normal force in NED points directly upward (-Z)
    const Math::Vec3d normalForceNed{ 0.0, 0.0, -out.normalForceNewtons };
    const Math::Vec3d normalForceBody = state.attitude.Unrotate(normalForceNed);

    // 7. Tire Kinematics & Wheel Friction
    f64 steerAngle = strut.isSteerable ? steerAngleRad : 0.0;
    f64 cosSteer = std::cos(steerAngle);
    f64 sinSteer = std::sin(steerAngle);

    // Rotate contact velocity into wheel plane
    f64 vWheelX = vContactBody.x * cosSteer + vContactBody.y * sinSteer;
    f64 vWheelY = -vContactBody.x * sinSteer + vContactBody.y * cosSteer;

    // Longitudinal friction coefficient (rolling resistance + brakes)
    f64 muLong = strut.rollingFrictionCoeff;
    if (strut.hasBrake) {
        muLong += strut.brakingFrictionCoeff * std::clamp(brakeFactor, 0.0, 1.0);
    }

    // Smooth tanh friction ramp near zero speed to eliminate numerical buzz/jitter
    f64 fWheelX = -out.normalForceNewtons * muLong * std::tanh(vWheelX * 12.0);
    out.frictionForceNewtons = fWheelX;

    // Lateral scrub friction (cornering stiffness and peak lateral friction limit)
    f64 absVx = std::abs(vWheelX);
    f64 slipAngleRad = std::atan2(vWheelY, std::max(0.15, absVx));
    out.slipAngleDeg = Math::Units::RadToDeg(slipAngleRad);

    f64 corneringForce = -strut.corneringStiffness * slipAngleRad;
    f64 maxLateralFriction = out.normalForceNewtons * 0.75; // 0.75 peak dry asphalt friction
    f64 fWheelY = std::clamp(corneringForce, -maxLateralFriction, maxLateralFriction);

    // Smooth lateral force dropoff when stationary
    f64 totalContactSpeed = vContactBody.Length();
    fWheelY *= std::tanh(totalContactSpeed * 8.0);
    out.lateralForceNewtons = fWheelY;

    // Rotate wheel plane forces back into Body frame
    f64 fBodyX = fWheelX * cosSteer - fWheelY * sinSteer;
    f64 fBodyY = fWheelX * sinSteer + fWheelY * cosSteer;

    // Total force for this gear in Body frame
    out.forceBody = normalForceBody + Math::Vec3d{ fBodyX, fBodyY, 0.0 };

    // Total moment about CG for this gear: M = r x F
    out.momentBody = out.contactPointBody.Cross(out.forceBody);

    return out;
}

GroundReactionsOutput GroundReactionsSystem::Evaluate(
    const RigidBodyState& state,
    f64 rudderInput,
    f64 brakeInput,
    [[maybe_unused]] f64 dt,
    f64 terrainElevationNed) const noexcept
{
    GroundReactionsOutput out{};

    // Steering angle on nose gear: rudder deflection * maxSteerAngle
    f64 steerAngleRad = std::clamp(rudderInput, -1.0, 1.0) * m_config.noseGear.maxSteerAngleRad;

    // 1. Evaluate Nose Gear
    out.gearStates[0] = EvaluateStrut(m_config.noseGear, state, steerAngleRad, 0.0, terrainElevationNed);
    out.noseInContact = out.gearStates[0].inContact;

    // 2. Evaluate Left Main Gear
    out.gearStates[1] = EvaluateStrut(m_config.leftMainGear, state, 0.0, brakeInput, terrainElevationNed);
    out.leftMainInContact = out.gearStates[1].inContact;

    // 3. Evaluate Right Main Gear
    out.gearStates[2] = EvaluateStrut(m_config.rightMainGear, state, 0.0, brakeInput, terrainElevationNed);
    out.rightMainInContact = out.gearStates[2].inContact;

    out.anyGearInContact = out.noseInContact || out.leftMainInContact || out.rightMainInContact;

    // Accumulate forces and moments from all 3 landing gear struts
    for (const auto& gear : out.gearStates) {
        if (gear.inContact) {
            out.totalForceBody += gear.forceBody;
            out.totalMomentBody += gear.momentBody;
            out.totalNormalForce += gear.normalForceNewtons;
        }
    }

    // 4. Tail Bumper Scrape Check (Protects empennage during over-rotation on takeoff/landing)
    const Math::Vec3d tailNed = state.positionNed + state.attitude.Rotate(m_config.tailBumperBody);
    f64 tailDelta = tailNed.z - terrainElevationNed;
    if (tailDelta > 0.0) {
        out.tailstrikeDetected = true;
        const Math::Vec3d vTailBody = state.velocityBody + state.angularVelocityBody.Cross(m_config.tailBumperBody);
        const Math::Vec3d vTailNed = state.attitude.Rotate(vTailBody);

        f64 tailNormal = std::max(0.0, 80000.0 * tailDelta - 5000.0 * vTailNed.z);
        Math::Vec3d fTailNed{ 0.0, 0.0, -tailNormal };
        Math::Vec3d fTailBody = state.attitude.Unrotate(fTailNed);

        // Friction on tail scrape
        fTailBody.x -= tailNormal * 0.40 * std::tanh(vTailBody.x * 10.0);

        out.tailBumperState.inContact = true;
        out.tailBumperState.normalForceNewtons = tailNormal;
        out.tailBumperState.forceBody = fTailBody;
        out.tailBumperState.momentBody = m_config.tailBumperBody.Cross(fTailBody);

        out.totalForceBody += out.tailBumperState.forceBody;
        out.totalMomentBody += out.tailBumperState.momentBody;
    }

    // 5. Wingtip Strike Checks (Left and Right)
    auto CheckWingtip = [&](const Math::Vec3d& wingtipBody) {
        const Math::Vec3d tipNed = state.positionNed + state.attitude.Rotate(wingtipBody);
        f64 tipDelta = tipNed.z - terrainElevationNed;
        if (tipDelta > 0.0) {
            out.wingtipStrikeDetected = true;
            f64 normal = std::max(0.0, 50000.0 * tipDelta);
            Math::Vec3d fTipBody = state.attitude.Unrotate(Math::Vec3d{ 0.0, 0.0, -normal });
            out.totalForceBody += fTipBody;
            out.totalMomentBody += wingtipBody.Cross(fTipBody);
        }
    };

    CheckWingtip(m_config.leftWingtipBody);
    CheckWingtip(m_config.rightWingtipBody);

    return out;
}

void GroundReactionsSystem::ApplyToRigidBody(RigidBody& body, const GroundReactionsOutput& out) const noexcept {
    if (out.anyGearInContact || out.tailstrikeDetected || out.wingtipStrikeDetected) {
        body.AddForceBody(out.totalForceBody);
        body.AddMomentBody(out.totalMomentBody);
    }
}

} // namespace LokYS::Physics
