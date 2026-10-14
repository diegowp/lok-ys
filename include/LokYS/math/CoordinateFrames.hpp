#pragma once

#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/math/Matrix3x3.hpp"
#include <cmath>

namespace LokYS::Math {

struct AerodynamicState {
    f64 trueAirspeed{0.0};  // V_TAS (m/s)
    f64 alphaRad{0.0};      // Angle of attack (radians)
    f64 betaRad{0.0};       // Sideslip angle (radians)
    f64 dynamicPressure{0.0}; // q = 0.5 * rho * V^2 (Pa)
    f64 mach{0.0};          // Mach number
};

class CoordinateFrames {
public:
    // Compute Aerodynamic state from Body relative wind vector (u, v, w) and air density (rho), speed of sound (a)
    [[nodiscard]] static AerodynamicState ComputeAerodynamicState(
        const Vec3d& relativeWindBody,
        f64 airDensityKgM3,
        f64 speedOfSoundMps) noexcept
    {
        AerodynamicState state{};
        f64 u = relativeWindBody.x;
        f64 v = relativeWindBody.y;
        f64 w = relativeWindBody.z;

        state.trueAirspeed = relativeWindBody.Length();

        if (state.trueAirspeed > 1e-4) {
            // alpha = atan2(w, u)
            state.alphaRad = std::atan2(w, u);

            // beta = asin(v / V_TAS) clamped to [-1, 1]
            f64 sinBeta = std::clamp(v / state.trueAirspeed, -1.0, 1.0);
            state.betaRad = std::asin(sinBeta);

            state.dynamicPressure = 0.5 * airDensityKgM3 * (state.trueAirspeed * state.trueAirspeed);
            if (speedOfSoundMps > 1e-3) {
                state.mach = state.trueAirspeed / speedOfSoundMps;
            }
        }

        return state;
    }

    // Transform Wind-frame forces [Drag, SideForce, Lift] into Body-frame forces [Fx, Fy, Fz]
    // Convention: Drag is along -X_wind, SideForce is along +Y_wind, Lift is along -Z_wind
    [[nodiscard]] static Vec3d WindForcesToBody(
        f64 liftNewtons,
        f64 dragNewtons,
        f64 sideForceNewtons,
        f64 alphaRad,
        f64 betaRad) noexcept
    {
        f64 ca = std::cos(alphaRad);
        f64 sa = std::sin(alphaRad);
        f64 cb = std::cos(betaRad);
        f64 sb = std::sin(betaRad);

        // Standard aerospace DCM from wind to body frame:
        // Fx = -D * ca * cb - Y * ca * sb + L * sa
        // Fy = -D * sb      + Y * cb
        // Fz = -D * sa * cb - Y * sa * sb - L * ca
        f64 fx = -dragNewtons * ca * cb - sideForceNewtons * ca * sb + liftNewtons * sa;
        f64 fy = -dragNewtons * sb      + sideForceNewtons * cb;
        f64 fz = -dragNewtons * sa * cb - sideForceNewtons * sa * sb - liftNewtons * ca;

        return { fx, fy, fz };
    }

    // Rotate vector from Body (FRD) to Navigation (NED) using aircraft attitude quaternion
    [[nodiscard]] static Vec3d BodyToNed(const Vec3d& vBody, const Quat4d& qAttitude) noexcept {
        return qAttitude.Rotate(vBody);
    }

    // Rotate vector from Navigation (NED) to Body (FRD) using aircraft attitude quaternion
    [[nodiscard]] static Vec3d NedToBody(const Vec3d& vNed, const Quat4d& qAttitude) noexcept {
        return qAttitude.Unrotate(vNed);
    }

    // Convert Camera-Relative position: Pos_rel = Pos_world - Camera_world
    [[nodiscard]] static Vec3f WorldToCameraRelative(const Vec3d& worldPos, const Vec3d& cameraPos) noexcept {
        Vec3d delta = worldPos - cameraPos;
        return {
            static_cast<f32>(delta.x),
            static_cast<f32>(delta.y),
            static_cast<f32>(delta.z)
        };
    }

    // Convert Aerospace NED coordinates to Direct3D 12 Graphics World (X East, Y Up, Z North)
    [[nodiscard]] static Vec3f NedToGraphics(const Vec3d& posNed) noexcept {
        return {
            static_cast<f32>(posNed.y),   // +X gfx is East
            static_cast<f32>(-posNed.z),  // +Y gfx is Up (altitude)
            static_cast<f32>(posNed.x)    // +Z gfx is North
        };
    }

    [[nodiscard]] static Vec3f NedToGraphics(const Vec3f& posNed) noexcept {
        return {
            posNed.y,
            -posNed.z,
            posNed.x
        };
    }
};

} // namespace LokYS::Math

