#include "LokYS/graphics/Camera.hpp"
#include "LokYS/math/CoordinateFrames.hpp"
#include <algorithm>

namespace LokYS::Graphics {

Camera::Camera() {
    SetPerspective(m_fovY, m_aspect, m_nearZ, m_farZ);
}

void Camera::SetPerspective(f32 fovYRadians, f32 aspectRatio, f32 nearZ, f32 farZ) noexcept {
    m_fovY = fovYRadians;
    m_aspect = aspectRatio;
    m_nearZ = nearZ;
    m_farZ = farZ;

    m_projMatrix = Math::Mat4f::PerspectiveFovLH(m_fovY, m_aspect, m_nearZ, m_farZ);
    m_viewProjMatrix = m_viewMatrix * m_projMatrix;
}

void Camera::SetTargetDimensions(f32 lengthM, f32 spanM, f32 heightM) noexcept {
    m_targetDimensions = Math::Vec3f(
        std::max(1.0f, lengthM),
        std::max(1.0f, spanM),
        std::max(0.5f, heightM));
}

void Camera::AdjustZoom(f32 deltaZoom) noexcept {
    m_zoomMultiplier = std::clamp(m_zoomMultiplier + deltaZoom, 0.35f, 4.0f);
}

void Camera::SetZoom(f32 zoom) noexcept {
    m_zoomMultiplier = std::clamp(zoom, 0.35f, 4.0f);
}

void Camera::AdjustOrbit(f32 deltaYawRad, f32 deltaPitchRad) noexcept {
    m_orbitYawRad += deltaYawRad;
    constexpr f32 twoPi = 6.2831853f;
    if (m_orbitYawRad > 3.14159265f) m_orbitYawRad -= twoPi;
    if (m_orbitYawRad < -3.14159265f) m_orbitYawRad += twoPi;

    constexpr f32 maxPitch = 80.0f * (3.14159265f / 180.0f);
    m_orbitPitchRad = std::clamp(m_orbitPitchRad + deltaPitchRad, -maxPitch, maxPitch);
}

void Camera::ResetOrbit() noexcept {
    m_orbitYawRad = 0.0f;
    m_orbitPitchRad = 0.0f;
}

void Camera::Update(const Math::Vec3d& aircraftPosNed, const Math::Quat4d& aircraftAttitude, f64 dt) noexcept {
    if (m_mode == CameraMode::Chase) {
        // Base distance and height scaled dynamically to target aircraft dimensions
        const f32 baseDist = std::max(8.0f, m_targetDimensions.x * 1.55f);
        const f32 baseHeight = std::max(2.5f, m_targetDimensions.z * 1.15f);

        const f32 currentDist = baseDist * m_zoomMultiplier;
        const f32 currentHeight = baseHeight * m_zoomMultiplier;

        // Look target slightly forward of center
        const f64 lookForward = static_cast<f64>(std::max(1.0f, m_targetDimensions.x * 0.15f));
        const Math::Vec3d lookTargetBody{ lookForward, 0.0, -0.2 };
        const Math::Vec3d lookTargetNed = aircraftPosNed + aircraftAttitude.Rotate(lookTargetBody);
        const Math::Vec3f lookTargetGfx = Math::CoordinateFrames::NedToGraphics(lookTargetNed);

        // Compute 3D spherical orbit offset relative to look target in aircraft body frame
        // Body frame: +X forward, +Y right, +Z down -> behind is -X, above is -Z
        const f32 baseRadius = std::sqrt(currentDist * currentDist + currentHeight * currentHeight);
        const f32 baseElevation = std::atan2(currentHeight, currentDist);

        const f32 elev = std::clamp(baseElevation + m_orbitPitchRad, -1.45f, 1.45f);
        const f32 azim = m_orbitYawRad;

        const f64 deltaX = -static_cast<f64>(baseRadius * std::cos(elev) * std::cos(azim));
        const f64 deltaY = static_cast<f64>(baseRadius * std::cos(elev) * std::sin(azim));
        const f64 deltaZ = -static_cast<f64>(baseRadius * std::sin(elev));

        const Math::Vec3d offsetBody{ deltaX, deltaY, deltaZ };
        const Math::Vec3d offsetNed = aircraftAttitude.Rotate(offsetBody);
        const Math::Vec3d targetCamPosNed = lookTargetNed + offsetNed;

        if (m_firstUpdate || (targetCamPosNed - m_smoothChasePosNed).Length() > 150.0) {
            m_smoothChasePosNed = targetCamPosNed;
            m_firstUpdate = false;
        } else {
            // Smooth spring-damper / exponential filter for gentle camera lag
            f64 blend = std::clamp(dt * 16.0, 0.0, 1.0);
            m_smoothChasePosNed = m_smoothChasePosNed + (targetCamPosNed - m_smoothChasePosNed) * blend;
        }

        m_cameraPosNed = m_smoothChasePosNed;
        m_cameraPosGfx = Math::CoordinateFrames::NedToGraphics(m_cameraPosNed);

        m_forwardGfx = (lookTargetGfx - m_cameraPosGfx).Normalized();

        // Stable world up unless looking directly vertical
        if (std::abs(m_forwardGfx.y) > 0.99f) {
            m_upGfx = (m_forwardGfx.y > 0.0f) ? Math::Vec3f(0.0f, 0.0f, -1.0f) : Math::Vec3f(0.0f, 0.0f, 1.0f);
        } else {
            m_upGfx = Math::Vec3f(0.0f, 1.0f, 0.0f);
        }
    } else {
        // Cockpit view: Pilot's dedicated eye position in Body FRD (Forward, Right, Down)
        const Math::Vec3d pilotEyeNed = aircraftAttitude.Rotate(m_cockpitEyeOffsetBody);

        m_cameraPosNed = aircraftPosNed + pilotEyeNed;
        m_cameraPosGfx = Math::CoordinateFrames::NedToGraphics(m_cameraPosNed);

        // Cockpit head rotation with orbit controls (pitch & yaw look around)
        Math::Quat4d headRot = Math::Quat4d::FromEuler(
            static_cast<f64>(m_orbitPitchRad),
            static_cast<f64>(m_orbitYawRad),
            0.0);
        Math::Quat4d combinedAttitude = aircraftAttitude * headRot;

        const Math::Vec3d lookDirNed = combinedAttitude.Rotate(Math::Vec3d(1.0, 0.0, 0.0));
        const Math::Vec3d upDirNed = combinedAttitude.Rotate(Math::Vec3d(0.0, 0.0, -1.0));

        m_forwardGfx = Math::CoordinateFrames::NedToGraphics(lookDirNed);
        m_upGfx = Math::CoordinateFrames::NedToGraphics(upDirNed);
        m_firstUpdate = true; // reset smooth filter for when user switches back to chase
    }

    // Camera-relative view matrix: Eye is at (0, 0, 0)
    m_viewMatrix = Math::Mat4f::CameraRelativeViewLH(m_forwardGfx, m_upGfx);
    m_viewProjMatrix = m_viewMatrix * m_projMatrix;
}

} // namespace LokYS::Graphics
