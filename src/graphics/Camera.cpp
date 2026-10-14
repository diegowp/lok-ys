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

void Camera::Update(const Math::Vec3d& aircraftPosNed, const Math::Quat4d& aircraftAttitude, f64 dt) noexcept {
    if (m_mode == CameraMode::Chase) {
        // Chase view: 14 meters behind, 3.5 meters above in aircraft body frame
        // Body frame: +X forward, +Y right, +Z down -> behind is -X, above is -Z
        const Math::Vec3d chaseOffsetBody{ -14.0, 0.0, -3.5 };
        const Math::Vec3d chaseOffsetNed = aircraftAttitude.Rotate(chaseOffsetBody);
        const Math::Vec3d targetCamPosNed = aircraftPosNed + chaseOffsetNed;

        if (m_firstUpdate) {
            m_smoothChasePosNed = targetCamPosNed;
            m_firstUpdate = false;
        } else {
            // Smooth spring-damper / exponential filter for gentle camera lag
            f64 blend = std::clamp(dt * 12.0, 0.0, 1.0);
            m_smoothChasePosNed = m_smoothChasePosNed + (targetCamPosNed - m_smoothChasePosNed) * blend;
        }

        m_cameraPosNed = m_smoothChasePosNed;
        m_cameraPosGfx = Math::CoordinateFrames::NedToGraphics(m_cameraPosNed);

        // Look target: 2.0m ahead of aircraft center
        const Math::Vec3d lookTargetNed = aircraftPosNed + aircraftAttitude.Rotate(Math::Vec3d(2.0, 0.0, 0.0));
        const Math::Vec3f lookTargetGfx = Math::CoordinateFrames::NedToGraphics(lookTargetNed);

        m_forwardGfx = (lookTargetGfx - m_cameraPosGfx).Normalized();
        // Standard world up for stable horizon in chase view
        m_upGfx = Math::Vec3f(0.0f, 1.0f, 0.0f);
    } else {
        // Cockpit view: Pilot's left seat eye position
        // Body frame: +X forward (nose), +Y right, +Z down
        // Pilot eye: 0.35m forward, -0.25m left, -0.65m above waterline
        const Math::Vec3d pilotEyeBody{ 0.35, -0.25, -0.65 };
        const Math::Vec3d pilotEyeNed = aircraftAttitude.Rotate(pilotEyeBody);

        m_cameraPosNed = aircraftPosNed + pilotEyeNed;
        m_cameraPosGfx = Math::CoordinateFrames::NedToGraphics(m_cameraPosNed);

        // Forward looking direction along aircraft nose
        const Math::Vec3d lookDirNed = aircraftAttitude.Rotate(Math::Vec3d(1.0, 0.0, 0.0));
        const Math::Vec3d upDirNed = aircraftAttitude.Rotate(Math::Vec3d(0.0, 0.0, -1.0));

        m_forwardGfx = Math::CoordinateFrames::NedToGraphics(lookDirNed);
        m_upGfx = Math::CoordinateFrames::NedToGraphics(upDirNed);
        m_firstUpdate = true; // reset smooth filter for when user switches back to chase
    }

    // Camera-relative view matrix: Eye is at (0, 0, 0)
    m_viewMatrix = Math::Mat4f::CameraRelativeViewLH(m_forwardGfx, m_upGfx);
    m_viewProjMatrix = m_viewMatrix * m_projMatrix;
}

} // namespace LokYS::Graphics
