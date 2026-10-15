#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/math/Matrix4x4.hpp"

namespace LokYS::Graphics {

enum class CameraMode {
    Chase,
    Cockpit
};

class Camera {
public:
    Camera();

    void SetMode(CameraMode mode) noexcept { m_mode = mode; }
    [[nodiscard]] CameraMode GetMode() const noexcept { return m_mode; }
    void ToggleMode() noexcept {
        m_mode = (m_mode == CameraMode::Chase) ? CameraMode::Cockpit : CameraMode::Chase;
    }

    void SetPerspective(f32 fovYRadians, f32 aspectRatio, f32 nearZ = 0.2f, f32 farZ = 20000.0f) noexcept;
    void Update(const Math::Vec3d& aircraftPosNed, const Math::Quat4d& aircraftAttitude, f64 dt) noexcept;
    void Reset() noexcept { m_firstUpdate = true; ResetOrbit(); }

    // Dynamic Framing & Model Dimension Adaptation
    void SetTargetDimensions(f32 lengthM, f32 spanM, f32 heightM) noexcept;
    [[nodiscard]] const Math::Vec3f& GetTargetDimensions() const noexcept { return m_targetDimensions; }

    // Interactive Zoom Controls (0.35x to 4.0x)
    void AdjustZoom(f32 deltaZoom) noexcept;
    void SetZoom(f32 zoom) noexcept;
    [[nodiscard]] f32 GetZoom() const noexcept { return m_zoomMultiplier; }

    // Interactive 360-degree Orbit Controls (Yaw and Pitch around aircraft)
    void AdjustOrbit(f32 deltaYawRad, f32 deltaPitchRad) noexcept;
    void ResetOrbit() noexcept;
    [[nodiscard]] f32 GetOrbitYaw() const noexcept { return m_orbitYawRad; }
    [[nodiscard]] f32 GetOrbitPitch() const noexcept { return m_orbitPitchRad; }

    [[nodiscard]] const Math::Vec3d& GetCameraPosNed() const noexcept { return m_cameraPosNed; }
    [[nodiscard]] const Math::Vec3f& GetCameraPosGfx() const noexcept { return m_cameraPosGfx; }
    [[nodiscard]] const Math::Mat4f& GetViewMatrix() const noexcept { return m_viewMatrix; }
    [[nodiscard]] const Math::Mat4f& GetProjMatrix() const noexcept { return m_projMatrix; }
    [[nodiscard]] const Math::Mat4f& GetViewProjMatrix() const noexcept { return m_viewProjMatrix; }
    [[nodiscard]] const Math::Vec3f& GetForward() const noexcept { return m_forwardGfx; }
    [[nodiscard]] const Math::Vec3f& GetUp() const noexcept { return m_upGfx; }

private:
    CameraMode m_mode{ CameraMode::Chase };

    Math::Vec3d m_cameraPosNed{ 0.0, 0.0, 0.0 };
    Math::Vec3f m_cameraPosGfx{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f m_forwardGfx{ 0.0f, 0.0f, 1.0f };
    Math::Vec3f m_upGfx{ 0.0f, 1.0f, 0.0f };

    Math::Mat4f m_viewMatrix{ Math::Mat4f::Identity() };
    Math::Mat4f m_projMatrix{ Math::Mat4f::Identity() };
    Math::Mat4f m_viewProjMatrix{ Math::Mat4f::Identity() };

    f32 m_fovY{ 1.0472f }; // 60 degrees in radians
    f32 m_aspect{ 16.0f / 9.0f };
    f32 m_nearZ{ 0.2f };
    f32 m_farZ{ 20000.0f };

    // Chase camera smooth tracking
    Math::Vec3d m_smoothChasePosNed{ 0.0, 0.0, 0.0 };
    bool m_firstUpdate{ true };

    // Dynamic Framing, Zoom, and Orbit
    Math::Vec3f m_targetDimensions{ 8.28f, 11.0f, 2.72f };
    f32 m_zoomMultiplier{ 1.0f };
    f32 m_orbitYawRad{ 0.0f };
    f32 m_orbitPitchRad{ 0.0f };
};

} // namespace LokYS::Graphics
