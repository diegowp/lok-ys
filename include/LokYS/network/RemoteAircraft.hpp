#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/network/NetworkTypes.hpp"
#include <deque>
#include <string>

namespace LokYS::Network {

class RemoteAircraft {
public:
    RemoteAircraft() = default;
    explicit RemoteAircraft(u32 entityId, const std::string& callsign = "");

    void PushSnapshot(const AircraftFlightPacket& packet);
    void Update(f64 currentSimTime, f64 dt, f64 interpDelay = Constants::INTERPOLATION_DELAY_S) noexcept;

    [[nodiscard]] bool IsExpired(f64 currentSimTime, f64 timeout = Constants::TIMEOUT_INTERVAL_S) const noexcept;

    [[nodiscard]] u32 GetEntityId() const noexcept { return m_entityId; }
    [[nodiscard]] const std::string& GetCallsign() const noexcept { return m_callsign; }

    [[nodiscard]] const Math::Vec3d& GetPositionNed() const noexcept { return m_positionNed; }
    [[nodiscard]] const Math::Vec3d& GetVelocityNed() const noexcept { return m_velocityNed; }
    [[nodiscard]] const Math::Quat4d& GetAttitude() const noexcept { return m_attitude; }

    [[nodiscard]] f32 GetElevator() const noexcept { return m_elevator; }
    [[nodiscard]] f32 GetAileron() const noexcept { return m_aileron; }
    [[nodiscard]] f32 GetRudder() const noexcept { return m_rudder; }
    [[nodiscard]] f32 GetFlaps() const noexcept { return m_flaps; }
    [[nodiscard]] f32 GetEngineRpm() const noexcept { return m_engineRpm; }
    [[nodiscard]] f64 GetPropellerAngleRad() const noexcept { return m_propellerAngleRad; }
    [[nodiscard]] bool IsDeadReckoning() const noexcept { return m_isDeadReckoning; }
    [[nodiscard]] size_t GetSnapshotCount() const noexcept { return m_snapshotHistory.size(); }

    /// @brief Computes cubic Hermite interpolation between two position/velocity samples.
    [[nodiscard]] static Math::Vec3d HermiteInterpolatePosition(
        const Math::Vec3d& p0,
        const Math::Vec3d& v0,
        const Math::Vec3d& p1,
        const Math::Vec3d& v1,
        f64 dt,
        f64 s) noexcept;

private:
    u32 m_entityId{ 0 };
    std::string m_callsign{ "PEER" };
    std::deque<AircraftFlightPacket> m_snapshotHistory;

    Math::Vec3d m_positionNed{ 0.0, 0.0, 0.0 };
    Math::Vec3d m_velocityNed{ 0.0, 0.0, 0.0 };
    Math::Quat4d m_attitude{ 1.0, 0.0, 0.0, 0.0 };

    f32 m_elevator{ 0.0f };
    f32 m_aileron{ 0.0f };
    f32 m_rudder{ 0.0f };
    f32 m_flaps{ 0.0f };
    f32 m_engineRpm{ 0.0f };
    f64 m_propellerAngleRad{ 0.0 };

    f64 m_lastReceivedSimTime{ 0.0 };
    bool m_isDeadReckoning{ false };
};

} // namespace LokYS::Network
