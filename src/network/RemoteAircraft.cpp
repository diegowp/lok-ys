#include "LokYS/network/RemoteAircraft.hpp"
#include <algorithm>
#include <cmath>

namespace LokYS::Network {

RemoteAircraft::RemoteAircraft(u32 entityId, const std::string& callsign)
    : m_entityId(entityId)
    , m_callsign(callsign.empty() ? "N" + std::to_string(entityId) : callsign)
{
}

void RemoteAircraft::PushSnapshot(const AircraftFlightPacket& packet) {
    m_entityId = packet.entityId;
    if (packet.callsign[0] != '\0') {
        m_callsign = packet.callsign;
    }
    m_lastReceivedSimTime = packet.timestamp;

    // Discard duplicates or out-of-order older packets
    if (!m_snapshotHistory.empty() && packet.timestamp <= m_snapshotHistory.back().timestamp) {
        return;
    }

    m_snapshotHistory.push_back(packet);

    // Keep circular buffer size bounded (e.g. up to 64 snapshots ~2 seconds at 30 Hz)
    while (m_snapshotHistory.size() > 64) {
        m_snapshotHistory.pop_front();
    }
}

Math::Vec3d RemoteAircraft::HermiteInterpolatePosition(
    const Math::Vec3d& p0,
    const Math::Vec3d& v0,
    const Math::Vec3d& p1,
    const Math::Vec3d& v1,
    f64 dt,
    f64 s) noexcept
{
    s = std::clamp(s, 0.0, 1.0);
    f64 s2 = s * s;
    f64 s3 = s2 * s;

    f64 h0 = 2.0 * s3 - 3.0 * s2 + 1.0;
    f64 h1 = s3 - 2.0 * s2 + s;
    f64 h2 = -2.0 * s3 + 3.0 * s2;
    f64 h3 = s3 - s2;

    return p0 * h0 + v0 * (h1 * dt) + p1 * h2 + v1 * (h3 * dt);
}

void RemoteAircraft::Update(f64 currentSimTime, f64 dt, f64 interpDelay) noexcept {
    if (m_snapshotHistory.empty()) {
        return;
    }

    // Target render time delayed in the past to absorb network jitter
    f64 targetTime = currentSimTime - interpDelay;

    // Case 1: Only 1 snapshot available
    if (m_snapshotHistory.size() == 1) {
        const auto& s = m_snapshotHistory.front();
        m_positionNed = Math::Vec3d(s.posX, s.posY, s.posZ);
        m_velocityNed = Math::Vec3d(s.velX, s.velY, s.velZ);
        m_attitude = Math::Quat4d(s.rotW, s.rotX, s.rotY, s.rotZ).Normalized();
        m_elevator = s.elevator;
        m_aileron = s.aileron;
        m_rudder = s.rudder;
        m_flaps = s.flaps;
        m_engineRpm = s.engineRpm;

        // Advance propeller animation
        m_propellerAngleRad += (m_engineRpm * 2.0 * 3.141592653589793 / 60.0) * dt;
        if (m_propellerAngleRad > 2.0 * 3.141592653589793) {
            m_propellerAngleRad = std::fmod(m_propellerAngleRad, 2.0 * 3.141592653589793);
        }
        m_isDeadReckoning = false;
        return;
    }

    // Case 2: Target time is older than our oldest stored snapshot
    if (targetTime <= m_snapshotHistory.front().timestamp) {
        const auto& s = m_snapshotHistory.front();
        m_positionNed = Math::Vec3d(s.posX, s.posY, s.posZ);
        m_velocityNed = Math::Vec3d(s.velX, s.velY, s.velZ);
        m_attitude = Math::Quat4d(s.rotW, s.rotX, s.rotY, s.rotZ).Normalized();
        m_elevator = s.elevator;
        m_aileron = s.aileron;
        m_rudder = s.rudder;
        m_flaps = s.flaps;
        m_engineRpm = s.engineRpm;
        m_isDeadReckoning = false;
    }
    // Case 3: Target time is ahead of our newest snapshot -> DEAD RECKONING
    else if (targetTime >= m_snapshotHistory.back().timestamp) {
        const auto& latest = m_snapshotHistory.back();
        f64 deltaExtrap = targetTime - latest.timestamp;

        // Kinematic projective dead reckoning
        Math::Vec3d pLatest(latest.posX, latest.posY, latest.posZ);
        Math::Vec3d vLatest(latest.velX, latest.velY, latest.velZ);
        m_positionNed = pLatest + vLatest * deltaExtrap;
        m_velocityNed = vLatest;

        Math::Quat4d qLatest(latest.rotW, latest.rotX, latest.rotY, latest.rotZ);
        qLatest.Renormalize();

        // Extrapolate rotation using angular rates
        Math::Vec3d omega(latest.angVelP, latest.angVelQ, latest.angVelR);
        f64 rateMag = omega.Length();
        if (rateMag > 1e-5) {
            Math::Quat4d rotDelta = Math::Quat4d::FromAxisAngle(omega * (1.0 / rateMag), rateMag * deltaExtrap);
            m_attitude = (qLatest * rotDelta).Normalized();
        } else {
            m_attitude = qLatest;
        }

        m_elevator = latest.elevator;
        m_aileron = latest.aileron;
        m_rudder = latest.rudder;
        m_flaps = latest.flaps;
        m_engineRpm = latest.engineRpm;
        m_isDeadReckoning = true;
    }
    // Case 4: Target time is bracketed between two snapshots -> INTERPOLATE
    else {
        m_isDeadReckoning = false;

        // Search for interval [s0, s1] containing targetTime
        size_t idx0 = 0;
        for (size_t i = 0; i < m_snapshotHistory.size() - 1; ++i) {
            if (targetTime >= m_snapshotHistory[i].timestamp && targetTime <= m_snapshotHistory[i + 1].timestamp) {
                idx0 = i;
                break;
            }
        }

        const auto& s0 = m_snapshotHistory[idx0];
        const auto& s1 = m_snapshotHistory[idx0 + 1];

        f64 intervalDt = s1.timestamp - s0.timestamp;
        f64 factor = (intervalDt > 1e-6) ? (targetTime - s0.timestamp) / intervalDt : 0.0;
        factor = std::clamp(factor, 0.0, 1.0);

        Math::Vec3d p0(s0.posX, s0.posY, s0.posZ);
        Math::Vec3d v0(s0.velX, s0.velY, s0.velZ);
        Math::Vec3d p1(s1.posX, s1.posY, s1.posZ);
        Math::Vec3d v1(s1.velX, s1.velY, s1.velZ);

        // Hermite cubic spline position interpolation
        m_positionNed = HermiteInterpolatePosition(p0, v0, p1, v1, intervalDt, factor);

        // Linear velocity interpolation
        m_velocityNed = v0 + (v1 - v0) * factor;

        // Spherical linear quaternion interpolation (SLERP)
        Math::Quat4d q0(s0.rotW, s0.rotX, s0.rotY, s0.rotZ);
        Math::Quat4d q1(s1.rotW, s1.rotX, s1.rotY, s1.rotZ);
        m_attitude = Math::Quat4d::Slerp(q0.Normalized(), q1.Normalized(), factor);

        // Linear controls interpolation
        float f = static_cast<float>(factor);
        m_elevator = s0.elevator + (s1.elevator - s0.elevator) * f;
        m_aileron = s0.aileron + (s1.aileron - s0.aileron) * f;
        m_rudder = s0.rudder + (s1.rudder - s0.rudder) * f;
        m_flaps = s0.flaps + (s1.flaps - s0.flaps) * f;
        m_engineRpm = s0.engineRpm + (s1.engineRpm - s0.engineRpm) * f;
    }

    // Advance continuous propeller spinning animation
    m_propellerAngleRad += (m_engineRpm * 2.0 * 3.141592653589793 / 60.0) * dt;
    if (m_propellerAngleRad > 2.0 * 3.141592653589793) {
        m_propellerAngleRad = std::fmod(m_propellerAngleRad, 2.0 * 3.141592653589793);
    }

    // Discard old snapshots older than targetTime - 0.5s to preserve memory
    while (m_snapshotHistory.size() > 4 && m_snapshotHistory[1].timestamp < (targetTime - 0.5)) {
        m_snapshotHistory.pop_front();
    }
}

bool RemoteAircraft::IsExpired(f64 currentSimTime, f64 timeout) const noexcept {
    return (currentSimTime - m_lastReceivedSimTime) > timeout;
}

} // namespace LokYS::Network
