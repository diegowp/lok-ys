#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/physics/RigidBodyState.hpp"
#include "LokYS/input/InputSystem.hpp"
#include "LokYS/network/NetworkTypes.hpp"
#include "LokYS/network/NetworkProtocol.hpp"
#include "LokYS/network/UdpSocket.hpp"
#include "LokYS/network/RemoteAircraft.hpp"
#include <memory>
#include <vector>
#include <string>

namespace LokYS::Network {

struct ClientSession {
    NetworkAddress address;
    u32 entityId{ 0 };
    std::string callsign{ "CLIENT" };
    f64 lastSeenSimTime{ 0.0 };
    AircraftFlightPacket latestSnapshot{};
    bool active{ false };
};

class NetworkSystem {
public:
    NetworkSystem();
    ~NetworkSystem();

    NetworkSystem(const NetworkSystem&) = delete;
    NetworkSystem& operator=(const NetworkSystem&) = delete;

    [[nodiscard]] bool Initialize();
    void Shutdown();

    /// @brief Starts dedicated UDP server hosting the flight session.
    bool StartServer(u16 port = Constants::DEFAULT_PORT);

    /// @brief Connects to a remote or local LokYS flight server.
    bool Connect(
        const std::string& hostIp = "127.0.0.1",
        u16 port = Constants::DEFAULT_PORT,
        const std::string& callsign = "N172SP");

    /// @brief Disconnects from current session and returns to Offline role.
    void Disconnect();

    /// @brief Cycles role: Offline -> Server (Host) -> Client (Join localhost) -> Offline.
    void CycleRole();

    /// @brief Injects a custom socket implementation (e.g. MockUdpSocket for testing).
    void SetCustomSocket(std::unique_ptr<INetworkSocket> customSocket);

    /// @brief Advances network packet transmission, reception, and peer interpolation.
    void Update(
        f64 dt,
        f64 simTime,
        const Physics::RigidBodyState& localState,
        const Input::FlightControls& localControls,
        f64 localEngineRpm,
        bool anyGearInContact = true) noexcept;

    [[nodiscard]] NetworkRole GetRole() const noexcept { return m_role; }
    [[nodiscard]] ConnectionState GetState() const noexcept { return m_state; }
    [[nodiscard]] const NetworkStats& GetStats() const noexcept { return m_stats; }
    [[nodiscard]] const std::vector<RemoteAircraft>& GetRemoteAircraft() const noexcept { return m_remoteAircraft; }
    [[nodiscard]] u32 GetLocalEntityId() const noexcept { return m_localEntityId; }
    [[nodiscard]] const std::string& GetCallsign() const noexcept { return m_callsign; }

    /// @brief Helper to format one-line network status string for window title / HUD.
    [[nodiscard]] std::wstring GetStatusString() const;

private:
    void ProcessIncomingPackets(f64 simTime) noexcept;
    void HandleServerPacket(const PacketHeader& header, const u8* payload, const NetworkAddress& sender, f64 simTime) noexcept;
    void HandleClientPacket(const PacketHeader& header, const u8* payload, const NetworkAddress& sender, f64 simTime) noexcept;

    void SendClientSnapshot(
        f64 simTime,
        const Physics::RigidBodyState& localState,
        const Input::FlightControls& localControls,
        f64 localEngineRpm,
        bool anyGearInContact) noexcept;

    void BroadcastServerWorldSnapshot(
        f64 simTime,
        const Physics::RigidBodyState& localState,
        const Input::FlightControls& localControls,
        f64 localEngineRpm,
        bool anyGearInContact) noexcept;

    void SendPing(f64 simTime) noexcept;

    std::unique_ptr<INetworkSocket> m_socket;
    NetworkRole m_role{ NetworkRole::Offline };
    ConnectionState m_state{ ConnectionState::Disconnected };
    NetworkAddress m_serverAddress;

    std::string m_callsign{ "N172SP" };
    u32 m_localEntityId{ 0 };
    u16 m_outgoingSequence{ 0 };

    f64 m_lastSnapshotSendTime{ 0.0 };
    f64 m_lastHeartbeatSendTime{ 0.0 };
    f64 m_lastServerPacketSimTime{ 0.0 };

    // Server state: connected clients
    std::vector<ClientSession> m_clients;
    u32 m_nextClientId{ 1 };

    // Remote peer entities visible in the airspace
    std::vector<RemoteAircraft> m_remoteAircraft;

    NetworkStats m_stats{};
    bool m_initialized{ false };
};

} // namespace LokYS::Network
