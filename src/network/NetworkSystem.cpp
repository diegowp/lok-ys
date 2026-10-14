#include "LokYS/network/NetworkSystem.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>

namespace LokYS::Network {

NetworkSystem::NetworkSystem() = default;

NetworkSystem::~NetworkSystem() {
    Shutdown();
}

bool NetworkSystem::Initialize() {
    if (!m_socket) {
        m_socket = std::make_unique<UdpSocket>();
    }
    m_initialized = true;
    return true;
}

void NetworkSystem::Shutdown() {
    Disconnect();
    if (m_socket) {
        m_socket->Close();
        m_socket.reset();
    }
    m_initialized = false;
}

void NetworkSystem::SetCustomSocket(std::unique_ptr<INetworkSocket> customSocket) {
    if (m_socket) {
        m_socket->Close();
    }
    m_socket = std::move(customSocket);
    m_initialized = (m_socket != nullptr);
}

bool NetworkSystem::StartServer(u16 port) {
    if (!m_socket) {
        m_socket = std::make_unique<UdpSocket>();
    }
    if (!m_socket->Open(port)) {
        return false;
    }

    m_role = NetworkRole::Server;
    m_state = ConnectionState::Connected;
    m_localEntityId = 0; // Host is entity 0
    m_clients.clear();
    m_remoteAircraft.clear();
    m_nextClientId = 1;
    m_stats = NetworkStats{};
    return true;
}

bool NetworkSystem::Connect(
    const std::string& hostIp,
    u16 port,
    const std::string& callsign)
{
    if (!m_socket) {
        m_socket = std::make_unique<UdpSocket>();
    }
    if (!m_socket->Open(0)) { // Bind ephemeral client port
        return false;
    }

    m_serverAddress.ip = hostIp;
    m_serverAddress.port = port;
    if (!callsign.empty()) {
        m_callsign = callsign;
    }

    m_role = NetworkRole::Client;
    m_state = ConnectionState::Connecting;
    m_remoteAircraft.clear();
    m_stats = NetworkStats{};
    m_lastServerPacketSimTime = 0.0;

    // Send initial ConnectRequest
    ConnectRequestPayload req{};
    size_t copyLen = std::min(m_callsign.length(), sizeof(req.callsign) - 1);
    std::memcpy(req.callsign, m_callsign.c_str(), copyLen);
    req.callsign[copyLen] = '\0';
    req.clientVersion = Constants::PROTOCOL_VERSION;

    u8 packetBuf[Constants::MAX_PACKET_SIZE];
    size_t packetSize = NetworkProtocol::BuildPacket(
        PacketType::ConnectRequest,
        m_outgoingSequence++,
        &req,
        sizeof(req),
        packetBuf,
        sizeof(packetBuf));

    if (packetSize > 0) {
        m_socket->SendTo(packetBuf, packetSize, m_serverAddress);
        m_stats.packetsSent++;
        m_stats.bytesSent += packetSize;
    }

    return true;
}

void NetworkSystem::Disconnect() {
    if (m_role == NetworkRole::Client && m_socket && m_socket->IsOpen()) {
        // Send Disconnect packet
        u8 packetBuf[Constants::MAX_PACKET_SIZE];
        size_t packetSize = NetworkProtocol::BuildPacket(
            PacketType::Disconnect,
            m_outgoingSequence++,
            nullptr,
            0,
            packetBuf,
            sizeof(packetBuf));
        if (packetSize > 0) {
            m_socket->SendTo(packetBuf, packetSize, m_serverAddress);
        }
    }

    if (m_socket) {
        m_socket->Close();
    }
    m_role = NetworkRole::Offline;
    m_state = ConnectionState::Disconnected;
    m_clients.clear();
    m_remoteAircraft.clear();
}

void NetworkSystem::CycleRole() {
    switch (m_role) {
    case NetworkRole::Offline:
        StartServer(Constants::DEFAULT_PORT);
        break;
    case NetworkRole::Server:
        Disconnect();
        Connect("127.0.0.1", Constants::DEFAULT_PORT, "N172SP");
        break;
    case NetworkRole::Client:
        Disconnect();
        break;
    }
}

void NetworkSystem::ProcessIncomingPackets(f64 simTime) noexcept {
    if (!m_socket || !m_socket->IsOpen()) {
        return;
    }

    u8 inBuf[Constants::MAX_PACKET_SIZE];
    NetworkAddress sender{};

    for (int loop = 0; loop < 128; ++loop) {
        int bytesRead = m_socket->ReceiveFrom(inBuf, sizeof(inBuf), sender);
        if (bytesRead <= 0) {
            break;
        }

        m_stats.packetsReceived++;
        m_stats.bytesReceived += static_cast<u64>(bytesRead);

        PacketHeader header{};
        const u8* payload = nullptr;
        if (!NetworkProtocol::ParsePacket(inBuf, static_cast<size_t>(bytesRead), header, payload)) {
            continue; // Dropped invalid / corrupted packet
        }

        if (m_role == NetworkRole::Server) {
            HandleServerPacket(header, payload, sender, simTime);
        } else if (m_role == NetworkRole::Client) {
            HandleClientPacket(header, payload, sender, simTime);
        }
    }
}

void NetworkSystem::HandleServerPacket(
    const PacketHeader& header,
    const u8* payload,
    const NetworkAddress& sender,
    f64 simTime) noexcept
{
    PacketType type = static_cast<PacketType>(header.type);

    switch (type) {
    case PacketType::ConnectRequest: {
        if (header.payloadSize < sizeof(ConnectRequestPayload)) {
            break;
        }
        const auto* req = reinterpret_cast<const ConnectRequestPayload*>(payload);

        // Find or register client session
        auto it = std::find_if(m_clients.begin(), m_clients.end(), [&](const ClientSession& c) {
            return c.address == sender;
        });

        u32 assignedId = 0;
        if (it != m_clients.end()) {
            assignedId = it->entityId;
            it->lastSeenSimTime = simTime;
        } else {
            assignedId = m_nextClientId++;
            ClientSession session{};
            session.address = sender;
            session.entityId = assignedId;
            session.callsign = std::string(req->callsign, strnlen(req->callsign, sizeof(req->callsign)));
            session.lastSeenSimTime = simTime;
            session.active = true;
            m_clients.push_back(session);
        }

        // Send ConnectAccept response
        ConnectAcceptPayload accept{};
        accept.assignedClientId = assignedId;
        accept.serverTick = m_outgoingSequence;
        accept.serverTime = simTime;

        u8 outBuf[Constants::MAX_PACKET_SIZE];
        size_t outSize = NetworkProtocol::BuildPacket(
            PacketType::ConnectAccept,
            m_outgoingSequence++,
            &accept,
            sizeof(accept),
            outBuf,
            sizeof(outBuf));

        if (outSize > 0) {
            m_socket->SendTo(outBuf, outSize, sender);
            m_stats.packetsSent++;
            m_stats.bytesSent += outSize;
        }
        break;
    }

    case PacketType::Disconnect: {
        auto it = std::find_if(m_clients.begin(), m_clients.end(), [&](const ClientSession& c) {
            return c.address == sender;
        });
        if (it != m_clients.end()) {
            u32 entId = it->entityId;
            m_clients.erase(it);

            // Remove from remote aircraft list
            m_remoteAircraft.erase(
                std::remove_if(m_remoteAircraft.begin(), m_remoteAircraft.end(), [&](const RemoteAircraft& r) {
                    return r.GetEntityId() == entId;
                }),
                m_remoteAircraft.end());
        }
        break;
    }

    case PacketType::Ping: {
        if (header.payloadSize < sizeof(PingPayload)) {
            break;
        }
        const auto* ping = reinterpret_cast<const PingPayload*>(payload);
        PongPayload pong{};
        pong.pingId = ping->pingId;
        pong.clientSendTime = ping->clientSendTime;
        pong.serverReceiveTime = simTime;

        u8 outBuf[Constants::MAX_PACKET_SIZE];
        size_t outSize = NetworkProtocol::BuildPacket(
            PacketType::Pong,
            m_outgoingSequence++,
            &pong,
            sizeof(pong),
            outBuf,
            sizeof(outBuf));

        if (outSize > 0) {
            m_socket->SendTo(outBuf, outSize, sender);
            m_stats.packetsSent++;
            m_stats.bytesSent += outSize;
        }
        break;
    }

    case PacketType::ClientSnapshot: {
        if (header.payloadSize < sizeof(AircraftFlightPacket)) {
            break;
        }
        const auto* snap = reinterpret_cast<const AircraftFlightPacket*>(payload);

        auto it = std::find_if(m_clients.begin(), m_clients.end(), [&](const ClientSession& c) {
            return c.address == sender;
        });

        if (it != m_clients.end()) {
            it->lastSeenSimTime = simTime;
            it->latestSnapshot = *snap;
            it->latestSnapshot.entityId = it->entityId;

            // Update host's local remote aircraft registry so server player sees client!
            auto peerIt = std::find_if(m_remoteAircraft.begin(), m_remoteAircraft.end(), [&](const RemoteAircraft& r) {
                return r.GetEntityId() == it->entityId;
            });

            if (peerIt != m_remoteAircraft.end()) {
                peerIt->PushSnapshot(it->latestSnapshot);
            } else {
                RemoteAircraft newPeer(it->entityId, it->callsign);
                newPeer.PushSnapshot(it->latestSnapshot);
                m_remoteAircraft.push_back(std::move(newPeer));
            }
        }
        break;
    }

    default:
        break;
    }
}

void NetworkSystem::HandleClientPacket(
    const PacketHeader& header,
    const u8* payload,
    [[maybe_unused]] const NetworkAddress& sender,
    f64 simTime) noexcept
{
    PacketType type = static_cast<PacketType>(header.type);
    m_lastServerPacketSimTime = simTime;

    switch (type) {
    case PacketType::ConnectAccept: {
        if (header.payloadSize >= sizeof(ConnectAcceptPayload)) {
            const auto* accept = reinterpret_cast<const ConnectAcceptPayload*>(payload);
            m_localEntityId = accept->assignedClientId;
            m_state = ConnectionState::Connected;
        }
        break;
    }

    case PacketType::Pong: {
        if (header.payloadSize >= sizeof(PongPayload)) {
            const auto* pong = reinterpret_cast<const PongPayload*>(payload);
            f64 rttSeconds = simTime - pong->clientSendTime;
            if (rttSeconds > 0.0 && rttSeconds < 5.0) {
                f32 instantRttMs = static_cast<f32>(rttSeconds * 1000.0);
                if (m_stats.roundTripTimeMs <= 0.001f) {
                    m_stats.roundTripTimeMs = instantRttMs;
                } else {
                    m_stats.roundTripTimeMs = m_stats.roundTripTimeMs * 0.85f + instantRttMs * 0.15f;
                }
            }
        }
        break;
    }

    case PacketType::WorldSnapshot: {
        if (header.payloadSize < sizeof(WorldSnapshotHeader)) {
            break;
        }
        const auto* worldHdr = reinterpret_cast<const WorldSnapshotHeader*>(payload);
        size_t availablePayload = header.payloadSize - sizeof(WorldSnapshotHeader);
        size_t entityCount = worldHdr->entityCount;

        if (entityCount * sizeof(AircraftFlightPacket) > availablePayload) {
            break; // Malformed payload
        }

        const auto* entities = reinterpret_cast<const AircraftFlightPacket*>(payload + sizeof(WorldSnapshotHeader));

        for (size_t i = 0; i < entityCount; ++i) {
            const auto& peerSnap = entities[i];
            if (peerSnap.entityId == m_localEntityId) {
                continue; // Skip local aircraft echo
            }

            auto peerIt = std::find_if(m_remoteAircraft.begin(), m_remoteAircraft.end(), [&](const RemoteAircraft& r) {
                return r.GetEntityId() == peerSnap.entityId;
            });

            if (peerIt != m_remoteAircraft.end()) {
                peerIt->PushSnapshot(peerSnap);
            } else {
                RemoteAircraft newPeer(peerSnap.entityId, peerSnap.callsign);
                newPeer.PushSnapshot(peerSnap);
                m_remoteAircraft.push_back(std::move(newPeer));
            }
        }
        break;
    }

    default:
        break;
    }
}

void NetworkSystem::SendClientSnapshot(
    f64 simTime,
    const Physics::RigidBodyState& localState,
    const Input::FlightControls& localControls,
    f64 localEngineRpm,
    bool anyGearInContact) noexcept
{
    if (m_state != ConnectionState::Connected) {
        return;
    }

    AircraftFlightPacket snap{};
    snap.entityId = m_localEntityId;
    size_t copyLen = std::min(m_callsign.length(), sizeof(snap.callsign) - 1);
    std::memcpy(snap.callsign, m_callsign.c_str(), copyLen);
    snap.callsign[copyLen] = '\0';
    snap.sequence = m_outgoingSequence++;
    snap.timestamp = simTime;

    snap.posX = localState.positionNed.x;
    snap.posY = localState.positionNed.y;
    snap.posZ = localState.positionNed.z;

    Math::Vec3d velNed = localState.attitude.Rotate(localState.velocityBody);
    snap.velX = static_cast<f32>(velNed.x);
    snap.velY = static_cast<f32>(velNed.y);
    snap.velZ = static_cast<f32>(velNed.z);

    snap.rotW = static_cast<f32>(localState.attitude.w);
    snap.rotX = static_cast<f32>(localState.attitude.x);
    snap.rotY = static_cast<f32>(localState.attitude.y);
    snap.rotZ = static_cast<f32>(localState.attitude.z);

    snap.angVelP = static_cast<f32>(localState.angularVelocityBody.x);
    snap.angVelQ = static_cast<f32>(localState.angularVelocityBody.y);
    snap.angVelR = static_cast<f32>(localState.angularVelocityBody.z);

    snap.elevator = static_cast<f32>(localControls.pitch);
    snap.aileron = static_cast<f32>(localControls.roll);
    snap.rudder = static_cast<f32>(localControls.yaw);
    snap.flaps = static_cast<f32>(localControls.flapSetting) / 3.0f;
    snap.engineRpm = static_cast<f32>(localEngineRpm);
    snap.gearFlags = anyGearInContact ? 0x07 : 0x00;

    u8 packetBuf[Constants::MAX_PACKET_SIZE];
    size_t packetSize = NetworkProtocol::BuildPacket(
        PacketType::ClientSnapshot,
        static_cast<u16>(snap.sequence),
        &snap,
        sizeof(snap),
        packetBuf,
        sizeof(packetBuf));

    if (packetSize > 0) {
        m_socket->SendTo(packetBuf, packetSize, m_serverAddress);
        m_stats.packetsSent++;
        m_stats.bytesSent += packetSize;
    }
}

void NetworkSystem::BroadcastServerWorldSnapshot(
    f64 simTime,
    const Physics::RigidBodyState& localState,
    const Input::FlightControls& localControls,
    f64 localEngineRpm,
    bool anyGearInContact) noexcept
{
    if (m_clients.empty()) {
        return;
    }

    // Prepare array of snapshots: Host + active clients
    std::vector<AircraftFlightPacket> snapshots;
    snapshots.reserve(1 + m_clients.size());

    // Host snapshot (entity 0)
    AircraftFlightPacket hostSnap{};
    hostSnap.entityId = 0;
    size_t copyLen = std::min(m_callsign.length(), sizeof(hostSnap.callsign) - 1);
    std::memcpy(hostSnap.callsign, m_callsign.c_str(), copyLen);
    hostSnap.callsign[copyLen] = '\0';
    hostSnap.sequence = m_outgoingSequence;
    hostSnap.timestamp = simTime;

    hostSnap.posX = localState.positionNed.x;
    hostSnap.posY = localState.positionNed.y;
    hostSnap.posZ = localState.positionNed.z;

    Math::Vec3d velNed = localState.attitude.Rotate(localState.velocityBody);
    hostSnap.velX = static_cast<f32>(velNed.x);
    hostSnap.velY = static_cast<f32>(velNed.y);
    hostSnap.velZ = static_cast<f32>(velNed.z);

    hostSnap.rotW = static_cast<f32>(localState.attitude.w);
    hostSnap.rotX = static_cast<f32>(localState.attitude.x);
    hostSnap.rotY = static_cast<f32>(localState.attitude.y);
    hostSnap.rotZ = static_cast<f32>(localState.attitude.z);

    hostSnap.angVelP = static_cast<f32>(localState.angularVelocityBody.x);
    hostSnap.angVelQ = static_cast<f32>(localState.angularVelocityBody.y);
    hostSnap.angVelR = static_cast<f32>(localState.angularVelocityBody.z);

    hostSnap.elevator = static_cast<f32>(localControls.pitch);
    hostSnap.aileron = static_cast<f32>(localControls.roll);
    hostSnap.rudder = static_cast<f32>(localControls.yaw);
    hostSnap.flaps = static_cast<f32>(localControls.flapSetting) / 3.0f;
    hostSnap.engineRpm = static_cast<f32>(localEngineRpm);
    hostSnap.gearFlags = anyGearInContact ? 0x07 : 0x00;
    snapshots.push_back(hostSnap);

    // Clients snapshots
    for (const auto& client : m_clients) {
        if (client.active && client.latestSnapshot.timestamp > 0.0) {
            snapshots.push_back(client.latestSnapshot);
        }
    }

    WorldSnapshotHeader hdr{};
    hdr.serverSequence = m_outgoingSequence++;
    hdr.serverTime = simTime;
    hdr.entityCount = static_cast<u16>(snapshots.size());

    u8 payloadBuf[Constants::MAX_PACKET_SIZE];
    size_t payloadSize = sizeof(WorldSnapshotHeader) + snapshots.size() * sizeof(AircraftFlightPacket);
    if (payloadSize > sizeof(payloadBuf)) {
        return; // Exceeds MTU
    }

    std::memcpy(payloadBuf, &hdr, sizeof(WorldSnapshotHeader));
    std::memcpy(payloadBuf + sizeof(WorldSnapshotHeader), snapshots.data(), snapshots.size() * sizeof(AircraftFlightPacket));

    u8 packetBuf[Constants::MAX_PACKET_SIZE];
    size_t packetSize = NetworkProtocol::BuildPacket(
        PacketType::WorldSnapshot,
        static_cast<u16>(hdr.serverSequence),
        payloadBuf,
        static_cast<u16>(payloadSize),
        packetBuf,
        sizeof(packetBuf));

    if (packetSize > 0) {
        for (const auto& client : m_clients) {
            if (client.active) {
                m_socket->SendTo(packetBuf, packetSize, client.address);
                m_stats.packetsSent++;
                m_stats.bytesSent += packetSize;
            }
        }
    }
}

void NetworkSystem::SendPing(f64 simTime) noexcept {
    PingPayload ping{};
    ping.pingId = m_outgoingSequence++;
    ping.clientSendTime = simTime;

    u8 packetBuf[Constants::MAX_PACKET_SIZE];
    size_t packetSize = NetworkProtocol::BuildPacket(
        PacketType::Ping,
        static_cast<u16>(ping.pingId),
        &ping,
        sizeof(ping),
        packetBuf,
        sizeof(packetBuf));

    if (packetSize > 0) {
        m_socket->SendTo(packetBuf, packetSize, m_serverAddress);
        m_stats.packetsSent++;
        m_stats.bytesSent += packetSize;
    }
}

void NetworkSystem::Update(
    f64 dt,
    f64 simTime,
    const Physics::RigidBodyState& localState,
    const Input::FlightControls& localControls,
    f64 localEngineRpm,
    bool anyGearInContact) noexcept
{
    if (m_role == NetworkRole::Offline) {
        return;
    }

    ProcessIncomingPackets(simTime);

    if (m_role == NetworkRole::Client) {
        if (m_state == ConnectionState::Connecting) {
            // Retry connect request every 1.0 second
            if (simTime - m_lastSnapshotSendTime >= 1.0) {
                m_lastSnapshotSendTime = simTime;
                ConnectRequestPayload req{};
                size_t copyLen = std::min(m_callsign.length(), sizeof(req.callsign) - 1);
                std::memcpy(req.callsign, m_callsign.c_str(), copyLen);
                req.callsign[copyLen] = '\0';
                req.clientVersion = Constants::PROTOCOL_VERSION;

                u8 packetBuf[Constants::MAX_PACKET_SIZE];
                size_t packetSize = NetworkProtocol::BuildPacket(
                    PacketType::ConnectRequest,
                    m_outgoingSequence++,
                    &req,
                    sizeof(req),
                    packetBuf,
                    sizeof(packetBuf));
                if (packetSize > 0) {
                    m_socket->SendTo(packetBuf, packetSize, m_serverAddress);
                }
            }
        } else if (m_state == ConnectionState::Connected) {
            // Periodic 30 Hz flight snapshot transmission
            if (simTime - m_lastSnapshotSendTime >= Constants::SNAPSHOT_SEND_INTERVAL_S) {
                m_lastSnapshotSendTime = simTime;
                SendClientSnapshot(simTime, localState, localControls, localEngineRpm, anyGearInContact);
            }

            // Periodic 1 Hz heartbeat ping
            if (simTime - m_lastHeartbeatSendTime >= Constants::HEARTBEAT_INTERVAL_S) {
                m_lastHeartbeatSendTime = simTime;
                SendPing(simTime);
            }

            // Detect server connection timeout
            if (m_lastServerPacketSimTime > 0.0 && (simTime - m_lastServerPacketSimTime) > Constants::TIMEOUT_INTERVAL_S) {
                m_state = ConnectionState::TimedOut;
            }
        }
    } else if (m_role == NetworkRole::Server) {
        // Broadcast world snapshot at 30 Hz to all connected clients
        if (simTime - m_lastSnapshotSendTime >= Constants::SNAPSHOT_SEND_INTERVAL_S) {
            m_lastSnapshotSendTime = simTime;
            BroadcastServerWorldSnapshot(simTime, localState, localControls, localEngineRpm, anyGearInContact);
        }

        // Timeout inactive clients
        m_clients.erase(
            std::remove_if(m_clients.begin(), m_clients.end(), [&](const ClientSession& c) {
                return (simTime - c.lastSeenSimTime) > Constants::TIMEOUT_INTERVAL_S;
            }),
            m_clients.end());
    }

    // Update entity interpolation and dead reckoning on all remote aircraft
    for (auto& peer : m_remoteAircraft) {
        peer.Update(simTime, dt, Constants::INTERPOLATION_DELAY_S);
    }

    // Prune expired remote aircraft entities
    m_remoteAircraft.erase(
        std::remove_if(m_remoteAircraft.begin(), m_remoteAircraft.end(), [&](const RemoteAircraft& r) {
            return r.IsExpired(simTime, Constants::TIMEOUT_INTERVAL_S);
        }),
        m_remoteAircraft.end());

    m_stats.connectedPeersCount = static_cast<u32>(
        (m_role == NetworkRole::Server) ? m_clients.size() : m_remoteAircraft.size());
}

std::wstring NetworkSystem::GetStatusString() const {
    std::wstringstream ss;
    switch (m_role) {
    case NetworkRole::Offline:
        ss << L"OFF";
        break;
    case NetworkRole::Server:
        ss << L"HOST (" << m_clients.size() << L" client" << (m_clients.size() == 1 ? L"" : L"s") << L")";
        break;
    case NetworkRole::Client:
        if (m_state == ConnectionState::Connecting) {
            ss << L"CONNECTING...";
        } else if (m_state == ConnectionState::Connected) {
            ss << L"CLIENT (" << m_remoteAircraft.size() << L" peer" << (m_remoteAircraft.size() == 1 ? L"" : L"s")
               << L", " << std::fixed << std::setprecision(0) << m_stats.roundTripTimeMs << L"ms)";
        } else if (m_state == ConnectionState::TimedOut) {
            ss << L"TIMED OUT";
        } else {
            ss << L"DISCONNECTED";
        }
        break;
    }
    return ss.str();
}

} // namespace LokYS::Network
