#include "LokYS/core/Types.hpp"
#include "LokYS/network/NetworkTypes.hpp"
#include "LokYS/network/NetworkProtocol.hpp"
#include "LokYS/network/UdpSocket.hpp"
#include "LokYS/network/RemoteAircraft.hpp"
#include "LokYS/network/NetworkSystem.hpp"
#include <iostream>
#include <cmath>
#include <cstring>
#include <memory>

using namespace LokYS;
using namespace LokYS::Network;

namespace {

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << msg << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

#define TEST_NEAR(actual, expected, tol, msg) \
    do { \
        if (std::abs((actual) - (expected)) > (tol)) { \
            std::cerr << "[FAIL] " << msg << ": actual=" << (actual) \
                      << ", expected=" << (expected) << " (diff=" << std::abs((actual) - (expected)) \
                      << " > " << (tol) << ") at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

bool TestPacketSerializationAndCrc() {
    std::cout << "[RUNNING] TestPacketSerializationAndCrc...\n";

    ConnectRequestPayload req{};
    strncpy_s(req.callsign, "N172SP", _TRUNCATE);
    req.clientVersion = Constants::PROTOCOL_VERSION;

    u8 buffer[Constants::MAX_PACKET_SIZE];
    size_t packetSize = NetworkProtocol::BuildPacket(
        PacketType::ConnectRequest,
        42,
        &req,
        sizeof(req),
        buffer,
        sizeof(buffer));

    TEST_ASSERT(packetSize == sizeof(PacketHeader) + sizeof(ConnectRequestPayload), "Packet size must match header + payload");

    PacketHeader header{};
    const u8* payload = nullptr;
    bool valid = NetworkProtocol::ParsePacket(buffer, packetSize, header, payload);
    TEST_ASSERT(valid, "Clean packet must parse successfully");
    TEST_ASSERT(header.magic == Constants::PROTOCOL_MAGIC, "Magic must match LYSN");
    TEST_ASSERT(header.version == Constants::PROTOCOL_VERSION, "Version must match");
    TEST_ASSERT(header.type == static_cast<u8>(PacketType::ConnectRequest), "Packet type must match ConnectRequest");
    TEST_ASSERT(header.sequence == 42, "Sequence must match 42");

    const auto* parsedReq = reinterpret_cast<const ConnectRequestPayload*>(payload);
    TEST_ASSERT(std::strcmp(parsedReq->callsign, "N172SP") == 0, "Callsign must match");

    // Corrupt one byte of payload to test CRC validation
    buffer[sizeof(PacketHeader) + 2] ^= 0xFF;
    bool corruptValid = NetworkProtocol::ParsePacket(buffer, packetSize, header, payload);
    TEST_ASSERT(!corruptValid, "Corrupted packet must fail CRC verification");

    std::cout << "[PASSED] TestPacketSerializationAndCrc\n";
    return true;
}

bool TestClientServerConnectHandshake() {
    std::cout << "[RUNNING] TestClientServerConnectHandshake...\n";

    // 1. Client builds ConnectRequest
    ConnectRequestPayload req{};
    strncpy_s(req.callsign, "BRAVO1", _TRUNCATE);
    req.clientVersion = Constants::PROTOCOL_VERSION;

    u8 reqBuf[256];
    size_t reqSize = NetworkProtocol::BuildPacket(PacketType::ConnectRequest, 1, &req, sizeof(req), reqBuf, sizeof(reqBuf));
    TEST_ASSERT(reqSize > 0, "ConnectRequest serialization failed");

    // 2. Server parses ConnectRequest and replies with ConnectAccept
    PacketHeader parsedReqHdr{};
    const u8* parsedReqPayload = nullptr;
    TEST_ASSERT(NetworkProtocol::ParsePacket(reqBuf, reqSize, parsedReqHdr, parsedReqPayload), "Server failed to parse request");

    ConnectAcceptPayload accept{};
    accept.assignedClientId = 7;
    accept.serverTick = 100;
    accept.serverTime = 12.345;

    u8 acceptBuf[256];
    size_t acceptSize = NetworkProtocol::BuildPacket(PacketType::ConnectAccept, 100, &accept, sizeof(accept), acceptBuf, sizeof(acceptBuf));
    TEST_ASSERT(acceptSize > 0, "ConnectAccept serialization failed");

    // 3. Client parses ConnectAccept
    PacketHeader parsedAccHdr{};
    const u8* parsedAccPayload = nullptr;
    TEST_ASSERT(NetworkProtocol::ParsePacket(acceptBuf, acceptSize, parsedAccHdr, parsedAccPayload), "Client failed to parse accept");
    const auto* acc = reinterpret_cast<const ConnectAcceptPayload*>(parsedAccPayload);
    TEST_ASSERT(acc->assignedClientId == 7, "Assigned client ID must be 7");
    TEST_NEAR(acc->serverTime, 12.345, 1e-4, "Server timestamp match");

    std::cout << "[PASSED] TestClientServerConnectHandshake\n";
    return true;
}

bool TestFlightSnapshotPackingAndUnpacking() {
    std::cout << "[RUNNING] TestFlightSnapshotPackingAndUnpacking...\n";

    AircraftFlightPacket original{};
    original.entityId = 42;
    strncpy_s(original.callsign, "C-FGAF", _TRUNCATE);
    original.sequence = 1234;
    original.timestamp = 45.678;
    original.posX = -800.25;
    original.posY = 150.50;
    original.posZ = -914.40;
    original.velX = 48.5f;
    original.velY = 2.1f;
    original.velZ = -0.5f;
    original.rotW = 0.92388f;
    original.rotX = 0.0f;
    original.rotY = 0.38268f;
    original.rotZ = 0.0f;
    original.angVelP = 0.02f;
    original.angVelQ = -0.01f;
    original.angVelR = 0.005f;
    original.elevator = 0.15f;
    original.aileron = -0.08f;
    original.rudder = 0.04f;
    original.flaps = 0.333f;
    original.engineRpm = 2450.0f;
    original.gearFlags = 0x07;

    u8 buf[512];
    size_t size = NetworkProtocol::BuildPacket(PacketType::ClientSnapshot, 1234, &original, sizeof(original), buf, sizeof(buf));
    TEST_ASSERT(size > 0, "Failed to serialize ClientSnapshot");

    PacketHeader hdr{};
    const u8* payload = nullptr;
    TEST_ASSERT(NetworkProtocol::ParsePacket(buf, size, hdr, payload), "Failed to parse ClientSnapshot");

    const auto* unpacked = reinterpret_cast<const AircraftFlightPacket*>(payload);
    TEST_ASSERT(unpacked->entityId == 42, "Entity ID match");
    TEST_ASSERT(std::strcmp(unpacked->callsign, "C-FGAF") == 0, "Callsign match");
    TEST_NEAR(unpacked->posX, -800.25, 1e-4, "PosX match");
    TEST_NEAR(unpacked->posY, 150.50, 1e-4, "PosY match");
    TEST_NEAR(unpacked->posZ, -914.40, 1e-4, "PosZ match");
    TEST_NEAR(unpacked->velX, 48.5f, 1e-4, "VelX match");
    TEST_NEAR(unpacked->rotW, 0.92388f, 1e-4, "RotW match");
    TEST_NEAR(unpacked->engineRpm, 2450.0f, 1e-2, "Engine RPM match");
    TEST_ASSERT(unpacked->gearFlags == 0x07, "Gear flags match");

    std::cout << "[PASSED] TestFlightSnapshotPackingAndUnpacking\n";
    return true;
}

bool TestHermiteCubicSplinePositionInterpolation() {
    std::cout << "[RUNNING] TestHermiteCubicSplinePositionInterpolation...\n";

    Math::Vec3d p0(0.0, 0.0, 0.0);
    Math::Vec3d v0(10.0, 0.0, 0.0);
    Math::Vec3d p1(100.0, 0.0, 0.0);
    Math::Vec3d v1(20.0, 0.0, 0.0);
    f64 dt = 2.0; // 2 seconds between snapshots

    // Boundary condition 1: s = 0 must exactly equal p0
    Math::Vec3d at0 = RemoteAircraft::HermiteInterpolatePosition(p0, v0, p1, v1, dt, 0.0);
    TEST_NEAR(at0.x, 0.0, 1e-6, "Hermite at s=0 must be p0");

    // Boundary condition 2: s = 1 must exactly equal p1
    Math::Vec3d at1 = RemoteAircraft::HermiteInterpolatePosition(p0, v0, p1, v1, dt, 1.0);
    TEST_NEAR(at1.x, 100.0, 1e-6, "Hermite at s=1 must be p1");

    // Midpoint analytical check:
    // h0(0.5) = 2(0.125) - 3(0.25) + 1 = 0.5
    // h1(0.5) = 0.125 - 2(0.25) + 0.5 = 0.125
    // h2(0.5) = -2(0.125) + 3(0.25) = 0.5
    // h3(0.5) = 0.125 - 0.25 = -0.125
    // P(0.5) = 0.5*0 + 0.125*2*10 + 0.5*100 + (-0.125)*2*20 = 0 + 2.5 + 50 - 5.0 = 47.5
    Math::Vec3d atHalf = RemoteAircraft::HermiteInterpolatePosition(p0, v0, p1, v1, dt, 0.5);
    TEST_NEAR(atHalf.x, 47.5, 1e-4, "Hermite midpoint match analytical 47.5");

    std::cout << "[PASSED] TestHermiteCubicSplinePositionInterpolation\n";
    return true;
}

bool TestQuaternionSlerpAttitudeInterpolation() {
    std::cout << "[RUNNING] TestQuaternionSlerpAttitudeInterpolation...\n";

    // Heading 000 (North): q0 = Identity
    Math::Quat4d q0 = Math::Quat4d::Identity();

    // Heading 090 (East): 90 deg rotation around Z axis (yaw)
    Math::Quat4d q1 = Math::Quat4d::FromEuler(3.141592653589793 * 0.5, 0.0, 0.0);

    // SLERP at midpoint s = 0.5 must produce exactly Heading 045 (45 deg)
    Math::Quat4d qMid = Math::Quat4d::Slerp(q0, q1, 0.5);
    TEST_NEAR(qMid.Norm(), 1.0, 1e-6, "SLERP result must be normalized unit quaternion");

    f64 yaw = 0.0, pitch = 0.0, roll = 0.0;
    qMid.ToEuler(yaw, pitch, roll);

    f64 expectedYawRad = 3.141592653589793 * 0.25; // 45 deg
    TEST_NEAR(yaw, expectedYawRad, 1e-4, "SLERP midpoint yaw must be 45 degrees");
    TEST_NEAR(pitch, 0.0, 1e-4, "Pitch should remain 0");
    TEST_NEAR(roll, 0.0, 1e-4, "Roll should remain 0");

    std::cout << "[PASSED] TestQuaternionSlerpAttitudeInterpolation\n";
    return true;
}

bool TestProjectiveDeadReckoningExtrapolation() {
    std::cout << "[RUNNING] TestProjectiveDeadReckoningExtrapolation...\n";

    RemoteAircraft remote(10, "G-BDRK");

    // Push two snapshots:
    // s0 at t = 1.0, pos = (0, 0, -500), vel = (50, 0, 0)
    AircraftFlightPacket s0{};
    s0.entityId = 10;
    s0.timestamp = 1.0;
    s0.posX = 0.0; s0.posY = 0.0; s0.posZ = -500.0;
    s0.velX = 50.0f; s0.velY = 0.0f; s0.velZ = 0.0f;
    s0.rotW = 1.0f;
    s0.engineRpm = 2300.0f;
    remote.PushSnapshot(s0);

    // s1 at t = 1.1, pos = (5, 0, -500), vel = (50, 0, 0)
    AircraftFlightPacket s1{};
    s1.entityId = 10;
    s1.timestamp = 1.1;
    s1.posX = 5.0; s1.posY = 0.0; s1.posZ = -500.0;
    s1.velX = 50.0f; s1.velY = 0.0f; s1.velZ = 0.0f;
    s1.rotW = 1.0f;
    s1.engineRpm = 2300.0f;
    remote.PushSnapshot(s1);

    // 1. Interpolation test: targetTime = 1.05 (between 1.0 and 1.1)
    // with interpDelay = 0.05, currentSimTime = 1.10 -> targetTime = 1.05
    remote.Update(1.10, 0.016, 0.05);
    TEST_ASSERT(!remote.IsDeadReckoning(), "Bracketed update must be interpolating, not dead reckoning");
    TEST_NEAR(remote.GetPositionNed().x, 2.5, 0.1, "Position at midpoint should be ~2.5m");

    // 2. Extrapolation / Dead Reckoning test:
    // No new packet arrives, time advances to 1.30 (targetTime = 1.25, 150ms ahead of s1 at 1.1)
    remote.Update(1.30, 0.016, 0.05);
    TEST_ASSERT(remote.IsDeadReckoning(), "Update beyond newest packet must flag dead reckoning");
    // Expected position: 5.0 + 50.0 * (1.25 - 1.10) = 5.0 + 7.5 = 12.5m
    TEST_NEAR(remote.GetPositionNed().x, 12.5, 0.1, "Dead reckoning projective extrapolation x");

    // 3. Expiration test
    TEST_ASSERT(!remote.IsExpired(2.0, 4.0), "Entity should not be expired at t=2.0 (last=1.1, timeout=4.0)");
    TEST_ASSERT(remote.IsExpired(6.0, 4.0), "Entity must be expired at t=6.0 (diff 4.9s > 4.0s)");

    std::cout << "[PASSED] TestProjectiveDeadReckoningExtrapolation\n";
    return true;
}

bool TestLoopbackClientServerReplicationWithMockChannel() {
    std::cout << "[RUNNING] TestLoopbackClientServerReplicationWithMockChannel...\n";

    auto hub = std::make_shared<MockNetworkHub>();

    // Server socket on port 49152 with 5ms simulated latency
    auto serverSock = std::make_unique<MockUdpSocket>(hub, 0.005, 0.0);
    serverSock->Open(49152);

    // Client socket with 5ms simulated latency
    auto clientSock = std::make_unique<MockUdpSocket>(hub, 0.005, 0.0);
    clientSock->Open(51000);

    // Raw pointers for time advancement
    MockUdpSocket* pServerSock = serverSock.get();
    MockUdpSocket* pClientSock = clientSock.get();

    NetworkSystem server;
    server.SetCustomSocket(std::move(serverSock));
    TEST_ASSERT(server.StartServer(49152), "Server failed to start");
    TEST_ASSERT(server.GetRole() == NetworkRole::Server, "Role must be Server");

    NetworkSystem client;
    client.SetCustomSocket(std::move(clientSock));
    TEST_ASSERT(client.Connect("127.0.0.1", 49152, "EAGLE1"), "Client failed to connect");
    TEST_ASSERT(client.GetRole() == NetworkRole::Client, "Role must be Client");
    TEST_ASSERT(client.GetState() == ConnectionState::Connecting, "State must be Connecting");

    Physics::RigidBodyState hostState{};
    hostState.positionNed = { 0.0, 0.0, -100.0 };
    Input::FlightControls hostCtrl{};

    Physics::RigidBodyState clientState{};
    clientState.positionNed = { -500.0, 20.0, -300.0 };
    Input::FlightControls clientCtrl{};
    clientCtrl.pitch = 0.25;

    // Simulate stepping forward in 10ms intervals
    f64 simTime = 0.0;
    f64 dt = 0.010;

    for (int step = 0; step < 20; ++step) {
        simTime += dt;
        pServerSock->SetSimulatedTime(simTime);
        pClientSock->SetSimulatedTime(simTime);

        server.Update(dt, simTime, hostState, hostCtrl, 2200.0, true);
        client.Update(dt, simTime, clientState, clientCtrl, 2400.0, false);
    }

    // 1. Verify Client is now Connected and assigned client ID
    TEST_ASSERT(client.GetState() == ConnectionState::Connected, "Client should have completed handshake");
    TEST_ASSERT(client.GetLocalEntityId() > 0, "Client entity ID should be assigned (> 0)");

    // 2. Step another 20 frames for flight snapshot replication
    for (int step = 0; step < 20; ++step) {
        simTime += dt;
        pServerSock->SetSimulatedTime(simTime);
        pClientSock->SetSimulatedTime(simTime);

        server.Update(dt, simTime, hostState, hostCtrl, 2200.0, true);
        client.Update(dt, simTime, clientState, clientCtrl, 2400.0, false);
    }

    // 3. Verify Server sees Client as a remote aircraft
    const auto& serverPeers = server.GetRemoteAircraft();
    TEST_ASSERT(!serverPeers.empty(), "Server must register connected client as remote aircraft");
    TEST_ASSERT(serverPeers.front().GetCallsign() == "EAGLE1", "Server peer callsign must be EAGLE1");
    TEST_NEAR(serverPeers.front().GetPositionNed().x, -500.0, 1.0, "Client position X replicated on server");

    // 4. Verify Client sees Host as remote aircraft (entity 0)
    const auto& clientPeers = client.GetRemoteAircraft();
    TEST_ASSERT(!clientPeers.empty(), "Client must receive host world snapshot");
    TEST_ASSERT(clientPeers.front().GetEntityId() == 0, "Host entity ID must be 0");
    TEST_NEAR(clientPeers.front().GetPositionNed().z, -100.0, 1.0, "Host altitude replicated on client");

    std::cout << "[PASSED] TestLoopbackClientServerReplicationWithMockChannel\n";
    return true;
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Network Multiplayer Tests\n";
    std::cout << "============================================================\n";

    bool allPassed = true;
    allPassed &= TestPacketSerializationAndCrc();
    allPassed &= TestClientServerConnectHandshake();
    allPassed &= TestFlightSnapshotPackingAndUnpacking();
    allPassed &= TestHermiteCubicSplinePositionInterpolation();
    allPassed &= TestQuaternionSlerpAttitudeInterpolation();
    allPassed &= TestProjectiveDeadReckoningExtrapolation();
    allPassed &= TestLoopbackClientServerReplicationWithMockChannel();

    std::cout << "============================================================\n";
    if (allPassed) {
        std::cout << "[ALL 7 NETWORK MULTIPLAYER TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cerr << "[ONE OR MORE NETWORK MULTIPLAYER TESTS FAILED]\n";
        return 1;
    }
}
