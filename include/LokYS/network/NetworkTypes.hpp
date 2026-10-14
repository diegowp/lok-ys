#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include <string>
#include <vector>
#include <array>

namespace LokYS::Network {

enum class NetworkRole : u8 {
    Offline = 0,
    Server  = 1,
    Client  = 2
};

enum class ConnectionState : u8 {
    Disconnected = 0,
    Connecting   = 1,
    Connected    = 2,
    TimedOut     = 3
};

enum class PacketType : u8 {
    ConnectRequest = 0x01,
    ConnectAccept  = 0x02,
    ConnectReject  = 0x03,
    Disconnect     = 0x04,
    Ping           = 0x05,
    Pong           = 0x06,
    ClientSnapshot = 0x10,
    WorldSnapshot  = 0x20
};

namespace Constants {
inline constexpr u32 PROTOCOL_MAGIC           = 0x4E53594C; // "LYSN" in little endian
inline constexpr u8  PROTOCOL_VERSION         = 1;
inline constexpr u16 DEFAULT_PORT             = 49152;
inline constexpr u16 MAX_PACKET_SIZE          = 1400;
inline constexpr u32 MAX_PEERS                = 16;
inline constexpr u32 MAX_CALLSIGN_LEN         = 16;
inline constexpr f64 SNAPSHOT_SEND_INTERVAL_S = 1.0 / 30.0; // 30 Hz
inline constexpr f64 HEARTBEAT_INTERVAL_S     = 1.0;
inline constexpr f64 TIMEOUT_INTERVAL_S       = 4.0;
inline constexpr f64 INTERPOLATION_DELAY_S    = 0.080; // 80 ms
} // namespace Constants

struct NetworkAddress {
    std::string ip{ "127.0.0.1" };
    u16 port{ Constants::DEFAULT_PORT };

    bool operator==(const NetworkAddress& other) const noexcept {
        return port == other.port && ip == other.ip;
    }
    bool operator!=(const NetworkAddress& other) const noexcept {
        return !(*this == other);
    }
};

#pragma pack(push, 1)
struct AircraftFlightPacket {
    u32 entityId{ 0 };
    char callsign[16]{ 0 };
    u32 sequence{ 0 };
    f64 timestamp{ 0.0 };
    f64 posX{ 0.0 }, posY{ 0.0 }, posZ{ 0.0 };         // NED position (m)
    f32 velX{ 0.0f }, velY{ 0.0f }, velZ{ 0.0f };      // NED velocity (m/s)
    f32 rotW{ 1.0f }, rotX{ 0.0f }, rotY{ 0.0f }, rotZ{ 0.0f }; // Attitude quaternion
    f32 angVelP{ 0.0f }, angVelQ{ 0.0f }, angVelR{ 0.0f };       // Body angular rates (rad/s)
    f32 elevator{ 0.0f }, aileron{ 0.0f }, rudder{ 0.0f }, flaps{ 0.0f }; // Normalized controls
    f32 engineRpm{ 0.0f };
    u8  gearFlags{ 0 }; // bit 0: nose, bit 1: left main, bit 2: right main
    u8  pad[3]{ 0, 0, 0 };
};
#pragma pack(pop)

struct NetworkStats {
    u64 packetsSent{ 0 };
    u64 packetsReceived{ 0 };
    u64 bytesSent{ 0 };
    u64 bytesReceived{ 0 };
    f32 roundTripTimeMs{ 0.0f };
    f32 packetLossPercent{ 0.0f };
    u32 connectedPeersCount{ 0 };
};

} // namespace LokYS::Network
