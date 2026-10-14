#pragma once

#include "LokYS/network/NetworkTypes.hpp"
#include <cstring>
#include <cstdint>

namespace LokYS::Network {

#pragma pack(push, 1)
struct PacketHeader {
    u32 magic{ Constants::PROTOCOL_MAGIC };
    u8  version{ Constants::PROTOCOL_VERSION };
    u8  type{ 0 };
    u16 payloadSize{ 0 };
    u16 crc16{ 0 };
    u16 sequence{ 0 };
};

struct ConnectRequestPayload {
    char callsign[Constants::MAX_CALLSIGN_LEN]{ 0 };
    u32  clientVersion{ Constants::PROTOCOL_VERSION };
};

struct ConnectAcceptPayload {
    u32 assignedClientId{ 0 };
    u32 serverTick{ 0 };
    f64 serverTime{ 0.0 };
};

struct PingPayload {
    u32 pingId{ 0 };
    f64 clientSendTime{ 0.0 };
};

struct PongPayload {
    u32 pingId{ 0 };
    f64 clientSendTime{ 0.0 };
    f64 serverReceiveTime{ 0.0 };
};

struct WorldSnapshotHeader {
    u32 serverSequence{ 0 };
    f64 serverTime{ 0.0 };
    u16 entityCount{ 0 };
};
#pragma pack(pop)

class NetworkProtocol {
public:
    /// @brief Computes standard CRC-16-CCITT (polynomial 0x1021, init 0xFFFF).
    [[nodiscard]] static u16 ComputeCrc16(const void* data, size_t length) noexcept {
        const u8* bytes = static_cast<const u8*>(data);
        u16 crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<u16>(bytes[i]) << 8;
            for (int bit = 0; bit < 8; ++bit) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc = crc << 1;
                }
            }
        }
        return crc;
    }

    /// @brief Serializes a packet into a destination buffer with header and CRC.
    /// @return Number of bytes written, or 0 on error / buffer overflow.
    static size_t BuildPacket(
        PacketType type,
        u16 sequence,
        const void* payload,
        u16 payloadSize,
        void* outBuffer,
        size_t maxBufferSize) noexcept
    {
        if (!outBuffer || payloadSize > Constants::MAX_PACKET_SIZE) {
            return 0;
        }

        size_t totalSize = sizeof(PacketHeader) + payloadSize;
        if (totalSize > maxBufferSize) {
            return 0;
        }

        PacketHeader header{};
        header.magic = Constants::PROTOCOL_MAGIC;
        header.version = Constants::PROTOCOL_VERSION;
        header.type = static_cast<u8>(type);
        header.payloadSize = payloadSize;
        header.sequence = sequence;
        header.crc16 = 0; // zeroed for CRC computation

        // Copy header and payload into buffer
        u8* dest = static_cast<u8*>(outBuffer);
        std::memcpy(dest, &header, sizeof(PacketHeader));
        if (payload && payloadSize > 0) {
            std::memcpy(dest + sizeof(PacketHeader), payload, payloadSize);
        }

        // Calculate CRC over entire packet (excluding crc16 field itself)
        // For simplicity and speed: compute CRC over the payload
        u16 crc = (payload && payloadSize > 0) ? ComputeCrc16(payload, payloadSize) : 0;
        PacketHeader* finalHeader = reinterpret_cast<PacketHeader*>(dest);
        finalHeader->crc16 = crc;

        return totalSize;
    }

    /// @brief Validates magic, version, payload size, and CRC of an incoming raw buffer.
    /// @return True if valid, false otherwise.
    [[nodiscard]] static bool ParsePacket(
        const void* inBuffer,
        size_t bytesRead,
        PacketHeader& outHeader,
        const u8*& outPayload) noexcept
    {
        if (!inBuffer || bytesRead < sizeof(PacketHeader)) {
            return false;
        }

        const u8* bytes = static_cast<const u8*>(inBuffer);
        std::memcpy(&outHeader, bytes, sizeof(PacketHeader));

        if (outHeader.magic != Constants::PROTOCOL_MAGIC) {
            return false;
        }
        if (outHeader.version != Constants::PROTOCOL_VERSION) {
            return false;
        }
        if (sizeof(PacketHeader) + outHeader.payloadSize > bytesRead) {
            return false;
        }

        outPayload = bytes + sizeof(PacketHeader);

        if (outHeader.payloadSize > 0) {
            u16 computedCrc = ComputeCrc16(outPayload, outHeader.payloadSize);
            if (computedCrc != outHeader.crc16) {
                return false; // CRC mismatch / corrupted packet
            }
        }

        return true;
    }
};

} // namespace LokYS::Network
