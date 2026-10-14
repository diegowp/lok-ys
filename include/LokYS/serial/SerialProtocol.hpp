#pragma once

#include "LokYS/serial/SerialTypes.hpp"
#include <cstring>
#include <functional>

namespace LokYS::Serial {

/// @brief Computes standard CRC-16-CCITT (Polynomial 0x1021, Initial 0xFFFF)
[[nodiscard]] constexpr u16 CalculateCrc16(const u8* data, size_t length, u16 initial = 0xFFFF) noexcept {
    u16 crc = initial;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<u16>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

[[nodiscard]] constexpr bool VerifyCrc16(const u8* data, size_t length, u16 expectedCrc) noexcept {
    return CalculateCrc16(data, length) == expectedCrc;
}

/// @brief Encodes a message type and payload into a binary framed packet.
/// Frame format: [0x55] [0xAA] [MsgId] [PayloadLen] [Payload...] [CRC_H] [CRC_L]
/// @return Total frame length in bytes, or 0 if buffer is insufficient.
inline size_t EncodePacket(
    SerialMessageType type,
    const void* payload,
    u8 payloadLength,
    u8* outBuffer,
    size_t outBufferSize) noexcept
{
    const size_t totalSize = SERIAL_HEADER_SIZE + payloadLength + SERIAL_CRC_SIZE;
    if (outBuffer == nullptr || outBufferSize < totalSize) {
        return 0;
    }

    outBuffer[0] = SERIAL_SYNC_BYTE_1;
    outBuffer[1] = SERIAL_SYNC_BYTE_2;
    outBuffer[2] = static_cast<u8>(type);
    outBuffer[3] = payloadLength;

    if (payload != nullptr && payloadLength > 0) {
        std::memcpy(&outBuffer[SERIAL_HEADER_SIZE], payload, payloadLength);
    }

    // CRC is calculated over MsgId + Length + Payload
    u16 crc = CalculateCrc16(&outBuffer[2], 2 + payloadLength);
    outBuffer[SERIAL_HEADER_SIZE + payloadLength] = static_cast<u8>((crc >> 8) & 0xFF);
    outBuffer[SERIAL_HEADER_SIZE + payloadLength + 1] = static_cast<u8>(crc & 0xFF);

    return totalSize;
}

struct DecodedPacket {
    SerialMessageType type{ SerialMessageType::Heartbeat };
    u8 payloadLength{ 0 };
    u8 payload[SERIAL_MAX_PAYLOAD_SIZE]{ 0 };

    template <typename T>
    [[nodiscard]] bool As(T& out) const noexcept {
        if (sizeof(T) != payloadLength) {
            return false;
        }
        std::memcpy(&out, payload, sizeof(T));
        return true;
    }
};

/// @brief Robust streaming parser for incoming byte streams with arbitrary chunking and noise recovery.
class SerialPacketDecoder {
public:
    enum class State {
        WaitingSync1,
        WaitingSync2,
        ReadingMsgId,
        ReadingLength,
        ReadingPayload,
        ReadingCrc1,
        ReadingCrc2
    };

    SerialPacketDecoder() = default;

    void Reset() noexcept {
        m_state = State::WaitingSync1;
        m_payloadIndex = 0;
        m_currentPacket = DecodedPacket{};
        m_crcHigh = 0;
    }

    template <typename Callback>
    void ProcessBytes(const u8* data, size_t length, Callback&& onPacket) noexcept {
        for (size_t i = 0; i < length; ++i) {
            ProcessByte(data[i], onPacket);
        }
    }

    [[nodiscard]] const SerialStats& GetStats() const noexcept { return m_stats; }
    void ResetStats() noexcept { m_stats = SerialStats{}; }

private:
    template <typename Callback>
    void ProcessByte(u8 b, Callback&& onPacket) noexcept {
        m_stats.bytesReceived++;

        switch (m_state) {
        case State::WaitingSync1:
            if (b == SERIAL_SYNC_BYTE_1) {
                m_state = State::WaitingSync2;
            }
            break;

        case State::WaitingSync2:
            if (b == SERIAL_SYNC_BYTE_2) {
                m_state = State::ReadingMsgId;
            } else if (b == SERIAL_SYNC_BYTE_1) {
                // Stay in WaitingSync2 in case of repeated 0x55
                m_state = State::WaitingSync2;
            } else {
                m_stats.framingErrors++;
                m_state = State::WaitingSync1;
            }
            break;

        case State::ReadingMsgId:
            m_currentPacket.type = static_cast<SerialMessageType>(b);
            m_state = State::ReadingLength;
            break;

        case State::ReadingLength:
            m_currentPacket.payloadLength = b;
            m_payloadIndex = 0;
            if (b == 0) {
                m_state = State::ReadingCrc1;
            } else {
                m_state = State::ReadingPayload;
            }
            break;

        case State::ReadingPayload:
            m_currentPacket.payload[m_payloadIndex++] = b;
            if (m_payloadIndex >= m_currentPacket.payloadLength) {
                m_state = State::ReadingCrc1;
            }
            break;

        case State::ReadingCrc1:
            m_crcHigh = b;
            m_state = State::ReadingCrc2;
            break;

        case State::ReadingCrc2: {
            u16 receivedCrc = (static_cast<u16>(m_crcHigh) << 8) | static_cast<u16>(b);

            // Compute expected CRC over MsgId + Length + Payload
            u8 headerBytes[2] = {
                static_cast<u8>(m_currentPacket.type),
                m_currentPacket.payloadLength
            };
            u16 calculatedCrc = CalculateCrc16(headerBytes, 2);
            if (m_currentPacket.payloadLength > 0) {
                calculatedCrc = CalculateCrc16(
                    m_currentPacket.payload,
                    m_currentPacket.payloadLength,
                    calculatedCrc);
            }

            if (calculatedCrc == receivedCrc) {
                m_stats.packetsReceived++;
                onPacket(m_currentPacket);
            } else {
                m_stats.crcErrors++;
            }

            m_state = State::WaitingSync1;
            break;
        }
        }
    }

    State m_state{ State::WaitingSync1 };
    u8 m_payloadIndex{ 0 };
    u8 m_crcHigh{ 0 };
    DecodedPacket m_currentPacket{};
    SerialStats m_stats{};
};

} // namespace LokYS::Serial
