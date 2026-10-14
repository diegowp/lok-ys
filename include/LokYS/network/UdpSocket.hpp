#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/network/NetworkTypes.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

namespace LokYS::Network {

class INetworkSocket {
public:
    virtual ~INetworkSocket() = default;
    virtual bool Open(u16 port = 0) = 0;
    virtual void Close() = 0;
    virtual bool SendTo(const void* data, size_t size, const NetworkAddress& target) = 0;
    virtual int ReceiveFrom(void* buffer, size_t maxBufferSize, NetworkAddress& outSender) = 0;
    [[nodiscard]] virtual bool IsOpen() const noexcept = 0;
    [[nodiscard]] virtual u16 GetBoundPort() const noexcept = 0;
};

class UdpSocket final : public INetworkSocket {
public:
    UdpSocket();
    ~UdpSocket() override;

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;

    bool Open(u16 port = 0) override;
    void Close() override;
    bool SendTo(const void* data, size_t size, const NetworkAddress& target) override;
    int ReceiveFrom(void* buffer, size_t maxBufferSize, NetworkAddress& outSender) override;

    [[nodiscard]] bool IsOpen() const noexcept override { return m_socket != INVALID_SOCKET; }
    [[nodiscard]] u16 GetBoundPort() const noexcept override { return m_boundPort; }

    static bool InitializeWinsock();
    static void CleanupWinsock();

private:
    SOCKET m_socket{ INVALID_SOCKET };
    u16 m_boundPort{ 0 };
};

/// @brief In-memory simulated packet for deterministic MockUdpSocket channels.
struct MockPacket {
    NetworkAddress sender;
    NetworkAddress receiver;
    std::vector<u8> data;
    f64 deliveryTime{ 0.0 };
};

/// @brief In-memory mock network hub for automated unit testing without socket I/O.
class MockNetworkHub {
public:
    MockNetworkHub() = default;

    void RoutePacket(const MockPacket& packet) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_packets.push_back(packet);
    }

    bool FetchPacket(const NetworkAddress& receiver, f64 currentTime, MockPacket& outPacket) {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto it = m_packets.begin(); it != m_packets.end(); ++it) {
            if (it->receiver == receiver && currentTime >= it->deliveryTime) {
                outPacket = *it;
                m_packets.erase(it);
                return true;
            }
        }
        return false;
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_packets.clear();
    }

private:
    std::mutex m_mutex;
    std::vector<MockPacket> m_packets;
};

class MockUdpSocket final : public INetworkSocket {
public:
    explicit MockUdpSocket(std::shared_ptr<MockNetworkHub> hub, f64 latencySeconds = 0.0, f64 dropRate = 0.0);
    ~MockUdpSocket() override = default;

    bool Open(u16 port = 0) override;
    void Close() override;
    bool SendTo(const void* data, size_t size, const NetworkAddress& target) override;
    int ReceiveFrom(void* buffer, size_t maxBufferSize, NetworkAddress& outSender) override;

    [[nodiscard]] bool IsOpen() const noexcept override { return m_open; }
    [[nodiscard]] u16 GetBoundPort() const noexcept override { return m_boundPort; }

    void SetSimulatedTime(f64 time) noexcept { m_simulatedTime = time; }
    void AdvanceSimulatedTime(f64 dt) noexcept { m_simulatedTime += dt; }

private:
    std::shared_ptr<MockNetworkHub> m_hub;
    NetworkAddress m_localAddress;
    u16 m_boundPort{ 0 };
    f64 m_simulatedTime{ 0.0 };
    f64 m_latencySeconds{ 0.0 };
    f64 m_dropRate{ 0.0 };
    bool m_open{ false };
};

} // namespace LokYS::Network
