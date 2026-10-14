#include "LokYS/network/UdpSocket.hpp"
#include <atomic>
#include <cstdlib>

namespace LokYS::Network {

namespace {
std::atomic<int> s_winsockRefCount{ 0 };
}

bool UdpSocket::InitializeWinsock() {
    if (s_winsockRefCount.fetch_add(1) == 0) {
        WSADATA wsaData{};
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            s_winsockRefCount.fetch_sub(1);
            return false;
        }
    }
    return true;
}

void UdpSocket::CleanupWinsock() {
    if (s_winsockRefCount.fetch_sub(1) == 1) {
        WSACleanup();
    }
}

UdpSocket::UdpSocket() {
    InitializeWinsock();
}

UdpSocket::~UdpSocket() {
    Close();
    CleanupWinsock();
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : m_socket(other.m_socket)
    , m_boundPort(other.m_boundPort)
{
    other.m_socket = INVALID_SOCKET;
    other.m_boundPort = 0;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
    if (this != &other) {
        Close();
        m_socket = other.m_socket;
        m_boundPort = other.m_boundPort;
        other.m_socket = INVALID_SOCKET;
        other.m_boundPort = 0;
    }
    return *this;
}

bool UdpSocket::Open(u16 port) {
    Close();

    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_socket == INVALID_SOCKET) {
        return false;
    }

    // Configure non-blocking mode
    u_long nonBlockingMode = 1;
    if (ioctlsocket(m_socket, FIONBIO, &nonBlockingMode) != 0) {
        Close();
        return false;
    }

    // Set socket receive buffer size
    int rcvBufSize = 64 * 1024;
    setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvBufSize), sizeof(rcvBufSize));

    // Bind address
    sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(port);

    if (bind(m_socket, reinterpret_cast<const sockaddr*>(&localAddr), sizeof(localAddr)) != 0) {
        Close();
        return false;
    }

    // Query assigned bound port (in case port == 0 was requested)
    sockaddr_in boundAddr{};
    int addrLen = sizeof(boundAddr);
    if (getsockname(m_socket, reinterpret_cast<sockaddr*>(&boundAddr), &addrLen) == 0) {
        m_boundPort = ntohs(boundAddr.sin_port);
    } else {
        m_boundPort = port;
    }

    return true;
}

void UdpSocket::Close() {
    if (m_socket != INVALID_SOCKET) {
        closesocket(m_socket);
        m_socket = INVALID_SOCKET;
        m_boundPort = 0;
    }
}

bool UdpSocket::SendTo(const void* data, size_t size, const NetworkAddress& target) {
    if (m_socket == INVALID_SOCKET || !data || size == 0) {
        return false;
    }

    sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(target.port);

    if (inet_pton(AF_INET, target.ip.c_str(), &destAddr.sin_addr) <= 0) {
        return false;
    }

    int bytesSent = sendto(
        m_socket,
        static_cast<const char*>(data),
        static_cast<int>(size),
        0,
        reinterpret_cast<const sockaddr*>(&destAddr),
        sizeof(destAddr));

    return (bytesSent == static_cast<int>(size));
}

int UdpSocket::ReceiveFrom(void* buffer, size_t maxBufferSize, NetworkAddress& outSender) {
    if (m_socket == INVALID_SOCKET || !buffer || maxBufferSize == 0) {
        return -1;
    }

    sockaddr_in senderAddr{};
    int senderLen = sizeof(senderAddr);

    int bytesRead = recvfrom(
        m_socket,
        static_cast<char*>(buffer),
        static_cast<int>(maxBufferSize),
        0,
        reinterpret_cast<sockaddr*>(&senderAddr),
        &senderLen);

    if (bytesRead > 0) {
        char ipStr[INET_ADDRSTRLEN]{};
        inet_ntop(AF_INET, &senderAddr.sin_addr, ipStr, sizeof(ipStr));
        outSender.ip = ipStr;
        outSender.port = ntohs(senderAddr.sin_port);
        return bytesRead;
    }

    if (bytesRead == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAECONNRESET) {
            return 0; // No data available right now
        }
        return -1; // Socket error
    }

    return 0;
}

// ============================================================================
// MockUdpSocket Implementation
// ============================================================================

MockUdpSocket::MockUdpSocket(std::shared_ptr<MockNetworkHub> hub, f64 latencySeconds, f64 dropRate)
    : m_hub(std::move(hub))
    , m_latencySeconds(latencySeconds)
    , m_dropRate(dropRate)
{
}

bool MockUdpSocket::Open(u16 port) {
    m_boundPort = (port != 0) ? port : 50000;
    m_localAddress.ip = "127.0.0.1";
    m_localAddress.port = m_boundPort;
    m_open = true;
    return true;
}

void MockUdpSocket::Close() {
    m_open = false;
    m_boundPort = 0;
}

bool MockUdpSocket::SendTo(const void* data, size_t size, const NetworkAddress& target) {
    if (!m_open || !m_hub || !data || size == 0) {
        return false;
    }

    // Simulate packet drop
    if (m_dropRate > 0.0) {
        double r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
        if (r < m_dropRate) {
            return true; // Dropped in transit
        }
    }

    MockPacket packet;
    packet.sender = m_localAddress;
    packet.receiver = target;
    const u8* bytes = static_cast<const u8*>(data);
    packet.data.assign(bytes, bytes + size);
    packet.deliveryTime = m_simulatedTime + m_latencySeconds;

    m_hub->RoutePacket(packet);
    return true;
}

int MockUdpSocket::ReceiveFrom(void* buffer, size_t maxBufferSize, NetworkAddress& outSender) {
    if (!m_open || !m_hub || !buffer || maxBufferSize == 0) {
        return -1;
    }

    MockPacket packet;
    if (m_hub->FetchPacket(m_localAddress, m_simulatedTime, packet)) {
        size_t copyBytes = (packet.data.size() < maxBufferSize) ? packet.data.size() : maxBufferSize;
        std::memcpy(buffer, packet.data.data(), copyBytes);
        outSender = packet.sender;
        return static_cast<int>(copyBytes);
    }

    return 0; // No packets available at current simulated time
}

} // namespace LokYS::Network
