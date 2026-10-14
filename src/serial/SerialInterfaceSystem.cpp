#include "LokYS/serial/SerialInterfaceSystem.hpp"
#include <chrono>

namespace LokYS::Serial {

SerialInterfaceSystem::SerialInterfaceSystem() = default;

SerialInterfaceSystem::~SerialInterfaceSystem() {
    Shutdown();
}

bool SerialInterfaceSystem::Initialize(const SerialConfig& config) {
    Shutdown();
    m_config = config;

    if (config.mockLoopback) {
        m_port.SetMockLoopback(true);
    }

    if (!m_port.Open(config.portName, config.baudRate)) {
        return false;
    }

    m_running.store(true, std::memory_order_release);
    m_workerThread = std::thread(&SerialInterfaceSystem::WorkerLoop, this);
    return true;
}

void SerialInterfaceSystem::Shutdown() {
    if (m_running.load(std::memory_order_acquire)) {
        m_running.store(false, std::memory_order_release);
        if (m_workerThread.joinable()) {
            m_workerThread.join();
        }
    }
    m_port.Close();
    m_hasInboundControls.store(false, std::memory_order_release);
}

bool SerialInterfaceSystem::IsConnected() const noexcept {
    return m_port.IsOpen();
}

SerialStats SerialInterfaceSystem::GetStats() const {
    SerialStats s = m_decoder.GetStats();
    s.bytesSent = m_stats.bytesSent;
    s.packetsSent = m_stats.packetsSent;
    return s;
}

bool SerialInterfaceSystem::QueueOutboundPacket(SerialMessageType type, const void* payload, u8 length) {
    u8 frameBuffer[SERIAL_MAX_FRAME_SIZE];
    size_t frameLen = EncodePacket(type, payload, length, frameBuffer, sizeof(frameBuffer));
    if (frameLen == 0) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_txMutex);
        m_txPendingBytes.insert(m_txPendingBytes.end(), frameBuffer, frameBuffer + frameLen);
    }

    m_stats.packetsSent++;
    return true;
}

bool SerialInterfaceSystem::SendFlightData(const TelemetryFlightDataPayload& payload) {
    return QueueOutboundPacket(SerialMessageType::TelemetryFlightData, &payload, sizeof(payload));
}

bool SerialInterfaceSystem::SendEngine(const TelemetryEnginePayload& payload) {
    return QueueOutboundPacket(SerialMessageType::TelemetryEngine, &payload, sizeof(payload));
}

bool SerialInterfaceSystem::SendNav(const TelemetryNavPayload& payload) {
    return QueueOutboundPacket(SerialMessageType::TelemetryNav, &payload, sizeof(payload));
}

bool SerialInterfaceSystem::SendAnnunciators(const TelemetryAnnunciatorsPayload& payload) {
    return QueueOutboundPacket(SerialMessageType::TelemetryAnnunciators, &payload, sizeof(payload));
}

bool SerialInterfaceSystem::SendHeartbeat() {
    HeartbeatPayload hb{};
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    hb.timestampMs = static_cast<u32>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    return QueueOutboundPacket(SerialMessageType::Heartbeat, &hb, sizeof(hb));
}

InboundFlightControlsPayload SerialInterfaceSystem::GetLatestFlightControls() {
    std::lock_guard<std::mutex> lock(m_rxMutex);
    return m_latestControls;
}

InboundSwitchesPayload SerialInterfaceSystem::GetLatestSwitches() {
    std::lock_guard<std::mutex> lock(m_rxMutex);
    return m_latestSwitches;
}

InboundNavControlsPayload SerialInterfaceSystem::ConsumeNavControls() {
    std::lock_guard<std::mutex> lock(m_rxMutex);
    InboundNavControlsPayload result = m_accumulatedNav;
    m_accumulatedNav = InboundNavControlsPayload{};
    return result;
}

void SerialInterfaceSystem::DispatchPacket(const DecodedPacket& packet) {
    switch (packet.type) {
    case SerialMessageType::InboundFlightControls: {
        InboundFlightControlsPayload ctrl{};
        if (packet.As(ctrl)) {
            std::lock_guard<std::mutex> lock(m_rxMutex);
            m_latestControls = ctrl;
            m_hasInboundControls.store(true, std::memory_order_release);
        }
        break;
    }
    case SerialMessageType::InboundSwitches: {
        InboundSwitchesPayload sw{};
        if (packet.As(sw)) {
            std::lock_guard<std::mutex> lock(m_rxMutex);
            m_latestSwitches = sw;
        }
        break;
    }
    case SerialMessageType::InboundNavControls: {
        InboundNavControlsPayload nav{};
        if (packet.As(nav)) {
            std::lock_guard<std::mutex> lock(m_rxMutex);
            m_accumulatedNav.navSwapTriggered |= nav.navSwapTriggered;
            m_accumulatedNav.navIdentTriggered |= nav.navIdentTriggered;
            m_accumulatedNav.obsDeltaTicks = static_cast<i8>(m_accumulatedNav.obsDeltaTicks + nav.obsDeltaTicks);
            m_accumulatedNav.freqDeltaTicks = static_cast<i8>(m_accumulatedNav.freqDeltaTicks + nav.freqDeltaTicks);
        }
        break;
    }
    case SerialMessageType::Heartbeat:
    case SerialMessageType::TelemetryFlightData:
    case SerialMessageType::TelemetryEngine:
    case SerialMessageType::TelemetryNav:
    case SerialMessageType::TelemetryAnnunciators:
    case SerialMessageType::CommandConfigure:
        // Handled or echoed by test loopbacks
        break;
    }
}

void SerialInterfaceSystem::ProcessSynchronous() {
    // 1. Flush outbound transmission
    std::vector<u8> toSend;
    {
        std::lock_guard<std::mutex> lock(m_txMutex);
        if (!m_txPendingBytes.empty()) {
            toSend = std::move(m_txPendingBytes);
            m_txPendingBytes.clear();
        }
    }

    if (!toSend.empty()) {
        size_t written = m_port.Write(toSend.data(), toSend.size());
        m_stats.bytesSent += written;
    }

    // 2. Read and decode inbound data
    u8 rxBuffer[512];
    size_t bytesRead = m_port.Read(rxBuffer, sizeof(rxBuffer));
    if (bytesRead > 0) {
        m_decoder.ProcessBytes(rxBuffer, bytesRead, [this](const DecodedPacket& packet) {
            DispatchPacket(packet);
        });
    }
}

void SerialInterfaceSystem::WorkerLoop() {
    while (m_running.load(std::memory_order_relaxed)) {
        ProcessSynchronous();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

} // namespace LokYS::Serial
