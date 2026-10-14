#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/serial/SerialTypes.hpp"
#include "LokYS/serial/SerialProtocol.hpp"
#include "LokYS/serial/SerialPort.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

namespace LokYS::Serial {

class SerialInterfaceSystem {
public:
    SerialInterfaceSystem();
    ~SerialInterfaceSystem();

    SerialInterfaceSystem(const SerialInterfaceSystem&) = delete;
    SerialInterfaceSystem& operator=(const SerialInterfaceSystem&) = delete;

    /// @brief Initializes serial port with specified configuration and launches background worker thread.
    bool Initialize(const SerialConfig& config);

    /// @brief Stops worker thread and closes serial communication.
    void Shutdown();

    [[nodiscard]] bool IsConnected() const noexcept;
    [[nodiscard]] const SerialConfig& GetConfig() const noexcept { return m_config; }
    [[nodiscard]] SerialPort& GetPort() noexcept { return m_port; }
    [[nodiscard]] SerialStats GetStats() const;

    // Outbound Telemetry Transmitters (Thread-Safe)
    bool SendFlightData(const TelemetryFlightDataPayload& payload);
    bool SendEngine(const TelemetryEnginePayload& payload);
    bool SendNav(const TelemetryNavPayload& payload);
    bool SendAnnunciators(const TelemetryAnnunciatorsPayload& payload);
    bool SendHeartbeat();

    // Inbound Hardware Control Ingestion (Thread-Safe)
    [[nodiscard]] bool HasHardwareControls() const noexcept { return m_hasInboundControls.load(std::memory_order_acquire); }
    [[nodiscard]] InboundFlightControlsPayload GetLatestFlightControls();
    [[nodiscard]] InboundSwitchesPayload GetLatestSwitches();
    [[nodiscard]] InboundNavControlsPayload ConsumeNavControls();

    /// @brief Runs a single synchronous iteration of reading/decoding and sending.
    /// Useful for deterministic testing or single-threaded operation.
    void ProcessSynchronous();

private:
    void WorkerLoop();
    void DispatchPacket(const DecodedPacket& packet);
    bool QueueOutboundPacket(SerialMessageType type, const void* payload, u8 length);

    SerialConfig m_config{};
    SerialPort m_port;
    SerialPacketDecoder m_decoder;

    std::atomic<bool> m_running{ false };
    std::thread m_workerThread;

    std::mutex m_txMutex;
    std::vector<u8> m_txPendingBytes;

    std::mutex m_rxMutex;
    std::atomic<bool> m_hasInboundControls{ false };
    InboundFlightControlsPayload m_latestControls{};
    InboundSwitchesPayload m_latestSwitches{};
    InboundNavControlsPayload m_accumulatedNav{};

    SerialStats m_stats{};
};

} // namespace LokYS::Serial
