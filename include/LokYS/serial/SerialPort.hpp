#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/serial/SerialTypes.hpp"
#include <string>
#include <vector>
#include <mutex>

namespace LokYS::Serial {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /// @brief Opens a native Win32 COM port (e.g., L"COM3", L"\\\\.\\COM10") or mock virtual port.
    bool Open(const std::wstring& portName, u32 baudRate = 115200);

    /// @brief Closes the active port and releases OS handles.
    void Close();

    [[nodiscard]] bool IsOpen() const noexcept;
    [[nodiscard]] const std::wstring& GetPortName() const noexcept { return m_portName; }
    [[nodiscard]] u32 GetBaudRate() const noexcept { return m_baudRate; }

    /// @brief Writes raw bytes out to the serial port.
    /// @return Number of bytes successfully transmitted.
    size_t Write(const u8* data, size_t length);

    /// @brief Reads available incoming bytes into buffer. Non-blocking.
    /// @return Number of bytes actually read.
    size_t Read(u8* buffer, size_t maxBytes);

    /// @brief Clears hardware transmit/receive buffers.
    void Purge();

    // =========================================================================
    // Mock / Loopback Simulation (for automated tests and offline CI)
    // =========================================================================
    void SetMockLoopback(bool enable) noexcept;
    [[nodiscard]] bool IsMockLoopback() const noexcept { return m_mockLoopback; }

    /// @brief Injects raw bytes into the simulated receive buffer.
    void InjectMockRxData(const u8* data, size_t length);

    /// @brief Fetches raw bytes sent to the simulated transmit buffer.
    std::vector<u8> FetchMockTxData();

private:
    std::wstring m_portName;
    u32 m_baudRate{ 115200 };
    void* m_handle{ nullptr }; // Win32 HANDLE

    void* m_readEvent{ nullptr };
    void* m_writeEvent{ nullptr };

    bool m_mockLoopback{ false };
    std::mutex m_mockMutex;
    std::vector<u8> m_mockRxBuffer;
    std::vector<u8> m_mockTxBuffer;
};

} // namespace LokYS::Serial
