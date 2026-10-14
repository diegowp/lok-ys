#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "LokYS/serial/SerialPort.hpp"
#include <windows.h>
#include <algorithm>
#include <cstring>

namespace LokYS::Serial {

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
    Close();
}

bool SerialPort::Open(const std::wstring& portName, u32 baudRate) {
    Close();
    m_portName = portName;
    m_baudRate = baudRate;

    // Check if mock/loopback mode requested or port name indicates virtual device
    if (m_mockLoopback || portName == L"MOCK" || portName == L"VIRTUAL" || portName == L"LOOPBACK") {
        m_mockLoopback = true;
        return true;
    }

    std::wstring devicePath = portName;
    if (devicePath.rfind(L"\\\\.\\", 0) != 0) {
        devicePath = L"\\\\.\\" + devicePath;
    }

    HANDLE h = CreateFileW(
        devicePath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // Exclusive access
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        nullptr);

    if (h == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 50;

    if (!SetCommTimeouts(h, &timeouts)) {
        CloseHandle(h);
        return false;
    }

    m_readEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    m_writeEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);

    if (m_readEvent == nullptr || m_writeEvent == nullptr) {
        if (m_readEvent) CloseHandle(static_cast<HANDLE>(m_readEvent));
        if (m_writeEvent) CloseHandle(static_cast<HANDLE>(m_writeEvent));
        m_readEvent = nullptr;
        m_writeEvent = nullptr;
        CloseHandle(h);
        return false;
    }

    m_handle = h;
    Purge();
    return true;
}

void SerialPort::Close() {
    if (m_handle != nullptr && m_handle != INVALID_HANDLE_VALUE) {
        CancelIo(static_cast<HANDLE>(m_handle));
        CloseHandle(static_cast<HANDLE>(m_handle));
        m_handle = nullptr;
    }

    if (m_readEvent != nullptr) {
        CloseHandle(static_cast<HANDLE>(m_readEvent));
        m_readEvent = nullptr;
    }

    if (m_writeEvent != nullptr) {
        CloseHandle(static_cast<HANDLE>(m_writeEvent));
        m_writeEvent = nullptr;
    }

    std::lock_guard<std::mutex> lock(m_mockMutex);
    m_mockRxBuffer.clear();
    m_mockTxBuffer.clear();
}

bool SerialPort::IsOpen() const noexcept {
    if (m_mockLoopback) {
        return true;
    }
    return (m_handle != nullptr && m_handle != INVALID_HANDLE_VALUE);
}

size_t SerialPort::Write(const u8* data, size_t length) {
    if (data == nullptr || length == 0) {
        return 0;
    }

    if (m_mockLoopback) {
        std::lock_guard<std::mutex> lock(m_mockMutex);
        m_mockTxBuffer.insert(m_mockTxBuffer.end(), data, data + length);
        // Loop back into receive buffer for bidirectional simulation
        m_mockRxBuffer.insert(m_mockRxBuffer.end(), data, data + length);
        return length;
    }

    if (!IsOpen()) {
        return 0;
    }

    OVERLAPPED ov{};
    ov.hEvent = static_cast<HANDLE>(m_writeEvent);
    ResetEvent(static_cast<HANDLE>(m_writeEvent));

    DWORD written = 0;
    if (!WriteFile(static_cast<HANDLE>(m_handle), data, static_cast<DWORD>(length), &written, &ov)) {
        if (GetLastError() == ERROR_IO_PENDING) {
            if (!GetOverlappedResult(static_cast<HANDLE>(m_handle), &ov, &written, TRUE)) {
                return 0;
            }
        } else {
            return 0;
        }
    }

    return static_cast<size_t>(written);
}

size_t SerialPort::Read(u8* buffer, size_t maxBytes) {
    if (buffer == nullptr || maxBytes == 0) {
        return 0;
    }

    if (m_mockLoopback) {
        std::lock_guard<std::mutex> lock(m_mockMutex);
        size_t bytesToRead = std::min(maxBytes, m_mockRxBuffer.size());
        if (bytesToRead > 0) {
            std::memcpy(buffer, m_mockRxBuffer.data(), bytesToRead);
            m_mockRxBuffer.erase(m_mockRxBuffer.begin(), m_mockRxBuffer.begin() + static_cast<std::ptrdiff_t>(bytesToRead));
        }
        return bytesToRead;
    }

    if (!IsOpen()) {
        return 0;
    }

    OVERLAPPED ov{};
    ov.hEvent = static_cast<HANDLE>(m_readEvent);
    ResetEvent(static_cast<HANDLE>(m_readEvent));

    DWORD bytesRead = 0;
    if (!ReadFile(static_cast<HANDLE>(m_handle), buffer, static_cast<DWORD>(maxBytes), &bytesRead, &ov)) {
        if (GetLastError() == ERROR_IO_PENDING) {
            if (WaitForSingleObject(static_cast<HANDLE>(m_readEvent), 2) == WAIT_OBJECT_0) {
                GetOverlappedResult(static_cast<HANDLE>(m_handle), &ov, &bytesRead, FALSE);
            } else {
                CancelIo(static_cast<HANDLE>(m_handle));
                bytesRead = 0;
            }
        } else {
            bytesRead = 0;
        }
    }

    return static_cast<size_t>(bytesRead);
}

void SerialPort::Purge() {
    if (m_mockLoopback) {
        std::lock_guard<std::mutex> lock(m_mockMutex);
        m_mockRxBuffer.clear();
        m_mockTxBuffer.clear();
        return;
    }

    if (IsOpen()) {
        PurgeComm(static_cast<HANDLE>(m_handle), PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
    }
}

void SerialPort::SetMockLoopback(bool enable) noexcept {
    m_mockLoopback = enable;
}

void SerialPort::InjectMockRxData(const u8* data, size_t length) {
    if (data != nullptr && length > 0) {
        std::lock_guard<std::mutex> lock(m_mockMutex);
        m_mockRxBuffer.insert(m_mockRxBuffer.end(), data, data + length);
    }
}

std::vector<u8> SerialPort::FetchMockTxData() {
    std::lock_guard<std::mutex> lock(m_mockMutex);
    std::vector<u8> result = std::move(m_mockTxBuffer);
    m_mockTxBuffer.clear();
    return result;
}

} // namespace LokYS::Serial
