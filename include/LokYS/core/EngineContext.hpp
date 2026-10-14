#pragma once

#include "LokYS/core/Types.hpp"
#include <string>

namespace LokYS {

struct SystemCapabilities {
    u32 cpuCores{0};
    u32 cpuLogicalProcessors{0};
    bool d3d12Supported{false};
    std::wstring adapterName{};
    u64 dedicatedVideoMemory{0};
};

class EngineContext {
public:
    EngineContext();
    ~EngineContext();

    EngineContext(const EngineContext&) = delete;
    EngineContext& operator=(const EngineContext&) = delete;

    [[nodiscard]] bool Initialize();
    void Shutdown();

    [[nodiscard]] const SystemCapabilities& GetCapabilities() const noexcept {
        return m_capabilities;
    }

private:
    bool ProbeSystemHardware();
    bool ProbeDirect3D12();

    SystemCapabilities m_capabilities{};
    bool m_initialized{false};
};

} // namespace LokYS
