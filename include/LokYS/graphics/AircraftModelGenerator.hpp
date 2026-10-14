#pragma once

#include "LokYS/core/Types.hpp"
#include <vector>
#include <string>

namespace LokYS::Graphics {

class AircraftModelGenerator {
public:
    // Generate binary glTF 2.0 (.glb) for Cessna 172 Skyhawk with articulated control surfaces
    [[nodiscard]] static std::vector<u8> GenerateCessna172Glb();

    // Generate and save to disk
    [[nodiscard]] static bool SaveCessna172Glb(const std::wstring& outputPath);
    [[nodiscard]] static bool SaveCessna172Glb(const std::string& outputPath);
};

} // namespace LokYS::Graphics
