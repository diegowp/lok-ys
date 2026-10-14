#pragma once

#include "LokYS/core/Types.hpp"
#include <vector>
#include <string>

namespace LokYS::Graphics {

class AircraftModelGenerator {
public:
    // Generate binary glTF 2.0 (.glb) for Cessna 172 Skyhawk with articulated control surfaces
    [[nodiscard]] static std::vector<u8> GenerateCessna172Glb();
    [[nodiscard]] static bool SaveCessna172Glb(const std::wstring& outputPath);
    [[nodiscard]] static bool SaveCessna172Glb(const std::string& outputPath);

    // Generate binary glTF 2.0 (.glb) for Beechcraft Baron 58 twin-engine
    [[nodiscard]] static std::vector<u8> GenerateBaron58Glb();
    [[nodiscard]] static bool SaveBaron58Glb(const std::wstring& outputPath);
    [[nodiscard]] static bool SaveBaron58Glb(const std::string& outputPath);

    // Generate binary glTF 2.0 (.glb) for Extra 330SC competition aerobatic monoplane
    [[nodiscard]] static std::vector<u8> GenerateExtra330Glb();
    [[nodiscard]] static bool SaveExtra330Glb(const std::wstring& outputPath);
    [[nodiscard]] static bool SaveExtra330Glb(const std::string& outputPath);
};

} // namespace LokYS::Graphics
