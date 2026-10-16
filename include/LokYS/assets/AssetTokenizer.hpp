#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>

namespace LokYS::Assets {

class AssetTokenizer {
public:
    // Strip comments initiated by '#' or full-line comments starting with 'REM'
    [[nodiscard]] static std::string_view StripComments(std::string_view line) noexcept;

    // Split a line into tokens, respecting quotes e.g. "Cessna 172R"
    [[nodiscard]] static std::vector<std::string> SplitTokens(std::string_view line);

    // Convert string with possible unit suffix into an SI double
    // Examples: "180HP" -> 134225.97, "630kg" -> 630.0, "15deg" -> 0.261799, "250km/h" -> 69.444
    [[nodiscard]] static double ParseValueWithUnit(std::string_view token, double defaultValue = 0.0) noexcept;

    // Convert string to boolean (TRUE/FALSE, 1/0, YES/NO)
    [[nodiscard]] static bool ParseBool(std::string_view token, bool defaultValue = false) noexcept;

    // Parse 3 coordinate components with units into a Vec3d
    [[nodiscard]] static Math::Vec3d ParseVec3(const std::vector<std::string>& tokens, size_t startIndex = 1) noexcept;

    // Case-insensitive string equality
    [[nodiscard]] static bool EqualsIgnoreCase(std::string_view a, std::string_view b) noexcept;

    // Case-insensitive prefix match
    [[nodiscard]] static bool StartsWithIgnoreCase(std::string_view str, std::string_view prefix) noexcept;
};

} // namespace LokYS::Assets
