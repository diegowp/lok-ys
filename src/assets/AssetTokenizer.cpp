#include "LokYS/assets/AssetTokenizer.hpp"
#include <cctype>
#include <charconv>
#include <algorithm>
#include <cmath>

namespace LokYS::Assets {

namespace {

constexpr double PI = 3.14159265358979323846;

// Trim whitespace from string_view
inline std::string_view Trim(std::string_view str) {
    while (!str.empty() && std::isspace(static_cast<unsigned char>(str.front()))) {
        str.remove_prefix(1);
    }
    while (!str.empty() && std::isspace(static_cast<unsigned char>(str.back()))) {
        str.remove_suffix(1);
    }
    return str;
}

} // namespace

std::string_view AssetTokenizer::StripComments(std::string_view line) noexcept {
    // Strip comments starting with '#'
    size_t hashPos = line.find('#');
    if (hashPos != std::string_view::npos) {
        line = line.substr(0, hashPos);
    }

    line = Trim(line);

    // If line begins with 'REM' (case-insensitive remark), treat as comment
    if (StartsWithIgnoreCase(line, "REM")) {
        // Ensure either whole line is "REM" or followed by space/tab
        if (line.size() == 3 || std::isspace(static_cast<unsigned char>(line[3]))) {
            return {};
        }
    }

    return line;
}

std::vector<std::string> AssetTokenizer::SplitTokens(std::string_view line) {
    std::vector<std::string> tokens;
    line = StripComments(line);

    size_t i = 0;
    while (i < line.size()) {
        // Skip whitespace
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) {
            ++i;
        }
        if (i >= line.size()) break;

        if (line[i] == '"') {
            // Quoted string
            ++i;
            size_t start = i;
            while (i < line.size() && line[i] != '"') {
                ++i;
            }
            tokens.emplace_back(line.substr(start, i - start));
            if (i < line.size() && line[i] == '"') {
                ++i;
            }
        } else {
            // Regular token
            size_t start = i;
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) {
                ++i;
            }
            tokens.emplace_back(line.substr(start, i - start));
        }
    }

    return tokens;
}

double AssetTokenizer::ParseValueWithUnit(std::string_view token, double defaultValue) noexcept {
    token = Trim(token);
    if (token.empty()) {
        return defaultValue;
    }

    // Split numeric prefix and unit suffix
    size_t splitIdx = 0;
    bool hasDigits = false;

    // Check optional leading +/-
    if (splitIdx < token.size() && (token[splitIdx] == '+' || token[splitIdx] == '-')) {
        splitIdx++;
    }

    while (splitIdx < token.size()) {
        char c = token[splitIdx];
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            hasDigits = true;
            splitIdx++;
        } else if ((c == 'e' || c == 'E') && hasDigits && splitIdx + 1 < token.size()) {
            char nextC = token[splitIdx + 1];
            if (std::isdigit(static_cast<unsigned char>(nextC)) || nextC == '+' || nextC == '-') {
                splitIdx += 2;
                while (splitIdx < token.size() && std::isdigit(static_cast<unsigned char>(token[splitIdx]))) {
                    splitIdx++;
                }
            } else {
                break;
            }
        } else {
            break;
        }
    }

    if (!hasDigits) {
        if (EqualsIgnoreCase(token, "TRUE")) return 1.0;
        if (EqualsIgnoreCase(token, "FALSE")) return 0.0;
        return defaultValue;
    }

    std::string_view numPart = token.substr(0, splitIdx);
    std::string_view unitPart = Trim(token.substr(splitIdx));

    double value = 0.0;
    // Fast double parsing via std::from_chars or std::strtod fallback
    std::string numStr(numPart);
    char* endPtr = nullptr;
    value = std::strtod(numStr.c_str(), &endPtr);
    if (endPtr == numStr.c_str()) {
        return defaultValue;
    }

    if (unitPart.empty()) {
        return value;
    }

    // Unit conversion table
    if (EqualsIgnoreCase(unitPart, "HP") || EqualsIgnoreCase(unitPart, "PS")) {
        return value * 745.699872; // Watts
    }
    if (EqualsIgnoreCase(unitPart, "KW")) {
        return value * 1000.0; // Watts
    }
    if (EqualsIgnoreCase(unitPart, "W")) {
        return value; // Watts
    }
    if (EqualsIgnoreCase(unitPart, "KG") || EqualsIgnoreCase(unitPart, "KGS")) {
        return value; // kg
    }
    if (EqualsIgnoreCase(unitPart, "T") || EqualsIgnoreCase(unitPart, "TON") || EqualsIgnoreCase(unitPart, "TONS")) {
        return value * 1000.0; // kg
    }
    if (EqualsIgnoreCase(unitPart, "LBS") || EqualsIgnoreCase(unitPart, "LB")) {
        return value * 0.45359237; // kg
    }
    if (EqualsIgnoreCase(unitPart, "M") || EqualsIgnoreCase(unitPart, "METRE") || EqualsIgnoreCase(unitPart, "METER")) {
        return value; // meters
    }
    if (EqualsIgnoreCase(unitPart, "M^2") || EqualsIgnoreCase(unitPart, "M2")) {
        return value; // square meters
    }
    if (EqualsIgnoreCase(unitPart, "FT") || EqualsIgnoreCase(unitPart, "FEET")) {
        return value * 0.3048; // meters
    }
    if (EqualsIgnoreCase(unitPart, "IN") || EqualsIgnoreCase(unitPart, "INCH") || EqualsIgnoreCase(unitPart, "INCHES")) {
        return value * 0.0254; // meters
    }
    if (EqualsIgnoreCase(unitPart, "KM/H") || EqualsIgnoreCase(unitPart, "KMH") || EqualsIgnoreCase(unitPart, "KPH")) {
        return value * (1000.0 / 3600.0); // m/s
    }
    if (EqualsIgnoreCase(unitPart, "KT") || EqualsIgnoreCase(unitPart, "KTS") || EqualsIgnoreCase(unitPart, "KNOT") || EqualsIgnoreCase(unitPart, "KNOTS")) {
        return value * 0.514444444444444; // m/s
    }
    if (EqualsIgnoreCase(unitPart, "MACH")) {
        return value * 340.29; // m/s
    }
    if (EqualsIgnoreCase(unitPart, "DEG") || EqualsIgnoreCase(unitPart, "DEGS") || EqualsIgnoreCase(unitPart, "DEGREE") || EqualsIgnoreCase(unitPart, "DEGREES")) {
        return value * (PI / 180.0); // radians
    }
    if (EqualsIgnoreCase(unitPart, "RAD") || EqualsIgnoreCase(unitPart, "RADS") || EqualsIgnoreCase(unitPart, "RADIAN") || EqualsIgnoreCase(unitPart, "RADIANS")) {
        return value; // radians
    }
    if (EqualsIgnoreCase(unitPart, "%")) {
        return value * 0.01; // dimensionless fraction
    }

    return value;
}

bool AssetTokenizer::ParseBool(std::string_view token, bool defaultValue) noexcept {
    token = Trim(token);
    if (EqualsIgnoreCase(token, "TRUE") || EqualsIgnoreCase(token, "YES") || token == "1") {
        return true;
    }
    if (EqualsIgnoreCase(token, "FALSE") || EqualsIgnoreCase(token, "NO") || token == "0") {
        return false;
    }
    return defaultValue;
}

Math::Vec3d AssetTokenizer::ParseVec3(const std::vector<std::string>& tokens, size_t startIndex) noexcept {
    if (startIndex + 2 >= tokens.size()) {
        return Math::Vec3d(0.0, 0.0, 0.0);
    }
    double x = ParseValueWithUnit(tokens[startIndex], 0.0);
    double y = ParseValueWithUnit(tokens[startIndex + 1], 0.0);
    double z = ParseValueWithUnit(tokens[startIndex + 2], 0.0);
    return Math::Vec3d(x, y, z);
}

bool AssetTokenizer::EqualsIgnoreCase(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

bool AssetTokenizer::StartsWithIgnoreCase(std::string_view str, std::string_view prefix) noexcept {
    if (str.size() < prefix.size()) return false;
    return EqualsIgnoreCase(str.substr(0, prefix.size()), prefix);
}

} // namespace LokYS::Assets
