#include "LokYS/assets/SpawnStpParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>

namespace LokYS::Assets {

const SpawnPoint* ScenerySpawnList::FindPoint(std::string_view name) const noexcept {
    for (const auto& pt : points) {
        if (AssetTokenizer::EqualsIgnoreCase(pt.name, name)) {
            return &pt;
        }
    }
    return nullptr;
}

std::vector<SpawnPoint> ScenerySpawnList::GetRunwaySpawns() const {
    std::vector<SpawnPoint> runwaySpawns;
    for (const auto& pt : points) {
        if (!pt.isAirborne) {
            runwaySpawns.push_back(pt);
        }
    }
    return runwaySpawns;
}

std::vector<SpawnPoint> ScenerySpawnList::GetAirSpawns() const {
    std::vector<SpawnPoint> airSpawns;
    for (const auto& pt : points) {
        if (pt.isAirborne) {
            airSpawns.push_back(pt);
        }
    }
    return airSpawns;
}

std::optional<ScenerySpawnList> SpawnStpParser::ParseFromString(std::string_view text) {
    ScenerySpawnList list;
    SpawnPoint currentPt;
    bool inPoint = false;

    size_t lineStart = 0;
    while (lineStart < text.size()) {
        size_t lineEnd = text.find('\n', lineStart);
        if (lineEnd == std::string_view::npos) lineEnd = text.size();

        std::string_view rawLine = text.substr(lineStart, lineEnd - lineStart);
        if (!rawLine.empty() && rawLine.back() == '\r') rawLine.remove_suffix(1);
        lineStart = lineEnd + 1;

        std::string_view cleanLine = AssetTokenizer::StripComments(rawLine);
        if (cleanLine.empty()) continue;

        auto tokens = AssetTokenizer::SplitTokens(cleanLine);
        if (tokens.empty()) continue;

        const auto& tag = tokens[0];

        if (tag == "N" && tokens.size() >= 2) {
            // Commit previous point if exists
            if (inPoint && !currentPt.name.empty()) {
                list.points.push_back(currentPt);
            }
            inPoint = true;
            currentPt = SpawnPoint{};
            currentPt.name = tokens[1];
        }
        else if (tag == "C" && tokens.size() >= 2) {
            const auto& cmd = tokens[1];

            if (AssetTokenizer::EqualsIgnoreCase(cmd, "POSITION") && tokens.size() >= 5) {
                f64 xYs = AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0);
                f64 yYs = AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0);
                f64 zYs = AssetTokenizer::ParseValueWithUnit(tokens[4], 0.0);

                // YSFlight -> NED: X_NED = +Z_YS, Y_NED = +X_YS, Z_NED = -Y_YS
                currentPt.positionNed = Math::Vec3d(zYs, xYs, -yYs);
                currentPt.altitudeMslM = yYs;
                if (currentPt.altitudeMslM > 50.0) {
                    currentPt.isAirborne = true;
                }
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "ATTITUDE") && tokens.size() >= 5) {
                // In YS: heading pitch roll in degrees
                f64 hRad = AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0);
                f64 pRad = AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0);
                f64 rRad = AssetTokenizer::ParseValueWithUnit(tokens[4], 0.0);

                constexpr f64 RadToDeg = 180.0 / 3.14159265358979323846;
                currentPt.headingDeg = hRad * RadToDeg;
                currentPt.pitchDeg = pRad * RadToDeg;
                currentPt.rollDeg = rRad * RadToDeg;
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "INITSPED") && tokens.size() >= 3) {
                currentPt.initialSpeedMps = AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0);
                if (currentPt.initialSpeedMps > 30.0) {
                    currentPt.isAirborne = true;
                }
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "CTLTHROT") && tokens.size() >= 3) {
                currentPt.initialThrottle = std::atof(tokens[2].c_str());
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "CTLLDGEA") && tokens.size() >= 3) {
                currentPt.gearDown = AssetTokenizer::ParseBool(tokens[2], true);
                if (!currentPt.gearDown) {
                    currentPt.isAirborne = true;
                }
            }
        }
    }

    if (inPoint && !currentPt.name.empty()) {
        list.points.push_back(currentPt);
    }

    if (list.points.empty()) {
        return std::nullopt;
    }

    return list;
}

std::optional<ScenerySpawnList> SpawnStpParser::ParseFromFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();

    return ParseFromString(text);
}

} // namespace LokYS::Assets
