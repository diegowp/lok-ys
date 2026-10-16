#include "LokYS/assets/SceneryFldParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace LokYS::Assets {

f32 TerrMeshBlock::SampleElevation(f64 xNed, f64 yNed) const noexcept {
    if (elevations.empty() || cols == 0 || rows == 0) {
        return 0.0f;
    }

    f64 localX = xNed - originNed.x;
    f64 localY = yNed - originNed.y;

    if (localX < 0.0 || localY < 0.0) return 0.0f;

    f64 totalWidth = static_cast<f64>(cols) * stepX;
    f64 totalHeight = static_cast<f64>(rows) * stepZ;

    if (localX > totalWidth || localY > totalHeight) return 0.0f;

    f64 fx = localX / stepX;
    f64 fy = localY / stepZ;

    u32 ix = std::min(static_cast<u32>(fx), cols - 1);
    u32 iy = std::min(static_cast<u32>(fy), rows - 1);

    size_t idx = iy * (cols + 1) + ix;
    if (idx < elevations.size()) {
        return elevations[idx];
    }
    return 0.0f;
}

f32 SceneryField::SampleElevationAt(f64 xNed, f64 yNed) const noexcept {
    f32 maxElev = 0.0f;
    for (const auto& block : terrainBlocks) {
        f32 h = block.SampleElevation(xNed, yNed);
        if (h > maxElev) {
            maxElev = h;
        }
    }
    return maxElev;
}

Scenery::AirportConfig SceneryField::ToAirportConfig(
    std::string_view icao,
    std::string_view airportName) const
{
    Scenery::AirportConfig apt;
    apt.icao = std::string(icao);
    apt.name = std::string(airportName);

    if (!runways.empty()) {
        apt.locationNed = runways[0].centerNed;
        apt.elevationM = runways[0].elevationM;
        apt.runways = runways;
    }

    return apt;
}

std::optional<SceneryField> SceneryFldParser::ParseFromString(
    std::string_view text,
    std::string_view fieldName)
{
    SceneryField field;
    field.name = std::string(fieldName);

    // Dictionary of raw TerrMesh templates from PCK blocks
    std::unordered_map<std::string, TerrMeshBlock> meshTemplates;

    // First pass: extract PCK *.ter blocks
    size_t pos = 0;
    while (pos < text.size()) {
        size_t pckPos = text.find("PCK", pos);
        if (pckPos == std::string_view::npos) break;

        size_t lineEnd = text.find('\n', pckPos);
        if (lineEnd == std::string_view::npos) lineEnd = text.size();

        std::string_view pckLine = text.substr(pckPos, lineEnd - pckPos);
        auto tokens = AssetTokenizer::SplitTokens(pckLine);

        if (tokens.size() >= 2 && tokens[1].ends_with(".ter")) {
            std::string terName = tokens[1];

            // Next block starts after this line
            size_t blockStart = lineEnd + 1;
            size_t nextEnd = text.find("\nEND", blockStart);
            if (nextEnd == std::string_view::npos) nextEnd = text.find("\r\nEND", blockStart);
            if (nextEnd == std::string_view::npos) break;

            std::string_view blockText = text.substr(blockStart, nextEnd - blockStart);
            std::string blockStr(blockText);
            std::stringstream ss(blockStr);
            std::string line;

            TerrMeshBlock mesh;
            mesh.name = terName;

            while (std::getline(ss, line)) {
                auto bTokens = AssetTokenizer::SplitTokens(line);
                if (bTokens.empty()) continue;

                const auto& cmd = bTokens[0];
                if (AssetTokenizer::EqualsIgnoreCase(cmd, "NBL") && bTokens.size() >= 3) {
                    mesh.cols = static_cast<u32>(std::atoi(bTokens[1].c_str()));
                    mesh.rows = static_cast<u32>(std::atoi(bTokens[2].c_str()));
                }
                else if (AssetTokenizer::EqualsIgnoreCase(cmd, "TMS") && bTokens.size() >= 3) {
                    mesh.stepX = std::atof(bTokens[1].c_str());
                    mesh.stepZ = std::atof(bTokens[2].c_str());
                }
                else if (AssetTokenizer::EqualsIgnoreCase(cmd, "BLO") && bTokens.size() >= 2) {
                    float elev = static_cast<float>(std::atof(bTokens[1].c_str()));
                    mesh.elevations.push_back(elev);
                }
            }

            meshTemplates[terName] = mesh;
            pos = nextEnd + 4;
        } else {
            pos = lineEnd + 1;
        }
    }

    // Second pass: Parse entities (GND, SKY, TER, RGN, PC2, PLT)
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

        if (AssetTokenizer::EqualsIgnoreCase(tag, "GND") && tokens.size() >= 4) {
            field.gndColor.x = static_cast<float>(std::atof(tokens[1].c_str())) / 255.0f;
            field.gndColor.y = static_cast<float>(std::atof(tokens[2].c_str())) / 255.0f;
            field.gndColor.z = static_cast<float>(std::atof(tokens[3].c_str())) / 255.0f;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "SKY") && tokens.size() >= 4) {
            field.skyColor.x = static_cast<float>(std::atof(tokens[1].c_str())) / 255.0f;
            field.skyColor.y = static_cast<float>(std::atof(tokens[2].c_str())) / 255.0f;
            field.skyColor.z = static_cast<float>(std::atof(tokens[3].c_str())) / 255.0f;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TER")) {
            // Read until END
            std::string terFile;
            Math::Vec3d posYs{ 0.0, 0.0, 0.0 };

            while (lineStart < text.size()) {
                size_t subEnd = text.find('\n', lineStart);
                if (subEnd == std::string_view::npos) subEnd = text.size();
                std::string_view subLine = text.substr(lineStart, subEnd - lineStart);
                lineStart = subEnd + 1;

                auto sTokens = AssetTokenizer::SplitTokens(subLine);
                if (sTokens.empty()) continue;
                if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "END")) break;

                if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "FIL") && sTokens.size() >= 2) {
                    terFile = sTokens[1];
                }
                else if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "POS") && sTokens.size() >= 4) {
                    posYs.x = std::atof(sTokens[1].c_str());
                    posYs.y = std::atof(sTokens[2].c_str());
                    posYs.z = std::atof(sTokens[3].c_str());
                }
            }

            auto it = meshTemplates.find(terFile);
            if (it != meshTemplates.end()) {
                TerrMeshBlock instance = it->second;
                // YS -> NED: X_NED = +Z_YS, Y_NED = +X_YS, Z_NED = -Y_YS
                instance.originNed = Math::Vec3d(posYs.z, posYs.x, -posYs.y);
                field.terrainBlocks.push_back(std::move(instance));
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "RGN")) {
            // Region: can be Runway ("RW") or Tower ("TOWER")
            f64 x1 = 0.0, z1 = 0.0, x2 = 0.0, z2 = 0.0;
            Math::Vec3d posYs{ 0.0, 0.0, 0.0 };
            f64 headingDeg = 0.0;
            std::string rgnTag;

            while (lineStart < text.size()) {
                size_t subEnd = text.find('\n', lineStart);
                if (subEnd == std::string_view::npos) subEnd = text.size();
                std::string_view subLine = text.substr(lineStart, subEnd - lineStart);
                lineStart = subEnd + 1;

                auto sTokens = AssetTokenizer::SplitTokens(subLine);
                if (sTokens.empty()) continue;
                if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "END")) break;

                if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "ARE") && sTokens.size() >= 5) {
                    x1 = std::atof(sTokens[1].c_str());
                    z1 = std::atof(sTokens[2].c_str());
                    x2 = std::atof(sTokens[3].c_str());
                    z2 = std::atof(sTokens[4].c_str());
                }
                else if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "POS") && sTokens.size() >= 4) {
                    posYs.x = std::atof(sTokens[1].c_str());
                    posYs.y = std::atof(sTokens[2].c_str());
                    posYs.z = std::atof(sTokens[3].c_str());
                    if (sTokens.size() >= 5) {
                        headingDeg = std::atof(sTokens[4].c_str());
                    }
                }
                else if (AssetTokenizer::EqualsIgnoreCase(sTokens[0], "TAG") && sTokens.size() >= 2) {
                    rgnTag = sTokens[1];
                }
            }

            if (rgnTag == "RW") {
                Scenery::RunwayConfig rnw;
                f64 width = std::abs(x2 - x1);
                f64 length = std::abs(z2 - z1);
                if (width > length) std::swap(width, length);

                rnw.widthM = (width > 5.0) ? width : 45.0;
                rnw.lengthM = (length > 100.0) ? length : 2000.0;

                // Position in NED
                rnw.centerNed = Math::Vec3d(posYs.z, posYs.x, -posYs.y);
                rnw.elevationM = posYs.y;
                rnw.headingDeg = headingDeg;
                rnw.ident = "36/18";
                rnw.hasAls = true;
                rnw.hasPapi = true;
                rnw.hasIls = true;
                rnw.ilsFrequencyMhz = 109.50;

                field.runways.push_back(rnw);
            }
            else if (rgnTag == "TOWER") {
                PlacedSceneryObject obj;
                obj.tag = "TOWER";
                obj.modelFile = "ground/atsugi_tower.srf";
                obj.positionNed = Math::Vec3d(posYs.z, posYs.x, -posYs.y);
                obj.headingDeg = headingDeg;
                field.placedObjects.push_back(obj);
            }
        }
    }

    return field;
}

std::optional<SceneryField> SceneryFldParser::ParseFromFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();

    size_t lastSlash = filePath.find_last_of("/\\");
    std::string name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

    return ParseFromString(text, name);
}

} // namespace LokYS::Assets
