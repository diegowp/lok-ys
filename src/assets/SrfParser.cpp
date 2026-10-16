#include "LokYS/assets/SrfParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace LokYS::Assets {

Math::Vec3f SrfParser::ComputeNewellNormal(const std::vector<Math::Vec3f>& polyVerts) {
    if (polyVerts.size() < 3) {
        return Math::Vec3f(0.0f, 1.0f, 0.0f);
    }

    float nx = 0.0f, ny = 0.0f, nz = 0.0f;
    size_t n = polyVerts.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& cur = polyVerts[i];
        const auto& next = polyVerts[(i + 1) % n];
        nx += (cur.y - next.y) * (cur.z + next.z);
        ny += (cur.z - next.z) * (cur.x + next.x);
        nz += (cur.x - next.x) * (cur.y + next.y);
    }

    Math::Vec3f normal(nx, ny, nz);
    float lenSq = normal.LengthSq();
    if (lenSq > 1e-8f) {
        return normal.Normalized();
    }
    return Math::Vec3f(0.0f, 1.0f, 0.0f);
}

std::optional<SrfModel> SrfParser::ParseFromString(std::string_view text, std::string_view modelName) {
    SrfModel model;
    model.name = std::string(modelName);

    std::vector<Math::Vec3f> rawPositions;
    rawPositions.reserve(1024);

    struct TempFace {
        std::vector<u32> vertexIndices;
        Math::Vec3f normal{ 0.0f, 1.0f, 0.0f };
        bool hasNormal = false;
        Math::Vec4f color{ 1.0f, 1.0f, 1.0f, 1.0f };
    };

    std::vector<TempFace> faces;
    faces.reserve(512);

    TempFace currentFace;
    bool inFace = false;

    // Scan lines
    size_t lineStart = 0;
    while (lineStart < text.size()) {
        size_t lineEnd = text.find('\n', lineStart);
        if (lineEnd == std::string_view::npos) {
            lineEnd = text.size();
        }

        std::string_view rawLine = text.substr(lineStart, lineEnd - lineStart);
        if (!rawLine.empty() && rawLine.back() == '\r') {
            rawLine.remove_suffix(1);
        }
        lineStart = lineEnd + 1;

        std::string_view cleanLine = AssetTokenizer::StripComments(rawLine);
        if (cleanLine.empty()) {
            continue;
        }

        auto tokens = AssetTokenizer::SplitTokens(cleanLine);
        if (tokens.empty()) {
            continue;
        }

        const auto& cmd = tokens[0];

        if (AssetTokenizer::EqualsIgnoreCase(cmd, "SURF")) {
            // Header, continue
            continue;
        }

        if (AssetTokenizer::EqualsIgnoreCase(cmd, "V")) {
            if (inFace) {
                // Inside face: list of vertex indices: V idx0 idx1 idx2 ...
                for (size_t i = 1; i < tokens.size(); ++i) {
                    char* endP = nullptr;
                    long idx = std::strtol(tokens[i].c_str(), &endP, 10);
                    if (endP != tokens[i].c_str() && idx >= 0) {
                        currentFace.vertexIndices.push_back(static_cast<u32>(idx));
                    }
                }
            } else {
                // Outside face: vertex declaration: V x y z
                if (tokens.size() >= 4) {
                    float x = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 0.0));
                    float y = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0));
                    float z = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0));
                    rawPositions.emplace_back(x, y, z);
                }
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(cmd, "F")) {
            // Start of face
            inFace = true;
            currentFace = TempFace{};
        }
        else if (AssetTokenizer::EqualsIgnoreCase(cmd, "N")) {
            // Normal declaration inside face: N cx cy cz nx ny nz OR N nx ny nz
            if (inFace) {
                if (tokens.size() >= 7) {
                    float nx = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[4], 0.0));
                    float ny = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[5], 0.0));
                    float nz = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[6], 0.0));
                    Math::Vec3f norm(nx, ny, nz);
                    if (norm.LengthSq() > 1e-6f) {
                        currentFace.normal = norm.Normalized();
                        currentFace.hasNormal = true;
                    }
                } else if (tokens.size() >= 4) {
                    float nx = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 0.0));
                    float ny = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0));
                    float nz = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0));
                    Math::Vec3f norm(nx, ny, nz);
                    if (norm.LengthSq() > 1e-6f) {
                        currentFace.normal = norm.Normalized();
                        currentFace.hasNormal = true;
                    }
                }
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(cmd, "C")) {
            // Color inside face: C r g b [a]
            if (inFace && tokens.size() >= 4) {
                float r = static_cast<float>(std::atof(tokens[1].c_str())) / 255.0f;
                float g = static_cast<float>(std::atof(tokens[2].c_str())) / 255.0f;
                float b = static_cast<float>(std::atof(tokens[3].c_str())) / 255.0f;
                float a = (tokens.size() >= 5) ? (static_cast<float>(std::atof(tokens[4].c_str())) / 255.0f) : 1.0f;
                currentFace.color = Math::Vec4f(r, g, b, a);
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(cmd, "E")) {
            // End of face
            if (inFace && currentFace.vertexIndices.size() >= 3) {
                faces.push_back(currentFace);
            }
            inFace = false;
        }
    }

    if (rawPositions.empty() || faces.empty()) {
        return std::nullopt;
    }

    // Initialize AABB bounds
    Math::Vec3f bMin(1e9f, 1e9f, 1e9f);
    Math::Vec3f bMax(-1e9f, -1e9f, -1e9f);

    for (const auto& pos : rawPositions) {
        bMin.x = std::min(bMin.x, pos.x);
        bMin.y = std::min(bMin.y, pos.y);
        bMin.z = std::min(bMin.z, pos.z);
        bMax.x = std::max(bMax.x, pos.x);
        bMax.y = std::max(bMax.y, pos.y);
        bMax.z = std::max(bMax.z, pos.z);
    }
    model.aabbMin = bMin;
    model.aabbMax = bMax;

    // Triangulate faces and generate vertex/index buffers
    model.polygonCount = static_cast<u32>(faces.size());

    for (const auto& face : faces) {
        // Collect face 3D positions
        std::vector<Math::Vec3f> facePts;
        facePts.reserve(face.vertexIndices.size());
        for (u32 idx : face.vertexIndices) {
            if (idx < rawPositions.size()) {
                facePts.push_back(rawPositions[idx]);
            }
        }

        if (facePts.size() < 3) continue;

        // Compute normal if not specified
        Math::Vec3f faceNorm = face.hasNormal ? face.normal : ComputeNewellNormal(facePts);

        u32 baseIdx = static_cast<u32>(model.vertices.size());
        for (const auto& pt : facePts) {
            model.vertices.push_back({ pt, faceNorm, face.color });
        }

        // Triangulate convex fan: 0, i, i+1
        size_t numPts = facePts.size();
        for (size_t i = 1; i + 1 < numPts; ++i) {
            Math::Vec3f v0 = facePts[0];
            Math::Vec3f v1 = facePts[i];
            Math::Vec3f v2 = facePts[i + 1];

            // Verify winding order against normal
            Math::Vec3f cross = (v1 - v0).Cross(v2 - v0);
            if (cross.Dot(faceNorm) < 0.0f) {
                // Inverted winding, swap v1 and v2
                model.indices.push_back(baseIdx + 0);
                model.indices.push_back(static_cast<u32>(baseIdx + i + 1));
                model.indices.push_back(static_cast<u32>(baseIdx + i));
            } else {
                model.indices.push_back(baseIdx + 0);
                model.indices.push_back(static_cast<u32>(baseIdx + i));
                model.indices.push_back(static_cast<u32>(baseIdx + i + 1));
            }
        }
    }

    if (model.vertices.empty() || model.indices.empty()) {
        return std::nullopt;
    }

    return model;
}

std::optional<SrfModel> SrfParser::ParseFromFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();

    // Extract filename as model name
    size_t lastSlash = filePath.find_last_of("/\\");
    std::string name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

    return ParseFromString(text, name);
}

} // namespace LokYS::Assets
