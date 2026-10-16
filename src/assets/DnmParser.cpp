#include "LokYS/assets/DnmParser.hpp"
#include "LokYS/assets/SrfParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>

namespace LokYS::Assets {

const DnmNode* DnmModel::FindNode(std::string_view name) const noexcept {
    for (const auto& node : nodes) {
        if (node.name == name) return &node;
    }
    return nullptr;
}

size_t DnmModel::GetTotalVertices() const noexcept {
    size_t count = 0;
    for (const auto& node : nodes) {
        count += node.vertices.size();
    }
    return count;
}

size_t DnmModel::GetTotalIndices() const noexcept {
    size_t count = 0;
    for (const auto& node : nodes) {
        count += node.indices.size();
    }
    return count;
}

std::optional<DnmModel> DnmParser::ParseFromString(std::string_view text, std::string_view modelName) {
    DnmModel model;
    model.modelName = std::string(modelName);

    // Dictionary of geometry submeshes parsed from PCK blocks
    std::unordered_map<std::string, SrfModel> submeshGeometries;

    // First pass: extract PCK blocks
    size_t pos = 0;
    while (pos < text.size()) {
        size_t pckPos = text.find("PCK", pos);
        if (pckPos == std::string_view::npos) {
            break;
        }

        // Find line end of PCK
        size_t lineEnd = text.find('\n', pckPos);
        if (lineEnd == std::string_view::npos) lineEnd = text.size();

        std::string_view pckLine = text.substr(pckPos, lineEnd - pckPos);
        auto tokens = AssetTokenizer::SplitTokens(pckLine);

        if (tokens.size() >= 2) {
            std::string partName = tokens[1];

            // Next block starts after this line
            size_t blockStart = lineEnd + 1;
            // The block ends at the next PCK, or at the first SRF tag, or EOF
            size_t nextPck = text.find("PCK", blockStart);
            size_t nextSrf = text.find("\nSRF ", blockStart);
            if (nextSrf == std::string_view::npos) {
                nextSrf = text.find("\r\nSRF ", blockStart);
            }

            size_t blockEnd = text.size();
            if (nextPck != std::string_view::npos) blockEnd = std::min(blockEnd, nextPck);
            if (nextSrf != std::string_view::npos) blockEnd = std::min(blockEnd, nextSrf);

            std::string_view surfText = text.substr(blockStart, blockEnd - blockStart);
            auto srf = SrfParser::ParseFromString(surfText, partName);
            if (srf && srf->IsValid()) {
                submeshGeometries[partName] = *srf;
            }

            pos = blockEnd;
        } else {
            pos = lineEnd + 1;
        }
    }

    // Second pass: Parse node hierarchy definitions (SRF ... END)
    std::unordered_map<std::string, DnmNode> parsedNodes;

    size_t srfPos = 0;
    while (srfPos < text.size()) {
        // Look for SRF "<name>" or SRF <name>
        size_t tagPos = text.find("SRF ", srfPos);
        if (tagPos == std::string_view::npos) {
            break;
        }

        // Ensure SRF is at the beginning of a line
        if (tagPos > 0 && text[tagPos - 1] != '\n' && text[tagPos - 1] != '\r') {
            srfPos = tagPos + 4;
            continue;
        }

        size_t endPos = text.find("END", tagPos);
        if (endPos == std::string_view::npos) {
            break;
        }

        // Find line end of END
        size_t endLine = text.find('\n', endPos);
        if (endLine == std::string_view::npos) endLine = text.size();

        std::string_view nodeBlock = text.substr(tagPos, endLine - tagPos);
        srfPos = endLine + 1;

        // Parse lines within the node block
        std::string nodeStr(nodeBlock);
        std::stringstream ss(nodeStr);
        std::string line;
        DnmNode node;

        while (std::getline(ss, line)) {
            auto tokens = AssetTokenizer::SplitTokens(line);
            if (tokens.empty()) continue;

            const auto& cmd = tokens[0];
            if (AssetTokenizer::EqualsIgnoreCase(cmd, "SRF") && tokens.size() >= 2) {
                node.name = tokens[1];
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "CLA") && tokens.size() >= 2) {
                node.animationClass = std::atoi(tokens[1].c_str());
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "CLD") && tokens.size() >= 2) {
                node.childrenNames.push_back(tokens[1]);
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "POS") && tokens.size() >= 4) {
                node.position.x = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 0.0));
                node.position.y = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0));
                node.position.z = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0));
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "CNT") && tokens.size() >= 4) {
                node.pivotCenter.x = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 0.0));
                node.pivotCenter.y = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0));
                node.pivotCenter.z = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0));
            }
            else if (AssetTokenizer::EqualsIgnoreCase(cmd, "PAX") && tokens.size() >= 4) {
                node.pivotAxis.x = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 0.0));
                node.pivotAxis.y = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[2], 0.0));
                node.pivotAxis.z = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[3], 0.0));
                if (node.pivotAxis.LengthSq() > 1e-6f) {
                    node.pivotAxis = node.pivotAxis.Normalized();
                }
            }
        }

        if (!node.name.empty()) {
            parsedNodes[node.name] = node;
        }
    }

    // Merge geometries into nodes
    Math::Vec3f bMin(1e9f, 1e9f, 1e9f);
    Math::Vec3f bMax(-1e9f, -1e9f, -1e9f);
    bool hasAnyVerts = false;

    // Add all nodes from hierarchy
    for (auto& [name, node] : parsedNodes) {
        auto it = submeshGeometries.find(name);
        if (it != submeshGeometries.end()) {
            node.vertices = std::move(it->second.vertices);
            node.indices = std::move(it->second.indices);
            submeshGeometries.erase(it);

            for (const auto& v : node.vertices) {
                bMin.x = std::min(bMin.x, v.position.x);
                bMin.y = std::min(bMin.y, v.position.y);
                bMin.z = std::min(bMin.z, v.position.z);
                bMax.x = std::max(bMax.x, v.position.x);
                bMax.y = std::max(bMax.y, v.position.y);
                bMax.z = std::max(bMax.z, v.position.z);
                hasAnyVerts = true;
            }
        }
        model.nodes.push_back(std::move(node));
    }

    // Add any remaining PCK geometries not explicitly declared in SRF hierarchy
    for (auto& [name, geom] : submeshGeometries) {
        DnmNode node;
        node.name = name;
        node.animationClass = 0; // static
        node.vertices = std::move(geom.vertices);
        node.indices = std::move(geom.indices);

        for (const auto& v : node.vertices) {
            bMin.x = std::min(bMin.x, v.position.x);
            bMin.y = std::min(bMin.y, v.position.y);
            bMin.z = std::min(bMin.z, v.position.z);
            bMax.x = std::max(bMax.x, v.position.x);
            bMax.y = std::max(bMax.y, v.position.y);
            bMax.z = std::max(bMax.z, v.position.z);
            hasAnyVerts = true;
        }
        model.nodes.push_back(std::move(node));
    }

    if (hasAnyVerts) {
        model.aabbMin = bMin;
        model.aabbMax = bMax;
    }

    if (model.nodes.empty()) {
        return std::nullopt;
    }

    return model;
}

std::optional<DnmModel> DnmParser::ParseFromFile(const std::string& filePath) {
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
