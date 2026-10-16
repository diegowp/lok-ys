#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>

namespace LokYS::Assets {

struct DnmNode {
    std::string name;
    int animationClass = 0; // CLA (0=Fuselage, 5=Flap, 6=Elevator, 7=Aileron, 8=Rudder, 24=Propeller)
    Math::Vec3f position{ 0.0f, 0.0f, 0.0f };     // POS translation
    Math::Vec3f pivotCenter{ 0.0f, 0.0f, 0.0f };  // CNT rotation center
    Math::Vec3f pivotAxis{ 1.0f, 0.0f, 0.0f };    // PAX rotation axis
    std::vector<std::string> childrenNames;
    std::vector<Graphics::Vertex3D> vertices;
    std::vector<u32> indices;

    [[nodiscard]] bool HasGeometry() const noexcept {
        return !vertices.empty() && !indices.empty();
    }
};

struct DnmModel {
    std::string modelName;
    std::vector<DnmNode> nodes;
    Math::Vec3f aabbMin{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f aabbMax{ 0.0f, 0.0f, 0.0f };

    [[nodiscard]] const DnmNode* FindNode(std::string_view name) const noexcept;
    [[nodiscard]] size_t GetTotalVertices() const noexcept;
    [[nodiscard]] size_t GetTotalIndices() const noexcept;
};

class DnmParser {
public:
    // Parse a DNM dynamic articulated model file from text
    [[nodiscard]] static std::optional<DnmModel> ParseFromString(
        std::string_view text,
        std::string_view modelName = "UnnamedDNM");

    // Parse a DNM file from disk
    [[nodiscard]] static std::optional<DnmModel> ParseFromFile(const std::string& filePath);
};

} // namespace LokYS::Assets
