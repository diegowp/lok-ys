#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>

namespace LokYS::Assets {

struct SrfModel {
    std::string name;
    std::vector<Graphics::Vertex3D> vertices;
    std::vector<u32> indices;
    Math::Vec3f aabbMin{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f aabbMax{ 0.0f, 0.0f, 0.0f };
    u32 polygonCount = 0;

    [[nodiscard]] bool IsValid() const noexcept {
        return !vertices.empty() && !indices.empty();
    }
};

class SrfParser {
public:
    // Parse an SRF surface file from text
    [[nodiscard]] static std::optional<SrfModel> ParseFromString(
        std::string_view text,
        std::string_view modelName = "UnnamedSRF");

    // Parse an SRF surface file from disk
    [[nodiscard]] static std::optional<SrfModel> ParseFromFile(const std::string& filePath);

private:
    static Math::Vec3f ComputeNewellNormal(const std::vector<Math::Vec3f>& polyVerts);
};

} // namespace LokYS::Assets
