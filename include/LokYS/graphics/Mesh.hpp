#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include <vector>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d12.h>
#include <wrl/client.h>

namespace LokYS::Graphics {

struct Vertex3D {
    Math::Vec3f position{ 0.0f, 0.0f, 0.0f }; // (x, y, z) in graphics coordinates
    Math::Vec3f normal{ 0.0f, 1.0f, 0.0f };   // surface normal for shading
    Math::Vec4f color{ 1.0f, 1.0f, 1.0f, 1.0f }; // RGBA tint
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh() = default;

    [[nodiscard]] bool Create(
        ID3D12Device* device,
        const std::vector<Vertex3D>& vertices,
        const std::vector<u32>& indices);

    void Destroy() noexcept {
        m_vertexBuffer.Reset();
        m_indexBuffer.Reset();
        m_vbView = {};
        m_ibView = {};
        m_vertexCount = 0;
        m_indexCount = 0;
    }

    void Draw(ID3D12GraphicsCommandList* cmdList) const noexcept;

    [[nodiscard]] u32 GetIndexCount() const noexcept { return m_indexCount; }
    [[nodiscard]] u32 GetVertexCount() const noexcept { return m_vertexCount; }

    // Procedural geometry factory methods
    [[nodiscard]] static Mesh CreateRunway(ID3D12Device* device);
    [[nodiscard]] static Mesh CreateTerrain(ID3D12Device* device);
    [[nodiscard]] static Mesh CreateCessna172(ID3D12Device* device);

private:
    Microsoft::WRL::ComPtr<ID3D12Resource> m_vertexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_indexBuffer;

    D3D12_VERTEX_BUFFER_VIEW m_vbView{};
    D3D12_INDEX_BUFFER_VIEW m_ibView{};

    u32 m_vertexCount{ 0 };
    u32 m_indexCount{ 0 };
};

} // namespace LokYS::Graphics
