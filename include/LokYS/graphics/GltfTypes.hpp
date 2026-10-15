#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/math/Quaternion.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include "LokYS/graphics/Mesh.hpp"
#include <string>
#include <vector>
#include <memory>

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

enum class GltfComponentType : u32 {
    Byte = 5120,
    UnsignedByte = 5121,
    Short = 5122,
    UnsignedShort = 5123,
    UnsignedInt = 5125,
    Float = 5126
};

enum class GltfAccessorType {
    Scalar,
    Vec2,
    Vec3,
    Vec4,
    Mat2,
    Mat3,
    Mat4,
    Unknown
};

enum class ArticulationType {
    None,
    Fuselage,
    Wings,
    Elevator,
    AileronLeft,
    AileronRight,
    Rudder,
    FlapLeft,
    FlapRight,
    Propeller,
    LandingGear,
    Cockpit
};

struct GltfMaterial {
    std::string name;
    Math::Vec4f baseColorFactor{ 1.0f, 1.0f, 1.0f, 1.0f };
    f32 metallicFactor{ 0.0f };
    f32 roughnessFactor{ 0.5f };
    bool doubleSided{ false };
};

struct GltfPrimitive {
    std::vector<Vertex3D> vertices;
    std::vector<u32> indices;
    i32 materialIndex{ -1 };

    // Direct3D 12 GPU Buffers
    Microsoft::WRL::ComPtr<ID3D12Resource> vertexBuffer;
    Microsoft::WRL::ComPtr<ID3D12Resource> indexBuffer;
    D3D12_VERTEX_BUFFER_VIEW vbView{};
    D3D12_INDEX_BUFFER_VIEW ibView{};
    u32 vertexCount{ 0 };
    u32 indexCount{ 0 };

    [[nodiscard]] bool CreateGPUResources(ID3D12Device* device);
    void Draw(ID3D12GraphicsCommandList* cmdList) const noexcept;
};

struct GltfMesh {
    std::string name;
    std::vector<GltfPrimitive> primitives;
};

struct ModelBounds {
    Math::Vec3f minBound{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f maxBound{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f center{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f dimensions{ 0.0f, 0.0f, 0.0f };
};

struct GltfNode {
    std::string name;
    i32 meshIndex{ -1 };
    std::vector<u32> children;
    i32 parentIndex{ -1 };

    // Rest transformation
    Math::Vec3f restTranslation{ 0.0f, 0.0f, 0.0f };
    Math::Quat4f restRotation{ Math::Quat4f::Identity() };
    Math::Vec3f restScale{ 1.0f, 1.0f, 1.0f };

    // Active transformation
    Math::Vec3f translation{ 0.0f, 0.0f, 0.0f };
    Math::Quat4f rotation{ Math::Quat4f::Identity() };
    Math::Vec3f scale{ 1.0f, 1.0f, 1.0f };

    // Matrix transformation (glTF node "matrix" property)
    bool hasMatrix{ false };
    Math::Mat4f matrix{ Math::Mat4f::Identity() };

    // Articulation parameters
    ArticulationType articulationType{ ArticulationType::None };
    Math::Vec3f pivotPoint{ 0.0f, 0.0f, 0.0f };
    Math::Vec3f hingeAxis{ 1.0f, 0.0f, 0.0f }; // Unit rotation axis
    f32 currentDeflectionRad{ 0.0f };

    // Computed world matrix (Forward Kinematics)
    Math::Mat4f localTransform{ Math::Mat4f::Identity() };
    Math::Mat4f worldTransform{ Math::Mat4f::Identity() };

    void ComputeLocalTransform() noexcept {
        if (hasMatrix) {
            if (currentDeflectionRad != 0.0f) {
                Math::Quat4f defRot = Math::Quat4f::FromAxisAngle(hingeAxis, currentDeflectionRad);
                Math::Mat4f tPivot = Math::Mat4f::Translation(pivotPoint);
                Math::Mat4f tNegPivot = Math::Mat4f::Translation(-pivotPoint);
                Math::Mat4f rMat = Math::Mat4f::FromQuaternion(defRot);
                localTransform = tNegPivot * rMat * tPivot * matrix;
            } else {
                localTransform = matrix;
            }
            return;
        }

        // M_local = T(translation) * R(rotation) * S(scale)
        // If pivot is set and deflection != 0:
        if (currentDeflectionRad != 0.0f) {
            Math::Quat4f defRot = Math::Quat4f::FromAxisAngle(hingeAxis, currentDeflectionRad);
            Math::Quat4f combinedRot = (restRotation * defRot).Normalized();

            Math::Mat4f tPivot = Math::Mat4f::Translation(pivotPoint);
            Math::Mat4f tNegPivot = Math::Mat4f::Translation(-pivotPoint);
            Math::Mat4f rMat = Math::Mat4f::FromQuaternion(combinedRot);
            Math::Mat4f tMat = Math::Mat4f::Translation(restTranslation);
            Math::Mat4f sMat = Math::Mat4f::Scale(restScale);

            localTransform = sMat * tNegPivot * rMat * tPivot * tMat;
        } else {
            Math::Mat4f tMat = Math::Mat4f::Translation(translation);
            Math::Mat4f rMat = Math::Mat4f::FromQuaternion(rotation);
            Math::Mat4f sMat = Math::Mat4f::Scale(scale);
            localTransform = sMat * rMat * tMat;
        }
    }
};

struct GltfBufferView {
    u32 bufferIndex{ 0 };
    size_t byteOffset{ 0 };
    size_t byteLength{ 0 };
    size_t byteStride{ 0 };
    u32 target{ 0 };
};

struct GltfAccessor {
    i32 bufferViewIndex{ -1 };
    size_t byteOffset{ 0 };
    GltfComponentType componentType{ GltfComponentType::Float };
    size_t count{ 0 };
    GltfAccessorType type{ GltfAccessorType::Scalar };
    std::vector<f32> minValues;
    std::vector<f32> maxValues;
};

} // namespace LokYS::Graphics
