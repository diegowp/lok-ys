#include "LokYS/graphics/GltfModel.hpp"
#include <iostream>

int main() {
    LokYS::Graphics::GltfModel model;
    std::string err;
    bool ok = model.LoadFromGlbFile("assets/models/c-17a_globemaster_iii.glb", &err);
    std::cout << "Load C-17 result: " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    if (!ok) {
        std::cout << "Error: " << err << std::endl;
        return 1;
    }
    std::cout << "Nodes: " << model.GetNodes().size() << std::endl;
    std::cout << "Meshes: " << model.GetMeshes().size() << std::endl;
    std::cout << "Total Vertices: " << model.GetTotalVertexCount() << std::endl;
    std::cout << "Total Triangles: " << model.GetTotalTriangleCount() << std::endl;
    
    model.AutoNormalizeToSpan(51.75f); // Real C-17 wingspan is 51.75m
    std::cout << "Bounds Min: (" << model.GetBounds().minBound.x << ", " << model.GetBounds().minBound.y << ", " << model.GetBounds().minBound.z << ")" << std::endl;
    std::cout << "Bounds Max: (" << model.GetBounds().maxBound.x << ", " << model.GetBounds().maxBound.y << ", " << model.GetBounds().maxBound.z << ")" << std::endl;
    std::cout << "Dimensions: (" << model.GetDimensions().x << ", " << model.GetDimensions().y << ", " << model.GetDimensions().z << ")" << std::endl;
    std::cout << "Effective Dim: (" << model.GetEffectiveDimensions().x << ", " << model.GetEffectiveDimensions().y << ", " << model.GetEffectiveDimensions().z << ")" << std::endl;
    return 0;
}
