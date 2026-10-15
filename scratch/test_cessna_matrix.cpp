#include "LokYS/graphics/GltfModel.hpp"
#include <iostream>

int main() {
    LokYS::Graphics::GltfModel model;
    std::string err;
    bool ok = model.LoadFromGlbFile("assets/models/cessna172.glb", &err);
    std::cout << "Load Cessna with matrix: " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    if (!ok) return 1;
    
    std::cout << "Original Bounds:" << std::endl;
    std::cout << "  Min: (" << model.GetBounds().minBound.x << ", " << model.GetBounds().minBound.y << ", " << model.GetBounds().minBound.z << ")" << std::endl;
    std::cout << "  Max: (" << model.GetBounds().maxBound.x << ", " << model.GetBounds().maxBound.y << ", " << model.GetBounds().maxBound.z << ")" << std::endl;
    std::cout << "  Dim: (" << model.GetDimensions().x << ", " << model.GetDimensions().y << ", " << model.GetDimensions().z << ")" << std::endl;
    std::cout << "  Center: (" << model.GetCenter().x << ", " << model.GetCenter().y << ", " << model.GetCenter().z << ")" << std::endl;
    
    model.AutoNormalizeToSpan(11.0f);
    std::cout << "\nAfter AutoNormalizeToSpan(11.0):" << std::endl;
    std::cout << "  Effective Dim: (" << model.GetEffectiveDimensions().x << ", " << model.GetEffectiveDimensions().y << ", " << model.GetEffectiveDimensions().z << ")" << std::endl;
    std::cout << "  Scale in matrix: " << model.GetNormalizationTransform().m[0][0] << std::endl;
    std::cout << "  Trans in matrix: (" << model.GetNormalizationTransform().m[3][0] << ", " << model.GetNormalizationTransform().m[3][1] << ", " << model.GetNormalizationTransform().m[3][2] << ")" << std::endl;
    return 0;
}
