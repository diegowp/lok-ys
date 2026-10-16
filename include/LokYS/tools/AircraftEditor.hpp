#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/aircraft/AircraftTypes.hpp"
#include "LokYS/assets/AircraftDatParser.hpp"
#include "LokYS/assets/DnmParser.hpp"
#include "LokYS/assets/CockpitIstParser.hpp"
#include "LokYS/assets/AcftPackage.hpp"
#include "LokYS/vfs/ZipWriter.hpp"
#include "LokYS/math/Matrix4x4.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <optional>
#include <span>

namespace LokYS::Tools {

struct AeroPolarSample {
    f64 alphaDeg{ 0.0 };
    f64 cl{ 0.0 };
    f64 cd{ 0.0 };
    f64 cm{ 0.0 };
};

struct AircraftEditorSummary {
    std::string name;
    std::string icao;
    f64 emptyMassKg{ 0.0 };
    f64 maxThrustN{ 0.0 };
    f64 wingAreaM2{ 0.0 };
    f64 wingspanM{ 0.0 };
    f64 stallSpeedKts{ 0.0 };
    f64 cruiseSpeedKts{ 0.0 };
    size_t dnmNodeCount{ 0 };
    size_t totalVertices{ 0 };
    size_t totalIndices{ 0 };
    bool isLongitudinallyStable{ false };
};

class AircraftEditor {
public:
    AircraftEditor() = default;
    ~AircraftEditor() = default;

    // Load from individual legacy files
    [[nodiscard]] bool LoadFromFiles(
        const std::string& datPath,
        const std::string& dnmPath = "",
        const std::string& istPath = "");

    // Load from raw text buffers
    [[nodiscard]] bool LoadFromMemory(
        std::string_view datText,
        std::string_view dnmText = "",
        std::string_view istText = "",
        const std::string& fallbackName = "Custom Aircraft");

    // Load from packaged .acft.zip archive
    [[nodiscard]] bool LoadFromPackageBuffer(std::span<const uint8_t> zipBuffer);
    [[nodiscard]] bool LoadFromPackageFile(const std::string& zipFilePath);

    // Node Hierarchy Inspector
    [[nodiscard]] size_t GetNodeCount() const noexcept;
    [[nodiscard]] const Assets::DnmNode* GetNode(size_t index) const noexcept;
    [[nodiscard]] int FindNodeByName(std::string_view name) const noexcept;
    [[nodiscard]] std::string GenerateHierarchyTree() const;

    // Control Surface & Animation Debugger
    void SetAnimationChannel(int animClass, f32 stateNormalized) noexcept;
    [[nodiscard]] f32 GetAnimationChannel(int animClass) const noexcept;
    void ResetAllAnimations() noexcept;

    // Evaluate animated node transformation matrix (local and world-relative within aircraft)
    [[nodiscard]] Math::Mat4f ComputeNodeLocalTransform(size_t nodeIndex) const noexcept;
    [[nodiscard]] Math::Mat4f ComputeNodeWorldTransform(size_t nodeIndex) const noexcept;

    // Aerodynamics Tuner & Polar Calculations
    [[nodiscard]] std::vector<AeroPolarSample> ComputeAeroPolars(
        f64 minAlphaDeg = -20.0,
        f64 maxAlphaDeg = +25.0,
        f64 stepDeg = 1.0) const;

    [[nodiscard]] bool ValidateLongitudinalStability() const noexcept {
        return m_acftConfig.aero.Cma < 0.0;
    }

    [[nodiscard]] f64 GetStaticMargin() const noexcept {
        return -m_acftConfig.aero.Cma / m_acftConfig.aero.CLa;
    }

    // Tuning Mutators
    void SetEngineHorsepower(f64 hp) noexcept;
    void SetCruiseSpeedKnots(f64 knots) noexcept;
    void SetEmptyMassKg(f64 massKg) noexcept;
    void SetWingAreaM2(f64 areaM2) noexcept;
    void SetWingspanM(f64 spanM) noexcept;
    void SetStallAlphaDeg(f64 stallDeg) noexcept;
    void SetPitchStiffnessCma(f64 cma) noexcept;

    // Cockpit Instrumentation
    [[nodiscard]] const std::optional<Assets::CockpitPanelLayout>& GetCockpitPanel() const noexcept {
        return m_cockpitPanel;
    }
    void SetAirspeedLimitsKnots(f32 vfe, f32 vno, f32 vne) noexcept;

    // Summary Statistics
    [[nodiscard]] AircraftEditorSummary GetSummary() const noexcept;

    // Exporting
    [[nodiscard]] std::string GenerateDatText() const;
    [[nodiscard]] std::vector<uint8_t> ExportToAcftPackage() const;
    [[nodiscard]] bool ExportToAcftFile(const std::string& outZipPath) const;

    // Direct accessors
    [[nodiscard]] const Aircraft::AircraftConfig& GetConfig() const noexcept { return m_acftConfig; }
    [[nodiscard]] const Assets::DnmModel& GetVisualModel() const noexcept { return m_dnmModel; }
    [[nodiscard]] bool HasVisualModel() const noexcept { return !m_dnmModel.nodes.empty(); }

private:
    [[nodiscard]] int FindParentNodeIndex(size_t childIndex) const noexcept;

    Aircraft::AircraftConfig m_acftConfig;
    Assets::YsGearPosition m_gearPositions;
    Assets::DnmModel m_dnmModel;
    std::optional<Assets::CockpitPanelLayout> m_cockpitPanel;

    // Raw sources preserved for lossless round-trip
    std::string m_rawDnmContent;
    std::string m_rawIstContent;

    // Active animation channels [animClass -> [0.0, 1.0]]
    std::unordered_map<int, f32> m_animationStates;
};

} // namespace LokYS::Tools
