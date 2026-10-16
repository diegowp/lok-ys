#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/vfs/ZipVfsReader.hpp"
#include "LokYS/assets/AircraftDatParser.hpp"
#include "LokYS/assets/DnmParser.hpp"
#include "LokYS/assets/SrfParser.hpp"
#include "LokYS/assets/CockpitIstParser.hpp"
#include <string>
#include <string_view>
#include <optional>
#include <memory>

namespace LokYS::Assets {

struct AcftManifest {
    u32 formatVersion{ 1 };
    std::string type{ "Aircraft" };
    std::string identifier;
    std::string displayName;
    std::string author{ "LokYS Interop" };
    std::string flightModelFile;
    std::string visualModelFile;
    std::string instrumentPanelFile;
    std::string collisionMeshFile;
    std::string cockpitInteriorFile;
};

class AcftPackage {
public:
    AcftPackage() = default;
    ~AcftPackage() = default;

    // Load aircraft from a packaged .acft.zip archive in memory or on disk
    [[nodiscard]] bool LoadFromArchive(Vfs::ZipVfsReader& vfs);
    [[nodiscard]] bool LoadFromZipFile(const std::string& zipFilePath);

    // Load aircraft from a directory with loose legacy files
    [[nodiscard]] bool LoadFromDirectory(
        const std::string& directoryPath,
        const std::string& datFileName);

    [[nodiscard]] bool IsValid() const noexcept {
        return m_isLoaded && m_flightData.has_value() && m_visualModel.has_value();
    }

    [[nodiscard]] const AcftManifest& GetManifest() const noexcept { return m_manifest; }
    [[nodiscard]] const ParsedAircraftDat& GetFlightData() const noexcept { return *m_flightData; }
    [[nodiscard]] const DnmModel& GetVisualModel() const noexcept { return *m_visualModel; }
    [[nodiscard]] const std::optional<CockpitPanelLayout>& GetCockpitPanel() const noexcept { return m_cockpitPanel; }
    [[nodiscard]] const std::optional<SrfModel>& GetCollisionMesh() const noexcept { return m_collisionMesh; }
    [[nodiscard]] const std::optional<SrfModel>& GetCockpitInterior() const noexcept { return m_cockpitInterior; }

private:
    bool ParseManifestJson(std::string_view jsonText);

    bool m_isLoaded{ false };
    AcftManifest m_manifest;
    std::optional<ParsedAircraftDat> m_flightData;
    std::optional<DnmModel> m_visualModel;
    std::optional<CockpitPanelLayout> m_cockpitPanel;
    std::optional<SrfModel> m_collisionMesh;
    std::optional<SrfModel> m_cockpitInterior;
};

} // namespace LokYS::Assets
