#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/vfs/ZipVfsReader.hpp"
#include "LokYS/assets/SceneryFldParser.hpp"
#include "LokYS/assets/SpawnStpParser.hpp"
#include <string>
#include <string_view>
#include <optional>

namespace LokYS::Assets {

struct LfsManifest {
    u32 formatVersion{ 1 };
    std::string type{ "Scenery" };
    std::string identifier;
    std::string displayName;
    std::string author{ "LokYS Interop" };
    std::string fieldFile;
    std::string spawnFile;
};

class LfsPackage {
public:
    LfsPackage() = default;
    ~LfsPackage() = default;

    // Load scenery package from a .lfs archive
    [[nodiscard]] bool LoadFromArchive(Vfs::ZipVfsReader& vfs);
    [[nodiscard]] bool LoadFromLfsFile(const std::string& lfsFilePath);

    // Load scenery package from a directory with loose .fld and .stp files
    [[nodiscard]] bool LoadFromDirectory(
        const std::string& directoryPath,
        const std::string& fldFileName,
        const std::string& stpFileName);

    [[nodiscard]] bool IsValid() const noexcept {
        return m_isLoaded && m_field.has_value();
    }

    [[nodiscard]] const LfsManifest& GetManifest() const noexcept { return m_manifest; }
    [[nodiscard]] const SceneryField& GetField() const noexcept { return *m_field; }
    [[nodiscard]] const ScenerySpawnList& GetSpawns() const noexcept { return m_spawns; }
    [[nodiscard]] const Scenery::AirportConfig& GetAirportConfig() const noexcept { return m_airport; }

private:
    bool ParseManifestJson(std::string_view jsonText);

    bool m_isLoaded{ false };
    LfsManifest m_manifest;
    std::optional<SceneryField> m_field;
    ScenerySpawnList m_spawns;
    Scenery::AirportConfig m_airport;
};

} // namespace LokYS::Assets
