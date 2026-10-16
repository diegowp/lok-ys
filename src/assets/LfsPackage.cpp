#include "LokYS/assets/LfsPackage.hpp"
#include "LokYS/core/JsonParser.hpp"
#include <filesystem>

namespace LokYS::Assets {

bool LfsPackage::ParseManifestJson(std::string_view jsonText) {
    Core::JsonValue root = Core::JsonValue::Parse(jsonText);
    if (!root.IsObject()) {
        return false;
    }

    m_manifest.formatVersion = static_cast<u32>(root[std::string("formatVersion")].AsInt(1));
    m_manifest.type = root[std::string("type")].AsString("Scenery");
    m_manifest.identifier = root[std::string("identifier")].AsString("");
    m_manifest.displayName = root[std::string("displayName")].AsString(m_manifest.identifier);
    m_manifest.author = root[std::string("author")].AsString("Unknown");

    const auto& files = root[std::string("files")];
    if (files.IsObject()) {
        m_manifest.fieldFile = files[std::string("fieldFile")].AsString("");
        m_manifest.spawnFile = files[std::string("spawnFile")].AsString("");
    }

    return !m_manifest.fieldFile.empty();
}

bool LfsPackage::LoadFromArchive(Vfs::ZipVfsReader& vfs) {
    m_isLoaded = false;
    m_field.reset();
    m_spawns = ScenerySpawnList{};

    // 1. Try reading scenery_manifest.json
    auto manifestText = vfs.ReadText("scenery_manifest.json");
    if (manifestText && ParseManifestJson(*manifestText)) {
        // Successfully parsed manifest
    } else {
        // Auto-discover files in archive
        auto allFiles = vfs.ListFiles();
        for (const auto& file : allFiles) {
            if (file.ends_with(".fld")) {
                m_manifest.fieldFile = file;
            } else if (file.ends_with(".stp")) {
                m_manifest.spawnFile = file;
            }
        }
    }

    if (m_manifest.fieldFile.empty()) {
        return false;
    }

    // 2. Read and parse .fld
    auto fldText = vfs.ReadText(m_manifest.fieldFile);
    if (!fldText) {
        return false;
    }
    m_field = SceneryFldParser::ParseFromString(*fldText, m_manifest.displayName);
    if (!m_field) {
        return false;
    }

    // 3. Read and parse .stp (optional)
    if (!m_manifest.spawnFile.empty()) {
        auto stpText = vfs.ReadText(m_manifest.spawnFile);
        if (stpText) {
            auto spawnOpt = SpawnStpParser::ParseFromString(*stpText);
            if (spawnOpt) {
                m_spawns = *spawnOpt;
            }
        }
    }

    // Generate airport config from field
    m_airport = m_field->ToAirportConfig("YFLD", m_field->name);
    m_isLoaded = true;
    return true;
}

bool LfsPackage::LoadFromLfsFile(const std::string& lfsFilePath) {
    Vfs::ZipVfsReader vfs;
    if (!vfs.LoadFromFile(lfsFilePath)) {
        return false;
    }
    return LoadFromArchive(vfs);
}

bool LfsPackage::LoadFromDirectory(
    const std::string& directoryPath,
    const std::string& fldFileName,
    const std::string& stpFileName)
{
    m_isLoaded = false;
    m_field.reset();
    m_spawns = ScenerySpawnList{};

    std::filesystem::path dir(directoryPath);
    std::filesystem::path fldPath = dir / fldFileName;
    std::filesystem::path stpPath = dir / stpFileName;

    m_field = SceneryFldParser::ParseFromFile(fldPath.string());
    if (!m_field) {
        return false;
    }

    if (std::filesystem::exists(stpPath)) {
        auto spawnOpt = SpawnStpParser::ParseFromFile(stpPath.string());
        if (spawnOpt) {
            m_spawns = *spawnOpt;
        }
    }

    m_manifest.identifier = m_field->name;
    m_manifest.displayName = m_field->name;
    m_manifest.fieldFile = fldFileName;
    m_manifest.spawnFile = stpFileName;

    m_airport = m_field->ToAirportConfig("YFLD", m_field->name);
    m_isLoaded = true;
    return true;
}

} // namespace LokYS::Assets
