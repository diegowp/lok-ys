#include "LokYS/assets/AcftPackage.hpp"
#include "LokYS/core/JsonParser.hpp"
#include <filesystem>
#include <iostream>

namespace LokYS::Assets {

bool AcftPackage::ParseManifestJson(std::string_view jsonText) {
    Core::JsonValue root = Core::JsonValue::Parse(jsonText);
    if (!root.IsObject()) {
        return false;
    }

    m_manifest.formatVersion = static_cast<u32>(root[std::string("formatVersion")].AsInt(1));
    m_manifest.type = root[std::string("type")].AsString("Aircraft");
    m_manifest.identifier = root[std::string("identifier")].AsString("");
    m_manifest.displayName = root[std::string("displayName")].AsString(m_manifest.identifier);
    m_manifest.author = root[std::string("author")].AsString("Unknown");

    const auto& files = root[std::string("files")];
    if (files.IsObject()) {
        m_manifest.flightModelFile = files[std::string("flightModel")].AsString("");
        m_manifest.visualModelFile = files[std::string("visualModel")].AsString("");
        m_manifest.instrumentPanelFile = files[std::string("instrumentPanel")].AsString("");
        m_manifest.collisionMeshFile = files[std::string("collisionMesh")].AsString("");
        m_manifest.cockpitInteriorFile = files[std::string("cockpitInterior")].AsString("");
    }

    return !m_manifest.flightModelFile.empty();
}

bool AcftPackage::LoadFromArchive(Vfs::ZipVfsReader& vfs) {
    m_isLoaded = false;
    m_flightData.reset();
    m_visualModel.reset();
    m_cockpitPanel.reset();
    m_collisionMesh.reset();
    m_cockpitInterior.reset();

    // 1. Try reading manifest.json
    auto manifestText = vfs.ReadText("manifest.json");
    if (manifestText && ParseManifestJson(*manifestText)) {
        // Manifest successfully parsed
    } else {
        // Auto-discover files in archive
        auto allFiles = vfs.ListFiles();
        for (const auto& file : allFiles) {
            if (file.ends_with(".dat")) {
                m_manifest.flightModelFile = file;
            } else if (file.ends_with(".dnm") && file.find("coarse") == std::string::npos) {
                m_manifest.visualModelFile = file;
            } else if (file.ends_with(".ist")) {
                m_manifest.instrumentPanelFile = file;
            } else if (file.find("coll") != std::string::npos && file.ends_with(".srf")) {
                m_manifest.collisionMeshFile = file;
            } else if (file.find("cockpit") != std::string::npos && file.ends_with(".srf")) {
                m_manifest.cockpitInteriorFile = file;
            }
        }
    }

    if (m_manifest.flightModelFile.empty()) {
        return false;
    }

    // 2. Read and parse .dat
    auto datText = vfs.ReadText(m_manifest.flightModelFile);
    if (!datText) {
        return false;
    }
    m_flightData = AircraftDatParser::ParseFromString(*datText, m_manifest.displayName);
    if (!m_flightData) {
        return false;
    }

    // 3. Read and parse .dnm
    if (!m_manifest.visualModelFile.empty()) {
        auto dnmText = vfs.ReadText(m_manifest.visualModelFile);
        if (dnmText) {
            m_visualModel = DnmParser::ParseFromString(*dnmText, m_flightData->config.name);
        }
    }

    // 4. Read and parse .ist (if specified in manifest or inside .dat)
    std::string istFile = m_manifest.instrumentPanelFile;
    if (istFile.empty() && !m_flightData->cockpitPanelPath.empty()) {
        // Strip folder prefix if present in YS dat (e.g. "aircraft/cessna172r.ist" -> "cessna172r.ist")
        size_t slash = m_flightData->cockpitPanelPath.find_last_of("/\\");
        istFile = (slash != std::string::npos) ? m_flightData->cockpitPanelPath.substr(slash + 1) : m_flightData->cockpitPanelPath;
    }
    if (!istFile.empty()) {
        auto istText = vfs.ReadText(istFile);
        if (istText) {
            m_cockpitPanel = CockpitIstParser::ParseFromString(*istText);
        }
    }

    // 5. Read collision .srf
    if (!m_manifest.collisionMeshFile.empty()) {
        auto srfText = vfs.ReadText(m_manifest.collisionMeshFile);
        if (srfText) {
            m_collisionMesh = SrfParser::ParseFromString(*srfText, "CollisionMesh");
        }
    }

    // 6. Read cockpit interior .srf
    if (!m_manifest.cockpitInteriorFile.empty()) {
        auto srfText = vfs.ReadText(m_manifest.cockpitInteriorFile);
        if (srfText) {
            m_cockpitInterior = SrfParser::ParseFromString(*srfText, "CockpitInterior");
        }
    }

    m_isLoaded = (m_flightData.has_value() && m_visualModel.has_value());
    return m_isLoaded;
}

bool AcftPackage::LoadFromZipFile(const std::string& zipFilePath) {
    Vfs::ZipVfsReader vfs;
    if (!vfs.LoadFromFile(zipFilePath)) {
        return false;
    }
    return LoadFromArchive(vfs);
}

bool AcftPackage::LoadFromDirectory(
    const std::string& directoryPath,
    const std::string& datFileName)
{
    m_isLoaded = false;
    m_flightData.reset();
    m_visualModel.reset();
    m_cockpitPanel.reset();
    m_collisionMesh.reset();
    m_cockpitInterior.reset();

    std::filesystem::path dir(directoryPath);
    std::filesystem::path datPath = dir / datFileName;

    // Parse .dat
    m_flightData = AircraftDatParser::ParseFromFile(datPath.string());
    if (!m_flightData) {
        return false;
    }

    // Derive stem name e.g. "cessna172r" from "cessna172r.dat"
    std::string stem = datPath.stem().string();

    // Look for .dnm
    std::filesystem::path dnmPath = dir / (stem + ".dnm");
    if (!std::filesystem::exists(dnmPath)) {
        // Fallback: look for coarse or variant
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (entry.path().extension() == ".dnm") {
                dnmPath = entry.path();
                break;
            }
        }
    }

    if (std::filesystem::exists(dnmPath)) {
        m_visualModel = DnmParser::ParseFromFile(dnmPath.string());
    }

    // Look for .ist
    std::string istName = m_flightData->cockpitPanelPath;
    if (!istName.empty()) {
        size_t slash = istName.find_last_of("/\\");
        if (slash != std::string::npos) istName = istName.substr(slash + 1);
    } else {
        istName = stem + ".ist";
    }

    std::filesystem::path istPath = dir / istName;
    if (std::filesystem::exists(istPath)) {
        m_cockpitPanel = CockpitIstParser::ParseFromFile(istPath.string());
    }

    // Look for collision .srf
    std::filesystem::path collPath = dir / (stem + "coll.srf");
    if (!std::filesystem::exists(collPath)) {
        collPath = dir / (stem + "_collision.srf");
    }
    if (std::filesystem::exists(collPath)) {
        m_collisionMesh = SrfParser::ParseFromFile(collPath.string());
    }

    // Look for cockpit interior .srf
    std::filesystem::path cockPath = dir / (stem + "cockpit.srf");
    if (!std::filesystem::exists(cockPath)) {
        cockPath = dir / (stem + "_cockpit.srf");
    }
    if (std::filesystem::exists(cockPath)) {
        m_cockpitInterior = SrfParser::ParseFromFile(cockPath.string());
    }

    m_manifest.identifier = m_flightData->config.name;
    m_manifest.displayName = m_flightData->config.name;
    m_manifest.flightModelFile = datFileName;
    m_manifest.visualModelFile = dnmPath.filename().string();
    m_manifest.instrumentPanelFile = istPath.filename().string();

    m_isLoaded = (m_flightData.has_value() && m_visualModel.has_value());
    return m_isLoaded;
}

} // namespace LokYS::Assets
