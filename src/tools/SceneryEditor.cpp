#include "LokYS/tools/SceneryEditor.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace LokYS::Tools {

namespace {

std::string ReadEntireFileText(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return "";
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

} // namespace

bool SceneryEditor::LoadFromFiles(
    const std::string& fldPath,
    const std::string& stpPath)
{
    std::string fldText = ReadEntireFileText(fldPath);
    if (fldText.empty()) {
        return false;
    }

    std::string stpText = !stpPath.empty() ? ReadEntireFileText(stpPath) : "";
    return LoadFromMemory(fldText, stpText, fldPath);
}

bool SceneryEditor::LoadFromMemory(
    std::string_view fldText,
    std::string_view stpText,
    const std::string& sceneryName)
{
    auto parsedFld = Assets::SceneryFldParser::ParseFromString(fldText, sceneryName);
    if (!parsedFld) {
        return false;
    }

    m_field = std::move(*parsedFld);
    m_sceneryName = sceneryName;

    if (!stpText.empty()) {
        auto parsedStp = Assets::SpawnStpParser::ParseFromString(stpText);
        if (parsedStp) {
            m_spawns = std::move(*parsedStp);
        }
    } else {
        m_spawns.points.clear();
    }

    return true;
}

bool SceneryEditor::LoadFromLfsBuffer(std::span<const uint8_t> lfsBuffer) {
    Assets::LfsPackage pkg;
    if (!pkg.LoadFromMemory(lfsBuffer)) {
        return false;
    }

    m_field = pkg.GetField();
    m_spawns = pkg.GetSpawnList();
    m_sceneryName = pkg.GetManifest().name;
    return true;
}

bool SceneryEditor::LoadFromLfsFile(const std::string& lfsPath) {
    std::ifstream file(lfsPath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        return false;
    }

    return LoadFromLfsBuffer(buffer);
}

f32 SceneryEditor::SampleElevationAt(f64 xNed, f64 yNed) const noexcept {
    return m_field.SampleElevationAt(xNed, yNed);
}

void SceneryEditor::SculptElevation(
    f64 centerNorth,
    f64 centerEast,
    f64 radiusM,
    f32 deltaHeightM) noexcept
{
    if (radiusM <= 0.0) return;

    for (auto& block : m_field.terrainBlocks) {
        for (u32 r = 0; r < block.rows; ++r) {
            f64 cellNorth = block.originNed.x + r * block.stepX;
            for (u32 c = 0; c < block.cols; ++c) {
                f64 cellEast = block.originNed.y + c * block.stepZ;

                f64 dNorth = cellNorth - centerNorth;
                f64 dEast = cellEast - centerEast;
                f64 dist = std::sqrt(dNorth * dNorth + dEast * dEast);

                if (dist <= radiusM) {
                    f64 weight = 1.0 - (dist / radiusM);
                    size_t idx = static_cast<size_t>(r * block.cols + c);
                    if (idx < block.elevations.size()) {
                        block.elevations[idx] += static_cast<f32>(deltaHeightM * weight);
                    }
                }
            }
        }
    }
}

void SceneryEditor::FlattenElevation(
    f64 centerNorth,
    f64 centerEast,
    f64 radiusM,
    f32 targetHeightM) noexcept
{
    if (radiusM <= 0.0) return;

    for (auto& block : m_field.terrainBlocks) {
        for (u32 r = 0; r < block.rows; ++r) {
            f64 cellNorth = block.originNed.x + r * block.stepX;
            for (u32 c = 0; c < block.cols; ++c) {
                f64 cellEast = block.originNed.y + c * block.stepZ;

                f64 dNorth = cellNorth - centerNorth;
                f64 dEast = cellEast - centerEast;
                f64 dist = std::sqrt(dNorth * dNorth + dEast * dEast);

                if (dist <= radiusM) {
                    f64 weight = 1.0 - (dist / radiusM);
                    size_t idx = static_cast<size_t>(r * block.cols + c);
                    if (idx < block.elevations.size()) {
                        f32 cur = block.elevations[idx];
                        block.elevations[idx] = static_cast<f32>(cur * (1.0 - weight) + targetHeightM * weight);
                    }
                }
            }
        }
    }
}

Math::Vec3d SceneryEditor::GetTerrainMinBounds() const noexcept {
    if (m_field.terrainBlocks.empty()) {
        return Math::Vec3d{ 0.0, 0.0, 0.0 };
    }

    Math::Vec3d minB{ 1e9, 1e9, 1e9 };
    for (const auto& block : m_field.terrainBlocks) {
        minB.x = std::min(minB.x, block.originNed.x);
        minB.y = std::min(minB.y, block.originNed.y);
        for (f32 elev : block.elevations) {
            minB.z = std::min(minB.z, static_cast<f64>(-elev)); // Down is negative elevation
        }
    }
    return minB;
}

Math::Vec3d SceneryEditor::GetTerrainMaxBounds() const noexcept {
    if (m_field.terrainBlocks.empty()) {
        return Math::Vec3d{ 0.0, 0.0, 0.0 };
    }

    Math::Vec3d maxB{ -1e9, -1e9, -1e9 };
    for (const auto& block : m_field.terrainBlocks) {
        f64 extentX = block.originNed.x + block.rows * block.stepX;
        f64 extentY = block.originNed.y + block.cols * block.stepZ;
        maxB.x = std::max(maxB.x, extentX);
        maxB.y = std::max(maxB.y, extentY);
        for (f32 elev : block.elevations) {
            maxB.z = std::max(maxB.z, static_cast<f64>(-elev));
        }
    }
    return maxB;
}

size_t SceneryEditor::GetPlacedObjectCount() const noexcept {
    return m_field.placedObjects.size();
}

const Assets::PlacedSceneryObject* SceneryEditor::GetPlacedObject(size_t index) const noexcept {
    if (index >= m_field.placedObjects.size()) {
        return nullptr;
    }
    return &m_field.placedObjects[index];
}

size_t SceneryEditor::AddPlacedObject(const Assets::PlacedSceneryObject& obj) {
    m_field.placedObjects.push_back(obj);
    return m_field.placedObjects.size() - 1;
}

bool SceneryEditor::RemovePlacedObject(size_t index) {
    if (index >= m_field.placedObjects.size()) {
        return false;
    }
    m_field.placedObjects.erase(m_field.placedObjects.begin() + index);
    return true;
}

bool SceneryEditor::SetObjectTransform(
    size_t index,
    const Math::Vec3d& posNed,
    f64 headingDeg,
    f64 pitchDeg,
    f64 rollDeg)
{
    if (index >= m_field.placedObjects.size()) {
        return false;
    }
    auto& obj = m_field.placedObjects[index];
    obj.positionNed = posNed;
    obj.headingDeg = headingDeg;
    obj.pitchDeg = pitchDeg;
    obj.rollDeg = rollDeg;
    return true;
}

bool SceneryEditor::SnapObjectToTerrain(size_t index) {
    if (index >= m_field.placedObjects.size()) {
        return false;
    }
    auto& obj = m_field.placedObjects[index];
    f32 elevMsl = SampleElevationAt(obj.positionNed.x, obj.positionNed.y);
    obj.positionNed.z = -static_cast<f64>(elevMsl); // Down = -MSL
    return true;
}

size_t SceneryEditor::GetSpawnPointCount() const noexcept {
    return m_spawns.points.size();
}

const Assets::SpawnPoint* SceneryEditor::GetSpawnPoint(size_t index) const noexcept {
    if (index >= m_spawns.points.size()) {
        return nullptr;
    }
    return &m_spawns.points[index];
}

size_t SceneryEditor::AddSpawnPoint(const Assets::SpawnPoint& pt) {
    m_spawns.points.push_back(pt);
    return m_spawns.points.size() - 1;
}

bool SceneryEditor::RemoveSpawnPoint(size_t index) {
    if (index >= m_spawns.points.size()) {
        return false;
    }
    m_spawns.points.erase(m_spawns.points.begin() + index);
    return true;
}

bool SceneryEditor::UpdateSpawnPoint(size_t index, const Assets::SpawnPoint& pt) {
    if (index >= m_spawns.points.size()) {
        return false;
    }
    m_spawns.points[index] = pt;
    return true;
}

size_t SceneryEditor::GetRunwayCount() const noexcept {
    return m_field.runways.size();
}

const Scenery::RunwayConfig* SceneryEditor::GetRunway(size_t index) const noexcept {
    if (index >= m_field.runways.size()) {
        return nullptr;
    }
    return &m_field.runways[index];
}

void SceneryEditor::AddRunway(const Scenery::RunwayConfig& runway) {
    m_field.runways.push_back(runway);
}

SceneryEditorStats SceneryEditor::GetStats() const noexcept {
    SceneryEditorStats s;
    s.name = m_sceneryName;
    s.terrainBlockCount = m_field.terrainBlocks.size();
    s.runwayCount = m_field.runways.size();
    s.placedObjectCount = m_field.placedObjects.size();
    s.spawnPointCount = m_spawns.points.size();
    s.minBounds = GetTerrainMinBounds();
    s.maxBounds = GetTerrainMaxBounds();
    return s;
}

std::string SceneryEditor::GenerateFldText() const {
    std::stringstream ss;
    ss << "# LokYS Field Scenery File (Exported by SceneryEditor)\n";
    ss << "GND " << static_cast<int>(m_field.gndColor.x * 255.0f) << " "
       << static_cast<int>(m_field.gndColor.y * 255.0f) << " "
       << static_cast<int>(m_field.gndColor.z * 255.0f) << "\n";
    ss << "SKY " << static_cast<int>(m_field.skyColor.x * 255.0f) << " "
       << static_cast<int>(m_field.skyColor.y * 255.0f) << " "
       << static_cast<int>(m_field.skyColor.z * 255.0f) << "\n\n";

    // Export TerrMesh blocks
    for (const auto& block : m_field.terrainBlocks) {
        // Coordinate conversion NED -> YS:
        // X_YS = Y_NED (East)
        // Y_YS = -Z_NED (Up)
        // Z_YS = X_NED (North)
        f64 originX_YS = block.originNed.y;
        f64 originY_YS = -block.originNed.z;
        f64 originZ_YS = block.originNed.x;

        ss << "TMS\n";
        ss << "NBL " << block.cols << " " << block.rows << " "
           << std::fixed << std::setprecision(1) << block.stepZ << " " << block.stepX << " "
           << originX_YS << " " << originY_YS << " " << originZ_YS << "\n";
        ss << "BLO\n";

        for (u32 r = 0; r < block.rows; ++r) {
            for (u32 c = 0; c < block.cols; ++c) {
                size_t idx = static_cast<size_t>(r * block.cols + c);
                f32 elev = (idx < block.elevations.size()) ? block.elevations[idx] : 0.0f;
                ss << std::fixed << std::setprecision(2) << elev << (c + 1 < block.cols ? " " : "");
            }
            ss << "\n";
        }
        ss << "END\n\n";
    }

    // Export Runways as RGN TAG "RW"
    for (const auto& rwy : m_field.runways) {
        f64 halfLen = rwy.lengthM * 0.5;
        f64 halfWid = rwy.widthM * 0.5;

        // Heading unit vector in NED: [cos(h), sin(h)]
        f64 hRad = rwy.headingDeg * (3.14159265358979323846 / 180.0);
        f64 cosH = std::cos(hRad);
        f64 sinH = std::sin(hRad);

        // 4 corners in NED:
        // C1 = Center + Forward * halfLen - Right * halfWid
        // C2 = Center + Forward * halfLen + Right * halfWid
        // C3 = Center - Forward * halfLen + Right * halfWid
        // C4 = Center - Forward * halfLen - Right * halfWid
        // In NED: Forward = [cosH, sinH], Right = [sinH, -cosH]
        Math::Vec3d c[4] = {
            { rwy.centerNed.x + cosH * halfLen - sinH * halfWid, rwy.centerNed.y + sinH * halfLen + cosH * halfWid, rwy.centerNed.z },
            { rwy.centerNed.x + cosH * halfLen + sinH * halfWid, rwy.centerNed.y + sinH * halfLen - cosH * halfWid, rwy.centerNed.z },
            { rwy.centerNed.x - cosH * halfLen + sinH * halfWid, rwy.centerNed.y - sinH * halfLen - cosH * halfWid, rwy.centerNed.z },
            { rwy.centerNed.x - cosH * halfLen - sinH * halfWid, rwy.centerNed.y - sinH * halfLen + cosH * halfWid, rwy.centerNed.z }
        };

        ss << "RGN TAG \"RW\"\n";
        ss << "POL 4\n";
        for (int i = 0; i < 4; ++i) {
            // Convert to YS coordinates (X_YS = East, Y_YS = Up, Z_YS = North)
            f64 x_YS = c[i].y;
            f64 y_YS = -c[i].z;
            f64 z_YS = c[i].x;
            ss << std::fixed << std::setprecision(1) << x_YS << " " << y_YS << " " << z_YS << "\n";
        }
        ss << "END\n\n";
    }

    // Export Placed Objects
    for (const auto& obj : m_field.placedObjects) {
        f64 x_YS = obj.positionNed.y;
        f64 y_YS = -obj.positionNed.z;
        f64 z_YS = obj.positionNed.x;

        ss << "POS " << std::fixed << std::setprecision(2)
           << x_YS << " " << y_YS << " " << z_YS << " "
           << obj.headingDeg << " " << obj.pitchDeg << " " << obj.rollDeg << "\n";
        ss << "TAG \"" << obj.tag << "\"\n";
    }

    return ss.str();
}

std::string SceneryEditor::GenerateStpText() const {
    std::stringstream ss;
    ss << "# LokYS Scenery Starting Positions File (Exported by SceneryEditor)\n";

    for (const auto& pt : m_spawns.points) {
        f64 x_YS = pt.positionNed.y;
        f64 y_YS = -pt.positionNed.z;
        f64 z_YS = pt.positionNed.x;

        f64 speedKts = pt.initialSpeedMps * 1.943844;

        ss << "N " << pt.name << "\n";
        ss << "C POSITION " << std::fixed << std::setprecision(2)
           << x_YS << " " << y_YS << " " << z_YS << "\n";
        ss << "C ATTITUDE " << std::fixed << std::setprecision(1)
           << pt.headingDeg << " " << pt.pitchDeg << " " << pt.rollDeg << "\n";
        ss << "C INITSPED " << static_cast<int>(speedKts) << "kt\n";
        ss << "C CTLTHROT " << std::fixed << std::setprecision(2) << pt.initialThrottle << "\n";
        ss << "C CTLLDGEA " << (pt.gearDown ? "1" : "0") << "\n\n";
    }

    return ss.str();
}

std::vector<uint8_t> SceneryEditor::CompileToLfsPackage() const {
    Vfs::ZipWriter writer;

    // 1. Manifest
    std::stringstream manifest;
    manifest << "{\n"
             << "  \"formatVersion\": 1,\n"
             << "  \"type\": \"Scenery\",\n"
             << "  \"identifier\": \"" << m_sceneryName << "\",\n"
             << "  \"name\": \"" << m_sceneryName << "\",\n"
             << "  \"author\": \"LokYS Scenery Editor\",\n"
             << "  \"terrainFile\": \"scenery.fld\",\n"
             << "  \"spawnFile\": \"scenery.stp\"\n"
             << "}\n";

    writer.AddText("manifest.json", manifest.str());

    // 2. Terrain & objects .fld
    writer.AddText("scenery.fld", GenerateFldText());

    // 3. Starting points .stp
    writer.AddText("scenery.stp", GenerateStpText());

    return writer.FinalizeToMemory();
}

bool SceneryEditor::CompileToLfsFile(const std::string& outLfsPath) const {
    auto data = CompileToLfsPackage();
    std::ofstream out(outLfsPath, std::ios::binary);
    if (!out.is_open()) return false;
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return out.good();
}

} // namespace LokYS::Tools
