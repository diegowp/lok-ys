#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/math/Vector3.hpp"
#include "LokYS/assets/SceneryFldParser.hpp"
#include "LokYS/assets/SpawnStpParser.hpp"
#include "LokYS/assets/LfsPackage.hpp"
#include "LokYS/vfs/ZipWriter.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include <span>

namespace LokYS::Tools {

struct SceneryEditorStats {
    std::string name;
    size_t terrainBlockCount{ 0 };
    size_t runwayCount{ 0 };
    size_t placedObjectCount{ 0 };
    size_t spawnPointCount{ 0 };
    Math::Vec3d minBounds{ 0.0, 0.0, 0.0 };
    Math::Vec3d maxBounds{ 0.0, 0.0, 0.0 };
};

class SceneryEditor {
public:
    SceneryEditor() = default;
    ~SceneryEditor() = default;

    // Load from individual legacy files
    [[nodiscard]] bool LoadFromFiles(
        const std::string& fldPath,
        const std::string& stpPath = "");

    // Load from memory text buffers
    [[nodiscard]] bool LoadFromMemory(
        std::string_view fldText,
        std::string_view stpText = "",
        const std::string& sceneryName = "Custom Scenery");

    // Load from packaged .lfs scenery archive
    [[nodiscard]] bool LoadFromLfsBuffer(std::span<const uint8_t> lfsBuffer);
    [[nodiscard]] bool LoadFromLfsFile(const std::string& lfsPath);

    // Terrain Elevation Queries & Sculpting
    [[nodiscard]] f32 SampleElevationAt(f64 xNed, f64 yNed) const noexcept;
    void SculptElevation(f64 centerNorth, f64 centerEast, f64 radiusM, f32 deltaHeightM) noexcept;
    void FlattenElevation(f64 centerNorth, f64 centerEast, f64 radiusM, f32 targetHeightM) noexcept;

    [[nodiscard]] Math::Vec3d GetTerrainMinBounds() const noexcept;
    [[nodiscard]] Math::Vec3d GetTerrainMaxBounds() const noexcept;

    // Placed Object Manipulation
    [[nodiscard]] size_t GetPlacedObjectCount() const noexcept;
    [[nodiscard]] const Assets::PlacedSceneryObject* GetPlacedObject(size_t index) const noexcept;
    size_t AddPlacedObject(const Assets::PlacedSceneryObject& obj);
    bool RemovePlacedObject(size_t index);
    bool SetObjectTransform(
        size_t index,
        const Math::Vec3d& posNed,
        f64 headingDeg,
        f64 pitchDeg = 0.0,
        f64 rollDeg = 0.0);
    bool SnapObjectToTerrain(size_t index);

    // Spawn Point & Starting Position Manager
    [[nodiscard]] size_t GetSpawnPointCount() const noexcept;
    [[nodiscard]] const Assets::SpawnPoint* GetSpawnPoint(size_t index) const noexcept;
    size_t AddSpawnPoint(const Assets::SpawnPoint& pt);
    bool RemoveSpawnPoint(size_t index);
    bool UpdateSpawnPoint(size_t index, const Assets::SpawnPoint& pt);

    // Runway Management
    [[nodiscard]] size_t GetRunwayCount() const noexcept;
    [[nodiscard]] const Scenery::RunwayConfig* GetRunway(size_t index) const noexcept;
    void AddRunway(const Scenery::RunwayConfig& runway);

    // Summary Statistics
    [[nodiscard]] SceneryEditorStats GetStats() const noexcept;

    // Serialization & Compiling to .lfs
    [[nodiscard]] std::string GenerateFldText() const;
    [[nodiscard]] std::string GenerateStpText() const;
    [[nodiscard]] std::vector<uint8_t> CompileToLfsPackage() const;
    [[nodiscard]] bool CompileToLfsFile(const std::string& outLfsPath) const;

    // Direct accessors
    [[nodiscard]] const Assets::SceneryField& GetSceneryField() const noexcept { return m_field; }
    [[nodiscard]] const Assets::ScenerySpawnList& GetSpawnList() const noexcept { return m_spawns; }
    [[nodiscard]] const std::string& GetSceneryName() const noexcept { return m_sceneryName; }
    void SetSceneryName(const std::string& name) { m_sceneryName = name; m_field.name = name; }

private:
    std::string m_sceneryName{ "Custom Scenery" };
    Assets::SceneryField m_field;
    Assets::ScenerySpawnList m_spawns;
};

} // namespace LokYS::Tools
