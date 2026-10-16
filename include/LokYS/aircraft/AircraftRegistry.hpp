#pragma once

#include "LokYS/aircraft/AircraftTypes.hpp"
#include <array>
#include <memory>

namespace LokYS::Aircraft {

class AircraftRegistry {
public:
    AircraftRegistry();
    ~AircraftRegistry() = default;

    /// @brief Retrieves the configuration for a given aircraft type.
    [[nodiscard]] const AircraftConfig& GetConfig(AircraftType type) const noexcept;

    /// @brief Gets the currently active aircraft configuration.
    [[nodiscard]] const AircraftConfig& GetActiveConfig() const noexcept {
        if (m_isCustomActive) {
            return m_customConfig;
        }
        return GetConfig(m_activeType);
    }

    /// @brief Sets a custom dynamic aircraft configuration (e.g. from YSFlight package).
    void SetCustomConfig(const AircraftConfig& cfg) noexcept {
        m_customConfig = cfg;
        m_isCustomActive = true;
        m_gearExtended = true;
    }

    /// @brief Clears custom config and restores active built-in archetype.
    void ClearCustomConfig() noexcept {
        m_isCustomActive = false;
    }

    /// @brief Gets whether a custom aircraft configuration is currently active.
    [[nodiscard]] bool IsCustomConfigActive() const noexcept { return m_isCustomActive; }

    /// @brief Gets the currently active aircraft type.
    [[nodiscard]] AircraftType GetActiveType() const noexcept { return m_activeType; }

    /// @brief Sets the active aircraft type and resets transient gear state.
    void SetActiveType(AircraftType type) noexcept;
    void SetActiveAircraft(AircraftType type) noexcept { SetActiveType(type); }

    /// @brief Cycles to the next available aircraft archetype in sequence.
    /// Cessna 172 -> Baron 58 -> Extra 330SC -> Cessna 172
    AircraftType CycleAircraft() noexcept;
    AircraftType CycleNextAircraft() noexcept { return CycleAircraft(); }

    // Retractable Landing Gear control
    [[nodiscard]] bool IsGearExtended() const noexcept { return m_gearExtended; }
    void SetGearExtended(bool extended) noexcept { m_gearExtended = extended; }
    void ToggleGear() noexcept;
    void ToggleLandingGear() noexcept { ToggleGear(); }

    /// @brief Calculates asymmetric twin-engine yaw moment for the active aircraft.
    [[nodiscard]] Math::Vec3d ComputeMultiEngineForces(f64 thrustLeftN, f64 thrustRightN) const noexcept {
        Math::Vec3d force{};
        Math::Vec3d moments{};
        ComputeMultiEngineForces(GetActiveConfig(), thrustLeftN, thrustRightN, force, moments);
        return moments;
    }

    /// @brief Calculates asymmetric twin-engine yaw moment and combined thrust.
    /// For twin-engine aircraft (Baron 58), asymmetric throttle commands create yaw moments.
    static void ComputeMultiEngineForces(
        const AircraftConfig& config,
        f64 thrustLeftN,
        f64 thrustRightN,
        Math::Vec3d& outTotalForceBody,
        Math::Vec3d& outAsymMomentBody) noexcept;

private:
    void InitializeConfigurations();

    AircraftType m_activeType{ AircraftType::Cessna172Skyhawk };
    bool m_gearExtended{ true }; // Gear down by default for takeoff/landing
    bool m_isCustomActive{ false };
    AircraftConfig m_customConfig{};

    std::array<AircraftConfig, 3> m_configs;
};

} // namespace LokYS::Aircraft
