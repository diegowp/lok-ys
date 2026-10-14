#pragma once

#include "LokYS/navigation/NavTypes.hpp"
#include <vector>
#include <optional>

namespace LokYS::Navigation {

class NavDatabase {
public:
    NavDatabase();

    /// @brief Clears and populates the database with regional default navaids.
    void PopulateDefaults();

    /// @brief Registers an external or custom navigation facility.
    void AddFacility(const NavFacility& facility);

    /// @brief Finds the best line-of-sight facility broadcasting on the specified frequency.
    /// Frequency matching tolerance is 0.02 MHz (accounting for channel spacing).
    [[nodiscard]] const NavFacility* FindByFrequency(
        f64 freqMhz,
        const Math::Vec3d& aircraftPosNed) const noexcept;

    /// @brief Retrieves the active approach marker beacon given current aircraft position.
    [[nodiscard]] MarkerBeaconType QueryMarkerBeacon(
        const Math::Vec3d& aircraftPosNed) const noexcept;

    /// @brief Checks line-of-sight radio propagation and returns received signal strength [0, 1].
    [[nodiscard]] static f64 CalculateSignalStrength(
        const Math::Vec3d& aircraftPosNed,
        const NavFacility& facility) noexcept;

    [[nodiscard]] const std::vector<NavFacility>& GetAllFacilities() const noexcept {
        return m_facilities;
    }

private:
    std::vector<NavFacility> m_facilities;
};

} // namespace LokYS::Navigation
