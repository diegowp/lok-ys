#pragma once

#include "LokYS/core/Types.hpp"
#include "LokYS/navigation/NavTypes.hpp"
#include "LokYS/navigation/NavDatabase.hpp"
#include "LokYS/math/Vector3.hpp"

namespace LokYS::Navigation {

class RadioNavSystem {
public:
    RadioNavSystem();
    ~RadioNavSystem() = default;

    /// @brief Initializes default database and receiver frequencies.
    void Initialize();

    /// @brief Main per-timestep update advancing radio signals, CDI deflection, DME, and marker beacons.
    void Update(
        const Math::Vec3d& aircraftPosNed,
        const Math::Vec3d& velocityNed,
        f64 dt);

    [[nodiscard]] const NavReceiverState& GetNav1() const noexcept { return m_nav1; }
    [[nodiscard]] const NavReceiverState& GetNav2() const noexcept { return m_nav2; }
    [[nodiscard]] const NavDatabase& GetDatabase() const noexcept { return m_database; }

    // Pilot Controls
    void SwapNav1Frequencies() noexcept;
    void AdjustNav1StandbyFrequency(f64 deltaMhz) noexcept;
    void AdjustNav1Obs(f64 deltaDeg) noexcept;
    void ToggleNav1AudioIdent() noexcept;

    // =========================================================================
    // Pure Mathematical & Physical Radionavigation Functions
    // =========================================================================

    [[nodiscard]] static f64 NormalizeDegrees360(f64 deg) noexcept;
    [[nodiscard]] static f64 NormalizeDegrees180(f64 deg) noexcept;

    /// @brief Computes magnetic radial (0..360 deg) FROM the VOR station to the aircraft.
    [[nodiscard]] static f64 CalculateVorRadial(
        const Math::Vec3d& aircraftNed,
        const Math::Vec3d& vorNed,
        f64 magVarDeg = 0.0) noexcept;

    /// @brief Resolves TO/FROM/NAV flag status based on selected OBS course and sensed radial.
    static void ResolveVorToFrom(
        f64 obsDeg,
        f64 radialDeg,
        bool& toFlag,
        bool& fromFlag,
        bool& navFlag) noexcept;

    /// @brief Computes CDI needle deflection in standard instrument dots [-5.0, 5.0].
    /// 1 dot = 2.0 degrees angular deviation for VOR.
    [[nodiscard]] static f64 CalculateCdiDeflection(
        f64 obsDeg,
        f64 radialDeg,
        bool toFlag) noexcept;

    /// @brief Computes ILS localizer needle deflection in dots [-5.0, 5.0].
    /// 1 dot = 0.5 degrees angular deviation (4x more sensitive than VOR).
    [[nodiscard]] static f64 CalculateLocalizerDeflection(
        const Math::Vec3d& aircraftNed,
        const Math::Vec3d& locAntennaNed,
        f64 rwyHeadingDeg) noexcept;

    /// @brief Computes ILS glideslope needle deflection in dots [-5.0, 5.0].
    /// 1 dot = 0.14 degrees elevation error relative to standard 3.0 deg descent path.
    [[nodiscard]] static f64 CalculateGlideslopeDeflection(
        const Math::Vec3d& aircraftNed,
        const Math::Vec3d& gsAntennaNed,
        f64 rwyHeadingDeg,
        f64 targetGsAngleDeg = 3.0) noexcept;

    /// @brief Computes radar direct slant range in nautical miles.
    [[nodiscard]] static f64 CalculateDmeSlantRangeNm(
        const Math::Vec3d& aircraftNed,
        const Math::Vec3d& dmeNed) noexcept;

private:
    NavDatabase m_database;
    NavReceiverState m_nav1{};
    NavReceiverState m_nav2{};

    f64 m_prevDmeRangeNm{ 0.0 };
    bool m_dmeInitialized{ false };
};

} // namespace LokYS::Navigation
