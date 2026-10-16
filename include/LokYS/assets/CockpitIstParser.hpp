#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <string_view>
#include <optional>

namespace LokYS::Assets {

struct InstrumentDial {
    float x{ 0.0f };
    float y{ 0.0f };
    float radius{ 0.15f };
    bool isPresent{ false };
};

struct RectRegion {
    float x1{ 0.0f };
    float y1{ 0.0f };
    float x2{ 0.0f };
    float y2{ 0.0f };
    bool isPresent{ false };
};

struct TachometerDial : public InstrumentDial {
    float minRpm{ 0.0f };
    float maxRpm{ 3500.0f };
    float greenArcMin{ 2000.0f };
    float greenArcMax{ 2500.0f };
    float redLineRpm{ 2700.0f };
};

struct CockpitPanelLayout {
    RectRegion panelRange{ -1.0f, -1.0f, 1.0f, 1.0f, true };
    InstrumentDial airspeed;
    InstrumentDial attitude;
    InstrumentDial altimeter;
    InstrumentDial turnCoordinator;
    InstrumentDial heading;
    InstrumentDial verticalSpeed;
    InstrumentDial ils;
    InstrumentDial vor;
    InstrumentDial adf;
    TachometerDial tachometer;
    RectRegion flapGauge;
    RectRegion fuelGauge;
    RectRegion brakeIndicator;

    float speedVfeKnots{ 85.0f };
    float speedVnoKnots{ 129.0f };
    float speedVneKnots{ 163.0f };
    float speedRangeKnots{ 200.0f };
};

class CockpitIstParser {
public:
    // Parse an instrument panel script (.ist) from text
    [[nodiscard]] static std::optional<CockpitPanelLayout> ParseFromString(std::string_view text);

    // Parse an instrument panel script (.ist) from file
    [[nodiscard]] static std::optional<CockpitPanelLayout> ParseFromFile(const std::string& filePath);
};

} // namespace LokYS::Assets
