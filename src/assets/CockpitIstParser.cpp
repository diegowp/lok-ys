#include "LokYS/assets/CockpitIstParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>
#include <cstdlib>

namespace LokYS::Assets {

namespace {

InstrumentDial ParseDial(const std::vector<std::string>& tokens) {
    InstrumentDial dial;
    if (tokens.size() >= 4) {
        dial.x = static_cast<float>(std::atof(tokens[1].c_str()));
        dial.y = static_cast<float>(std::atof(tokens[2].c_str()));
        dial.radius = static_cast<float>(std::atof(tokens[3].c_str()));
        dial.isPresent = true;
    }
    return dial;
}

RectRegion ParseRect(const std::vector<std::string>& tokens) {
    RectRegion rect;
    if (tokens.size() >= 5) {
        rect.x1 = static_cast<float>(std::atof(tokens[1].c_str()));
        rect.y1 = static_cast<float>(std::atof(tokens[2].c_str()));
        rect.x2 = static_cast<float>(std::atof(tokens[3].c_str()));
        rect.y2 = static_cast<float>(std::atof(tokens[4].c_str()));
        rect.isPresent = true;
    }
    return rect;
}

} // namespace

std::optional<CockpitPanelLayout> CockpitIstParser::ParseFromString(std::string_view text) {
    CockpitPanelLayout layout;

    size_t lineStart = 0;
    while (lineStart < text.size()) {
        size_t lineEnd = text.find('\n', lineStart);
        if (lineEnd == std::string_view::npos) lineEnd = text.size();

        std::string_view rawLine = text.substr(lineStart, lineEnd - lineStart);
        if (!rawLine.empty() && rawLine.back() == '\r') rawLine.remove_suffix(1);
        lineStart = lineEnd + 1;

        std::string_view cleanLine = AssetTokenizer::StripComments(rawLine);
        if (cleanLine.empty()) continue;

        auto tokens = AssetTokenizer::SplitTokens(cleanLine);
        if (tokens.empty()) continue;

        const auto& tag = tokens[0];

        if (AssetTokenizer::EqualsIgnoreCase(tag, "PNLRANGE")) {
            layout.panelRange = ParseRect(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "AIRSPEED")) {
            layout.airspeed = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "ATTITUDE")) {
            layout.attitude = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "ALTITUDE")) {
            layout.altimeter = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TRNCOORD")) {
            layout.turnCoordinator = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "HEADING_") || AssetTokenizer::EqualsIgnoreCase(tag, "HEADING")) {
            layout.heading = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "VERTICAL")) {
            layout.verticalSpeed = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "ILS_____") || AssetTokenizer::EqualsIgnoreCase(tag, "ILS")) {
            layout.ils = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "VOR_____") || AssetTokenizer::EqualsIgnoreCase(tag, "VOR")) {
            layout.vor = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "ADF_____") || AssetTokenizer::EqualsIgnoreCase(tag, "ADF")) {
            layout.adf = ParseDial(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TACHOPOS") && tokens.size() >= 5) {
            layout.tachometer.x = static_cast<float>(std::atof(tokens[2].c_str()));
            layout.tachometer.y = static_cast<float>(std::atof(tokens[3].c_str()));
            layout.tachometer.radius = static_cast<float>(std::atof(tokens[4].c_str()));
            layout.tachometer.isPresent = true;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TACHORPM") && tokens.size() >= 4) {
            layout.tachometer.minRpm = static_cast<float>(std::atof(tokens[2].c_str()));
            layout.tachometer.maxRpm = static_cast<float>(std::atof(tokens[3].c_str()));
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TACHOGRN") && tokens.size() >= 4) {
            layout.tachometer.greenArcMin = static_cast<float>(std::atof(tokens[2].c_str()));
            layout.tachometer.greenArcMax = static_cast<float>(std::atof(tokens[3].c_str()));
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TACHORED") && tokens.size() >= 3) {
            layout.tachometer.redLineRpm = static_cast<float>(std::atof(tokens[2].c_str()));
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "FLAP____") || AssetTokenizer::EqualsIgnoreCase(tag, "FLAP")) {
            layout.flapGauge = ParseRect(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "FUEL____") || AssetTokenizer::EqualsIgnoreCase(tag, "FUEL")) {
            layout.fuelGauge = ParseRect(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "BRAKE___") || AssetTokenizer::EqualsIgnoreCase(tag, "BRAKE")) {
            layout.brakeIndicator = ParseRect(tokens);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "SPEEDVFE") && tokens.size() >= 2) {
            layout.speedVfeKnots = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 85.0 * 0.5144) / 0.514444);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "SPEEDVNO") && tokens.size() >= 2) {
            layout.speedVnoKnots = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 129.0 * 0.5144) / 0.514444);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "SPEEDVNE") && tokens.size() >= 2) {
            layout.speedVneKnots = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 163.0 * 0.5144) / 0.514444);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "SPEEDRNG") && tokens.size() >= 2) {
            layout.speedRangeKnots = static_cast<float>(AssetTokenizer::ParseValueWithUnit(tokens[1], 200.0 * 0.5144) / 0.514444);
        }
    }

    return layout;
}

std::optional<CockpitPanelLayout> CockpitIstParser::ParseFromFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();

    return ParseFromString(text);
}

} // namespace LokYS::Assets
