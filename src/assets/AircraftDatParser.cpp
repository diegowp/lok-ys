#include "LokYS/assets/AircraftDatParser.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace LokYS::Assets {

void AircraftDatParser::EstimateInertiaTensor(
    f64 massKg,
    f64 wingspanM,
    f64 lengthM,
    Math::Mat3d& outTensor) noexcept
{
    // Empirical radii of gyration for general aviation aircraft (Roskam Vol. V / NASA CR-2144)
    // Roll radius: ~ 0.25 * wingspan
    f64 r_xx = 0.25 * wingspanM;
    // Pitch radius: ~ 0.38 * length
    f64 r_yy = 0.38 * lengthM;
    // Yaw radius: ~ 0.42 * length
    f64 r_zz = 0.42 * lengthM;

    f64 Ixx = massKg * r_xx * r_xx;
    f64 Iyy = massKg * r_yy * r_yy;
    f64 Izz = massKg * r_zz * r_zz;

    outTensor = Math::Mat3d(
        Ixx,  0.0,  0.0,
        0.0,  Iyy,  0.0,
        0.0,  0.0,  Izz
    );
}

std::optional<ParsedAircraftDat> AircraftDatParser::ParseFromString(
    std::string_view text,
    std::string_view fallbackName)
{
    ParsedAircraftDat result;
    auto& cfg = result.config;

    cfg.name = std::string(fallbackName);
    cfg.icaoCode = "C172";
    cfg.type = Aircraft::AircraftType::Cessna172Skyhawk;
    cfg.category = Aircraft::AircraftCategory::SingleEnginePiston;

    // Scan line by line
    size_t lineStart = 0;
    f64 parsedPayload = 0.0;
    f64 parsedMaxSpeedMps = 69.44; // ~135 kt
    f64 parsedVneMps = 83.85;      // ~163 kt
    f64 parsedVlandMps = 25.72;    // ~50 kt
    f64 parsedVcrusMps = 55.56;    // ~108 kt

    Math::Vec3d rawLeftGearYS{ -1.28, -1.08, 1.25 };
    Math::Vec3d rawRightGearYS{ 1.28, -1.08, 1.25 };
    Math::Vec3d rawWheelGearYS{ 0.0, -1.19, 2.96 };

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

        if (AssetTokenizer::EqualsIgnoreCase(tag, "IDENTIFY") && tokens.size() >= 2) {
            cfg.name = tokens[1];
            if (cfg.name.size() >= 4) {
                cfg.icaoCode = cfg.name.substr(0, 4);
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CATEGORY") && tokens.size() >= 2) {
            if (AssetTokenizer::EqualsIgnoreCase(tokens[1], "AEROBATIC")) {
                cfg.category = Aircraft::AircraftCategory::Aerobatic;
                cfg.type = Aircraft::AircraftType::Extra330SC;
            } else if (AssetTokenizer::EqualsIgnoreCase(tokens[1], "HEAVY") ||
                       AssetTokenizer::EqualsIgnoreCase(tokens[1], "TRANSPORT")) {
                cfg.category = Aircraft::AircraftCategory::MultiEnginePiston;
                cfg.type = Aircraft::AircraftType::Baron58;
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "WEIGHCLN") && tokens.size() >= 2) {
            cfg.mass.emptyMassKg = AssetTokenizer::ParseValueWithUnit(tokens[1], 757.0);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "WEIGFUEL") && tokens.size() >= 2) {
            f64 fuelKg = AssetTokenizer::ParseValueWithUnit(tokens[1], 144.5);
            cfg.propulsion.maxFuelCapacityKg = fuelKg;
            cfg.mass.defaultFuelKg = fuelKg * 0.75;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "WEIGLOAD") && tokens.size() >= 2) {
            parsedPayload = AssetTokenizer::ParseValueWithUnit(tokens[1], 200.0);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "WINGAREA") && tokens.size() >= 2) {
            cfg.geometry.wingAreaM2 = AssetTokenizer::ParseValueWithUnit(tokens[1], 16.165);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "PROPELLR") && tokens.size() >= 2) {
            // Converts power (e.g. 180HP) to horsepower
            f64 powerWatts = AssetTokenizer::ParseValueWithUnit(tokens[1], 160.0 * 745.7);
            cfg.propulsion.horsepowerPerEngine = powerWatts / 745.699872;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "NREALPRP") && tokens.size() >= 2) {
            cfg.propulsion.engineCount = static_cast<u32>(std::atoi(tokens[1].c_str()));
            if (cfg.propulsion.engineCount > 1) {
                cfg.category = Aircraft::AircraftCategory::MultiEnginePiston;
                cfg.propulsion.lateralOffsetM = 1.83; // Standard twin nacelle offset
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "REALPROP") && tokens.size() >= 4) {
            const auto& propSubTag = tokens[2];
            if (AssetTokenizer::EqualsIgnoreCase(propSubTag, "MAXPOWER")) {
                f64 pWatts = AssetTokenizer::ParseValueWithUnit(tokens[3], 160.0 * 745.7);
                cfg.propulsion.horsepowerPerEngine = pWatts / 745.699872;
            }
            else if (AssetTokenizer::EqualsIgnoreCase(propSubTag, "SNDRPMRANGE") && tokens.size() >= 5) {
                cfg.propulsion.idleRpm = std::atof(tokens[3].c_str());
                cfg.propulsion.maxRatedRpm = std::atof(tokens[4].c_str());
            }
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CRITAOAP") && tokens.size() >= 2) {
            cfg.aero.stallAlphaRad = AssetTokenizer::ParseValueWithUnit(tokens[1], 0.279);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CRITAOAM") && tokens.size() >= 2) {
            cfg.aero.stallAlphaNegRad = AssetTokenizer::ParseValueWithUnit(tokens[1], -0.279);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CRITSPED") && tokens.size() >= 2) {
            parsedVneMps = AssetTokenizer::ParseValueWithUnit(tokens[1], 83.85);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "MAXSPEED") && tokens.size() >= 2) {
            parsedMaxSpeedMps = AssetTokenizer::ParseValueWithUnit(tokens[1], 69.44);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "REFVLAND") && tokens.size() >= 2) {
            parsedVlandMps = AssetTokenizer::ParseValueWithUnit(tokens[1], 25.72);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "REFVCRUS") && tokens.size() >= 2) {
            parsedVcrusMps = AssetTokenizer::ParseValueWithUnit(tokens[1], 55.56);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CLBYFLAP") && tokens.size() >= 2) {
            cfg.aero.CLdf = std::atof(tokens[1].c_str());
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CDBYFLAP") && tokens.size() >= 2) {
            cfg.aero.CDdf = std::atof(tokens[1].c_str());
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "CDBYGEAR") && tokens.size() >= 2) {
            f64 gearDeltaCd = std::atof(tokens[1].c_str());
            cfg.aero.CD0_gearDown = cfg.aero.CD0 + gearDeltaCd;
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "RETRGEAR") && tokens.size() >= 2) {
            cfg.hasRetractableGear = AssetTokenizer::ParseBool(tokens[1]);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "TIREFRIC") && tokens.size() >= 2) {
            result.tireFriction = std::atof(tokens[1].c_str());
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "INSTPANL") && tokens.size() >= 2) {
            result.cockpitPanelPath = tokens[1];
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "LEFTGEAR") && tokens.size() >= 4) {
            rawLeftGearYS = AssetTokenizer::ParseVec3(tokens, 1);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "RIGHGEAR") && tokens.size() >= 4) {
            rawRightGearYS = AssetTokenizer::ParseVec3(tokens, 1);
        }
        else if (AssetTokenizer::EqualsIgnoreCase(tag, "WHELGEAR") && tokens.size() >= 4) {
            rawWheelGearYS = AssetTokenizer::ParseVec3(tokens, 1);
        }
    }

    // Convert Landing Gear positions from YS Body (+X Right, +Y Up, +Z Forward) to FRD (+X Fwd, +Y Right, +Z Down)
    // X_FRD = +Z_YS, Y_FRD = +X_YS, Z_FRD = -Y_YS
    result.gearPositions.leftMainFRD = Math::Vec3d(rawLeftGearYS.z, rawLeftGearYS.x, -rawLeftGearYS.y);
    result.gearPositions.rightMainFRD = Math::Vec3d(rawRightGearYS.z, rawRightGearYS.x, -rawRightGearYS.y);
    result.gearPositions.noseOrTailFRD = Math::Vec3d(rawWheelGearYS.z, rawWheelGearYS.x, -rawWheelGearYS.y);

    // Detect if taildragger: if wheel is behind main gear in FRD (X_FRD < main X_FRD)
    f64 mainGearX = (result.gearPositions.leftMainFRD.x + result.gearPositions.rightMainFRD.x) * 0.5;
    result.gearPositions.isTaildragger = (result.gearPositions.noseOrTailFRD.x < mainGearX);

    // Compute Maximum Takeoff Weight
    cfg.mass.maxTakeoffMassKg = cfg.mass.emptyMassKg + cfg.propulsion.maxFuelCapacityKg + parsedPayload;

    // Wingspan estimation based on standard aspect ratio AR = 7.49 if AR is default
    f64 spanM = std::sqrt(cfg.geometry.wingAreaM2 * cfg.geometry.aspectRatio);
    cfg.geometry.wingspanM = spanM;
    cfg.geometry.meanChordM = cfg.geometry.wingAreaM2 / spanM;

    // Estimated Fuselage length
    f64 estLengthM = spanM * 0.75;

    // Estimate realistic Inertia Tensor using parallel axis & radius of gyration
    EstimateInertiaTensor(cfg.mass.emptyMassKg, cfg.geometry.wingspanM, estLengthM, cfg.mass.inertiaTensor);

    // Populate V-speeds in Knots
    constexpr f64 MpsToKnots = 1.0 / 0.514444444444444;
    cfg.vSpeeds.Vne = parsedVneMps * MpsToKnots;
    cfg.vSpeeds.Vno = parsedMaxSpeedMps * MpsToKnots;
    cfg.vSpeeds.Vso = parsedVlandMps * MpsToKnots * 0.80; // Stall dirty
    cfg.vSpeeds.Vs  = parsedVlandMps * MpsToKnots * 0.96; // Stall clean
    cfg.vSpeeds.Vr  = parsedVlandMps * MpsToKnots * 1.10; // Rotation speed
    cfg.vSpeeds.Vx  = parsedVlandMps * MpsToKnots * 1.25; // Best angle
    cfg.vSpeeds.Vy  = parsedVcrusMps * MpsToKnots * 0.70; // Best rate
    cfg.vSpeeds.Vfe = parsedVlandMps * MpsToKnots * 1.70; // Flap limit

    return result;
}

std::optional<ParsedAircraftDat> AircraftDatParser::ParseFromFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();

    size_t lastSlash = filePath.find_last_of("/\\");
    std::string name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

    return ParseFromString(text, name);
}

} // namespace LokYS::Assets
