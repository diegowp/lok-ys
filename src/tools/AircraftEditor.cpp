#include "LokYS/tools/AircraftEditor.hpp"
#include "LokYS/assets/AssetTokenizer.hpp"
#include "LokYS/math/Units.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace LokYS::Tools {

namespace {

constexpr f64 PI_VAL = 3.14159265358979323846;
constexpr f64 DEG_TO_RAD_VAL = PI_VAL / 180.0;
constexpr f64 RAD_TO_DEG_VAL = 180.0 / PI_VAL;

std::string ReadEntireFileText(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return "";
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

} // namespace

bool AircraftEditor::LoadFromFiles(
    const std::string& datPath,
    const std::string& dnmPath,
    const std::string& istPath)
{
    std::string datText = ReadEntireFileText(datPath);
    if (datText.empty()) {
        return false;
    }

    std::string dnmText = !dnmPath.empty() ? ReadEntireFileText(dnmPath) : "";
    std::string istText = !istPath.empty() ? ReadEntireFileText(istPath) : "";

    return LoadFromMemory(datText, dnmText, istText, datPath);
}

bool AircraftEditor::LoadFromMemory(
    std::string_view datText,
    std::string_view dnmText,
    std::string_view istText,
    const std::string& fallbackName)
{
    auto parsedDat = Assets::AircraftDatParser::ParseFromString(datText, fallbackName);
    if (!parsedDat) {
        return false;
    }

    m_acftConfig = parsedDat->config;
    m_gearPositions = parsedDat->gearPositions;
    m_animationStates.clear();

    if (!dnmText.empty()) {
        m_rawDnmContent = std::string(dnmText);
        auto parsedDnm = Assets::DnmParser::ParseFromString(dnmText, m_acftConfig.name);
        if (parsedDnm) {
            m_dnmModel = std::move(*parsedDnm);
        }
    } else {
        m_rawDnmContent.clear();
        m_dnmModel = {};
    }

    if (!istText.empty()) {
        m_rawIstContent = std::string(istText);
        m_cockpitPanel = Assets::CockpitIstParser::ParseFromString(istText);
    } else {
        m_rawIstContent.clear();
        m_cockpitPanel.reset();
    }

    return true;
}

bool AircraftEditor::LoadFromPackageBuffer(std::span<const uint8_t> zipBuffer) {
    Vfs::ZipVfsReader vfs;
    if (!vfs.LoadFromMemory(zipBuffer)) {
        return false;
    }

    Assets::AcftPackage pkg;
    if (!pkg.LoadFromArchive(vfs)) {
        return false;
    }

    m_acftConfig = pkg.GetFlightData().config;
    m_gearPositions = pkg.GetFlightData().gearPositions;
    m_dnmModel = pkg.GetVisualModel();
    m_cockpitPanel = pkg.GetCockpitPanel();
    m_animationStates.clear();

    // Cache raw content from archive if present
    const auto& manifest = pkg.GetManifest();
    if (!manifest.visualModelFile.empty()) {
        auto text = vfs.ReadText(manifest.visualModelFile);
        if (text) m_rawDnmContent = std::move(*text);
    }
    if (!manifest.instrumentPanelFile.empty()) {
        auto text = vfs.ReadText(manifest.instrumentPanelFile);
        if (text) m_rawIstContent = std::move(*text);
    }

    return true;
}

bool AircraftEditor::LoadFromPackageFile(const std::string& zipFilePath) {
    std::ifstream file(zipFilePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        return false;
    }

    return LoadFromPackageBuffer(buffer);
}

size_t AircraftEditor::GetNodeCount() const noexcept {
    return m_dnmModel.nodes.size();
}

const Assets::DnmNode* AircraftEditor::GetNode(size_t index) const noexcept {
    if (index >= m_dnmModel.nodes.size()) {
        return nullptr;
    }
    return &m_dnmModel.nodes[index];
}

int AircraftEditor::FindNodeByName(std::string_view name) const noexcept {
    for (size_t i = 0; i < m_dnmModel.nodes.size(); ++i) {
        if (m_dnmModel.nodes[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::string AircraftEditor::GenerateHierarchyTree() const {
    if (m_dnmModel.nodes.empty()) {
        return "[Empty Scene Graph: No DNM Nodes Loaded]\n";
    }

    std::stringstream ss;
    ss << "=== DNM HIERARCHY TREE: " << m_acftConfig.name << " ===\n";
    ss << "Total Nodes: " << m_dnmModel.nodes.size() 
       << ", Vertices: " << m_dnmModel.GetTotalVertices() 
       << ", Indices: " << m_dnmModel.GetTotalIndices() << "\n\n";

    for (size_t i = 0; i < m_dnmModel.nodes.size(); ++i) {
        const auto& node = m_dnmModel.nodes[i];
        int parentIdx = FindParentNodeIndex(i);

        ss << "[" << std::setw(2) << i << "] Node: \"" << node.name << "\"";
        if (parentIdx >= 0) {
            ss << " (Child of [" << parentIdx << "] \"" << m_dnmModel.nodes[parentIdx].name << "\")";
        } else {
            ss << " (Root Node)";
        }

        ss << "\n     CLA=" << node.animationClass 
           << " | Pos=(" << node.position.x << ", " << node.position.y << ", " << node.position.z << ")"
           << " | Verts=" << node.vertices.size() << " Tris=" << (node.indices.size() / 3) << "\n";

        if (!node.childrenNames.empty()) {
            ss << "     Children: ";
            for (size_t c = 0; c < node.childrenNames.size(); ++c) {
                ss << "\"" << node.childrenNames[c] << "\"" << (c + 1 < node.childrenNames.size() ? ", " : "");
            }
            ss << "\n";
        }
    }

    return ss.str();
}

int AircraftEditor::FindParentNodeIndex(size_t childIndex) const noexcept {
    if (childIndex >= m_dnmModel.nodes.size()) return -1;
    const std::string& childName = m_dnmModel.nodes[childIndex].name;

    for (size_t i = 0; i < m_dnmModel.nodes.size(); ++i) {
        for (const auto& cName : m_dnmModel.nodes[i].childrenNames) {
            if (cName == childName) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

void AircraftEditor::SetAnimationChannel(int animClass, f32 stateNormalized) noexcept {
    f32 clamped = std::clamp(stateNormalized, 0.0f, 1.0f);
    m_animationStates[animClass] = clamped;
}

f32 AircraftEditor::GetAnimationChannel(int animClass) const noexcept {
    auto it = m_animationStates.find(animClass);
    return (it != m_animationStates.end()) ? it->second : 0.0f;
}

void AircraftEditor::ResetAllAnimations() noexcept {
    m_animationStates.clear();
}

Math::Mat4f AircraftEditor::ComputeNodeLocalTransform(size_t nodeIndex) const noexcept {
    if (nodeIndex >= m_dnmModel.nodes.size()) {
        return Math::Mat4f::Identity();
    }

    const auto& node = m_dnmModel.nodes[nodeIndex];
    Math::Mat4f translation = Math::Mat4f::Translation(node.position);

    if (node.animationClass == 0) {
        return translation;
    }

    f32 t = GetAnimationChannel(node.animationClass);
    f32 angleRad = 0.0f;

    switch (node.animationClass) {
    case 5: // Wing Flaps (0 deg to 35 deg down)
        angleRad = t * static_cast<f32>(35.0 * DEG_TO_RAD_VAL);
        break;
    case 6: // Elevator (-25 deg to +20 deg)
        angleRad = (t * 2.0f - 1.0f) * static_cast<f32>(20.0 * DEG_TO_RAD_VAL);
        break;
    case 7: // Aileron (-18 deg to +18 deg)
        angleRad = (t * 2.0f - 1.0f) * static_cast<f32>(18.0 * DEG_TO_RAD_VAL);
        break;
    case 8: // Rudder (-25 deg to +25 deg)
        angleRad = (t * 2.0f - 1.0f) * static_cast<f32>(25.0 * DEG_TO_RAD_VAL);
        break;
    case 24: // Propeller (continuous rotation)
        angleRad = t * static_cast<f32>(2.0 * PI_VAL);
        break;
    case 1:  // Landing gear transit (0 = retracted 90 deg, 1 = extended 0 deg)
    case 2:
        angleRad = (1.0f - t) * static_cast<f32>(90.0 * DEG_TO_RAD_VAL);
        break;
    default:
        angleRad = t * static_cast<f32>(30.0 * DEG_TO_RAD_VAL);
        break;
    }

    Math::Vec3f axis = node.pivotAxis;
    if (axis.LengthSq() < 1e-6f) {
        axis = Math::Vec3f(1.0f, 0.0f, 0.0f);
    }
    axis = axis.Normalized();

    Math::Quat4f rotQ = Math::Quat4f::FromAxisAngle(axis, angleRad);
    Math::Mat4f rotMat = Math::Mat4f::FromQuaternion(rotQ);

    Math::Mat4f toPivot = Math::Mat4f::Translation(-node.pivotCenter);
    Math::Mat4f fromPivot = Math::Mat4f::Translation(node.pivotCenter);

    // Composite: Translation * fromPivot * rotMat * toPivot
    return translation * fromPivot * rotMat * toPivot;
}

Math::Mat4f AircraftEditor::ComputeNodeWorldTransform(size_t nodeIndex) const noexcept {
    if (nodeIndex >= m_dnmModel.nodes.size()) {
        return Math::Mat4f::Identity();
    }

    Math::Mat4f local = ComputeNodeLocalTransform(nodeIndex);
    int parentIdx = FindParentNodeIndex(nodeIndex);

    if (parentIdx >= 0) {
        Math::Mat4f parentWorld = ComputeNodeWorldTransform(static_cast<size_t>(parentIdx));
        return parentWorld * local;
    }

    return local;
}

std::vector<AeroPolarSample> AircraftEditor::ComputeAeroPolars(
    f64 minAlphaDeg,
    f64 maxAlphaDeg,
    f64 stepDeg) const
{
    std::vector<AeroPolarSample> samples;
    const auto& aero = m_acftConfig.aero;
    const auto& geom = m_acftConfig.geometry;

    f64 step = (stepDeg > 0.01) ? stepDeg : 1.0;
    f64 inducedFactor = 1.0 / (PI_VAL * geom.oswaldEfficiency * geom.aspectRatio);

    for (f64 alpha = minAlphaDeg; alpha <= maxAlphaDeg + 1e-4; alpha += step) {
        f64 alphaRad = alpha * DEG_TO_RAD_VAL;
        f64 cl = 0.0;

        if (std::abs(alphaRad) <= aero.stallAlphaRad) {
            cl = aero.CL0 + aero.CLa * alphaRad;
        } else {
            // Post-stall linear drop-off
            f64 stallCl = aero.CL0 + aero.CLa * std::copysign(aero.stallAlphaRad, alphaRad);
            f64 excess = std::abs(alphaRad) - aero.stallAlphaRad;
            cl = stallCl - std::copysign(excess * 1.5, alphaRad);
        }

        f64 cd = aero.CD0 + inducedFactor * (cl * cl);
        f64 cm = aero.Cm0 + aero.Cma * alphaRad;

        samples.push_back(AeroPolarSample{ alpha, cl, cd, cm });
    }

    return samples;
}

void AircraftEditor::SetEngineHorsepower(f64 hp) noexcept {
    m_acftConfig.propulsion.horsepowerPerEngine = std::max(hp, 10.0);
}

void AircraftEditor::SetCruiseSpeedKnots(f64 knots) noexcept {
    m_acftConfig.vSpeeds.Vno = std::max(knots, 30.0);
}

void AircraftEditor::SetEmptyMassKg(f64 massKg) noexcept {
    m_acftConfig.mass.emptyMassKg = std::max(massKg, 50.0);
}

void AircraftEditor::SetWingAreaM2(f64 areaM2) noexcept {
    if (areaM2 > 1.0) {
        m_acftConfig.geometry.wingAreaM2 = areaM2;
        m_acftConfig.geometry.aspectRatio = (m_acftConfig.geometry.wingspanM * m_acftConfig.geometry.wingspanM) / areaM2;
    }
}

void AircraftEditor::SetWingspanM(f64 spanM) noexcept {
    if (spanM > 1.0) {
        m_acftConfig.geometry.wingspanM = spanM;
        m_acftConfig.geometry.aspectRatio = (spanM * spanM) / m_acftConfig.geometry.wingAreaM2;
    }
}

void AircraftEditor::SetStallAlphaDeg(f64 stallDeg) noexcept {
    f64 rad = std::clamp(stallDeg, 5.0, 35.0) * DEG_TO_RAD_VAL;
    m_acftConfig.aero.stallAlphaRad = rad;
    m_acftConfig.aero.stallAlphaNegRad = -rad;
}

void AircraftEditor::SetPitchStiffnessCma(f64 cma) noexcept {
    m_acftConfig.aero.Cma = cma;
}

void AircraftEditor::SetAirspeedLimitsKnots(f32 vfe, f32 vno, f32 vne) noexcept {
    m_acftConfig.vSpeeds.Vfe = vfe;
    m_acftConfig.vSpeeds.Vno = vno;
    m_acftConfig.vSpeeds.Vne = vne;

    if (m_cockpitPanel) {
        m_cockpitPanel->speedVfeKnots = vfe;
        m_cockpitPanel->speedVnoKnots = vno;
        m_cockpitPanel->speedVneKnots = vne;
    }
}

AircraftEditorSummary AircraftEditor::GetSummary() const noexcept {
    AircraftEditorSummary s;
    s.name = m_acftConfig.name;
    s.icao = m_acftConfig.icaoCode;
    s.emptyMassKg = m_acftConfig.mass.emptyMassKg;
    s.maxThrustN = m_acftConfig.propulsion.horsepowerPerEngine * 745.7 / std::max(m_acftConfig.vSpeeds.Vy * 0.514444, 20.0);
    s.wingAreaM2 = m_acftConfig.geometry.wingAreaM2;
    s.wingspanM = m_acftConfig.geometry.wingspanM;
    s.stallSpeedKts = m_acftConfig.vSpeeds.Vs;
    s.cruiseSpeedKts = m_acftConfig.vSpeeds.Vno;
    s.dnmNodeCount = m_dnmModel.nodes.size();
    s.totalVertices = m_dnmModel.GetTotalVertices();
    s.totalIndices = m_dnmModel.GetTotalIndices();
    s.isLongitudinallyStable = ValidateLongitudinalStability();
    return s;
}

std::string AircraftEditor::GenerateDatText() const {
    std::stringstream ss;
    ss << "# LokYS Flight Model Parameter File (Exported by AircraftEditor)\n";
    ss << "IDENTIFY \"" << m_acftConfig.name << "\"\n";
    ss << "WEIGHCLN " << static_cast<int>(m_acftConfig.mass.emptyMassKg) << "kg\n";
    ss << "WEIGFUEL " << static_cast<int>(m_acftConfig.propulsion.maxFuelCapacityKg) << "kg\n";
    ss << "WINGAREA " << std::fixed << std::setprecision(2) << m_acftConfig.geometry.wingAreaM2 << "m^2\n";

    if (m_acftConfig.propulsion.horsepowerPerEngine > 0.0) {
        ss << "PROPELLR " << static_cast<int>(m_acftConfig.propulsion.horsepowerPerEngine) << "HP\n";
    }

    ss << "REFVCRUS " << static_cast<int>(m_acftConfig.vSpeeds.Vno) << "kt\n";
    ss << "REFVLAND " << static_cast<int>(m_acftConfig.vSpeeds.Vs) << "kt\n";
    ss << "CRITSPED " << static_cast<int>(m_acftConfig.vSpeeds.Vne) << "kt\n";
    ss << "CRITAOAP " << std::fixed << std::setprecision(2) << m_acftConfig.aero.stallAlphaRad * RAD_TO_DEG_VAL << "deg\n";
    ss << "CRITAOAM " << std::fixed << std::setprecision(2) << m_acftConfig.aero.stallAlphaNegRad * RAD_TO_DEG_VAL << "deg\n";

    if (m_acftConfig.hasRetractableGear) {
        ss << "RETRGEAR TRUE\n";
    }

    if (m_cockpitPanel) {
        ss << "INSTPANL cockpit.ist\n";
    }

    return ss.str();
}

std::vector<uint8_t> AircraftEditor::ExportToAcftPackage() const {
    Vfs::ZipWriter writer;

    // 1. Generate Manifest
    std::stringstream manifest;
    manifest << "{\n"
             << "  \"formatVersion\": 1,\n"
             << "  \"type\": \"Aircraft\",\n"
             << "  \"identifier\": \"" << m_acftConfig.name << "\",\n"
             << "  \"displayName\": \"" << m_acftConfig.name << "\",\n"
             << "  \"author\": \"LokYS Aircraft Editor\",\n"
             << "  \"flightModel\": \"aircraft.dat\"";

    if (!m_rawDnmContent.empty()) {
        manifest << ",\n  \"visualModel\": \"visual.dnm\"";
    }
    if (m_cockpitPanel && !m_rawIstContent.empty()) {
        manifest << ",\n  \"instrumentPanel\": \"cockpit.ist\"";
    }
    manifest << "\n}\n";

    writer.AddText("manifest.json", manifest.str());

    // 2. Flight model .dat
    writer.AddText("aircraft.dat", GenerateDatText());

    // 3. Visual model .dnm
    if (!m_rawDnmContent.empty()) {
        writer.AddText("visual.dnm", m_rawDnmContent);
    }

    // 4. Cockpit panel .ist
    if (m_cockpitPanel && !m_rawIstContent.empty()) {
        writer.AddText("cockpit.ist", m_rawIstContent);
    }

    return writer.FinalizeToMemory();
}

bool AircraftEditor::ExportToAcftFile(const std::string& outZipPath) const {
    auto data = ExportToAcftPackage();
    std::ofstream out(outZipPath, std::ios::binary);
    if (!out.is_open()) return false;
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return out.good();
}

} // namespace LokYS::Tools
