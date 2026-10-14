#include "LokYS/serial/SerialTypes.hpp"
#include "LokYS/serial/SerialProtocol.hpp"
#include "LokYS/serial/SerialPort.hpp"
#include "LokYS/serial/SerialInterfaceSystem.hpp"
#include "LokYS/input/InputSystem.hpp"
#include <iostream>
#include <cmath>
#include <vector>

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

namespace {

using namespace LokYS;
using namespace LokYS::Serial;

bool TestCrc16CalculationAndVerification() {
    std::cout << "[RUNNING] TestCrc16CalculationAndVerification...\n";

    // Standard ASCII "123456789" test vector
    const u8 testData[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    u16 crc = CalculateCrc16(testData, sizeof(testData));
    TEST_ASSERT(crc == 0x29B1, "CRC-16-CCITT of '123456789' must equal 0x29B1");
    TEST_ASSERT(VerifyCrc16(testData, sizeof(testData), 0x29B1), "VerifyCrc16 must succeed with expected CRC");

    // Single bit corruption must cause verification failure
    u8 corruptedData[sizeof(testData)];
    std::memcpy(corruptedData, testData, sizeof(testData));
    corruptedData[4] ^= 0x01; // flip 1 bit
    TEST_ASSERT(!VerifyCrc16(corruptedData, sizeof(corruptedData), 0x29B1), "Corrupted data must fail CRC check");

    std::cout << "[PASSED] TestCrc16CalculationAndVerification\n";
    return true;
}

bool TestPacketEncodingAndFraming() {
    std::cout << "[RUNNING] TestPacketEncodingAndFraming...\n";

    TelemetryFlightDataPayload flight{};
    flight.indicatedAirspeedKnots = 112.5f;
    flight.pressureAltitudeFeet = 3500.0f;
    flight.pitchDeg = 4.2f;
    flight.rollDeg = -12.0f;
    flight.headingDeg = 270.0f;
    flight.verticalSpeedFpm = 500.0f;

    u8 buffer[SERIAL_MAX_FRAME_SIZE];
    size_t encodedLen = EncodePacket(
        SerialMessageType::TelemetryFlightData,
        &flight,
        sizeof(flight),
        buffer,
        sizeof(buffer));

    TEST_ASSERT(encodedLen == SERIAL_HEADER_SIZE + sizeof(flight) + SERIAL_CRC_SIZE, "Encoded length must match frame specification");
    TEST_ASSERT(buffer[0] == SERIAL_SYNC_BYTE_1, "Sync byte 1 must be 0x55");
    TEST_ASSERT(buffer[1] == SERIAL_SYNC_BYTE_2, "Sync byte 2 must be 0xAA");
    TEST_ASSERT(buffer[2] == static_cast<u8>(SerialMessageType::TelemetryFlightData), "MsgId must match TelemetryFlightData");
    TEST_ASSERT(buffer[3] == sizeof(flight), "Payload length must match sizeof(TelemetryFlightDataPayload)");

    // Validate payload bytes
    TelemetryFlightDataPayload decodedPayload{};
    std::memcpy(&decodedPayload, &buffer[SERIAL_HEADER_SIZE], sizeof(decodedPayload));
    TEST_ASSERT(std::abs(decodedPayload.indicatedAirspeedKnots - 112.5f) < 0.001f, "Decoded airspeed must match");
    TEST_ASSERT(std::abs(decodedPayload.pressureAltitudeFeet - 3500.0f) < 0.001f, "Decoded altitude must match");

    // Validate encoded CRC bytes
    u16 expectedCrc = CalculateCrc16(&buffer[2], 2 + sizeof(flight));
    u16 packetCrc = (static_cast<u16>(buffer[SERIAL_HEADER_SIZE + sizeof(flight)]) << 8) |
                     static_cast<u16>(buffer[SERIAL_HEADER_SIZE + sizeof(flight) + 1]);
    TEST_ASSERT(packetCrc == expectedCrc, "Frame CRC bytes must match computed CRC");

    std::cout << "[PASSED] TestPacketEncodingAndFraming\n";
    return true;
}

bool TestPacketDecodingStream() {
    std::cout << "[RUNNING] TestPacketDecodingStream...\n";

    SerialPacketDecoder decoder;
    std::vector<DecodedPacket> received;

    HeartbeatPayload hb1{ 1000, 1 };
    HeartbeatPayload hb2{ 2000, 2 };

    u8 buf1[SERIAL_MAX_FRAME_SIZE];
    u8 buf2[SERIAL_MAX_FRAME_SIZE];
    size_t len1 = EncodePacket(SerialMessageType::Heartbeat, &hb1, sizeof(hb1), buf1, sizeof(buf1));
    size_t len2 = EncodePacket(SerialMessageType::Heartbeat, &hb2, sizeof(hb2), buf2, sizeof(buf2));

    // Combine packets into single stream
    std::vector<u8> stream;
    stream.insert(stream.end(), buf1, buf1 + len1);
    stream.insert(stream.end(), buf2, buf2 + len2);

    // Feed in fragmented chunks of 3 bytes
    for (size_t i = 0; i < stream.size(); i += 3) {
        size_t chunk = std::min<size_t>(3, stream.size() - i);
        decoder.ProcessBytes(&stream[i], chunk, [&](const DecodedPacket& pkt) {
            received.push_back(pkt);
        });
    }

    TEST_ASSERT(received.size() == 2, "Decoder must extract both packets from fragmented stream");
    TEST_ASSERT(received[0].type == SerialMessageType::Heartbeat, "Packet 1 type must be Heartbeat");
    TEST_ASSERT(received[1].type == SerialMessageType::Heartbeat, "Packet 2 type must be Heartbeat");

    HeartbeatPayload outHb1{};
    HeartbeatPayload outHb2{};
    TEST_ASSERT(received[0].As(outHb1), "Packet 1 cast must succeed");
    TEST_ASSERT(received[1].As(outHb2), "Packet 2 cast must succeed");
    TEST_ASSERT(outHb1.timestampMs == 1000, "Packet 1 timestamp must match");
    TEST_ASSERT(outHb2.timestampMs == 2000, "Packet 2 timestamp must match");

    std::cout << "[PASSED] TestPacketDecodingStream\n";
    return true;
}

bool TestNoiseRejectionAndRecovery() {
    std::cout << "[RUNNING] TestNoiseRejectionAndRecovery...\n";

    SerialPacketDecoder decoder;
    std::vector<DecodedPacket> received;

    HeartbeatPayload hb{ 54321, 0 };
    u8 validBuf[SERIAL_MAX_FRAME_SIZE];
    size_t validLen = EncodePacket(SerialMessageType::Heartbeat, &hb, sizeof(hb), validBuf, sizeof(validBuf));

    // Create stream with leading noise, then corrupted packet, then valid packet
    std::vector<u8> noisyStream;

    // 1. Random noise bytes
    noisyStream.push_back(0xFF);
    noisyStream.push_back(0x55); // partial sync
    noisyStream.push_back(0x00); // false sync
    noisyStream.push_back(0xAA);
    noisyStream.push_back(0x12);

    // 2. Corrupted packet (flip bit in payload)
    std::vector<u8> corrupted(validBuf, validBuf + validLen);
    corrupted[SERIAL_HEADER_SIZE + 1] ^= 0x80;
    noisyStream.insert(noisyStream.end(), corrupted.begin(), corrupted.end());

    // 3. Trailing valid packet
    noisyStream.insert(noisyStream.end(), validBuf, validBuf + validLen);

    decoder.ProcessBytes(noisyStream.data(), noisyStream.size(), [&](const DecodedPacket& pkt) {
        received.push_back(pkt);
    });

    TEST_ASSERT(received.size() == 1, "Only the uncorrupted valid packet must be accepted");
    TEST_ASSERT(decoder.GetStats().crcErrors == 1, "Decoder must record exactly 1 CRC error");
    TEST_ASSERT(decoder.GetStats().framingErrors > 0, "Decoder must record framing errors from leading noise");

    HeartbeatPayload outHb{};
    TEST_ASSERT(received[0].As(outHb), "Payload cast must succeed");
    TEST_ASSERT(outHb.timestampMs == 54321, "Decoded payload must match valid packet");

    std::cout << "[PASSED] TestNoiseRejectionAndRecovery\n";
    return true;
}

bool TestVirtualSerialPortMockLoopback() {
    std::cout << "[RUNNING] TestVirtualSerialPortMockLoopback...\n";

    SerialPort port;
    port.SetMockLoopback(true);
    TEST_ASSERT(port.Open(L"MOCK", 115200), "Opening mock port must succeed");
    TEST_ASSERT(port.IsOpen(), "Port must report as open in mock mode");

    TelemetryEnginePayload engine{};
    engine.engineRpm = 2450.0f;
    engine.manifoldPressureInHg = 24.5f;
    engine.fuelFlowGph = 8.5f;

    u8 txFrame[SERIAL_MAX_FRAME_SIZE];
    size_t txLen = EncodePacket(SerialMessageType::TelemetryEngine, &engine, sizeof(engine), txFrame, sizeof(txFrame));

    size_t written = port.Write(txFrame, txLen);
    TEST_ASSERT(written == txLen, "All bytes must be written to mock port");

    u8 rxBuffer[SERIAL_MAX_FRAME_SIZE];
    size_t readBytes = port.Read(rxBuffer, sizeof(rxBuffer));
    TEST_ASSERT(readBytes == txLen, "Mock port loopback must return written bytes");

    SerialPacketDecoder decoder;
    bool packetDecoded = false;
    bool packetValid = true;
    decoder.ProcessBytes(rxBuffer, readBytes, [&](const DecodedPacket& pkt) {
        if (pkt.type != SerialMessageType::TelemetryEngine) {
            packetValid = false;
            return;
        }
        TelemetryEnginePayload rxEngine{};
        if (!pkt.As(rxEngine)) {
            packetValid = false;
            return;
        }
        if (std::abs(rxEngine.engineRpm - 2450.0f) >= 0.1f) {
            packetValid = false;
            return;
        }
        if (std::abs(rxEngine.manifoldPressureInHg - 24.5f) >= 0.1f) {
            packetValid = false;
            return;
        }
        packetDecoded = true;
    });

    TEST_ASSERT(packetValid, "Packet content must be valid");
    TEST_ASSERT(packetDecoded, "Packet must be parsed through mock serial loopback");
    port.Close();
    TEST_ASSERT(!port.IsOpen() || port.IsMockLoopback(), "Port must close cleanly");

    std::cout << "[PASSED] TestVirtualSerialPortMockLoopback\n";
    return true;
}

bool TestSerialInterfaceSystemEndToEnd() {
    std::cout << "[RUNNING] TestSerialInterfaceSystemEndToEnd...\n";

    SerialConfig config{};
    config.portName = L"VIRTUAL";
    config.mockLoopback = true;

    SerialInterfaceSystem serialSys;
    TEST_ASSERT(serialSys.Initialize(config), "SerialInterfaceSystem must initialize in mock loopback mode");
    TEST_ASSERT(serialSys.IsConnected(), "SerialInterfaceSystem must report connected in mock mode");

    // Transmit telemetry
    TelemetryNavPayload navOut{};
    navOut.nav1ActiveFreqMhz = 114.50f;
    navOut.nav1ObsDeg = 360.0f;
    navOut.cdiDeviationDots = 1.5f;
    navOut.flags = 0x01; // TO flag

    TEST_ASSERT(serialSys.SendNav(navOut), "SendNav must succeed");

    // Inject simulated microcontroller control packet into mock port
    InboundFlightControlsPayload hwControls{};
    hwControls.pitchAxis = 16384;   // +0.5 nose up
    hwControls.rollAxis = -16384;   // -0.5 left roll
    hwControls.throttleAxis = 49151;// 0.75 throttle
    hwControls.flapSetting = 2;     // 20 deg flaps
    hwControls.validAxesMask = 0x7F;// All axes valid

    u8 hwFrame[SERIAL_MAX_FRAME_SIZE];
    size_t hwLen = EncodePacket(
        SerialMessageType::InboundFlightControls,
        &hwControls,
        sizeof(hwControls),
        hwFrame,
        sizeof(hwFrame));

    serialSys.GetPort().InjectMockRxData(hwFrame, hwLen);
    serialSys.ProcessSynchronous();

    TEST_ASSERT(serialSys.HasHardwareControls(), "System must report presence of hardware controls");
    InboundFlightControlsPayload receivedHw = serialSys.GetLatestFlightControls();
    TEST_ASSERT(receivedHw.pitchAxis == 16384, "Pitch axis must match injected value");
    TEST_ASSERT(receivedHw.rollAxis == -16384, "Roll axis must match injected value");
    TEST_ASSERT(receivedHw.throttleAxis == 49151, "Throttle axis must match injected value");
    TEST_ASSERT(receivedHw.flapSetting == 2, "Flap setting must match injected value");

    serialSys.Shutdown();
    std::cout << "[PASSED] TestSerialInterfaceSystemEndToEnd\n";
    return true;
}

bool TestInputSystemHardwareControlMerging() {
    std::cout << "[RUNNING] TestInputSystemHardwareControlMerging...\n";

    Input::InputSystem input;

    InboundFlightControlsPayload hw{};
    hw.pitchAxis = 16384;   // 16384 / 32767.0 ~= +0.50
    hw.rollAxis = -16384;   // -16384 / 32767.0 ~= -0.50
    hw.yawAxis = 8192;      // 8192 / 32767.0 ~= +0.25
    hw.throttleAxis = 52428;// 52428 / 65535.0 ~= 0.80
    hw.mixtureAxis = 65535; // 1.0
    hw.leftBrake = 32768;   // 0.5
    hw.rightBrake = 0;
    hw.flapSetting = 1;     // Flaps 10 deg
    hw.validAxesMask = 0x7F;

    InboundNavControlsPayload nav{};
    nav.navSwapTriggered = 1;
    nav.obsDeltaTicks = 5; // +5 deg
    nav.freqDeltaTicks = 2; // +100 kHz

    input.ApplySerialControls(hw, nav);
    const auto& ctrl = input.GetControls();

    TEST_ASSERT(std::abs(ctrl.pitch - 0.50) < 0.01, "Pitch must scale to +0.50");
    TEST_ASSERT(std::abs(ctrl.roll - (-0.50)) < 0.01, "Roll must scale to -0.50");
    TEST_ASSERT(std::abs(ctrl.yaw - 0.25) < 0.01, "Yaw must scale to +0.25");
    TEST_ASSERT(std::abs(ctrl.throttle - 0.80) < 0.01, "Throttle must scale to 0.80");
    TEST_ASSERT(std::abs(ctrl.mixture - 1.0) < 0.01, "Mixture must scale to 1.0");
    TEST_ASSERT(std::abs(ctrl.brakes - 0.50) < 0.01, "Brakes must take max of L/R (0.50)");
    TEST_ASSERT(ctrl.flapSetting == 1, "Flap setting must be 1");
    TEST_ASSERT(input.WasNavSwapTriggered(), "Nav swap must be triggered");
    TEST_ASSERT(std::abs(input.GetObsDelta() - 5.0) < 0.01, "OBS delta must be +5 deg");
    TEST_ASSERT(std::abs(input.GetNavFrequencyDelta() - 0.10) < 0.01, "Nav freq delta must be +0.10 MHz (2 * 50 kHz)");

    std::cout << "[PASSED] TestInputSystemHardwareControlMerging\n";
    return true;
}

} // namespace

int main() {
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;
    std::cout << "============================================================\n";
    std::cout << "LokYS Native Windows Flight Simulator — Serial Hardware Tests\n";
    std::cout << "============================================================\n";

    bool allPassed = true;
    allPassed &= TestCrc16CalculationAndVerification();
    allPassed &= TestPacketEncodingAndFraming();
    allPassed &= TestPacketDecodingStream();
    allPassed &= TestNoiseRejectionAndRecovery();
    allPassed &= TestVirtualSerialPortMockLoopback();
    allPassed &= TestSerialInterfaceSystemEndToEnd();
    allPassed &= TestInputSystemHardwareControlMerging();

    std::cout << "============================================================\n";
    if (allPassed) {
        std::cout << "[ALL 7 SERIAL HARDWARE TEST SUITES PASSED (100%)]\n";
        return 0;
    } else {
        std::cout << "[ONE OR MORE SERIAL HARDWARE TESTS FAILED]\n";
        return 1;
    }
}
