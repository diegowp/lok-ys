#pragma once

#include "LokYS/core/Types.hpp"
#include <string>

namespace LokYS::Serial {

inline constexpr u8 SERIAL_SYNC_BYTE_1 = 0x55;
inline constexpr u8 SERIAL_SYNC_BYTE_2 = 0xAA;
inline constexpr u32 SERIAL_MAX_PAYLOAD_SIZE = 256;
inline constexpr u32 SERIAL_HEADER_SIZE = 4; // Sync1 (1) + Sync2 (1) + MsgId (1) + Length (1)
inline constexpr u32 SERIAL_CRC_SIZE = 2;    // CRC-16 (2)
inline constexpr u32 SERIAL_MAX_FRAME_SIZE = SERIAL_HEADER_SIZE + SERIAL_MAX_PAYLOAD_SIZE + SERIAL_CRC_SIZE;

enum class SerialMessageType : u8 {
    Heartbeat             = 0x01,
    TelemetryFlightData   = 0x10,
    TelemetryEngine       = 0x11,
    TelemetryNav          = 0x12,
    TelemetryAnnunciators = 0x13,
    InboundFlightControls = 0x20,
    InboundSwitches       = 0x21,
    InboundNavControls    = 0x22,
    CommandConfigure      = 0xF0
};

#pragma pack(push, 1)

struct HeartbeatPayload {
    u32 timestampMs{ 0 };
    u8 echo{ 0 };
};

struct TelemetryFlightDataPayload {
    f32 indicatedAirspeedKnots{ 0.0f };
    f32 pressureAltitudeFeet{ 0.0f };
    f32 pitchDeg{ 0.0f };
    f32 rollDeg{ 0.0f };
    f32 headingDeg{ 0.0f };
    f32 verticalSpeedFpm{ 0.0f };
    f32 turnRateDps{ 0.0f };
    f32 slipSkidBall{ 0.0f };
};

struct TelemetryEnginePayload {
    f32 engineRpm{ 0.0f };
    f32 manifoldPressureInHg{ 0.0f };
    f32 fuelFlowGph{ 0.0f };
    f32 remainingFuelKg{ 0.0f };
    u8 magnetosState{ 3 }; // 0=Off, 1=R, 2=L, 3=Both
    u8 starterActive{ 0 };
};

struct TelemetryNavPayload {
    f32 nav1ActiveFreqMhz{ 114.50f };
    f32 nav1StandbyFreqMhz{ 109.50f };
    f32 nav1ObsDeg{ 360.0f };
    f32 cdiDeviationDots{ 0.0f };
    f32 gsDeviationDots{ 0.0f };
    f32 dmeDistanceNm{ 0.0f };
    f32 dmeSpeedKnots{ 0.0f };
    u8 flags{ 0 };       // bit 0: TO, bit 1: FROM, bit 2: NAV flag, bit 3: GS flag, bit 4: DME active
    u8 markerFlags{ 0 }; // bit 0: OM, bit 1: MM, bit 2: IM
};

struct TelemetryAnnunciatorsPayload {
    u8 gearNose{ 0 };
    u8 gearLeft{ 0 };
    u8 gearRight{ 0 };
    u8 brakesActive{ 0 };
    u8 stallWarning{ 0 };
    u8 lowFuel{ 0 };
    u8 vacuumFail{ 0 };
    u8 alternatorFail{ 0 };
};

struct InboundFlightControlsPayload {
    i16 pitchAxis{ 0 };     // -32767 .. 32767 -> -1.0 .. +1.0
    i16 rollAxis{ 0 };      // -32767 .. 32767 -> -1.0 .. +1.0
    i16 yawAxis{ 0 };       // -32767 .. 32767 -> -1.0 .. +1.0
    u16 throttleAxis{ 0 };  // 0 .. 65535      ->  0.0 ..  1.0
    u16 mixtureAxis{ 0 };   // 0 .. 65535      ->  0.0 ..  1.0
    u16 leftBrake{ 0 };     // 0 .. 65535      ->  0.0 ..  1.0
    u16 rightBrake{ 0 };    // 0 .. 65535      ->  0.0 ..  1.0
    u8 flapSetting{ 0 };    // 0, 1, 2, 3
    u8 validAxesMask{ 0 };  // Bit 0: Pitch, 1: Roll, 2: Yaw, 3: Throttle, 4: Mixture, 5: Brakes, 6: Flaps
};

struct InboundSwitchesPayload {
    u32 switchBitmask{ 0 }; // bit 0: Master Bat, 1: Alt, 2: Avionics, 3: Fuel Pump, 4: Pitot Heat, etc.
};

struct InboundNavControlsPayload {
    u8 navSwapTriggered{ 0 };
    u8 navIdentTriggered{ 0 };
    i8 obsDeltaTicks{ 0 };
    i8 freqDeltaTicks{ 0 };
};

struct CommandConfigurePayload {
    u16 telemetryRateHz{ 30 };
    u8 enableFlightData{ 1 };
    u8 enableEngine{ 1 };
    u8 enableNav{ 1 };
};

#pragma pack(pop)

struct SerialConfig {
    std::wstring portName{ L"COM3" };
    u32 baudRate{ 115200 };
    u32 telemetryRateHz{ 30 };
    bool mockLoopback{ false };
};

struct SerialStats {
    u64 bytesSent{ 0 };
    u64 bytesReceived{ 0 };
    u64 packetsSent{ 0 };
    u64 packetsReceived{ 0 };
    u64 crcErrors{ 0 };
    u64 framingErrors{ 0 };
};

} // namespace LokYS::Serial
