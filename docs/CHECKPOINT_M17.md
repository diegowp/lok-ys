# LokYS Flight Simulator — Project Checkpoint (Milestone M17)

**Timestamp:** 2026-10-14T15:02:00-03:00  
**Current Milestone:** M17 — Serial Cockpit Hardware Interface  
**Status:** COMPLETED & VERIFIED (All 10 CTest Suites Passing, 0 Warnings, `/W4 /WX /permissive-`)  
**Target Architecture:** Native Windows x64 (MSVC 19.29 / Visual Studio 2019 BuildTools), Direct3D 12, XAudio2 / X3DAudio, Pure C++20 (`/std:c++latest`)  
**Design Philosophy:** Zero-Bloat, Zero Third-Party Runtime Dependencies (YSFlight Inspiration)

---

## 1. Executive Summary

Milestone M17 introduces a high-performance native Windows **Serial Cockpit Hardware Interface Subsystem** that connects the LokYS flight simulator to physical cockpit panels and microcontrollers (Arduino, STM32, Teensy, Raspberry Pi Pico, or custom home flight decks).

Built directly upon the Win32 Communications API (`CreateFileW`, `SetCommState`, `SetCommTimeouts`, `PurgeComm`, `GetOverlappedResult`), the subsystem requires **zero external serial libraries** (no Boost.Asio, QtSerialPort, or libserialport). It runs on a dedicated asynchronous worker thread, ensuring that COM port communication and I/O latency never block the 120 Hz fixed-timestep physics engine or the Direct3D 12 presentation loop.

A compact binary packet protocol with CRC-16-CCITT checksum validation enables high-frequency outbound telemetry streaming (~30 Hz) to drive physical instruments (analog dials, stepper motors, LED displays, and annunciator lamps) while ingesting 16-bit analog potentiometer axes and digital switch matrices directly into [`InputSystem`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/input/InputSystem.hpp).

---

## 2. Milestone Completion Status (M0 through M17)

| Milestone | Subsystem | Status | Key Deliverables |
| :--- | :--- | :--- | :--- |
| **M0** | Requirements & Baseline | **COMPLETED** | `REQUIREMENTS.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD.md` |
| **M1** | Toolchain & Build | **COMPLETED** | CMake 3.20+, Ninja, MSVC `/W4 /WX /permissive-`, zero warning tolerance |
| **M2** | Win32 Platform | **COMPLETED** | Native Win32 window, message pump, raw OS handles, High-DPI awareness |
| **M3** | Direct3D 12 Core | **COMPLETED** | DXGI 1.6 factory, D3D12 device, command queues, swapchain, fence synchronization |
| **M4** | GPU Adapter Enumeration | **COMPLETED** | VRAM budget query, adapter feature levels (RTX 3050 Laptop / Intel UHD / RDNA2 targets) |
| **M5** | Aerospace Math Engine | **COMPLETED** | Quaternions, 3D vectors, 3x3 / 4x4 matrices, NED/Body/Wind frames, SI $\leftrightarrow$ Aviation units |
| **M6** | Simulation Scheduler | **COMPLETED** | Decoupled 120 Hz fixed-timestep accumulator, triple-buffered snapshot state |
| **M7** | 6-DOF Rigid-Body Dynamics | **COMPLETED** | Runge-Kutta 4th Order (RK4) integrator, inertia tensors, gravitational dynamics |
| **M8** | Aerodynamics Subsystem | **COMPLETED** | Cessna 172 polar curves (NASA CR-2144), stall breakaway hysteresis, control authority |
| **M9** | Input Engine & Hardware | **COMPLETED** | DirectInput/Win32 joystick polling, quadratic curves, keyboard flight controls |
| **M10** | Integrated Flight Prototype | **COMPLETED** | 3D rendering pipeline, runway & terrain, Cockpit/Chase cameras, takeoff/landing loop |
| **M11** | Ground Reactions & Struts | **COMPLETED** | Multi-point oleo struts, progressive stiffness, hydraulic damping, tire scrub friction |
| **M12** | Propulsion & Fuel | **COMPLETED** | Lycoming O-320 engine, McCauley propeller polars, rotational integrator, P-factor, fuel burn |
| **M13** | glTF 2.0 Aircraft Loading | **COMPLETED** | Spec-compliant `.glb` parser, zero-dep JSON parser, 12-node articulated scene graph |
| **M14** | Analog Six-Pack Avionics | **COMPLETED** | 6 physical flight instruments (ASI, AI, ALT, TC, DG, VSI), sensor lag, D3D12 vector dials |
| **M15** | Spatial Audio Subsystem | **COMPLETED** | Native XAudio2/X3DAudio engine, Lycoming O-320 harmonic synthesis, aerodynamic wind, tire screech |
| **M16** | Radio Navigation Systems | **COMPLETED** | VOR radials, TO/FROM resolution, ILS Localizer & Glideslope, DME radar slant range, 3-marker beacons, 4-column avionics |
| **M17** | Serial Cockpit Hardware Interface | **COMPLETED** | Win32 COM overlapped async I/O, binary packet framing, CRC-16-CCITT, outbound telemetry streaming (~30 Hz), inbound 16-bit ADC control ingestion, virtual loopback testing |

---

## 3. Deep-Dive: Milestone M17 (Serial Cockpit Hardware Interface)

### 3.1 Overlapped Asynchronous Threading Model & Driver (`SerialPort`)

The [`SerialPort`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/serial/SerialPort.hpp) class interfaces directly with Windows kernel drivers:

- **Overlapped I/O Handles:** Opened with `FILE_FLAG_OVERLAPPED` via `CreateFileW`. Asynchronous read and write operations maintain dedicated OS event handles (`m_readEvent`, `m_writeEvent`), enabling non-blocking transfers.
- **Line Parameters (`DCB`):** Configured for standard baud rates (115200, 250000, 500000, 921600 bps), 8 data bits, no parity (`NOPARITY`), 1 stop bit (`ONESTOPBIT`), and hardware DTR/RTS flow asserted.
- **Immediate Return Timeouts:** Configured via `COMMTIMEOUTS` (`ReadIntervalTimeout = MAXDWORD`, `ReadTotalTimeoutMultiplier = 0`, `ReadTotalTimeoutConstant = 0`) to immediately return all bytes currently residing in the kernel buffer without polling delays.
- **Graceful Offline Fallback:** If the target COM port is not connected (e.g. on clean developer machines or automated CI runners), `Open()` returns `false` without crashing, throwing, or blocking. The flight simulator continues executing at full performance with keyboard/joystick controls.

---

### 3.2 Binary Packet Protocol & CRC-16-CCITT Framing

The protocol in [`SerialProtocol.hpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/serial/SerialProtocol.hpp) provides deterministic, zero-overhead binary framing:

```
+------------+------------+----------+--------------+------------------+-------------+-------------+
| SYNC_1     | SYNC_2     | MSG_ID   | LENGTH (N)   | PAYLOAD (N bytes)| CRC_16 (H)  | CRC_16 (L)  |
| 1 byte     | 1 byte     | 1 byte   | 1 byte       | N bytes          | 1 byte      | 1 byte      |
| 0x55       | 0xAA       | Enum     | 0..255       | Binary Struct    | MSB         | LSB         |
+------------+------------+----------+--------------+------------------+-------------+-------------+
```

#### CRC-16-CCITT Specification
- **Polynomial:** $0\text{x}1021$ ($x^{16} + x^{12} + x^5 + 1$)
- **Initial Remainder:** $0\text{xFFFF}$
- **Coverage:** Calculated over `[MSG_ID, LENGTH, PAYLOAD[0..N-1]]`.
- **Integrity Guarantee:** Detects all single-bit, double-bit, odd-numbered error bursts, and burst errors up to 16 bits.

#### Message Catalog ([`SerialTypes.hpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/serial/SerialTypes.hpp))
1. **`Heartbeat` (`0x01`):** Millisecond timestamp ping/pong for link health verification.
2. **`TelemetryFlightData` (`0x10`):** IAS, Pressure Altitude, Pitch, Roll, Heading, Vertical Speed, Rate of Turn, Slip/Skid Ball.
3. **`TelemetryEngine` (`0x11`):** Engine RPM, Manifold Pressure, Fuel Flow, Remaining Fuel, Magnetos state, Starter status.
4. **`TelemetryNav` (`0x12`):** NAV1 Active/Standby frequencies, OBS degree, CDI deflection dots, Glideslope deflection dots, DME distance/speed, TO/FROM flags, Marker lamp flags (OM/MM/IM).
5. **`TelemetryAnnunciators` (`0x13`):** Nose/Left/Right Gear down-and-locked, Brakes active, Stall warning, Low fuel, Vacuum fail.
6. **`InboundFlightControls` (`0x20`):** 16-bit integer ADC axes for Pitch, Roll, Yaw, Throttle, Mixture, Left Brake, Right Brake, Flap setting, and Active Axes bitmask.
7. **`InboundSwitches` (`0x21`):** 32-bit bitmask for cockpit toggle switches (Master Battery, Alternator, Avionics, Fuel Pump, Pitot Heat, Lights).
8. **`InboundNavControls` (`0x22`):** Frequency swap pulse, ident pulse, OBS rotary encoder ticks, frequency tuning ticks.

---

### 3.3 Streaming Parser & Noise Rejection State Machine

The [`SerialPacketDecoder`](file:///c:/Users/diego/Documents/Projetos/LokYS/include/LokYS/serial/SerialProtocol.hpp) implements a 7-state streaming parser:

```
[WaitingSync1] --(0x55)--> [WaitingSync2] --(0xAA)--> [ReadingMsgId] --> [ReadingLength]
      ^                          | (other)                                     |
      +-------(Framing Error)----+                                             v
      |                                                                 [ReadingPayload]
      +-------(CRC Mismatch)-----+                                             | (N bytes)
      |                          |                                             v
      +---(Dispatch Packet)<--- [ReadingCrc2] <--(CRC_H)-- [ReadingCrc1] <-----+
```

- **Resynchronization:** Automatically detects false preambles (e.g. isolated `0x55` or repeated `0x55 0x55 0xAA`) and recovers instantly.
- **Chunk Invariance:** Correctly decodes frames regardless of how bytes are partitioned across Windows buffer reads (from 1 byte per read to multiple complete frames in a single chunk).
- **Diagnostics:** Tracks cumulative statistics: `bytesSent`, `bytesReceived`, `packetsSent`, `packetsReceived`, `crcErrors`, `framingErrors`.

---

### 3.4 Inbound Hardware Control Ingestion

In [`InputSystem.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/src/input/InputSystem.cpp), `ApplySerialControls()` seamlessly ingests 16-bit microcontroller axes:

$$\text{Pitch}_{\text{norm}} = \operatorname{clamp}\left(\frac{\text{pitchAxis}}{32767.0}, -1.0, 1.0\right)$$
$$\text{Throttle}_{\text{norm}} = \operatorname{clamp}\left(\frac{\text{throttleAxis}}{65535.0}, 0.0, 1.0\right)$$
$$\text{Brakes}_{\text{norm}} = \max\left(\frac{\text{leftBrake}}{65535.0}, \frac{\text{rightBrake}}{65535.0}\right)$$

- **Rotary Encoders:** Relative tick counts directly update the OBS card and tune the NAV standby frequency in 50 kHz steps.
- **Axis Masking:** The `validAxesMask` ensures unmapped microcontroller pins do not override keyboard or joystick flight controls.

---

### 3.5 Virtual Loopback / Mock Serial Port Architecture

To achieve 100% automated testability on machines without physical USB serial hardware:
- `SerialPort::SetMockLoopback(true)` diverts transmissions into an internal thread-safe FIFO buffer that loops back into read requests.
- `InjectMockRxData()` enables automated tests to simulate arbitrary microcontroller packet streams with noise injection, bit flipping, and fragmentation.

---

## 4. Automated Test Verification & Metrics

The test suite [`SerialInterfaceTests.cpp`](file:///c:/Users/diego/Documents/Projetos/LokYS/tests/unit/SerialInterfaceTests.cpp) executes 7 automated test suites:

1. `TestCrc16CalculationAndVerification`: Standard ASCII vector "123456789" equals `0x29B1`; verifies bit corruption detection.
2. `TestPacketEncodingAndFraming`: Binary layout verification, sync preamble, and CRC positioning.
3. `TestPacketDecodingStream`: Multi-packet decoding across fragmented 3-byte chunks.
4. `TestNoiseRejectionAndRecovery`: Leading noise, corrupted packet rejection, and automatic recovery on subsequent valid packet.
5. `TestVirtualSerialPortMockLoopback`: Bidirectional mock transmission and packet deserialization.
6. `TestSerialInterfaceSystemEndToEnd`: Full-stack `SerialInterfaceSystem` telemetry broadcast and hardware control ingestion.
7. `TestInputSystemHardwareControlMerging`: 16-bit ADC to floating-point control axis mapping and rotary encoder OBS/frequency step integration.

### Test Execution Results
```
LokYS Native Windows Flight Simulator — Serial Hardware Tests
============================================================
[RUNNING] TestCrc16CalculationAndVerification...
[PASSED] TestCrc16CalculationAndVerification
[RUNNING] TestPacketEncodingAndFraming...
[PASSED] TestPacketEncodingAndFraming
[RUNNING] TestPacketDecodingStream...
[PASSED] TestPacketDecodingStream
[RUNNING] TestNoiseRejectionAndRecovery...
[PASSED] TestNoiseRejectionAndRecovery
[RUNNING] TestVirtualSerialPortMockLoopback...
[PASSED] TestVirtualSerialPortMockLoopback
[RUNNING] TestSerialInterfaceSystemEndToEnd...
[PASSED] TestSerialInterfaceSystemEndToEnd
[RUNNING] TestInputSystemHardwareControlMerging...
[PASSED] TestInputSystemHardwareControlMerging
============================================================
[ALL 7 SERIAL HARDWARE TEST SUITES PASSED (100%)]
```

### Full CTest Test Results
```
100% tests passed, 0 tests failed out of 10

Total Test time (real) =   0.23 sec
 1/10 Test  #1: AerospaceMathTests ...............   Passed    0.01 sec
 2/10 Test  #2: SixDofPhysicsTests ...............   Passed    0.01 sec
 3/10 Test  #3: AeroInputTests ...................   Passed    0.06 sec
 4/10 Test  #4: GroundReactionsTests .............   Passed    0.01 sec
 5/10 Test  #5: PropulsionTests ..................   Passed    0.01 sec
 6/10 Test  #6: GltfModelTests ...................   Passed    0.02 sec
 7/10 Test  #7: SixPackAvionicsTests .............   Passed    0.02 sec
 8/10 Test  #8: SpatialAudioTests ................   Passed    0.05 sec
 9/10 Test  #9: RadioNavTests ....................   Passed    0.01 sec
10/10 Test #10: SerialInterfaceTests .............   Passed    0.01 sec
```

### Verification Probe Exit
```
LokYS Native Windows Flight Simulator [Milestone M17 Serial Cockpit Hardware Interface]
============================================================
Detected GPU: NVIDIA GeForce RTX 3050 6GB Laptop GPU
Dedicated VRAM: 6001 MB
Direct3D 12 Feature Probe: PASSED
Fixed-Timestep Simulation Scheduler: 120 Hz OPERATIONAL
6-DOF Rigid-Body Dynamics Engine (RK4): OPERATIONAL
Cessna 172 Aerodynamic Model (NASA CR-2144): OPERATIONAL
Multi-Point Oleo-Pneumatic Strut Suspension & Ground Reactions: OPERATIONAL
Lycoming O-320 & McCauley Propeller Dynamics: OPERATIONAL
Fuel Burn Rate & Mixture Control: OPERATIONAL
Propeller Torque Reaction, P-Factor & Gyroscopic Precession: OPERATIONAL
glTF 2.0 Binary Loader & Articulated Aircraft Kinematics: OPERATIONAL
Analog Six-Pack Avionics Cluster (ASI, AI, ALT, TC, DG, VSI): OPERATIONAL
Pitot-Static & Aneroid Barometric Pressure Simulation: OPERATIONAL
Gyroscopic Gimbal Dynamics & Mechanical Precession Drift: OPERATIONAL
Calibrated Metering Capillary Pneumatic Sensor Lag: OPERATIONAL
Direct3D 12 2D Flight Instrument Cluster Vector Pipeline: OPERATIONAL
XAudio2 & X3DAudio Spatial Sound Subsystem (Engine, Wind, Ground): OPERATIONAL
VHF Radio Navigation & Instrument Landing Subsystem (VOR, ILS, DME, Markers): OPERATIONAL
Serial Cockpit Hardware Interface (Win32 COM Overlapped I/O & Microcontroller Ingestion): OPERATIONAL
============================================================
[M17 SERIAL COCKPIT HARDWARE INTERFACE VERIFICATION EXIT 0]
ExitCode=0
```

---

## 5. Next Steps

With Milestone M17 fully completed and verified, the next milestone in sequence is:
- **Milestone M18: Atmospheric & Weather Systems:**
  - International Standard Atmosphere (ISA) continuous lapse modeling ($T(h), p(h), \rho(h), a(h)$) through troposphere and tropopause.
  - Steady horizontal wind layer gradient with logarithmic surface boundary friction.
  - Crosswind runway vector decomposition and drift angle effects.
  - Discrete dryden gust model and stochastic thermal updraft/downdraft turbulence injection.
