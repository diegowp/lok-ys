cmake_minimum_required(VERSION 3.20)

project(LokYSFlightSimulator
    VERSION 0.1.0
    DESCRIPTION "High-Performance Native Windows Direct3D 12 Flight Simulator"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DUNICODE -D_UNICODE)

# Enable strict warning hygiene
if(MSVC)
    add_compile_options(
        /W4
        /WX
        /permissive-
        /utf-8
        /Zc:inline
        /Zc:preprocessor
        /wd5105
        /wd4324
        /diagnostics:column
    )

    # Release optimizations
    add_compile_options("$<$<CONFIG:Release>:/O2;/Oi;/Ot;/GS->")
    # Debug settings
    add_compile_options("$<$<CONFIG:Debug>:/Od;/Zi;/RTC1>")
endif()

# Global include directory
include_directories(include)

# Source files for Milestones M1 - M13
set(LOKYS_SOURCES
    src/app/main.cpp
    src/core/EngineContext.cpp
    src/core/Timer.cpp
    src/platform/Window.cpp
    src/graphics/GraphicsDevice.cpp
    src/graphics/GpuCapabilities.cpp
    src/graphics/Camera.cpp
    src/graphics/Mesh.cpp
    src/graphics/Renderer3D.cpp
    src/graphics/GltfModel.cpp
    src/graphics/AircraftModelGenerator.cpp
    src/physics/RigidBody.cpp
    src/physics/GroundReactions.cpp
    src/aerodynamics/AerodynamicsSystem.cpp
    src/input/InputSystem.cpp
    src/propulsion/PropulsionSystem.cpp
    src/avionics/AvionicsSystem.cpp
    src/graphics/AvionicsRenderer.cpp
    src/audio/AudioSystem.cpp
    src/navigation/NavDatabase.cpp
    src/navigation/RadioNavSystem.cpp
    src/serial/SerialPort.cpp
    src/serial/SerialInterfaceSystem.cpp
    src/environment/AtmosphereModel.cpp
    src/environment/WindModel.cpp
    src/environment/EnvironmentSystem.cpp
    src/graphics/ShadowMap.cpp
    src/graphics/TerrainSystem.cpp
    src/graphics/ScenerySystem.cpp
    src/network/UdpSocket.cpp
    src/network/RemoteAircraft.cpp
    src/network/NetworkSystem.cpp
    src/autopilot/AutopilotSystem.cpp
    src/graphics/GlassCockpitRenderer.cpp
)

set(LOKYS_HEADERS
    include/LokYS/core/EngineContext.hpp
    include/LokYS/core/Types.hpp
    include/LokYS/core/Timer.hpp
    include/LokYS/core/JsonParser.hpp
    include/LokYS/platform/Window.hpp
    include/LokYS/graphics/GraphicsDevice.hpp
    include/LokYS/graphics/GpuCapabilities.hpp
    include/LokYS/graphics/Camera.hpp
    include/LokYS/graphics/Mesh.hpp
    include/LokYS/graphics/Renderer3D.hpp
    include/LokYS/graphics/GltfTypes.hpp
    include/LokYS/graphics/GltfModel.hpp
    include/LokYS/graphics/AircraftModelGenerator.hpp
    include/LokYS/graphics/AvionicsRenderer.hpp
    include/LokYS/avionics/AvionicsTypes.hpp
    include/LokYS/avionics/AvionicsSystem.hpp
    include/LokYS/math/Units.hpp
    include/LokYS/math/Vector3.hpp
    include/LokYS/math/Matrix3x3.hpp
    include/LokYS/math/Matrix4x4.hpp
    include/LokYS/math/Quaternion.hpp
    include/LokYS/math/CoordinateFrames.hpp
    include/LokYS/physics/RigidBodyState.hpp
    include/LokYS/physics/RigidBody.hpp
    include/LokYS/physics/GroundReactions.hpp
    include/LokYS/simulation/SimulationSnapshot.hpp
    include/LokYS/simulation/SimulationScheduler.hpp
    include/LokYS/aerodynamics/AerodynamicsSystem.hpp
    include/LokYS/input/InputSystem.hpp
    include/LokYS/propulsion/PropulsionSystem.hpp
    include/LokYS/audio/AudioTypes.hpp
    include/LokYS/audio/AudioSystem.hpp
    include/LokYS/navigation/NavTypes.hpp
    include/LokYS/navigation/NavDatabase.hpp
    include/LokYS/navigation/RadioNavSystem.hpp
    include/LokYS/serial/SerialTypes.hpp
    include/LokYS/serial/SerialProtocol.hpp
    include/LokYS/serial/SerialPort.hpp
    include/LokYS/serial/SerialInterfaceSystem.hpp
    include/LokYS/environment/AtmosphereTypes.hpp
    include/LokYS/environment/AtmosphereModel.hpp
    include/LokYS/environment/WindModel.hpp
    include/LokYS/environment/EnvironmentSystem.hpp
    include/LokYS/graphics/ShadowMap.hpp
    include/LokYS/graphics/TerrainSystem.hpp
    include/LokYS/graphics/ScenerySystem.hpp
    include/LokYS/network/NetworkTypes.hpp
    include/LokYS/network/NetworkProtocol.hpp
    include/LokYS/network/UdpSocket.hpp
    include/LokYS/network/RemoteAircraft.hpp
    include/LokYS/network/NetworkSystem.hpp
    include/LokYS/autopilot/AutopilotTypes.hpp
    include/LokYS/autopilot/PidController.hpp
    include/LokYS/autopilot/AutopilotSystem.hpp
    include/LokYS/graphics/GlassCockpitRenderer.hpp
)

add_executable(LokYSFlightSimulator WIN32
    ${LOKYS_SOURCES}
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSFlightSimulator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link essential Windows, Direct3D 12, XAudio2, and Winsock2 libraries
target_link_libraries(LokYSFlightSimulator PRIVATE
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
    user32.lib
    gdi32.lib
    shell32.lib
    kernel32.lib
    winmm.lib
    xaudio2.lib
    ole32.lib
    ws2_32.lib
)

set_target_properties(LokYSFlightSimulator PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Automated Test Suite for Aerospace Math and Invariants
enable_testing()

add_executable(LokYSMathTests
    tests/unit/MathTests.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSMathTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME AerospaceMathTests COMMAND LokYSMathTests)

add_executable(LokYSPhysicsTests
    tests/unit/PhysicsTests.cpp
    src/physics/RigidBody.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSPhysicsTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME SixDofPhysicsTests COMMAND LokYSPhysicsTests)

add_executable(LokYSAeroInputTests
    tests/unit/AeroInputTests.cpp
    src/aerodynamics/AerodynamicsSystem.cpp
    src/input/InputSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSAeroInputTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSAeroInputTests PRIVATE
    winmm.lib
)

add_test(NAME AeroInputTests COMMAND LokYSAeroInputTests)

add_executable(LokYSGroundReactionsTests
    tests/unit/GroundReactionsTests.cpp
    src/physics/GroundReactions.cpp
    src/physics/RigidBody.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSGroundReactionsTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME GroundReactionsTests COMMAND LokYSGroundReactionsTests)

add_executable(LokYSPropulsionTests
    tests/unit/PropulsionTests.cpp
    src/propulsion/PropulsionSystem.cpp
    src/physics/RigidBody.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSPropulsionTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME PropulsionTests COMMAND LokYSPropulsionTests)

add_executable(LokYSGltfTests
    tests/unit/GltfTests.cpp
    src/graphics/GltfModel.cpp
    src/graphics/AircraftModelGenerator.cpp
    src/physics/RigidBody.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSGltfTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSGltfTests PRIVATE
    d3d12.lib
    dxgi.lib
)

add_test(NAME GltfModelTests COMMAND LokYSGltfTests)

add_executable(LokYSAvionicsTests
    tests/unit/AvionicsTests.cpp
    src/avionics/AvionicsSystem.cpp
    src/physics/RigidBody.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSAvionicsTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME SixPackAvionicsTests COMMAND LokYSAvionicsTests)

add_executable(LokYSAudioTests
    tests/unit/AudioTests.cpp
    src/audio/AudioSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSAudioTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSAudioTests PRIVATE
    xaudio2.lib
    ole32.lib
)

add_test(NAME SpatialAudioTests COMMAND LokYSAudioTests)

add_executable(LokYSRadioNavTests
    tests/unit/RadioNavTests.cpp
    src/navigation/NavDatabase.cpp
    src/navigation/RadioNavSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSRadioNavTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME RadioNavTests COMMAND LokYSRadioNavTests)

add_executable(LokYSSerialInterfaceTests
    tests/unit/SerialInterfaceTests.cpp
    src/serial/SerialPort.cpp
    src/serial/SerialInterfaceSystem.cpp
    src/input/InputSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSSerialInterfaceTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSSerialInterfaceTests PRIVATE
    winmm.lib
)

add_test(NAME SerialInterfaceTests COMMAND LokYSSerialInterfaceTests)

add_executable(LokYSAtmosphereWeatherTests
    tests/unit/AtmosphereWeatherTests.cpp
    src/environment/AtmosphereModel.cpp
    src/environment/WindModel.cpp
    src/environment/EnvironmentSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSAtmosphereWeatherTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME AtmosphereWeatherTests COMMAND LokYSAtmosphereWeatherTests)

add_executable(LokYSGraphicsOptimizationTests
    tests/unit/GraphicsOptimizationTests.cpp
    src/graphics/ShadowMap.cpp
    src/graphics/TerrainSystem.cpp
    src/graphics/ScenerySystem.cpp
    src/graphics/Mesh.cpp
    src/graphics/Camera.cpp
    src/environment/AtmosphereModel.cpp
    src/environment/WindModel.cpp
    src/environment/EnvironmentSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSGraphicsOptimizationTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSGraphicsOptimizationTests PRIVATE
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

add_test(NAME GraphicsOptimizationTests COMMAND LokYSGraphicsOptimizationTests)

add_executable(LokYSNetworkMultiplayerTests
    tests/unit/NetworkMultiplayerTests.cpp
    src/network/UdpSocket.cpp
    src/network/RemoteAircraft.cpp
    src/network/NetworkSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSNetworkMultiplayerTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LokYSNetworkMultiplayerTests PRIVATE
    ws2_32.lib
)

add_test(NAME NetworkMultiplayerTests COMMAND LokYSNetworkMultiplayerTests)

add_executable(LokYSGlassCockpitAutopilotTests
    tests/unit/GlassCockpitAutopilotTests.cpp
    src/autopilot/AutopilotSystem.cpp
    src/physics/RigidBody.cpp
    src/aerodynamics/AerodynamicsSystem.cpp
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSGlassCockpitAutopilotTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME GlassCockpitAutopilotTests COMMAND LokYSGlassCockpitAutopilotTests)

