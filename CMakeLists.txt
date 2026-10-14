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
)

add_executable(LokYSFlightSimulator WIN32
    ${LOKYS_SOURCES}
    ${LOKYS_HEADERS}
)

target_include_directories(LokYSFlightSimulator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link essential Windows and Direct3D 12 libraries
target_link_libraries(LokYSFlightSimulator PRIVATE
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
    user32.lib
    gdi32.lib
    shell32.lib
    kernel32.lib
    winmm.lib
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



