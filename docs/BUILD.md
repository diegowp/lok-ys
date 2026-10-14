# Build & Toolchain Guide: LokYS Flight Simulator

## 1. Verified System Environment

* **Compiler:** Microsoft Visual C++ Optimizing Compiler (MSVC v142, version 19.29.30159 / VS 2019 Build Tools)
* **Windows SDK:** Windows 10 SDK 10.0.19041.0
* **Build Generator:** CMake 3.20+ with Ninja or Visual Studio 16 2019
* **DirectX Version:** Native Direct3D 12 (`d3d12.lib`, `dxgi.lib`, `d3dcompiler.lib`)

---

## 2. Environment Initialization

To configure your command prompt with the 64-bit MSVC toolchain, run:

```cmd
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
```

Or in PowerShell:
```powershell
cmd /c 'call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" && set' | Foreach-Object {
    if ($_ -match '^(.*?)=(.*)$') { Set-Item -Path "env:$($matches[1])" -Value $matches[2] }
}
```

---

## 3. CMake Build Commands

### 3.1 Configure (Release or Debug)
Using the bundled CMake and Ninja:

```cmd
"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM="C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
```

Or using the Visual Studio Solution generator:
```cmd
"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -B build -G "Visual Studio 16 2019" -A x64
```

### 3.2 Build
```cmd
"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build --config Release
```

### 3.3 Run Tests
```cmd
ctest --test-dir build --output-on-failure -C Release
```
