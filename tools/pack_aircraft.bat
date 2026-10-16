@echo off
if "%~1"=="" (
    echo ========================================================
    echo Assistente de Empacotamento de Aeronaves LokYS
    echo ========================================================
    echo Uso: pack_aircraft.bat nome_do_aviao
    echo.
    echo Exemplos:
    echo   tools\pack_aircraft.bat cessna172r
    echo   tools\pack_aircraft.bat f16
    echo   tools\pack_aircraft.bat b747
    echo ========================================================
    exit /b 1
)
powershell -ExecutionPolicy Bypass -File "%~dp0pack_aircraft.ps1" -AircraftName "%~1"
