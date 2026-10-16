param (
    [Parameter(Mandatory=$true, Position=0)]
    [string]$AircraftName,
    [string]$YsflightAircraftDir = "Ysflight\aircraft",
    [string]$OutputDir = "assets\aircraft"
)

$ProgressPreference = 'SilentlyContinue'

$baseName = [System.IO.Path]::GetFileNameWithoutExtension($AircraftName).ToLowerInvariant()

# Procura arquivos correspondentes ao aviao (.dat, .dnm, .ist, .srf)
$files = Get-ChildItem -Path $YsflightAircraftDir -Filter "$baseName*.*" -ErrorAction SilentlyContinue | Where-Object {
    $_.Extension -in @(".dat", ".dnm", ".ist", ".srf")
}

if (-not $files) {
    Write-Host "Erro: Nenhum arquivo encontrado para '$baseName' em '$YsflightAircraftDir'!" -ForegroundColor Red
    exit 1
}

Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

$targetDirs = @($OutputDir)
if (Test-Path "build\assets\aircraft") {
    $targetDirs += "build\assets\aircraft"
}

foreach ($targetDir in $targetDirs) {
    if (-not (Test-Path $targetDir)) {
        New-Item -ItemType Directory -Force -Path $targetDir | Out-Null
    }

    $zipPath = Join-Path $targetDir "$baseName.acft.zip"
    if (Test-Path $zipPath) {
        Remove-Item $zipPath -Force
    }

    $zipArchive = [System.IO.Compression.ZipFile]::Open($zipPath, [System.IO.Compression.ZipArchiveMode]::Create)
    try {
        foreach ($f in $files) {
            [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
                $zipArchive,
                $f.FullName,
                $f.Name,
                [System.IO.Compression.CompressionLevel]::NoCompression
            ) | Out-Null
        }
    }
    finally {
        $zipArchive.Dispose()
    }

    Write-Host "Sucesso! Aeronave '$baseName' empacotada em: $zipPath" -ForegroundColor Green
}

Write-Host "Arquivos incluidos no pacote:"
$files | ForEach-Object { Write-Host "  - $($_.Name)" }
