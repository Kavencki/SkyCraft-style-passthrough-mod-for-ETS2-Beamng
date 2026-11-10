# Builds the EuroBeamNG release packages:
#   release\EuroBeamNG-<version>.zip         with the installer (install.bat / uninstall.bat)
#   release\EuroBeamNG-<version>-manual.zip  without it: INSTALL_MANUAL.md says where everything goes
#   powershell -ExecutionPolicy Bypass -File make_release.ps1 -Version 0.1.0
# Needs VS 2019+ Build Tools (C++) and the .NET 10 SDK. The road generator is published with .NET
# inside it, so players need nothing else.
#
# Layout: bin\ (bridge, overlay, generator, settings), ets2\ (plugin), beamng\eurobeamng\ (mod),
# and the scripts, README, guides, licences at the top.
param([Parameter(Mandatory = $true)][string]$Version)
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

# 1. Everything C++, and the generator (self-contained, win-x64) once for both packages.
cmd /c "`"$root\build.bat`""
if ($LASTEXITCODE -ne 0) { throw "build.bat failed" }
$gen = Join-Path $root "release\_generator"
if (Test-Path $gen) { Remove-Item $gen -Recurse -Force }
dotnet publish "$root\tools\ets2_roads" -c Release -r win-x64 --self-contained true -o $gen --nologo -v q
if ($LASTEXITCODE -ne 0) { throw "dotnet publish failed" }
Get-ChildItem $gen -Filter *.pdb | Remove-Item

function Package([string]$name, [bool]$manual) {
    $out = Join-Path $root "release\$name"
    if (Test-Path $out) { Remove-Item $out -Recurse -Force }
    New-Item -ItemType Directory -Force "$out\bin", "$out\ets2", "$out\beamng\eurobeamng\art\eurobeamng" | Out-Null
    Copy-Item $gen "$out\bin\ets2_roads" -Recurse
    foreach ($f in @("lfs_bridge.exe", "overlay.exe", "wheeltest.exe", "codescan_test.exe", "sigscan.exe")) { Copy-Item "$root\build\$f" "$out\bin\" }
    Copy-Item "$root\config\*.ini" "$out\bin\"
    Copy-Item "$root\build\eurobeamng.dll", "$root\ets2_plugin\eurobeamng.ini" "$out\ets2\"
    foreach ($d in @("lua", "scripts")) { Copy-Item "$root\beamng_mod\$d" "$out\beamng\eurobeamng\" -Recurse }
    foreach ($f in @("keybox.dae", "main.materials.json", "terrain")) { Copy-Item "$root\beamng_mod\art\eurobeamng\$f" "$out\beamng\eurobeamng\art\eurobeamng\" -Recurse }
    $files = @("start.bat", "start_bridge.bat", "start_overlay.bat", "README.md", "INSTALL_MANUAL.md", "LICENSE", "THIRD_PARTY.md")
    if ($manual) {
        Copy-Item "$root\dist\manual\make_invisible_truck.bat" "$out\"   # runs the generator only
    } else {
        $files += @("install.bat", "install.ps1", "uninstall.bat", "uninstall.ps1", "paths.ps1", "make_invisible_truck.bat")
    }
    foreach ($f in $files) { Copy-Item "$root\$f" "$out\" }
    Set-Content -Path "$out\VERSION.txt" -Value ("EuroBeamNG $Version" + $(if ($manual) { " (manual install)" } else { "" })) -Encoding ascii
    $zip = Join-Path $root "release\$name.zip"
    if (Test-Path $zip) { Remove-Item $zip }
    Compress-Archive -Path $out -DestinationPath $zip -CompressionLevel Optimal
    Write-Host ("release: {0} ({1:N1} MB)" -f $zip, ((Get-Item $zip).Length / 1MB)) -ForegroundColor Green
}

Package "EuroBeamNG-$Version" $false
Package "EuroBeamNG-$Version-manual" $true
Remove-Item $gen -Recurse -Force
