# Builds a EuroSpeed release: release\EuroSpeed-<version>\ and release\EuroSpeed-<version>.zip.
#   powershell -ExecutionPolicy Bypass -File make_release.ps1 -Version 0.1.0
# Needs VS 2019+ Build Tools (C++) and the .NET 10 SDK. The road generator is published with .NET
# inside it, so players need nothing else.
#
# Layout: bin\ (bridge, overlay, generator, settings), ets2\ (plugin), beamng\eurospeed\ (mod),
# and the install / start scripts, README, licences at the top.
param([Parameter(Mandatory = $true)][string]$Version)
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$name = "EuroSpeed-$Version"
$out = Join-Path $root "release\$name"

# 1. Everything C++.
cmd /c "`"$root\build.bat`""
if ($LASTEXITCODE -ne 0) { throw "build.bat failed" }

# 2. A clean release folder.
if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Force "$out\bin", "$out\ets2", "$out\beamng\eurospeed" | Out-Null

# 3. The road generator, self-contained (win-x64).
dotnet publish "$root\tools\ets2_roads" -c Release -r win-x64 --self-contained true -o "$out\bin\ets2_roads" --nologo -v q
if ($LASTEXITCODE -ne 0) { throw "dotnet publish failed" }
Get-ChildItem "$out\bin\ets2_roads" -Filter *.pdb | Remove-Item

# 4. The rest.
foreach ($f in @("lfs_bridge.exe", "overlay.exe", "wheeltest.exe", "codescan_test.exe", "sigscan.exe")) { Copy-Item "$root\build\$f" "$out\bin\" }
Copy-Item "$root\config\*.ini" "$out\bin\"
Copy-Item "$root\build\eurospeed.dll", "$root\ets2_plugin\eurospeed.ini" "$out\ets2\"
foreach ($d in @("lua", "scripts")) { Copy-Item "$root\beamng_mod\$d" "$out\beamng\eurospeed\" -Recurse }
New-Item -ItemType Directory -Force "$out\beamng\eurospeed\art\eurospeed" | Out-Null
foreach ($f in @("keybox.dae", "main.materials.json", "terrain")) { Copy-Item "$root\beamng_mod\art\eurospeed\$f" "$out\beamng\eurospeed\art\eurospeed\" -Recurse }
foreach ($f in @("install.bat", "install.ps1", "uninstall.bat", "uninstall.ps1", "paths.ps1", "make_invisible_truck.bat",
                 "start.bat", "start_bridge.bat", "start_overlay.bat", "README.md", "LICENSE", "THIRD_PARTY.md")) {
    Copy-Item "$root\$f" "$out\"
}
Set-Content -Path "$out\VERSION.txt" -Value "EuroSpeed $Version" -Encoding ascii

# 5. The zip.
$zip = Join-Path $root "release\$name.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path $out -DestinationPath $zip -CompressionLevel Optimal
$mb = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Host "release: $zip ($mb MB)" -ForegroundColor Green
