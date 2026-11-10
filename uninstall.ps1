# EuroBeamNG uninstaller: removes what install.ps1 put in place (close ETS2 and BeamNG first):
# the ETS2 plugin and its settings, the BeamNG mod (with its road tiles) and the ETS2 helper mods.
# This folder (and its settings) stays: delete it yourself.
$ErrorActionPreference = "Stop"
. "$PSScriptRoot\paths.ps1"
Say "EuroBeamNG uninstaller" Cyan
Assert-Closed
$gone = @()
$ets2 = Find-Ets2
Remove-OldName $ets2
if ($ets2) {
    foreach ($f in @("eurobeamng.dll", "eurobeamng.ini", "eurobeamng_re.dll")) {   # (eurobeamng_re: an early research build)
        $p = Join-Path $ets2 "bin\win_x64\plugins\$f"
        if (Test-Path $p) { Remove-Item $p; $gone += $p }
    }
}
foreach ($f in @("eurobeamng_invisible_truck.scs", "eurobeamng_invisible_plates.scs")) {
    $p = Join-Path $Ets2Docs "mod\$f"
    if (Test-Path $p) { Remove-Item $p; $gone += $p }
}
$modDir = Join-Path $BeamUser "mods\unpacked\eurobeamng"
if (Test-Path $modDir) { Remove-Item $modDir -Recurse; $gone += $modDir }
$ter = Join-Path $BeamUser "temp\eurobeamng"
if (Test-Path $ter) { Remove-Item $ter -Recurse; $gone += $ter }
if ($gone.Count -eq 0) { Say "Nothing of EuroBeamNG was installed." } else { $gone | ForEach-Object { Say "removed $_" Green } }
