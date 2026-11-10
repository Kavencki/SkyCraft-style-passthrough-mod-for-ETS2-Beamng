# EuroBeamNG installer: puts the ETS2 plugin, the BeamNG mod and the ETS2 helper mods in place.
# Run install.bat (close ETS2 and BeamNG first). Works from a release folder or from the source
# folder after build.bat. Nothing outside these places is changed:
#   <ETS2>\bin\win_x64\plugins\eurobeamng.dll (+ eurobeamng.ini, kept if already there)
#   %LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\eurobeamng\
#   Documents\Euro Truck Simulator 2\mod\eurobeamng_invisible_*.scs
#   install.ps1 [-Truck <truck id, e.g. volvo.fh16_2012>] [-TruckOnly]
param([string]$Truck = "", [switch]$TruckOnly)
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
. "$root\paths.ps1"
function First([string[]]$paths) { foreach ($p in $paths) { if (Test-Path $p) { return $p } } return $null }

# The files: release layout first, then the source folder's build output.
$plugin = First @("$root\ets2\eurobeamng.dll", "$root\build\eurobeamng.dll")
$pluginIni = First @("$root\ets2\eurobeamng.ini", "$root\ets2_plugin\eurobeamng.ini")
$beamMod = First @("$root\beamng\eurobeamng", "$root\beamng_mod")
$gen = First @("$root\bin\ets2_roads\ets2_roads.exe", "$root\build\ets2_roads\ets2_roads.exe")
if (-not $plugin -or -not $beamMod -or -not $gen) { Say "EuroBeamNG files missing next to this script (source folder: run build.bat first)." Red; exit 1 }

Say "EuroBeamNG installer" Cyan
Assert-Closed
$ets2 = Find-Ets2
if (-not $ets2) { Say "Euro Truck Simulator 2 (Steam) not found." Red; exit 1 }
Remove-OldName $ets2

if (-not $TruckOnly) {
    # 1. ETS2 plugin.
    $plugDir = Join-Path $ets2 "bin\win_x64\plugins"
    New-Item -ItemType Directory -Force $plugDir | Out-Null
    Copy-Item $plugin (Join-Path $plugDir "eurobeamng.dll") -Force
    if (-not (Test-Path (Join-Path $plugDir "eurobeamng.ini"))) { Copy-Item $pluginIni (Join-Path $plugDir "eurobeamng.ini") }
    Say "ETS2 plugin -> $plugDir" Green

    # 2. BeamNG mod (unpacked: the road tiles are written into it).
    if (-not (Test-Path $BeamUser)) { Say "BeamNG.drive's user folder not found ($BeamUser): start BeamNG.drive once, then run this again." Red; exit 1 }
    $modDir = Join-Path $BeamUser "mods\unpacked\eurobeamng"
    New-Item -ItemType Directory -Force $modDir | Out-Null
    foreach ($d in @("lua", "scripts", "art")) {
        if (Test-Path "$beamMod\$d") { Copy-Item "$beamMod\$d" $modDir -Recurse -Force }
    }
    Say "BeamNG mod -> $modDir" Green
}

# 3. ETS2 helper mods: invisible licence plates, and the invisible player truck - built here from
# the game's own files, for the truck you drive (its id is in ETS2's log once the plugin has run).
$log = Join-Path $Ets2Docs "game.log.txt"
if (-not $Truck -and (Test-Path $log)) {
    # ([EuroSpeed]: the log of a version from before the new name)
    $m = Select-String -Path $log -Pattern "\[Euro(?:BeamNG|Speed)\] player truck: brand_id \S+, id vehicle\.(\S+)" | Select-Object -Last 1
    if ($m) { $Truck = $m.Matches[0].Groups[1].Value }
}
if ($Truck) {
    Say "Building the invisible truck mod for $Truck (about a minute) ..."
    $env:ES_INVISIBLE = $Truck
    $env:ES_ETS2_DIR = $ets2
    & $gen --x 0 --z 0 | Select-Object -Last 2 | ForEach-Object { Say "  $_" }
    Remove-Item Env:ES_INVISIBLE, Env:ES_ETS2_DIR
    Say "In ETS2's Mod Manager enable 'EuroBeamNG invisible truck' and 'EuroBeamNG invisible licence plates'." Yellow
} else {
    Say "Invisible truck mod: start ETS2 once (with the plugin, in your truck), then run make_invisible_truck.bat." Yellow
}
if (-not $TruckOnly) { Say "Done. ETS2 asks once about 'advanced SDK features': answer OK." Cyan }
