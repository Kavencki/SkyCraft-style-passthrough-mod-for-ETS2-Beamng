# Where ETS2 and BeamNG.drive keep their files (dot-sourced by install.ps1 and uninstall.ps1).

# ETS2's folder, through Steam's own list of its libraries (libraryfolders.vdf), like the road
# generator does. $null if not found.
function Find-Ets2 {
    $steams = @()
    foreach ($k in @("HKCU:\Software\Valve\Steam", "HKLM:\SOFTWARE\WOW6432Node\Valve\Steam")) {
        try { $v = Get-ItemProperty $k -ErrorAction Stop; foreach ($n in @("SteamPath", "InstallPath")) { if ($v.$n) { $steams += ($v.$n -replace "/", "\") } } } catch {}
    }
    $libs = @()
    foreach ($s in ($steams | Select-Object -Unique)) {
        $libs += $s
        $vdf = Join-Path $s "steamapps\libraryfolders.vdf"
        if (Test-Path $vdf) { foreach ($m in [regex]::Matches((Get-Content $vdf -Raw), '"path"\s+"([^"]+)"')) { $libs += ($m.Groups[1].Value -replace "\\\\", "\") } }
    }
    foreach ($l in ($libs | Select-Object -Unique)) {
        $d = Join-Path $l "steamapps\common\Euro Truck Simulator 2"
        if (Test-Path (Join-Path $d "base.scs")) { return $d }
    }
    return $null
}

# BeamNG.drive's user folder (mods, settings).
$BeamUser = Join-Path $env:LOCALAPPDATA "BeamNG\BeamNG.drive\current"
# ETS2's documents folder (game.log.txt, mod\).
$Ets2Docs = Join-Path ([Environment]::GetFolderPath("MyDocuments")) "Euro Truck Simulator 2"

function Say([string]$m, [string]$c = "Gray") { Write-Host $m -ForegroundColor $c }

# The games and the bridge must be closed (their files are in use).
function Assert-Closed {
    foreach ($p in @("eurotrucks2", "BeamNG.drive.x64", "lfs_bridge")) {
        if (Get-Process $p -ErrorAction SilentlyContinue) { Say "Close $p first, then run this again." Red; exit 1 }
    }
}
