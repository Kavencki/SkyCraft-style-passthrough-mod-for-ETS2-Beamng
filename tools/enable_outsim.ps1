# Enables LFS OutSim for EuroSpeed (127.0.0.1:30000, opts 3f). Run with LFS closed:
# LFS rewrites cfg.txt on exit. Backs up cfg.txt to cfg.txt.eurospeed-backup first.
param([string]$Lfs = "D:\LFS NEXT GEN\LFS")
$cfg = Join-Path $Lfs "cfg.txt"
if (Get-Process LFS -ErrorAction SilentlyContinue) { Write-Error "Close LFS first."; exit 1 }
Copy-Item $cfg "$cfg.eurospeed-backup" -Force
(Get-Content $cfg) -replace '^OutSim Mode .*', 'OutSim Mode 2' `
                   -replace '^OutSim IP .*', 'OutSim IP 127.0.0.1' `
                   -replace '^OutSim Port .*', 'OutSim Port 30000' `
                   -replace '^OutSim Opts .*', 'OutSim Opts 3f' | Set-Content $cfg -Encoding ascii
Select-String $cfg -Pattern '^OutSim' | ForEach-Object Line
