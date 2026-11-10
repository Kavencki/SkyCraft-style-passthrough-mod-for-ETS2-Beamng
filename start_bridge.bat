@echo off
rem EuroSpeed: starts the bridge between BeamNG (physics) and ETS2 (world).
rem Start ETS2 and BeamNG (Grid Map / smallgrid, freeroam) in any order. Keep this window open while driving.
rem Keys here: R = re-anchor, Q = quit. Everything shown here is also written to bridge.log.
cd /d "%~dp0bin" 2>nul || cd /d "%~dp0build"
powershell -NoProfile -ExecutionPolicy Bypass -Command "& '.\lfs_bridge.exe' --tiles \"$env:LOCALAPPDATA\BeamNG\BeamNG.drive\current\mods\unpacked\eurospeed\art\eurospeed\tiles\tiles.idx\" | Tee-Object -FilePath 'bridge.log'"
pause
