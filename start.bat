@echo off
rem EuroBeamNG: starts the bridge and, a few seconds later, the overlay, each in its own window.
rem Start ETS2 (in your truck) and BeamNG (Grid Map / smallgrid, freeroam, a car) first.
start "EuroBeamNG bridge" "%~dp0start_bridge.bat"
timeout /t 5 /nobreak >nul
start "EuroBeamNG overlay" "%~dp0start_overlay.bat"
