@echo off
rem EuroBeamNG overlay: shows the BeamNG car (cabin / whole car) on top of ETS2.
rem Start it after ETS2, BeamNG and start_bridge.bat. Close this window to remove the overlay.
rem BeamNG's window is resized to ETS2's picture size so the two cameras line up.
cd /d "%~dp0bin" 2>nul || cd /d "%~dp0build"
overlay.exe
pause
