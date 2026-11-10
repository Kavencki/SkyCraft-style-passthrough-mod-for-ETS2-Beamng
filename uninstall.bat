@echo off
rem EuroBeamNG: removes the ETS2 plugin, the BeamNG mod and the ETS2 helper mods. Close ETS2 and BeamNG first.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0uninstall.ps1"
pause
