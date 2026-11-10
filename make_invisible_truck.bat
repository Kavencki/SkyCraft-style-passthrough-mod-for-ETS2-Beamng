@echo off
rem Builds the invisible-truck ETS2 mod for the truck you drive (shadow and lights stay).
rem Usage: make_invisible_truck.bat [truck id, e.g. volvo.fh16_2012]
rem Without an id it takes the truck from ETS2's game.log.txt ("[EuroBeamNG] player truck: ... id vehicle.<id>").
rem Then enable "EuroBeamNG invisible truck" in ETS2's Mod Manager. Run it again after you change trucks.
if "%~1"=="" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -TruckOnly
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -TruckOnly -Truck "%~1"
)
pause
