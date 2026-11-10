@echo off
rem Builds the invisible-truck ETS2 mod for the truck you drive, from your own game's files
rem (shadow and lights stay), and the invisible licence plates mod. Both go to
rem Documents\Euro Truck Simulator 2\mod\ - enable them in ETS2's Mod Manager.
rem Usage: make_invisible_truck.bat <truck id>, e.g. make_invisible_truck.bat volvo.fh16_2012
rem The id is in Documents\Euro Truck Simulator 2\game.log.txt: "[EuroBeamNG] player truck: ... id vehicle.<id>".
if "%~1"=="" (
  echo usage: make_invisible_truck.bat ^<truck id, e.g. volvo.fh16_2012^>
  echo The id is in Documents\Euro Truck Simulator 2\game.log.txt: "[EuroBeamNG] player truck: ... id vehicle.^<id^>"
  pause
  exit /b 2
)
set "ES_INVISIBLE=%~1"
"%~dp0bin\ets2_roads\ets2_roads.exe" --x 0 --z 0
pause
