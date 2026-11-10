# Installing EuroBeamNG by hand

Everything `install.bat` does, step by step. Close ETS2 and BeamNG.drive first.

## What goes where

| From this folder | To | |
|---|---|---|
| `ets2\eurobeamng.dll` | `<ETS2>\bin\win_x64\plugins\` | the ETS2 plugin |
| `ets2\eurobeamng.ini` | `<ETS2>\bin\win_x64\plugins\` | its settings |
| `beamng\eurobeamng\` (the whole folder) | `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\` | the BeamNG mod |
| built by `make_invisible_truck.bat` | `Documents\Euro Truck Simulator 2\mod\` | `eurobeamng_invisible_truck.scs`, `eurobeamng_invisible_plates.scs` |
| `bin\` | stays here | bridge, overlay, road generator, their settings |

## 1. The ETS2 plugin

1. Find ETS2's folder: in Steam, right-click *Euro Truck Simulator 2 → Manage → Browse local
   files*. That folder is `<ETS2>` below.
2. Open `<ETS2>\bin\win_x64\`. Make a folder `plugins` there if there is none.
3. Copy `ets2\eurobeamng.dll` and `ets2\eurobeamng.ini` into `<ETS2>\bin\win_x64\plugins\`.

## 2. The BeamNG mod

1. Start BeamNG.drive once if you never have, then close it.
2. Press `Win + R`, type `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods` and press Enter.
3. Make a folder `unpacked` there if there is none.
4. Copy the folder `beamng\eurobeamng` into `unpacked`, so you get
   `...\mods\unpacked\eurobeamng\lua`, `...\scripts` and `...\art`.

   It must stay an unpacked folder (not a zip): the bridge writes the road tiles into it.

## 3. The invisible truck and licence plates (ETS2 mods)

The truck mod is made from your own game's files, for the truck you drive.

1. Start ETS2, load your profile and sit in your truck. ETS2 asks once about
   *advanced SDK features*: answer **OK**. Close ETS2.
2. Open `Documents\Euro Truck Simulator 2\game.log.txt` and find the line
   `[EuroBeamNG] player truck: brand_id ..., id vehicle.<truck id>`,
   e.g. `id vehicle.volvo.fh16_2012`. The truck id is the part after `vehicle.`: `volvo.fh16_2012`.
3. Open a command prompt in this folder (type `cmd` in Explorer's address bar and press Enter)
   and run `make_invisible_truck.bat volvo.fh16_2012` with your id. It takes about a minute and
   writes both mods to `Documents\Euro Truck Simulator 2\mod\`.
   (ETS2 not from Steam? First run `set ES_ETS2_DIR=<ETS2 folder>` in the same prompt.)
4. Start ETS2 and enable *EuroBeamNG invisible truck* and *EuroBeamNG invisible licence plates* in
   the **Mod Manager**. Make the truck mod again after you change trucks.

## 4. Game settings

- **Both games in windowed mode**: ETS2 *Options → Graphics → Fullscreen* off; BeamNG
  *Options → Graphics → Display mode → Window*.
- BeamNG *Options → Graphics → Reflections → Dynamic Reflections → Off* (required);
  *Lighting Quality* lowest (optional).
- Force feedback off in both games' controls (the bridge drives the wheel). With an H shifter,
  BeamNG's gearbox on manual.
- `bin\wheel.ini`: your wheel's axes and buttons (`bin\wheeltest.exe` shows them).

## 5. Play

1. ETS2: your profile, sit in your truck.
2. BeamNG: *Freeroam → Grid, Small, Pure*, pick a car.
3. Run `start.bat` (or `start_bridge.bat`, then `start_overlay.bat`).

## Had it under the old name (EuroSpeed)?

Delete the old files first, or both versions run at once: `<ETS2>\bin\win_x64\plugins\eurospeed.dll`
(rename its `eurospeed.ini` to `eurobeamng.ini` to keep your settings),
`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\eurospeed`, `...\current\temp\eurospeed`
and `Documents\Euro Truck Simulator 2\mod\eurospeed_invisible_*.scs`. (`install.bat` does this itself.)

## Removing it

Delete `<ETS2>\bin\win_x64\plugins\eurobeamng.dll` and `eurobeamng.ini`, the folder
`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\eurobeamng`, the folder
`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\temp\eurobeamng`, the two `eurobeamng_invisible_*.scs`
in `Documents\Euro Truck Simulator 2\mod\`, and this folder.
