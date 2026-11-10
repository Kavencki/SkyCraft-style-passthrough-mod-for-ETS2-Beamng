# EuroSpeed

Drive a **BeamNG.drive** car in the **Euro Truck Simulator 2** world. BeamNG simulates the car
(soft-body physics, your wheel, its cabin); ETS2 draws the world around it (the map, traffic,
weather). Neither game is rewritten:

- an **ETS2 plugin** (official SCS SDK) moves ETS2's camera to where the BeamNG car is, and keeps
  the (invisible) truck following it so ETS2 loads the map around you;
- a **BeamNG mod** builds invisible ground and roads from ETS2's own map data, so the car drives
  on ETS2's roads, bridges and hills;
- a **bridge** program connects the two games and reads your wheel and pedals;
- an **overlay** draws the BeamNG car's cabin on top of ETS2's picture.

**Single player only.** EuroSpeed does nothing in ETS2 multiplayer (Convoy, TruckersMP) or BeamMP.

## Requirements

- Windows 10/11, 64-bit, and a PC strong enough to run both games at once
- Euro Truck Simulator 2 **1.62** (Steam)
- BeamNG.drive **0.40**
- Optional: a steering wheel (defaults for a Logitech G29) and an H shifter

## Install

1. Download `EuroSpeed-<version>.zip` from [Releases](../../releases) and unzip it anywhere.
2. Start BeamNG.drive once if you never have (it creates its user folder), then close both games.
3. Run **`install.bat`**. It finds ETS2 through Steam and puts in place:
   the ETS2 plugin (`<ETS2>\bin\win_x64\plugins\eurospeed.dll`), the BeamNG mod
   (`%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\mods\unpacked\eurospeed`) and the
   invisible licence plates mod.
4. Start ETS2, load your profile and sit in your truck. ETS2 asks once about
   *advanced SDK features*: answer **OK**. Close ETS2 again.
5. Run **`make_invisible_truck.bat`**: it builds a mod that hides your truck (shadow and lights
   stay), from your own game's files, for the truck you drive. Run it again after you change trucks.
6. In ETS2's **Mod Manager** enable *EuroSpeed invisible truck* and *EuroSpeed invisible licence plates*.

## Play

> **Both games must run in windowed mode** (ETS2: *Options → Graphics → Fullscreen* off; BeamNG:
> *Display mode → Window*, below). In fullscreen the overlay cannot draw over ETS2 or capture
> BeamNG.

1. **ETS2**: windowed mode (**required**). Load your profile and sit in your truck (parked).
2. **BeamNG.drive** settings (*Options → Graphics*), once:

   | Setting | Value | |
   |---|---|---|
   | Display mode | **Window** | **required**: in Fullscreen BeamNG minimizes itself when you click into ETS2, and the overlay has nothing to show |
   | Reflections → Dynamic Reflections | **Off** | **required**: otherwise the overlay does not show the car (it reflects the magenta key colour) |
   | Lighting Quality | lowest | optional: faster, the overlay only needs the cabin |

   Then *Freeroam → Grid, Small, Pure* (a level without its own terrain), pick a car.
   Turn force feedback off in BeamNG's and ETS2's controls (the bridge drives the wheel); with an
   H shifter set BeamNG's gearbox to manual.
3. Run **`start.bat`**: the bridge and then the overlay start in their own windows. The first time
   in an area the bridge makes the roads around you (about half a minute), then it puts the car
   on the road next to your truck.
4. Drive. While ETS2 has focus your keyboard goes to BeamNG (Esc = BeamNG's menu, your BeamNG
   key bindings); **F12** switches it to ETS2 and back (e.g. for ETS2's map). ETS2's GPS is shown
   on top of the car (Ctrl+Shift+G hides it).
5. In the bridge window: **R** puts the car back on the road by the truck, **Q** quits.

Fuel: the truck's fuel follows the car's, and refuelling at an ETS2 station fills the car. The
plugin looks for ETS2's fuel value at the start; if it says *fuel value was not found* in ETS2's
log, the link is off for that session.

## Settings

| File | What |
|---|---|
| `bin\wheel.ini` | `[wheel]` axes, force feedback, H shifter buttons (`bin\wheeltest.exe` shows them) · `[keys]` keyboard routing · `[view]` car height in ETS2's picture · `[ground]` invisible terrain on/off · `[time]` ETS2's sunrise/sunset and the key colour's glow |
| `bin\overlay.ini` | `[gps]` where ETS2's navigation is taken from and shown |
| `<ETS2>\bin\win_x64\plugins\eurospeed.ini` | `[camera]` field of view · `[truck]` the following truck · `[fuel]` fuel link |

Restart the bridge (or ETS2, for the plugin's file) after a change.

## When something is off

- Logs: `bin\bridge.log`, ETS2's `Documents\Euro Truck Simulator 2\game.log.txt` (lines with
  `[EuroSpeed]`), BeamNG's `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\beamng.log`.
- **After an ETS2 update** the plugin finds the game code it needs by itself in most cases. If
  ETS2's camera no longer follows the car (`camera driver unavailable` in ETS2's log), run
  `bin\codescan_test.exe "<ETS2>\bin\win_x64\eurotrucks2.exe"` and open an issue with its output.
- **Very fast driving**: the roads ahead are made while you drive. At 200+ km/h you can catch up
  with them; the bridge warns *only N m of generated road left ahead - slow down*.
- To remove everything: `uninstall.bat`.

## Building from source

- C++ (bridge, plugin, overlay, tools): `build.bat` with Visual Studio 2019+ Build Tools (C++).
- Road generator (C#): .NET 10 SDK, `dotnet build tools\ets2_roads -c Release -o build\ets2_roads`.
- A release zip: `powershell -ExecutionPolicy Bypass -File make_release.ps1 -Version x.y.z`.
- `docs/DESIGN.md` is the original design (from when Live for Speed was the physics game).

## Licence

GPL-2.0 (see `LICENSE`): the road generator uses [TruckLib](https://github.com/sk-zk/TruckLib),
which is GPL-2.0. Credits and third-party parts: `THIRD_PARTY.md`. Not affiliated with SCS
Software or BeamNG GmbH.

---

## Lietuviškai

**EuroSpeed**: BeamNG.drive mašina ETS2 pasaulyje. BeamNG skaičiuoja mašiną, ETS2 piešia
pasaulį. Tik vieno žaidėjo režimas (multiplayer'yje nieko nedaro).

**Įdiegimas:** išarchyvuok zip, uždaryk abu žaidimus, paleisk `install.bat`. Tada įjunk ETS2, sėsk
į furą (į klausimą apie *advanced SDK features* atsakyk OK), uždaryk ETS2 ir paleisk
`make_invisible_truck.bat`. ETS2 Mod Manager'yje įjunk abu *EuroSpeed invisible* mod'us.

**Abu žaidimai būtinai lange (windowed mode):** ETS2 *Options → Graphics → Fullscreen* išjungtas,
BeamNG *Display mode → Window*. Per visą ekraną overlay negali nei piešti ant ETS2, nei filmuoti BeamNG.

**BeamNG nustatymai** (*Options → Graphics*), vieną kartą:
- *Display mode* → **Window** (būtina: Fullscreen pats susimažina paspaudus ETS2, overlay nieko nerodo);
- *Reflections → Dynamic Reflections* → **Off** (būtina: kitaip overlay nerodo mašinos);
- *Lighting Quality* → žemiausia (nebūtina, bet greičiau).

**Žaidimas:** ETS2 lange, sėdi furoje; BeamNG lange: *Freeroam → Grid, Small, Pure* su
mašina; tada `start.bat`. F12 perjungia klaviatūrą tarp BeamNG ir ETS2. Nustatymai:
`bin\wheel.ini`, `bin\overlay.ini`.
