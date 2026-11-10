# Third-party parts and credits

| What | Where | Licence |
|---|---|---|
| [TruckLib](https://github.com/sk-zk/TruckLib) by sk-zk: reads ETS2's archives, map and models | road generator (`tools/ets2_roads`, NuGet `TruckLib` 0.5.1) | GPL-2.0, which is why EuroSpeed is GPL-2.0 too |
| SCS Software's Telemetry/Input SDK headers | `third_party/scssdk` | SCS Software's SDK, published for plugin authors |
| .NET runtime (Microsoft) | `bin\ets2_roads` in a release (the generator is published self-contained) | MIT |

## Not included, only used

- **Euro Truck Simulator 2**: the map, models and road data are read from your own installed copy.
  The invisible-truck mod is built on your PC from your game's files. Nothing of the game is
  shipped here.
- **BeamNG.drive**: the ground block's terrain materials use BeamNG's own stock textures by path.

## Ideas

- [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth): an invisible TerrainBlock as the
  drivable ground, whose `.ter` file is swapped as the car moves.
- SkyCraft: one game draws the world while another runs the physics, joined by an overlay.

EuroSpeed is not affiliated with SCS Software or BeamNG GmbH.
