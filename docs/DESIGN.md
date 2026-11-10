# EuroSpeed — Design Doc

> **Historical.** This is the first design, from when Live for Speed was the physics game.
> BeamNG.drive replaced it; the core principle (neither game is rewritten) stayed. See the README.

> Drive the Euro Truck Simulator 2 open map in a Live for Speed car: LFS car physics and vehicle,
> ETS2 world. Same approach as [SkyCraft](https://github.com/chasmlol/SkyCraft).

Status: draft v0.1 · 2026-10-10 · Phase 0 done

## 1. Core principle

**Neither game is rewritten.** LFS runs its own, unmodified car physics (tyres, suspension,
drivetrain, collisions). ETS2 runs its world: map, roads, traffic, weather, rendering. EuroSpeed
only translates:

- ETS2 tells LFS *what the road is shaped like* (Phase 2+).
- LFS tells ETS2 *where the car is* and, later, *what the car looks like*.

If we ever re-implement tyre physics in the ETS2 plugin, the design has gone wrong.

## 2. Target environment

| Thing | Value | Notes |
|---|---|---|
| ETS2 | 1.61.1.1s (telemetry game version 1.20) | SCS SDK 1.14 headers, `third_party/scssdk` |
| LFS | 0.8C28, InSim v10 | OutSim / InSim from `docs\` in the LFS folder |
| Toolchain | MSVC 14.29 (VS 2019 Build Tools), C++17, x64 | `build.bat` |

## 3. Components and the one big asymmetry

SkyCraft has a code loader on both sides (SKSE + Fabric). **LFS has none.** So the "LFS mod" is
an external process that drives LFS through its official interfaces.

```
┌──────── LFS.exe (unmodified) ────────┐   ┌──── lfs_bridge.exe ────┐   ┌──── eurotrucks2.exe ─────────────┐
│ car physics                          │   │                        │   │ eurospeed.dll (SCS SDK plugin)   │
│   OutSim UDP :30000 ─────────────────┼──▶│ parse, anchor, map     │   │                                  │
│   InSim TCP :29999 ◀─────────────────┼───│ (Phase 2: road objects,│   │ telemetry: truck placement ──────┼─┐
│                                      │   │  re-centring)          │   │ TruckPuppet (Phase 1, RE)        │ │
└──────────────────────────────────────┘   └───────────┬────────────┘   └──────────────┬───────────────────┘ │
                                                       └── shared memory Local\EuroSpeed_v1 ──┘◀─────────────┘
```

| SkyCraft | EuroSpeed |
|---|---|
| Minecraft (authoritative player physics) | **LFS** (authoritative car physics) |
| Skyrim (world, renders everything) | **ETS2** (world, renders everything) |
| Fabric mod | `lfs_bridge.exe` (OutSim/InSim, no injection) |
| SKSE plugin | `eurospeed.dll`, official SCS SDK plugin |
| `Local\SkyCraft_v1` | `Local\EuroSpeed_v1` (`protocol/eurospeed_protocol.h`) |
| `fake_skyrim.py` | `fake_ets2.exe` (loads the DLL through the real SDK ABI), `fake_lfs.exe` |

## 4. Coordinate mapping (`protocol/mapping.h`, unit-tested)

- LFS: X east, Y north, Z up; heading radians CCW from north.
- ETS2: X east, Y up, Z south; heading turns CCW from north. Both are metres.
- The worlds share no origin, so the bridge **anchors**: the LFS car's pose at link time is pinned
  to the ETS2 truck's pose. LFS motion relative to the anchor is rotated by the heading
  difference: `ets = anchor + R(dθ)·(Δx, Δy) → (x, Δz, −y)`.
- Pitch matches by convention (+ = nose up). The roll sign is assumed and still needs pinning
  against the real game in Phase 1.

## 5. The road (Phase 2+: CollisionField analogue)

LFS can't load ETS2 geometry. Options, cheapest first:

1. **Flat:** drive on LFS's open **LA1 Layout Square**. ETS2 height comes from ETS2 (puppet
   snaps to ground), and LFS only gives planar motion. Good enough for Phase 1.
2. **Layout objects:** the ETS2 plugin samples road height and slope ahead of the car, and the
   bridge places LFS concrete slabs/ramps via InSim `IS_AXM` (max 60 per packet, single player
   only).
3. **Re-centring:** the LFS area is a few km and ETS2 is a continent. When the car nears the
   edge, the bridge shifts the anchor. Moving the car with `IS_JRR` resets it, so velocity
   handling needs testing (risk §9).

## 6. The vehicle in ETS2

- **Phase 1 TruckPuppet:** set the player vehicle's transform each frame from `EtsTarget`. The
  SDK is read-only, so this needs reverse engineering of `eurotrucks2.exe` 1.61 (pattern scans,
  pinned to the version), done inside the plugin. ETS2's own truck physics must be suppressed
  while puppeted.
- **Phase 3 look:** an ETS2 content mod (`.scs`) defining a car-sized "truck" to puppet. LFS
  car models aren't exportable, so a stand-in body comes first.

## 7. Phased plan

| # | Phase | Done when |
|---|---|---|
| 0 | **Link** ✅ | Plugin loads in real ETS2; bridge ↔ plugin handshake; real LFS OutSim mapped to ETS2 targets anchored on the real truck; mapping unit-tested |
| 1 | **Puppet** | Driving in LFS on LA1 moves the ETS2 truck/camera along the ETS2 road (flat assumption) |
| 2 | **Road shape** | ETS2 road height/slope reaches LFS via layout objects; hills feel right |
| 3 | **Car body** | Car-sized vehicle in ETS2; LFS inputs/dash sound bridged |
| 4 | **Traffic** | ETS2 AI cars become LFS collision objects |

## 8. Multiplayer rules (never touched)

- ETS2: plugin is inert if `multiplayer.time.offset` ≠ 0 (Convoy) or `core_ets2mp.dll` /
  `core_atsmp.dll` is loaded (TruckersMP).
- LFS: bridge only reads OutSim (like a dashboard). Anything that writes to LFS (`lfs_insim`,
  future road objects) refuses when `IS_STA` has `ISS_MULTI` or `IS_ISM` names a host.

## 9. Risks

| Risk | Mitigation |
|---|---|
| ETS2 truck transform needs RE; breaks on game updates | Version-pinned patterns, plugin refuses on unknown versions |
| `IS_JRR` re-centring resets car velocity | Re-centre only at low speed first; measure |
| LFS layout object limits / area bounds | Stream a small window ahead of the car; flat fallback |
| Two games' CPU/GPU cost | LFS can run windowed, low settings |
