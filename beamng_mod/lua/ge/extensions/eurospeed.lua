-- EuroSpeed: the BeamNG half. BeamNG simulates the car; ETS2 draws the world.
-- Every frame this sends the player camera and vehicle pose to lfs_bridge.exe
-- (UDP 127.0.0.1:30001). From the bridge (UDP :30002) it receives ETS2 road strips,
-- which become solid MeshRoads so the car drives on the real ETS2 road shape, and
-- a one-time teleport onto them. Inert while any multiplayer session is active.
--
-- Packet (text, one line):
--   ESB1 seq tMs  camPos(3) camFwd(3) camUp(3) fovDeg  vehPos(3) vehFwd(3) vehUp(3) speed
-- BeamNG world: X east, Y north, Z up, metres.

local M = {}
local logTag = 'eurospeed'

local host, port = '127.0.0.1', 30001
local inputPort = 30002            -- ETS2 driver inputs from lfs_bridge.exe
local udp = nil
local udpIn = nil
local seq = 0
local inert = false

-- Steering filter for forwarded input: 2 = direct (wheels), 1 = pad smoothing, 0 = keyboard smoothing.
local steerFilter = 1
local lastBridgeMsg = 0            -- when anything last arrived from the bridge (ms)
local lastClockMin = nil            -- ETS2 clock last applied (minutes after midnight)
local wantKeyNits = nil            -- key glow for the time of day (nits), from the bridge
local lastInput = nil              -- { steer, throttle, brake, clutch, t, parkingBrake, gear }
local shifterGear = nil            -- the bridge's H shifter gear, nil = no shifter
-- Fuel link (ETS2 truck <-> this car): the car's fuel tanks, read in the vehicle's Lua.
-- Routed keys: the bridge catches the keyboard while ETS2 has focus and sends it here; the
-- actions the player's BeamNG keyboard bindings give that key are triggered, as a real key
-- press would (BeamNG itself only reads the keyboard while its window has focus).
local keyActions = {}   -- base key -> actions triggered on its press (released with it)
local menuOpenSent, menuSentAt = nil, 0
local function controlParts(control)
  local mods, key = {}, nil
  for tok in control:gsub('-', ' '):gmatch('%S+') do
    if tok == 'ctrl' or tok == 'alt' or tok == 'shift' then mods[tok] = true else key = tok end
  end
  return mods, key
end
local function sameMods(a, b)
  return (a.ctrl or false) == (b.ctrl or false) and (a.alt or false) == (b.alt or false) and (a.shift or false) == (b.shift or false)
end
local function onRoutedKey(down, control)
  local mods, key = controlParts(control)
  if not key then return end
  local now = os.clockhp()
  if not down then
    for _, action in ipairs(keyActions[key] or {}) do ActionMap.triggerBindingByNameDigital(action, false, now) end
    keyActions[key] = nil
    return
  end
  -- Which actions apply now: menu navigation only in a menu, the big map's only on it.
  local menu = core_input_bindings and core_input_bindings.isMenuActive
  local bigMap = freeroam_bigMapMode and freeroam_bigMapMode.bigMapActive and freeroam_bigMapMode.bigMapActive()
  local defs = core_input_actions and core_input_actions.getActiveActions() or {}
  -- The world editor's actions (Esc, Q/E, Shift...) only while it is open: their code needs it
  -- ("attempt to index field 'editModes'"), as with a real key press, whose editor maps are off then.
  local editorOn = editor and editor.isEditorActive and editor.isEditorActive() or false
  local filter = core_input_actionFilter
  local fired, seen = {}, {}
  for _, device in ipairs(core_input_bindings and core_input_bindings.bindings or {}) do
    if device.devname and device.devname:find('^keyboard') and device.contents then
      for _, b in ipairs(device.contents.bindings or {}) do
        if b.control and b.action and not seen[b.action] and not b.unused then
          local bm, bk = controlParts(b.control)
          if bk == key and sameMods(bm, mods) then
            local def = defs[b.action]
            local map = def and def.actionMap
            local ok
            if def and def.cat == 'editor' and not editorOn then ok = false
            elseif def and def.cat == 'flowgraph' and not editor_flowgraphEditor then ok = false   -- (its editor)
            elseif filter and filter.isActionBlocked and filter.isActionBlocked(b.action) then ok = false
            elseif map == 'MenuIndependent' then ok = menu
            elseif map == 'BigMap' then ok = bigMap
            elseif map == 'First' then ok = true
            else ok = not menu end
            if ok then
              seen[b.action] = true
              ActionMap.triggerBindingByNameDigital(b.action, true, now)
              table.insert(fired, b.action)
            end
          end
        end
      end
    end
  end
  keyActions[key] = fired
end

local fuelLitres, fuelCapacity, fuelAt, fuelPolledAt, fuelSentAt = nil, 0, 0, 0, 0
local refuelLogged, refuelLoggedAt = false, 0
local fuelPoll = [[
local l, c = 0, 0
for _, s in pairs((energyStorage and energyStorage.getStorages and energyStorage.getStorages()) or {}) do
  if s.type == 'fuelTank' and (s.capacity or 0) > 0 then
    c = c + s.capacity
    l = l + (s.remainingVolume or ((s.remainingRatio or 0) * s.capacity))
  end
end
if c > 0 then obj:queueGameEngineLua(string.format('if eurospeed then eurospeed.onFuel(%f, %f) end', l, c)) end
]]
-- Fills the car's fuel tanks to `litres` in total (same ratio in every tank).
local fuelSet = [[
local want = %f
local c = 0
local tanks = {}
for _, s in pairs((energyStorage and energyStorage.getStorages and energyStorage.getStorages()) or {}) do
  if s.type == 'fuelTank' and (s.capacity or 0) > 0 then c = c + s.capacity table.insert(tanks, s) end
end
if c > 0 then
  local r = math.max(0, math.min(1, want / c))
  for _, s in ipairs(tanks) do
    if s.setRemainingRatio then s:setRemainingRatio(r)
    elseif s.setRemainingVolume then s:setRemainingVolume(r * s.capacity) end
  end
end
]]
local lastShifterGear = nil
local inputActive = false
local lastPark = nil

-- Returns a reason string when a multiplayer session is active, else nil.
local function multiplayerReason()
  -- BeamMP's mod is loaded even offline; ask it whether we're on a server.
  if MPCoreNetwork then
    local okA, a = pcall(MPCoreNetwork.isMPSession)
    local okB, b = pcall(MPCoreNetwork.isGoingMPSession)
    if (okA and a) or (okB and b) then return 'BeamMP' end
  end
  if multiplayer_sessionManager and multiplayer_sessionManager.getCurrentSession then
    -- Truthy = in a session (same check BeamNG's own gameplay code uses; offline returns false/nil).
    local ok, s = pcall(multiplayer_sessionManager.getCurrentSession)
    if ok and s then return 'BeamNG session' end
  end
  return nil
end

local function openSocket()
  if udp then return true end
  if not socket or not socket.udp then
    log('E', logTag, 'LuaSocket not available')
    return false
  end
  udp = socket.udp()
  udp:settimeout(0)
  udp:setpeername(host, port)
  log('I', logTag, 'streaming pose to ' .. host .. ':' .. port)
  return true
end

local function openInput()
  if udpIn then return true end
  if not socket or not socket.udp then return false end
  udpIn = socket.udp()
  udpIn:settimeout(0)
  if udpIn:setsockname(host, inputPort) == nil then
    log('E', logTag, 'cannot listen for ETS2 input on ' .. inputPort)
    udpIn:close()
    udpIn = nil
    return false
  end
  log('I', logTag, 'listening for ETS2 driver input on ' .. host .. ':' .. inputPort)
  return true
end

---------------------------------------------------------------------------
-- CollisionField: ETS2 roads as solid MeshRoads (sent by the bridge).
local roads = {}          -- id -> MeshRoad object
local pendingRoads = {}   -- queue of { id, width, points } to build
local pendingTeleport = nil
local roadsBuilt = 0
local maxBuildsPerFrame = 1   -- one tile per frame: a first-time import (COLLADA) is spread over frames
-- Vehicles only collide with static geometry after be:reloadCollision().
local collisionDirty = false
local lastCollisionReload = 0
local dirtySince = 0
local ackIds = {}          -- roads received, to confirm to the bridge (it resends lost ones)

local function deleteRoad(id)
  local obj = roads[id]
  if obj then
    obj:delete()
    roads[id] = nil
    -- No collision rebuild for a removal: a rebuild costs a frame hitch for every loaded tile,
    -- and a tile is only removed far behind the car (its collision goes with the next rebuild).
  end
end

-- One collision rebuild per batch of new tiles: once no tile has come for 1.5 s (tiles arrive
-- 600+ m ahead of the car, so there is time), or after 6 s at the latest.
local lastBuildAt = 0
local function reloadCollisionIfNeeded()
  local now = Engine.Platform.getSystemTimeMS()
  if not collisionDirty then return end
  if dirtySince == 0 then dirtySince = now end
  if (#pendingRoads == 0 and now - lastBuildAt > 1500) or now - dirtySince > 6000 then
    be:reloadCollision()
    collisionDirty = false
    lastCollisionReload = now
    dirtySince = 0
  end
end

local function sendAcks()
  if #ackIds == 0 or not udp then return end
  for i = 1, #ackIds, 100 do
    udp:send('ESA1 ' .. table.concat(ackIds, ' ', i, math.min(i + 99, #ackIds)))
  end
  ackIds = {}
end

local function clearRoads()
  for id in pairs(roads) do deleteRoad(id) end
  pendingRoads = {}
  log('I', logTag, 'cleared ETS2 roads')
end

local function parseRoad(msg)
  -- ESR1 id kind width n x y z x y z ...
  local it = msg:gmatch('%S+')
  it() -- tag
  local id, kind, width, n = tonumber(it()), tonumber(it()), tonumber(it()), tonumber(it())
  if not (id and width and n) then return nil end
  local pts = {}
  for i = 1, n do
    local x, y, z = tonumber(it()), tonumber(it()), tonumber(it())
    if not z then return nil end
    pts[i] = vec3(x, y, z)
  end
  return { id = id, kind = kind, width = width, points = pts }
end

local function buildRoad(r)
  if roads[r.id] or #r.points < 2 then return end
  local road = createObject('MeshRoad')
  road:registerObject('eurospeed_road_' .. r.id)
  if scenetree.MissionGroup then scenetree.MissionGroup:add(road) end
  road:setPosition(r.points[1])
  local up = vec3(0, 0, 1)
  -- Road: 1.5 m thick slab under the surface. Barrier (kind 2): points are the wall top,
  -- 1.6 m deep, so it stands 1 m above the road and reaches 0.6 m below it.
  local depth = (r.kind == 2) and 1.6 or 1.5
  for i, p in ipairs(r.points) do
    road:insertNode(p, r.width, depth, up, i - 1)
  end
  roads[r.id] = road
  roadsBuilt = roadsBuilt + 1
  collisionDirty = true
  lastBuildAt = Engine.Platform.getSystemTimeMS()
end

-- A road mesh tile (.dae written by ets2_roads) as one static object with collision.
-- The bridge sends the yaw and a check point: tile-local (100,0,0) must land there.
local tileYawSign = nil   -- learned once: +1 or -1 (Torque's rotation field sign)

local function placeTile(obj, t, sign)
  obj:setPosition(t.pos)
  -- The bridge aligns BeamNG north with ETS2 north, so tiles normally need no rotation.
  if math.abs(t.yaw) > 1e-4 then
    obj:setField('rotation', 0, string.format('0 0 1 %f', math.deg(t.yaw * sign)))
  end
end

local function tileError(obj, t)
  local ok, w = pcall(function() return obj:getTransform():mulP(vec3(100, 0, 0)) end)
  if not ok or not w then return nil end
  return (w - t.check):length()
end

local function buildTile(t)
  if roads[t.id] then return end
  local obj = createObject('TSStatic')
  obj:setField('shapeName', 0, t.path)
  obj:setField('collisionType', 0, 'Visible Mesh Final')
  obj.canSave = false
  placeTile(obj, t, tileYawSign or 1)
  obj:registerObject('eurospeed_tile_' .. t.id)
  if scenetree.MissionGroup then scenetree.MissionGroup:add(obj) end
  if not tileYawSign and math.abs(t.yaw) > 1e-4 then
    -- First tile: find the rotation sign that puts the check point where the bridge expects.
    local e1 = tileError(obj, t)
    if e1 and e1 > 1 then
      placeTile(obj, t, -1)
      local e2 = tileError(obj, t)
      tileYawSign = (e2 and e2 < e1) and -1 or 1
      if tileYawSign == 1 then placeTile(obj, t, 1) end
      log('I', logTag, string.format('tile rotation sign %d (check errors %.2f / %s)', tileYawSign, e1, tostring(e2)))
    else
      tileYawSign = 1
      log('I', logTag, 'tile rotation sign 1 (check error ' .. tostring(e1) .. ')')
    end
  end
  roads[t.id] = obj
  roadsBuilt = roadsBuilt + 1
  collisionDirty = true
  lastBuildAt = Engine.Platform.getSystemTimeMS()
end

-- The key colours: es_key (the box around the car) and es_road (the road tiles), both glowing
-- magenta. BeamNG does not pick up the mod's materials file by itself here ("NO-MATERIAL es_key"):
-- without it they are only lit magenta, so they turned orange at dusk and black at night. Loaded
-- before the first tile: a tile built without them keeps the plain colour.
local keyMaterialsLoaded = false
local function loadKeyMaterials()
  if keyMaterialsLoaded then return end
  keyMaterialsLoaded = true
  local ok, err = pcall(loadJsonMaterialsFile, '/art/eurospeed/main.materials.json')
  log(ok and 'I' or 'E', logTag, ok and 'key colour materials loaded' or ('key colour materials: ' .. tostring(err)))
end

-- The key glow for the time of day (sent with ETS2's clock): less in the dark, so BeamNG's auto
-- exposure (which follows the bright box) opens up for the cabin; the box looks the same.
local keyNitsApplied = nil
local function applyKeyNits()
  if not wantKeyNits or not keyMaterialsLoaded then return end
  if keyNitsApplied and math.abs(wantKeyNits - keyNitsApplied) < keyNitsApplied * 0.04 then return end
  for _, name in ipairs({ 'es_key', 'es_road' }) do
    local m = scenetree.findObject(name)
    if m then
      m:setField('emissiveIntensityNits', 0, string.format('%.1f', wantKeyNits))
      m:flush()
      m:reload()
    end
  end
  if not keyNitsApplied or math.abs(wantKeyNits - keyNitsApplied) > keyNitsApplied * 0.5 then
    log('I', logTag, string.format('key glow %.0f nits', wantKeyNits))
  end
  keyNitsApplied = wantKeyNits
end

local function buildPending()
  if #pendingRoads > 0 then loadKeyMaterials() end
  local n = 0
  while n < maxBuildsPerFrame and #pendingRoads > 0 do
    local r = table.remove(pendingRoads, 1)
    local ok, err = pcall(r.path and buildTile or buildRoad, r)
    if not ok then log('E', logTag, 'road ' .. tostring(r.id) .. ': ' .. tostring(err)) end
    n = n + 1
  end
end

---------------------------------------------------------------------------
-- Ground: the drivable ground around the car as one TerrainBlock that is not drawn (the way
-- BeamNG.Earth does it). The bridge writes BeamNG's terrain file (.ter) of the 1 km around the car
-- and sends "ESTR path x y z maxHeight square seq"; swapping it in (terrainFile + postApply) takes
-- ~10 ms and its collision follows with no collision rebuild - unlike mesh tiles, whose every change
-- rebuilt all static collision (a hitch). Only bridges, walls and such stay mesh tiles.
-- A level can hold one TerrainBlock: in a level with its own terrain the bridge sends full mesh tiles.
local groundName = 'eurospeed_ground'
local groundDir = '/art/eurospeed/terrain/'
local groundSeq = 0           -- the bridge's number of the block in use (0 = none)
local groundFailed = false    -- the block could not be made: mesh tiles instead
local groundNext = nil        -- the latest block sent, not applied yet
local groundSentAt = 0
local groundErrSeq = nil

local function otherTerrain()
  for _, name in ipairs(scenetree.findClassObjects('TerrainBlock') or {}) do
    if name ~= groundName then return true end
  end
  return false
end

-- Creation mirrors BeamNG's util/terrainGenerator.lua (TerrainBlock(), setName, registerObject,
-- setTerrFileLvlFolder, MissionGroup:addObject, addMaterial / updateMaterial, then importMaps).
local function groundBlock(maxH)
  local tb = scenetree.findObject(groundName)
  if tb then return tb, false end
  if not scenetree.findObject('eurospeed_asphalt') or not scenetree.findObject('eurospeed_TerrainMaterialTextureSet') then
    loadJsonMaterialsFile(groundDir .. 'main.materials.json')
  end
  tb = TerrainBlock()
  tb:setName(groundName)
  tb:setField('baseTexSize', 0, '1024')
  tb:setField('castShadows', 0, '0')
  tb:registerObject(groundName)
  tb.canSave = false
  tb:setTerrFileLvlFolder('/temp/eurospeed/')
  tb:setField('materialTextureSet', 0, 'eurospeed_TerrainMaterialTextureSet')
  scenetree.MissionGroup:addObject(tb)
  -- The .ter files name these two (layer 0 road surfaces, 1 the rest).
  local mats = { 'eurospeed_asphalt', 'eurospeed_grass' }
  for i, m in ipairs(mats) do
    tb:addMaterial(m, -1)
    tb:updateMaterial(i - 1, m)
  end
  local maps = {
    { path = groundDir .. 'ground_layer_1024.png', selected = false, material = mats[1], materialId = 0, channel = 'R', channelId = 0 },
    { path = groundDir .. 'ground_none_1024.png', selected = false, material = mats[2], materialId = 1, channel = 'R', channelId = 0 },
  }
  if tb:importMaps(groundDir .. 'ground_flat_1024.png', 1, maxH, '', mats, maps, false) == false then
    error('importMaps returned false')
  end
  return tb, true
end

local function applyGround(g)
  if not FS:fileExists(g.path) then error('no file ' .. g.path) end
  -- (sent before the bridge heard that this level has a terrain: never a second block)
  if not scenetree.findObject(groundName) and otherTerrain() then error('this level has its own terrain') end
  local t0 = Engine.Platform.getSystemTimeMS()
  local tb, created = groundBlock(g.maxH)
  tb:setField('maxHeight', 0, tostring(g.maxH))
  tb:setPosition(g.pos)
  tb:setField('terrainFile', 0, g.path)
  tb:postApply()
  if tb.setRenderEnabled then tb:setRenderEnabled(false) end
  -- A new block's collision: one rebuild. Swaps after that need none.
  if created then be:reloadCollision() end
  groundSeq = g.seq
  if created or g.seq % 20 == 0 then
    log('I', logTag, string.format('ground block %d in %d ms%s', g.seq, Engine.Platform.getSystemTimeMS() - t0,
                                   created and ' (created)' or ''))
  end
end

local function updateGround()
  if groundNext and scenetree.MissionGroup then   -- (a level is loaded)
    local g = groundNext
    groundNext = nil
    local hadBlock = scenetree.findObject(groundName) ~= nil
    local ok, err = pcall(applyGround, g)
    if not ok then
      if not hadBlock then
        -- The block could not be made: none at all, the bridge sends full mesh tiles.
        groundFailed = true
        local tb = scenetree.findObject(groundName)
        if tb then pcall(function() tb:delete() end) end
        log('E', logTag, 'ground block: ' .. tostring(err) .. ' - roads as meshes')
      elseif groundErrSeq ~= g.seq then
        groundErrSeq = g.seq
        log('W', logTag, 'ground block ' .. g.seq .. ': ' .. tostring(err))
      end
    end
  end
  -- The level went (or changed): its block with it.
  if groundSeq > 0 and not scenetree.findObject(groundName) then groundSeq = 0 end
  local now = Engine.Platform.getSystemTimeMS()
  if now - groundSentAt > 2000 then
    groundSentAt = now
    local capable = not groundFailed and scenetree.MissionGroup ~= nil and not otherTerrain()
    udp:send(string.format('ESTM %d %d', capable and 1 or 0, groundSeq))
  end
end

local function clearGround()
  local tb = scenetree.findObject(groundName)
  if tb then tb:delete() end
  groundSeq = 0
  groundNext = nil
end

-- A new level: it may take the block (again).
local function onClientStartMission()
  groundFailed = false
  groundSeq = 0
  groundErrSeq = nil
  groundSentAt = 0
end

local function teleportCar(veh, t)
  -- heading: radians CCW from north (BeamNG +Y). quatFromDir alone faces the car along fwd
  -- (the extra 180° turn used in BeamNG's AI tests put the car backwards - checked in game).
  local fwd = vec3(-math.sin(t.h), math.cos(t.h), 0)
  local rot = quatFromDir(fwd, vec3(0, 0, 1))
  spawn.safeTeleport(veh, t.pos, rot)
  log('I', logTag, string.format('car placed on ETS2 road at (%.1f, %.1f, %.1f)', t.pos.x, t.pos.y, t.pos.z))
end

-- Drain the socket: driver input, roads, teleports.
local function pollInput()
  if not openInput() then return end
  while true do
    local msg = udpIn:receive(65507)
    if not msg then break end
    lastBridgeMsg = Engine.Platform.getSystemTimeMS()
    local tag = msg:sub(1, 4)
    if tag == 'ESI1' then
      local s, steer, thr, brk, clu, park, gear =
        msg:match('^ESI1 (%d+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) (%d+) ([%-%d]+)')
      -- Optional 8th field: 1 = a real wheel is steering (no smoothing), 0 = keyboard.
      local wheelFlag = msg:match('^ESI1 %d+ %S+ %S+ %S+ %S+ %d+ [%-%d]+ (%d)')
      steerFilter = (wheelFlag == '1') and 2 or 1
      -- Optional 9th field: the H shifter's gear (-1 reverse, 0 neutral, 1..7), -9 = no shifter.
      local sg = tonumber(msg:match('^ESI1 %d+ %S+ %S+ %S+ %S+ %d+ [%-%d]+ %d ([%-%d]+)'))
      shifterGear = (sg and sg > -9) and sg or nil
      if s then
        lastInput = { tonumber(steer), tonumber(thr), tonumber(brk), tonumber(clu), Engine.Platform.getSystemTimeMS(),
                      tonumber(park), tonumber(gear) }
      end
    elseif tag == 'ESR1' then
      local r = parseRoad(msg)
      if r then
        table.insert(pendingRoads, r)
        table.insert(ackIds, r.id)
      end
    elseif tag == 'ESG1' then
      local id, path, px, py, pz, yaw, cx, cy, cz =
        msg:match('^ESG1 (%d+) (%S+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+)')
      if id then
        table.insert(pendingRoads, { id = tonumber(id), path = path, pos = vec3(tonumber(px), tonumber(py), tonumber(pz)),
                                     yaw = tonumber(yaw), check = vec3(tonumber(cx), tonumber(cy), tonumber(cz)) })
        table.insert(ackIds, tonumber(id))
      end
    elseif tag == 'ESD1' then
      local id = tonumber(msg:match('^ESD1 (%d+)'))
      if id then deleteRoad(id) end
    elseif tag == 'ESTR' then
      -- The ground block: its .ter, south-west corner, height range, square size, number.
      local path, x, y, z, maxH, sq, s =
        msg:match('^ESTR (%S+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%d%.]+) ([%d%.]+) (%d+)')
      if s and tonumber(s) ~= groundSeq and not groundFailed then
        if tonumber(sq) ~= 1 then
          log('E', logTag, 'ground block square size ' .. sq .. ' (the block is made with 1)')
        else
          groundNext = { path = path, pos = vec3(tonumber(x), tonumber(y), tonumber(z)), maxH = tonumber(maxH),
                         seq = tonumber(s) }
        end
      end
    elseif tag == 'ESKY' then
      -- A key pressed while ETS2 has focus, routed to BeamNG by the bridge: "ESKY 1 ctrl r".
      local down, control = msg:match('^ESKY (%d) (.+)$')
      if down then pcall(onRoutedKey, down == '1', control) end
    elseif tag == 'ESKM' then
      -- The bridge switched where the keyboard goes (shown in BeamNG's UI, seen over ETS2).
      local mode = msg:match('^ESKM (%S+)')
      if mode then
        pcall(ui_message, mode == 'beamng' and 'Keyboard -> BeamNG' or 'Keyboard -> ETS2', 2, 'eurospeed_keys')
      end
    elseif tag == 'ESK1' then
      -- Refuelling at an ETS2 station: the car's tanks get this many litres.
      local litres = tonumber(msg:match('^ESK1 ([%d%.]+)'))
      local veh = getPlayerVehicle(0)
      if litres and veh then
        veh:queueLuaCommand(string.format(fuelSet, litres))
        if not refuelLogged then log('I', logTag, 'ETS2 refuel: filling the car') refuelLogged = true end
        refuelLoggedAt = Engine.Platform.getSystemTimeMS()
      end
    elseif tag == 'ESX1' then
      clearRoads()
    elseif tag == 'ESC1' then
      -- ETS2's clock (minutes after midnight, mapped by the bridge onto BeamNG's sunrise 06:00 /
      -- sunset 18:00): BeamNG's time of day follows it, clock stopped. Then the key glow (nits)
      -- for this time of day.
      local m = tonumber(msg:match('^ESC1 (%d+)'))
      local nits = tonumber(msg:match('^ESC1 %d+ ([%d%.]+)'))
      if nits and nits > 0 then wantKeyNits = nits end
      if m and m ~= lastClockMin then
        lastClockMin = m
        local t = (m / 1440 - 0.5) % 1   -- BeamNG: 0 = noon, 0.5 = midnight
        if core_solarTimeOfDay and core_solarTimeOfDay.timeFromMinutes then
          local ok, v = pcall(core_solarTimeOfDay.timeFromMinutes, m)
          if ok and v then t = v end
        end
        local ok, err = pcall(core_environment.setTimeOfDay, { time = t, play = false })
        if not ok then log('E', logTag, 'time of day: ' .. tostring(err)) end
      end
    elseif tag == 'EST1' then
      local x, y, z, h = msg:match('^EST1 ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+) ([%-%d%.]+)')
      if h then pendingTeleport = { pos = vec3(tonumber(x), tonumber(y), tonumber(z)), h = tonumber(h) } end
    end
  end
end

---------------------------------------------------------------------------
-- Force feedback: BeamNG only computes it for a force-feedback device it drives itself, and it
-- loses the wheel when its window is not focused. Its "virtual wheel" mode computes the same
-- force and hands it to our function instead; the bridge applies it to the real wheel.
local ffbVehId = nil          -- vehicle with the virtual wheel on
local ffbValue, ffbAt = 0, 0  -- latest force (-1..1) and when it arrived (ms)

local virtualWheelOn = [[
esSteer = esSteer or 0
local esLastFfb = 0
local clk = os.clockhp or os.clock
hydros.enableVirtualWheel(true, function() return esSteer end, function(o, id, torque)
  local t = clk()
  if t - esLastFfb >= 0.01 then
    esLastFfb = t
    local lim = hydros.wheelFFBForceLimit
    if not lim or lim <= 0 then lim = 1 end
    obj:queueGameEngineLua(string.format('if eurospeed then eurospeed.onFFB(%.4f) end', torque / lim))
  end
end)
]]

local function onFFB(v)
  ffbValue = v
  ffbAt = Engine.Platform.getSystemTimeMS()
end

-- Virtual wheel only while a real wheel steers (keyboard steering keeps BeamNG's smoothing).
local function updateFFB(veh, wheelSteering)
  local id = veh and veh:getId() or nil
  local want = wheelSteering and id or nil
  if want ~= ffbVehId then
    if ffbVehId then
      local old = be:getObjectByID(ffbVehId)
      if old then old:queueLuaCommand('hydros.enableVirtualWheel(false)') end
    end
    if want then veh:queueLuaCommand(virtualWheelOn) end
    ffbVehId = want
    log('I', logTag, want and 'force feedback: BeamNG force goes to the bridge' or 'force feedback: off')
  end
end

local function applyInput(veh)
  local fresh = lastInput and (Engine.Platform.getSystemTimeMS() - lastInput[5]) < 500
  pcall(updateFFB, veh, fresh and steerFilter == 2)
  if fresh then
    -- ETS2 steering is counterclockwise (+ = left); BeamNG's is + = right. esSteer feeds the
    -- virtual wheel (force feedback mode), input.event the normal path.
    veh:queueLuaCommand(string.format(
      'esSteer = %.4f input.event("steering", %.4f, %d) input.event("throttle", %.4f, 2) input.event("brake", %.4f, 2) input.event("clutch", %.4f, 2)',
      -lastInput[1], -lastInput[1], steerFilter, lastInput[2], lastInput[3], lastInput[4]))
    -- Parking brake mirrors ETS2's. Gears are NOT forwarded: ETS2's truck gearbox (even with
    -- the engine off) changes gears on its own, so BeamNG keeps its own original gearbox.
    local park = lastInput[6]
    if park ~= nil and park ~= lastPark then
      veh:queueLuaCommand(string.format('input.event("parkingbrake", %d, 2)', park))
      lastPark = park
    end
    -- H shifter (read by the bridge): the gear goes to BeamNG's gearbox as its own H-shifter
    -- bindings do. Needs a manual gearbox (BeamNG's realistic/manual shifting mode).
    if shifterGear ~= nil and shifterGear ~= lastShifterGear then
      veh:queueLuaCommand(string.format(
        'if controller and controller.mainController and controller.mainController.shiftToGearIndex then controller.mainController.shiftToGearIndex(%d) end',
        shifterGear))
      if lastShifterGear == nil then log('I', logTag, 'H shifter drives the gearbox') end
      lastShifterGear = shifterGear
    end
    if not inputActive then log('I', logTag, 'driving from ETS2 input') end
    inputActive = true
  elseif inputActive then
    veh:queueLuaCommand('input.event("throttle", 0, 2) input.event("brake", 0, 2) input.event("steering", 0, 2) input.event("clutch", 0, 2)')
    log('I', logTag, 'ETS2 input stopped: released controls')
    lastShifterGear = nil
    inputActive = false
  end
end

---------------------------------------------------------------------------
-- Key box: everything around the car renders as one key colour (magenta), so the
-- EuroSpeed overlay can show just the car (cabin or whole car) on top of ETS2.
local keyBox = nil

local function setScaleCompat(obj, s)
  if not pcall(function() obj:setScale(s) end) then obj.scale = s end
end

-- Fog tints the key colour (pale pink instead of magenta), so it is off while the key box is.
local savedFog = nil
local function setFogOff(off)
  local li = scenetree.findObject('theLevelInfo')
  if not li then return end
  if off then
    if savedFog == nil then savedFog = li:getField('fogDensity', 0) or '0' end
    li:setField('fogDensity', 0, '0')
  elseif savedFog ~= nil then
    li:setField('fogDensity', 0, savedFog)
    savedFog = nil
  end
  pcall(function() li:postApply() end)
end

-- BeamNG mutes itself when its window loses focus; with EuroSpeed the driver sits in ETS2's
-- window, so keep the car's sound playing (a normal BeamNG setting, also in Options > Audio).
local function keepAudioInBackground()
  if settings.getValue('AudioMuteOnWindowLoseFocus') ~= false then
    settings.setValue('AudioMuteOnWindowLoseFocus', false)
    log('I', logTag, 'sound keeps playing while ETS2 has focus (AudioMuteOnWindowLoseFocus off)')
  end
end

-- Bloom and the level's colour grading wash the key colour out to pale pink: off with the key box.
local savedGrade = nil
local function setPostFxForKey(on)
  local bloom = scenetree.findObject('PostEffectBloomObject')
  if bloom then pcall(function() if on then bloom:disable() else bloom:enable() end end) end
  local combine = scenetree.findObject('PostEffectCombinePassObject')
  if combine then
    if on then
      if savedGrade == nil then savedGrade = combine.colorCorrectionStrength or 1 end
      combine.colorCorrectionStrength = 0
    elseif savedGrade ~= nil then
      combine.colorCorrectionStrength = savedGrade
      savedGrade = nil
    end
  end
end

local function updateKeyBox(veh)
  local want = veh ~= nil and (Engine.Platform.getSystemTimeMS() - lastBridgeMsg) < 3000
  if not want then
    if keyBox then keyBox:delete() keyBox = nil pcall(setFogOff, false) pcall(setPostFxForKey, false) end
    return
  end
  if not keyBox then loadKeyMaterials() end
  if not keyBox then pcall(setFogOff, true) pcall(setPostFxForKey, true) pcall(keepAudioInBackground) end
  if not keyBox then
    keyBox = createObject('TSStatic')
    keyBox:setField('shapeName', 0, '/art/eurospeed/keybox.dae')
    keyBox:setField('collisionType', 0, 'None')
    keyBox:setField('decalType', 0, 'None')
    keyBox.canSave = false
    keyBox:registerObject('eurospeed_keybox')
    if scenetree.MissionGroup then scenetree.MissionGroup:add(keyBox) end
    log('I', logTag, 'key box on (overlay mode)')
  end
  local obb = veh:getSpawnWorldOOBB()
  local c, he = obb:getCenter(), obb:getHalfExtents()
  local axes = { obb:getAxis(0), obb:getAxis(1), obb:getAxis(2) }
  local ext = { he.x, he.y, he.z }
  -- The box axis closest to the car's up direction is "up".
  local upDir = veh:getDirectionVectorUp()
  local iz, best = 3, -1
  for i = 1, 3 do
    local d = math.abs(axes[i]:dot(upDir))
    if d > best then best, iz = d, i end
  end
  local az = axes[iz] * (axes[iz]:dot(upDir) >= 0 and 1 or -1)
  local i1, i2 = (iz % 3) + 1, ((iz + 1) % 3) + 1
  local ax, ay = axes[i1], az:cross(axes[i1])
  -- Floor just under the wheels; big enough around the car for chase cameras to stay inside.
  local bottom, top = -ext[iz] - 0.03, ext[iz] + 7
  local pos = c + az * ((top + bottom) / 2)
  local m = MatrixF(true)
  m:setColumn(0, ax)
  m:setColumn(1, ay)
  m:setColumn(2, az)
  m:setColumn(3, pos)
  keyBox:setTransform(m)
  setScaleCompat(keyBox, vec3(ext[i1] + 14, ext[i2] + 14, (top - bottom) / 2))
end

local function f3(v) return string.format('%.4f %.4f %.4f', v.x, v.y, v.z) end

local function onPreRender(dtReal)
  local reason = multiplayerReason()
  local mp = reason ~= nil
  if mp ~= inert then
    inert = mp
    log('W', logTag, inert and ('multiplayer (' .. reason .. '): EuroSpeed inert') or 'no multiplayer: EuroSpeed active')
  end
  if inert or not openSocket() then return end
  if not core_camera then return end

  local okCam, camPos = pcall(core_camera.getPosition)
  if not okCam or not camPos then return end
  local camFwd, camUp = core_camera.getForward(), core_camera.getUp()
  local fov = core_camera.getFovDeg() or 60

  local veh = getPlayerVehicle(0)
  pollInput()
  sendAcks()
  buildPending()
  reloadCollisionIfNeeded()
  local okN, errN = pcall(applyKeyNits)
  if not okN then log('E', logTag, 'key glow: ' .. tostring(errN)) end
  local okG, errG = pcall(updateGround)
  if not okG then log('E', logTag, 'ground: ' .. tostring(errG)) end
  -- Teleport only once the roads (or the ground block) under the spot exist and are solid.
  if veh and pendingTeleport and #pendingRoads == 0 and (roadsBuilt > 0 or groundSeq > 0) and not collisionDirty then
    local ok, err = pcall(teleportCar, veh, pendingTeleport)
    if not ok then log('E', logTag, 'teleport: ' .. tostring(err)) end
    pendingTeleport = nil
  end
  if veh then applyInput(veh) end
  local okBox, errBox = pcall(updateKeyBox, veh)
  if not okBox then log('E', logTag, 'key box: ' .. tostring(errBox)) end
  local vehPart = '0 0 0 0 1 0 0 0 1 0'
  if veh then
    vehPart = f3(veh:getPosition()) .. ' ' .. f3(veh:getDirectionVector()) .. ' ' ..
              f3(veh:getDirectionVectorUp()) .. ' ' .. string.format('%.3f', veh:getVelocity():length())
  end

  seq = seq + 1
  local line = string.format('ESB1 %d %d ', seq, Engine.Platform.getSystemTimeMS() % 2147483647) ..
               f3(camPos) .. ' ' .. f3(camFwd) .. ' ' .. f3(camUp) .. ' ' .. string.format('%.2f', fov) ..
               ' ' .. vehPart
  udp:send(line)
  -- Force feedback for the bridge (only while it keeps coming from the vehicle).
  if ffbVehId and Engine.Platform.getSystemTimeMS() - ffbAt < 200 then
    udp:send(string.format('ESF1 %.4f', ffbValue))
  end
  if refuelLogged and Engine.Platform.getSystemTimeMS() - refuelLoggedAt > 10000 then
    refuelLogged = false
    log('I', logTag, 'ETS2 refuel done')
  end
  -- BeamNG's menu open or not: the bridge then routes every key here (menu navigation).
  local menuNow = core_input_bindings and core_input_bindings.isMenuActive and true or false
  if menuNow ~= menuOpenSent or Engine.Platform.getSystemTimeMS() - menuSentAt > 2000 then
    menuOpenSent, menuSentAt = menuNow, Engine.Platform.getSystemTimeMS()
    udp:send(menuNow and 'ESMN 1' or 'ESMN 0')
  end
  -- Fuel: the car's tanks, once a second (the ETS2 truck's fuel follows them).
  local nowMs = Engine.Platform.getSystemTimeMS()
  if veh and nowMs - fuelPolledAt > 1000 then
    fuelPolledAt = nowMs
    veh:queueLuaCommand(fuelPoll)
  end
  if fuelLitres and nowMs - fuelAt < 3000 then
    if nowMs - fuelSentAt > 1000 then
      fuelSentAt = nowMs
      udp:send(string.format('ESU1 %.2f %.2f', fuelLitres, fuelCapacity))
    end
  end
end

local function onFuel(litres, capacity)
  if capacity and capacity > 0 then
    fuelLitres, fuelCapacity, fuelAt = litres, capacity, Engine.Platform.getSystemTimeMS()
  end
end

local function onExtensionLoaded()
  log('I', logTag, 'loaded')
end

local function onExtensionUnloaded()
  pcall(updateFFB, nil, false)
  if udp then udp:close() udp = nil end
  if udpIn then udpIn:close() udpIn = nil end
  clearRoads()
  pcall(clearGround)
  if keyBox then keyBox:delete() keyBox = nil pcall(setFogOff, false) pcall(setPostFxForKey, false) end
end

M.onPreRender = onPreRender
M.onFFB = onFFB
M.onFuel = onFuel
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onClientStartMission = onClientStartMission
return M
