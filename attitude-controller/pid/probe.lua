-- UAC PROBE -- throwaway diagnostic, delete once uac.lua is calibrated.
--
-- The UAC's attitude loop needs three things this probe pins down, none of which
-- can be read off the mod's docs with certainty:
--   1. The quaternion namespace: which CONSTRUCTOR exists (fromAxisAngle /
--      fromEuler / new) and its argument order, plus which METHODS an orientation
--      quaternion instance exposes (inverse/getAxis/getAngle/toEuler/...).
--   2. The mat3x3 shape returned by sublevel.getInertiaTensor() -- nested table,
--      flat 9, or an object -- so the UAC's matrix*vector helper can read it.
--   3. The FRAME of sublevel.getAngularVelocity(): does it read in WORLD axes or
--      the craft's BODY axes? Hand-rotate the craft and watch which components
--      move (see the live readout at the bottom).
--
-- Run it, read the one-shot API dump at the top, then hand-rotate/tilt the craft
-- and watch the live block. Nothing here commands a thruster.

local function hr(t) print(("-"):rep(18) .. " " .. t) end

-- Dump a table's keys and value types (one level), sorted.
local function dumpKeys(tbl, label)
    hr(label)
    if type(tbl) ~= "table" then print("  (not a table: " .. type(tbl) .. ")"); return end
    local keys = {}
    for k in pairs(tbl) do keys[#keys + 1] = tostring(k) end
    table.sort(keys)
    if #keys == 0 then print("  (no iterable keys)") end
    for _, k in ipairs(keys) do print("  " .. k .. " : " .. type(tbl[k])) end
end

-- Recursively render a value compactly, for the inertia tensor's unknown shape.
local function show(v, depth)
    depth = depth or 0
    if depth > 3 then return "..." end
    if type(v) == "number" then return string.format("%.4g", v) end
    if type(v) ~= "table" then return tostring(v) end
    local parts = {}
    local n = 0
    for k, val in pairs(v) do
        n = n + 1
        if n > 12 then parts[#parts + 1] = "..."; break end
        parts[#parts + 1] = tostring(k) .. "=" .. show(val, depth + 1)
    end
    return "{" .. table.concat(parts, ", ") .. "}"
end

term.clear(); term.setCursorPos(1, 1)

if not sublevel then error("No sublevel API on this computer") end

-- ---- 1. quaternion namespace + instance API -------------------------------
if type(quaternion) == "table" then
    dumpKeys(quaternion, "global 'quaternion' functions")
else
    print("global 'quaternion' is " .. type(quaternion) .. " (constructors may be missing)")
end

local pose = sublevel.getLogicalPose()
local q = pose.orientation
hr("orientation instance")
print("  type            : " .. type(q))
print("  metatable.__name: " .. tostring((getmetatable(q) or {}).__name))
local mt = getmetatable(q)
if mt and type(mt.__index) == "table" then dumpKeys(mt.__index, "orientation methods (__index)") end

-- toEuler shape: three numbers, or one vector?
local a, b, c = q:toEuler()
if type(a) == "number" then
    print(string.format("toEuler -> 3 numbers  (pitch %.3f, yaw %.3f, roll %.3f)", a, b or 0/0, c or 0/0))
else
    print("toEuler -> single value: " .. show(a))
end

-- Try each plausible constructor; report which build a valid quaternion and what
-- toEuler reads back for a known +0.5 rad yaw about world-up (0,1,0).
local function tryCtor(name, fn)
    local ok, res = pcall(fn)
    if not ok or type(res) ~= "table" then
        print(string.format("  %-14s : NO (%s)", name, tostring(res):sub(1, 40)))
        return
    end
    local pa, pb, pc = res:toEuler()
    if type(pa) == "number" then
        print(string.format("  %-14s : ok  toEuler(p %.3f, y %.3f, r %.3f)", name, pa, pb or 0, pc or 0))
    else
        print(string.format("  %-14s : ok  toEuler=%s", name, show(pa)))
    end
end
hr("constructor test (target: level, yaw = +0.5 rad about Y)")
tryCtor("fromAxisAngle", function() return quaternion.fromAxisAngle(vector.new(0, 1, 0), 0.5) end)
tryCtor("fromEuler(p,y,r)", function() return quaternion.fromEuler(0, 0.5, 0) end)
tryCtor("new(w,x,y,z)", function() local h = 0.25; return quaternion.new(math.cos(h), 0, math.sin(h), 0) end)
tryCtor("new(x,y,z,w)", function() local h = 0.25; return quaternion.new(0, math.sin(h), 0, math.cos(h)) end)

-- ---- 2. inertia tensor shape ----------------------------------------------
hr("inertia tensor shape")
local ok, I = pcall(sublevel.getInertiaTensor)
if ok then print("  getInertiaTensor -> " .. show(I)) else print("  getInertiaTensor error: " .. tostring(I)) end
local okv, Iinv = pcall(sublevel.getInverseInertiaTensor)
if okv then print("  getInverseInertiaTensor -> " .. show(Iinv)) end

-- ---- 3. live frame observation --------------------------------------------
-- Hand-rotate the craft: if angular velocity is BODY-frame, a pure roll moves the
-- SAME component no matter which way the nose points; if WORLD-frame, the moving
-- component changes as you yaw the craft. Compare against the euler rates too.
hr("LIVE  (Ctrl-T to stop)  -- hand-rotate the craft and watch which axes move")
local lastP, lastY, lastR, lastT
while true do
    local p2 = sublevel.getLogicalPose()
    local o = p2.orientation
    local w = sublevel.getAngularVelocity()
    local ep, ey, er = o:toEuler()
    if type(ep) ~= "number" then ep, ey, er = o:toEuler().x, o:toEuler().y, o:toEuler().z end

    local now = os.clock()
    local dP, dY, dR = 0, 0, 0
    if lastT then
        local dt = math.max(now - lastT, 1e-3)
        dP = (ep - lastP) / dt; dY = (ey - lastY) / dt; dR = (er - lastR) / dt
    end
    lastP, lastY, lastR, lastT = ep, ey, er, now

    term.setCursorPos(1, 16); term.clearLine()
    term.write(string.format("EULER p%+6.2f y%+6.2f r%+6.2f rad", ep, ey, er))
    term.setCursorPos(1, 17); term.clearLine()
    term.write(string.format("angVel x%+6.2f y%+6.2f z%+6.2f", w.x, w.y, w.z))
    term.setCursorPos(1, 18); term.clearLine()
    term.write(string.format("dEuler p%+6.2f y%+6.2f r%+6.2f /s", dP, dY, dR))
    sleep(0.1)
end
