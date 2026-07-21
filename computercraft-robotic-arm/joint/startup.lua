-- startup.lua (JOINT COMPUTER)
-- Runs on each individual joint computer. Listens for move/gripper
-- commands from the Master, drives the local gearshift, and reports
-- back. Never does any kinematics -- just local motor control.

package.path = "/?.lua;" .. package.path

local config    = require("config")
local gearshift = require("gearshift")

-- ---------------------------------------------------------------
-- Persisted current angle, so this joint remembers its position
-- across reboots even before the first command arrives.
-- ---------------------------------------------------------------
local currentAngle = 0

local function loadState()
    if fs.exists(config.STATE_FILE) then
        local f = fs.open(config.STATE_FILE, "r")
        local n = tonumber(f.readAll())
        f.close()
        if n then currentAngle = n end
    end
end

local function saveState()
    local f = fs.open(config.STATE_FILE, "w")
    f.write(tostring(currentAngle))
    f.close()
end

-- ---------------------------------------------------------------
-- Networking
-- ---------------------------------------------------------------
local function openModem()
    local side = config.MODEM_SIDE
    if not side then
        -- Prefer a wireless modem if there's more than one attached --
        -- a wired modem (labeled "peripheral_hub" in `peripherals`)
        -- only reaches other computers on the same cabled network, not
        -- the Master over rednet wirelessly.
        local m = peripheral.find("modem", function(name, p) return p.isWireless and p.isWireless() end)
        if not m then
            m = peripheral.find("modem")
        end
        if not m then error("No modem attached to this joint computer.") end
        side = peripheral.getName(m)
    end
    rednet.open(side)
    rednet.host(config.PROTOCOL, config.JOINT_NAME)
    print("rednet open on '" .. side .. "', hosting as '" .. config.JOINT_NAME .. "'")
end

-- Shortest signed delta in (-180, 180].
local function shortestDelta(from, to)
    local d = (to - from) % 360
    if d > 180 then d = d - 360 end
    return d
end

-- ---------------------------------------------------------------
-- Command handling
-- ---------------------------------------------------------------
local function handleMove(senderId, msg)
    local target = msg.angle

    -- If we have a swivel bearing to read from, trust its real angle
    -- over our own locally-saved guess before computing the move --
    -- this stops small errors from ever accumulating across moves.
    local actual = gearshift.getActualAngle()
    if actual then currentAngle = actual end

    local delta = shortestDelta(currentAngle, target)

    local ok, err = pcall(function()
        gearshift.rotate(delta)
    end)

    if ok then
        -- After moving, prefer the bearing's real angle again as the
        -- value we report/persist, rather than our commanded target.
        local finalAngle = gearshift.getActualAngle() or target
        currentAngle = finalAngle
        saveState()
        rednet.send(senderId, { type = "ack", angle = currentAngle }, config.PROTOCOL)
    else
        rednet.send(senderId, { type = "error", reason = tostring(err) }, config.PROTOCOL)
    end
end

local function handleGripper(senderId, msg)
    local ok, err = pcall(function()
        gearshift.setGripper(msg.state)
    end)

    if ok then
        rednet.send(senderId, { type = "ack", state = msg.state }, config.PROTOCOL)
    else
        rednet.send(senderId, { type = "error", reason = tostring(err) }, config.PROTOCOL)
    end
end

-- Lightweight query: report the REAL angle right now, straight from the
-- bearing, without touching the gearshift at all. Doesn't move
-- anything, doesn't block on isRunning()/settle -- safe to call any
-- time, even mid-move, to check for drift.
local function handleStatus(senderId, msg)
    local actual = gearshift.getActualAngle()
    rednet.send(senderId, {
        type = "status",
        actualAngle = actual,       -- nil if no swivel_bearing found
        commandedAngle = currentAngle, -- this joint's own last-known target
    }, config.PROTOCOL)
end

-- ---------------------------------------------------------------
-- Main loop
-- ---------------------------------------------------------------
loadState()
openModem()

local assembled, reason = gearshift.ensureBearingAssembled()
if assembled then
    print("Swivel bearing: " .. reason)
else
    print("WARNING: swivel bearing not assembled (" .. reason .. ")")
    print("Rotation may spin freely with nothing attached until this is fixed.")
end

print("Joint computer online: " .. config.JOINT_NAME)
print("Current angle: " .. currentAngle)
print("Waiting for commands from Master...")

while true do
    local senderId, msg = rednet.receive(config.PROTOCOL)
    if type(msg) == "table" then
        if msg.type == "move" and not config.IS_GRIPPER then
            handleMove(senderId, msg)
        elseif msg.type == "gripper" and config.IS_GRIPPER then
            handleGripper(senderId, msg)
        elseif msg.type == "status" then
            handleStatus(senderId, msg)
        end
        -- silently ignore anything else (e.g. move commands sent to the
        -- gripper computer by mistake, or unrelated protocol traffic)
    end
end
