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
        local m = peripheral.find("modem")
        if not m then error("No modem attached to this joint computer.") end
        side = peripheral.getName(m)
    end
    rednet.open(side)
    rednet.host(config.PROTOCOL, config.JOINT_NAME)
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
    local delta = shortestDelta(currentAngle, target)

    local ok, err = pcall(function()
        gearshift.rotate(delta)
    end)

    if ok then
        currentAngle = target
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

-- ---------------------------------------------------------------
-- Main loop
-- ---------------------------------------------------------------
loadState()
openModem()

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
        end
        -- silently ignore anything else (e.g. move commands sent to the
        -- gripper computer by mistake, or unrelated protocol traffic)
    end
end
