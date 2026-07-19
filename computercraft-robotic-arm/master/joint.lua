-- joint.lua (MASTER)
-- Handles all rednet communication with the joint computers.
-- The master never touches redstone/peripherals directly for the arm --
-- it only ever sends {type="move", angle=...} style messages and waits
-- for {type="ack", ...} replies. This keeps kinematics and hardware
-- control fully decoupled.

local config = require("config")

local joint = {}

local resolvedIds = {} -- hostname -> computer id cache

-- Opens the configured (or first available) wireless modem and enables
-- rednet. Safe to call multiple times.
function joint.init()
    if rednet.isOpen(config.MODEM_SIDE) then return true end

    local side = config.MODEM_SIDE
    if not side then
        side = peripheral.find("modem", function(name, p) return p.isWireless and p.isWireless() end)
        if side then
            side = peripheral.getName(side)
        else
            side = peripheral.find("modem")
            side = side and peripheral.getName(side) or nil
        end
    end

    if not side then
        error("No modem found. Attach a (preferably wireless) modem to the Master computer.")
    end

    rednet.open(side)
    return true
end

-- Resolves a joint name ("base", "shoulder", ...) to a computer id via
-- rednet.lookup, caching the result. Returns nil if not found.
local function resolve(jointName)
    if resolvedIds[jointName] then return resolvedIds[jointName] end

    local jcfg = config.JOINTS[jointName] or (jointName == "gripper" and config.GRIPPER)
    if not jcfg then error("Unknown joint: " .. tostring(jointName)) end

    local id = rednet.lookup(config.PROTOCOL, jcfg.hostname)
    if id then resolvedIds[jointName] = id end
    return id
end

-- Sends a move command to a single joint and blocks until it acks,
-- errors, or times out. Returns true, finalAngle  or  false, errorMessage.
function joint.moveTo(jointName, angle)
    local id = resolve(jointName)
    if not id then
        return false, "joint '" .. jointName .. "' not found on network"
    end

    rednet.send(id, { type = "move", angle = angle }, config.PROTOCOL)

    local timer = os.startTimer(config.MOVE_TIMEOUT)
    while true do
        local ev, p1, p2, p3 = os.pullEvent()
        if ev == "rednet_message" and p1 == id and type(p2) == "table" then
            local msg = p2
            if msg.type == "ack" then
                return true, msg.angle
            elseif msg.type == "error" then
                return false, msg.reason or "joint reported an error"
            end
            -- ignore unrelated rednet traffic and keep waiting
        elseif ev == "timer" and p1 == timer then
            return false, "timeout waiting for '" .. jointName .. "' to ack"
        end
    end
end

-- Sends move commands to several joints in parallel and waits for all
-- acks. `moves` is a table like { base = 45, shoulder = 12 }.
-- Returns true if all succeeded, or false plus a table of {joint=reason}
-- for any that failed.
function joint.moveMany(moves)
    local results = {}
    local fns = {}

    for name, angle in pairs(moves) do
        fns[#fns + 1] = function()
            local ok, info = joint.moveTo(name, angle)
            results[name] = { ok = ok, info = info }
        end
    end

    if #fns > 0 then
        parallel.waitForAll(table.unpack(fns))
    end

    local allOk = true
    local errors = {}
    for name, r in pairs(results) do
        if not r.ok then
            allOk = false
            errors[name] = r.info
        end
    end

    return allOk, allOk and results or errors
end

-- Gripper is a simple open/close state, sent the same way as a move.
function joint.setGripper(state)
    local id = resolve("gripper")
    if not id then return false, "gripper joint not found on network" end

    rednet.send(id, { type = "gripper", state = state }, config.PROTOCOL)

    local timer = os.startTimer(config.MOVE_TIMEOUT)
    while true do
        local ev, p1, p2 = os.pullEvent()
        if ev == "rednet_message" and p1 == id and type(p2) == "table" and p2.type == "ack" then
            return true, p2.state
        elseif ev == "timer" and p1 == timer then
            return false, "timeout waiting for gripper to ack"
        end
    end
end

return joint
