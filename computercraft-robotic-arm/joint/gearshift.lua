-- gearshift.lua (JOINT COMPUTER)
-- Hardware abstraction layer. Everything else in joint/ just calls
-- gearshift.rotate(deltaDegrees) -- this file is the only place that
-- needs to know how the Sequenced Gearshift is actually being driven.

local config = require("config")

local gearshift = {}

local function findGearshift()
    if config.PERIPHERAL_SIDE then
        local p = peripheral.wrap(config.PERIPHERAL_SIDE)
        if not p then
            error("No peripheral found on side '" .. config.PERIPHERAL_SIDE ..
                "'. Is the Sequenced Gearshift placed directly against that face?")
        end
        return p
    end

    local p = peripheral.find("Create_SequencedGearshift")
    if not p then
        error("No Sequenced Gearshift peripheral found. Make sure it's placed " ..
            "directly against this computer (any face), or set config.PERIPHERAL_SIDE " ..
            "explicitly if you have more than one and need to pick.")
    end
    return p
end

local function rotatePeripheral(deltaDegrees)
    if deltaDegrees == 0 then return end

    local p = findGearshift()

    local angle = math.floor(math.abs(deltaDegrees) + 0.5)
    if angle == 0 then return end

    local modifier = deltaDegrees < 0 and -1 or 1
    if config.INVERT_DIRECTION then modifier = -modifier end

    p.rotate(angle, modifier)

    local deadline = os.clock() + config.MOVE_TIMEOUT
    while p.isRunning() do
        if os.clock() > deadline then
            error("gearshift did not finish rotating within " .. config.MOVE_TIMEOUT .. "s")
        end
        sleep(0.1)
    end

    gearshift.waitForBearingSettle(config.MOVE_TIMEOUT)
end

local function rotateRedstone(deltaDegrees)
    local pulses = math.floor(math.abs(deltaDegrees) / config.DEGREES_PER_PULSE + 0.5)
    if pulses == 0 then return end

    for _ = 1, pulses do
        redstone.setOutput(config.REDSTONE_SIDE, true)
        sleep(config.PULSE_LENGTH)
        redstone.setOutput(config.REDSTONE_SIDE, false)
        sleep(config.PULSE_GAP)
    end
end

function gearshift.getActualAngle()
    local p = gearshift.findBearing()
    if not p or not p.getTargetAngle then return nil end
    return p.getTargetAngle()
end

function gearshift.findBearing()
    if config.SWIVEL_PERIPHERAL_SIDE then
        return peripheral.wrap(config.SWIVEL_PERIPHERAL_SIDE)
    end
    return peripheral.find("swivel_bearing")
end

function gearshift.ensureBearingAssembled()
    local bearing = gearshift.findBearing()
    if not bearing then
        return false, "no swivel_bearing peripheral found"
    end
    if bearing.isAssembled() then
        return true, "already assembled"
    end
    local ok, err = pcall(function() bearing.assemble() end)
    if ok and bearing.isAssembled() then
        return true, "assembled"
    end
    local reason = (bearing.getLastAssemblyException and bearing.getLastAssemblyException()) or err
    return false, tostring(reason)
end

function gearshift.waitForBearingSettle(timeoutSeconds)
    local bearing = gearshift.findBearing()
    if not bearing or not bearing.getTargetAngle then return end

    local lastReading = bearing.getTargetAngle()
    local stableTicks = 0
    local deadline = os.clock() + (timeoutSeconds or 5)
    while stableTicks < 3 and os.clock() < deadline do
        sleep(0.1)
        local reading = bearing.getTargetAngle()
        if math.abs(reading - lastReading) < 0.05 then
            stableTicks = stableTicks + 1
        else
            stableTicks = 0
        end
        lastReading = reading
    end
end

function gearshift.rotate(deltaDegrees)
    if config.GEARSHIFT_MODE == "peripheral" then
        rotatePeripheral(deltaDegrees)
    elseif config.GEARSHIFT_MODE == "redstone" then
        rotateRedstone(deltaDegrees)
    else
        error("Unknown GEARSHIFT_MODE: " .. tostring(config.GEARSHIFT_MODE))
    end
end

function gearshift.setGripper(state)
    if config.GEARSHIFT_MODE == "peripheral" then
        local p = findGearshift()
        if state == "closed" then
            p.rotate(45, 1)
        else
            p.rotate(45, -1)
        end
        local deadline = os.clock() + config.MOVE_TIMEOUT
        while p.isRunning() do
            if os.clock() > deadline then break end
            sleep(0.1)
        end
    else
        redstone.setOutput(config.REDSTONE_SIDE, state == "closed")
    end
end

return gearshift
