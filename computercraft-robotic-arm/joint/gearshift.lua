-- gearshift.lua (JOINT COMPUTER)
-- Hardware abstraction layer. Everything else in joint/ just calls
-- gearshift.rotate(deltaDegrees) -- this file is the only place that
-- needs to know how the Sequenced Gearshift is actually being driven.
--
-- PREFERRED: "peripheral" mode. Recent Create versions expose the
-- Sequenced Gearshift directly to CC:Tweaked with:
--   rotate(angle, [modifier])  -- angle: positive degrees, modifier -1 reverses
--   isRunning()                -- true while the shaft is still turning
-- No addon and no redstone wiring needed for this -- just place the
-- computer directly adjacent to the gearshift (or link them over a
-- wired modem network), and leave the gearshift's own instruction list
-- EMPTY (a computer-controlled gearshift ignores its programmed
-- sequence while a computer is attached).
--
-- FALLBACK: "redstone" mode, for setups where the peripheral isn't
-- available -- pulses a redstone side some number of times, each pulse
-- advancing a gearshift programmed with "Await New Redstone Pulse" /
-- "Turn by Angle" instructions.

local config = require("config")

local gearshift = {}

-- ---------------------------------------------------------------
-- Peripheral implementation (recommended)
-- ---------------------------------------------------------------
local function findGearshift()
    if config.PERIPHERAL_SIDE then
        local p = peripheral.wrap(config.PERIPHERAL_SIDE)
        if not p then
            error("No peripheral found on side '" .. config.PERIPHERAL_SIDE ..
                "'. Is the Sequenced Gearshift placed directly against that face?")
        end
        return p
    end

    -- Proven-working default: auto-detect, same as the reference
    -- implementation this was checked against (`peripheral.find`
    -- with no side needed at all for a directly-adjacent gearshift).
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

    -- deltaDegrees is how far the BEARING needs to turn. Scale it by
    -- GEAR_RATIO to get how far to command the GEARSHIFT itself to
    -- turn, since there may be gearing (e.g. a Large Cogwheel) between
    -- the two that changes the relationship.
    local commandDegrees = deltaDegrees * config.GEAR_RATIO
    local angle = math.floor(math.abs(commandDegrees) + 0.5)
    if angle == 0 then return end

    local modifier = deltaDegrees < 0 and -1 or 1
    if config.INVERT_DIRECTION then modifier = -modifier end

    p.rotate(angle, modifier)

    -- rotate() starts the motion and returns immediately -- wait for it
    -- to actually finish before we ack back to the Master.
    local deadline = os.clock() + config.MOVE_TIMEOUT
    while p.isRunning() do
        if os.clock() > deadline then
            error("gearshift did not finish rotating within " .. config.MOVE_TIMEOUT .. "s")
        end
        sleep(0.1)
    end

    -- isRunning() only reflects the gearshift's own instruction state --
    -- the assembled contraption can still be catching up physically for
    -- a moment after that. Wait briefly for the bearing's real angle to
    -- settle, but capped much shorter than MOVE_TIMEOUT -- this is a
    -- courtesy wait, not something that should ever double the timeout.
    gearshift.waitForBearingSettle(2)
end

-- ---------------------------------------------------------------
-- Redstone pulse implementation (fallback)
-- ---------------------------------------------------------------
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

-- If a swivel_bearing peripheral is present, returns its real current
-- angle (degrees) straight from the game -- ground truth, not a
-- locally-counted guess. Uses config.SWIVEL_PERIPHERAL_SIDE if set,
-- otherwise auto-detects (matching the reference implementation's
-- `peripheral.find("swivel_bearing")` pattern). Returns nil if none
-- is found -- callers should fall back to the locally-persisted angle.
function gearshift.getActualAngle()
    local p = gearshift.findBearing()
    if not p or not p.getTargetAngle then return nil end
    return p.getTargetAngle()
end

-- Returns the swivel_bearing peripheral for this joint, or nil.
function gearshift.findBearing()
    if config.SWIVEL_PERIPHERAL_SIDE then
        return peripheral.wrap(config.SWIVEL_PERIPHERAL_SIDE)
    end
    return peripheral.find("swivel_bearing")
end

-- A Swivel Bearing must be ASSEMBLED into a Simulated Contraption
-- before rotating it actually moves anything attached -- otherwise the
-- input cog spins freely with nothing physically attached to turn.
-- Call this once on startup. Returns true if assembled (or already
-- was), false + a reason string otherwise.
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

-- Waits for the bearing's real angle to stop changing before
-- returning. isRunning() going false only means the GEARSHIFT's
-- instruction finished -- the assembled contraption's actual angle can
-- still be catching up for a moment. Skips silently if no bearing is
-- configured/found.
function gearshift.waitForBearingSettle(timeoutSeconds)
    local bearing = gearshift.findBearing()
    if not bearing or not bearing.getTargetAngle then return end

    local lastReading = bearing.getTargetAngle()
    local stableTicks = 0
    local deadline = os.clock() + (timeoutSeconds or 5)
    while stableTicks < 2 and os.clock() < deadline do
        sleep(0.1)
        local reading = bearing.getTargetAngle()
        if math.abs(reading - lastReading) < 0.3 then
            stableTicks = stableTicks + 1
        else
            stableTicks = 0
        end
        lastReading = reading
    end
end

-- Rotates the bearing by `deltaDegrees` (positive/negative = direction).
-- Blocks until the physical move should be complete.
function gearshift.rotate(deltaDegrees)
    if config.GEARSHIFT_MODE == "peripheral" then
        rotatePeripheral(deltaDegrees)
    elseif config.GEARSHIFT_MODE == "redstone" then
        rotateRedstone(deltaDegrees)
    else
        error("Unknown GEARSHIFT_MODE: " .. tostring(config.GEARSHIFT_MODE))
    end
end

-- ---------------------------------------------------------------
-- Gripper (open/close) control -- adjust to match how your gripper is
-- actuated (another Sequenced Gearshift, a piston, redstone, etc).
-- ---------------------------------------------------------------
function gearshift.setGripper(state)
    if config.GEARSHIFT_MODE == "peripheral" then
        local p = findGearshift()
        -- Treat "closed" as a fixed rotate, "open" as the reverse.
        -- Adjust the angle to whatever your gripper mechanism needs.
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
