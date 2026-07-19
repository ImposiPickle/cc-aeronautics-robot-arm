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

    local p = peripheral.find("Create_SequencedGearshift")
    if not p then
        error("No Sequenced Gearshift peripheral found. Set config.PERIPHERAL_SIDE " ..
            "explicitly, or make sure the gearshift is adjacent to this computer.")
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

    -- rotate() starts the motion and returns immediately -- wait for it
    -- to actually finish before we ack back to the Master.
    local deadline = os.clock() + config.MOVE_TIMEOUT
    while p.isRunning() do
        if os.clock() > deadline then
            error("gearshift did not finish rotating within " .. config.MOVE_TIMEOUT .. "s")
        end
        sleep(0.1)
    end
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
