-- gearshift.lua (JOINT COMPUTER)
-- Hardware abstraction layer. This is the ONE file you should need to
-- edit to match how your Sequenced Gearshift is actually wired/exposed.
-- Everything else in joint/ just calls gearshift.rotate(deltaDegrees).
--
-- Two implementations are provided:
--   "redstone"   -- pulses a redstone side; works with a Rotation Speed
--                   Controller, Redstone Link, or anything that advances
--                   the gearshift one increment per pulse.
--   "peripheral" -- calls directly into a wrapped peripheral, for setups
--                   using an addon that exposes the gearshift to CC.
--                   Method names below are placeholders -- check your
--                   addon's documentation and adjust them.

local config = require("config")

local gearshift = {}

-- ---------------------------------------------------------------
-- Redstone pulse implementation
-- ---------------------------------------------------------------
local function rotateRedstone(deltaDegrees)
    local pulses = math.floor(math.abs(deltaDegrees) / config.DEGREES_PER_PULSE + 0.5)
    if pulses == 0 then return end

    -- NOTE: forward/reverse direction on a real Sequenced Gearshift is
    -- usually set by the gearshift's own programmed instruction, not by
    -- redstone polarity. If your setup needs a direction signal (e.g. a
    -- second redstone side, or a comparator-set value), add it here.
    for _ = 1, pulses do
        redstone.setOutput(config.REDSTONE_SIDE, true)
        sleep(config.PULSE_LENGTH)
        redstone.setOutput(config.REDSTONE_SIDE, false)
        sleep(config.PULSE_GAP)
    end
end

-- ---------------------------------------------------------------
-- Direct peripheral implementation (placeholder method names --
-- adjust `p.rotate(...)` etc. to match your actual peripheral API)
-- ---------------------------------------------------------------
local function rotatePeripheral(deltaDegrees)
    local p = peripheral.wrap(config.PERIPHERAL_SIDE)
    if not p then
        error("No gearshift peripheral found on side '" .. config.PERIPHERAL_SIDE .. "'")
    end
    -- TODO: replace with your peripheral's real method(s), e.g.:
    --   p.setTargetAngle(currentAngle + deltaDegrees)
    -- or a step-based API:
    --   p.rotate(deltaDegrees)
    if p.rotate then
        p.rotate(deltaDegrees)
    else
        error("gearshift peripheral has no 'rotate' method -- edit gearshift.lua")
    end
end

-- Rotates the bearing by `deltaDegrees` (positive/negative = direction).
-- Blocks until the physical move should be complete.
function gearshift.rotate(deltaDegrees)
    if config.GEARSHIFT_MODE == "redstone" then
        rotateRedstone(deltaDegrees)
    elseif config.GEARSHIFT_MODE == "peripheral" then
        rotatePeripheral(deltaDegrees)
    else
        error("Unknown GEARSHIFT_MODE: " .. tostring(config.GEARSHIFT_MODE))
    end
end

-- ---------------------------------------------------------------
-- Gripper (open/close) control -- adjust to match how your gripper is
-- actuated (redstone piston, another gearshift, etc).
-- ---------------------------------------------------------------
function gearshift.setGripper(state)
    if state == "open" then
        redstone.setOutput(config.REDSTONE_SIDE, false)
    else
        redstone.setOutput(config.REDSTONE_SIDE, true)
    end
end

return gearshift
