-- planner.lua (MASTER)
-- Turns a start pose and end pose into a sequence of intermediate
-- waypoints so joints move smoothly instead of snapping to target.

local config = require("config")

local planner = {}

-- Shortest signed angular delta from a to b, in degrees, in (-180, 180].
local function shortestDelta(a, b)
    local d = (b - a) % 360
    if d > 180 then d = d - 360 end
    return d
end

-- Builds a list of waypoints (each a table of joint->angle) interpolating
-- from `startPose` to `endPose`. Both poses are tables like
-- { base=, shoulder=, elbow=, wrist= }.
--
-- The number of steps is config.TRAJECTORY_STEPS, but is increased
-- automatically if that would exceed config.MAX_STEP_DEGREES for any
-- joint, so fast/long moves stay smooth.
function planner.plan(startPose, endPose)
    local maxDelta = 0
    local deltas = {}

    for jointName, target in pairs(endPose) do
        local from = startPose[jointName] or target
        local d = shortestDelta(from, target)
        deltas[jointName] = d
        if math.abs(d) > maxDelta then maxDelta = math.abs(d) end
    end

    local steps = math.max(config.TRAJECTORY_STEPS, math.ceil(maxDelta / config.MAX_STEP_DEGREES))
    steps = math.max(steps, 1)

    local waypoints = {}
    for step = 1, steps do
        local t = step / steps
        local pose = {}
        for jointName, target in pairs(endPose) do
            local from = startPose[jointName] or target
            pose[jointName] = from + deltas[jointName] * t
        end
        waypoints[#waypoints + 1] = pose
    end

    return waypoints
end

-- Convenience: walks a set of waypoints, calling `sendFn(pose)` for each
-- one and waiting for it to return before sending the next. `sendFn`
-- should return true/false (success). Stops early on failure.
function planner.execute(waypoints, sendFn)
    for _, pose in ipairs(waypoints) do
        local ok, err = sendFn(pose)
        if not ok then
            return false, err
        end
    end
    return true
end

return planner
