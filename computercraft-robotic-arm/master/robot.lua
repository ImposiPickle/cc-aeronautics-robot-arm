-- robot.lua (MASTER)
-- Stores robot state and exposes the high-level API. This is the ONLY
-- module ui.lua, record.lua, and any user scripts should call into --
-- nothing outside robot.lua/joint.lua should ever address a joint
-- computer directly.

local config     = require("config")
local jointComms = require("joint")
local kinematics = require("kinematics")
local planner    = require("planner")

local robot = {}

-- Current known state. Angles are in degrees, matching config.JOINTS.
robot.state = {
    angles = {
        base = config.JOINTS.base.homeAngle,
        shoulder = config.JOINTS.shoulder.homeAngle,
        elbow = config.JOINTS.elbow.homeAngle,
        wrist = config.JOINTS.wrist.homeAngle,
    },
    gripper = "open",
    lengths = config.LINK_LENGTHS,
    busy = false,
    lastError = nil,
}

-- Anything that wants to react to state changes (renderer, ui status
-- line) can subscribe here instead of polling.
local listeners = {}
function robot.onUpdate(fn) listeners[#listeners + 1] = fn end
local function notify()
    for _, fn in ipairs(listeners) do fn(robot.state) end
end

function robot.init()
    jointComms.init()
    notify()
end

-- Returns forward-kinematics joint/effector positions for the current
-- (or a supplied) pose.
function robot.getPositions(angles)
    return kinematics.forward(angles or robot.state.angles, robot.state.lengths)
end

function robot.maxReach()
    return kinematics.maxReach(robot.state.lengths)
end

-- Clamp a target angle to the configured limits for that joint.
local function clamp(jointName, angle)
    local lim = config.JOINTS[jointName]
    if not lim then return angle end
    if angle < lim.minAngle then return lim.minAngle end
    if angle > lim.maxAngle then return lim.maxAngle end
    return angle
end

-- Move a single named joint to an absolute angle (degrees). Sent as
-- ONE command, not chunked through planner.lua -- Create interpolates
-- the physical rotation smoothly on its own for a single joint, and
-- chunking here only adds per-step round-trip overhead (and more
-- chances for rounding error to accumulate). Blocks until complete.
function robot.moveJoint(jointName, angle)
    if not config.JOINTS[jointName] then
        robot.state.lastError = "no such joint: " .. tostring(jointName)
        return false, robot.state.lastError
    end

    local target = clamp(jointName, angle)
    return robot.executeTrajectory({ { [jointName] = target } })
end

-- Move to a world-space target (x, y, z) via inverse kinematics, planned
-- as a smooth multi-joint trajectory. Blocks until complete.
function robot.moveTo(x, y, z)
    local angles, err = kinematics.inverse({ x = x, y = y, z = z }, robot.state.lengths, config.JOINTS)
    if not angles then
        robot.state.lastError = err
        notify()
        return false, err
    end

    local waypoints = planner.plan(robot.state.angles, angles)
    return robot.executeTrajectory(waypoints)
end

-- Runs a pre-built list of waypoints (see planner.lua), sending each one
-- to all relevant joints in parallel and updating state as it goes.
function robot.executeTrajectory(waypoints)
    robot.state.busy = true
    robot.state.lastError = nil
    notify()

    for _, pose in ipairs(waypoints) do
        local ok, info = jointComms.moveMany(pose)
        if not ok then
            robot.state.busy = false
            robot.state.lastError = textutils.serialize(info)
            notify()
            return false, robot.state.lastError
        end
        -- `info` here is moveMany's per-joint results table on success:
        -- { jointName = { ok = true, info = <actual angle the joint
        -- reported back, read from its bearing> }, ... }. Use THAT,
        -- not the commanded pose value -- otherwise the Master's
        -- tracked state just echoes what it asked for regardless of
        -- where the joint actually ended up.
        for name in pairs(pose) do
            local reported = info[name] and info[name].info
            robot.state.angles[name] = (type(reported) == "number") and reported or pose[name]
        end
        notify()
    end

    robot.state.busy = false
    notify()
    return true
end

-- Sends every joint back to its configured home angle.
function robot.home()
    local endPose = {}
    for name, jcfg in pairs(config.JOINTS) do
        endPose[name] = jcfg.homeAngle
    end
    local waypoints = planner.plan(robot.state.angles, endPose)
    return robot.executeTrajectory(waypoints)
end

function robot.openGripper()
    local ok, result = jointComms.setGripper(config.GRIPPER.openState)
    if ok then robot.state.gripper = "open" end
    notify()
    return ok, result
end

function robot.closeGripper()
    local ok, result = jointComms.setGripper(config.GRIPPER.closedState)
    if ok then robot.state.gripper = "closed" end
    notify()
    return ok, result
end

return robot
