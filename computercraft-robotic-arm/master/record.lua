-- record.lua (MASTER)
-- Stores and replays named robot poses, persisted to disk so they
-- survive a reboot. A "pose" here is just a joint-angle table plus the
-- gripper state -- see robot.lua for the pipeline that plays it back.

local config = require("config")

local record = {}

local poses = {}

local function load()
    if fs.exists(config.POSES_FILE) then
        local f = fs.open(config.POSES_FILE, "r")
        local data = f.readAll()
        f.close()
        local ok, decoded = pcall(textutils.unserialize, data)
        if ok and type(decoded) == "table" then
            poses = decoded
        end
    end
end

local function persist()
    local f = fs.open(config.POSES_FILE, "w")
    f.write(textutils.serialize(poses))
    f.close()
end

load()

-- Saves the given angles (defaults to nothing useful without robot.lua
-- calling this with robot.state.angles) under `name`.
function record.save(name, angles, gripperState)
    poses[name] = {
        angles = {
            base = angles.base,
            shoulder = angles.shoulder,
            elbow = angles.elbow,
            wrist = angles.wrist,
        },
        gripper = gripperState,
    }
    persist()
end

-- Plays back a saved pose using the robot's high-level API. Moves each
-- joint (planned/smoothed by robot.moveJoint) then sets the gripper.
function record.play(name, robot)
    local pose = poses[name]
    if not pose then return false, "no such pose: " .. tostring(name) end

    for jointName, angle in pairs(pose.angles) do
        local ok, err = robot.moveJoint(jointName, angle)
        if not ok then return false, err end
    end

    if pose.gripper == "open" then
        robot.openGripper()
    elseif pose.gripper == "closed" then
        robot.closeGripper()
    end

    return true
end

-- Runs a named sequence of poses in order -- simple pick-and-place
-- scripting. `names` is an ordered list of pose names.
function record.playSequence(names, robot)
    for _, name in ipairs(names) do
        local ok, err = record.play(name, robot)
        if not ok then return false, name .. ": " .. tostring(err) end
    end
    return true
end

function record.list()
    local names = {}
    for name in pairs(poses) do names[#names + 1] = name end
    table.sort(names)
    return names
end

function record.delete(name)
    poses[name] = nil
    persist()
end

return record
