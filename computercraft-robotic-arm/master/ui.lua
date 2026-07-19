-- ui.lua (MASTER)
-- Keyboard jog control on the terminal, plus click-to-move on the
-- monitor. Talks only to robot.lua's high-level API.

local config = require("config")

local ui = {}

local JOG_STEP = 5 -- degrees per keypress
local selected = 1 -- index into config.JOINT_ORDER

local KEY_SELECT_NEXT = keys.tab
local KEY_INCREASE     = keys.up
local KEY_DECREASE     = keys.down
local KEY_GRIPPER      = keys.g
local KEY_HOME         = keys.h
local KEY_SAVE_POSE    = keys.s
local KEY_QUIT         = keys.q

local function printHelp()
    print("Robotic Arm Manual Control")
    print("---------------------------------")
    print("TAB        select next joint")
    print("UP / DOWN  jog selected joint +-" .. JOG_STEP .. " deg")
    print("g          toggle gripper")
    print("h          home all joints")
    print("s          save current pose (prompts for name)")
    print("q          quit UI (renderer keeps running)")
    print("Click the monitor to move the end effector there")
    print("---------------------------------")
end

-- Keyboard loop: manual jogging of individual joints.
local function keyboardLoop(robot)
    printHelp()
    while true do
        local jointName = config.JOINT_ORDER[selected]
        print("selected joint: " .. jointName)

        local ev, key = os.pullEvent("key")
        if key == KEY_SELECT_NEXT then
            selected = (selected % #config.JOINT_ORDER) + 1
        elseif key == KEY_INCREASE then
            robot.moveJoint(jointName, robot.state.angles[jointName] + JOG_STEP)
        elseif key == KEY_DECREASE then
            robot.moveJoint(jointName, robot.state.angles[jointName] - JOG_STEP)
        elseif key == KEY_GRIPPER then
            if robot.state.gripper == "open" then robot.closeGripper() else robot.openGripper() end
        elseif key == KEY_HOME then
            robot.home()
        elseif key == KEY_SAVE_POSE then
            write("pose name: ")
            local name = read()
            if name and name ~= "" then
                local record = require("record")
                record.save(name, robot.state.angles)
                print("saved pose '" .. name .. "'")
            end
        elseif key == KEY_QUIT then
            print("UI stopped (renderer still running).")
            return
        end
    end
end

-- Monitor click loop: converts a touch on the plan view back into a
-- world-space (x, z) coordinate at the current effector height, and
-- issues a moveTo. This mirrors the transform used in renderer.lua, so
-- if you change the renderer's projection, update this too.
local function monitorClickLoop(robot, kinematics, renderer)
    while true do
        local ev, side, cx, cy = os.pullEvent("monitor_touch")

        local maxReach = kinematics.maxReach(robot.state.lengths)
        local mon = peripheral.wrap(config.MONITOR_SIDE)
        local w, h = mon.getSize()
        local planW = math.floor(w * 0.6)
        local originX = math.floor(planW * 0.5)
        local scale = (planW * 0.45) / math.max(maxReach, 1)

        if cx <= planW then
            local worldX = (cx - originX) / scale
            local worldZ = (cy - h * 0.5) / (scale * 0.5)
            local target = { x = worldX, y = robot.state.lengths.upperArm * 0.3, z = worldZ }
            renderer.setTarget(target)
            local ok, err = robot.moveTo(target.x, target.y, target.z)
            if not ok then print("move failed: " .. tostring(err)) end
        end
    end
end

-- Runs both input loops in parallel. Call inside parallel.waitForAny
-- alongside renderer.loop.
function ui.loop(robot, kinematics, renderer)
    parallel.waitForAny(
        function() keyboardLoop(robot) end,
        function() monitorClickLoop(robot, kinematics, renderer) end
    )
end

return ui
