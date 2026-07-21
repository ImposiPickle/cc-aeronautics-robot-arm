-- ui.lua (MASTER)
-- Keyboard jog control on the terminal, plus click-to-aim-base on the
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
    print("Click the monitor to point the base at that direction")
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

-- Monitor click loop: converts a touch on the isometric ground grid
-- back into a world-space (x, z) direction and rotates the base to
-- point at it (aim only -- doesn't move shoulder/elbow/wrist). This
-- inverts the same isometric transform used in renderer.lua's ground
-- grid, so if you change that projection, update this too.
local ISO_COS = math.cos(math.rad(30))
local ISO_SIN = math.sin(math.rad(30))

local function monitorClickLoop(robot, kinematics, renderer)
    while true do
        local ev, side, cx, cy = os.pullEvent("monitor_touch")

        local maxReach = kinematics.maxReach(robot.state.lengths)
        local mon = peripheral.wrap(config.MONITOR_SIDE)
        local w, h = mon.getSize()
        local viewW = math.floor(w * 0.65)
        local originX = math.floor(viewW * 0.5)
        local originY = math.floor(h * 0.65)
        local scale = (viewW * 0.4) / math.max(maxReach, 1)

        if cx <= viewW then
            if robot.state.busy then
                print("Ignoring click -- base is still moving. Wait for 'idle'.")
            else
                -- Invert the ground-plane (y=0) isometric projection:
                --   a = x - z, b = x + z  ->  x = (a+b)/2, z = (b-a)/2
                local a = (cx - originX) / (ISO_COS * scale)
                local b = (cy - originY) / (ISO_SIN * 0.5 * scale)
                local worldX = (a + b) / 2
                local worldZ = (b - a) / 2

                if math.abs(worldX) > 0.01 or math.abs(worldZ) > 0.01 then
                    local targetAngle = math.deg(math.atan2(worldZ, worldX))
                    renderer.setTarget({ x = worldX, y = 0, z = worldZ })
                    local ok, err = robot.moveJoint("base", targetAngle)
                    if not ok then print("base move failed: " .. tostring(err)) end
                end
            end
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
