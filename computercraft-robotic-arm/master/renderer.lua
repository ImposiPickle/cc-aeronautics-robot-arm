-- renderer.lua (MASTER)
-- Draws a live top-down (X/Z) schematic of the arm on a monitor, plus a
-- text readout of joint angles, end-effector coordinates, target, and
-- motion status. Always renders from robot.state (the model) -- never
-- estimates or extrapolates from prior frames.

local config = require("config")

local renderer = {}

local mon, w, h
local originX, originZ, scale -- world-to-screen transform
local targetMarker = nil      -- {x=,y=,z=} set by ui.lua, or nil

function renderer.setTarget(t) targetMarker = t end

function renderer.init()
    mon = peripheral.wrap(config.MONITOR_SIDE)
    if not mon then
        error("No monitor found on side '" .. config.MONITOR_SIDE .. "'")
    end
    mon.setTextScale(0.5)
    w, h = mon.getSize()
    mon.setBackgroundColor(colors.black)
    mon.clear()
end

-- Draws a single "pixel" using a space with a background colour. Monitor
-- character cells are roughly 2:1 (taller than wide); we don't attempt
-- sub-cell precision.
local function px(x, y, color)
    if x < 1 or y < 1 or x > w or y > h then return end
    mon.setCursorPos(x, y)
    mon.setBackgroundColor(color)
    mon.write(" ")
end

-- Bresenham line between two screen points.
local function line(x0, y0, x1, y1, color)
    x0, y0, x1, y1 = math.floor(x0), math.floor(y0), math.floor(x1), math.floor(y1)
    local dx, dy = math.abs(x1 - x0), -math.abs(y1 - y0)
    local sx = x0 < x1 and 1 or -1
    local sy = y0 < y1 and 1 or -1
    local err = dx + dy
    while true do
        px(x0, y0, color)
        if x0 == x1 and y0 == y1 then break end
        local e2 = 2 * err
        if e2 >= dy then err = err + dy; x0 = x0 + sx end
        if e2 <= dx then err = err + dx; y0 = y0 + sy end
    end
end

-- Rough circle outline, used for the reach boundary.
local function circle(cx, cy, r, color)
    local steps = 90
    for i = 0, steps do
        local a = (i / steps) * 2 * math.pi
        px(cx + r * math.cos(a), cy + r * math.sin(a) * 0.5, color) -- *0.5 to offset cell aspect ratio
    end
end

local function worldToScreen(x, z)
    return originX + x * scale, (h * 0.5) + z * scale
end

-- Draws the plan (top-down) view: base at a fixed screen anchor, arm
-- projected onto the horizontal plane, reach boundary, target marker.
local function drawPlan(robotState, kinematics)
    local positions = kinematics.forward(robotState.angles, robotState.lengths)
    local maxReach = kinematics.maxReach(robotState.lengths)

    local planW = math.floor(w * 0.6)
    originX = math.floor(planW * 0.5)
    scale = (planW * 0.45) / math.max(maxReach, 1)

    circle(originX, h * 0.5, maxReach * scale, colors.gray)

    local bx, by = worldToScreen(0, 0)
    local ex, ey = worldToScreen(positions.elbow.x, positions.elbow.z)
    local wx, wy = worldToScreen(positions.wrist.x, positions.wrist.z)
    local tx, ty = worldToScreen(positions.effector.x, positions.effector.z)

    line(bx, by, ex, ey, colors.lightBlue)
    line(ex, ey, wx, wy, colors.blue)
    line(wx, wy, tx, ty, colors.cyan)

    px(bx, by, colors.white)
    px(ex, ey, colors.white)
    px(wx, wy, colors.white)
    px(tx, ty, robotState.gripper == "open" and colors.lime or colors.red)

    if targetMarker then
        local mx, my = worldToScreen(targetMarker.x, targetMarker.z)
        px(mx, my, colors.yellow)
    end

    return planW
end

-- Draws the text status panel to the right of the plan view.
local function drawStatus(robotState, kinematics, planW)
    local positions = kinematics.forward(robotState.angles, robotState.lengths)
    local col = planW + 2
    local row = 1

    local function line_(text, color)
        mon.setCursorPos(col, row)
        mon.setTextColor(color or colors.white)
        mon.setBackgroundColor(colors.black)
        mon.write(text)
        row = row + 1
    end

    line_("== ARM STATUS ==", colors.white)
    row = row + 1
    line_(string.format("base:     %6.1f", robotState.angles.base))
    line_(string.format("shoulder: %6.1f", robotState.angles.shoulder))
    line_(string.format("elbow:    %6.1f", robotState.angles.elbow))
    line_(string.format("wrist:    %6.1f", robotState.angles.wrist))
    row = row + 1
    line_(string.format("x: %6.2f", positions.effector.x))
    line_(string.format("y: %6.2f", positions.effector.y))
    line_(string.format("z: %6.2f", positions.effector.z))
    row = row + 1
    line_("gripper: " .. robotState.gripper, robotState.gripper == "open" and colors.lime or colors.red)
    row = row + 1
    if robotState.busy then
        line_("status: MOVING", colors.yellow)
    else
        line_("status: idle", colors.lime)
    end
    if robotState.lastError then
        line_("err: " .. tostring(robotState.lastError), colors.red)
    end
end

-- Renders one frame from the given robot state. `kinematics` is passed
-- in to avoid a circular require between robot.lua and renderer.lua.
function renderer.draw(robotState, kinematics)
    mon.setBackgroundColor(colors.black)
    mon.clear()
    local planW = drawPlan(robotState, kinematics)
    drawStatus(robotState, kinematics, planW)
end

-- Runs forever, redrawing at ~20 FPS whenever robot state changes or on
-- a fixed tick, whichever comes first. Intended to be run inside
-- parallel.waitForAny alongside the UI loop.
function renderer.loop(robot, kinematics)
    renderer.init()
    local FRAME_TIME = 0.05 -- ~20 FPS
    while true do
        renderer.draw(robot.state, kinematics)
        sleep(FRAME_TIME)
    end
end

return renderer
