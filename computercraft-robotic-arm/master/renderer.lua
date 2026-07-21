-- renderer.lua (MASTER)
-- Draws a live ISOMETRIC 3D schematic of the arm on a monitor: a ground
-- grid, the arm rising off it as a bent line, and dashed projection
-- lines showing the end effector's horizontal offset (red) and height
-- (green) -- plus a text readout of angles, coordinates, and status.
-- Always renders from robot.state (the model) -- never estimates or
-- extrapolates from prior frames.

local config = require("config")

local renderer = {}

local mon, w, h
local originX, originY, scale -- world-to-screen isometric transform
local targetMarker = nil      -- {x=,y=,z=} set by ui.lua, or nil

-- Standard 2:1 isometric angle constants.
local ISO_COS = math.cos(math.rad(30))
local ISO_SIN = math.sin(math.rad(30))

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
-- character cells are roughly 2:1 (taller than wide); we compensate for
-- that in the Y axis of the isometric transform below, not here.
local function px(x, y, color)
    x, y = math.floor(x + 0.5), math.floor(y + 0.5)
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

-- Dashed line: same as `line`, but skips alternating chunks of pixels.
local function dashedLine(x0, y0, x1, y1, color, dashLen)
    dashLen = dashLen or 2
    x0, y0, x1, y1 = math.floor(x0), math.floor(y0), math.floor(x1), math.floor(y1)
    local dx, dy = math.abs(x1 - x0), -math.abs(y1 - y0)
    local sx = x0 < x1 and 1 or -1
    local sy = y0 < y1 and 1 or -1
    local err = dx + dy
    local count = 0
    while true do
        if math.floor(count / dashLen) % 2 == 0 then px(x0, y0, color) end
        count = count + 1
        if x0 == x1 and y0 == y1 then break end
        local e2 = 2 * err
        if e2 >= dy then err = err + dy; x0 = x0 + sx end
        if e2 <= dx then err = err + dx; y0 = y0 + sy end
    end
end

-- Small circle outline, used as a marker/handle around the effector.
local function circle(cx, cy, r, color)
    local steps = 48
    for i = 0, steps do
        local a = (i / steps) * 2 * math.pi
        px(cx + r * math.cos(a), cy + r * math.sin(a) * 0.5, color) -- *0.5 to offset cell aspect ratio
    end
end

-- World (x, y-up, z) -> isometric screen coordinates.
local function worldToScreenIso(x, y, z)
    local isoX = (x - z) * ISO_COS
    local isoY = (x + z) * ISO_SIN * 0.5 - y -- *0.5 to offset cell aspect ratio
    return originX + isoX * scale, originY + isoY * scale
end

-- Draws a diamond-shaped ground grid centered on the arm's base.
local function drawGrid(halfExtent, divisions)
    local step = halfExtent / divisions
    for i = -divisions, divisions do
        local a1x, a1y = worldToScreenIso(-halfExtent, 0, i * step)
        local a2x, a2y = worldToScreenIso(halfExtent, 0, i * step)
        line(a1x, a1y, a2x, a2y, colors.gray)

        local b1x, b1y = worldToScreenIso(i * step, 0, -halfExtent)
        local b2x, b2y = worldToScreenIso(i * step, 0, halfExtent)
        line(b1x, b1y, b2x, b2y, colors.gray)
    end
end

-- Draws the isometric 3D view: ground grid, arm skeleton, and dashed
-- horizontal/vertical projection lines from the base to the effector.
local function drawIsometric(robotState, kinematics)
    local positions = kinematics.forward(robotState.angles, robotState.lengths)
    local maxReach = kinematics.maxReach(robotState.lengths)

    local viewW = math.floor(w * 0.65)
    originX = math.floor(viewW * 0.5)
    originY = math.floor(h * 0.65)
    scale = (viewW * 0.4) / math.max(maxReach, 1)

    drawGrid(maxReach, 5)

    local bx, by = worldToScreenIso(0, 0, 0)
    local ex, ey = worldToScreenIso(positions.elbow.x, positions.elbow.y, positions.elbow.z)
    local wx, wy = worldToScreenIso(positions.wrist.x, positions.wrist.y, positions.wrist.z)
    local tx, ty = worldToScreenIso(positions.effector.x, positions.effector.y, positions.effector.z)

    -- Dashed projection lines: red = horizontal offset (ground plane,
    -- base to directly beneath the effector), green = height (straight
    -- up from that ground point to the effector).
    local groundX, groundY = worldToScreenIso(positions.effector.x, 0, positions.effector.z)
    dashedLine(bx, by, groundX, groundY, colors.red, 2)
    dashedLine(groundX, groundY, tx, ty, colors.lime, 2)

    -- Arm skeleton.
    line(bx, by, ex, ey, colors.lightBlue)
    line(ex, ey, wx, wy, colors.lightBlue)
    line(wx, wy, tx, ty, colors.lightBlue)

    px(bx, by, colors.white)
    px(ex, ey, colors.white)
    px(wx, wy, colors.white)

    -- Small circle/handle around the effector.
    circle(tx, ty, 4, robotState.gripper == "open" and colors.lime or colors.red)
    px(tx, ty, colors.white)

    if targetMarker then
        local mx, my = worldToScreenIso(targetMarker.x, 0, targetMarker.z)
        px(mx, my, colors.yellow)
    end

    return viewW
end

-- Draws the text status panel to the right of the isometric view.
local function drawStatus(robotState, kinematics, viewW)
    local positions = kinematics.forward(robotState.angles, robotState.lengths)
    local col = viewW + 2
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
    local viewW = drawIsometric(robotState, kinematics)
    drawStatus(robotState, kinematics, viewW)
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
