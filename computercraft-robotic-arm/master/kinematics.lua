-- kinematics.lua (MASTER)
-- Forward and inverse kinematics for the 4-DOF arm (base yaw + shoulder,
-- elbow, wrist pitch, all in the plane defined by the base rotation).
--
-- Convention:
--   base angle     -- rotation around the vertical (Y) axis
--   shoulder angle  -- pitch from horizontal, measured at the shoulder joint
--   elbow angle     -- pitch relative to the upper arm, measured at the elbow
--   wrist angle     -- pitch relative to the forearm, measured at the wrist
--
-- World coordinates: X/Z horizontal plane, Y vertical (matches Minecraft).

local kinematics = {}

local function rad(d) return d * math.pi / 180 end
local function deg(r) return r * 180 / math.pi end

-- Rotates a point (x, 0, z) that lies in the arm's local vertical plane
-- out into world space by the base angle.
local function rotateByBase(x, y, baseAngleDeg)
    local b = rad(baseAngleDeg)
    return { x = x * math.cos(b), y = y, z = x * math.sin(b) }
end

-- Forward kinematics: given joint angles and link lengths, returns the
-- world-space position of every joint plus the end effector.
-- angles = { base=, shoulder=, elbow=, wrist= } (degrees)
-- lengths = { upperArm=, forearm=, wrist= }
function kinematics.forward(angles, lengths)
    local shoulder = rad(angles.shoulder)
    local elbowAbs = rad(angles.shoulder + angles.elbow)
    local wristAbs = rad(angles.shoulder + angles.elbow + angles.wrist)

    -- Planar (pre-base-rotation) positions in the arm's local X-Y plane.
    local ex = lengths.upperArm * math.cos(shoulder)
    local ey = lengths.upperArm * math.sin(shoulder)

    local wx = ex + lengths.forearm * math.cos(elbowAbs)
    local wy = ey + lengths.forearm * math.sin(elbowAbs)

    local tx = wx + lengths.wrist * math.cos(wristAbs)
    local ty = wy + lengths.wrist * math.sin(wristAbs)

    return {
        base     = { x = 0, y = 0, z = 0 },
        shoulder = { x = 0, y = 0, z = 0 },
        elbow    = rotateByBase(ex, ey, angles.base),
        wrist    = rotateByBase(wx, wy, angles.base),
        effector = rotateByBase(tx, ty, angles.base),
    }
end

-- Returns the maximum horizontal+vertical reach of the arm (fully
-- extended), used by the renderer to draw a reach boundary.
function kinematics.maxReach(lengths)
    return lengths.upperArm + lengths.forearm + lengths.wrist
end

-- Inverse kinematics: given a target world position, returns joint
-- angles that place the end effector there, or nil + reason on failure.
--
-- Approach: solve base from the horizontal direction to the target,
-- reduce to a 2-link (upperArm/forearm) planar problem for the wrist
-- joint's position (target pulled back by the wrist link length along
-- the approach direction), solve shoulder/elbow via law of cosines, then
-- set the wrist angle so the gripper approaches level (horizontal).
--
-- `limits` (optional) = config.JOINTS, used to clamp/validate the result.
function kinematics.inverse(target, lengths, limits)
    local baseAngle = deg(math.atan2(target.z, target.x))

    local planarDist = math.sqrt(target.x ^ 2 + target.z ^ 2)
    local height = target.y

    -- Pull the target back by the wrist link length, assuming a level
    -- (horizontal) final approach angle for the gripper.
    local wx = planarDist - lengths.wrist
    local wy = height

    local reach = math.sqrt(wx ^ 2 + wy ^ 2)
    local maxReach = lengths.upperArm + lengths.forearm
    local minReach = math.abs(lengths.upperArm - lengths.forearm)

    if reach > maxReach then
        return nil, "target out of reach (needs " .. string.format("%.1f", reach) ..
            ", max " .. string.format("%.1f", maxReach) .. ")"
    end
    if reach < minReach then
        return nil, "target too close to base joint"
    end

    -- Law of cosines for the elbow angle.
    local cosElbow = (lengths.upperArm ^ 2 + lengths.forearm ^ 2 - reach ^ 2) /
        (2 * lengths.upperArm * lengths.forearm)
    cosElbow = math.max(-1, math.min(1, cosElbow))
    local elbowInterior = math.acos(cosElbow)
    local elbowAngle = deg(elbowInterior) - 180 -- convert to our relative-angle convention

    -- Shoulder angle: angle to the wrist point, offset by the triangle's
    -- interior angle at the shoulder.
    local angleToWrist = math.atan2(wy, wx)
    local cosShoulderOffset = (lengths.upperArm ^ 2 + reach ^ 2 - lengths.forearm ^ 2) /
        (2 * lengths.upperArm * reach)
    cosShoulderOffset = math.max(-1, math.min(1, cosShoulderOffset))
    local shoulderOffset = math.acos(cosShoulderOffset)
    local shoulderAngle = deg(angleToWrist + shoulderOffset)

    -- Keep the gripper level: wrist compensates for shoulder+elbow pitch.
    local wristAngle = -(shoulderAngle + elbowAngle)

    local result = { base = baseAngle, shoulder = shoulderAngle, elbow = elbowAngle, wrist = wristAngle }

    if limits then
        for name, angleVal in pairs(result) do
            local lim = limits[name]
            if lim and (angleVal < lim.minAngle or angleVal > lim.maxAngle) then
                return nil, name .. " angle " .. string.format("%.1f", angleVal) ..
                    " outside limits [" .. lim.minAngle .. ", " .. lim.maxAngle .. "]"
            end
        end
    end

    return result
end

return kinematics
