-- startup.lua (MASTER)
-- Entry point for the Master computer. Wires together robot, renderer,
-- and ui, and keeps them running in parallel.
--
-- File layout expected on this computer (all in the root, or adjust the
-- `package.path` line below to match wherever you place them):
--   startup.lua  robot.lua  joint.lua  kinematics.lua
--   planner.lua  renderer.lua  ui.lua  record.lua  config.lua

package.path = "/?.lua;" .. package.path

local robot      = require("robot")
local kinematics = require("kinematics")
local renderer   = require("renderer")
local ui         = require("ui")

print("Initializing arm master...")
robot.init()
print("Rednet open, robot state ready.")
print("Waiting to discover joint computers on first move...")
print("")

parallel.waitForAny(
    function() renderer.loop(robot, kinematics) end,
    function() ui.loop(robot, kinematics, renderer) end
)

print("Arm master stopped.")
