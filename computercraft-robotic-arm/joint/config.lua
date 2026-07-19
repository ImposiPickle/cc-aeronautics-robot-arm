-- config.lua (JOINT COMPUTER)
-- Copy this whole `joint/` folder onto EVERY joint computer, then edit
-- JOINT_NAME (and the gearshift settings below) for that specific
-- computer before running startup.lua.

local config = {}

-- Must match one of config.JOINTS' hostnames in master/config.lua,
-- e.g. "joint_base", "joint_shoulder", "joint_elbow", "joint_wrist",
-- or "joint_gripper" for the gripper computer.
config.JOINT_NAME = "joint_base"

-- Must match master/config.lua's config.PROTOCOL exactly.
config.PROTOCOL = "arm_control"

-- Modem side on this computer. nil = auto-detect first modem found.
config.MODEM_SIDE = nil

-- true if this computer runs the gripper instead of a rotating joint.
config.IS_GRIPPER = false

-- ---- Sequenced Gearshift hardware settings ----
-- The exact peripheral API for a Create Sequenced Gearshift depends on
-- your modpack/CC-Create bridge (vanilla Create does not expose one to
-- ComputerCraft on its own -- you likely need an addon, or you are
-- driving it via redstone). See joint/gearshift.lua for the two
-- implementations provided (PERIPHERAL and REDSTONE) and pick/adjust
-- the one that matches your setup.
config.GEARSHIFT_MODE = "redstone" -- "redstone" or "peripheral"

-- --- redstone mode settings ---
-- Side facing the gearshift's redstone input (e.g. a Redstone Link or
-- a Rotation Speed Controller wired to accept pulses).
config.REDSTONE_SIDE = "back"
-- How many degrees one pulse rotates the bearing. Calibrate this against
-- your actual gear ratio / Sequenced Gearshift program.
config.DEGREES_PER_PULSE = 3.75  -- e.g. 96 pulses per full 360 rotation
config.PULSE_LENGTH = 0.1        -- seconds the redstone signal stays high
config.PULSE_GAP = 0.1           -- seconds between pulses

-- --- peripheral mode settings ---
-- Name/side of the peripheral, if your Create bridge exposes one
-- directly (e.g. peripheral.wrap("back")). Adjust method names inside
-- gearshift.lua's PERIPHERAL implementation to match your API.
config.PERIPHERAL_SIDE = "back"

-- File used to persist this joint's last known angle across reboots
-- (so the joint computer can report its true position on boot even
-- before receiving a command).
config.STATE_FILE = "joint_state.dat"

return config
