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
-- Create (recent versions) exposes the Sequenced Gearshift directly as
-- a CC:Tweaked peripheral -- no addon, no redstone hack needed. It must
-- be placed directly adjacent to this computer (or connected via wired
-- modem + network cable) for peripheral.wrap/peripheral.find to see it.
-- See joint/gearshift.lua. A redstone-pulse fallback is also included
-- for older setups where the peripheral isn't available.
config.GEARSHIFT_MODE = "peripheral" -- "peripheral" (recommended) or "redstone"

-- --- peripheral mode settings ---
-- Side (or network name from `peripherals` / peripheral.getNames())
-- the Sequenced Gearshift is reachable at. Run `peripherals` on this
-- computer to see exactly what's attached and copy the name from there.
config.PERIPHERAL_SIDE = "back"

-- Whether positive angle deltas should be sent as modifier=1 or -1.
-- If the joint moves backwards from what you commanded, flip this.
config.INVERT_DIRECTION = false

-- Optional: side/network name of a swivel_bearing peripheral on this
-- same joint (Create: Aeronautics exposes these directly to CC). If
-- set, the joint computer reads the bearing's REAL angle via
-- getTargetAngle() after every move and reports that (not just its
-- own pulse-counted guess) back to the Master -- this stops any drift
-- from silently accumulating. Leave nil if you don't have one wired
-- to this joint, or the joint has no swivel_bearing peripheral.
config.SWIVEL_PERIPHERAL_SIDE = nil

-- Timeout (seconds) to wait for isRunning() to go false before giving
-- up on a move and reporting an error back to the Master.
config.MOVE_TIMEOUT = 8

-- --- redstone mode settings (fallback only) ---
config.REDSTONE_SIDE = "back"
config.DEGREES_PER_PULSE = 3.75
config.PULSE_LENGTH = 0.1
config.PULSE_GAP = 0.1

-- File used to persist this joint's last known angle across reboots
-- (so the joint computer can report its true position on boot even
-- before receiving a command).
config.STATE_FILE = "joint_state.dat"

return config
