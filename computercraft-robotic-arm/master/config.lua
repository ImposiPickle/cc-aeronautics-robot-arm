-- config.lua (MASTER)
-- Central configuration for the Master computer.
-- Adjust hostnames, joint limits, and link lengths to match your build.

local config = {}

-- Rednet protocol name shared by master and all joint computers
config.PROTOCOL = "arm_control"

-- Modem side on the Master computer. Set to nil to auto-detect the first
-- modem peripheral found.
config.MODEM_SIDE = nil

-- One entry per joint computer, matching the hostname each joint computer
-- registers itself under (see joint/config.lua -> JOINT_NAME).
-- minAngle/maxAngle are in degrees, homeAngle is the resting position.
config.JOINTS = {
    base     = { hostname = "joint_base",     minAngle = -180, maxAngle = 180, homeAngle = 0 },
    shoulder = { hostname = "joint_shoulder", minAngle = -10,  maxAngle = 100, homeAngle = 0 },
    elbow    = { hostname = "joint_elbow",    minAngle = -125, maxAngle = 125, homeAngle = 0 },
    wrist    = { hostname = "joint_wrist",    minAngle = -180, maxAngle = 180, homeAngle = 0 },
}

-- Ordered list of joints, base to tip. Used anywhere ordering matters
-- (rendering, sequential homing, etc).
config.JOINT_ORDER = { "base", "shoulder", "elbow", "wrist" }

-- Gripper is handled as a special two-state "joint".
config.GRIPPER = {
    hostname    = "joint_gripper",
    openState   = "open",
    closedState = "closed",
}

-- Link lengths in blocks. These drive both forward and inverse kinematics.
config.LINK_LENGTHS = {
    upperArm = 5,
    forearm  = 4,
    wrist    = 2,
}

-- Monitor the live view is drawn on.
config.MONITOR_SIDE = "top"

-- How long (seconds) the master waits for a joint to ack a move before
-- treating it as failed/stalled.
config.MOVE_TIMEOUT = 30

-- Number of interpolation steps used by planner.lua when generating a
-- smooth trajectory between two poses. Higher = smoother, slower.
config.TRAJECTORY_STEPS = 24

-- Maximum degrees any single joint may move in one trajectory step.
-- The planner will add extra steps if a naive split would exceed this.
config.MAX_STEP_DEGREES = 6

-- File used by record.lua to persist saved poses across reboots.
config.POSES_FILE = "poses.dat"

return config
