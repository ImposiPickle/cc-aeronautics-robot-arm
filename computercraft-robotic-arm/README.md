# ComputerCraft Robotic Arm

Implementation of the modular arm control system: one Master computer
running kinematics/planning/UI/rendering, and one small computer per
joint doing only local motor control.

## Folder layout

```
master/     -- put all of these files on the Master computer (root dir)
  startup.lua
  robot.lua
  joint.lua
  kinematics.lua
  planner.lua
  renderer.lua
  ui.lua
  record.lua
  config.lua

joint/      -- copy this whole folder onto EACH joint computer
  startup.lua
  gearshift.lua
  config.lua
```

## Setup

1. **Master computer**: attach a wireless modem and a monitor. Copy all
   files from `master/` into its root directory. Edit `master/config.lua`:
   - `MONITOR_SIDE` to match where you placed the monitor
   - `JOINTS` hostnames/limits/home angles to match your build
   - `LINK_LENGTHS` to match your arm's actual block lengths

2. **Each joint computer** (base, shoulder, elbow, wrist, gripper):
   attach a wireless modem and wire it to that joint's Sequenced
   Gearshift. Copy all files from `joint/` into its root directory, then
   edit `joint/config.lua`:
   - `JOINT_NAME` — must exactly match the hostname used in
     `master/config.lua` (e.g. `"joint_base"`)
   - `IS_GRIPPER = true` on the gripper computer only
   - `GEARSHIFT_MODE`, `REDSTONE_SIDE` / `PERIPHERAL_SIDE`,
     `DEGREES_PER_PULSE` — see the note below, this is the one part
     you'll need to adapt to your exact build

3. Boot the joint computers first (they `rednet.host` themselves and
   wait), then boot the Master.

## About `joint/gearshift.lua`

Sequenced Gearshifts in Create aren't natively exposed to ComputerCraft
— there's no vanilla peripheral for "rotate to angle X." This file gives
you two starting points:

- **`redstone` mode** (default): pulses a redstone side some number of
  times, where each pulse advances the gearshift by
  `DEGREES_PER_PULSE`. Works with a Redstone Link or Rotation Speed
  Controller wired to step the gearshift per pulse. You'll need to
  calibrate `DEGREES_PER_PULSE` against your actual gear ratio.
- **`peripheral` mode**: calls directly into a wrapped peripheral, for
  setups using a Create/CC bridge addon. The method names in there are
  placeholders — swap them for whatever your addon actually exposes.

This is the one piece of the system whose exact implementation depends
on hardware/addon choices outside a stock CC:Tweaked install, so it's
intentionally isolated to a single, clearly-commented file.

## Using it

Once running, the Master's terminal shows manual jog controls
(TAB/UP/DOWN/g/h/s/q) and the monitor shows a live top-down schematic
with joint angles, effector coordinates, target marker, reach boundary,
and motion status. Clicking the monitor's plan view drives the arm
there via inverse kinematics.

From the Lua console or your own scripts (after `robot.init()`):

```lua
robot.moveJoint("base", 45)
robot.moveJoint("shoulder", 30)
robot.moveTo(4, 2, 3)
robot.home()
robot.openGripper()
robot.closeGripper()

local record = require("record")
record.save("pick_pose", robot.state.angles)
record.play("pick_pose", robot)
record.playSequence({ "home_pose", "pick_pose", "place_pose" }, robot)
```

## Notes / next steps

- IK currently assumes a level (horizontal) gripper approach; extend
  `kinematics.inverse` if you want controllable approach angle/orientation.
- `planner.lua` interpolates linearly in joint space — fine for this
  arm, but swap in your own easing if you want acceleration curves.
- Collision checking, 3D visualization, and airship-relative coordinates
  (listed as future features in the design doc) aren't implemented —
  the module boundaries here (kinematics/planner/renderer) are where
  they'd plug in.
