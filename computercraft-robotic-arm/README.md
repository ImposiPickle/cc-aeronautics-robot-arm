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

Recent versions of Create expose the Sequenced Gearshift **directly**
as a CC:Tweaked peripheral — no addon required. This is the default
(`GEARSHIFT_MODE = "peripheral"`):

- `rotate(angle, [modifier])` — rotates by a positive number of
  degrees; `modifier = -1` reverses direction.
- `isRunning()` — true while the shaft is still turning.

**Setup for peripheral mode:**
- Place the computer directly adjacent to the Sequenced Gearshift (or
  connect them over a wired modem network with network cable).
- Set `PERIPHERAL_SIDE` in `joint/config.lua` to the face the gearshift
  is on (or leave `nil` to auto-detect).
- **Leave the gearshift's own instruction list empty.** A gearshift
  currently linked to a computer ignores its programmed sequence — you
  don't need to (and shouldn't) also program "Turn by Angle" /
  "Await Redstone Signal" steps on it manually.
- If the joint rotates the wrong way, flip `INVERT_DIRECTION` in that
  joint's config rather than editing `gearshift.lua`.

A `redstone` mode is also included as a fallback for older Create/CC
versions where the peripheral isn't available — it pulses a redstone
side, with the gearshift itself programmed to advance one step per
pulse. This needs `DEGREES_PER_PULSE` calibrated to your build and only
supports one rotation direction as written.

## Swivel Bearing assembly (important, easy to miss)

A Swivel Bearing must be **assembled** into a Simulated Contraption
before rotating it moves anything physically attached — otherwise its
input cog just spins freely with nothing to actually turn. `startup.lua`
calls `gearshift.ensureBearingAssembled()` automatically on boot, which
right-clicks it in code (`bearing.assemble()`) if it isn't assembled
yet. If assembly keeps failing, check the console for the reason string
it prints — usually something physically blocking the parts trying to
become a contraption.

Separately: the bearing only rotates the contraption when driven
through the cog on its **side** — if a gearshift feeds into its
*center* shaft instead, rotation passes straight through without
turning the bearing itself.

## Gearing/RPM matters more than code

If a joint's motion pulses or stutters, that's very likely the kinetic
input speed (RPM) reaching the gearshift being too high for the
contraption to keep up with — not something to fix in software. Gear it
down (Large Cogwheel reduction, or lower a Rotation Speed Controller /
Creative Motor's target speed) rather than trying to chunk/throttle
moves in Lua. A single `rotate()` call per move is correct and Create
will interpolate the motion smoothly on its own at a sane RPM.

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
