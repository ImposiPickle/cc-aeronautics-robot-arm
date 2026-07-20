-- Minimal base-joint controller. No config file, no modules.
-- Edit the two values below if needed, then save as startup.lua.

local GEARSHIFT_SIDE = "right"   -- confirmed from your `peripherals` output
local PROTOCOL = "arm_control"
local HOSTNAME = "joint_base"

-- No chunking here on purpose. A working reference implementation of
-- this exact setup does the whole move in ONE rotate() call and just
-- waits on isRunning() -- Create interpolates the contraption's motion
-- smoothly on its own. If movement pulses/stutters, that's the kinetic
-- input speed (RPM) being too high for the contraption to keep up with,
-- not something to work around in software -- gear it down instead
-- (Large Cogwheel reduction, or lower a Rotation Speed Controller /
-- Creative Motor's target speed).

local gearshift = peripheral.wrap(GEARSHIFT_SIDE)
if not gearshift then error("No gearshift on side '" .. GEARSHIFT_SIDE .. "'") end

-- The Swivel Bearing must be ASSEMBLED (turned into a Simulated
-- Contraption) before spinning it actually moves anything -- otherwise
-- the input cog just spins freely with nothing attached to rotate.
local bearing = peripheral.find("swivel_bearing")
if bearing then
    if bearing.isAssembled() then
        print("Swivel bearing already assembled.")
    else
        print("Assembling swivel bearing...")
        local ok, err = pcall(function() bearing.assemble() end)
        if ok and bearing.isAssembled() then
            print("Assembled successfully.")
        else
            print("Assembly failed: " .. tostring(bearing.getLastAssemblyException and bearing.getLastAssemblyException() or err))
            print("Check nothing is blocking the arm's rotation path, then rerun startup.")
        end
    end
else
    print("WARNING: no swivel_bearing peripheral found -- rotation may spin freely with nothing attached.")
end

local modem = peripheral.find("modem", function(_, p) return p.isWireless and p.isWireless() end)
if not modem then modem = peripheral.find("modem") end
if not modem then error("No modem found") end

rednet.open(peripheral.getName(modem))
rednet.host(PROTOCOL, HOSTNAME)
print("Base joint online. Gearshift found on '" .. GEARSHIFT_SIDE .. "'.")
print("Rednet open on '" .. peripheral.getName(modem) .. "', hosting as '" .. HOSTNAME .. "'")

local currentAngle = 0

-- ---------------------------------------------------------------
-- Free-spin mode: press an arrow key to start spinning continuously
-- in that direction, press any key to stop. Runs alongside the
-- rednet move listener below -- handy for testing/calibrating RPM
-- without needing the Master to be involved at all.
--
-- IMPORTANT: this issues ONE big rotate() call rather than repeated
-- small chunks. Chunking causes a visible stop/restart pulse every
-- chunk (Create decelerates to a full stop at the end of each
-- instruction, then reaccelerates for the next one) -- a single call
-- lets Create interpolate the motion smoothly the whole way, same as
-- a normal move-to-target. "Stopping" works by issuing a new, tiny
-- rotate() call, which overrides/interrupts whatever instruction is
-- currently active. If this doesn't actually halt movement on your
-- setup, tell me and we'll find another way to cancel it.
-- ---------------------------------------------------------------
local SPIN_ANGLE = 100000 -- large enough to be "indefinite" in practice

local spinning = false
local spinDir = 1

local function startSpin(dir)
    spinning = true
    spinDir = dir
    gearshift.rotate(SPIN_ANGLE, dir)
end

local function stopSpin()
    spinning = false
    gearshift.rotate(0.01, spinDir) -- tiny instruction overrides the running one
end

local function freeSpinManager()
    print("Press UP to spin forward, DOWN to spin backward, any key to stop.")
    while true do
        local ev, key = os.pullEvent("key")
        if not spinning then
            if key == keys.up then
                print("Free-spin: forward.")
                startSpin(1)
            elseif key == keys.down then
                print("Free-spin: backward.")
                startSpin(-1)
            end
        else
            print("Free-spin: stopped.")
            stopSpin()
        end
    end
end

-- ---------------------------------------------------------------
-- Rednet move listener (moves to an absolute target angle, as
-- commanded by the Master). Note: if free-spin is active when a
-- move command arrives, both will try to drive the gearshift at
-- once -- stop free-spin before sending Master commands.
-- ---------------------------------------------------------------
local function rednetLoop()
    while true do
        local senderId, msg = rednet.receive(PROTOCOL)
        if type(msg) == "table" and msg.type == "move" then
            local target = msg.angle
            local delta = ((target - currentAngle) % 360)
            if delta > 180 then delta = delta - 360 end

            print("Moving base: " .. currentAngle .. " -> " .. target .. " (delta " .. delta .. ")")

            local angle = math.abs(delta)
            local dir = delta < 0 and -1 or 1

            if angle > 0.01 then
                gearshift.rotate(angle, dir)
                while gearshift.isRunning() do sleep(0.1) end

                -- isRunning() going false only means the GEARSHIFT's
                -- instruction finished -- the assembled contraption's
                -- actual angle can still be catching up for a moment.
                -- Wait for it to settle before acking.
                if bearing then
                    local lastReading = bearing.getTargetAngle()
                    local stableTicks = 0
                    local deadline = os.clock() + 5
                    while stableTicks < 3 and os.clock() < deadline do
                        sleep(0.1)
                        local reading = bearing.getTargetAngle()
                        if math.abs(reading - lastReading) < 0.05 then
                            stableTicks = stableTicks + 1
                        else
                            stableTicks = 0
                        end
                        lastReading = reading
                    end
                end
            end

            currentAngle = target
            print("Base now at " .. currentAngle)
            rednet.send(senderId, { type = "ack", angle = currentAngle }, PROTOCOL)
        end
    end
end

parallel.waitForAny(rednetLoop, freeSpinManager)
