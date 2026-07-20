-- Minimal base-joint controller. No config file, no modules.
-- Edit the two values below if needed, then save as startup.lua.

local GEARSHIFT_SIDE = "right"   -- confirmed from your `peripherals` output
local PROTOCOL = "arm_control"
local HOSTNAME = "joint_base"

-- Chunking controls -- THESE are the actual "pulse" settings for this
-- script. Each move is split into steps of at most CHUNK_DEGREES,
-- with a pause of CHUNK_PAUSE seconds between each one so the
-- assembled contraption has time to physically catch up.
local CHUNK_DEGREES = 10   -- max degrees moved per rotate() call
local CHUNK_PAUSE = 0.2    -- seconds paused between chunks

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
            local remaining = angle
            while remaining > 0.01 do
                local step = math.min(remaining, CHUNK_DEGREES)
                gearshift.rotate(step, dir)
                while gearshift.isRunning() do sleep(0.1) end
                remaining = remaining - step
                if remaining > 0.01 then sleep(CHUNK_PAUSE) end
            end

            -- isRunning() going false only means the GEARSHIFT's
            -- instruction finished -- the assembled contraption's
            -- actual angle can still be catching up. Wait for it to
            -- settle before acking.
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
