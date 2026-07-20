-- Minimal base-joint controller. No config file, no modules.
-- Edit the two values below if needed, then save as startup.lua.

local GEARSHIFT_SIDE = "right"   -- confirmed from your `peripherals` output
local PROTOCOL = "arm_control"
local HOSTNAME = "joint_base"

local gearshift = peripheral.wrap(GEARSHIFT_SIDE)
if not gearshift then error("No gearshift on side '" .. GEARSHIFT_SIDE .. "'") end

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
            gearshift.rotate(angle, dir)
            while gearshift.isRunning() do sleep(0.1) end
        end

        currentAngle = target
        print("Base now at " .. currentAngle)
        rednet.send(senderId, { type = "ack", angle = currentAngle }, PROTOCOL)
    end
end
