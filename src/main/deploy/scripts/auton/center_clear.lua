
--Change this edit the ring pick-up order
ring_order = {4,5,3}
max_speed = 3.0
intake_speed = 3.0
--TODO: Filter out invalid rings IE. <1 and >5
intake_system:setVoltage(-5.5)


io.write("Picking up rings in the following order: ")
printTable(ring_order, ", ")
print("Ring " .. tostring(ring_order[#ring_order]) .. " will be held at the end")

SHOOT_LINE_X = inchesToMeters(230.19)
SCAN_LINE_X = inchesToMeters(250.0) --Between line I and E
RING_SPIT_LOCATIONS = {
    [1] = {
        ["x"] = SHOOT_LINE_X,
        ["y"] = inchesToMeters(99),
        ["angle"] = 45
    },
    [2] = {
        ["x"] = SHOOT_LINE_X,
        ["y"] = inchesToMeters(99),
        ["angle"] = 45
    },
    [3] = {
        ["x"] = SHOOT_LINE_X,
        ["y"] = CENTER_RING_Y_CORDINATES[5] + inchesToMeters(33),
        ["angle"] = 0
    },
    [4] = {
        ["x"] = SHOOT_LINE_X,
        ["y"] = CENTER_RING_Y_CORDINATES[5] + inchesToMeters(33),
        ["angle"] = 0
    },
    [5] = {
        ["x"] = SHOOT_LINE_X,
        ["y"] = CENTER_RING_Y_CORDINATES[5] + inchesToMeters(33),
        ["angle"] = 0
    }
}
SPIT_SHOOTER_SPEED = 8.0 --Volts
SPIT_ARM_ANGLE = -22.0 --Degrees
SHOOT_DELAY = 0.35 --Seconds
INTAKE_ARM_ANGLE = -22.0 --Degrees

function spitRing(ring_index)
    shooter_system:setTargetSpeed(SPIT_SHOOTER_SPEED) --Turn on the shooter
    shooter_system:setTargetArmAngle(SPIT_ARM_ANGLE) --Set arm angle
    --Drive to the corresponding spit location and angle
    driveToAlliance(
        RING_SPIT_LOCATIONS[ring_order[ring_index]]["x"],
        RING_SPIT_LOCATIONS[ring_order[ring_index]]["y"],
        RING_SPIT_LOCATIONS[ring_order[ring_index]]["angle"]
    )
    waitUntilDistance(inchesToMeters(2))
    drive_system:stop()
    waitUntilArmAngle(SPIT_ARM_ANGLE)
    intake_system:feed()--Shoot
    control:sleep(SHOOT_DELAY)
    intake_system:stop()
    shooter_system:setTargetSpeed(0.0)
    driveToAlliance(SCAN_LINE_X, RING_SPIT_LOCATIONS[ring_order[ring_index]]["y"], 0)--Drive to the scan line
    waitUntilDistance(inchesToMeters(2))
end

setOdometryAlliance(inchesToMeters(58.5), inchesToMeters(99.0), 0.0)
drive_system:setMaxDriveSpeed(1.5)--Slowed down for initial testing, set to 3 after paths tested to not crash into things
drive_system:setMinDrivePercentage(0.25)


--first ring
drive_system:setMaxDriveSpeed(max_speed)
drive_system:setMinDrivePercentage(0.25)
shooter_system:setTargetArmAngle(INTAKE_ARM_ANGLE)


--Drive to path branch point
driveToAlliance(inchesToMeters(76.1), inchesToMeters(99), 0)
waitUntilDistance(inchesToMeters(2))



--Set up for first ring, try to get there fast
--Path for if ring 1 or 2 are first
if tableContains({1, 2}, ring_order[1]) then --Hint, if a table in lua is defined with a literal, its first index will be "1" not "0"
    print("Taking left path")
    angle = 0
    if ring_order[1] == 1 then
        angle = 45 --Turn to ring 1 while we travel to point
    else
        angle = -45 --Turn to ring 2 while we travel to point
    end
    driveToAlliance(SCAN_LINE_X, inchesToMeters(66.0 + 16.5), angle)
    waitUntilDistance(inchesToMeters(2))
else --Path for if ring 3, 4 or 5 are first
    print("Taking right path")
    angle = 0
    if ring_order[1] == 3 then
        angle = 0
    elseif ring_order[1] == 4 then
        angle = -45
    else --ring 5
        angle = -45
    end
    print("turning to: " .. tostring(angle))
    driveToAlliance(inchesToMeters(230.19), inchesToMeters(0.0), angle)
    waitUntilDistance(inchesToMeters(2))
    drive_system:stop()
    print("under stage")
    if ring_order[1] == 5 then --Move a little more to the right for ring 5
        print("Ring 5 is first, reposition to bring it into frame")
        driveToAlliance(SCAN_LINE_X, inchesToMeters(-114.0 - 18.0), angle)
        waitUntilDistance(inchesToMeters(2))
    elseif ring_order[1] == 4 then
        print("Ring 4 is first, reposition to bring it into frame")
        driveToAlliance(inchesToMeters(250.0), inchesToMeters(0.0), angle)
        waitUntilDistance(inchesToMeters(2))
    end
end
CRASH()

--See if first ring is already taken, if so skip obtaining
if vision:ringInView() then
    print("I see first ring :D")
    getRing(intake_speed, CENTER_PICKUP_CUTOFF, 5.0)
    drive_system:setMaxDriveSpeed(max_speed)
    --If we have a ring, drive to spit it
    if intake_system:hasRing() then
        spitRing(1) --Drive to the spit location and spit  
    end
end
driveToAlliance(SCAN_LINE_X, CENTER_RING_Y_CORDINATES[ring_order[2]], 0)--Drive to the scan line
waitUntilDistance(inchesToMeters(2))

for current_ring_index = 2, #ring_order do --Loop through the rest of the rings
    shooter_system:setTargetArmAngle(INTAKE_ARM_ANGLE) --Get arm ready to intake
    current_ring = ring_order[current_ring_index]
    print("Going for ring " .. tostring(current_ring))
    --Got to the scan location
    driveToAlliance(SCAN_LINE_X, CENTER_RING_Y_CORDINATES[current_ring], 0) --Drive along scan line to new scan location
    waitUntilDistance(inchesToMeters(2))
    drive_system:stop()
    waitUntilArmAngle(INTAKE_ARM_ANGLE)
    if vision:ringInView() then --We see the ring
        getRing(intake_speed, CENTER_PICKUP_CUTOFF, 1.5) --Attempt to pick up the ring
        if intake_system:hasRing() and current_ring ~= #ring_order then --If we have a ring and it is not the last one in the list. No use in shooting it just to pick it back up
            spitRing(current_ring_index) --Drive to the spit location and spit
            driveToAlliance(SCAN_LINE_X, CENTER_RING_Y_CORDINATES[current_ring + 1], 0)--Drive to the scan line
        else
            driveToAlliance(SCAN_LINE_X, CENTER_RING_Y_CORDINATES[current_ring], 0)--Drive to the scan line
        end
        waitUntilDistance(inchesToMeters(2))
    end --Ring not seen, go to next one. We are already on the scan line at this time
end

drive_system:stop()

--Pick up and spit rings in order. Not doing last ring, no use in shooting it just to pick it back up


