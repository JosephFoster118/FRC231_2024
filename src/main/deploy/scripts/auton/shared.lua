print("Loading shared script assets")
--Helper functions

start_print_time_stamp = 0
function initStartTimeStamp()
    start_print_time_stamp = drive_system:getTime()
end
function timeStampPrint(message)
    time_passed = drive_system:getTime() - start_print_time_stamp
    print(tostring(time_passed) .. ": " .. message)
end

function waitUntilDistance(distance)
    control:waitUntilTrue( function ()
        return drive_system:getDistanceFromTarget() < distance
    end
    )
end

function waitUntilTarget()
    control:waitUntilTrue( function ()
        return drive_system:isAtTarget()
    end
    )
end

function waitUntilAimLocked(angle)
    control:waitUntilTrue(function()
        return (shooter_system:getArmAngle() > (angle - 4.0) and shooter_system:getArmAngle() < (angle + 4.0) and drive_system:speakerLockedOn())
    end
    )
end

function startShooter()
    shooter_system:setTargetSpeed(12.0)
end

function stopShooter()
    shooter_system:setTargetSpeed(0.0)
end

function waitUntilArmAngle(angle)
    control:waitUntilTrue(function()
        return (shooter_system:getArmAngle() > (angle - 4.0) and shooter_system:getArmAngle() < (angle + 4.0))
    end
    )
end

function setOdometryAlliance(x, red_y, red_angle)
    if robot:getAllianceColor() == "red" then
        drive_system:setOdometry(x, red_y, red_angle)
    else
        drive_system:setOdometry(x, -red_y, -red_angle)
    end
end

function driveToAlliance(x, red_y, red_angle)
    if robot:getAllianceColor() == "red" then
        drive_system:driveTo(x, red_y, red_angle)
    else
        drive_system:driveTo(x, -red_y, -red_angle)
    end
end

function turnToAlliance(red_angle)
    if robot:getAllianceColor() == "red" then
        drive_system:turnTo(red_angle)
    else
        drive_system:turnTo(-red_angle)
    end 
end

function inchesToMeters(inches)
    return inches*0.0254
end

function getRing(speed, max_x, max_time)--TODO: Revert to last max speed
    if vision:ringInView() then
        drive_system:ringLock(0.0)
        control:waitUntilTrue(function()
            return drive_system:ringLockedOn()
        end
        )
        last_max_drive_speed = drive_system:getMaxDriveSpeed()
        start_time = drive_system:getTime()
        drive_system:setMaxDriveSpeed(speed)
        intake_system:intake()
        drive_system:ringLock(-1.0)
        control:waitUntilTrue(function()
            has_ring = intake_system:hasRing()
            robot_x = drive_system:getRobotX()
            time_passed = drive_system:getTime() - start_time
            -- print("has_ring " .. tostring(has_ring) .. " robot_x " .. tostring(robot_x) .. " time_passed " .. tostring(time_passed)) --Print these to figureout what is going wrong
            -- print("has_ring: " .. tostring(has_ring) .. " robot_x > max_x: " .. tostring(robot_x > max_x) .. " time_passed >= max_time: " .. tostring(time_passed >= max_time))
            return has_ring or robot_x > max_x or time_passed >= max_time
        end
        )
    end
    drive_system:stop()
    --drive_system:setMaxDriveSpeed(last_max_drive_speed)
end

function shootRing()
    drive_system:autoAim(0.0, 0.0)--Stop to shoot
    distance = drive_system:distanceToSpeaker()
    --arm_angle = shooter_system:autoAimArmAngle(distance)
    arm_angle = -3.25
    timeStampPrint("shootRing(): Setting arm angle and locking on")
    shooter_system:setTargetArmAngle(arm_angle)
    waitUntilAimLocked(arm_angle)
    timeStampPrint("shootRing(): Locked on")
    drive_system:stop()
    timeStampPrint("shootRing(): Feeding")
    intake_system:feed()--Shoot
    control:sleep(0.5)
    intake_system:stop()
    timeStampPrint("shootRing(): Finished shooting ring")
end

function shootRing2()
    drive_system:autoAim(0.0, 0.0)
    control:sleep(0.4)
    drive_system:stop()
    intake_system:feed()--Shoot
    control:sleep(0.5)
    intake_system:stop()
end

function printTable(table, --[[optional]]delim, --[[optional]]end_line)
    delim = delim or " " --Default to spaces if not provided
    end_line = end_line or true --Default to adding a new line after printing the table
    for key,value in pairs(table) do
        if key == #table then
            io.write(tostring(value))
        else
            io.write(tostring(value) .. delim)
        end
    end
    if end_line then
        io.write("\n")
    end
end

function tableContains(table, item)
    for key, value in pairs(table) do
        if value == item then
            return true
        end
    end
    return false --Item not found
end


--Constants

--Ring locations. Ring 1 is the left most ring from the red side
CENTER_RING_Y_CORDINATES = {
    [1] = inchesToMeters(132),
    [2] = inchesToMeters(66),
    [3] = 0,
    [4] = inchesToMeters(-66),
    [5] = inchesToMeters(-132),
}

--Center line X location
CENTER_LINE_X = inchesToMeters(325.6)

--Drive too far threshold
CENTER_PICKUP_CUTOFF = inchesToMeters(365.0)

print("Shared script assets loaded!")
