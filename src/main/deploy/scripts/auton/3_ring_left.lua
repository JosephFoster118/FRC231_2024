print("Script started")

--first ring
setOdometryAlliance(inchesToMeters(42.0), inchesToMeters(-11.0), 54.0) --start
max_speed = 3.5
intake_speed = 2.5
shooter_system:setTargetSpeed(10.0)
drive_system:setMaxDriveSpeed(max_speed)
drive_system:setMinDrivePercentage(0.10)
shooter_system:setTargetArmAngle(-21.5)
-- driveToAlliance(inchesToMeters(60.0), inchesToMeters(-11.0), 54.0)
-- drive_system:setMinDrivePercentage(0.05)
-- waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
waitUntilArmAngle(-21.5)
intake_system:feed()
control:sleep(0.4)


--second ring
shooter_system:setTargetSpeed(0.0)
drive_system:setMinDrivePercentage(0.5)
driveToAlliance(inchesToMeters(114), inchesToMeters(58), 0.0) -- under stage
waitUntilDistance(inchesToMeters(1.0))
driveToAlliance(inchesToMeters(140), inchesToMeters(58), 0.0) -- under stage
waitUntilDistance(inchesToMeters(1.0))
driveToAlliance(inchesToMeters(190), inchesToMeters(0.0), 0.0)
waitUntilDistance(inchesToMeters(1.0))
    --in front of ring
drive_system:setMinDrivePercentage(0.25)
driveToAlliance(inchesToMeters(230.19 + 20.0), inchesToMeters(0.0), 0.0)
waitUntilDistance(inchesToMeters(2.0))
vision:takeSnapshot()
getRing(3.0, inchesToMeters(326.0), 3.5)
drive_system:setMinDrivePercentage(0.5)
shooter_system:setTargetSpeed(10.0)
drive_system:setMaxDriveSpeed(max_speed)
driveToAlliance(inchesToMeters(260.895), inchesToMeters(0.0), 0.0)
waitUntilDistance(inchesToMeters(2.0))
shooter_system:setTargetSpeed(9.0)
driveToAlliance(inchesToMeters(190), inchesToMeters(0.0), 54.0)
waitUntilDistance(inchesToMeters(1.0))
driveToAlliance(inchesToMeters(140), inchesToMeters(58), 54.0) -- under stage
waitUntilDistance(inchesToMeters(1.0))
drive_system:setMinDrivePercentage(0.10)
driveToAlliance(inchesToMeters(58.0), inchesToMeters(-11.0), 54.0)--start
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
intake_system:feed()
control:sleep(0.6)
intake_system:stop()

--third ring
drive_system:setMinDrivePercentage(0.85)
driveToAlliance(inchesToMeters(60.0), inchesToMeters(24.0), 0.0)--start
waitUntilDistance(inchesToMeters(1.0))
shooter_system:setTargetSpeed(0.0)
driveToAlliance(inchesToMeters(230.19), inchesToMeters(110.0), 0.0)
waitUntilDistance(inchesToMeters(1.0))
if(not vision:ringInView()) then
    turnToAlliance(-22.0)
    waitUntilTarget()
end
if(not vision:ringInView()) then
    turnToAlliance(18.0)
    waitUntilTarget()
end
if(not vision:ringInView()) then
    turnToAlliance(0.0)
    waitUntilTarget()
end
getRing(3.0, inchesToMeters(350.1), 3.5)
drive_system:setMaxDriveSpeed(max_speed)
driveToAlliance(inchesToMeters(230.19), inchesToMeters(99.0), 0.0)
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
control:sleep(0.5)

intake_system:stop()
shooter_system:stop()
