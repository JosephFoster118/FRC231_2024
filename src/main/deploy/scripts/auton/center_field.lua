print("Script started")

setOdometryAlliance(inchesToMeters(52.0), inchesToMeters(-5.0), 47.2)

startShooter()
shooter_system:setTargetArmAngle(-20.0)
waitUntilArmAngle(-20.0)
intake_system:feed()
control:sleep(0.5)
intake_system:stop()
stopShooter()

--under stage
drive_system:setMaxDriveSpeed(3.0)
driveToAlliance(inchesToMeters(92.0), inchesToMeters(81.0), 0.0)
waitUntilDistance(inchesToMeters(6.0))
drive_system:setMaxDriveSpeed(3.0)
driveToAlliance(inchesToMeters(231.0), inchesToMeters(0.0), 0.0)
waitUntilDistance(inchesToMeters(3.0))

--pick up
intake_system:intake()
drive_system:setMaxDriveSpeed(1.5)
driveToAlliance( inchesToMeters(27.0*12), 0.0 + inchesToMeters(3.0), 0.0)
waitUntilDistance(0.024)
drive_system:setMaxDriveSpeed(3.0)
startShooter()
driveToAlliance(inchesToMeters(231.0), inchesToMeters(0.0), 0.0)
waitUntilDistance(inchesToMeters(3.0))

--under stage
shooter_system:setTargetArmAngle(-5.7)
driveToAlliance(inchesToMeters(121.0), -1.68 + 0.024, 0.0)--Drive to score
waitUntilDistance(0.024)
drive_system:stop()
intake_system:feed()
control:sleep(0.5)
intake_system:stop()
stopShooter()

driveToAlliance(inchesToMeters(27.0*12), 0.0 + inchesToMeters(69.0), 0.0)
waitUntilDistance(0.024)

drive_system:stop()

