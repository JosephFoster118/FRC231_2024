print("Script started")

--first ring
shooter_system:setTargetSpeed(9.2)
shooter_system:setTargetArmAngle(-21.5)
driveToAlliance(inchesToMeters(68.0), inchesToMeters(-57.0), 0.0)
timeStampPrint("Shooting for first ring")
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
waitUntilArmAngle(-21.5)
control:sleep(0.2)
intake_system:feed()-- score
timeStampPrint("Feeding intake")
control:sleep(0.4)

--second ring
vision:takeSnapshot()
getRing(2.0, inchesToMeters(114.0 - 18.0), 1.5) --(speed, max_x)
drive_system:setMaxDriveSpeed(4.3)
driveToAlliance(inchesToMeters(58.5), inchesToMeters(-57.0), 0.0)--Drive to score
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
waitUntilArmAngle(-21.5)
control:sleep(0.2)
intake_system:feed()-- score
timeStampPrint("Feeding intake")
control:sleep(0.4)

--third ring
driveToAlliance(inchesToMeters(58.5), inchesToMeters(-114.0), 0.0)--Drive in front of ring
waitUntilDistance(0.024)
getRing(2.0, inchesToMeters(114.0 - 18.0), 1.0) --(speed, max_x)

drive_system:setMaxDriveSpeed(4.3)
driveToAlliance(inchesToMeters(58.5), inchesToMeters(-57.0), 0.0)--Drive to score
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
waitUntilArmAngle(-21.5)
control:sleep(0.2)
intake_system:feed()-- score
timeStampPrint("Feeding intake")
control:sleep(0.4)

--fourth ring
driveToAlliance(inchesToMeters(58.5), 0.0, 0.0)--Drive in front of ring
waitUntilDistance(0.024)
getRing(2.0, inchesToMeters(114.0 - 18.0), 1.0) --(speed, max_x)

drive_system:setMaxDriveSpeed(4.3)
driveToAlliance(inchesToMeters(58.5), inchesToMeters(-57.0), 0.0)--Drive to score
waitUntilDistance(inchesToMeters(1.0))
drive_system:stop()
waitUntilArmAngle(-21.5)
control:sleep(0.2)
intake_system:feed()-- score
timeStampPrint("Feeding intake")
control:sleep(0.4)

--fifth ring
-- drive_system:setMinDrivePercentage(0.75)
-- driveToAlliance(inchesToMeters(230.19), inchesToMeters(0.0), 0.0)
-- waitUntilDistance(inchesToMeters(5.0))
-- driveToAlliance(inchesToMeters(260.895), inchesToMeters(0.0), 0.0)
-- waitUntilDistance(inchesToMeters(1.0))

-- getRing(2.0, 300, 1.0)

-- drive_system:setMaxDriveSpeed(4.0)
-- drive_system:setMinDrivePercentage(0.9)
-- driveToAlliance(inchesToMeters(265.19), inchesToMeters(0.0), 30.0)
-- waitUntilDistance(inchesToMeters(1.0))


shooter_system:setTargetSpeed(0.0)

intake_system:stop()

drive_system:stop()

print(robot:getTimeLeft())
