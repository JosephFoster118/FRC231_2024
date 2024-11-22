print("Script started")
initStartTimeStamp()

setOdometryAlliance(inchesToMeters(58.5), inchesToMeters(-57.0), 0.0)
drive_system:setMaxDriveSpeed(4.0)
drive_system:setMinDrivePercentage(0.25)
intake_system:setVoltage(-5.5)
max_speed = 4.2
intake_speed = 3.2


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
timeStampPrint("Picking up second ring")
--vison:takeSnapshot()
shooter_system:setTargetSpeed(9.0)
getRing(2.8, inchesToMeters(150.0), 1.0)
drive_system:setMaxDriveSpeed(max_speed)
driveToAlliance(inchesToMeters(68.0), inchesToMeters(-57.0), 0.0) -- score
waitUntilDistance(inchesToMeters(1.0))
shootRing2()

--third ring
shooter_system:setTargetSpeed(0.0)
drive_system:setMaxDriveSpeed(5.0)
timeStampPrint("Driving to the third ring")
driveToAlliance(inchesToMeters(230.19 + 20.0), inchesToMeters(0.0), 0.0) -- in front of ring
waitUntilDistance(inchesToMeters(1.0))
vision:takeSnapshot()
getRing(intake_speed, inchesToMeters(326.0), 1.0)
drive_system:setMaxDriveSpeed(max_speed)
drive_system:setMinDrivePercentage(0.9)
shooter_system:setTargetSpeed(9.0)
driveToAlliance(inchesToMeters(190.0), inchesToMeters(0.0), 0.0) -- under stage
waitUntilDistance(inchesToMeters(2.0))
--drive_system:setMinDrivePercentage(0.85)
drive_system:setMinDrivePercentage(0.25)
driveToAlliance(inchesToMeters(68.0), inchesToMeters(-57.0), 0.0)-- score
waitUntilDistance(inchesToMeters(1.0))
shootRing2()

driveToAlliance(inchesToMeters(68.0), inchesToMeters(-114.0), 0.0)--Drive in front of ring
waitUntilDistance(0.024)
getRing(intake_speed, inchesToMeters(114.0 - 18.0), 1.0)
drive_system:setMaxDriveSpeed(max_speed)
driveToAlliance(inchesToMeters(68.0), inchesToMeters(-57.0), 0.0)-- score
waitUntilDistance(inchesToMeters(1.0))
shootRing2()
shooter_system:setTargetSpeed(0.0)

driveToAlliance(inchesToMeters(68.0), inchesToMeters(0.0), 0.0)--Drive in front of ring
waitUntilDistance(0.024)

drive_system:stop()
timeStampPrint("Auto has finished")
