print("Script started")

--first ring
setOdometryAlliance(inchesToMeters(42.0), inchesToMeters(-11.0), 54.0) --start
max_speed = 3.5
intake_speed = 2.5
shooter_system:setTargetSpeed(10.0)
drive_system:setMaxDriveSpeed(max_speed)
drive_system:setMinDrivePercentage(0.10)
shooter_system:setTargetArmAngle(-21.0)
waitUntilArmAngle(-21.0)
intake_system:feed()
control:sleep(0.6)

intake_system:stop()
shooter_system:setTargetSpeed(0.0)
drive_system:stop()
