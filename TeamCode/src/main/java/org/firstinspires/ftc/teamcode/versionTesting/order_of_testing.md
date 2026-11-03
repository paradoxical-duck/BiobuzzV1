# Order of Version Testing

#### Tune in this order only:

1. Start with `IntakeTestV1` to tune intake power and direction
2. Next, run `IntakeDeployTestV1` to tune the flower ramp, finishing IntakeSS tuning
3. After that, start ShooterSS tuning with `ShooterTestV1`, to tune the shooter motor direction individually, and shooter PIDF
4. Next, run `ShooterSSTuning` to tune fixed shooting positions for turret, shooter and hood in teleOp for when interpol/tracking is disabled, and for auto (using fixed shooting only in auto)
5. Next, run `TurretTestV1` to get turret extreme servo positions for turret tracking
6. After this, run `TrackingTestV1` to test and tune all tracking. Use FTC Dashboard to tune the values stored in TrackingSS and Globals while running this. Drive robot around using controller for this stage.
7. Finally, run `LockTestV1` to tune shooter lock open and closed positions, and the robot is completely ready for auto and teleOp

Note: Alternatively, can run `LockTestV1` before tuning tracking, so robot is ready for fixed shooting in teleOp and is also ready for auto. Tracking can be tuned later.


FTC Dashboard uses `com.acmerobotics.dashboard:dashboard:0.6.0`. After installing the Robot Controller app, connect to the robot Wi-Fi and open [Control Hub Dashboard](http://192.168.43.1:8080/dash) or [phone Robot Controller Dashboard](http://192.168.49.1:8080/dash). Version Testing OpModes publish to both Dashboard and Driver Station. Edit the public static fields in the matching Config entry while the OpMode runs. Shooter velocities use encoder ticks per second, matching `DcMotorEx.setVelocity(double)`, despite the legacy RPM field names in TrackingSS.

TrackingTestV1 requires the two shooter motors, both turrets and hoods, plus the configured drivetrain and Pinpoint for driving and localization. It does not map intake, feed locks, beam breakers, or the color sensor. The remaining tests map only their named actuators or color sensor. DriveTestV1 retains its existing `m1` through `m4` bench configuration names. Servo position feedback is the commanded SDK position, not a measured physical angle.
