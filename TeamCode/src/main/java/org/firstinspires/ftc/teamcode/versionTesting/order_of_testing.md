# Order of Version Testing

#### Tune in this order only:

1. Start with `IntakeTestV1` to tune intake power and direction
2. Next, run `IntakeDeployTestV1` to tune the flower ramp, finishing IntakeSS tuning
3. After that, start ShooterSS tuning with `ShooterTestV1`, to tune the shooter motor direction individually, and shooter PIDF
4. Next, run `ShooterSSTuning` to tune fixed shooting positions for turret, shooter and hood in teleOp for when interpol/tracking is disabled, and for auto (using fixed shooting only in auto)
5. Next, run `TurretTestV1` to get turret extreme servo positions for turret tracking
6. After this, run `TrackingTestV1` to test and tune all tracking. Use FTC Dashboard to tune the values stored in TrackingSS while running this. Drive robot around using controller for this stage.
7. Finally, run `LockTestV1` to tune shooter lock open and closed positions, and the robot is completely ready for auto and teleOp

Note: Alternatively, can run `LockTestV1` before tuning tracking, so robot is ready for fixed shooting in teleOp and is also ready for auto. Tracking can be tuned later.
