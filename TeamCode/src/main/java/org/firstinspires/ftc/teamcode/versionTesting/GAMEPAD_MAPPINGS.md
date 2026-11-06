# VersionTesting gamepad tuning (Gamepad 1)

All tunable values are adjusted live and displayed on Driver Station telemetry.
Settings are in-memory for this OpMode. Copy final calibrated values into Globals/TrackingSS afterward.
Motor tests initialize at ZERO. Keep the robot raised and mechanisms clear during testing.

* IntakeTestV1: LEFT/RIGHT select intake; UP/DOWN power +/-; A coarse 0.05; B fine 0.01; Y selected full power; X stop both.
* IntakeDeployTestV1: UP/DOWN position +/-; A coarse 0.01; B fine 0.001; X position 0; Y position 1.
* LockTestV1: LEFT/RIGHT select lock; UP/DOWN position +/-; A coarse 0.01; B fine 0.001; X position 0; Y position 1.
* TurretTestV1: LEFT/RIGHT select turret; LB/RB position -/+; A coarse 0.001; B fine 0.0001; X center selected.
* ShooterTestV1: LEFT/RIGHT select pollen/nectar RPM or PIDF; UP/DOWN adjust; A coarse; B fine; X stop both flywheels.
* ShooterSSTuning: LEFT/RIGHT select one of six outputs; UP/DOWN adjust; A coarse; B fine; X stop both shooters.
* DriveTestV1: Left stick translates; right stick X rotates; HOLD RB to enable movement; A/B/X select 30/60/100% drive power.
* ColorTestV1: X toggles RED/BLUE classification. Telemetry shows HSV/distance/type.
* TrackingTestV1: In INIT LB selects RED/BLUE. During RUN START toggles turret tracking; BACK toggles interpolation; RB toggles shooter output; X stops shooters; LEFT/RIGHT select 14 tuning parameters; UP/DOWN adjust; A/B coarse/fine; HOLD RIGHT TRIGGER to allow 30% drive power via sticks.

* IMPORTANT: TrackingTestV1 initializes Pedro follower, which requires the drivetrain and Pinpoint hardware in Constants. It is NOT an actuator-only test. All other individual actuator tests map only their own devices.
* IMPORTANT: Set positions change servo mechanisms immediately. Limit movement if your linkage has physical stops before 0 or 1.
* IMPORTANT: Drive motor configuration in Constants includes a trailing space in the back-right motor name ("right back "). That must match your Driver Station configuration exactly.
