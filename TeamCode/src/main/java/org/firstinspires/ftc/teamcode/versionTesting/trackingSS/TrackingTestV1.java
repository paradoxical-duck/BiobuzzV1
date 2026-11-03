
package org.firstinspires.ftc.teamcode.versionTesting.trackingSS;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(group = "Version Testing")
@Configurable
//todo tune TrackingSS values while running this
public class TrackingTestV1 extends LinearOpMode {
    TrackingSS trackingSS;
    RobotHardware robot = RobotHardware.getInstance();
    Follower follower;
    RobotHardware.Alliance alliance = RobotHardware.Alliance.RED;
    ShooterSS shooterSS;
    boolean lastLB = false;
    boolean red = true;
    boolean lastStart = false;
    boolean lastBack = false;
    boolean lastRB = false;
    public static Pose startPose = new Pose(0, 0, 0);
    public static boolean trackingEnabled = false, interpolEnabled = false, shooterEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        shooterSS = new ShooterSS(robot);

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) {
                red = !red;
            }
            lastLB = gamepad1.left_bumper;

            if (red) {
                alliance = RobotHardware.Alliance.RED;
            }
            else {
                alliance = RobotHardware.Alliance.BLUE;
            }

            RobotHardware.alliance = alliance;

            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.alliance + " alliance active");
            telemetry.update();
        }

        trackingSS = new TrackingSS(robot, follower, alliance);

        waitForStart();

        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.start && !lastStart) {
                trackingEnabled = !trackingEnabled;

                if (trackingEnabled) {
                    shooterSS.setTurretTarget(TrackingSS.getTurretPos());
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_ENABLED);
                }
                else {
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_DISABLED);
                }
            }
            lastStart = gamepad1.start;

            if (gamepad1.back && !lastBack) {
                interpolEnabled = !interpolEnabled;

                if (interpolEnabled) {
                    TrackingSS.interpolate();
                    shooterSS.setHoodTarget(TrackingSS.getTargetHood());
                    shooterSS.setShooterTarget(TrackingSS.getTargetRpm());
                    shooterSS.updateState(ShooterSS.HoodState.INTERPOLATED);

                    if (shooterEnabled) {
                        shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                    }
                }
                else {
                    shooterSS.updateState(ShooterSS.HoodState.SHOOT);

                    if (shooterEnabled) {
                        shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                    }
                }
            }
            lastBack = gamepad1.back;

            if (gamepad1.right_bumper && !lastRB) {
                shooterEnabled = !shooterEnabled;

                if (!shooterEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INIT);
                }
                else if (interpolEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
                else {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }
            lastRB = gamepad1.right_bumper;

            if (trackingEnabled) {
                shooterSS.setTurretTarget(TrackingSS.getTurretPos());
            }

            if (interpolEnabled) {
                TrackingSS.interpolate();
                shooterSS.setHoodTarget(TrackingSS.getTargetHood());

                if (shooterEnabled) {
                    shooterSS.setShooterTarget(TrackingSS.getTargetRpm());
                }
            }

            //*************************** telemetry ********************************
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Turret tracking", trackingEnabled);
            telemetry.addData("Interpolation", interpolEnabled);
            telemetry.addData("Shooter enabled", shooterEnabled);

            telemetry.addLine("");
            telemetry.addLine("***Robot pose***");
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));

            telemetry.addLine("");
            telemetry.addLine("***Turret***");
            telemetry.addData("Turret state", shooterSS.getPollenTurretState() + " / " + shooterSS.getNectarTurretState());
            telemetry.addData("Turret position", robot.turret1.getPosition() + " / " + robot.turret2.getPosition());
            telemetry.addData("Turret angle", TrackingSS.getTurret());
            telemetry.addData("Target dx", TrackingSS.dx);
            telemetry.addData("Target dy", TrackingSS.dy);

            telemetry.addLine("");
            telemetry.addLine("***Shooter***");
            telemetry.addData("Shooter state", shooterSS.getPollenShooterState() + " / " + shooterSS.getNectarShooterState());
            telemetry.addData("Target velocity", TrackingSS.getTargetRpm());
            telemetry.addData("Shooter 1 velocity", robot.shooter1.getVelocity());
            telemetry.addData("Shooter 2 velocity", robot.shooter2.getVelocity());

            telemetry.addLine("");
            telemetry.addLine("***Hood***");
            telemetry.addData("Hood state", shooterSS.getPollenHoodState() + " / " + shooterSS.getNectarHoodState());
            telemetry.addData("Target hood", TrackingSS.getTargetHood());
            telemetry.addData("Hood position", robot.hood1.getPosition() + " / " + robot.hood2.getPosition());

            telemetry.addLine("");
            telemetry.addLine("***Interpolation***");
            telemetry.addData("Distance", TrackingSS.rangeVal);
            telemetry.addData("Reference distance", TrackingSS.referenceDist);
            telemetry.addData("Distance error", TrackingSS.rangeVal - TrackingSS.referenceDist);
            telemetry.addData("RPM per inch", TrackingSS.rpmPerInch);
            telemetry.addData("Hood per inch", TrackingSS.hoodPerInch);

            telemetry.update();
        }
    }
}
