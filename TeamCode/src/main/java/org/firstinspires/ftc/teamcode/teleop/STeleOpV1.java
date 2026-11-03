
package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(name = "Para Tele V1 💛")
@Configurable
public class STeleOpV1 extends LinearOpMode {
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    TrackingSS trackingSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);

    boolean red = false;
    boolean printData = false;
    boolean trackingEnabled = false;
    boolean shooterEnabled = false;
    boolean intakeEnabled = false;
    boolean intakeReversed = false;

    //***edge detection***
    boolean lastLB = false;
    boolean lastRB = false;
    boolean lastLT = false;
    boolean lastStart = false;
    boolean lastA = false;
    boolean lastX = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.HoodState.INIT);
        shooterSS.updateState(ShooterSS.TurretState.INIT);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        while (opModeInInit()) {
            if (gamepad1.left_bumper && !lastLB) {
                red = !red;
            }
            lastLB = gamepad1.left_bumper;

            if (red) {
                RobotHardware.alliance = RobotHardware.Alliance.RED;
            }
            else {
                RobotHardware.alliance = RobotHardware.Alliance.BLUE;
            }

            if (gamepad1.left_trigger > 0.3 && !lastLT) {
                printData = !printData;
            }
            lastLT = gamepad1.left_trigger > 0.3;

            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.alliance + " alliance active");
            telemetry.addData("Print data", printData);
            telemetry.addLine("*********They not like ducks**********");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        trackingSS = new TrackingSS(robot, follower, RobotHardware.alliance);

        waitForStart();

        while (opModeIsActive()) {
            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            if (gamepad1.start && !lastStart) {
                trackingEnabled = !trackingEnabled;

                if (trackingEnabled) {
                    TrackingSS.updateTracking(shooterSS);
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_ENABLED);
                    shooterSS.updateState(ShooterSS.HoodState.INTERPOLATED);
                }
                else {
                    shooterSS.updateState(ShooterSS.TurretState.TRACKING_DISABLED);
                    shooterSS.updateState(ShooterSS.HoodState.SHOOT);
                }

                if (shooterEnabled && trackingEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
                else if (shooterEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }
            lastStart = gamepad1.start;

            if (gamepad1.a && !lastA) {
                shooterEnabled = !shooterEnabled;

                if (!shooterEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INIT);
                }
                else if (trackingEnabled) {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOLATED);
                }
                else {
                    shooterSS.updateState(ShooterSS.ShooterState.INTERPOL_DISABLED);
                }
            }
            lastA = gamepad1.a;

            if (gamepad1.right_bumper && !lastRB) {
                intakeEnabled = !intakeEnabled;
                intakeReversed = false;

                if (intakeEnabled) {
                    intakeSS.updateState(IntakeSS.IntakeState.ON);
                }
                else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }
            lastRB = gamepad1.right_bumper;

            if (gamepad1.x && !lastX) {
                intakeReversed = !intakeReversed;
                intakeEnabled = intakeReversed;

                if (intakeReversed) {
                    intakeSS.updateState(IntakeSS.IntakeState.REVERSE);
                }
                else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }
            lastX = gamepad1.x;

            if (trackingEnabled) {
                TrackingSS.updateTracking(shooterSS);
            }

            //*************************** telemetry ********************************
            telemetry.addData("Alliance", RobotHardware.alliance);
            telemetry.addData("Tracking", trackingEnabled);
            telemetry.addData("Shooter", shooterEnabled);
            telemetry.addData("Intake", intakeSS.getIntakeState());

            if (printData) {
                telemetry.addLine("");
                telemetry.addLine("***Robot pose***");
                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));

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
                telemetry.addLine("***Turret***");
                telemetry.addData("Turret state", shooterSS.getPollenTurretState() + " / " + shooterSS.getNectarTurretState());
                telemetry.addData("Turret position", robot.turret1.getPosition() + " / " + robot.turret2.getPosition());
                telemetry.addData("Turret angle", TrackingSS.getTurret());

                telemetry.addLine("");
                telemetry.addLine("***Interpolation***");
                telemetry.addData("Distance", TrackingSS.rangeVal);
                telemetry.addData("Reference distance", TrackingSS.referenceDist);
                telemetry.addData("RPM per inch", TrackingSS.rpmPerInch);
                telemetry.addData("Hood per inch", TrackingSS.hoodPerInch);
            }

            telemetry.update();
        }
    }
}
