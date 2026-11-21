package org.firstinspires.ftc.teamcode.scrimmage3.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

@TeleOp(name = "duckY")
public class Scrimmage3Tele extends LinearOpMode {
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);

    boolean red = false;
    boolean shooterEnabled = false;
    boolean intakeEnabled = false;
    boolean intakeReversed = false;

    //***edge detection***
    boolean lastFrontBeam = false;
    boolean lastPollenBeam = false;

    //*TODO COUNT VARIABLE
    public static int ballCount;

    //*TODO BALL COLORS
    boolean isShooting = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        shooterSS.setAutoS3Up(false);
        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.HoodState.SHOOT);
        shooterSS.updateState(ShooterSS.TurretState.SHOOT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        ballCount = 0;

        while (opModeInInit()) {
            red = RobotHardware.isRed();
            //alliance selection carried over from auto but can still be manually changed
            if (gamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                red = !red;
            }

            //since red = RobotHardware.isRed() it is carried over from auto by default, however can be manually changed
            if (red) {
                RobotHardware.setAlliance(RobotHardware.Alliance.RED);
            }
            else {
                RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
            }

            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.getAlliance() + " alliance active");
            telemetry.update();
            robot.previousGamepad1.copy(gamepad1);
        }

        waitForStart();
        robot.clearBulkCache();

        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();

        if (!lastFrontBeam) {
            ballCount = 1;
        }

        while (opModeIsActive()) {
            robot.clearBulkCache();

            follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            follower.update();

            //*todo intake logic
            if (gamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
                intakeEnabled = !intakeEnabled;
                intakeReversed = false;

                if (intakeEnabled && ballCount < 4) {
                    intakeSS.updateState(IntakeSS.IntakeState.ON);
                }
                else {
                    intakeEnabled = false;
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            if (gamepad1.x && !robot.previousGamepad1.x) {
                intakeReversed = !intakeReversed;
                intakeEnabled = intakeReversed;

                if (intakeReversed) {
                    intakeSS.updateState(IntakeSS.IntakeState.REVERSE);
                }
                else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            //*todo intake beambreaker logic
            boolean frontBeam = robot.frontBeam.getState();
            boolean pollenBeam = robot.pollenBeam.getState();

            if (!frontBeam && lastFrontBeam && ballCount < 4) {
                ballCount++;
            }

            /* if front beam not tripped, then lastFrontbeam = true, else if front beam tripped, then last front beam = false
             * so if front beam wasn't tripped and front beam is now tripped, ball count increases by one
             */

            if (isShooting && pollenBeam && !lastPollenBeam && ballCount > 0) {
                ballCount--;
            }

            lastFrontBeam = frontBeam;
            lastPollenBeam = pollenBeam;

            if (ballCount == 4 && intakeEnabled && !intakeReversed) {
                if (!pollenBeam) {
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.INIT) {
                        intakeSS.updateState(IntakeSS.IntakeState.INIT);
                    }
                }
                else {
                    if (intakeSS.getIntakeState() != IntakeSS.IntakeState.FEED) {
                        intakeSS.updateState(IntakeSS.IntakeState.FEED);
                    }
                }
            }

            if (ballCount >= 1 && !shooterEnabled) {
                shooterEnabled = true;
                shooterSS.updatePollenState(ShooterSS.ShooterState.AUTO_S3);
            }
            else if (ballCount == 0 && shooterEnabled) {
                shooterEnabled = false;
                shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);
            }

            if (gamepad1.a && !robot.previousGamepad1.a && ballCount > 0) {
                shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);
                isShooting = true;
            }

            if (isShooting && ballCount == 0) {
                shooterSS.updateState(ShooterSS.LockState.LOCKED);
                isShooting = false;
                lastFrontBeam = robot.frontBeam.getState();
            }

            telemetry.addLine("******** CRITICAL INFO ********");
            telemetry.addData("Alliance", RobotHardware.getAlliance());
            telemetry.addData("Ball count", ballCount);
            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Shooting", isShooting);
            telemetry.addData("Intake state", intakeSS.getIntakeState());

            telemetry.addLine("******** BEAMBREAKERS ********");
            telemetry.addData("Front beam", frontBeam);
            telemetry.addData("Pollen beam", pollenBeam);

            telemetry.addLine("******** SHOOTER ********");
            telemetry.addData("Shooter state", shooterSS.getPollenShooterState());
            telemetry.addData("Shooter power", robot.shooter1.getPower());
            telemetry.addData("Shooter velocity", robot.shooter1.getVelocity());
            telemetry.addData("Shooter current", robot.amps(robot.shooter1));
            telemetry.addData("Lock state", shooterSS.getLockState());

            telemetry.addLine("******** TURRET AND HOOD ********");
            telemetry.addData("Turret position", robot.turret1.getPosition());
            telemetry.addData("Hood position", robot.hood1.getPosition());

            telemetry.addLine("******** POSE ********");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));

            telemetry.update();
            robot.previousGamepad1.copy(gamepad1);
        }

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
    }
}