package org.firstinspires.ftc.teamcode.scrimmage3.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.PrintData;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.TrackingSS;

@TeleOp(name = "ducky")
public class ParaTeleS3 extends LinearOpMode {
    IntakeSS intakeSS;
    ShooterSS shooterSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);

    boolean red = false;
    boolean shooterEnabled = false;
    boolean intakeEnabled = false;
    boolean intakeReversed = false;
    boolean trackingEnabled = false;
    boolean shooterPaused = false;

    //***edge detection***
    boolean lastFrontBeam = false;
    boolean lastPollenBeam = false;

    //TODO COUNT VARIABLE
    public static int ballCount;
    int lastBallCount = 0;

    //TODO BALL COLORS
    boolean isShooting = false;

    boolean finishingShot = false;
    ElapsedTime finishTimer = new ElapsedTime();
    public static double finishDelayMs = 100;

    void resetGamepads() {
        robot.currentGamepad1.copy(gamepad1);
        robot.previousGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);
        robot.previousGamepad2.copy(gamepad2);
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        resetGamepads();
        PrintData.reset();

        red = RobotHardware.isRed();
        shooterEnabled = false;
        intakeEnabled = false;
        intakeReversed = false;
        trackingEnabled = false;
        shooterPaused = false;
        isShooting = false;
        finishingShot = false;
        lastFrontBeam = false;
        lastPollenBeam = false;
        finishTimer.reset();

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        startPose = RobotHardware.currentPose != null ? RobotHardware.currentPose : new Pose(0, 0, 0); //todo set pose after auto
        follower.setPose(startPose);

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.HoodState.SHOOT);
        shooterSS.updateState(ShooterSS.TurretState.SHOOT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        intakeSS.updateState(IntakeSS.FlowerRampState.INIT);

        robot.turret1.setPosition(0.5);
        robot.turret2.setPosition(0.5);

        ballCount = 0;
        lastBallCount = 0;

        while (opModeInInit()) {
            robot.readGamepads(gamepad1, gamepad2);

            red = RobotHardware.isRed();
            //alliance selection carried over from auto but can still be manually changed
            if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                red = !red;
            }

            //since red = RobotHardware.isRed() it is carried over from auto by default, however can be manually changed
            if (red) {
                RobotHardware.setAlliance(RobotHardware.Alliance.RED);
            }
            else {
                RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
            }

            if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
                startPose = new Pose(0, 0, startPose.heading());
                follower.setPose(startPose);
            }
            else if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
                startPose = new Pose(141.5, 0, startPose.heading());
                follower.setPose(startPose);
            }
            else if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
                startPose = new Pose(0, 141.5, startPose.heading());
                follower.setPose(startPose);
            }
            else if (robot.currentGamepad1.y && !robot.previousGamepad1.y) {
                startPose = new Pose(141.5, 141.5, startPose.heading());
                follower.setPose(startPose);
            }

            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.getAlliance() + " alliance active");

            telemetry.addLine("***Starting Pose***");
            telemetry.addLine("A: (0, 0) | B: (141.5, 0)");
            telemetry.addLine("X: (0, 141.5) | Y: (141.5, 141.5)");
            telemetry.addData("Starting X", startPose.x());
            telemetry.addData("Starting Y", startPose.y());
            telemetry.addData("Starting heading", Math.toDegrees(startPose.heading()));

            PrintData.menu(robot.currentGamepad1, robot.previousGamepad1, telemetry);

            telemetry.update();
        }

        new TrackingSS(robot, follower, RobotHardware.getAlliance());

        waitForStart();
        resetGamepads();
        robot.clearBulkCache();

        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();

        if (!lastFrontBeam) {
            ballCount = 1;
        }

        lastBallCount = ballCount;

        while (opModeIsActive()) {
            robot.clearBulkCache();
            robot.update();
            robot.readGamepads(gamepad1, gamepad2);

            Gamepad gamepad1 = robot.currentGamepad1;
            Gamepad gamepad2 = robot.currentGamepad2;
            Gamepad previousGamepad1 = robot.previousGamepad1;
            Gamepad previousGamepad2 = robot.previousGamepad2;

            follower.manual(ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            ));
            follower.update();

            if (gamepad2.a && !previousGamepad2.a) {
                follower.setPose(new Pose(red ? 0 : 141.5, 0, follower.pose().heading()));
                TrackingSS.resetHiveSelection();
            }

            if (gamepad2.b && !previousGamepad2.b) {
                follower.setPose(new Pose(red ? 0 : 141.5, 141.5, follower.pose().heading()));
                TrackingSS.resetHiveSelection();
            }

            TrackingSS.setDxAndDy();

            if (gamepad2.back && !previousGamepad2.back) {
                TrackingSS.toggleHive();
            }

            if (gamepad2.start && !previousGamepad2.start) {
                trackingEnabled = !trackingEnabled;

                if (trackingEnabled) {
                    shooterSS.setPollenTurretTarget(TrackingSS.getPollenTurretPos());
                    shooterSS.updatePollenState(ShooterSS.TurretState.TRACKING_ENABLED);
                }
                else {
                    shooterSS.updatePollenState(ShooterSS.TurretState.SHOOT);
                }
            }

            if (trackingEnabled) {
                shooterSS.setPollenTurretTarget(TrackingSS.getPollenTurretPos());
            }

            //todo intake logic
            if (!isShooting && gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeEnabled = !intakeEnabled || intakeReversed;
                intakeReversed = false;

                if (intakeEnabled && ballCount < 4) {
                    intakeSS.updateState(IntakeSS.IntakeState.ON);
                }
                else {
                    intakeEnabled = false;
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            if (!isShooting && gamepad1.x && !previousGamepad1.x) {
                intakeReversed = !intakeReversed;
                intakeEnabled = intakeReversed;

                if (intakeReversed) {
                    intakeSS.updateState(IntakeSS.IntakeState.REVERSE);
                }
                else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            //todo intake beambreaker logic
            boolean frontBeam = robot.frontBeam.getState();
            boolean pollenBeam = robot.pollenBeam.getState();

            if (!frontBeam && lastFrontBeam && ballCount < 4) {
                ballCount++;
                shooterPaused = false;
            }

            /* if front beam not tripped, then lastFrontbeam = true, else if front beam tripped, then last front beam = false
             * so if front beam wasn't tripped and front beam is now tripped, ball count increases by one
             */

            if (isShooting && pollenBeam && !lastPollenBeam && ballCount > 0) {
                ballCount--;
            }

            lastFrontBeam = frontBeam;
            lastPollenBeam = pollenBeam;

            if (ballCount == 4 && lastBallCount < 4) {
                this.gamepad1.rumble(250);
            }

            lastBallCount = ballCount;

            if (intakeEnabled && !intakeReversed && !isShooting) {
                IntakeSS.IntakeState targetState;

                if (ballCount == 4) {
                    targetState = IntakeSS.IntakeState.INIT;
                }
                else if (ballCount >= 2) {
                    targetState = IntakeSS.IntakeState.POINT_FIVE;
                }
                else {
                    targetState = IntakeSS.IntakeState.ON;
                }

                if (intakeSS.getIntakeState() != targetState) {
                    intakeSS.updateState(targetState);
                }
            }

            if (ballCount >= 1 && !shooterEnabled && !shooterPaused) {
                shooterEnabled = true;
                shooterSS.updatePollenState(ShooterSS.ShooterState.ON);
            }
            else if (ballCount == 0 && shooterEnabled && !isShooting) {
                shooterEnabled = false;
                shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);
            }

            if (gamepad1.left_bumper && !previousGamepad1.left_bumper && ballCount > 0 && !isShooting) {
                shooterPaused = false;

                if (!shooterEnabled) {
                    shooterEnabled = true;
                    shooterSS.updatePollenState(ShooterSS.ShooterState.ON);
                }

                shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);
                intakeSS.updateState(IntakeSS.IntakeState.ON);
                intakeReversed = false;
                isShooting = true;
                finishingShot = false;
            }

            if (isShooting) {
                if (ballCount == 0) {
                    if (!finishingShot) {
                        finishingShot = true;
                        finishTimer.reset();
                    }

                    if (finishTimer.milliseconds() >= finishDelayMs) {
                        shooterSS.updateState(ShooterSS.LockState.LOCKED);
                        shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);

                        isShooting = false;
                        finishingShot = false;
                        shooterEnabled = false;

                        intakeSS.updateState(intakeEnabled ? IntakeSS.IntakeState.ON : IntakeSS.IntakeState.INIT);

                        lastFrontBeam = robot.frontBeam.getState();
                    }
                }
                else {
                    finishingShot = false;
                }
            }

            if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                if (intakeSS.getFlowerRampState() != IntakeSS.FlowerRampState.DEPLOYED) {
                    intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED);
                }
                else {
                    intakeSS.updateState(IntakeSS.FlowerRampState.UP);
                }
            }

            if (gamepad1.back && !previousGamepad1.back) {
                intakeEnabled = false;
                intakeReversed = false;
                shooterEnabled = false;
                shooterPaused = true;
                isShooting = false;
                finishingShot = false;

                finishTimer.reset();

                intakeSS.updateState(IntakeSS.IntakeState.INIT);
                shooterSS.updateState(ShooterSS.ShooterState.INIT);
                shooterSS.updateState(ShooterSS.LockState.LOCKED);
            }

            PrintData.print(robot, follower, intakeSS, shooterSS, null, trackingEnabled, shooterEnabled, ballCount, telemetry);

            telemetry.update();
        }

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        TrackingSS.resetHiveSelection();
    }
}