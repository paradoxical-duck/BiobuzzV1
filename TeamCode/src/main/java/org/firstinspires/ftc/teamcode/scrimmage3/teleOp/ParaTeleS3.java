package org.firstinspires.ftc.teamcode.scrimmage3.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    TrackingSS trackingSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);

    RobotHardware.Alliance alliance;

    //***edge detection***
    boolean lastFrontBeam = false;
    boolean lastPollenBeam = false;

    //TODO COUNT VARIABLE
    public enum Count {EMPTY, ONE, TWO, THREE, FOUR}
    public static Count ballCount = Count.EMPTY;
    public static Count lastBallCount = Count.EMPTY;

    //TODO BALL COLORS
    enum ShootingMode {INIT, SHOOTING, FINAL_SHOT, RELOCK}
    ShootingMode shootingMode = ShootingMode.INIT;

    enum IntakeMode {ON, REVERSE, INIT}
    IntakeMode currentMode = IntakeMode.INIT;

    ElapsedTime finishTimer = new ElapsedTime();
    public static double finishDelayMs = 100;

    ElapsedTime intakeLockoutTimer = new ElapsedTime();
    public static double intakeLockoutMs = 250;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        resetGamepads();
        PrintData.reset();

        alliance = RobotHardware.getAlliance();
        shootingMode = ShootingMode.INIT;
        currentMode = IntakeMode.INIT;

        lastFrontBeam = false;
        lastPollenBeam = false;

        finishTimer.reset();
        intakeLockoutTimer.reset();

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        startPose = RobotHardware.currentPose != null ? RobotHardware.currentPose : new Pose(0, 0, 0); //todo set pose after auto
        follower.setPose(startPose);

        robot.init(shooterSS, intakeSS);

        ballCount = Count.EMPTY;
        lastBallCount = Count.EMPTY;

        while (opModeInInit()) {
            robot.readGamepads(gamepad1, gamepad2);

            alliance = RobotHardware.getAlliance();

            //alliance selection carried over from auto but can still be manually changed
            if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                alliance = alliance == RobotHardware.Alliance.RED ? RobotHardware.Alliance.BLUE : RobotHardware.Alliance.RED;
            }

            //since red = RobotHardware.isRed() it is carried over from auto by default, however can be manually changed
            RobotHardware.setAlliance(alliance);

            if (robot.currentGamepad1.a && !robot.previousGamepad1.a) {
                startPose = new Pose(0, 0, startPose.heading());
            }
            else if (robot.currentGamepad1.b && !robot.previousGamepad1.b) {
                startPose = new Pose(141.5, 0, startPose.heading());
            }
            else if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
                startPose = new Pose(0, 141.5, startPose.heading());
            }
            else if (robot.currentGamepad1.y && !robot.previousGamepad1.y) {
                startPose = new Pose(141.5, 141.5, startPose.heading());
            }

            follower.setPose(startPose);

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

        trackingSS = new TrackingSS(robot, follower, alliance);

        waitForStart();
        resetGamepads();
        robot.clearBulkCache();

        lastFrontBeam = robot.frontBeam.getState();
        lastPollenBeam = robot.pollenBeam.getState();

        if (!lastFrontBeam) {
            ballCount = Count.ONE;
        }

        lastBallCount = ballCount;

        while (opModeIsActive()) {
            robot.clearBulkCache();
            robot.update();
            robot.readGamepads(gamepad1, gamepad2);

            follower.manual(ManualDrive.fieldCentric(-robot.currentGamepad1.left_stick_y, -robot.currentGamepad1.left_stick_x, -robot.currentGamepad1.right_stick_x, follower.pose().heading()));
            follower.update();

            if (robot.currentGamepad2.a && !robot.previousGamepad2.a) {
                follower.setPose(new Pose(alliance == RobotHardware.Alliance.RED ? 0 : 141.5, 0, follower.pose().heading()));
                trackingSS.resetSelectedHive();
            }

            if (robot.currentGamepad2.b && !robot.previousGamepad2.b) {
                follower.setPose(new Pose(alliance == RobotHardware.Alliance.RED ? 0 : 141.5, 141.5, follower.pose().heading()));
                trackingSS.resetSelectedHive();
            }

            trackingSS.setDxAndDy();

            if (robot.currentGamepad2.back && !robot.previousGamepad2.back) {
                trackingSS.toggleSelectedHive();
            }

            if (robot.currentGamepad2.start && !robot.previousGamepad2.start) {
                if (trackingEnabled()) {
                    shooterSS.updatePollenState(ShooterSS.TurretState.SHOOT);
                }
                else {
                    shooterSS.setPollenTurretTarget(trackingSS.getPollenTurretPos());
                    shooterSS.updatePollenState(ShooterSS.TurretState.TRACKING_ENABLED);
                }
            }

            if (trackingEnabled()) {
                shooterSS.setPollenTurretTarget(trackingSS.getPollenTurretPos());
            }

            //todo intake beambreaker logic

            if (!robot.frontBeam.getState() && lastFrontBeam) {
                increaseCount(ballCount);
            }

            /* if front beam not tripped, then lastFrontbeam = true, else if front beam tripped, then last front beam = false
             * so if front beam wasn't tripped and front beam is now tripped, ball count increases by one
             */

            if (isShooting() && robot.pollenBeam.getState() && !lastPollenBeam) {
                decreaseCount(ballCount);
            }

            lastFrontBeam = robot.frontBeam.getState();
            lastPollenBeam = robot.pollenBeam.getState();

            if (ballCount == Count.FOUR && lastBallCount != Count.FOUR) {
                this.gamepad1.rumble(500);
            }

            lastBallCount = ballCount;

            if (ballCount != Count.EMPTY && !shooterEnabled() && shootingMode != ShootingMode.RELOCK) {
                shooterSS.updatePollenState(ShooterSS.ShooterState.ON);
            }
            else if (ballCount == Count.EMPTY && shooterEnabled() && !isShooting()) {
                shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);
            }

            if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper
                    && ballCount != Count.EMPTY && shootingMode == ShootingMode.INIT) {

                shooterSS.updatePollenState(ShooterSS.ShooterState.ON);
                shooterSS.updateState(ShooterSS.LockState.POLLEN_UNLOCKED);

                currentMode = IntakeMode.ON;
                shootingMode = ShootingMode.SHOOTING;
            }

            //todo intake logic
            if (shootingMode != ShootingMode.RELOCK && (ballCount != Count.FOUR || isShooting())) {
                if (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
                    currentMode = currentMode == IntakeMode.ON ? IntakeMode.INIT : IntakeMode.ON;
                }

                if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
                    currentMode = currentMode == IntakeMode.REVERSE ? IntakeMode.INIT : IntakeMode.REVERSE;
                }
            }

            switch (shootingMode) {
                case INIT:
                    break;

                case SHOOTING:
                    if (ballCount == Count.EMPTY) {
                        shootingMode = ShootingMode.FINAL_SHOT;
                        finishTimer.reset();
                    }
                    break;

                case FINAL_SHOT:
                    if (ballCount != Count.EMPTY) {
                        shootingMode = ShootingMode.SHOOTING;
                    }
                    else if (finishTimer.milliseconds() >= finishDelayMs) {
                        currentMode = IntakeMode.INIT;
                        intakeSS.updateState(IntakeSS.IntakeState.INIT);

                        shooterSS.updateState(ShooterSS.LockState.LOCKED);
                        shooterSS.updatePollenState(ShooterSS.ShooterState.INIT);

                        shootingMode = ShootingMode.RELOCK;
                        intakeLockoutTimer.reset();
                    }
                    break;

                case RELOCK:
                    if (intakeLockoutTimer.milliseconds() >= intakeLockoutMs
                            && shooterSS.getLockState() == ShooterSS.LockState.LOCKED) {
                        shootingMode = ShootingMode.INIT;
                        currentMode = ballCount != Count.FOUR ? IntakeMode.ON : IntakeMode.INIT;
                    }
                    break;
            }

            if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                if (intakeSS.getFlowerRampState() != IntakeSS.FlowerRampState.DEPLOYED) {
                    intakeSS.updateState(IntakeSS.FlowerRampState.DEPLOYED);
                }
                else {
                    intakeSS.updateState(IntakeSS.FlowerRampState.UP);
                }
            }

            if (robot.currentGamepad1.back && !robot.previousGamepad1.back) {
                shootingMode = ShootingMode.INIT;
                currentMode = IntakeMode.INIT;

                ballCount = Count.EMPTY;
                lastBallCount = Count.EMPTY;

                finishTimer.reset();
                intakeLockoutTimer.reset();

                lastFrontBeam = robot.frontBeam.getState();
                lastPollenBeam = robot.pollenBeam.getState();

                intakeSS.updateState(IntakeSS.IntakeState.INIT);
                shooterSS.updateState(ShooterSS.ShooterState.INIT);
                shooterSS.updateState(ShooterSS.LockState.LOCKED);
            }

            IntakeSS.IntakeState targetState;

            if (shootingMode == ShootingMode.RELOCK || (ballCount == Count.FOUR && !isShooting())) {
                currentMode = IntakeMode.INIT;
                targetState = IntakeSS.IntakeState.INIT;
            }
            else {
                switch (currentMode) {
                    case ON:
                        targetState = isShooting() || ballCount == Count.EMPTY || ballCount == Count.ONE
                                ? IntakeSS.IntakeState.ON
                                : IntakeSS.IntakeState.POINT_FIVE;
                        break;

                    case REVERSE:
                        targetState = IntakeSS.IntakeState.REVERSE;
                        break;

                    case INIT:
                    default:
                        targetState = IntakeSS.IntakeState.INIT;
                        break;
                }
            }

            if (intakeSS.getIntakeState() != targetState) {
                intakeSS.updateState(targetState);
            }

            PrintData.print(robot, follower, intakeSS, shooterSS, null, trackingEnabled(), shooterEnabled(),
                    ballCount == Count.EMPTY ? 0 :
                            ballCount == Count.ONE ? 1 :
                            ballCount == Count.TWO ? 2 :
                            ballCount == Count.THREE ? 3 : 4, telemetry);

            telemetry.update();
        }

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        trackingSS.resetSelectedHive();
    }

    public boolean isShooting() {
        return shootingMode == ShootingMode.SHOOTING || shootingMode == ShootingMode.FINAL_SHOT;
    }

    public boolean trackingEnabled() {
        return shooterSS.getPollenTurretState() == ShooterSS.TurretState.TRACKING_ENABLED;
    }

    public boolean shooterEnabled() {
        return shooterSS.getPollenShooterState() == ShooterSS.ShooterState.ON;
    }

    public void resetGamepads() {
        robot.currentGamepad1.copy(gamepad1);
        robot.previousGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);
        robot.previousGamepad2.copy(gamepad2);
    }
    public void increaseCount(Count count) {
        switch (count) {
            case EMPTY:
                ballCount = Count.ONE;
                break;
            case ONE:
                ballCount = Count.TWO;
                break;
            case TWO:
                ballCount = Count.THREE;
                break;
            case THREE:
                ballCount = Count.FOUR;
                break;
            case FOUR:
                break;
        }
    }
    public void decreaseCount(Count count) {
        switch (count) {
            case FOUR:
                ballCount = Count.THREE;
                break;
            case THREE:
                ballCount = Count.TWO;
                break;
            case TWO:
                ballCount = Count.ONE;
                break;
            case ONE:
                ballCount = Count.EMPTY;
                break;
            case EMPTY:
                break;
        }
    }
}