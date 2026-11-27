package org.firstinspires.ftc.teamcode.scrimmage3.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        resetGamepads();
        PrintData.reset();

        alliance = RobotHardware.getAlliance();

        //***subsystems***
        intakeSS = new IntakeSS(robot);
        shooterSS = new ShooterSS(robot);

        follower = Constants.create(hardwareMap);
        startPose = RobotHardware.currentPose != null ? RobotHardware.currentPose : new Pose(0, 0, 0); //todo set pose after auto
        follower.setPose(startPose);

        robot.init(shooterSS, intakeSS);

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

        intakeSS.initBallCount();

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

            if (robot.currentGamepad2.back && !robot.previousGamepad2.back) {
                trackingSS.toggleSelectedHive();
            }

            if (robot.currentGamepad2.start && !robot.previousGamepad2.start) {
                trackingSS.togglePollenTracking(shooterSS);
            }

            trackingSS.updatePollenTurretTracking(shooterSS);

            if (robot.currentGamepad1.back && !robot.previousGamepad1.back) {
                shooterSS.resetShooting(intakeSS);
            }
            else {
                if (intakeSS.updateBallCount(shooterSS.isShooting())) {
                    gamepad1.rumble(500);
                }

                if (robot.currentGamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                    shooterSS.startShooting(intakeSS);
                }

                //todo intake logic
                if (robot.currentGamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
                    intakeSS.toggleIntake(IntakeSS.IntakeMode.ON, shooterSS.getShootingMode());
                }

                if (robot.currentGamepad1.x && !robot.previousGamepad1.x) {
                    intakeSS.toggleIntake(IntakeSS.IntakeMode.REVERSE, shooterSS.getShootingMode());
                }

                shooterSS.updateShooting(intakeSS);
                intakeSS.updateIntake(shooterSS.getShootingMode());
            }

            if (robot.currentGamepad1.dpad_up && !robot.previousGamepad1.dpad_up) {
                intakeSS.toggleFlowerRamp();
            }

            PrintData.print(robot, follower, intakeSS, shooterSS, null, trackingSS.trackingEnabled(shooterSS), shooterSS.shooterEnabled(), intakeSS.getBallCountValue(), telemetry);

            telemetry.update();
        }

        shooterSS.updateState(ShooterSS.ShooterState.INIT);
        shooterSS.updateState(ShooterSS.LockState.LOCKED);
        intakeSS.updateState(IntakeSS.IntakeState.INIT);
        trackingSS.resetSelectedHive();
    }

    public void resetGamepads() {
        robot.currentGamepad1.copy(gamepad1);
        robot.previousGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);
        robot.previousGamepad2.copy(gamepad2);
    }
}