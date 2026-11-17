package org.firstinspires.ftc.teamcode.scrimmage2.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

@TeleOp(name = "Scrimmage Tele")
public class ScrimmageTele1 extends LinearOpMode {
    IntakeSS intakeSS;
    Follower follower;
    RobotHardware robot = RobotHardware.getInstance();
    public static Pose startPose = new Pose(0, 0, 0);

    boolean red = false;
    boolean intakeEnabled = false;
    boolean intakeReversed = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //***subsystems***
        intakeSS = new IntakeSS(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intakeSS.updateState(IntakeSS.IntakeState.INIT);

        while (opModeInInit()) {
            red = RobotHardware.isRed();

            //alliance selection carried over from auto but can still be manually changed
            if (gamepad1.left_bumper && !robot.previousGamepad1.left_bumper) {
                red = !red;
            }

            if (red) {
                RobotHardware.setAlliance(RobotHardware.Alliance.RED);
            } else {
                RobotHardware.setAlliance(RobotHardware.Alliance.BLUE);
            }

            telemetry.addLine("Initialized");
            telemetry.addLine(RobotHardware.getAlliance() + " alliance active");
            telemetry.update();
            robot.previousGamepad1.copy(gamepad1);
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.clearBulkCache();

            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            //todo intake logic
            if (gamepad1.right_bumper && !robot.previousGamepad1.right_bumper) {
                intakeEnabled = !intakeEnabled;
                intakeReversed = false;

                if (intakeEnabled) {
                    intakeSS.updateState(IntakeSS.IntakeState.ON);
                } else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            if (gamepad1.x && !robot.previousGamepad1.x) {
                intakeReversed = !intakeReversed;
                intakeEnabled = intakeReversed;

                if (intakeReversed) {
                    intakeSS.updateState(IntakeSS.IntakeState.REVERSE);
                } else {
                    intakeSS.updateState(IntakeSS.IntakeState.INIT);
                }
            }

            telemetry.addData("Alliance", RobotHardware.getAlliance());
            telemetry.addData("Intake", intakeSS.getIntakeState());
            telemetry.update();

            robot.previousGamepad1.copy(gamepad1);
        }
    }
}