package org.firstinspires.ftc.teamcode.VersionTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class TurretTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    boolean lastRB = false;
    boolean lastLB = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper && !lastRB) {
                robot.turret.setPosition(robot.turret.getPosition() + 0.0001);
            }
            lastRB = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lastLB) {
                robot.turret.setPosition(robot.turret.getPosition() - 0.0001);
            }
            lastLB = gamepad1.left_bumper;

            telemetry.addData("turret pos", robot.turret.getPosition());
            telemetry.update();
        }

    }
}
