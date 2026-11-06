package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Version Testing")
public class TurretTestV1 extends LinearOpMode {
    Servo turret1, turret2;
    double pos1, pos2, step = 0.001;
    int selected = 0;
    boolean lastLeft, lastRight, lastLB, lastRB, lastA, lastB, lastX;

    @Override
    public void runOpMode() throws InterruptedException {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        pos1 = turret1.getPosition();
        pos2 = turret2.getPosition();
        telemetry.addLine("LEFT/RIGHT: choose turret | LB/RB: position -/+ | A: .001 | B: .0001 | X: center selected");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !lastLeft) selected = 0;
            if (gamepad1.dpad_right && !lastRight) selected = 1;
            if (gamepad1.a && !lastA) step = 0.001;
            if (gamepad1.b && !lastB) step = 0.0001;
            if (gamepad1.right_bumper && !lastRB) {
                if (selected == 0) pos1 = Range.clip(pos1 + step, 0, 1);
                else pos2 = Range.clip(pos2 + step, 0, 1);
            }
            if (gamepad1.left_bumper && !lastLB) {
                if (selected == 0) pos1 = Range.clip(pos1 - step, 0, 1);
                else pos2 = Range.clip(pos2 - step, 0, 1);
            }
            if (gamepad1.x && !lastX) {
                if (selected == 0) pos1 = 0.5;
                else pos2 = 0.5;
            }
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper;
            lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x;
            turret1.setPosition(pos1);
            turret2.setPosition(pos2);
            telemetry.addLine("LEFT/RIGHT: select | LB/RB: -/+ | A: .001 | B: .0001 | X: center");
            telemetry.addData("Selected", selected == 0 ? "Pollen turret" : "Nectar turret");
            telemetry.addData("Step", step);
            telemetry.addData("Pollen turret", pos1);
            telemetry.addData("Nectar turret", pos2);
            telemetry.update();
            idle();
        }
    }
}
