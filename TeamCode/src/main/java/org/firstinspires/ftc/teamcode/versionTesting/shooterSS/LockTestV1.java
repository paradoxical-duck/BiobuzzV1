package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Version Testing")
public class LockTestV1 extends LinearOpMode {
    Servo feedLock1;
    Servo feedLock2;
    double lock1Pos, lock2Pos, step = 0.01;
    int selected = 0;
    boolean lastLeft, lastRight, lastUp, lastDown, lastA, lastB, lastX, lastY;

    @Override
    public void runOpMode() throws InterruptedException {
        feedLock1 = hardwareMap.get(Servo.class, "feedLock1");
        feedLock2 = hardwareMap.get(Servo.class, "feedLock2");
        lock1Pos = feedLock1.getPosition();
        lock2Pos = feedLock2.getPosition();
        telemetry.addLine("LEFT/RIGHT: choose lock | UP/DOWN: +/- | A: .01 | B: .001 | X: 0 | Y: 1");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !lastLeft) selected = 0;
            if (gamepad1.dpad_right && !lastRight) selected = 1;
            if (gamepad1.a && !lastA) step = 0.01;
            if (gamepad1.b && !lastB) step = 0.001;
            if (gamepad1.dpad_up && !lastUp) {
                if (selected == 0) lock1Pos = Range.clip(lock1Pos + step, 0, 1);
                else lock2Pos = Range.clip(lock2Pos + step, 0, 1);
            }
            if (gamepad1.dpad_down && !lastDown) {
                if (selected == 0) lock1Pos = Range.clip(lock1Pos - step, 0, 1);
                else lock2Pos = Range.clip(lock2Pos - step, 0, 1);
            }
            if (gamepad1.x && !lastX) {
                if (selected == 0) lock1Pos = 0;
                else lock2Pos = 0;
            }
            if (gamepad1.y && !lastY) {
                if (selected == 0) lock1Pos = 1;
                else lock2Pos = 1;
            }
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a; lastB = gamepad1.b;
            lastX = gamepad1.x; lastY = gamepad1.y;
            feedLock1.setPosition(lock1Pos);
            feedLock2.setPosition(lock2Pos);
            telemetry.addLine("LEFT/RIGHT: select | UP/DOWN: +/-step | A: .01 | B: .001 | X: 0 | Y: 1");
            telemetry.addData("Selected", selected == 0 ? "Pollen lock" : "Nectar lock");
            telemetry.addData("Step", step);
            telemetry.addData("Pollen lock", lock1Pos);
            telemetry.addData("Nectar lock", lock2Pos);
            telemetry.update();
            idle();
        }
    }
}
