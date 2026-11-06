package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Version Testing")
public class IntakeDeployTestV1 extends LinearOpMode {
    Servo intakeDeploy;
    double position;
    double step = 0.01;
    boolean lastUp, lastDown, lastA, lastB, lastX, lastY;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeDeploy = hardwareMap.get(Servo.class, "intakeDeploy");
        position = intakeDeploy.getPosition();
        telemetry.addLine("UP/DOWN: position +/- | A: .01 step | B: .001 step | X: 0 | Y: 1");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) step = 0.01;
            if (gamepad1.b && !lastB) step = 0.001;
            if (gamepad1.dpad_up && !lastUp) position = Range.clip(position + step, 0, 1);
            if (gamepad1.dpad_down && !lastDown) position = Range.clip(position - step, 0, 1);
            if (gamepad1.x && !lastX) position = 0;
            if (gamepad1.y && !lastY) position = 1;
            lastA = gamepad1.a; lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x; lastY = gamepad1.y;
            intakeDeploy.setPosition(position);
            telemetry.addLine("UP/DOWN: position +/- | A: .01 | B: .001 | X: 0 | Y: 1");
            telemetry.addData("Step", step);
            telemetry.addData("Target position", position);
            telemetry.addData("Commanded position", intakeDeploy.getPosition());
            telemetry.update();
            idle();
        }
    }
}
