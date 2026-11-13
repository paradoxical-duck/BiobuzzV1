package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TESTTTT", group = "Version Testing")
public class IntakeTestV1 extends LinearOpMode {
    DcMotorEx intake1;
    DcMotorEx intake2;
    double intake1Pwr = 0;
    double intake2Pwr = 0;
    double step = 0.05;
    int selected = 0;
    boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastB, lastX, lastY, lastBack;
    Follower follower;
    boolean lastRB;
    @Override
    public void runOpMode() throws InterruptedException {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        follower = Constants.create(hardwareMap);

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Dpad LEFT/RIGHT: choose intake | Dpad UP/DOWN: power +/-");
        telemetry.addLine("A: +/-0.05 step | B: +/-0.01 step | Y: selected full power | X: stop both");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                follower.update();
                if (gamepad1.dpad_left && !lastLeft) {
                    selected = 0;
                }
                if (gamepad1.dpad_right && !lastRight) {
                    selected = 1;
                }
                if (gamepad1.a && !lastA) {
                    step = 0.05;
                }
                if (gamepad1.b && !lastB) {
                    step = 0.01;
                }
                if (gamepad1.dpad_up && !lastUp) {
                    if (selected == 0) {
                        intake1Pwr = Range.clip(intake1Pwr + step, -1, 1);
                    }
                    else {
                        intake2Pwr = Range.clip(intake2Pwr + step, -1, 1);
                    }
                }
                if (gamepad1.dpad_down && !lastDown) {
                    if (selected == 0) {
                        intake1Pwr = Range.clip(intake1Pwr - step, -1, 1);
                    }
                    else {
                        intake2Pwr = Range.clip(intake2Pwr - step, -1, 1);
                    }
                }
                if (gamepad1.y && !lastY) {
                    if (selected == 0) {
                        intake1Pwr = 1;
                    }
                    else {
                        intake2Pwr = 1;
                    }
                }
                if (gamepad1.x && !lastX) {
                    intake1Pwr = 0;
                    intake2Pwr = 0;
                }
                if (gamepad1.back && !lastBack) {
                    if (selected == 0) {
                        intake1Pwr = -intake1Pwr;
                    }
                    else {
                        intake2Pwr = -intake2Pwr;
                    }
                }
                if (gamepad1.right_bumper && !lastRB) {
                    intake1Pwr = 1;
                    intake2Pwr = 1;
                }

                lastLeft = gamepad1.dpad_left;
                lastRight = gamepad1.dpad_right;
                lastA = gamepad1.a;
                lastB = gamepad1.b;
                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;
                lastX = gamepad1.x;
                lastY = gamepad1.y;
                lastBack = gamepad1.back;
                lastRB = gamepad1.right_bumper;

                intake1.setPower(intake1Pwr);
                intake2.setPower(intake2Pwr);

                telemetry.addLine("LEFT/RIGHT: select | UP/DOWN: +/-step | A: .05 | B: .01 | Y: full | X: STOP BOTH");

                telemetry.addData("Selected", selected == 0 ? "Intake 1" : "Intake 2");
                telemetry.addData("Step", step);
                telemetry.addData("Intake 1 power", intake1Pwr);
                telemetry.addData("Intake 2 power", intake2Pwr);
                telemetry.addData("Intake 1 current (A)", intake1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Intake 2 current (A)", intake2.getCurrent(CurrentUnit.AMPS));
                telemetry.update();

                idle();
            }
        } finally {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
}
