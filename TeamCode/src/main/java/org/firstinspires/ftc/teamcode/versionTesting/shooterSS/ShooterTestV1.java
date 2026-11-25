package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
public class ShooterTestV1 extends LinearOpMode {
    DcMotorEx shooter, intake1, intake2;

    double[] values = new double[5];
    String[] names = {"Velocity", "P", "I", "D", "F"};
    int selected = 0;
    double step = 1;

    boolean intakeEnabled = false;
    boolean lastLB, lastRB, lastLeft, lastRight, lastUp, lastDown, lastX;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setPower(0);
        intake2.setPower(0);

        values[0] = 1000;
        values[1] = Globals.kP_Pollen;
        values[2] = Globals.kI_Pollen;
        values[3] = Globals.kD_Pollen;
        values[4] = Globals.kF_Pollen;

        if (values[1] == 0 && values[2] == 0 && values[3] == 0 && values[4] == 0) {
            PIDFCoefficients pidf = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            values[1] = pidf.p;
            values[2] = pidf.i;
            values[3] = pidf.d;
            values[4] = pidf.f;
        }

        waitForStart();

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastX = gamepad1.x;

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && !lastLB) selected = (selected + 1) % 5;

            if (gamepad1.right_bumper && !lastRB) {
                intakeEnabled = !intakeEnabled;
                intake1.setPower(intakeEnabled ? Globals.intakeOn : Globals.intakeOff);
                intake2.setPower(intakeEnabled ? Globals.intakeOn : Globals.intakeOff);
            }

            if (gamepad1.dpad_left && !lastLeft) step /= 10;
            if (gamepad1.dpad_right && !lastRight) step *= 10;

            if (gamepad1.dpad_up && !lastUp) values[selected] = Math.max(0, values[selected] + step);
            if (gamepad1.dpad_down && !lastDown) values[selected] = Math.max(0, values[selected] - step);

            if (gamepad1.x && !lastX) values[0] = 0;

            shooter.setVelocityPIDFCoefficients(values[1], values[2], values[3], values[4]);
            shooter.setVelocity(values[0]);

            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x;

            telemetry.addLine("******** POLLEN PIDF TUNING ********");
            telemetry.addLine("LEFT BUMPER: Next parameter");
            telemetry.addLine("RIGHT BUMPER: Toggle intake");
            telemetry.addLine("LEFT/RIGHT: Step /10 or x10");
            telemetry.addLine("UP/DOWN: Increase/decrease value");
            telemetry.addLine("X: Stop shooter");

            telemetry.addData("Selected", names[selected]);
            telemetry.addData("Step size", step);
            telemetry.addData("Selected value", values[selected]);
            telemetry.addData("Intake enabled", intakeEnabled);

            telemetry.addLine("******** PIDF ********");
            telemetry.addData("P", values[1]);
            telemetry.addData("I", values[2]);
            telemetry.addData("D", values[3]);
            telemetry.addData("F", values[4]);

            telemetry.addLine("******** SHOOTER ********");
            telemetry.addData("Target velocity (ticks/s)", values[0]);
            telemetry.addData("Actual velocity (ticks/s)", shooter.getVelocity());
            telemetry.addData("Current (A)", shooter.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
            idle();
        }

        shooter.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }
}