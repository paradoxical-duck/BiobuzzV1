package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
public class ShooterTestV1 extends LinearOpMode {
    DcMotorEx shooter;
    double[] values = new double[5];
    String[] names = {"Velocity", "P", "I", "D", "F"};
    int selected = 0;
    double step = 1;
    boolean lastRB, lastLeft, lastRight, lastUp, lastDown, lastX;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        values[0] = 1000;
        values[1] = Globals.kP_Pollen;
        values[2] = Globals.kI_Pollen;
        values[3] = Globals.kD_Pollen;
        values[4] = Globals.kF_Pollen;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper && !lastRB) selected = (selected + 1) % 5;

            if (gamepad1.dpad_left && !lastLeft) step /= 10;
            if (gamepad1.dpad_right && !lastRight) step *= 10;

            if (gamepad1.dpad_up && !lastUp) values[selected] = Math.max(0, values[selected] + step);
            if (gamepad1.dpad_down && !lastDown) values[selected] = Math.max(0, values[selected] - step);

            if (gamepad1.x && !lastX) values[0] = 0;

            shooter.setVelocityPIDFCoefficients(values[1], values[2], values[3], values[4]);
            shooter.setVelocity(values[0]);

            lastRB = gamepad1.right_bumper;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x;

            telemetry.addLine("******** POLLEN PIDF TUNING ********");
            telemetry.addLine("RIGHT BUMPER: Next parameter");
            telemetry.addLine("LEFT/RIGHT: Step /10 or x10");
            telemetry.addLine("UP/DOWN: Increase/decrease value");
            telemetry.addLine("X: Stop shooter");

            telemetry.addData("Selected", names[selected]);
            telemetry.addData("Step size", step);
            telemetry.addData("Selected value", values[selected]);

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
    }
}