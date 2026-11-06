package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
public class ShooterTestV1 extends LinearOpMode {
    DcMotorEx shooter1, shooter2;
    double shooter1Vel = 0, shooter2Vel = 0;
    boolean fine = false;
    String[] labels = {"Pollen velocity", "Nectar velocity", "Pollen P", "Pollen I", "Pollen D", "Pollen F", "Nectar P", "Nectar I", "Nectar D", "Nectar F"};
    int selected = 0;
    boolean lastLeft, lastRight, lastUp, lastDown, lastA, lastB, lastX;

    double getValue() {
        switch (selected) {
            case 0: return shooter1Vel;
            case 1: return shooter2Vel;
            case 2: return Globals.kP_Pollen;
            case 3: return Globals.kI_Pollen;
            case 4: return Globals.kD_Pollen;
            case 5: return Globals.kF_Pollen;
            case 6: return Globals.kP_Nectar;
            case 7: return Globals.kI_Nectar;
            case 8: return Globals.kD_Nectar;
            default: return Globals.kF_Nectar;
        }
    }

    void setValue(double value) {
        value = Math.max(0, value);
        switch (selected) {
            case 0: shooter1Vel = value; break;
            case 1: shooter2Vel = value; break;
            case 2: Globals.kP_Pollen = value; break;
            case 3: Globals.kI_Pollen = value; break;
            case 4: Globals.kD_Pollen = value; break;
            case 5: Globals.kF_Pollen = value; break;
            case 6: Globals.kP_Nectar = value; break;
            case 7: Globals.kI_Nectar = value; break;
            case 8: Globals.kD_Nectar = value; break;
            case 9: Globals.kF_Nectar = value; break;
        }
    }

    double getStep() {
        if (selected < 2) return fine ? 10 : 50;
        if (selected == 3 || selected == 7) return fine ? 0.001 : 0.01;
        if (selected == 4 || selected == 8) return fine ? 0.01 : 0.1;
        return fine ? 0.1 : 1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
        telemetry.addLine("LEFT/RIGHT: select RPM or PIDF | UP/DOWN: +/- | A: coarse | B: fine | X: STOP BOTH");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_left && !lastLeft) selected = (selected + labels.length - 1) % labels.length;
                if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % labels.length;
                if (gamepad1.a && !lastA) fine = false;
                if (gamepad1.b && !lastB) fine = true;
                boolean changed = false;
                if (gamepad1.dpad_up && !lastUp) { setValue(getValue() + getStep()); changed = selected >= 2; }
                if (gamepad1.dpad_down && !lastDown) { setValue(getValue() - getStep()); changed = selected >= 2; }
                if (changed) {
                    shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
                    shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
                }
                if (gamepad1.x && !lastX) { shooter1Vel = 0; shooter2Vel = 0; }
                lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
                lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
                lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x;
                shooter1.setVelocity(shooter1Vel);
                shooter2.setVelocity(shooter2Vel);
                telemetry.addLine("LEFT/RIGHT: parameter | UP/DOWN: +/-step | A: coarse | B: fine | X: STOP BOTH");
                telemetry.addData("Selected", labels[selected]);
                telemetry.addData("Selected value", getValue());
                telemetry.addData("Step", getStep());
                telemetry.addData("Pollen target (ticks/s)", shooter1Vel);
                telemetry.addData("Pollen actual (ticks/s)", shooter1.getVelocity());
                telemetry.addData("Pollen current (A)", shooter1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Nectar target (ticks/s)", shooter2Vel);
                telemetry.addData("Nectar actual (ticks/s)", shooter2.getVelocity());
                telemetry.addData("Nectar current (A)", shooter2.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Pollen PIDF", "%.4f %.4f %.4f %.4f", Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
                telemetry.addData("Nectar PIDF", "%.4f %.4f %.4f %.4f", Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
                telemetry.update();
                idle();
            }
        } finally {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }
    }
}
