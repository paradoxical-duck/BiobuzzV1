package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
public class ShooterSSTuning extends LinearOpMode
/*todo use this to tune shooter, hood, turret values at auto shoot pose before S3*/
{
    DcMotorEx shooter1, shooter2;
    Servo turret1, turret2, hood1, hood2;
    double[] target = new double[6];
    int selected = 0;
    boolean fine = false;
    boolean lastLeft, lastRight, lastUp, lastDown, lastA, lastB, lastX;
    String[] names = {"Pollen shooter", "Pollen turret", "Pollen hood", "Nectar shooter", "Nectar turret", "Nectar hood"};

    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood1 = hardwareMap.get(Servo.class, "hood1");
        hood2 = hardwareMap.get(Servo.class, "hood2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
        shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);
        target[1] = turret1.getPosition(); target[2] = hood1.getPosition();
        target[4] = turret2.getPosition(); target[5] = hood2.getPosition();
        telemetry.addLine("LEFT/RIGHT: parameter | UP/DOWN: +/- | A: coarse | B: fine | X: STOP BOTH SHOOTERS");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_left && !lastLeft) selected = (selected + 5) % 6;
                if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % 6;
                if (gamepad1.a && !lastA) fine = false;
                if (gamepad1.b && !lastB) fine = true;
                double step = (selected == 0 || selected == 3) ? (fine ? 10 : 50) : (fine ? 0.001 : 0.01);
                if (gamepad1.dpad_up && !lastUp) target[selected] += step;
                if (gamepad1.dpad_down && !lastDown) target[selected] -= step;
                if (selected == 0 || selected == 3) target[selected] = Math.max(0, target[selected]);
                else target[selected] = Range.clip(target[selected], 0, 1);
                if (gamepad1.x && !lastX) { target[0] = 0; target[3] = 0; }
                lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
                lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
                lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x;
                shooter1.setVelocity(target[0]); shooter2.setVelocity(target[3]);
                turret1.setPosition(target[1]); hood1.setPosition(target[2]);
                turret2.setPosition(target[4]); hood2.setPosition(target[5]);
                telemetry.addLine("LEFT/RIGHT: select 1 of 6 | UP/DOWN: +/- | A: coarse | B: fine | X: STOP SHOOTERS");
                telemetry.addData("Selected", names[selected]);
                telemetry.addData("Step", step);
                for (int i = 0; i < target.length; i++) telemetry.addData(names[i], target[i]);
                telemetry.addData("Pollen actual ticks/s", shooter1.getVelocity());
                telemetry.addData("Nectar actual ticks/s", shooter2.getVelocity());
                telemetry.addData("Pollen current A", shooter1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Nectar current A", shooter2.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
                idle();
            }
        } finally {
            shooter1.setVelocity(0); shooter2.setVelocity(0);
        }
    }
}
