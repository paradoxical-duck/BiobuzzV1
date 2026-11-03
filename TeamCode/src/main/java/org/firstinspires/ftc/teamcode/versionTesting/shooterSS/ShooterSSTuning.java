package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
@Config
public class ShooterSSTuning extends LinearOpMode {
    public static double shooter1Vel = 0;
    public static double turret1Pos = 0;
    public static double hood1Pos = 0;

    public static double shooter2Vel = 0;
    public static double turret2Pos = 0;
    public static double hood2Pos = 0;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    Servo turret1;
    Servo hood1;
    Servo turret2;
    Servo hood2;
    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        hood1 = hardwareMap.get(Servo.class, "hood1");
        hood2 = hardwareMap.get(Servo.class, "hood2");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            shooter1.setVelocityPIDFCoefficients(Globals.kP_Pollen, Globals.kI_Pollen, Globals.kD_Pollen, Globals.kF_Pollen);
            shooter2.setVelocityPIDFCoefficients(Globals.kP_Nectar, Globals.kI_Nectar, Globals.kD_Nectar, Globals.kF_Nectar);

            shooter1.setVelocity(shooter1Vel);
            turret1.setPosition(turret1Pos);

            hood1.setPosition(hood1Pos);

            shooter2.setVelocity(shooter2Vel);
            turret2.setPosition(turret2Pos);
            hood2.setPosition(hood2Pos);

            telemetry.addData("shooter1 vel", shooter1Vel);
            telemetry.addData("turret1 pos", turret1Pos);
            telemetry.addData("hood1 pos", hood1Pos);
            telemetry.addData("shooter2 vel", shooter2Vel);
            telemetry.addData("turret2 pos", turret2Pos);
            telemetry.addData("hood2 pos", hood2Pos);
            telemetry.update();
        }
    }
}
