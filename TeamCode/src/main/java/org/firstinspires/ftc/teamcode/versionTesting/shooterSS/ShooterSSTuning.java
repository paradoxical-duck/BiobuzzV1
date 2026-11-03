package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(group = "Version Testing")
@Config
public class ShooterSSTuning extends LinearOpMode {
    DcMotorEx shooter1;
    public static double shooter1Vel = 0;
    DcMotorEx shooter2;
    public static double shooter2Vel = 0;
    Servo turret1;
    public static double turret1Pos = 0;
    Servo hood1;
    public static double hood1Pos = 0;
    Servo turret2;
    public static double turret2Pos = 0;
    Servo hood2;
    public static double hood2Pos = 0;
    public static double kP_Pollen = Globals.kP_Pollen;
    public static double kI_Pollen = Globals.kI_Pollen;
    public static double kD_Pollen = Globals.kD_Pollen;
    public static double kF_Pollen = Globals.kF_Pollen;
    public static double kP_Nectar = Globals.kP_Nectar;
    public static double kI_Nectar = Globals.kI_Nectar;
    public static double kD_Nectar = Globals.kD_Nectar;
    public static double kF_Nectar = Globals.kF_Nectar;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        hood1 = hardwareMap.get(Servo.class, "hood1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood2 = hardwareMap.get(Servo.class, "hood2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {
                shooter1.setVelocityPIDFCoefficients(kP_Pollen, kI_Pollen, kD_Pollen, kF_Pollen);
                shooter2.setVelocityPIDFCoefficients(kP_Nectar, kI_Nectar, kD_Nectar, kF_Nectar);
                shooter1.setVelocity(shooter1Vel);
                shooter2.setVelocity(shooter2Vel);
                turret1.setPosition(turret1Pos);
                hood1.setPosition(hood1Pos);
                turret2.setPosition(turret2Pos);
                hood2.setPosition(hood2Pos);
                telemetry.addData("shooter1 target (ticks/s)", shooter1Vel);
                telemetry.addData("shooter1 velocity (ticks/s)", shooter1.getVelocity());
                telemetry.addData("shooter1 current (A)", shooter1.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("shooter2 target (ticks/s)", shooter2Vel);
                telemetry.addData("shooter2 velocity (ticks/s)", shooter2.getVelocity());
                telemetry.addData("shooter2 current (A)", shooter2.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("turret1 pos", turret1.getPosition());
                telemetry.addData("hood1 pos", hood1.getPosition());
                telemetry.addData("turret2 pos", turret2.getPosition());
                telemetry.addData("hood2 pos", hood2.getPosition());
                telemetry.update();
            }
        } finally {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }
    }
}
