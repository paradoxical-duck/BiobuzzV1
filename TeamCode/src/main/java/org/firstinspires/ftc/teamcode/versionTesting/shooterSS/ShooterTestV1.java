package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;

@Config
@TeleOp(group = "Version Testing")
public class ShooterTestV1 extends LinearOpMode {
    public static double shooter1Vel = 0;
    public static double shooter2Vel = 0;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
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
            shooter2.setVelocity(shooter2Vel);

            telemetry.addData("shooter1 vel", shooter1Vel);
            telemetry.addData("shooter2 vel", shooter2Vel);
        }

    }
}
