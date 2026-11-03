package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group = "Version Testing")
@Config
public class IntakeDeployTestV1 extends LinearOpMode {
    public static double intakeDeployPos = 0;
    Servo intakeDeploy;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeDeploy = hardwareMap.get(Servo.class, "intakeDeploy");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            intakeDeploy.setPosition(intakeDeployPos);

            telemetry.addData("target pos", intakeDeployPos);
            telemetry.addData("current pos", intakeDeployPos);
            telemetry.update();
        }
    }
}
