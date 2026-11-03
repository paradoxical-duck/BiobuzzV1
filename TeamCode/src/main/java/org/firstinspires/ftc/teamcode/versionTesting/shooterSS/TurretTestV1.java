package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
@Config
public class TurretTestV1 extends LinearOpMode {
    boolean lastRB = false;
    boolean lastLB = false;
    Servo turret1;
    @Override
    public void runOpMode() throws InterruptedException {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper && !lastRB) {
                turret1.setPosition(turret1.getPosition() + 0.0001);
            }
            lastRB = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lastLB) {
                turret1.setPosition(turret1.getPosition() - 0.0001);
            }
            lastLB = gamepad1.left_bumper;

            telemetry.addData("turret pos", turret1.getPosition());
            telemetry.update();
        }

    }
}
