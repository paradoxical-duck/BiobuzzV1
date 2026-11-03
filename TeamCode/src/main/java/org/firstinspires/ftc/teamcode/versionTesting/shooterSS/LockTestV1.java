package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(group = "Version Testing")
public class LockTestV1 extends LinearOpMode {
    public static double lock1Pos = 0;
    public static double lock2Pos = 0;
    Servo feedLock1;
    Servo feedLock2;
    @Override
    public void runOpMode() throws InterruptedException {
        feedLock1 = hardwareMap.get(Servo.class, "feedLock1");
        feedLock2 = hardwareMap.get(Servo.class, "feedLock2");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            feedLock1.setPosition(lock1Pos);
            feedLock2.setPosition(lock2Pos);
            telemetry.addData("feedlock1 pos", feedLock1.getPosition());
            telemetry.addData("feedlock2 pos", feedLock2.getPosition());
            telemetry.update();
        }
    }
}
