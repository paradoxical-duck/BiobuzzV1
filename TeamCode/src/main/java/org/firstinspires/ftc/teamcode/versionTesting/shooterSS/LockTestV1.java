package org.firstinspires.ftc.teamcode.versionTesting.shooterSS;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
        waitForStart();
        while (opModeIsActive()) {
            feedLock1.setPosition(lock1Pos);
            feedLock2.setPosition(lock2Pos);
            telemetry.addData("feedlock1 pos", lock1Pos);
            telemetry.addData("feedlock2 pos", lock2Pos);
            telemetry.update();
        }
    }
}
