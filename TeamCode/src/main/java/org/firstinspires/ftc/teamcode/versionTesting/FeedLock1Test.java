package org.firstinspires.ftc.teamcode.versionTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Version Testing")
public class FeedLock1Test extends LinearOpMode {
    Servo feedLock1;
    @Override
    public void runOpMode() throws InterruptedException {
        feedLock1 = hardwareMap.get(Servo.class, "feedLock1");
        waitForStart();
        while (opModeIsActive()) {
            feedLock1.setPosition(0.5);
        }
    }
}
