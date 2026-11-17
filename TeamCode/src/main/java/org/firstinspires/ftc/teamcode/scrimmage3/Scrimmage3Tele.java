package org.firstinspires.ftc.teamcode.scrimmage3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PARA REDEMPTION ARC 💛")
public class Scrimmage3Tele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //todo scrimmage 3 teleop logic
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Scrimmage 3 TeleOp - not implemented yet");
            telemetry.update();
        }
    }
}
