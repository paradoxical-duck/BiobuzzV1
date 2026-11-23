package org.firstinspires.ftc.teamcode.versionTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(group = "Version Testing")
public class BeambreakerTestV1 extends LinearOpMode {
    DigitalChannel frontBeam;
    DigitalChannel pollenBeam;
    @Override
    public void runOpMode() throws InterruptedException {
        frontBeam = hardwareMap.get(DigitalChannel.class, "frontBeam");
        pollenBeam = hardwareMap.get(DigitalChannel.class, "pollenBeam");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("front beam", frontBeam.getState());
            telemetry.addData("pollen beam", pollenBeam.getState());
            telemetry.update();
        }
    }
}
