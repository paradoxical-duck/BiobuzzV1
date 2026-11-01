package org.firstinspires.ftc.teamcode.versionTesting.intakeSS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Version Testing")
@Configurable
public class FlowerRampTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double flowerRampPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.flowerRamp.setPosition(flowerRampPos);

            telemetry.addData("target pos", flowerRampPos);
            telemetry.addData("current pos", flowerRampPos);
            telemetry.update();
        }
    }
}
