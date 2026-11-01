package org.firstinspires.ftc.teamcode.versionTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Configurable
@TeleOp(group = "Version Testing")
public class LockTestV1 extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static double lockPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.lock.setPosition(lockPos);
            telemetry.addData("lock pos", lockPos);
            telemetry.update();
        }
    }
}
